const { shell } = require('electron');
const { exec } = require('child_process');
const config = require('./config');

// Ejecuta en el PC las macros que el propio teclado NO puede reproducir solo
// (avisa por CDC con MACRO:<id> en vez de tocarlas él, ver trigger_macro en
// main.cpp): hoy eso es únicamente lo que necesita posición ABSOLUTA del
// ratón, que exige conocer la resolución/escala de la pantalla. El resto de
// pasos (tecla, espera, clic, movimiento relativo) los toca el teclado por su
// cuenta —más rápido y sin depender de que esta app esté abierta— así que ya
// no deberían llegar aquí; se deja el reproductor completo por compatibilidad
// con macros antiguas que no se hayan podido subir al dispositivo.

let nutjs = null;
let nutjsFailed = false;

function loadNutJs() {
  if (nutjs || nutjsFailed) return nutjs;
  try {
    nutjs = require('@nut-tree-fork/nut-js');
    // nut.js mete su propio hueco ANTES de cada pulsación/clic (keyboard 300ms,
    // mouse 100ms, ver keyboard.class.js/mouse.class.js autoDelayMs), pensado
    // para separar llamadas sueltas hechas a mano. Aquí ya controlamos nosotros
    // el ritmo -la "Espera" entre pasos y el "gap" entre repeticiones, ver
    // repeatWithGap-, así que ese hueco extra solo suma un despropósito por
    // pulsación (con 11 repeticiones, 11 × 600ms = 6.6s de nada) que ningún
    // ajuste del editor podía tocar: no era el gap configurable, era este.
    nutjs.keyboard.config.autoDelayMs = 0;
    nutjs.mouse.config.autoDelayMs = 0;
  } catch (err) {
    nutjsFailed = true;
    console.error('No se pudo cargar @nut-tree-fork/nut-js:', err.message);
  }
  return nutjs;
}

// Solo cubre lo que se puede grabar desde el editor de la app (letras, dígitos,
// Enter, Escape, Espacio). Para ampliarlo basta con añadir la entrada aquí: el
// editor ya captura cualquier KeyboardEvent.code, solo falta traducirlo.
function mapCodeToNutKey(code, Key) {
  if (/^Key[A-Z]$/.test(code)) return Key[code.slice(3)];
  if (/^Digit[0-9]$/.test(code)) return Key[`Num${code.slice(5)}`];
  if (code === 'Enter') return Key.Enter;
  if (code === 'Escape') return Key.Escape;
  if (code === 'Space') return Key.Space;
  return null;
}

const CLICK_BUTTONS = { left: 'LEFT', middle: 'MIDDLE', right: 'RIGHT' };

// Acciones de energía: no hay opcode de firmware para esto (igual que
// open_app), siempre corren por el camino del PC. Solo Windows por ahora,
// que es la única plataforma que soporta esta app (ver foreground.js).
const POWER_COMMANDS = {
  sleep:     { win32: 'rundll32.exe powrprof.dll,SetSuspendState 0,1,0' },
  hibernate: { win32: 'shutdown /h' },
  restart:   { win32: 'shutdown /r /t 0' },
  shutdown:  { win32: 'shutdown /s /t 0' },
  lock:      { win32: 'rundll32.exe user32.dll,LockWorkStation' },
  logoff:    { win32: 'shutdown /l' },
};

function runPowerAction(mode) {
  const cmd = POWER_COMMANDS[mode]?.[process.platform];
  if (!cmd) {
    console.error(`Acción de energía "${mode}" no soportada en ${process.platform}`);
    return;
  }
  exec(cmd, (err) => {
    if (err) console.error(`Acción de energía "${mode}":`, err.message);
  });
}

// --- Lámpara LampDesk -------------------------------------------------------
// Habla directamente con la API HTTP del firmware de la lámpara (ver el README
// de lampDesk). Como open_app y system_power, no hay opcode de firmware que
// valga: siempre corre por el camino del PC.
//
// Los incrementos se mandan como tales (`dbrightness`, `dcolorTemp`) en vez de
// leer el estado, sumar y escribir el resultado: la lámpara resuelve la suma, y
// así el encoder no se pelea con la app Casa ni con el panel de la bandeja por
// quién tenía el valor bueno.
const LAMP_OPS = {
  toggle:           () => ({ on: 'toggle' }),
  on:               () => ({ on: 1 }),
  off:              () => ({ on: 0 }),
  brightness_delta: (v) => ({ dbrightness: v }),
  color_delta:      (v) => ({ dcolor: v }),
};

const LAMP_DELTA_OPS = new Set(['brightness_delta', 'color_delta']);
// Cada escritura hace que la lámpara avise por HomeKit a los iPhone
// emparejados, y esa ida y vuelta le bloquea el bucle principal unos cientos de
// ms: la petición siguiente se come la espera. Lo normal son 20-35 ms; el pico
// medido pasa de dos segundos.
const LAMP_TIMEOUT_MS = 4000;

// El firmware del Orby manda un MACRO:<id> **por cada muesca** del encoder (ver
// emit_discrete en main.cpp), así que un giro rápido son veinte disparos en un
// suspiro. Sin agrupar, eso son veinte peticiones HTTP en cola contra una ESP32
// que las atiende de una en una, y el brillo llega tarde y a tirones.
// 120 ms y no menos: la lámpara resuelve una petición suelta en unos 20 ms,
// pero encadenadas cada 60 ms se le acumulan y la mediana se va a 240 ms. Como
// el incremento se agrupa, esperar un poco más no pierde ninguna muesca — solo
// las junta en una petición más gorda.
const LAMP_COALESCE_MS = 120;

let lampPending = null;
let lampFlushTimer = null;
let lampInFlight = false;

function lampHost() {
  return config.load().lamp?.host || '192.168.1.35:8080';
}

// Una lámpara desenchufada no puede dejar colgada la ejecución de macros: sin
// AbortController, fetch espera al agotamiento del TCP (más de un minuto).
async function lampRequest(params, host = lampHost()) {
  const control = new AbortController();
  const timer = setTimeout(() => control.abort(), LAMP_TIMEOUT_MS);
  try {
    const query = new URLSearchParams(params).toString();
    const res = await fetch(`http://${host}/set?${query}`, { signal: control.signal });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return await res.json();
  } finally {
    clearTimeout(timer);
  }
}

function scheduleLampFlush() {
  if (lampFlushTimer || lampInFlight) return;
  lampFlushTimer = setTimeout(async () => {
    lampFlushTimer = null;
    if (!lampPending) return;
    const params = lampPending;
    lampPending = null;
    lampInFlight = true;
    try {
      await lampRequest(params);
    } catch (err) {
      console.error(`Lámpara (${lampHost()}):`, err.message);
    } finally {
      lampInFlight = false;
      if (lampPending) scheduleLampFlush();
    }
  }, LAMP_COALESCE_MS);
}

function runLampAction(action) {
  const build = LAMP_OPS[action.op];
  if (!build) {
    console.error(`Acción de lámpara "${action.op}" desconocida`);
    return;
  }

  const params = build(Math.trunc(Number(action.value)) || 0);

  // Encender y apagar van sin agrupar: dos pulsaciones seguidas son dos
  // órdenes, y fundidas en una sola "alterna" se anularían entre ellas.
  if (!LAMP_DELTA_OPS.has(action.op)) {
    lampRequest(params).catch((err) => console.error(`Lámpara (${lampHost()}):`, err.message));
    return;
  }

  lampPending = lampPending || {};
  for (const [key, value] of Object.entries(params)) {
    lampPending[key] = Number(lampPending[key] || 0) + Number(value);
  }
  scheduleLampFlush();
}

// Comprobación desde los ajustes: pide el estado sin cambiar nada.
//
// Con un reintento, porque leer es inofensivo: si la lámpara estaba ocupada
// avisando a HomeKit, el primer intento se queda sin respuesta y la tarjeta de
// ajustes diría "no responde" con la lámpara perfectamente viva. Las escrituras
// **no** se reintentan: un incremento repetido se aplicaría dos veces.
async function lampTest(host) {
  const pedir = async () => {
    const control = new AbortController();
    const timer = setTimeout(() => control.abort(), LAMP_TIMEOUT_MS);
    try {
      const res = await fetch(`http://${host || lampHost()}/state`, { signal: control.signal });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return await res.json();
    } finally {
      clearTimeout(timer);
    }
  };

  try {
    return { ok: true, state: await pedir() };
  } catch {
    try {
      return { ok: true, state: await pedir() };
    } catch (err) {
      return { ok: false, error: String(err.message || err) };
    }
  }
}

// Traduce el mismo usage HID (Usage Page 0x07) que usa el editor de atajos
// (ver src/hid-keys.js, KEY_GROUPS) al nombre de tecla de nut.js. Cubre
// exactamente los códigos que ofrece el selector de teclas de la app.
const HID_TO_NUT_KEY = {
  // Letras 0x04-0x1D: A-Z
  ...Object.fromEntries(Array.from({ length: 26 }, (_, i) => [0x04 + i, String.fromCharCode(65 + i)])),
  // Números (fila superior) 0x1e-0x26: 1-9, 0x27: 0
  0x1e: 'Num1', 0x1f: 'Num2', 0x20: 'Num3', 0x21: 'Num4', 0x22: 'Num5',
  0x23: 'Num6', 0x24: 'Num7', 0x25: 'Num8', 0x26: 'Num9', 0x27: 'Num0',
  // F1-F12 (0x3a-0x45), F13-F24 (0x68-0x73)
  ...Object.fromEntries(Array.from({ length: 12 }, (_, i) => [0x3a + i, `F${i + 1}`])),
  ...Object.fromEntries(Array.from({ length: 12 }, (_, i) => [0x68 + i, `F${i + 13}`])),
  // Edición
  0x28: 'Return', 0x29: 'Escape', 0x2a: 'Backspace', 0x2b: 'Tab', 0x2c: 'Space',
  0x49: 'Insert', 0x4c: 'Delete', 0x4a: 'Home', 0x4d: 'End',
  0x4b: 'PageUp', 0x4e: 'PageDown', 0x46: 'Print',
  // Flechas
  0x50: 'Left', 0x4f: 'Right', 0x52: 'Up', 0x51: 'Down',
  // Símbolos
  0x2d: 'Minus', 0x2e: 'Equal', 0x2f: 'LeftBracket', 0x30: 'RightBracket',
  0x31: 'Backslash', 0x33: 'Semicolon', 0x34: 'Quote', 0x35: 'Grave',
  0x36: 'Comma', 0x37: 'Period', 0x38: 'Slash',
  // Teclado numérico
  0x54: 'Divide', 0x55: 'Multiply', 0x56: 'Subtract', 0x57: 'Add', 0x58: 'Enter',
  0x59: 'NumPad1', 0x5a: 'NumPad2', 0x5b: 'NumPad3', 0x5c: 'NumPad4', 0x5d: 'NumPad5',
  0x5e: 'NumPad6', 0x5f: 'NumPad7', 0x60: 'NumPad8', 0x61: 'NumPad9', 0x62: 'NumPad0',
};

// Bits de src/hid-keys.js MODIFIERS.
const HID_MODIFIER_TO_NUT_KEY = {
  0x01: 'LeftControl', 0x02: 'LeftShift', 0x04: 'LeftAlt', 0x08: 'LeftSuper',
  0x10: 'RightControl', 0x20: 'RightShift', 0x40: 'RightAlt', 0x80: 'RightSuper',
};

function modifierKeys(modifier, Key) {
  const keys = [];
  for (const [bit, name] of Object.entries(HID_MODIFIER_TO_NUT_KEY)) {
    if (modifier & Number(bit)) keys.push(Key[name]);
  }
  return keys;
}

// Hueco por defecto entre repeticiones cuando la acción no trae el suyo
// propio (secuencias guardadas antes de que el editor dejara elegirlo, ver
// DEFAULT_REPEAT_GAP_MS en profiles.js — debe coincidir con ese valor).
const DEFAULT_REPEAT_GAP_MS = 20;

async function runAction(action) {
  // Antes de cargar nut.js: la lámpara solo necesita la red, y sin esta salida
  // temprana un fallo al cargar el binario nativo (OneDrive con node_modules
  // como marcador de nube, por ejemplo) también dejaría la luz sin funcionar.
  if (action.type === 'lamp') {
    runLampAction(action);
    return;
  }

  const nut = loadNutJs();
  if (!nut) return;
  const { mouse, keyboard, screen, Point, Key, Button } = nut;

  if (action.type === 'center_mouse') {
    // Formato antiguo, de antes de poder capturar cualquier posición: se deja
    // por compatibilidad con secuencias ya guardadas.
    const [w, h] = await Promise.all([screen.width(), screen.height()]);
    await mouse.setPosition(new Point(w / 2, h / 2));
  } else if (action.type === 'mouse_position') {
    await mouse.setPosition(new Point(action.x, action.y));
  } else if (action.type === 'mouse_click') {
    const button = Button[CLICK_BUTTONS[action.button] || 'LEFT'];
    await repeatWithGap(action.count, action.gap, () => mouse.click(button));
  } else if (action.type === 'delay') {
    await new Promise((resolve) => setTimeout(resolve, Math.max(0, action.ms || 0)));
  } else if (action.type === 'key') {
    const key = mapCodeToNutKey(action.code, Key);
    if (key === null || key === undefined) return;
    await repeatWithGap(action.count, action.gap, async () => {
      await keyboard.pressKey(key);
      await keyboard.releaseKey(key);
    });
  } else if (action.type === 'open_app') {
    // No hay opcode de firmware para esto (abrir algo requiere el SO): siempre
    // corre por el camino del PC, nunca se sube al teclado (ver DEVICE_STEP_TYPE
    // en profiles.js, que no lo incluye a propósito).
    if (!action.target) return;
    const err = await shell.openPath(action.target);
    if (err) console.error(`No se pudo abrir "${action.target}":`, err);
  } else if (action.type === 'system_power') {
    runPowerAction(action.mode);
  } else if (action.type === 'hotkey') {
    // Cualquier tecla (con o sin modificadores) del mismo selector que usa la
    // pestaña Atajo: { modifier, keycode } en usage HID.
    const mods = modifierKeys(action.modifier || 0, Key);
    const mainName = HID_TO_NUT_KEY[action.keycode];
    const main = mainName ? Key[mainName] : undefined;
    const keys = main !== undefined ? [...mods, main] : mods;
    if (!keys.length) return;
    await repeatWithGap(action.count, action.gap, async () => {
      await keyboard.pressKey(...keys);
      await keyboard.releaseKey(...keys);
    });
  }
}

// Repite un gesto (pulsación o clic) `count` veces, con un hueco de `gap` ms
// entre cada una: sin él, el SO puede llegar a fundir pulsaciones consecutivas
// en una sola (o no reconocer un doble clic como tal). Con `count` 1 o sin
// definir corre una sola vez, igual que antes de que existiera esta opción.
async function repeatWithGap(count, gap, run) {
  const times = Math.max(1, Math.round(count) || 1);
  const ms = Math.max(0, Math.round(gap ?? DEFAULT_REPEAT_GAP_MS));
  for (let i = 0; i < times; i++) {
    await run();
    if (i < times - 1) await new Promise((resolve) => setTimeout(resolve, ms));
  }
}

async function executeMacro(macroId) {
  const macro = (config.load().macros || []).find((m) => m.id === macroId);
  if (!macro) return;

  for (const action of macro.actions || []) {
    try {
      await runAction(action);
    } catch (err) {
      console.error(`Macro ${macroId}, acción "${action.type}":`, err.message);
    }
  }
}

// Posición del cursor en el mismo sistema de coordenadas que usa mouse.setPosition.
// Antes se leía con el módulo screen de Electron, pero en pantallas con escalado
// (125%/150% de Windows) ese sistema no coincide con el de nut.js y las posiciones
// capturadas quedaban con un desplazamiento al reproducirlas. Preguntándoselo a
// nut.js directamente, captura y reproducción siempre hablan el mismo idioma.
async function getMousePosition() {
  const nut = loadNutJs();
  if (!nut) return { x: 0, y: 0 };
  const p = await nut.mouse.getPosition();
  return { x: Math.round(p.x), y: Math.round(p.y) };
}

// Carga nut.js por adelantado al arrancar la app, en vez de esperar a la
// primera secuencia que caiga por el camino del PC (una macro no subida
// todavía al teclado, o con posición absoluta). Sin esto, esa primera
// ejecución paga sola el coste de cargar el binario nativo -normal que tarde
// notablemente, y peor aún si OneDrive tiene el node_modules como marcador de
// nube y tiene que bajarlo primero-, lo que se nota como un retraso enorme e
// inexplicable la primera vez.
function warmup() {
  loadNutJs();
}

module.exports = { executeMacro, getMousePosition, warmup, lampTest };
