const { shell, clipboard } = require('electron');
const { exec } = require('child_process');
const config = require('./config');
const plugins = require('./plugins');

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

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, Math.max(0, ms)));

// Hueco entre cada letra al escribir un texto letra a letra. Con 0 (lo que deja
// puesto loadNutJs para el resto de pasos) hay aplicaciones que se comen
// caracteres sueltos porque no llegan a procesar el evento anterior.
const TEXT_CHAR_DELAY_MS = 4;

// Escribir letra a letra tiene un techo que ningún ajuste arregla: cada carácter
// es un evento de teclado que el sistema tiene que entregar y la app destino
// procesar, así que un texto largo (una firma, un correo, un trozo de código) se
// ve teclear. A partir de este umbral se va por el portapapeles y un Ctrl+V, que
// mete el texto entero de golpe sin importar la longitud. Por debajo no
// compensa: pisar el portapapeles del usuario por cuatro letras sale más caro
// que escribirlas.
const TEXT_PASTE_MIN_CHARS = 5;

// Margen para que el portapapeles quede escrito antes de mandar el Ctrl+V, y
// para que la app destino lo lea antes de devolverle al usuario lo que tenía.
// Restaurarlo de inmediato hace que algunas apps peguen lo viejo: leen el
// portapapeles al procesar el atajo, no al recibirlo.
const CLIPBOARD_SETTLE_MS = 30;
const CLIPBOARD_RESTORE_MS = 120;

// Mete el texto de golpe por el portapapeles. Devuelve false si algo falla, para
// que el que llama pueda caer a escribirlo letra a letra.
//
// Se restaura el portapapeles anterior, pero solo su parte de texto: si lo que
// había copiado era una imagen o un archivo, se pierde. Y hay entornos donde
// Ctrl+V no pega (la consola clásica de Windows, algún juego, escritorios
// remotos); ahí el texto no aparece y no hay forma de detectarlo desde aquí.
async function pasteText(texto, keyboard, Key) {
  let previo = '';
  try {
    previo = clipboard.readText();
    clipboard.writeText(texto);
    await sleep(CLIPBOARD_SETTLE_MS);
    await keyboard.pressKey(Key.LeftControl, Key.V);
    await keyboard.releaseKey(Key.LeftControl, Key.V);
    await sleep(CLIPBOARD_RESTORE_MS);
    return true;
  } catch (err) {
    console.error('No se pudo pegar el texto por el portapapeles:', err.message);
    return false;
  } finally {
    if (previo) clipboard.writeText(previo);
  }
}

// Escribe un texto tal cual. No pasa por códigos de tecla HID (como sí hace
// "hotkey", ver más abajo), sino por el camino unicode del sistema: así sale
// igual con cualquier distribución de teclado -ñ, acentos, símbolos- sin tener
// que saber cuál tiene puesta el usuario. Eso mismo es lo que impide subirlo al
// teclado, que solo sabe mandar usages HID (ver DEVICE_STEP_TYPE en profiles.js).
//
// Un salto de línea o un tabulador no son caracteres que se puedan "escribir" ni
// pegar sueltos: se mandan como pulsación de Intro y de Tab, que es lo que el
// usuario espera (en un chat, Intro envía; pegar un "\n" no).
async function typeText(text, keyboard, Key) {
  if (!text) return;
  const previo = keyboard.config.autoDelayMs;
  keyboard.config.autoDelayMs = TEXT_CHAR_DELAY_MS;
  try {
    for (const trozo of String(text).split(/(\r\n|\n|\r|\t)/)) {
      if (!trozo) continue;
      if (trozo === '\t') {
        await keyboard.pressKey(Key.Tab);
        await keyboard.releaseKey(Key.Tab);
      } else if (trozo === '\n' || trozo === '\r' || trozo === '\r\n') {
        await keyboard.pressKey(Key.Return);
        await keyboard.releaseKey(Key.Return);
      } else if (trozo.length >= TEXT_PASTE_MIN_CHARS && await pasteText(trozo, keyboard, Key)) {
        continue;
      } else {
        await keyboard.type(trozo);
      }
    }
  } finally {
    keyboard.config.autoDelayMs = previo;
  }
}

async function runAction(action) {
  // Antes de cargar nut.js: un complemento no tiene por qué necesitar el ratón
  // ni el teclado (el de la lámpara solo usa la red), y sin esta salida
  // temprana un fallo al cargar el binario nativo (OneDrive con node_modules
  // como marcador de nube, por ejemplo) dejaría también sin funcionar lo que no
  // dependía de él.
  if (action.type === 'plugin') {
    await plugins.runStep(action);
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
    await sleep(action.ms || 0);
  } else if (action.type === 'key') {
    const key = mapCodeToNutKey(action.code, Key);
    if (key === null || key === undefined) return;
    await repeatWithGap(action.count, action.gap, async () => {
      await keyboard.pressKey(key);
      await keyboard.releaseKey(key);
    });
  } else if (action.type === 'text') {
    await repeatWithGap(action.count, action.gap, () => typeText(action.text, keyboard, Key));
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
    if (i < times - 1) await sleep(ms);
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

module.exports = { executeMacro, getMousePosition, warmup };
