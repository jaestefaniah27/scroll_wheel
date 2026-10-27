// Una macro del editor → los pasos que reproduce el propio teclado.
//
// Antes una macro se subía tal cual si todos sus pasos eran de los que el
// firmware sabía tocar (espera, atajo, clic, movimiento relativo), y si no se
// quedaba entera en el PC. Desde el firmware 4.7 casi todo tiene traducción:
// el texto se convierte en pulsaciones, abrir una app es Win+R + la ruta,
// suspender es un informe de System Control… Este módulo hace esa traducción.
//
// Lo que sigue sin tenerla (un complemento, un carácter fuera de
// Windows-1252, una posición de ratón sin saber de qué pantalla) devuelve
// { ok: false, reason } y la macro se queda en el PC, como antes.
//
// Módulo puro: sin DOM ni window.orby, para poder probarlo con node --test.

import { compileText, MOD } from './layouts.mjs';
import { compileRecording, toAbs } from './recording.mjs';
import { MAX_BLOB } from './pool.mjs';

// MacroStepType en main.cpp.
export const STEP = {
  DELAY: 1, KEY: 2, MOVE: 3, CLICK: 4, HOME_MOVE: 5,
  CONSUMER: 6, TEXT: 7, SYSTEM: 8, ABS_MOVE: 9, EVENTS: 10, PREFER_PC: 11, LOOP: 12,
};

// Los que apuntan al pool: llevan su trozo aparte y `a`/`gap` los rellena
// quien sabe dónde ha quedado (macros-store.js).
export const POOL_STEPS = [STEP.TEXT, STEP.EVENTS];

export const MAX_STEPS = 48; // MACRO_MAX_STEPS en main.cpp
export const CLICK_BUTTON_CODE = { left: 0, middle: 1, right: 2 };
const SYSCTL = { POWER_DOWN: 1, SLEEP: 2 };

// El hueco por defecto entre repeticiones (DEFAULT_REPEAT_GAP_MS en macros-store.js).
const DEFAULT_REPEAT_GAP_MS = 20;

// Tabla de multimedia del firmware: índice → uso de la página de consumo.
// Copia de get_consumer_key_from_index en main.cpp; un test comprueba que no
// se desincronizan.
export const CONSUMER_USAGE = {
  1: 0x0194, 2: 0x0192, 3: 0x00CD, 4: 0x00B7, 5: 0x00B5, 6: 0x00B6, 7: 0x00E9, 8: 0x00EA,
  9: 0x00E2, 10: 0x006F, 11: 0x0070, 12: 0x0196, 13: 0x018A, 14: 0x0221, 15: 0x022D, 16: 0x022E,
};

// El paso «tecla» que graba el editor pulsando teclas de verdad: KeyboardEvent.code.
function codeToHid(code) {
  const c = String(code || '');
  let m = /^Key([A-Z])$/.exec(c);
  if (m) return 0x04 + m[1].charCodeAt(0) - 65;
  m = /^Digit([0-9])$/.exec(c);
  if (m) return m[1] === '0' ? 0x27 : 0x1E + Number(m[1]) - 1;
  return { Enter: 0x28, Escape: 0x29, Space: 0x2C }[c] ?? null;
}

const HID_R = 0x15;
const HID_L = 0x0F;
const HID_ENTER = 0x28;

// Cuánto tarda en salir el diálogo Ejecutar tras Win+R. Corto, y el texto se
// pierde por delante; de sobra, y solo se nota como una pausa al abrir.
export const RUN_DIALOG_WAIT_MS = 500;

// Lo que se escribe en Ejecutar para abrir `target`. Una ruta con espacios va
// entre comillas, o Ejecutar intentaría abrir lo que hay antes del primero;
// una URL o un `shell:` van tal cual.
export function runCommandFor(target) {
  const t = String(target || '').trim();
  if (!t) return '';
  if (t.startsWith('"') || !/\s/.test(t)) return t;
  if (/^[a-z][a-z0-9+.-]*:\/\//i.test(t) || /^shell:/i.test(t)) return t;
  return `"${t}"`;
}

// Las acciones de energía que no tienen informe HID propio se piden a
// Windows por Ejecutar. `shutdown` es igual en cualquier idioma, a diferencia
// de navegar el menú de Win+X.
const POWER_RUN = {
  hibernate: 'shutdown /h',
  restart: 'shutdown /r /t 0',
  shutdown: 'shutdown /s /t 0',
  logoff: 'shutdown /l',
};

// Lo que el PC hace mejor si está delante: con estos pasos, la macro lleva la
// marca PREFER_PC y el teclado se la pasa a la app cuando está viva (pegar el
// texto en vez de teclearlo, abrir sin enseñar Ejecutar, SetCursorPos en vez
// de un puntero absoluto sin probar en varias pantallas).
const PC_IS_BETTER = new Set(['text', 'open_app', 'system_power', 'mouse_position']);

const step = (type, a = 0, b = 0, repeat = 1, gap = 0) => ({ type, a, b, repeat, gap });

function repeatOf(a) {
  const repeat = Math.max(1, Math.min(255, a.count || 1));
  const gap = repeat > 1 ? Math.max(0, Math.min(32767, Math.round(a.gap ?? DEFAULT_REPEAT_GAP_MS))) : 0;
  return { repeat, gap };
}

// Un texto en uno o varios pasos TEXT: cada trozo cabe en el `b` de un paso,
// y se corta siempre en pares completos.
function textSteps(text, layout) {
  const r = compileText(text, layout);
  if (!r.ok) return { ok: false, reason: `el carácter «${r.char}» no se puede escribir desde el teclado` };
  const out = [];
  const max = MAX_BLOB - (MAX_BLOB % 2);
  for (let i = 0; i < r.bytes.length; i += max) {
    out.push({ ...step(STEP.TEXT), blob: r.bytes.slice(i, i + max) });
  }
  return { ok: true, steps: out };
}

function runSteps(command, layout) {
  const typed = textSteps(command, layout);
  if (!typed.ok) return typed;
  return {
    ok: true,
    steps: [
      step(STEP.KEY, MOD.GUI, HID_R),
      step(STEP.DELAY, RUN_DIALOG_WAIT_MS),
      ...typed.steps,
      step(STEP.DELAY, 50),
      step(STEP.KEY, 0, HID_ENTER),
    ],
  };
}

function actionSteps(a, opts) {
  switch (a.type) {
    case 'delay':
      return { ok: true, steps: [step(STEP.DELAY, Math.max(0, Math.min(32767, Math.round(a.ms) || 0)))] };

    case 'hotkey': {
      const { repeat, gap } = repeatOf(a);
      return { ok: true, steps: [step(STEP.KEY, a.modifier || 0, a.keycode || 0, repeat, gap)] };
    }

    case 'key': {
      const key = codeToHid(a.code);
      if (key === null) return { ok: false, reason: `la tecla «${a.code}» no tiene equivalente HID` };
      const { repeat, gap } = repeatOf(a);
      return { ok: true, steps: [step(STEP.KEY, 0, key, repeat, gap)] };
    }

    case 'mouse_move':
      return { ok: true, steps: [step(STEP.MOVE, a.dx || 0, a.dy || 0)] };

    case 'mouse_click': {
      const { repeat, gap } = repeatOf(a);
      return { ok: true, steps: [step(STEP.CLICK, CLICK_BUTTON_CODE[a.button] ?? 0, 0, repeat, gap)] };
    }

    case 'consumer': {
      const usage = CONSUMER_USAGE[a.index];
      if (!usage) return { ok: false, reason: 'acción multimedia desconocida' };
      const { repeat, gap } = repeatOf(a);
      return { ok: true, steps: [step(STEP.CONSUMER, usage, 0, repeat, gap)] };
    }

    case 'text':
      if (!a.text) return { ok: true, steps: [] };
      return textSteps(a.text, opts.layout);

    case 'open_app': {
      const command = runCommandFor(a.target);
      if (!command) return { ok: true, steps: [] };
      return runSteps(command, opts.layout);
    }

    case 'system_power':
      if (a.mode === 'lock') return { ok: true, steps: [step(STEP.KEY, MOD.GUI, HID_L)] };
      if (a.mode === 'sleep') return { ok: true, steps: [step(STEP.SYSTEM, SYSCTL.SLEEP)] };
      if (POWER_RUN[a.mode]) return runSteps(POWER_RUN[a.mode], opts.layout);
      return { ok: false, reason: 'acción de energía desconocida' };

    case 'mouse_position': {
      // La pantalla de la captura viaja con el paso; si es de antes de que
      // se guardara, vale la del PC que tenga la app delante.
      const desk = a.desk || opts.desk;
      if (!desk) return { ok: false, reason: 'no se sabe el tamaño de la pantalla de esta posición' };
      const [x, y] = toAbs(a.x, a.y, desk);
      return { ok: true, steps: [step(STEP.ABS_MOVE, x, y)] };
    }

    case 'center_mouse':
      return { ok: false, reason: 'el «centrar ratón» antiguo solo lo sabe hacer el PC' };

    case 'plugin':
      return { ok: false, reason: 'los complementos necesitan la red del PC' };

    default:
      return { ok: false, reason: `paso «${a.type}» desconocido` };
  }
}

// Las secuencias de antes del 4.7: solo los pasos que el firmware siempre supo
// tocar. Con un teclado sin NATIVE=1 es lo único que se puede subir.
const CLASSIC = new Set(['delay', 'hotkey', 'mouse_move', 'mouse_click']);

// macro → { ok: true, steps, preferPc } | { ok: false, reason }.
//
//   opts.native: el teclado tiene NATIVE=1 (si no, solo valen los CLASSIC).
//   opts.layout: distribución de teclado del PC (layouts.mjs).
//   opts.desk:   escritorio virtual de este PC {x, y, w, h}, si se sabe.
export function compileMacro(m, opts = {}) {
  if (!m) return { ok: false, reason: 'no existe' };

  if (m.kind === 'recording-reset') return { ok: false, reason: 'borrar una grabación lo hace el PC' };

  if (m.kind === 'recording') {
    if (!opts.native) return { ok: false, reason: 'este firmware no reproduce grabaciones' };
    if (!m.events?.length) return { ok: false, reason: 'todavía no hay nada grabado' };
    const r = compileRecording(m.events, { desk: m.desk || opts.desk, speed: m.speed });
    if (!r.ok) return r;
    const steps = [step(STEP.PREFER_PC), ...r.chunks.map((blob) => ({ ...step(STEP.EVENTS), blob }))];
    if (m.mode === 'loop') steps.push(step(STEP.LOOP, 0));
    else if (m.mode === 'hold') steps.push(step(STEP.LOOP, 1));
    if (steps.length > MAX_STEPS) return { ok: false, reason: 'la grabación es demasiado larga para el teclado' };
    return { ok: true, steps, preferPc: true };
  }

  const actions = m.actions || [];
  if (!actions.length) return { ok: false, reason: 'no tiene pasos' };

  if (!opts.native && !actions.every((a) => CLASSIC.has(a.type))) {
    return { ok: false, reason: 'este firmware solo reproduce esperas, atajos y clics' };
  }

  const steps = [];
  for (const a of actions) {
    const r = actionSteps(a, opts);
    if (!r.ok) return r;
    steps.push(...r.steps);
  }
  if (!steps.length) return { ok: false, reason: 'no tiene pasos' };

  const preferPc = actions.some((a) => PC_IS_BETTER.has(a.type));
  if (preferPc) steps.unshift(step(STEP.PREFER_PC));

  if (steps.length > MAX_STEPS) {
    return { ok: false, reason: `ocupa más de ${MAX_STEPS} pasos en el teclado` };
  }
  return { ok: true, steps, preferPc };
}
