// Grabar una operación del PC y volver a reproducirla.
//
// A diferencia de una secuencia (que se monta paso a paso en el editor y, si
// cabe, la toca el propio teclado), esto captura lo que el usuario hace de
// verdad —ratón y teclado, con sus tiempos— para repetirlo tal cual. Por eso
// vive entero en el PC: el teclado no puede ver lo que haces con el ratón.
//
// La captura la hace uiohook-napi, que trae binarios precompilados para
// win32-x64, así que no hace falta node-gyp ni Visual Studio Build Tools. La
// reproducción va por nut.js, el mismo que ya usan las secuencias.

const { uIOhook, UiohookKey } = require('uiohook-napi');

let nutjs = null;
let nutjsFailed = false;

function loadNutJs() {
  if (nutjs || nutjsFailed) return nutjs;
  try {
    nutjs = require('@nut-tree-fork/nut-js');
    // Los tiempos los pone la propia grabación: el hueco automático de nut.js
    // solo desharía el ritmo original.
    nutjs.keyboard.config.autoDelayMs = 0;
    nutjs.mouse.config.autoDelayMs = 0;
  } catch (err) {
    nutjsFailed = true;
    console.error('No se pudo cargar @nut-tree-fork/nut-js:', err.message);
  }
  return nutjs;
}

// --- Traducción de teclas ---------------------------------------------------
// uiohook da su propio código numérico; nut.js quiere su enum. Los nombres
// coinciden en casi todo, así que se construye el puente por nombre y solo se
// listan a mano los 42 que se llaman distinto.
const UIOHOOK_TO_NUT_NAME = {
  ArrowLeft: 'Left', ArrowUp: 'Up', ArrowRight: 'Right', ArrowDown: 'Down',
  Backquote: 'Grave', BracketLeft: 'LeftBracket', BracketRight: 'RightBracket',
  PrintScreen: 'Print',
  Ctrl: 'LeftControl', CtrlRight: 'RightControl',
  Alt: 'LeftAlt', AltRight: 'RightAlt',
  Shift: 'LeftShift', ShiftRight: 'RightShift',
  Meta: 'LeftSuper', MetaRight: 'RightSuper',
  Numpad0: 'NumPad0', Numpad1: 'NumPad1', Numpad2: 'NumPad2', Numpad3: 'NumPad3',
  Numpad4: 'NumPad4', Numpad5: 'NumPad5', Numpad6: 'NumPad6', Numpad7: 'NumPad7',
  Numpad8: 'NumPad8', Numpad9: 'NumPad9',
  NumpadMultiply: 'Multiply', NumpadAdd: 'Add', NumpadSubtract: 'Subtract',
  NumpadDecimal: 'Decimal', NumpadDivide: 'Divide', NumpadEnter: 'Enter',
  // Teclas del bloque numérico con Bloq Num apagado: uiohook las nombra por su
  // función y nut.js por su número, que es la misma tecla física.
  NumpadEnd: 'NumPad1', NumpadArrowDown: 'NumPad2', NumpadPageDown: 'NumPad3',
  NumpadArrowLeft: 'NumPad4', NumpadArrowRight: 'NumPad6',
  NumpadHome: 'NumPad7', NumpadArrowUp: 'NumPad8', NumpadPageUp: 'NumPad9',
  NumpadInsert: 'NumPad0', NumpadDelete: 'Decimal',
};

// código de uiohook -> nombre de nut.js
const KEYCODE_TO_NUT_NAME = new Map();
for (const [name, code] of Object.entries(UiohookKey)) {
  const target = UIOHOOK_TO_NUT_NAME[name] || name;
  if (!KEYCODE_TO_NUT_NAME.has(code)) KEYCODE_TO_NUT_NAME.set(code, target);
}

function nutKey(code) {
  const nut = loadNutJs();
  const name = KEYCODE_TO_NUT_NAME.get(code);
  if (!nut || !name) return undefined;
  return nut.Key[name];
}

// libuiohook numera 1 = izquierdo, 2 = derecho, 3 = central.
const BUTTON_TO_NUT = { 1: 'LEFT', 2: 'RIGHT', 3: 'MIDDLE' };

// --- Captura ----------------------------------------------------------------

// El ratón dispara cientos de eventos por segundo. Guardarlos todos hincha la
// grabación sin que se note al reproducir, así que se queda uno cada 16 ms (y
// solo si el puntero se ha movido de verdad).
const MOVE_MIN_MS = 16;
const MOVE_MIN_PX = 3;

let hooked = false;      // uIOhook.start() ya llamado
let recording = null;    // { events, t0, lastMoveAt, lastX, lastY }

function ensureHook() {
  if (hooked) return;
  uIOhook.on('keydown', (e) => push({ k: 'kdown', c: e.keycode }));
  uIOhook.on('keyup',   (e) => push({ k: 'kup',   c: e.keycode }));
  uIOhook.on('mousedown', (e) => push({ k: 'mdown', b: e.button, x: e.x, y: e.y }));
  uIOhook.on('mouseup',   (e) => push({ k: 'mup',   b: e.button, x: e.x, y: e.y }));
  uIOhook.on('wheel', (e) => push({ k: 'wheel', r: e.rotation, d: e.direction }));
  uIOhook.on('mousemove', onMove);
  uIOhook.on('mousedrag', onMove);
  uIOhook.start();
  hooked = true;
}

function onMove(e) {
  if (!recording) return;
  const now = Date.now();
  if (now - recording.lastMoveAt < MOVE_MIN_MS) return;
  if (Math.abs(e.x - recording.lastX) < MOVE_MIN_PX
   && Math.abs(e.y - recording.lastY) < MOVE_MIN_PX) return;
  recording.lastMoveAt = now;
  recording.lastX = e.x;
  recording.lastY = e.y;
  push({ k: 'move', x: e.x, y: e.y });
}

function push(ev) {
  if (!recording) return;
  ev.t = Date.now() - recording.t0;
  recording.events.push(ev);
}

function startRecording() {
  ensureHook();
  const now = Date.now();
  recording = { events: [], t0: now, lastMoveAt: 0, lastX: -9999, lastY: -9999 };
}

// Devuelve lo grabado. Se recorta el hueco muerto del final (desde el último
// evento hasta que se pulsa la tecla para parar): si no, cada repetición en
// bucle arrastraría esa pausa.
function stopRecording() {
  if (!recording) return [];
  const events = recording.events;
  recording = null;
  return events;
}

function isRecording() {
  return recording !== null;
}

// --- Reproducción -----------------------------------------------------------

let playback = null;  // { id, mode, stop }

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function applyEvent(ev) {
  const nut = loadNutJs();
  if (!nut) return;
  const { mouse, keyboard, Point, Button } = nut;

  if (ev.k === 'move') {
    await mouse.setPosition(new Point(ev.x, ev.y));
  } else if (ev.k === 'mdown' || ev.k === 'mup') {
    // La pulsación va donde estaba el puntero al grabarla: sin esto, un clic
    // caería donde el ratón se hubiera quedado si el movimiento previo se
    // descartó por el filtro de MOVE_MIN_PX.
    if (Number.isFinite(ev.x)) await mouse.setPosition(new Point(ev.x, ev.y));
    const button = Button[BUTTON_TO_NUT[ev.b] || 'LEFT'];
    if (ev.k === 'mdown') await mouse.pressButton(button);
    else await mouse.releaseButton(button);
  } else if (ev.k === 'kdown' || ev.k === 'kup') {
    const key = nutKey(ev.c);
    if (key === undefined) return;
    if (ev.k === 'kdown') await keyboard.pressKey(key);
    else await keyboard.releaseKey(key);
  } else if (ev.k === 'wheel') {
    // direction 3 = vertical, 4 = horizontal (libuiohook)
    const amount = Math.abs(ev.r) || 1;
    if (ev.d === 4) await (ev.r < 0 ? mouse.scrollLeft(amount) : mouse.scrollRight(amount));
    else await (ev.r < 0 ? mouse.scrollUp(amount) : mouse.scrollDown(amount));
  }
}

// Suelta lo que la grabación hubiera dejado hundido al cortarla a mitad: sin
// esto, parar un bucle mientras tenía Ctrl pulsado dejaría el Ctrl puesto.
async function releaseHeld(held) {
  const nut = loadNutJs();
  if (!nut) return;
  for (const code of held.keys) {
    const key = nutKey(code);
    if (key !== undefined) { try { await nut.keyboard.releaseKey(key); } catch { /* nada que hacer */ } }
  }
  for (const button of held.buttons) {
    const b = nut.Button[BUTTON_TO_NUT[button] || 'LEFT'];
    try { await nut.mouse.releaseButton(b); } catch { /* nada que hacer */ }
  }
}

async function runPass(events, session, held) {
  const start = Date.now();
  for (const ev of events) {
    if (session.stopped) return;
    const wait = ev.t - (Date.now() - start);
    if (wait > 0) await sleep(wait);
    if (session.stopped) return;

    if (ev.k === 'kdown') held.keys.add(ev.c);
    else if (ev.k === 'kup') held.keys.delete(ev.c);
    else if (ev.k === 'mdown') held.buttons.add(ev.b);
    else if (ev.k === 'mup') held.buttons.delete(ev.b);

    try {
      await applyEvent(ev);
    } catch (err) {
      console.error('Reproduciendo grabación:', err.message);
    }
  }
}

// `mode`: 'once' (una vez), 'loop' (hasta que se vuelva a pulsar) o 'hold'
// (mientras la tecla siga pulsada). `onEnd` avisa para poder devolver la
// pantalla de la tecla a su estado normal.
async function startPlayback(id, events, mode, onEnd) {
  if (!events?.length) return;
  const session = { id, mode, stopped: false };
  playback = session;

  const held = { keys: new Set(), buttons: new Set() };
  try {
    do {
      await runPass(events, session, held);
    } while (!session.stopped && mode !== 'once');
  } finally {
    await releaseHeld(held);
    if (playback === session) playback = null;
    // Quien corta la reproducción para sustituirla por otra cosa (el borrado)
    // ya se encarga de dejar la tecla como toca: avisar aquí llegaría tarde y
    // le devolvería el icono anterior por encima.
    if (!session.silent) onEnd?.();
  }
}

// `silent` corta sin avisar del final. La suelta de la reproducción no es
// inmediata (se está esperando al siguiente evento), así que ese aviso llega
// cuando el que la cortó ya ha hecho su trabajo.
function stopPlayback(silent = false) {
  if (playback) {
    playback.silent = silent;
    playback.stopped = true;
  }
}

function playingId() {
  return playback ? playback.id : null;
}

function playingMode() {
  return playback ? playback.mode : null;
}

// Al cerrar la app: sin esto el hilo nativo del hook mantiene el proceso vivo.
function shutdown() {
  stopPlayback();
  recording = null;
  if (hooked) {
    try { uIOhook.stop(); } catch { /* ya estaba parado */ }
    hooked = false;
  }
}

module.exports = {
  startRecording, stopRecording, isRecording,
  startPlayback, stopPlayback, playingId, playingMode,
  shutdown,
};
