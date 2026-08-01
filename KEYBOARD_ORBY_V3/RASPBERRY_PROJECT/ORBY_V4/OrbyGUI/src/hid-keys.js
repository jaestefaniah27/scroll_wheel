// Tabla de códigos HID (Usage Page 0x07) usada por el editor de perfiles.
// El firmware guarda cada acción como { modifier, keycode }; esta tabla traduce
// esos bytes a algo legible y permite construirlos desde la interfaz.

export const MODIFIERS = [
  { bit: 0x01, label: 'Ctrl',    short: 'Ctrl'  },
  { bit: 0x02, label: 'Shift',   short: 'Shift' },
  { bit: 0x04, label: 'Alt',     short: 'Alt'   },
  { bit: 0x08, label: 'Win',     short: 'Win'   },
  { bit: 0x10, label: 'Ctrl der',  short: 'RCtrl'  },
  { bit: 0x20, label: 'Shift der', short: 'RShift' },
  { bit: 0x40, label: 'AltGr',     short: 'AltGr'  },
  { bit: 0x80, label: 'Win der',   short: 'RWin'   },
];

// modifier == 0xFE marca una acción multimedia; keycode es el índice de la
// tabla get_consumer_key_from_index() del firmware.
export const CONSUMER_MODIFIER = 0xFE;

// Acciones de página, del firmware 4.0. Como las multimedia, viajan en el campo
// del modificador, donde un atajo normal nunca llega.
export const GOTO_PAGE_MODIFIER  = 0xFD;  // keycode = número de página, base 1
export const PAGE_STATE_MODIFIER = 0xFC;  // la tecla enseña la página y abre el gestor

// keycode = id de una macro (secuencia) del PC, guardada en la configuración
// local. El firmware no la ejecuta: solo manda MACRO:<id> por CDC.
export const MACRO_MODIFIER = 0xFB;

// Índices de la tabla get_consumer_key_from_index() del firmware.
export const CONSUMER_ACTIONS = [
  { index: 7,  label: 'Subir volumen' },
  { index: 8,  label: 'Bajar volumen' },
  { index: 9,  label: 'Silenciar' },
  { index: 10, label: 'Subir brillo' },
  { index: 11, label: 'Bajar brillo' },
  { index: 3,  label: 'Reproducir / Pausa' },
  { index: 4,  label: 'Detener' },
  { index: 5,  label: 'Pista siguiente' },
  { index: 6,  label: 'Pista anterior' },
  { index: 12, label: 'Abrir navegador' },
  { index: 1,  label: 'Abrir explorador' },
  { index: 2,  label: 'Abrir calculadora' },
  { index: 13, label: 'Abrir correo' },
  { index: 14, label: 'Buscar' },
  { index: 15, label: 'Acercar (zoom +)' },
  { index: 16, label: 'Alejar (zoom −)' },
];

// Tipos de acción para encoders y rueda. Deben coincidir con el enum
// RotaryType del firmware.
export const ROTARY_TYPES = {
  NONE: 0,
  CONSUMER: 1,
  KEY: 2,
  SCROLL_V: 3,
  SCROLL_H: 4,
  ZOOM: 5,
};

// Huecos de acciones giratorias dentro de cada perfil (enum RotarySlot).
export const ROTARY_SLOTS = {
  ENC1_CW: 0, ENC1_CCW: 1, ENC1_CLICK: 2,
  ENC2_CW: 3, ENC2_CCW: 4, ENC2_CLICK: 5,
  WHEEL_CW: 6, WHEEL_CCW: 7,
};

// Los tipos de desplazamiento llevan el sentido en el signo: una sola acción
// cubre los dos giros, así que el firmware ignora la del sentido contrario.
export function isScrollType(type) {
  return type === ROTARY_TYPES.SCROLL_V
      || type === ROTARY_TYPES.SCROLL_H
      || type === ROTARY_TYPES.ZOOM;
}

export function describeRotary(action) {
  if (!action) return 'Sin asignar';
  switch (action.type) {
    case ROTARY_TYPES.SCROLL_V: return 'Desplazar vertical';
    case ROTARY_TYPES.SCROLL_H: return 'Desplazar horizontal';
    case ROTARY_TYPES.ZOOM:     return 'Zoom (Ctrl + rueda)';
    case ROTARY_TYPES.CONSUMER: {
      const c = CONSUMER_ACTIONS.find((a) => a.index === action.keycode);
      return c ? c.label : `Multimedia ${action.keycode}`;
    }
    case ROTARY_TYPES.KEY:      return describeAction(action.modifier, action.keycode);
    default:                    return 'Sin asignar';
  }
}

function range(from, to, fn) {
  const out = [];
  for (let i = from; i <= to; i++) out.push(fn(i));
  return out;
}

export const KEY_GROUPS = [
  {
    name: 'Letras',
    keys: range(0, 25, (i) => ({ code: 0x04 + i, label: String.fromCharCode(65 + i) })),
  },
  {
    name: 'Números',
    keys: [
      ...range(0, 8, (i) => ({ code: 0x1e + i, label: String(i + 1) })),
      { code: 0x27, label: '0' },
    ],
  },
  {
    name: 'Función',
    keys: [
      ...range(0, 11, (i) => ({ code: 0x3a + i, label: `F${i + 1}` })),
      ...range(0, 11, (i) => ({ code: 0x68 + i, label: `F${i + 13}` })),
    ],
  },
  {
    name: 'Edición',
    keys: [
      { code: 0x28, label: 'Enter' },
      { code: 0x29, label: 'Esc' },
      { code: 0x2a, label: 'Retroceso' },
      { code: 0x2b, label: 'Tab' },
      { code: 0x2c, label: 'Espacio' },
      { code: 0x49, label: 'Insert' },
      { code: 0x4c, label: 'Supr' },
      { code: 0x4a, label: 'Inicio' },
      { code: 0x4d, label: 'Fin' },
      { code: 0x4b, label: 'Re Pág' },
      { code: 0x4e, label: 'Av Pág' },
      { code: 0x46, label: 'Impr Pant' },
    ],
  },
  {
    name: 'Flechas',
    keys: [
      { code: 0x50, label: '←' },
      { code: 0x4f, label: '→' },
      { code: 0x52, label: '↑' },
      { code: 0x51, label: '↓' },
    ],
  },
  {
    name: 'Símbolos',
    keys: [
      { code: 0x2d, label: '-' },
      { code: 0x2e, label: '=' },
      { code: 0x2f, label: '[' },
      { code: 0x30, label: ']' },
      { code: 0x31, label: '\\' },
      { code: 0x33, label: ';' },
      { code: 0x34, label: "'" },
      { code: 0x35, label: '`' },
      { code: 0x36, label: ',' },
      { code: 0x37, label: '.' },
      { code: 0x38, label: '/' },
    ],
  },
  {
    name: 'Teclado numérico',
    keys: [
      { code: 0x54, label: 'Num /' },
      { code: 0x55, label: 'Num *' },
      { code: 0x56, label: 'Num -' },
      { code: 0x57, label: 'Num +' },
      { code: 0x58, label: 'Num Enter' },
      ...range(1, 9, (i) => ({ code: 0x58 + i, label: `Num ${i}` })),
      { code: 0x62, label: 'Num 0' },
    ],
  },
];

const CODE_TO_LABEL = new Map();
for (const group of KEY_GROUPS) {
  for (const k of group.keys) CODE_TO_LABEL.set(k.code, k.label);
}

export function keyLabel(code) {
  return CODE_TO_LABEL.get(code) || (code ? `0x${code.toString(16).toUpperCase()}` : '—');
}

// Descripción legible de una acción tal y como la almacena el firmware.
export function describeAction(modifier, keycode) {
  if (modifier === CONSUMER_MODIFIER) {
    const action = CONSUMER_ACTIONS.find((a) => a.index === keycode);
    return action ? action.label : `Multimedia ${keycode}`;
  }
  if (modifier === GOTO_PAGE_MODIFIER)  return `Ir a la página ${keycode}`;
  if (modifier === PAGE_STATE_MODIFIER) return 'Estado de páginas';
  if (modifier === MACRO_MODIFIER)      return `Secuencia ${keycode}`;
  if (!modifier && !keycode) return 'Sin asignar';

  const parts = MODIFIERS.filter((m) => modifier & m.bit).map((m) => m.short);
  if (keycode) parts.push(keyLabel(keycode));
  return parts.join(' + ');
}

// Traduce KeyboardEvent.code al usage HID equivalente, para poder capturar un
// atajo pulsándolo directamente en el teclado del PC.
const EVENT_CODE_TO_HID = (() => {
  const map = new Map();
  for (let i = 0; i < 26; i++) map.set(`Key${String.fromCharCode(65 + i)}`, 0x04 + i);
  for (let i = 1; i <= 9; i++) map.set(`Digit${i}`, 0x1e + i - 1);
  map.set('Digit0', 0x27);
  for (let i = 1; i <= 24; i++) map.set(`F${i}`, i <= 12 ? 0x3a + i - 1 : 0x68 + i - 13);

  const direct = {
    Enter: 0x28, Escape: 0x29, Backspace: 0x2a, Tab: 0x2b, Space: 0x2c,
    Minus: 0x2d, Equal: 0x2e, BracketLeft: 0x2f, BracketRight: 0x30,
    Backslash: 0x31, Semicolon: 0x33, Quote: 0x34, Backquote: 0x35,
    Comma: 0x36, Period: 0x37, Slash: 0x38, CapsLock: 0x39,
    PrintScreen: 0x46, ScrollLock: 0x47, Pause: 0x48, Insert: 0x49,
    Home: 0x4a, PageUp: 0x4b, Delete: 0x4c, End: 0x4d, PageDown: 0x4e,
    ArrowRight: 0x4f, ArrowLeft: 0x50, ArrowDown: 0x51, ArrowUp: 0x52,
    NumLock: 0x53, NumpadDivide: 0x54, NumpadMultiply: 0x55,
    NumpadSubtract: 0x56, NumpadAdd: 0x57, NumpadEnter: 0x58, Numpad0: 0x62,
  };
  for (const [k, v] of Object.entries(direct)) map.set(k, v);
  for (let i = 1; i <= 9; i++) map.set(`Numpad${i}`, 0x58 + i);
  return map;
})();

export function eventToAction(e) {
  const keycode = EVENT_CODE_TO_HID.get(e.code) || 0;
  let modifier = 0;
  if (e.ctrlKey)  modifier |= 0x01;
  if (e.shiftKey) modifier |= 0x02;
  if (e.altKey)   modifier |= 0x04;
  if (e.metaKey)  modifier |= 0x08;
  return { modifier, keycode };
}
