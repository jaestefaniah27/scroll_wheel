// Texto → pulsaciones HID, para que el teclado lo escriba sin la app.
//
// El teclado manda posiciones de tecla, no caracteres: qué sale al pulsar la
// tecla 0x1F lo decide la distribución que tenga puesta Windows («2» con Shift
// es «"» en España y «@» en EE. UU.). Por eso el texto se traduce aquí, con la
// distribución que dice el usuario, y al teclado le llega ya hecho pulsaciones.
//
// Lo que no está en la distribución sale por código Alt+teclado numérico
// (Alt+0241 = ñ), que cubre Windows-1252. Fuera de eso (emoji, ∑, CJK) no hay
// forma fiable de escribirlo desde un teclado HID, y la secuencia se queda en
// el PC.
//
// Módulo puro: sin DOM ni window.orby, para poder probarlo con node --test.

// Modificadores HID (byte de modificadores del informe de teclado).
export const MOD = { CTRL: 0x01, SHIFT: 0x02, ALT: 0x04, GUI: 0x08, ALTGR: 0x40 };

// Órdenes del reproductor de texto (TEXT_HOLD_MODS/TEXT_RELEASE_MODS en main.cpp).
export const TEXT_HOLD_MODS = 0xF0;
export const TEXT_RELEASE_MODS = 0xF1;

const KEY_ENTER = 0x28;
const KEY_TAB = 0x2B;
const KEY_SPACE = 0x2C;
const KP = [0x62, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61]; // 0..9 del numérico

// Letras y dígitos: iguales en todas las distribuciones que se ofrecen.
function base() {
  const t = new Map();
  for (let i = 0; i < 26; i++) {
    const lower = String.fromCharCode(97 + i);
    t.set(lower, [0, 0x04 + i]);
    t.set(lower.toUpperCase(), [MOD.SHIFT, 0x04 + i]);
  }
  for (let d = 1; d <= 9; d++) t.set(String(d), [0, 0x1E + d - 1]);
  t.set('0', [0, 0x27]);
  t.set(' ', [0, KEY_SPACE]);
  return t;
}

// [carácter, modificador, tecla HID]. Las teclas se nombran por su sitio en
// un teclado de EE. UU., que es como las numera el HID.
function add(t, rows) {
  for (const [ch, mod, key] of rows) if (!t.has(ch)) t.set(ch, [mod, key]);
}

const S = MOD.SHIFT;
const G = MOD.ALTGR;

// Español (España), KBDSP de Windows.
function layoutEs() {
  const t = base();
  add(t, [
    ['!', S, 0x1E], ['"', S, 0x1F], ['·', S, 0x20], ['$', S, 0x21], ['%', S, 0x22],
    ['&', S, 0x23], ['/', S, 0x24], ['(', S, 0x25], [')', S, 0x26], ['=', S, 0x27],
    ['|', G, 0x1E], ['@', G, 0x1F], ['#', G, 0x20], ['¬', G, 0x23], ['€', G, 0x08],
    ["'", 0, 0x2D], ['?', S, 0x2D],
    ['¡', 0, 0x2E], ['¿', S, 0x2E],
    ['+', 0, 0x30], ['*', S, 0x30], [']', G, 0x30],
    ['[', G, 0x2F],
    ['ç', 0, 0x31], ['Ç', S, 0x31], ['}', G, 0x31],
    ['ñ', 0, 0x33], ['Ñ', S, 0x33],
    ['{', G, 0x34],
    ['º', 0, 0x35], ['ª', S, 0x35], ['\\', G, 0x35],
    [',', 0, 0x36], [';', S, 0x36],
    ['.', 0, 0x37], [':', S, 0x37],
    ['-', 0, 0x38], ['_', S, 0x38],
    ['<', 0, 0x64], ['>', S, 0x64],
  ]);
  // Teclas muertas: se pulsan y el carácter sale con la letra siguiente. Solas
  // (seguidas de espacio) dan el acento suelto.
  const dead = {
    '`': [0, 0x2F], '^': [S, 0x2F], '´': [0, 0x34], '¨': [S, 0x34], '~': [G, 0x21],
  };
  const compose = {
    '`': { a: 'à', e: 'è', i: 'ì', o: 'ò', u: 'ù', A: 'À', E: 'È', I: 'Ì', O: 'Ò', U: 'Ù' },
    '´': { a: 'á', e: 'é', i: 'í', o: 'ó', u: 'ú', y: 'ý', A: 'Á', E: 'É', I: 'Í', O: 'Ó', U: 'Ú', Y: 'Ý' },
    '^': { a: 'â', e: 'ê', i: 'î', o: 'ô', u: 'û', A: 'Â', E: 'Ê', I: 'Î', O: 'Ô', U: 'Û' },
    '¨': { a: 'ä', e: 'ë', i: 'ï', o: 'ö', u: 'ü', y: 'ÿ', A: 'Ä', E: 'Ë', I: 'Ï', O: 'Ö', U: 'Ü' },
    '~': { a: 'ã', o: 'õ', n: 'ñ', A: 'Ã', O: 'Õ', N: 'Ñ' },
  };
  return { keys: t, dead, compose };
}

// Inglés (EE. UU.), KBDUS: sin teclas muertas.
function layoutUs() {
  const t = base();
  add(t, [
    ['!', S, 0x1E], ['@', S, 0x1F], ['#', S, 0x20], ['$', S, 0x21], ['%', S, 0x22],
    ['^', S, 0x23], ['&', S, 0x24], ['*', S, 0x25], ['(', S, 0x26], [')', S, 0x27],
    ['-', 0, 0x2D], ['_', S, 0x2D], ['=', 0, 0x2E], ['+', S, 0x2E],
    ['[', 0, 0x2F], ['{', S, 0x2F], [']', 0, 0x30], ['}', S, 0x30],
    ['\\', 0, 0x31], ['|', S, 0x31], [';', 0, 0x33], [':', S, 0x33],
    ["'", 0, 0x34], ['"', S, 0x34], ['`', 0, 0x35], ['~', S, 0x35],
    [',', 0, 0x36], ['<', S, 0x36], ['.', 0, 0x37], ['>', S, 0x37],
    ['/', 0, 0x38], ['?', S, 0x38],
  ]);
  return { keys: t, dead: {}, compose: {} };
}

export const LAYOUTS = {
  es: { label: 'Español (España)', build: layoutEs },
  us: { label: 'Inglés (EE. UU.)', build: layoutUs },
};

// La que toca si el usuario no ha elegido: la del idioma del sistema.
export function defaultLayout(lang) {
  return String(lang || '').toLowerCase().startsWith('es') ? 'es' : 'us';
}

const built = new Map();
function layout(id) {
  if (!built.has(id)) built.set(id, (LAYOUTS[id] || LAYOUTS.us).build());
  return built.get(id);
}

// Los 32 caracteres de Windows-1252 entre 0x80 y 0x9F que no coinciden con
// Latin-1. El resto de 0xA0-0xFF es el mismo código que en Unicode.
const CP1252_HIGH = {
  '€': 0x80, '‚': 0x82, 'ƒ': 0x83, '„': 0x84, '…': 0x85, '†': 0x86, '‡': 0x87,
  'ˆ': 0x88, '‰': 0x89, 'Š': 0x8A, '‹': 0x8B, 'Œ': 0x8C, 'Ž': 0x8E, '‘': 0x91,
  '’': 0x92, '“': 0x93, '”': 0x94, '•': 0x95, '–': 0x96, '—': 0x97, '˜': 0x98,
  '™': 0x99, 'š': 0x9A, '›': 0x9B, 'œ': 0x9C, 'ž': 0x9E, 'Ÿ': 0x9F,
};

export function cp1252Code(ch) {
  if (ch in CP1252_HIGH) return CP1252_HIGH[ch];
  const c = ch.codePointAt(0);
  if (c >= 0x20 && c <= 0x7E) return c;
  if (c >= 0xA0 && c <= 0xFF) return c;
  return null;
}

// Alt + 0nnn en el numérico. El 0 delante elige Windows-1252 y no la página de
// códigos OEM (850 en España), que daría otro carácter.
function altCode(code) {
  const digits = String(code).padStart(4, '0');
  return [[MOD.ALT, TEXT_HOLD_MODS], ...[...digits].map((d) => [0, KP[Number(d)]]), [0, TEXT_RELEASE_MODS]];
}

// Las pulsaciones de un carácter, o null si no hay forma de escribirlo.
function strokesOf(ch, l) {
  const direct = l.keys.get(ch);
  if (direct) return [direct];

  // El acento suelto: tecla muerta seguida de espacio.
  if (l.dead[ch]) return [l.dead[ch], [0, KEY_SPACE]];

  // Letra acentuada: tecla muerta y luego la letra.
  for (const [accent, table] of Object.entries(l.compose)) {
    for (const [letter, result] of Object.entries(table)) {
      if (result === ch && l.dead[accent] && l.keys.get(letter)) return [l.dead[accent], l.keys.get(letter)];
    }
  }

  const code = cp1252Code(ch);
  if (code !== null) return altCode(code);
  return null;
}

// Texto → bytes del pool (pares modificador, tecla). Devuelve
// { ok: true, bytes } o { ok: false, char } con el primer carácter que no se
// puede escribir.
export function compileText(text, layoutId) {
  const l = layout(layoutId);
  const out = [];
  const chars = [...String(text ?? '')];
  for (let i = 0; i < chars.length; i++) {
    const ch = chars[i];
    // Saltos de línea y tabuladores como teclas de verdad, igual que en el PC
    // (ver escribir_texto en macros.rs). «\r\n» es un salto, no dos.
    if (ch === '\r') {
      if (chars[i + 1] === '\n') i++;
      out.push([0, KEY_ENTER]);
      continue;
    }
    if (ch === '\n') { out.push([0, KEY_ENTER]); continue; }
    if (ch === '\t') { out.push([0, KEY_TAB]); continue; }

    const strokes = strokesOf(ch, l);
    if (!strokes) return { ok: false, char: ch };
    out.push(...strokes);
  }
  const bytes = new Uint8Array(out.length * 2);
  out.forEach(([mod, key], i) => { bytes[i * 2] = mod; bytes[i * 2 + 1] = key; });
  return { ok: true, bytes };
}
