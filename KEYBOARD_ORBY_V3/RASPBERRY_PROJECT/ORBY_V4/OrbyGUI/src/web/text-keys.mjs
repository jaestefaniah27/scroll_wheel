// De un texto a las teclas HID que lo escriben, para que lo teclee el propio
// teclado (paso MSTEP_TEXT del firmware 4.20) sin la app delante.
//
// Un teclado USB no manda letras sino posiciones de tecla, y qué carácter sale
// de cada una lo decide la distribución que el PC tenga puesta. Aquí solo está la
// española (ISO, la de Windows en España): con otra distribución en el PC, el
// mismo atajo escribiría otros caracteres. Por eso lo que no se puede traducir
// con certeza devuelve null y la macro se queda en el camino del PC, que escribe
// el texto por el sistema y no depende de la distribución.

const SHIFT = 0x02;
const ALTGR = 0x40;

const K = { ENTER: 0x28, TAB: 0x2B, SPACE: 0x2C, ACUTE: 0x34, GRAVE: 0x2F };

// carácter -> [modificador, tecla]
const MAPA = new Map();
const poner = (c, mod, key) => MAPA.set(c, [mod, key]);

for (let i = 0; i < 26; i++) {
  poner(String.fromCharCode(97 + i), 0, 0x04 + i);
  poner(String.fromCharCode(65 + i), SHIFT, 0x04 + i);
}
// Fila de números: [sin nada, con Mayús, con AltGr]
const FILA = [
  [0x1E, '1', '!', '|'], [0x1F, '2', '"', '@'], [0x20, '3', '·', '#'], [0x21, '4', '$', null],
  [0x22, '5', '%', '€'], [0x23, '6', '&', '¬'], [0x24, '7', '/', null], [0x25, '8', '(', null],
  [0x26, '9', ')', null], [0x27, '0', '=', null],
];
for (const [key, base, shift, altgr] of FILA) {
  poner(base, 0, key);
  poner(shift, SHIFT, key);
  if (altgr) poner(altgr, ALTGR, key);
}
const SUELTAS = [
  [0x35, 'º', 'ª', '\\'], [0x2D, "'", '?', null], [0x30, '+', '*', null],
  [0x31, 'ç', 'Ç', null], [0x33, 'ñ', 'Ñ', null], [0x36, ',', ';', null],
  [0x37, '.', ':', null], [0x38, '-', '_', null], [0x64, '<', '>', null],
  [0x2E, '¡', '¿', null],
];
for (const [key, base, shift, altgr] of SUELTAS) {
  poner(base, 0, key);
  poner(shift, SHIFT, key);
  if (altgr) poner(altgr, ALTGR, key);
}
// Corchetes y llaves: en la distribución española no están en la fila de números
// sino en AltGr + las teclas de la derecha de la P y de la Ñ.
poner('[', ALTGR, K.GRAVE);
poner(']', ALTGR, 0x30);
poner('{', ALTGR, K.ACUTE);
poner('}', ALTGR, 0x31);

// Acentos: son teclas muertas, así que se teclean antes que la vocal.
// [tecla muerta, modificador de la tecla muerta]
const ACENTOS = {
  'áéíóú': [K.ACUTE, 0], 'ÁÉÍÓÚ': [K.ACUTE, 0],
  'àèìòù': [K.GRAVE, 0], 'ÀÈÌÒÙ': [K.GRAVE, 0],
  'âêîôû': [K.GRAVE, SHIFT], 'ÂÊÎÔÛ': [K.GRAVE, SHIFT],
  'äëïöü': [K.ACUTE, SHIFT], 'ÄËÏÖÜ': [K.ACUTE, SHIFT],
};
const VOCAL = { á: 'a', é: 'e', í: 'i', ó: 'o', ú: 'u', à: 'a', è: 'e', ì: 'i', ò: 'o', ù: 'u',
                â: 'a', ê: 'e', î: 'i', ô: 'o', û: 'u', ä: 'a', ë: 'e', ï: 'i', ö: 'o', ü: 'u' };

// Las que son una tecla muerta por sí solas necesitan un espacio detrás para
// salir como carácter (y ~ también: AltGr+4 es muerta en la distribución española).
const MUERTAS_SOLAS = {
  '´': [0, K.ACUTE], '¨': [SHIFT, K.ACUTE], '`': [0, K.GRAVE], '^': [SHIFT, K.GRAVE], '~': [ALTGR, 0x21],
};

function teclasDe(c) {
  if (c === '\n') return [[0, K.ENTER]];
  if (c === '\t') return [[0, K.TAB]];
  if (c === ' ') return [[0, K.SPACE]];
  const directa = MAPA.get(c);
  if (directa) return [directa];
  if (MUERTAS_SOLAS[c]) return [MUERTAS_SOLAS[c], [0, K.SPACE]];
  for (const [vocales, [muerta, mod]] of Object.entries(ACENTOS)) {
    const i = [...vocales].indexOf(c);
    if (i < 0) continue;
    const mayus = c !== c.toLowerCase();
    const base = mayus ? VOCAL[c.toLowerCase()].toUpperCase() : VOCAL[c];
    return [[mod, muerta], MAPA.get(base)];
  }
  return null;
}

// [{ mod, key }, ...] para el texto, o null si algún carácter no se sabe teclear
// (emojis, letras de otros alfabetos, saltos de línea de Windows sueltos...).
export function textToKeys(texto) {
  const salida = [];
  for (const c of String(texto).replace(/\r\n?/g, '\n')) {
    const teclas = teclasDe(c);
    if (!teclas) return null;
    for (const [mod, key] of teclas) salida.push({ mod, key });
  }
  return salida;
}

// Las teclas en pasos del firmware: tres por paso, cada una como
// (modificador << 8) | tecla, con 0 de relleno (ver MSTEP_TEXT en main.cpp).
// Un `{ a, b, gap }` por paso; el que llama le pone el tipo.
export function textToSteps(texto) {
  const teclas = textToKeys(texto);
  if (!teclas) return null;
  const pasos = [];
  for (let i = 0; i < teclas.length; i += 3) {
    const [a = 0, b = 0, gap = 0] = teclas.slice(i, i + 3).map((t) => (t.mod << 8) | t.key);
    pasos.push({ a, b, gap });
  }
  return pasos;
}
