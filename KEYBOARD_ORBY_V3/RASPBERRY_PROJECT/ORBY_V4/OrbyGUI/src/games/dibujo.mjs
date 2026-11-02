// Primitivas de dibujo para los juegos, sobre los búferes de 72x40 de las pantallas.
//
// El formato es el de siempre del teclado (y de oled-fb.js): 5 páginas de 72 columnas, un
// bit por píxel, el bit 0 de cada byte arriba. Aquí se repite lo mínimo en vez de importar
// oled-fb.js porque aquel arrastra el código de canvas del editor de iconos y esto tiene
// que correr también en Node, en los tests.

import { screenText, screenIcon, screenKey } from '../oled-ui.mjs';
import { FB } from './proto.mjs';

export const W = 72;
export const H = 40;

export function px(fb, x, y, valor = 1) {
  x = Math.trunc(x);
  y = Math.trunc(y);
  if (x < 0 || x >= W || y < 0 || y >= H) return;
  const i = (y >> 3) * W + x;
  const bit = 1 << (y & 7);
  fb[i] = valor ? fb[i] | bit : fb[i] & ~bit;
}

export function leer(fb, x, y) {
  if (x < 0 || x >= W || y < 0 || y >= H) return 0;
  return (fb[(y >> 3) * W + x] >> (y & 7)) & 1;
}

export function rect(fb, x, y, w, h, valor = 1) {
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) px(fb, x + i, y + j, valor);
}

export function marco(fb, x, y, w, h, valor = 1) {
  for (let i = 0; i < w; i++) { px(fb, x + i, y, valor); px(fb, x + i, y + h - 1, valor); }
  for (let j = 0; j < h; j++) { px(fb, x, y + j, valor); px(fb, x + w - 1, y + j, valor); }
}

export function disco(fb, cx, cy, r, valor = 1) {
  for (let j = -r; j <= r; j++) {
    for (let i = -r; i <= r; i++) if (i * i + j * j <= r * r + r * 0.5) px(fb, cx + i, cy + j, valor);
  }
}

export function anillo(fb, cx, cy, r, valor = 1) {
  const dentro = (r - 1) * (r - 1);
  for (let j = -r; j <= r; j++) {
    for (let i = -r; i <= r; i++) {
      const d = i * i + j * j;
      if (d <= r * r + r * 0.5 && d >= dentro) px(fb, cx + i, cy + j, valor);
    }
  }
}

// Elipse rellena. Con un disco no salen ni los agujeros vistos en perspectiva ni las
// cabezas achatadas: a 40 px de alto, lo que se ve como «un óvalo» es lo que se reconoce.
export function elipse(fb, cx, cy, rx, ry, valor = 1) {
  for (let j = -ry; j <= ry; j++) {
    const ancho = Math.round(rx * Math.sqrt(Math.max(0, 1 - (j * j) / (ry * ry + ry * 0.5))));
    for (let i = -ancho; i <= ancho; i++) px(fb, cx + i, cy + j, valor);
  }
}

// Línea de un píxel, por si un trazo no cae en horizontal ni en vertical.
export function linea(fb, x0, y0, x1, y1, valor = 1) {
  const n = Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0), 1);
  for (let k = 0; k <= n; k++) px(fb, Math.round(x0 + ((x1 - x0) * k) / n), Math.round(y0 + ((y1 - y0) * k) / n), valor);
}

// Un destello de cuatro puntas: el «¡toma!» de un acierto, sin texto que leer.
export function destello(fb, cx, cy, r, valor = 1) {
  rect(fb, cx - r, cy, 2 * r + 1, 1, valor);
  rect(fb, cx, cy - r, 1, 2 * r + 1, valor);
  rect(fb, cx - 1, cy - 1, 3, 3, valor);
}

// Punta de flecha abierta, gruesa, con el vértice en (x, y). `dir` es -1 (izquierda) o 1.
export function punta(fb, x, y, alto, dir, grosor = 3, valor = 1) {
  for (let j = -alto; j <= alto; j++) {
    const bx = x - dir * Math.abs(j);
    for (let g = 0; g < grosor; g++) px(fb, bx - dir * g, y + j, valor);
  }
}

// Un pulsador con las esquinas romas: a 72x40, un rectángulo a secas parece un marco de
// foto y no un botón.
export function pastilla(fb, x, y, w, h, r, valor) {
  for (let j = 0; j < h; j++) {
    const dy = Math.max(0, r - j, j - (h - 1 - r));
    const recorte = dy ? r - Math.round(Math.sqrt(r * r - dy * dy)) : 0;
    rect(fb, x + recorte, y + j, w - 2 * recorte, 1, valor);
  }
}

export function invertir(fb) {
  for (let i = 0; i < FB; i++) fb[i] = ~fb[i] & 0xff;
}

// Texto lo más grande que quepa, centrado, en una pantalla entera.
export function texto(fb, s) {
  screenText(String(s), fb);
}

// El icono grande del catálogo del teclado para una etiqueta (las «#...» de los juegos, ver
// tools/oled_ui/catalog_juegos.py), o el texto de la propia etiqueta si no está.
export function icono(fb, etiqueta) {
  screenKey('', etiqueta, { big: true }, fb);
}

// Sube el contenido `dy` píxeles; lo que sale por arriba se pierde.
export function subir(fb, dy) {
  for (let x = 0; x < W; x++) {
    for (let y = 0; y < H; y++) px(fb, x, y, y + dy < H ? leer(fb, x, y + dy) : 0);
  }
}

// Un valor grande con su nombre debajo en letra pequeña ("PARTIDAS" / 12). El teclado no
// tiene una pantalla así de serie: se compone el texto grande, subido, con el rótulo.
export function etiquetaValor(fb, rotulo, valor) {
  const grande = new Uint8Array(FB);
  screenText(String(valor), grande);
  subir(grande, 7);
  const pie = screenIcon(null, rotulo, new Uint8Array(FB));
  for (let i = 0; i < FB; i++) fb[i] = grande[i] | pie[i];
}

// Varias pantallas vistas como un solo lienzo continuo. Cada tecla lleva su pantalla
// y entre ellas hay un hueco que no se ve: dibujar un objeto que cruza de una a otra
// sin contarlo hace que parezca que se frena al pasar. Con el hueco como "píxeles
// fantasma" el movimiento mantiene la velocidad y el objeto desaparece un instante
// entre pantallas, que es lo que haría de verdad.
//
// `filas` es la rejilla de pantallas, de arriba abajo y de izquierda a derecha, por
// número de pantalla (1-10). Las medidas del hueco son un cálculo, no una medición:
// valen para que el movimiento parezca razonable, no para dibujar a escala.
export class Malla {
  constructor(lienzo, filas, huecoX = 70, huecoY = 70) {
    this.lienzo = lienzo;
    this.filas = filas;
    this.huecoX = huecoX;
    this.huecoY = huecoY;
    this.ancho = filas[0].length * W + (filas[0].length - 1) * huecoX;
    this.alto = filas.length * H + (filas.length - 1) * huecoY;
  }

  // Dónde cae un punto del lienzo grande: qué pantalla y en qué punto de ella. Null si
  // cae en un hueco o fuera.
  destino(x, y) {
    x = Math.trunc(x);
    y = Math.trunc(y);
    if (x < 0 || y < 0 || x >= this.ancho || y >= this.alto) return null;
    const cx = Math.floor(x / (W + this.huecoX));
    const cy = Math.floor(y / (H + this.huecoY));
    const ox = x - cx * (W + this.huecoX);
    const oy = y - cy * (H + this.huecoY);
    if (ox >= W || oy >= H) return null;
    const pantalla = this.filas[cy]?.[cx];
    return pantalla ? { pantalla, x: ox, y: oy } : null;
  }

  px(x, y, valor = 1) {
    const d = this.destino(x, y);
    if (d) px(this.lienzo[d.pantalla - 1], d.x, d.y, valor);
  }

  rect(x, y, w, h, valor = 1) {
    for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) this.px(x + i, y + j, valor);
  }

  disco(cx, cy, r, valor = 1) {
    for (let j = -r; j <= r; j++) {
      for (let i = -r; i <= r; i++) if (i * i + j * j <= r * r + r * 0.5) this.px(cx + i, cy + j, valor);
    }
  }

  limpiar() {
    for (const fila of this.filas) for (const p of fila) this.lienzo[p - 1].fill(0);
  }
}
