// Vuelca a PNG el mapa del juego Misil para ver carreteras, estáticos y pantallas sin jugar.
// El PNG se escribe a mano: así no hace falta canvas ni dependencias.

import { deflateSync } from 'node:zlib';
import { writeFileSync } from 'node:fs';
import { construirMapa, ESTATICOS, ZONAS, MAPA_W, MAPA_H } from '../../OrbyGUI/src/games/misil-mapa.mjs';
import { OBJETIVOS, orientar, dibujoCrater, MUNDO_W, MUNDO_H, HUECO_X, HUECO_Y } from '../../OrbyGUI/src/games/misil.mjs';
import { W, H } from '../../OrbyGUI/src/games/dibujo.mjs';
import { crc32 } from '../../OrbyGUI/src/web/macro-blob.mjs';

const [salida, zonaArg] = process.argv.slice(2);
if (!salida) {
  console.error('uso: node tools/misil/ver-mapa.mjs <salida.png> [zona | hoja]');
  process.exit(1);
}

// `hoja`: todos los dibujos de objetivos (con su fotograma 2) y, debajo, cada uno junto al cráter que deja,
// para comprobar a ojo que un cráter no se confunde con un tanque ni con un depósito.
function hoja() {
  const HW = 190, celdas = [];
  let x = 2, y = 2, alto = 0;
  const poner = (d, fondo) => {
    if (x + d.w + 4 > HW) { x = 2; y += alto + 4; alto = 0; }
    celdas.push({ d, x, y, fondo }); x += d.w + 4; alto = Math.max(alto, d.h);
  };
  for (const o of OBJETIVOS) { poner(o); if (o.filas2) poner({ ...o, filas: o.filas2 }); }
  x = 2; y += alto + 8; alto = 0;
  for (const n of ['tanque', 'deposito', 'camion', 'tienda', 'soldado', 'caza']) {
    const o = OBJETIVOS.find((q) => q.nombre === n);
    poner(o); poner(dibujoCrater({ x: 100, y: 100, w: o.w, h: o.h, tipo: n }), true);
  }
  const HH = y + alto + 4, im = new Uint8Array(HW * HH);
  for (const { d, x: ox, y: oy } of celdas) d.filas.forEach((f, j) => [...f].forEach((c, i) => { if (c === '#') im[(oy + j) * HW + ox + i] = 255; }));
  escribirPng(salida, HW, HH, 5, (px, py) => im[py * HW + px]);
}

// Un estático sin dibujo no tumba el volcado: se pinta un cuadrado para que se vea igual.
const piezas = ESTATICOS.map(([nombre, x, y, dir = 'der']) => {
  const obj = OBJETIVOS.find((o) => o.nombre === nombre);
  if (!obj) console.warn(`sin dibujo para "${nombre}": cuadrado de 5x5`);
  const s = obj ? orientar(obj, dir) : { w: 5, h: 5, filas: Array(5).fill('#####') };
  return { x, y, ...s };
});

const mapa = construirMapa(piezas.map(({ x, y, w, h }) => ({ x, y, w, h })));
const img = new Uint8Array(MAPA_W * MAPA_H);
for (let i = 0; i < img.length; i++) img[i] = mapa[i] ? 255 : 0;

const en = (x, y) => x >= 0 && y >= 0 && x < MAPA_W && y < MAPA_H;
const pon = (x, y, v) => { if (en(x, y)) img[y * MAPA_W + x] = v; };

// Cada estático lleva su halo negro antes de su dibujo: así se separa de la carretera.
for (const p of piezas) {
  for (let y = p.y - 1; y <= p.y + p.h; y++)
    for (let x = p.x - 1; x <= p.x + p.w; x++) pon(x, y, 0);
  p.filas.forEach((f, j) => [...f].forEach((c, i) => { if (c === '#') pon(p.x + i, p.y + j, 255); }));
}

// Contorno de cada pantalla. Solo sobre negro, para no tapar el dibujo del mapa.
const pasoX = W + HUECO_X, pasoY = H + HUECO_Y;
const contorno = (x, y) => { if (en(x, y) && img[y * MAPA_W + x] === 0) img[y * MAPA_W + x] = 90; };
for (const z of ZONAS) {
  for (let j = 0; j < 3; j++) for (let i = 0; i < 3; i++) {
    const x0 = z.x + i * pasoX, y0 = z.y + j * pasoY;
    for (let x = x0; x < x0 + W; x++) { contorno(x, y0); contorno(x, y0 + H - 1); }
    for (let y = y0; y < y0 + H; y++) { contorno(x0, y); contorno(x0 + W - 1, y); }
  }
}

// PNG en escala de grises de 8 bits. `gris(x, y)` mira la imagen sin escalar; cada píxel sale k×k.
function escribirPng(ruta, w, h, k, gris) {
  const ancho = w * k, alto = h * k;
  const filas = Buffer.alloc((ancho + 1) * alto);   // byte de filtro 0 al principio de cada fila
  for (let Y = 0; Y < alto; Y++) {
    for (let X = 0; X < ancho; X++) filas[Y * (ancho + 1) + 1 + X] = gris(Math.floor(X / k), Math.floor(Y / k));
  }
  const chunk = (tipo, datos) => {
    const t = Buffer.from(tipo, 'latin1');
    const len = Buffer.alloc(4); len.writeUInt32BE(datos.length);
    const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(Buffer.concat([t, datos])));
    return Buffer.concat([len, t, datos, crc]);
  };
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(ancho, 0); ihdr.writeUInt32BE(alto, 4);
  ihdr[8] = 8; ihdr[9] = 0;                          // profundidad 8, tipo 0 (gris); el resto a cero
  writeFileSync(ruta, Buffer.concat([
    Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]),
    chunk('IHDR', ihdr), chunk('IDAT', deflateSync(filas)), chunk('IEND', Buffer.alloc(0)),
  ]));
}

if (zonaArg === 'hoja') {
  hoja();
} else if (zonaArg === undefined) {
  escribirPng(salida, MAPA_W, MAPA_H, 2, (x, y) => img[y * MAPA_W + x]);
} else {
  const idx = /^\d+$/.test(zonaArg) ? Number(zonaArg) : ZONAS.findIndex((z) => z.nombre === zonaArg);
  const z = ZONAS[idx];
  if (!z) {
    console.error(`zona desconocida "${zonaArg}". Zonas: ${ZONAS.map((q) => q.nombre).join(', ')}`);
    process.exit(1);
  }
  // Lo que ve el jugador: los huecos entre pantallas van en gris 40 para verlas separadas.
  escribirPng(salida, MUNDO_W, MUNDO_H, 4, (x, y) => {
    const enPantalla = x % pasoX < W && y % pasoY < H;
    return enPantalla ? img[(z.y + y) * MAPA_W + z.x + x] : 40;
  });
}
console.log(`escrito ${salida}`);
