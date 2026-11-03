// El mapa de Misil: un territorio grande del que las nueve pantallas solo ven una zona.
// Aparte del juego para que éste se lea de un vistazo y para poder probar el mapa sin jugar.
//
// El dibujo fijo (terreno, río, carreteras, muros) se pinta una vez en un mapa de bits del
// tamaño del territorio; el juego copia de él la ventana que mira la cámara. Lo que se puede
// destruir (ESTATICOS) y lo que circula va aparte, encima.
//
// Decisiones:
//   · estilo de mapa cartográfico, no de foto: líneas limpias y pocas. A un bit por píxel, las
//     motas sueltas de «arena» o «hierba» no se leían como terreno sino como ruido, y
//     competían con los objetivos. Cada rasgo es una forma que se reconoce: el río con sus
//     dos orillas, las colinas sombreadas a trazos (las «hachures» de los mapas antiguos), las
//     dunas con su cresta y la ladera rayada, palmeras, muros con torres, la pista de aterrizaje
//   · lo destruible se reconoce por ser macizo y estar rodeado de negro (el halo). El decorado
//     es de trazo: así no se confunde una roca con un soldado
//   · las carreteras son solo horizontales y verticales: un vehículo tiene cuatro orientaciones
//     y siempre se le puede dibujar de cara a donde va
//   · tres zonas (ZONAS), cada una con su carácter: el puesto del desierto, el puente sobre el
//     río y la base con la pista. Están colocadas para que lo importante caiga dentro de las
//     pantallas y no en los huecos entre ellas (las carreteras, en el centro de una fila)
//   · el mapa es siempre el mismo: la gracia está en lo que se mueve, y un mapa que cambia
//     cada partida no deja aprender dónde están las cosas

export const MAPA_W = 600;
export const MAPA_H = 440;
export const SEMI = 4;            // la carretera mide 2·SEMI+1 de ancho, bordes incluidos
const RIO_SEMI = 9;               // medio ancho del río, de orilla a orilla

// Dónde mira la cámara en cada zona: la esquina de arriba a la izquierda de la vista (las
// nueve pantallas con sus huecos, 248x148).
export const ZONAS = [
  { nombre: 'puesto', x: 20, y: 20 },
  { nombre: 'puente', x: 300, y: 126 },
  { nombre: 'base', x: 60, y: 266 },
];

// La red de carreteras: [x, y] del centro. Los extremos son callejones sin salida (o el borde
// del mapa), donde el vehículo da la vuelta.
export const NODOS = {
  a: [0, 200], b: [144, 200], c: [184, 200], d: [512, 200], e: [MAPA_W - 1, 200],
  f: [144, 104],                  // la puerta del puesto
  g: [184, 324],                  // dentro de la base, hasta la pista
  h: [512, MAPA_H - 1],
};
export const TRAMOS = ['ab', 'bc', 'cd', 'de', 'bf', 'cg', 'dh'];

export const ADYACENTES = {};
for (const [p, q] of TRAMOS) {
  (ADYACENTES[p] ??= []).push(q);
  (ADYACENTES[q] ??= []).push(p);
}

const SEGMENTOS = TRAMOS.map(([p, q]) => {
  const [x0, y0] = NODOS[p];
  const [x1, y1] = NODOS[q];
  return { x0: Math.min(x0, x1), x1: Math.max(x0, x1), y0: Math.min(y0, y1), y1: Math.max(y0, y1), horizontal: y0 === y1 };
});

function dentro(s, x, y, semi, ext) {
  return s.horizontal
    ? x >= s.x0 - ext && x <= s.x1 + ext && Math.abs(y - s.y0) <= semi
    : y >= s.y0 - ext && y <= s.y1 + ext && Math.abs(x - s.x0) <= semi;
}

// ¿Cae (x, y) en una carretera, con `margen` de más a cada lado?
export function enCarretera(x, y, margen = 0) {
  return SEGMENTOS.some((s) => dentro(s, x, y, SEMI + margen, SEMI + margen));
}

// Lo que se puede destruir y está quieto: [tipo, x, y, dirección], con (x, y) la esquina de
// arriba a la izquierda. Por zonas, y dentro de cada una por pantallas.
export const ESTATICOS = [
  // --- El puesto del desierto: un recinto amurallado con la puerta al sur.
  ['tienda', 120, 33], ['tienda', 137, 33], ['tienda', 154, 33],
  ['camion', 118, 80, 'arr'], ['tanque', 155, 78, 'aba'],
  ['soldado', 128, 88], ['soldado', 133, 92],
  // fuera: una patrulla en las dunas, un nido junto a la carretera y un campamento al este
  ['soldado', 50, 90], ['soldado', 56, 94],
  ['soldado', 160, 140],
  ['tienda', 210, 82], ['tienda', 230, 92], ['camion', 214, 104, 'der'],
  ['soldado', 60, 140],

  // --- El puente: un control en la orilla este y un aparcamiento de camiones.
  ['soldado', 446, 186], ['soldado', 446, 211],
  ['camion', 486, 136, 'aba'], ['camion', 504, 136, 'aba'], ['camion', 522, 136, 'aba'],
  ['tienda', 490, 213], ['soldado', 532, 214],
  ['tanque', 330, 150, 'der'],
  ['tienda', 316, 244], ['tienda', 334, 250], ['soldado', 352, 246], ['soldado', 339, 212],
  ['tanque', 395, 255, 'izq'],

  // --- La base: depósitos de combustible, carros en fila, camiones y tiendas.
  ['deposito', 240, 281], ['deposito', 254, 281], ['deposito', 268, 281], ['deposito', 247, 294], ['deposito', 261, 294],
  ['tanque', 68, 394, 'arr'], ['tanque', 84, 394, 'arr'], ['tanque', 100, 394, 'arr'],
  ['camion', 156, 394, 'arr'], ['camion', 166, 394, 'arr'], ['camion', 176, 394, 'arr'],
  ['tienda', 244, 394], ['tienda', 262, 394], ['soldado', 282, 400], ['soldado', 286, 406],
  ['soldado', 200, 300], ['soldado', 112, 322],
];

// --- Dibujo ---------------------------------------------------------------------------

class Bits {
  constructor(w, h) { this.w = w; this.h = h; this.d = new Uint8Array(w * h); }
  px(x, y, v = 1) {
    x = Math.round(x); y = Math.round(y);
    if (x >= 0 && y >= 0 && x < this.w && y < this.h) this.d[y * this.w + x] = v;
  }
  get(x, y) { return x >= 0 && y >= 0 && x < this.w && y < this.h ? this.d[y * this.w + x] : 0; }
  rect(x, y, w, h, v = 1) { for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) this.px(x + i, y + j, v); }
  marco(x, y, w, h, v = 1) {
    for (let i = 0; i < w; i++) { this.px(x + i, y, v); this.px(x + i, y + h - 1, v); }
    for (let j = 0; j < h; j++) { this.px(x, y + j, v); this.px(x + w - 1, y + j, v); }
  }
  linea(x0, y0, x1, y1, v = 1) {
    const n = Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0), 1);
    for (let k = 0; k <= n; k++) this.px(x0 + ((x1 - x0) * k) / n, y0 + ((y1 - y0) * k) / n, v);
  }
  circulo(cx, cy, r, v = 1) {
    const n = Math.max(12, Math.round(r * 8));
    for (let k = 0; k < n; k++) this.px(cx + r * Math.cos((k / n) * 2 * Math.PI), cy + r * Math.sin((k / n) * 2 * Math.PI), v);
  }
}

// Curva suave que pasa por los puntos (Catmull-Rom), muestreada cada medio píxel o menos.
function curva(puntos) {
  const out = [];
  for (let i = 0; i < puntos.length - 1; i++) {
    const p0 = puntos[Math.max(0, i - 1)], p1 = puntos[i], p2 = puntos[i + 1], p3 = puntos[Math.min(puntos.length - 1, i + 2)];
    const n = Math.ceil(Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) * 3);
    for (let k = 0; k < n; k++) {
      const t = k / n, t2 = t * t, t3 = t2 * t;
      const f = (a, b, c, d) => 0.5 * (2 * b + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
      out.push([f(p0[0], p1[0], p2[0], p3[0]), f(p0[1], p1[1], p2[1], p3[1])]);
    }
  }
  return out;
}

// El río: dos orillas y, dentro, ondas sueltas de tres píxeles. `agua` marca lo que es río,
// para las ondas y para tender los puentes.
function rio(b, agua) {
  const c = curva([[352, 0], [372, 50], [396, 110], [420, 200], [428, 236], [458, 258], [512, 264], [560, 278], [MAPA_W + 40, 300]]);
  for (let i = 1; i < c.length; i++) {
    const [x0, y0] = c[i - 1], [x1, y1] = c[i];
    const l = Math.hypot(x1 - x0, y1 - y0) || 1;
    const nx = -(y1 - y0) / l, ny = (x1 - x0) / l;
    for (let d = -RIO_SEMI; d <= RIO_SEMI; d += 0.5) agua.px(x1 + nx * d, y1 + ny * d);
  }
  // Las orillas: lo que es agua y toca algo que no lo es.
  for (let y = 0; y < b.h; y++) {
    for (let x = 0; x < b.w; x++) {
      if (!agua.get(x, y)) continue;
      if (!agua.get(x - 1, y) || !agua.get(x + 1, y) || !agua.get(x, y - 1) || !agua.get(x, y + 1)) b.px(x, y);
    }
  }
  // Ondas: un «~» aplastado, en una rejilla al tresbolillo, solo donde cabe entero con aire.
  for (let y = 2; y < b.h; y += 5) {
    for (let x = (Math.floor(y / 5) % 2) * 6; x < b.w; x += 12) {
      let cabe = true;
      for (let j = -2; j <= 2 && cabe; j++) for (let i = -2; i <= 5 && cabe; i++) if (!agua.get(x + i, y + j) || b.get(x + i, y + j)) cabe = false;
      if (cabe) { b.px(x, y); b.px(x + 1, y - 1); b.px(x + 2, y - 1); b.px(x + 3, y); }
    }
  }
}

// Una meseta: el borde, de un trazo continuo, y los dientes del cortado hacia fuera, cortos
// en la cara de luz y largos en la de sombra (abajo a la derecha). Dentro, una curva de nivel
// discontinua. Probadas antes las «hachures» sin contorno: a un bit salían manchas peludas.
function colina(b, cx, cy, rx, ry, semilla = 0) {
  const forma = (a) => 1 + 0.13 * Math.sin(3 * a + semilla) + 0.07 * Math.sin(5 * a + 2 * semilla);
  const punto = (a, e) => [cx + rx * e * forma(a) * Math.cos(a), cy + ry * e * forma(a) * Math.sin(a)];
  const n = Math.round(Math.PI * (rx + ry) * 3);
  let dist = 0;
  let prev = punto(0, 1);
  for (let k = 0; k <= n; k++) {
    const a = (k / n) * 2 * Math.PI;
    const [x, y] = punto(a, 1);
    b.px(x, y);
    dist += Math.hypot(x - prev[0], y - prev[1]);
    prev = [x, y];
    if (dist >= 3) {
      dist = 0;
      let nx = Math.cos(a) / rx, ny = Math.sin(a) / ry;
      const l = Math.hypot(nx, ny); nx /= l; ny /= l;
      const largo = nx + ny > 0.2 ? 4 : nx + ny > -0.6 ? 2 : 1;
      b.linea(x + nx, y + ny, x + nx * largo, y + ny * largo);
    }
    if (k % 6 < 3) { const [ix, iy] = punto(a, 0.5); b.px(ix, iy); }
  }
}

// Una duna: la cresta, curva y larga, y debajo la ladera de sotavento rayada en vertical,
// más larga en el centro.
function duna(b, x, y, largo, alto) {
  for (let i = 0; i <= largo; i++) {
    const t = i / largo;
    const yc = y - alto * Math.sin(t * Math.PI) + 2 * Math.sin(t * 2 * Math.PI);
    b.px(x + i, yc);
    if (i % 3 === 1 && t > 0.12 && t < 0.88) {
      const h = Math.round(1 + alto * 0.6 * Math.sin(t * Math.PI));
      for (let j = 2; j <= h + 1; j++) b.px(x + i, yc + j);
    }
  }
}

// Palmera vista desde arriba: cinco hojas.
function palmera(b, x, y) {
  for (const [i, j] of [[0, -2], [-2, -1], [2, -1], [-1, 1], [1, 1], [0, 0], [-2, 2], [2, 2], [0, -1]]) b.px(x + i, y + j);
}

// Roca: un bloque de trazo, para que no parezca un soldado.
function roca(b, x, y, grande) {
  if (grande) { b.linea(x + 1, y, x + 4, y); b.px(x, y + 1); b.px(x + 5, y + 1); b.px(x, y + 2); b.px(x + 5, y + 2); b.linea(x + 1, y + 3, x + 4, y + 3); b.px(x + 2, y + 1); }
  else { b.linea(x + 1, y, x + 2, y); b.px(x, y + 1); b.px(x + 3, y + 1); b.linea(x + 1, y + 2, x + 2, y + 2); }
}

// Un nido de sacos terreros: medio anillo abierto hacia `hacia` ('arr', 'aba', 'izq', 'der').
function nido(b, cx, cy, hacia) {
  const base = { aba: -Math.PI / 2, arr: Math.PI / 2, der: Math.PI, izq: 0 }[hacia];
  for (let k = -9; k <= 9; k++) {
    const a = base + (k / 9) * (Math.PI * 0.62);
    b.px(cx + 4 * Math.cos(a), cy + 4 * Math.sin(a));
    b.px(cx + 5 * Math.cos(a), cy + 5 * Math.sin(a));
  }
}

function torre(b, x, y) { b.rect(x - 2, y - 2, 5, 5); b.px(x, y, 0); }

// Muro doble con torres en las esquinas y un hueco de puerta en el lado `puerta` centrado en
// `px` (o `py`).
function recinto(b, x, y, w, h, puerta, centro, ancho) {
  const hueco = (i, lado) => lado === puerta && Math.abs(i - centro) <= ancho;
  for (let i = x; i < x + w; i++) {
    for (const [yy, lado] of [[y, 'arr'], [y + h - 1, 'aba']]) {
      if (hueco(i, lado)) continue;
      b.px(i, yy); b.px(i, yy + (lado === 'arr' ? 2 : -2));
    }
  }
  for (let j = y; j < y + h; j++) {
    for (const [xx, lado] of [[x, 'izq'], [x + w - 1, 'der']]) {
      if (hueco(j, lado)) continue;
      b.px(xx, j); b.px(xx + (lado === 'izq' ? 2 : -2), j);
    }
  }
  for (const [tx, ty] of [[x, y], [x + w - 1, y], [x, y + h - 1], [x + w - 1, y + h - 1]]) torre(b, tx + (tx === x ? 1 : -1), ty + (ty === y ? 1 : -1));
  if (puerta === 'aba' || puerta === 'arr') {
    const yy = puerta === 'aba' ? y + h - 2 : y + 1;
    torre(b, centro - ancho - 3, yy); torre(b, centro + ancho + 3, yy);
  }
}

// Valla: una línea con postes hacia fuera cada pocos píxeles; con un hueco de puerta.
function valla(b, x, y, w, h, puertaX, ancho) {
  for (let i = x; i < x + w; i++) {
    if (Math.abs(i - puertaX) > ancho) { b.px(i, y); if ((i - x) % 5 === 0) b.px(i, y - 1); }
    b.px(i, y + h - 1); if ((i - x) % 5 === 0) b.px(i, y + h);
  }
  for (let j = y; j < y + h; j++) {
    b.px(x, j); b.px(x + w - 1, j);
    if ((j - y) % 5 === 0) { b.px(x - 1, j); b.px(x + w, j); }
  }
}

// La pista: rectángulo, eje discontinuo y las barras de cabecera en los dos extremos.
function pista(b, x, y, w, h) {
  b.rect(x, y, w, h, 0);
  b.marco(x, y, w, h);
  const cy = y + Math.floor(h / 2);
  for (let i = x + 14; i < x + w - 14; i++) if ((i - x) % 10 < 5) b.px(i, cy);
  for (const ex of [x + 3, x + w - 10]) for (let k = 0; k < 4; k++) b.linea(ex, y + 3 + k * 3, ex + 6, y + 3 + k * 3);
}

// Hangar: nave con el tejado ondulado (rayas a lo largo).
function hangar(b, x, y, w, h) {
  b.rect(x - 1, y - 1, w + 2, h + 2, 0);
  b.marco(x, y, w, h);
  for (let j = y + 2; j < y + h - 1; j += 2) b.linea(x + 2, j, x + w - 3, j);
}

function helipuerto(b, cx, cy) {
  b.circulo(cx, cy, 8);
  b.linea(cx - 3, cy - 4, cx - 3, cy + 4); b.linea(cx + 3, cy - 4, cx + 3, cy + 4); b.linea(cx - 3, cy, cx + 3, cy);
}

// Carreteras: se limpia la calzada (así cruzan el río y lo que haya) y se pintan los bordes y
// la raya central. Donde pasan sobre agua, el puente: un pretil por fuera de cada borde.
function carreteras(b, agua) {
  for (const s of SEGMENTOS) {
    if (s.horizontal) b.rect(s.x0 - SEMI, s.y0 - SEMI, s.x1 - s.x0 + 2 * SEMI + 1, 2 * SEMI + 1, 0);
    else b.rect(s.x0 - SEMI, s.y0 - SEMI, 2 * SEMI + 1, s.y1 - s.y0 + 2 * SEMI + 1, 0);
  }
  for (const s of SEGMENTOS) {
    const [a0, a1] = s.horizontal ? [s.x0 - SEMI, s.x1 + SEMI] : [s.y0 - SEMI, s.y1 + SEMI];
    for (let t = a0; t <= a1; t++) {
      for (const lado of [-SEMI, SEMI]) {
        const x = s.horizontal ? t : s.x0 + lado;
        const y = s.horizontal ? s.y0 + lado : t;
        if (!SEGMENTOS.some((o) => o !== s && dentro(o, x, y, SEMI - 1, SEMI - 1))) b.px(x, y);
        // El puente: el pretil va por fuera, y se alarga un poco sobre la orilla.
        const fx = s.horizontal ? x : x + Math.sign(lado) * 2;
        const fy = s.horizontal ? y + Math.sign(lado) * 2 : y;
        const sobreAgua = [-3, 0, 3].some((d) => (s.horizontal ? agua.get(x + d, s.y0) : agua.get(s.x0, y + d)));
        if (sobreAgua) { b.px(fx, fy); if (t % 4 === 0) b.px(s.horizontal ? fx : fx + Math.sign(lado), s.horizontal ? fy + Math.sign(lado) : fy); }
      }
      const pos = s.horizontal ? t - s.x0 : t - s.y0;
      const enTramo = s.horizontal ? t >= s.x0 && t <= s.x1 : t >= s.y0 && t <= s.y1;
      const [mx, my] = s.horizontal ? [t, s.y0] : [s.x0, t];
      if (enTramo && ((pos % 8) + 8) % 8 < 4 && !SEGMENTOS.some((o) => o !== s && dentro(o, mx, my, SEMI + 1, SEMI + 1))) b.px(mx, my);
    }
  }
}

let cache = null;

// El mapa de bits del territorio, con todo el dibujo fijo. Uint8Array de MAPA_W·MAPA_H, un
// byte por píxel (0/1). Se construye la primera vez y se reutiliza.
export function construirMapa(cajas = []) {
  if (cache && cache.cajas === cajas.length) return cache.d;
  const b = new Bits(MAPA_W, MAPA_H);
  const agua = new Bits(MAPA_W, MAPA_H);

  // Relieve.
  colina(b, 232, 42, 22, 11, 1);           // la mesa al este del puesto
  colina(b, 290, 70, 18, 12, 2);
  colina(b, 470, 60, 30, 16, 3);
  colina(b, 560, 120, 18, 10, 4);
  colina(b, 352, 300, 14, 8, 5);
  colina(b, 30, 270, 20, 12, 6);
  colina(b, 400, 360, 28, 14, 7);
  colina(b, 560, 390, 22, 12, 8);
  

  // Dunas: campos en el oeste del puesto, al sur y al sureste.
  for (const [x, y, l, h] of [
    [24, 40, 30, 5], [50, 56, 26, 4], [28, 74, 22, 4], [70, 30, 20, 4],
    [24, 118, 28, 5], [62, 112, 22, 4], [40, 160, 26, 4], [200, 130, 26, 4], [230, 160, 22, 4],
    [300, 250, 24, 4], [250, 230, 26, 5], [100, 250, 28, 5], [140, 240, 22, 4],
    [330, 410, 26, 4], [440, 420, 28, 5], [580, 220, 18, 4], [302, 140, 20, 4], [320, 270, 22, 4], [10, 380, 24, 4], [20, 410, 22, 4],
  ]) duna(b, x, y, l, h);

  // Rocas, en grupitos.
  for (const [x, y, g] of [
    [96, 140, 1], [102, 146, 0], [186, 70, 1], [80, 84, 0], [270, 110, 1], [276, 116, 0],
    [372, 250, 1], [366, 256, 0], [540, 170, 1], [460, 300, 1], [466, 306, 0], [580, 330, 0],
    [8, 210, 1], [40, 146, 1], [48, 152, 0], [34, 154, 0],
  ]) roca(b, x, y, g);

  rio(b, agua);

  // Palmeras a lo largo de las orillas, junto al puente.
  for (const [x, y] of [
    [402, 160], [410, 172], [396, 182], [436, 222], [444, 232], [430, 186], [438, 174],
    [470, 256], [480, 264], [494, 244], [380, 120], [386, 134], [530, 248], [544, 254],
    [410, 228], [404, 240], [416, 250], [452, 240], [470, 248],
  ]) if (![-3, 0, 3].some((d) => agua.get(x + d, y) || agua.get(x, y + d))) palmera(b, x, y);

  carreteras(b, agua);

  // El puesto del desierto: recinto amurallado con la puerta al sur, por donde entra la
  // carretera; nidos a los lados de la puerta y uno más abajo.
  recinto(b, 112, 26, 66, 80, 'aba', 144, 6);
  nido(b, 128, 116, 'aba'); nido(b, 162, 116, 'aba');
  nido(b, 74, 102, 'der');

  // El control del puente: dos garitas a los lados de la carretera, en la orilla este.
  torre(b, 452, 188); torre(b, 452, 212);
  nido(b, 340, 214, 'der');
  // El aparcamiento de camiones: un patio con valla, abierto a la carretera.
  valla(b, 480, 130, 60, 26, 512, 40);

  // La base: valla con la puerta al norte, la pista, hangares, helipuerto y los depósitos.
  valla(b, 66, 276, 238, 137, 184, 7);
  torre(b, 175, 276); torre(b, 193, 276);
  pista(b, 72, 334, 226, 16);
  hangar(b, 74, 288, 24, 14); hangar(b, 104, 288, 24, 14);
  helipuerto(b, 158, 292);
  nido(b, 212, 296, 'aba');

  // Debajo de lo destruible no queda dibujo: si no, al volarlo se quedaría ahí el borde de una
  // duna con forma de tienda.
  for (const c of cajas) b.rect(c.x - 1, c.y - 1, c.w + 2, c.h + 2, 0);

  cache = { cajas: cajas.length, d: b.d };
  return b.d;
}
