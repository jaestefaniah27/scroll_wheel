// Transiciones de las pantallas: copia en JS de include/oled_anim.h.
//
// El dashboard tiene que moverse igual que el teclado, no "parecido": por eso
// esto no es una animación CSS que imite el gesto, sino el mismo motor. Mismos
// muelles de SpringBoard, mismo tramado Bayer para los fundidos en 1 bit, mismo
// escalado por vecino más cercano. Recibe framebuffers en el formato del
// SSD1306 (360 bytes, 5 páginas de 8 filas, bit 0 arriba) y devuelve otro.
//
// Si cambias una constante o una rama de oled_anim.h, cámbiala aquí igual.
// test/oled-anim.test.mjs comprueba duraciones y extremos para que no se
// desincronicen en silencio. Sin DOM ni nada de la app: se prueba con node.

export const W = 72;
export const H = 40;
export const FB = 360;
export const N = 10;

export const COL = [0, 1, 2, 0, 1, 2, 0, 1, 2, 1];
export const ROW = [0, 0, 0, 1, 1, 1, 2, 2, 2, 3];

function screenAt(col, row) {
  for (let i = 0; i < N; i++) if (COL[i] === col && ROW[i] === row) return i;
  return -1;
}

// --- Muelles (mass, stiffness, damping), ver el origen de cada uno en el .h ---
export const FOLDER = { mass: 1, stiffness: 341.51, damping: 36.96 };
export const NUDGE = { mass: 1, stiffness: 374.45, damping: 42.572 };
export const PULL = { mass: 1, stiffness: 135.39, damping: 23.271 };

export function springLeft(s, t) {
  if (t <= 0) return 1;
  const w = Math.sqrt(s.stiffness / s.mass);
  const z = s.damping / (2 * Math.sqrt(s.stiffness * s.mass));
  if (z < 0.999) {
    const wd = w * Math.sqrt(1 - z * z);
    return Math.exp(-z * w * t) * (Math.cos(wd * t) + (z * w / wd) * Math.sin(wd * t));
  }
  if (z < 1.001) return Math.exp(-w * t) * (1 + w * t);
  const r = Math.sqrt(z * z - 1);
  const r1 = -w * (z - r), r2 = -w * (z + r);
  return (r2 * Math.exp(r1 * t) - r1 * Math.exp(r2 * t)) / (r2 - r1);
}

export const spring = (s, t) => 1 - springLeft(s, t);

function settle(s, travelPx) {
  const eps = 0.5 / (travelPx > 1 ? travelPx : 1);
  let t = 0;
  while (t < 2 && Math.abs(springLeft(s, t)) > eps) t += 0.005;
  return t;
}

// --- Fundidos con curva ---
const EASE_IN = 0, EASE_OUT = 1, EASE_IN_OUT = 2;

function cubicBezier(x1, y1, x2, y2, u) {
  let lo = 0, hi = 1, s = u;
  for (let i = 0; i < 14; i++) {
    s = 0.5 * (lo + hi);
    const m = 1 - s;
    const x = 3 * m * m * s * x1 + 3 * m * s * s * x2 + s * s * s;
    if (x < u) lo = s; else hi = s;
  }
  const m = 1 - s;
  return 3 * m * m * s * y1 + 3 * m * s * s * y2 + s * s * s;
}

function fade(f, t) {
  const u = (t - f.delay) / f.duration;
  if (u <= 0) return 0;
  if (u >= 1) return 1;
  switch (f.curve) {
    case EASE_IN: return cubicBezier(0.42, 0, 1, 1, u);
    case EASE_OUT: return cubicBezier(0, 0, 0.58, 1, u);
    case EASE_IN_OUT: return cubicBezier(0.42, 0, 0.58, 1, u);
    default: return u;
  }
}

const fadeEnd = (f) => f.delay + f.duration;

const OPEN_OUTER = { delay: 0, duration: 0.4, curve: EASE_IN };
const OPEN_INNER = { delay: 0.1, duration: 0.3, curve: EASE_IN };
const CLOSE_INNER = { delay: 0, duration: 0.2, curve: EASE_OUT };
const FOLDER_OUT = { delay: 0, duration: 0.25, curve: EASE_IN };
const FOLDER_IN = { delay: 0.2, duration: 0.2, curve: EASE_OUT };
const SWAP = CLOSE_INNER;

function ring(i) {
  return Math.max(Math.abs(COL[i] - 1), Math.abs(ROW[i] - 1));
}

function hopDelay(i) {
  let d = 0, inc = 2;
  for (let r = ring(i); r > 0; r--) {
    d += inc;
    inc -= 1;
    if (inc < 1) inc = 1;
  }
  return d / 60;
}

// Mismo orden que OledAnim::Kind: el teclado manda el número tal cual.
export const KIND = {
  CUT: 0, CROSSFADE: 1, PAGE_NEXT: 2, PAGE_PREV: 3, MENU_OPEN: 4,
  MENU_CLOSE: 5, PUSH: 6, POP: 7, PROFILE: 8,
};

const FOLDER_MIN = 0.03;
const BG_ZOOM = 1.6;
const PROFILE_MIN = 0.6;

const CAROUSEL_GAP = 70;
const PITCH = W + CAROUSEL_GAP;
const CANVAS = 3 * PITCH;

const SETTLE = (() => {
  const s = new Array(9).fill(0);
  s[KIND.CROSSFADE] = fadeEnd(SWAP);
  s[KIND.PAGE_NEXT] = settle(NUDGE, CANVAS);
  s[KIND.PAGE_PREV] = s[KIND.PAGE_NEXT];
  const viaje = Math.max(1 - FOLDER_MIN, BG_ZOOM - 1) * W / 2;
  s[KIND.MENU_OPEN] = Math.max(fadeEnd(FOLDER_OUT),
    Math.max(FOLDER_IN.delay + settle(FOLDER, viaje), fadeEnd(FOLDER_IN)));
  s[KIND.MENU_CLOSE] = s[KIND.MENU_OPEN];
  s[KIND.PUSH] = settle(PULL, W);
  s[KIND.POP] = s[KIND.PUSH];
  s[KIND.PROFILE] = Math.max(fadeEnd(OPEN_OUTER),
    Math.max(settle(FOLDER, (1 - PROFILE_MIN) * W / 2), fadeEnd(OPEN_INNER)) + hopDelay(9));
  return s;
})();

// Lo que dura una transición a `rate` veces el ritmo de Apple. El teclado ya
// manda el ritmo del grupo calculado (base de fábrica x ajuste del usuario),
// así que aquí no hace falta saber nada de grupos.
export function duration(kind, rate = 1) {
  return SETTLE[kind] / rate;
}

// --- Dibujo ---
function getPx(fb, x, y) {
  return (fb[(y >> 3) * W + x] >> (y & 7)) & 1;
}
function putPx(fb, x, y, on) {
  const bit = 1 << (y & 7);
  if (on) fb[(y >> 3) * W + x] |= bit;
  else fb[(y >> 3) * W + x] &= ~bit & 0xff;
}

const BAYER = [
  [0, 8, 2, 10],
  [12, 4, 14, 6],
  [3, 11, 1, 9],
  [15, 7, 13, 5],
];

function alpha16(a) {
  if (a <= 0) return 0;
  if (a >= 1) return 16;
  return Math.trunc(a * 16 + 0.5);
}

const mx = new Int8Array(W);
const my = new Int8Array(H);

function drawLayer(out, src, scale, dx, dy, lo, hi, opaque) {
  if (lo >= hi || !src) return;
  const inv = 1 / scale;
  for (let x = 0; x < W; x++) {
    const i = Math.floor((x + 0.5 - W * 0.5 - dx) * inv + W * 0.5);
    mx[x] = i >= 0 && i < W ? i : -1;
  }
  for (let y = 0; y < H; y++) {
    const i = Math.floor((y + 0.5 - H * 0.5 - dy) * inv + H * 0.5);
    my[y] = i >= 0 && i < H ? i : -1;
  }
  const full = lo === 0 && hi >= 16;
  for (let y = 0; y < H; y++) {
    if (my[y] < 0) continue;
    const brow = BAYER[y & 3];
    for (let x = 0; x < W; x++) {
      if (mx[x] < 0) continue;
      if (!full) {
        const b = brow[x & 3];
        if (b < lo || b >= hi) continue;
      }
      const on = getPx(src, mx[x], my[y]);
      if (on || opaque) putPx(out, x, y, on);
    }
  }
}

function vline(out, x) {
  if (x < 1 || x >= W - 1) return;
  for (let p = 0; p < 5; p++) out[p * W + x] = 0xff;
}

// lroundf: redondeo a la entera más cercana, los .5 lejos de cero.
const lround = (v) => (v < 0 ? -Math.round(-v) : Math.round(v));

function drawCarousel(out, i, shift, from, to) {
  const s = lround(shift);
  for (let x = 0; x < W; x++) {
    let X = COL[i] * PITCH + x + s;
    let page = from;
    if (X >= CANVAS) { X -= CANVAS; page = to; }
    else if (X < 0) { X += CANVAS; page = to; }
    const lx = X % PITCH;
    const j = screenAt(Math.floor(X / PITCH), ROW[i]);
    if (lx >= W || j < 0) {
      for (let p = 0; p < 5; p++) out[p * W + x] = 0;
    } else {
      for (let p = 0; p < 5; p++) out[p * W + x] = page[j][p * W + lx];
    }
  }
}

function same(a, b) {
  for (let k = 0; k < FB; k++) if (a[k] !== b[k]) return false;
  return true;
}

// Un fotograma de la pantalla i en el instante t (segundos reales desde que
// empezó). `from` y `to` son arrays de 10 framebuffers.
export function render(kind, t, i, from, to, out, rate = 1) {
  if (kind === KIND.CUT || t >= duration(kind, rate)) { out.set(to[i]); return; }

  out.fill(0);
  const a = from[i];
  const b = to[i];

  if (kind !== KIND.PAGE_NEXT && kind !== KIND.PAGE_PREV && same(a, b)) {
    out.set(b);
    return;
  }

  const tv = t * rate;

  switch (kind) {
    case KIND.CROSSFADE: {
      const n = alpha16(fade(SWAP, tv));
      drawLayer(out, a, 1, 0, 0, n, 16, false);
      drawLayer(out, b, 1, 0, 0, 0, n, false);
      return;
    }
    case KIND.PAGE_NEXT:
    case KIND.PAGE_PREV: {
      const p = spring(NUDGE, tv) * CANVAS;
      drawCarousel(out, i, kind === KIND.PAGE_NEXT ? p : -p, from, to);
      return;
    }
    case KIND.MENU_OPEN:
    case KIND.MENU_CLOSE: {
      const abre = kind === KIND.MENU_OPEN;
      const saleA = abre ? BG_ZOOM : FOLDER_MIN;
      const entraDe = abre ? FOLDER_MIN : BG_ZOOM;
      const po = spring(FOLDER, tv);
      const pi = spring(FOLDER, tv - FOLDER_IN.delay);
      const no = alpha16(1 - fade(FOLDER_OUT, tv));
      const ni = alpha16(fade(FOLDER_IN, tv));
      drawLayer(out, a, 1 + (saleA - 1) * po, 0, 0, 16 - no, 16, false);
      drawLayer(out, b, entraDe + (1 - entraDe) * pi, 0, 0, 0, ni, false);
      return;
    }
    case KIND.PUSH: {
      const p = spring(PULL, tv);
      const xn = W * (1 - p);
      drawLayer(out, a, 1, -W * 0.3 * p, 0, 0, alpha16(1 - 0.5 * p), false);
      drawLayer(out, b, 1, xn, 0, 0, 16, true);
      vline(out, lround(xn) - 1);
      return;
    }
    case KIND.POP: {
      const p = spring(PULL, tv);
      const xo = W * p;
      drawLayer(out, b, 1, -W * 0.3 * (1 - p), 0, 0, alpha16(0.5 + 0.5 * p), false);
      drawLayer(out, a, 1, xo, 0, 0, 16, true);
      vline(out, lround(xo) - 1);
      return;
    }
    case KIND.PROFILE: {
      const tl = tv - hopDelay(i);
      const p = spring(FOLDER, tl);
      const fo = fade(OPEN_OUTER, tv);
      const no = alpha16(1 - fo);
      const ni = alpha16(fade(OPEN_INNER, tl));
      drawLayer(out, a, 1 - 0.1 * fo, 0, 0, 16 - no, 16, false);
      if (tl > 0) drawLayer(out, b, PROFILE_MIN + (1 - PROFILE_MIN) * p, 0, 0, 0, ni, false);
      return;
    }
    default:
      out.set(b);
  }
}

// --- Pulsación: se hunde un 12 % con el muelle de la carpeta ---
const PRESS_DEPTH = 0.12;

export const newPress = () => ({ x: 0, v: 0 });

export function pressStep(p, target, dt) {
  const s = FOLDER;
  while (dt > 0) {
    const h = dt > 0.002 ? 0.002 : dt;
    const acc = (-s.stiffness * (p.x - target) - s.damping * p.v) / s.mass;
    p.v += acc * h;
    p.x += p.v * h;
    dt -= h;
  }
  if (Math.abs(p.x - target) < 0.002 && Math.abs(p.v) < 0.02) {
    p.x = target; p.v = 0;
    return false;
  }
  return true;
}

export function renderPress(src, p, out) {
  if (p.x <= 0) { out.set(src); return; }
  out.fill(0);
  drawLayer(out, src, 1 - PRESS_DEPTH * p.x, 0, 0, 0, 16, false);
}
