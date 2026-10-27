// Las diez pantallas del teclado, en vivo, en el dashboard.
//
// No es una imitación: es el mismo motor. Con un firmware 4.12 o posterior la
// app pide MIRROR:1 y el teclado le cuenta, en cada transición, qué va a
// enseñar cada pantalla al acabar (EV:SCR:<n>:<hex>) y qué transición toca
// (EV:SCR:GO:<tipo>:<ritmo>). A partir de ahí la app hace exactamente lo que
// hace Core 1 en el teclado (core1_entry, main.cpp): parte de lo que tenía en
// pantalla en ese instante, corre oled-anim.mjs —la copia en JS de
// oled_anim.h, comprobada bit a bit— y repinta a 60 fotogramas por segundo.
// Hasta la pulsación con muelle y el negativo de la tecla sujeta (EV:INV).
//
// Sin espejo (firmware anterior, o la copia del PC sin teclado) no hay forma
// de saber qué dibuja el teclado en sus menús. Entonces el dashboard monta los
// destinos con lo que sí sabe la app —los iconos subidos— y los anima con las
// mismas reglas que classify() del firmware al cambiar de perfil, de página o
// de capa. Ver fallback() más abajo.

import * as A from './oled-anim.mjs';
import * as device from './device.js';
import * as compat from './compat.js';

const blank = () => Array.from({ length: A.N }, () => new Uint8Array(A.FB));

const shown = blank();      // lo que hay ahora en cada pantalla
const from = blank();       // foto al empezar la transición en curso
const target = blank();     // a donde va
const incoming = blank();   // destinos que van llegando antes del GO
const frame = new Uint8Array(A.FB);
const pressed = new Uint8Array(A.FB);
const press = Array.from({ length: A.N }, A.newPress);

let kind = A.KIND.CUT;
let rate = 1;
let t0 = 0;
let animating = false;
let inversions = 0;
let displaysOn = true;
let live = false;           // el teclado está mandando sus pantallas
let raf = 0;
let last = 0;

let canvases = [];
const listeners = new Set();

export const isLive = () => live;

export function onLiveChange(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

function setLive(v) {
  if (live === v) return;
  live = v;
  for (const fn of listeners) fn(live);
}

// --- Pintar -------------------------------------------------------------------
// Cada canvas mide lo mismo que su pantalla en píxeles reales del monitor, y
// cada píxel del OLED se pinta como un rectángulo ajustado a esa rejilla.
//
// Antes era un canvas de 72x40 ampliado por CSS con image-rendering:
// pixelated. Con la ventana estrecha la pantalla de la réplica mide menos de
// 72 píxeles de ancho, y el escalado sin suavizar se come columnas enteras: el
// trazo de un píxel de la P de «PLAY» desaparecía según el tamaño. Aquí cada
// píxel encendido ocupa al menos uno de pantalla, así que ninguno se pierde.
let onRGB = [214, 246, 255];

export function setPixelColor(hex) {
  const m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(hex.trim());
  if (m) onRGB = [parseInt(m[1], 16), parseInt(m[2], 16), parseInt(m[3], 16)];
  for (let i = 0; i < A.N; i++) paint(i);
}

function paint(i) {
  const cv = canvases[i];
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const sx = cv.width / A.W, sy = cv.height / A.H;
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = `rgb(${onRGB[0]},${onRGB[1]},${onRGB[2]})`;
  const src = shown[i];
  // El negativo de la tecla sujeta va por hardware en el SSD1306: invierte el
  // panel entero, encima de lo que esté dibujando la animación.
  const inv = displaysOn && ((inversions >> i) & 1) === 1;
  const edge = (v, s) => Math.round(v * s);
  for (let y = 0; y < A.H; y++) {
    const row = (y >> 3) * A.W, bit = 1 << (y & 7);
    const y0 = edge(y, sy), h = Math.max(1, edge(y + 1, sy) - y0);
    // Por tramos seguidos: un rectángulo por racha de píxeles encendidos, no
    // uno por píxel. Apagado es transparente: debajo está el negro del
    // cristal, y así el brillo de los encendidos no tiñe el fondo.
    let x = 0;
    while (x < A.W) {
      const lit = (k) => displaysOn && (((src[row + k] & bit) !== 0) !== inv);
      if (!lit(x)) { x++; continue; }
      let x1 = x + 1;
      while (x1 < A.W && lit(x1)) x1++;
      const x0 = edge(x, sx);
      ctx.fillRect(x0, y0, Math.max(1, edge(x1, sx) - x0), h);
      x = x1;
    }
  }
}

// El tamaño del lienzo sigue al de la pantalla en la réplica, que escala con la
// ventana.
const sizer = typeof ResizeObserver === 'function' ? new ResizeObserver((entries) => {
  for (const e of entries) {
    const cv = e.target;
    const dpr = window.devicePixelRatio || 1;
    const box = e.devicePixelContentBoxSize?.[0];
    const w = box ? box.inlineSize : Math.round(e.contentRect.width * dpr);
    const h = box ? box.blockSize : Math.round(e.contentRect.height * dpr);
    if (w > 0 && h > 0 && (cv.width !== w || cv.height !== h)) {
      cv.width = w;
      cv.height = h;
      paint(canvases.indexOf(cv));
    }
  }
}) : null;

export function attach(list) {
  sizer?.disconnect();
  canvases = list;
  for (const cv of canvases) {
    if (!cv) continue;
    cv.width = A.W;
    cv.height = A.H;
    sizer?.observe(cv);
  }
  for (let i = 0; i < A.N; i++) paint(i);
}

// --- Bucle --------------------------------------------------------------------
function tick(now) {
  raf = 0;
  // Mismo tope que el teclado: tras una pestaña en segundo plano, el muelle
  // de la pulsación no debe dar un salto de golpe.
  let dt = (now - last) / 1000;
  if (dt > 0.05) dt = 0.05;
  last = now;

  let muelles = false;
  for (let i = 0; i < A.N; i++) {
    if (A.pressStep(press[i], (inversions >> i) & 1 ? 1 : 0, dt)) muelles = true;
  }

  const t = animating ? (now - t0) / 1000 : 0;
  for (let i = 0; i < A.N; i++) {
    if (animating) A.render(kind, t, i, from, target, frame, rate);
    else frame.set(target[i]);
    let f = frame;
    if (press[i].x > 0) { A.renderPress(frame, press[i], pressed); f = pressed; }
    if (!sameFb(f, shown[i])) { shown[i].set(f); paint(i); }
  }
  if (animating && t >= A.duration(kind, rate)) animating = false;

  if (animating || muelles) raf = requestAnimationFrame(tick);
}

function kick() {
  if (raf) return;
  last = performance.now();
  raf = requestAnimationFrame(tick);
}

function sameFb(a, b) {
  for (let k = 0; k < A.FB; k++) if (a[k] !== b[k]) return false;
  return true;
}

// Arranca una transición hacia `dest` desde lo que haya en pantalla ahora: si
// llega otra a medias, la nueva sale de ese fotograma, igual que en el teclado.
function start(k, r, dest) {
  for (let i = 0; i < A.N; i++) { from[i].set(shown[i]); target[i].set(dest[i]); }
  kind = k;
  rate = r > 0 ? r : 1;
  t0 = performance.now();
  animating = k !== A.KIND.CUT;
  if (!animating) {
    for (let i = 0; i < A.N; i++) { shown[i].set(target[i]); paint(i); }
  }
  kick();
}

// --- Lo que manda el teclado --------------------------------------------------
function hexInto(hex, out) {
  const n = Math.min(out.length, hex.length >> 1);
  for (let b = 0; b < n; b++) out[b] = parseInt(hex.substr(b * 2, 2), 16);
}

function onTelemetry(line) {
  if (line.startsWith('EV:SCR:GO:')) {
    const [k, r] = line.slice(10).split(':').map(Number);
    setLive(true);
    start(Number.isFinite(k) ? k : A.KIND.CROSSFADE, (r || 1000) / 1000, incoming);
    return;
  }
  if (line.startsWith('EV:SCR:')) {
    const rest = line.slice(7);
    const sep = rest.indexOf(':');
    const i = parseInt(rest.slice(0, sep), 10);
    if (i >= 0 && i < A.N) hexInto(rest.slice(sep + 1), incoming[i]);
    return;
  }
  if (line.startsWith('EV:INV:')) {
    inversions = parseInt(line.slice(7), 10) || 0;
    for (let i = 0; i < A.N; i++) paint(i);
    kick();
    return;
  }
  if (line.startsWith('EV:DISP:')) {
    displaysOn = line.slice(8) !== '0';
    for (let i = 0; i < A.N; i++) paint(i);
  }
}

// --- Sin espejo -----------------------------------------------------------------
// Lo que la app sabe dibujar por su cuenta: los iconos subidos, tal cual. Las
// etiquetas de texto y los iconos del catálogo los pinta el firmware con su
// propia fuente, que la app no tiene; esas teclas quedan a negro en el lienzo y
// el dashboard les pone el nombre encima (ver views/dashboard.js).
let fbCtx = null;

const sameCtx = (a, b) => a && b && a.profile === b.profile && a.page === b.page
  && a.pages === b.pages && a.super === b.super;

// Mismas reglas que classify() en main.cpp para la parte que la app ve.
function classify(a, b) {
  if (!a) return A.KIND.CUT;
  if (a.profile !== b.profile) return A.KIND.PROFILE;
  if (a.page !== b.page) {
    if (b.pages > 1 && b.page === (a.page + 1) % b.pages) return A.KIND.PAGE_NEXT;
    if (b.pages > 1 && a.page === (b.page + 1) % b.pages) return A.KIND.PAGE_PREV;
    return b.page > a.page ? A.KIND.PAGE_NEXT : A.KIND.PAGE_PREV;
  }
  if (a.super !== b.super) return b.super ? A.KIND.MENU_OPEN : A.KIND.MENU_CLOSE;
  return A.KIND.CROSSFADE;
}

// `ctx` es {profile, page, pages, super}; `screens` diez framebuffers (o null
// para negro). `rates` el ritmo por grupo que tenga puesto el teclado.
export function fallback(ctx, screens, rates = {}) {
  if (live) return;
  const dest = blank();
  screens.forEach((s, i) => { if (s) dest[i].set(s); });
  const k = classify(fbCtx, ctx);
  const changed = !fbCtx || !sameCtx(fbCtx, ctx) || dest.some((d, i) => !sameFb(d, target[i]));
  fbCtx = { ...ctx };
  if (!changed) return;
  const group = { [A.KIND.PROFILE]: 'profile', [A.KIND.PAGE_NEXT]: 'carousel', [A.KIND.PAGE_PREV]: 'carousel',
    [A.KIND.MENU_OPEN]: 'menu', [A.KIND.MENU_CLOSE]: 'menu' }[k] || 'fade';
  start(k, rates[group] || 1, dest);
}

// Para la pulsación sin espejo: el dashboard sabe qué tecla se pulsa por KEY_EV.
export function setPressed(screenIdx, down) {
  if (live || screenIdx < 0) return;
  if (down) inversions |= 1 << screenIdx;
  else inversions &= ~(1 << screenIdx);
  paint(screenIdx);
  kick();
}

export function init() {
  device.on('telemetry', onTelemetry);
  device.on('connected', (info) => {
    if (compat.supports(info, 'mirror')) device.setMirror(true).catch(() => {});
  });
  device.on('disconnected', () => {
    setLive(false);
    fbCtx = null;
    inversions = 0;
    displaysOn = true;
  });
}
