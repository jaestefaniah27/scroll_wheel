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
// Un canvas de 72x40 por pantalla, ampliado por CSS con image-rendering:
// pixelated. Es lo más barato que hay: un putImageData por pantalla que
// cambia, y el navegador escala sin suavizar.
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
  const img = ctx.createImageData(A.W, A.H);
  const d = img.data;
  const src = shown[i];
  // El negativo de la tecla sujeta va por hardware en el SSD1306: invierte el
  // panel entero, encima de lo que esté dibujando la animación.
  const inv = displaysOn && ((inversions >> i) & 1) === 1;
  for (let y = 0; y < A.H; y++) {
    const row = (y >> 3) * A.W, bit = 1 << (y & 7);
    for (let x = 0; x < A.W; x++) {
      let on = displaysOn && (src[row + x] & bit) !== 0;
      if (inv) on = !on;
      const k = (y * A.W + x) * 4;
      if (on) { d[k] = onRGB[0]; d[k + 1] = onRGB[1]; d[k + 2] = onRGB[2]; d[k + 3] = 255; }
      // Apagado es transparente: debajo está el negro del cristal, y así el
      // brillo de los píxeles encendidos no tiñe el fondo.
    }
  }
  ctx.putImageData(img, 0, 0);
}

export function attach(list) {
  canvases = list;
  for (const cv of canvases) { if (cv) { cv.width = A.W; cv.height = A.H; } }
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
