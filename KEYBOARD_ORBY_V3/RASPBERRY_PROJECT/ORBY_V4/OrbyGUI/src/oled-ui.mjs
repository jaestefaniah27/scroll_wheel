// Copia en JS de include/oled_ui.h: lo que dibuja el teclado en una pantalla
// cuando no tiene un icono subido.
//
// Los perfiles de la biblioteca no suben ni un bitmap: el firmware busca cada
// etiqueta (COPY, WIDTH, RTMODE…) en su catálogo y dibuja el icono con su
// nombre debajo. Sin esto la app no sabía pintarlos, y donde no había espejo
// del teclado (la capa SUPER mientras se edita, un firmware sin MIRROR, la
// copia del PC) salía la etiqueta en texto en vez del icono.
//
// Los datos (fuente, dibujos y catálogo) los genera tools/oled_ui/build.py en
// oled-ui-data.mjs, de los mismos ficheros que oled_ui_data.h. test/oled-ui.test.mjs
// comprueba con los casos de referencia de test_ui.cpp que esto pone cada
// píxel donde el firmware.

import * as D from './oled-ui-data.mjs';

export const W = 72, H = 40, FB = 360;
const ICON_X = (W - 24) / 2;
const ICON_Y = 2;
const CAP_Y = 28;

function setPx(fb, x, y) {
  if (x < 0 || x >= W || y < 0 || y >= H) return;
  fb[(y >> 3) * W + x] |= 1 << (y & 7);
}

function putCol(fb, x, y, bits, h) {
  for (let r = 0; r < h; r++) if ((bits >>> r) & 1) setPx(fb, x, y + r);
}

function drawSprite(fb, id, x, y) {
  const s = D.SPRITES[id];
  if (!s) return;
  const [w, h, off] = s;
  for (let c = 0; c < w; c++) putCol(fb, x + c, y, D.SPRITE_COLS[off + c], h);
}

// --- Texto -------------------------------------------------------------------
// El firmware guarda las etiquetas en bytes (8 como mucho, ver copy_name en
// main.cpp) y las lee como UTF-8; un byte que no forme una secuencia válida
// se toma tal cual. Aquí se hace lo mismo sobre los bytes, para que una ñ
// cortada a la mitad por el límite salga igual que en el teclado.
function codepoints(bytes) {
  const out = [];
  let i = 0;
  while (i < bytes.length && bytes[i]) {
    const c = bytes[i];
    if (c < 0x80) { out.push(c); i++; continue; }
    if ((c & 0xe0) === 0xc0 && (bytes[i + 1] & 0xc0) === 0x80) {
      out.push(((c & 0x1f) << 6) | (bytes[i + 1] & 0x3f));
      i += 2; continue;
    }
    if ((c & 0xf0) === 0xe0 && (bytes[i + 1] & 0xc0) === 0x80 && (bytes[i + 2] & 0xc0) === 0x80) {
      out.push(((c & 0x0f) << 12) | ((bytes[i + 1] & 0x3f) << 6) | (bytes[i + 2] & 0x3f));
      i += 3; continue;
    }
    out.push(c); i++;
  }
  return out;
}

const enc = new TextEncoder();
const bytesOf = (s, max = Infinity) => enc.encode(String(s ?? '')).slice(0, max);

function findGlyph(cp) {
  for (let pass = 0; pass < 2; pass++) {
    let lo = 0, hi = D.FONT_SMALL.length - 1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      const g = D.FONT_SMALL[mid][0];
      if (g === cp) return mid;
      if (g < cp) lo = mid + 1; else hi = mid - 1;
    }
    const f = D.FOLD.find(([a]) => a === cp);
    if (!f) break;
    cp = f[1];
  }
  return findGlyph(0x3f);   // '?'
}

// `gap` es el aire entre letras: 1 en pequeño, 2 en grande y 1 en la grande
// apretada (ver screenName).
function textWidth(cps, big, gap = big ? 2 : 1) {
  if (!cps.length) return 0;
  let w = 0;
  for (const cp of cps) w += (big ? D.FONT_BIG : D.FONT_SMALL)[findGlyph(cp)][1];
  return w + (cps.length - 1) * gap;
}

function drawText(fb, cps, cx, top, big, gap = big ? 2 : 1) {
  let x = cx - Math.trunc(textWidth(cps, big, gap) / 2);
  for (const cp of cps) {
    const [, gw, off] = (big ? D.FONT_BIG : D.FONT_SMALL)[findGlyph(cp)];
    for (let c = 0; c < gw; c++) {
      if (big) putCol(fb, x + c, top, D.BIG_COLS[off + c], 18);
      else putCol(fb, x + c, top, D.FONT_COLS[off + c], 9);
    }
    x += gw + gap;
  }
}

// --- Pantallas tipo -------------------------------------------------------------
export function screenIcon(icon, caption, fb = new Uint8Array(FB)) {
  fb.fill(0);
  drawSprite(fb, icon, ICON_X, ICON_Y);
  const cps = codepoints(bytesOf(caption));
  if (cps.length) drawText(fb, cps, W / 2, CAP_Y, false);
  return fb;
}

// Solo texto: lo más grande que quepa, con las reglas de screenName.
export function screenText(s, fb = new Uint8Array(FB)) {
  return screenName(s, fb);
}

export function screenToggle(on, caption, fb = new Uint8Array(FB)) {
  fb.fill(0);
  drawSprite(fb, on ? D.SPR.TOGGLE_ON : D.SPR.TOGGLE_OFF, (W - 34) >> 1, 5);
  drawText(fb, codepoints(bytesOf(caption)), W / 2, CAP_Y, false);
  return fb;
}

const DOT = 7, DOT_STEP = 12, DOT_Y = 10;

export function screenDots(cur, count, caption, fb = new Uint8Array(FB)) {
  fb.fill(0);
  const x = (W - (DOT + (count - 1) * DOT_STEP)) >> 1;
  for (let i = 0; i < count; i++) drawSprite(fb, i === cur ? D.SPR.DOT_ON : D.SPR.DOT_OFF, x + i * DOT_STEP, DOT_Y);
  drawText(fb, codepoints(bytesOf(caption)), W / 2, CAP_Y, false);
  return fb;
}

const PILL_CORNER = [3, 1, 1];

export function invertPill(fb) {
  for (let i = 0; i < FB; i++) fb[i] = ~fb[i] & 0xff;
  const clr = (x, y) => { fb[(y >> 3) * W + x] &= ~(1 << (y & 7)) & 0xff; };
  PILL_CORNER.forEach((n, k) => {
    for (let i = 0; i < n; i++) {
      clr(i, k); clr(W - 1 - i, k);
      clr(i, H - 1 - k); clr(W - 1 - i, H - 1 - k);
    }
  });
  return fb;
}

// --- Teclas: icono grande y modo ayuda -------------------------------------------
// Firmware 4.28: el icono de una tecla sale a x1.5 y sin nombre debajo; el
// nombre lo enseña el modo ayuda (screenName). Ver screen_icon_big en oled_ui.h.
const BIG_X = (W - D.BIG_W) >> 1;
const BIG_Y = (H - D.BIG_H) >> 1;
const BIG_COUNT = D.BIG_ICON_COLS.length / (D.BIG_W * 5);

export function screenIconBig(icon, fb = new Uint8Array(FB)) {
  fb.fill(0);
  if (icon >= BIG_COUNT) return fb;
  let p = icon * D.BIG_W * 5;
  for (let c = 0; c < D.BIG_W; c++, p += 5) {
    for (let r = 0; r < D.BIG_H; r++) {
      if ((D.BIG_ICON_COLS[p + (r >> 3)] >> (r & 7)) & 1) setPx(fb, BIG_X + c, BIG_Y + r);
    }
  }
  return fb;
}

// Lo más grande que quepa, como screen_name en oled_ui.h: una línea grande,
// dos por espacio, dos con guion (cada paso, normal y luego apretada: 1 px
// entre letras y sin margen) y, si nada cabe, pequeña.
const BIG_FITS = [[2, W - 6], [1, W]];   // [aire entre letras, ancho disponible]
const SMALL_AVAIL = W - 6;
const NAME_BIG_TOP = [4, 22];
const NAME_SMALL_TOP = [10, 22];
const VOWELS = new Set([...'aeiouAEIOUáéíóúüÁÉÍÓÚÜ'].map((c) => c.codePointAt(0)));
const isLetter = (c) => (c >= 0x61 && c <= 0x7a) || (c >= 0x41 && c <= 0x5a)
  || (c >= 0xc0 && c <= 0x17f && c !== 0xd7 && c !== 0xf7);
const isConsonant = (c) => isLetter(c) && !VOWELS.has(c);
const lower = (c) => (c >= 0x41 && c <= 0x5a ? c + 32 : c);

// Parejas que no se separan: «com-pro-bar», no «compr-obar».
function glued(a, b) {
  a = String.fromCharCode(lower(a)); b = String.fromCharCode(lower(b));
  if (a + b === 'll' || a + b === 'rr' || a + b === 'ch') return true;
  return 'bcdfgkpt'.includes(a) && a.length === 1 && (b === 'l' || b === 'r');
}

function hyphenAt(c, i) {
  const n = c.length;
  if (i < 2 || i > n - 2) return false;
  if (!isLetter(c[i - 2]) || !isLetter(c[i - 1]) || !isLetter(c[i]) || !isLetter(c[i + 1])) return false;
  const cv = isConsonant(c[i]) && VOWELS.has(c[i + 1]) && !glued(c[i - 1], c[i]);
  const ccv = i + 2 < n && glued(c[i], c[i + 1]) && VOWELS.has(c[i + 2]);
  return cv || ccv;
}

// La partición (por espacio o con guion) que cabe con la línea más ancha más
// estrecha; a igualdad, la primera.
function split(c, big, gap, avail, hyphen) {
  let best = null;
  for (let i = 1; i < c.length; i++) {
    let a, b;
    if (!hyphen) {
      if (c[i] !== 0x20 || i + 1 >= c.length) continue;
      a = c.slice(0, i); b = c.slice(i + 1);
    } else {
      if (!hyphenAt(c, i)) continue;
      a = [...c.slice(0, i), 0x2d]; b = c.slice(i);
    }
    const wa = textWidth(a, big, gap), wb = textWidth(b, big, gap);
    if (wa > avail || wb > avail) continue;
    const m = Math.max(wa, wb);
    if (!best || m < best.m) best = { m, a, b };
  }
  return best;
}

export function screenName(s, fb = new Uint8Array(FB)) {
  fb.fill(0);
  const c = codepoints(bytesOf(s)).slice(0, 40);
  if (!c.length) return fb;
  const two = (sp, big, gap) => {
    const tops = big ? NAME_BIG_TOP : NAME_SMALL_TOP;
    drawText(fb, sp.a, W / 2, tops[0], big, gap);
    drawText(fb, sp.b, W / 2, tops[1], big, gap);
    return fb;
  };
  for (const [gap, avail] of BIG_FITS) {
    if (textWidth(c, true, gap) <= avail) { drawText(fb, c, W / 2, (H - 14) >> 1, true, gap); return fb; }
  }
  for (const hyphen of [false, true]) {
    for (const [gap, avail] of BIG_FITS) {
      const sp = split(c, true, gap, avail, hyphen);
      if (sp) return two(sp, true, gap);
    }
  }
  if (textWidth(c, false) <= SMALL_AVAIL) { drawText(fb, c, W / 2, (H - 7) >> 1, false); return fb; }
  for (const hyphen of [false, true]) {
    const sp = split(c, false, 1, SMALL_AVAIL, hyphen);
    if (sp) return two(sp, false, 1);
  }
  drawText(fb, c, W / 2, (H - 7) >> 1, false);
  return fb;
}

// --- Catálogo -----------------------------------------------------------------
// Sin distinguir mayúsculas, y solo las ASCII: same_label() del firmware no
// sube de caja una ñ ni una tilde.
// Se compara byte a byte, como el firmware: cada byte UTF-8 pasa a un carácter
// (latin1) a los dos lados, las claves del catálogo y lo que llega del perfil.
const upper = (s) => s.replace(/[a-z]/g, (c) => c.toUpperCase());
const raw = (bytes) => String.fromCharCode(...bytes);
const index = new Map();
D.ENTRIES.forEach(([key], i) => { const k = upper(raw(bytesOf(key))); if (!index.has(k)) index.set(k, i); });

const name8 = (s) => raw(bytesOf(s, 8));   // los 8 bytes de copy_name, tal cual

export function lookup(profile, label) {
  if (!label) return -1;
  const p = bytesOf(profile, 8), l = bytesOf(label, 8);
  // key[20] en el firmware: perfil + '/' + etiqueta tiene que caber con su '\0'.
  if (p.length && p.length + 1 + l.length < 20) {
    const e = index.get(upper(name8(profile) + '/' + name8(label)));
    if (e !== undefined) return e;
  }
  const e = index.get(upper(name8(label)));
  return e === undefined ? -1 : e;
}

export function lookupProfile(name) {
  if (!name) return -1;
  const e = index.get(upper('@' + name8(name).slice(0, 10)));
  return e === undefined ? -1 : e;
}

// Pantalla de una tecla sin icono subido: con el del catálogo si su etiqueta
// está en él, con el texto tal cual si no, y a negro si no tiene etiqueta.
// `big: false` es como lo dibujaba un firmware anterior a la 4.28: icono
// pequeño con el nombre debajo.
export function screenKey(profile, label, { big = true } = {}, fb = new Uint8Array(FB)) {
  const e = lookup(profile, label);
  if (e >= 0) return big ? screenIconBig(D.ENTRIES[e][1], fb) : screenIcon(D.ENTRIES[e][1], D.ENTRIES[e][2], fb);
  const l = bytesOf(label, 8);
  if (l.length) return screenText(new TextDecoder().decode(l), fb);
  fb.fill(0);
  return fb;
}

// Pantalla de un perfil en el menú de perfiles: el icono de fábrica si su nombre
// está en el catálogo, y si no el nombre en grande (render_profile_tile).
export function screenProfile(name, fb = new Uint8Array(FB)) {
  const e = lookupProfile(name);
  if (e >= 0) return screenIcon(D.ENTRIES[e][1], D.ENTRIES[e][2], fb);
  return screenText(new TextDecoder().decode(bytesOf(name, 8)), fb);
}
