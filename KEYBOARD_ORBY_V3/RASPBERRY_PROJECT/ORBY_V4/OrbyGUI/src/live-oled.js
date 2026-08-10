// Visor en vivo en la pantalla de una tecla: cuando una tecla está asignada a
// un "visor" de un complemento (ver docs/PLUGINS.md, brightness_view/color_view
// de lampdesk), esta pantalla enseña el valor real —brillo, color...— en vez de
// quedarse con el icono fijo que se le haya dibujado.
//
// Es una superposición en RAM del teclado, igual que las variaciones
// (variants.js): nunca toca `oledMask` ni la caché del icono guardado, así que
// "Guardar en Flash" no se entera y el icono de verdad sigue ahí debajo. Solo se
// empuja mientras esa tecla está en la página que el teclado tiene puesta ahora
// mismo (no se fuerza ningún cambio de página para pintar en segundo plano), y
// solo mientras OrbyGUI esté abierto: al cerrarla, la pantalla se queda con el
// último valor hasta que el propio teclado la repinte (cambio de página,
// reconexión...), igual que cualquier otra acción de complemento.

import { state, subscribe, KEY_TO_SCREEN, labelSlot, keymapSlot } from './store.js';
import { MACRO_MODIFIER } from './hid-keys.js';
import * as device from './device.js';
import * as plugins from './plugins.js';
import * as cache from './oled-cache.js';
import * as fb from './oled-fb.js';
import { icon as uiIcon } from './icons.js';
import { macroById, loadPCMacros } from './views/profiles/macros-store.js';
import * as platform from './platform.js';

const POLL_MS = 2000;

// Icono → { fuente lista para rasterizar }, en caché: es siempre el mismo
// trazo, y makeSvgSource() carga una imagen (asíncrono) que no tiene sentido
// repetir en cada sondeo.
const sourceCache = new Map();
function iconSource(name) {
  let p = sourceCache.get(name);
  if (!p) {
    // Blanco explícito: el rasterizador mide luminancia, y el trazo por
    // defecto (currentColor) no enciende ningún píxel sobre fondo transparente.
    const markup = uiIcon(name, 96).replace('currentColor', '#fff');
    p = fb.makeSvgSource(markup, { size: 96 });
    sourceCache.set(name, p);
  }
  return p;
}

// Coloca un icono ya cargado con su alto de tinta en `targetH` píxeles, con la
// esquina superior izquierda de lo dibujado en (targetX, targetY).
function iconLayer(source, targetX, targetY, targetH) {
  const ink = fb.sourceInkBox(source);
  const scale = targetH / Math.max(1, ink.height);
  return {
    x: targetX - ink.x * scale, y: targetY - ink.y * scale, scale,
    threshold: 128, blur: 0, dither: false, invert: 'none',
  };
}

async function stampIcon(base, name, x, y, h) {
  try {
    const source = await iconSource(name);
    const raster = fb.rasterizeLayer(source, iconLayer(source, x, y, h));
    return fb.compose(base, raster, 'merge');
  } catch (err) {
    // Un icono que no carga no debe tirar el frame entero: la barra sigue
    // diciendo el valor aunque el dibujo salga sin uno de los dos iconos.
    console.error(`[live-oled] icono "${name}":`, err);
    return base;
  }
}

const BAR_X0 = 4, BAR_X1 = fb.OLED_W - 4; // 4..68
const BAR_Y0 = 26, BAR_Y1 = 34;           // 8 px de alto

function drawBar(base, value) {
  for (let x = BAR_X0; x <= BAR_X1; x++) { fb.setPixel(base, x, BAR_Y0, 1); fb.setPixel(base, x, BAR_Y1, 1); }
  for (let y = BAR_Y0; y <= BAR_Y1; y++) { fb.setPixel(base, BAR_X0, y, 1); fb.setPixel(base, BAR_X1, y, 1); }

  const pct = Math.max(0, Math.min(100, Number(value) || 0)) / 100;
  const inner0 = BAR_X0 + 2, inner1 = BAR_X1 - 2;
  const filled = Math.round(inner0 + (inner1 - inner0) * pct);
  for (let x = inner0; x <= filled; x++) {
    for (let y = BAR_Y0 + 2; y <= BAR_Y1 - 2; y++) fb.setPixel(base, x, y, 1);
  }
}

// El icono del complemento (izquierda) + el del visor concreto (derecha) +
// la barra debajo con el valor. Un frame por sondeo: no se guarda nada entre
// medias porque el único dato que cambia es el número.
async function renderFrame(pluginIcon, viewIcon, value) {
  let base = fb.createFramebuffer();
  base = await stampIcon(base, pluginIcon, 4, 3, 15);
  base = await stampIcon(base, viewIcon, fb.OLED_W - 19, 3, 15);
  drawBar(base, value);
  return base;
}

// --- Qué teclas tienen que enseñar un visor ahora mismo ---------------------
//
// Solo la página que el teclado tiene puesta de verdad (state.activeProfileIdx
// + state.pageIdx), no la que se esté editando en otra pestaña: empujar a una
// página que no se ve no sirve de nada y gasta CDC.
function currentTargets() {
  if (!state.connected) return [];
  const profileIdx = state.activeProfileIdx;
  const prof = state.profiles[profileIdx];
  const page = prof?.pages?.[state.pageIdx];
  if (!page) return [];

  const layer = state.superActive ? 'super' : 'normal';
  const out = [];

  for (let keyIndex = 0; keyIndex < KEY_TO_SCREEN.length; keyIndex++) {
    if (!KEY_TO_SCREEN[keyIndex]) continue; // esa tecla no tiene pantalla propia

    const action = page.keys[keymapSlot(keyIndex, layer)];
    if (!action || action.modifier !== MACRO_MODIFIER) continue;

    const m = macroById(action.keycode);
    const step = m?.actions?.length === 1 && m.actions[0].type === 'plugin' ? m.actions[0] : null;
    if (!step) continue;

    const plugin = plugins.byId(step.plugin);
    if (!plugin?.enabled || !plugin.hasRead) continue;
    const view = plugins.viewsFor(plugin).find((v) => v.op === step.op);
    if (!view) continue;

    out.push({
      key: `${profileIdx}:${state.pageIdx}:${labelSlot(keyIndex, layer)}`,
      profileIdx, pageIdx: state.pageIdx, oledSlot: labelSlot(keyIndex, layer),
      pluginId: plugin.id, pluginIcon: plugin.icon, op: step.op, viewIcon: view.icon,
    });
  }
  return out;
}

// --- Sondeo y empuje ---------------------------------------------------------

let timer = null;
const active = new Map(); // key -> target (lo que se está enseñando ahora mismo)

async function restore(target) {
  // Si el teclado ya cambió de página o de perfil por su cuenta, ya repintó
  // esa tecla desde su propia Flash: escribir aquí iría a la página nueva con
  // la dirección de la vieja. Solo hace falta restaurar cuando el motivo de
  // dejar de enseñar el visor es otro (complemento desactivado, tecla
  // reasignada...) y la página sigue siendo la misma.
  if (target.profileIdx !== state.activeProfileIdx || target.pageIdx !== state.pageIdx) return;
  try {
    const bytes = cache.get(target.profileIdx, target.oledSlot, target.pageIdx);
    if (bytes) await device.uploadOled(target.profileIdx, target.oledSlot, bytes);
    else await device.clearOled(target.profileIdx, target.oledSlot);
  } catch { /* sin conexión: no hay nada que restaurar */ }
}

async function pushOne(target) {
  const res = await plugins.readView(target.pluginId, target.op);
  if (!res.ok) { console.warn(`[live-oled] "${target.op}" de "${target.pluginId}" no respondió`); return; }
  try {
    const frame = await renderFrame(target.pluginIcon, target.viewIcon, res.value);
    await device.uploadOled(target.profileIdx, target.oledSlot, frame);
  } catch (err) {
    console.error(`[live-oled] no se pudo subir la tecla ${target.oledSlot}:`, err);
  }
}

// Solo se avisa por consola cuando la lista cambia, para poder ver en
// DevTools si el problema es "no encuentro ninguna tecla visora" (nada que
// sondear) o "encuentro la tecla pero algo falla al sondear/subir" (ver
// pushOne). Con esto callado en cada sondeo sería ruido inútil.
let lastKeysSeen = '';

async function tick() {
  const targets = currentTargets();
  const seen = new Set(targets.map((t) => t.key));

  const keysNow = targets.map((t) => t.key).join(',');
  if (keysNow !== lastKeysSeen) {
    lastKeysSeen = keysNow;
    console.debug('[live-oled] teclas visoras activas:', targets);
  }

  for (const [key, prev] of active) {
    if (!seen.has(key)) { active.delete(key); restore(prev); }
  }
  for (const t of targets) active.set(t.key, t);

  await Promise.all(targets.map(pushOne));
}

function schedule() {
  if (timer) return;
  timer = setInterval(tick, POLL_MS);
  tick();
}

function unschedule() {
  clearInterval(timer);
  timer = null;
}

export function init() {
  // Los complementos son de escritorio (ver PC_ONLY en platform.js): en la
  // WebGUI la lista siempre está vacía y este sondeo no tendría nada que hacer.
  if (!platform.can('plugins')) return;

  loadPCMacros();
  subscribe(() => (state.connected ? schedule() : (unschedule(), active.clear())));
  plugins.onChange(() => { if (state.connected) tick(); });
  device.on('connected', () => { loadPCMacros(); schedule(); });
  device.on('disconnected', () => { unschedule(); active.clear(); });
}
