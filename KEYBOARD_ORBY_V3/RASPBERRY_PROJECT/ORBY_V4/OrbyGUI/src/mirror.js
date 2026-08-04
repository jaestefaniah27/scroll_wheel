// Espejo local de lo que hay en el teclado.
//
// Hasta ahora los perfiles solo existían en la memoria del Orby: sin él
// enchufado la app se abría vacía y no se podía tocar nada. Aquí se guarda en
// el PC una copia completa —nombres, etiquetas, atajos, mandos, rueda, páginas
// e iconos OLED— después de cada cambio, con el mismo formato que la copia de
// seguridad manual (backup.js), para no tener dos maneras distintas de
// representar lo mismo.
//
// Con esa copia:
//   - la app arranca enseñando los perfiles aunque no haya teclado,
//   - se pueden editar teclas, etiquetas, iconos, mandos, rueda y secuencias
//     sin él (los comandos que irían por el puerto serie se dan por hechos, ver
//     el cortocircuito de device.js),
//   - al reconectar se pregunta si volcar esos cambios al teclado.
//
// Lo que NO se puede hacer sin el teclado es crear o borrar perfiles y páginas:
// eso desplaza los índices de todo lo demás y dejaría el espejo apuntando a
// huecos que no existen. device.js lo rechaza con un aviso claro.
//
// Las secuencias/macros del PC ya vivían en esta misma configuración local
// (`macros`, ver profiles.js), así que no hacía falta espejarlas: nunca han
// dependido del teclado para guardarse.

import { state, notify, subscribe } from './store.js';
import { blankPage, bindPageAliases } from './device.js';
import { snapshotFromState, importAll } from './backup.js';
import * as cache from './oled-cache.js';

// Hasta que no se haya leído (o descartado) el espejo del disco no se guarda
// nada: si no, el primer notify() de una app aún vacía machacaría la copia
// buena con cero perfiles.
let ready = false;
let offlineEdits = false;
let saveTimer = null;

const SAVE_DELAY_MS = 400;

// --- Carga ------------------------------------------------------------------

// Lee el espejo y, si no hay teclado, lo vuelca en el estado de la app para
// poder trabajar con él. Devuelve true si se ha usado para rellenar la app.
export async function init() {
  let stored = null;
  try {
    stored = (await window.orby.getConfig())?.deviceMirror ?? null;
  } catch {
    stored = null;
  }

  ready = true;
  if (!stored?.snapshot?.profiles?.length) return false;

  offlineEdits = Boolean(stored.offlineEdits);

  // Con teclado conectado manda el teclado: syncFromDevice ya habrá traído (o
  // estará trayendo) lo de verdad, y pisarlo con la copia sería ir hacia atrás.
  if (state.connected) return false;

  hydrate(stored);
  notify();
  return true;
}

function hydrate(stored) {
  const snap = stored.snapshot;

  state.profiles = snap.profiles.map((p, idx) => profileFromSnapshot(p, idx));
  state.activeProfileIdx = Math.min(snap.activeProfile || 0, state.profiles.length - 1);
  if (Number.isInteger(snap.timeout)) state.timeout = snap.timeout;
  if (Number.isInteger(snap.maxProfiles)) state.maxProfiles = snap.maxProfiles;

  const active = state.profiles[state.activeProfileIdx];
  state.pageIdx = 0;
  state.pageCount = active?.pageCount || 1;
  state.maxPages = Math.max(...state.profiles.map((p) => p.maxPages || 1), 1);

  cache.restore(stored.icons);
}

// Reconstruye un perfil con la misma forma que devuelve device.getProfile: las
// vistas leen `prof.labels`, `prof.keys`… y esos nombres son accesos a la
// página seleccionada, no campos propios (ver bindPageAliases).
function profileFromSnapshot(p, idx) {
  const prof = {
    idx,
    name: p.name || '',
    pageCount: p.pageCount || 1,
    maxPages: p.maxPages || 1,
    pageIdx: 0,
    pages: [],
  };

  const source = (p.pages && p.pages.length)
    ? p.pages
    : [{ labels: p.labels, keys: p.keys, rotary: p.rotary, scroll: p.scroll, oledMask: p.oledMask }];

  prof.pages = source.map((pg) => {
    const page = blankPage();
    fill(page.labels, pg?.labels, '');
    fill(page.keys, pg?.keys, null);
    fill(page.rotary, pg?.rotary, null);
    fill(page.scroll, pg?.scroll, null);
    page.oledMask = pg?.oledMask || 0;
    return page;
  });

  // Una copia con menos páginas de las que dice su recuento dejaría pageIdx
  // apuntando al vacío en cuanto se cambiase de pestaña.
  prof.pageCount = Math.max(1, Math.min(prof.pageCount, prof.pages.length));
  return bindPageAliases(prof);
}

// Copia `src` sobre `dst` sin cambiar su longitud: lo que falte se queda con el
// valor en blanco que trae blankPage().
function fill(dst, src, empty) {
  if (!Array.isArray(src)) return;
  for (let i = 0; i < dst.length && i < src.length; i++) {
    if (src[i] === undefined || src[i] === null) continue;
    dst[i] = empty === '' ? String(src[i]) : src[i];
  }
}

// --- Guardado ---------------------------------------------------------------

export function watch() {
  subscribe(() => {
    if (!ready || state.syncing || !state.profiles.length) return;
    // Un cambio marcado como pendiente sin teclado delante es, por definición,
    // una edición que el Orby todavía no conoce.
    if (state.dirty && !state.connected) offlineEdits = true;
    schedule();
  });
}

function schedule() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(save, SAVE_DELAY_MS);
}

export function save() {
  clearTimeout(saveTimer);
  if (!ready || !state.profiles.length) return;
  try {
    window.orby.setConfig({
      deviceMirror: {
        savedAt: new Date().toISOString(),
        offlineEdits,
        snapshot: snapshotFromState(),
        icons: cache.dump(),
      },
    });
  } catch (err) {
    console.error('No se pudo guardar el espejo local:', err);
  }
}

// --- Cambios hechos sin el teclado ------------------------------------------

// Devuelve si había ediciones sin subir y las da por atendidas de una vez: hay
// que leerlo ANTES de que un notify() con el teclado ya conectado pase por
// aquí, o el aviso se perdería.
export function takeOfflineEdits() {
  const had = offlineEdits;
  offlineEdits = false;
  return had;
}

export function hasOfflineEdits() {
  return offlineEdits;
}

// Sube al teclado lo que se editó sin él. Reutiliza el restaurador de copias de
// seguridad, que ya sabe crear las páginas que falten y escribir perfil a
// perfil; los iconos que suba son los de la página activa de cada perfil, igual
// que en una restauración normal.
export async function push(onProgress = () => {}) {
  const stored = (await window.orby.getConfig())?.deviceMirror;
  if (!stored?.snapshot?.profiles?.length) return;
  await importAll(stored.snapshot, onProgress);
  offlineEdits = false;
  save();
}
