// Espejo local de lo que hay en el teclado.
//
// Hasta ahora los perfiles solo existían en la memoria del Orby: sin él
// enchufado la app se abría vacía. Aquí se guarda en el PC una copia completa
// —nombres, etiquetas, atajos, mandos, rueda, páginas e iconos OLED— después de
// cada cambio, con el mismo formato que la copia de seguridad manual
// (backup.js), para no tener dos maneras distintas de representar lo mismo.
//
// Esa copia sirve para dos cosas:
//
//   - Abrir la app sin el teclado y ver la configuración entera. En modo
//     LECTURA: sin el Orby delante no se deja cambiar nada, porque entonces
//     habría dos versiones distintas de lo mismo y al reconectar alguien tendría
//     que perder. device.js rechaza cualquier comando de edición sin conexión.
//
//   - Evitar descargarlo todo en cada conexión. El teclado resume su
//     configuración con GET_HASH y la app calcula la misma huella sobre esta
//     copia (hash.js): los perfiles cuya huella coincide no se piden. Antes se
//     leían siempre los perfiles enteros y los veinte iconos de cada uno.
//
// Las secuencias/macros del PC ya vivían en esta misma configuración local
// (`macros`, ver profiles.js), así que no hacía falta espejarlas: nunca han
// dependido del teclado para guardarse.

import { state, notify, subscribe, pageCountOf } from './store.js';
import { blankPage, bindPageAliases } from './device.js';
import { snapshotFromState } from './backup.js';
import * as cache from './oled-cache.js';

// Hasta que no se haya leído (o descartado) el espejo del disco no se guarda
// nada: si no, el primer notify() de una app aún vacía machacaría la copia
// buena con cero perfiles.
let ready = false;
let saveTimer = null;

const SAVE_DELAY_MS = 400;

// Los iconos guardados por una versión anterior a la 3.2.2 pueden estar
// cruzados entre páginas: si se cambiaba de pestaña mientras se leían del
// teclado, los que faltaban se guardaban con la clave de la página anterior
// (arreglado en oled-cache.loadProfile). Se marca el volcado con esta versión
// para descartar una sola vez los que pudieron salir mal.
const ICONS_VERSION = 2;

// --- Carga ------------------------------------------------------------------

// Lee el espejo del disco y lo vuelca en el estado de la app. Devuelve true si
// había copia y se ha usado.
//
// Se carga SIEMPRE, haya teclado o no, y cuanto antes: con el Orby conectado es
// justo lo que se compara contra las huellas de GET_HASH para no descargar los
// perfiles que no han cambiado. Si algo no cuadra, syncFromDevice lo sustituye
// por lo que diga el teclado, así que una copia vieja no puede hacer daño.
//
// Lo único que no se pisa es una configuración ya cargada: si el teclado se ha
// adelantado, lo suyo manda.
export async function init() {
  let stored = null;
  try {
    stored = (await window.orby.getConfig())?.deviceMirror ?? null;
  } catch {
    stored = null;
  }

  ready = true;
  if (!stored?.snapshot?.profiles?.length) return false;
  if (state.profiles.length) return false;

  hydrate(stored);
  console.debug(`[mirror] copia del PC cargada: ${state.profiles.length} perfiles,`,
                `${Object.keys(stored.icons || {}).length} iconos`);
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

  // Volcado de una versión con el fallo de páginas cruzadas: se tiran los
  // iconos de los perfiles con más de una página, que son los únicos que
  // pudieron mezclarse. Los de una sola página nunca cambian de página, así
  // que se conservan y la app sigue enseñándolos sin el teclado delante. Lo
  // descartado se vuelve a leer en cuanto se conecte el Orby.
  if ((stored.iconsVersion || 0) < ICONS_VERSION) {
    for (const p of state.profiles) {
      if (pageCountOf(p) > 1) cache.dropProfile(p.idx);
    }
  }
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
  subscribe(() => touch());

  // Los iconos que llegan del teclado no pasan por notify(): la caché tiene sus
  // propios avisos. Sin engancharse aquí, un icono descargado al cambiar de
  // página se quedaba fuera de la copia hasta que algo más la hiciera guardar,
  // y sin él la conexión siguiente no puede comparar huellas.
  cache.onChange(() => touch());
}

// Programa el guardado si toca. Se queda corto a propósito en dos casos:
// mientras se está sincronizando (el estado está a medias) y sin teclado
// delante, donde lo que hay en pantalla es esta misma copia en solo lectura y
// reescribirla no aporta nada.
export function touch() {
  if (!ready || state.syncing || !state.profiles.length) return;
  if (!state.connected) return;
  schedule();
}

function schedule() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(save, SAVE_DELAY_MS);
}

// Lo último que se escribió, para no reescribir lo mismo. El guardado se
// dispara con cada notify(), y muchos no cambian la configuración: girar la
// rueda, pulsar SUPER o un cambio de página del propio teclado también notifican.
// Cada uno de esos volcaba a disco los perfiles enteros con sus iconos (más de
// cien kilobytes, y el proceso principal los escribe de una vez).
let lastSignature = null;

export function save() {
  clearTimeout(saveTimer);
  if (!ready || !state.profiles.length) return;

  const snapshot = snapshotFromState();
  const icons = cache.dump();
  // savedAt cambia siempre, así que queda fuera de la comparación: si no,
  // nunca coincidiría nada.
  const signature = JSON.stringify({ snapshot: { ...snapshot, savedAt: null }, icons });
  if (signature === lastSignature) return;

  try {
    window.orby.setConfig({
      deviceMirror: {
        savedAt: new Date().toISOString(),
        snapshot,
        icons,
        iconsVersion: ICONS_VERSION,
      },
    });
    lastSignature = signature;
  } catch (err) {
    console.error('No se pudo guardar el espejo local:', err);
  }
}
