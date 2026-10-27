// Biblioteca de perfiles listos para usar (Altium, VS Code, Excel...).
//
// Los perfiles salen de tools/perfiles/biblioteca.py, que genera
// profile-library.json: no se editan aquí. Cada uno trae sus páginas completas
// (etiquetas, atajos, mandos y rueda); los iconos de las pantallas los pone el
// firmware a partir de las etiquetas, así que no hace falta subir ni un bitmap.

import LIBRARY_DATA from './profile-library.json';
import * as device from './device.js';
import { state, syncFromDevice, pageCountOf, withPinnedPage, markDirty } from './store.js';
import * as cache from './oled-cache.js';
import { MACRO_MODIFIER } from './hid-keys.js';
import * as macros from './views/profiles/macros-store.js';

export const LIBRARY = LIBRARY_DATA.profiles;

// Pausa entre las teclas de una secuencia. Los aceleradores de menú de Altium
// («P, V») necesitan que el menú llegue a abrirse antes de la segunda tecla.
const SEQUENCE_GAP_MS = 50;

function sequenceActions(seq) {
  const actions = [];
  seq.forEach((s, i) => {
    if (i) actions.push({ type: 'delay', ms: SEQUENCE_GAP_MS });
    actions.push({ type: 'hotkey', modifier: s.modifier, keycode: s.keycode });
  });
  return actions;
}

function sameActions(a, b) {
  return a.length === b.length && a.every((x, i) => {
    const y = b[i];
    return x.type === y.type && (x.ms ?? 0) === (y.ms ?? 0)
        && (x.modifier ?? 0) === (y.modifier ?? 0) && (x.keycode ?? 0) === (y.keycode ?? 0);
  });
}

// Una secuencia de la biblioteca se guarda como una macro más. Si ya existe una
// idéntica (por instalar dos veces el mismo perfil, o porque dos perfiles usan
// la misma), se reutiliza: los ids del teclado son pocos.
async function macroForSequence(seq) {
  const actions = sequenceActions(seq);
  const all = macros.allMacros();
  const same = all.find((m) => sameActions(m.actions || [], actions));
  if (same) return same.id;

  const id = macros.nextMacroId();
  // Se mete en la lista directamente en vez de con ensureMacro: esa la crea
  // vacía y la sube, y esa subida vacía podría llegar al teclado DESPUÉS de la
  // buena y dejarla borrada.
  all.push({ id, actions });
  macros.savePCMacros();
  await macros.syncMacroToDevice(id);
  return id;
}

// Lo que se escribe en una tecla: un atajo tal cual, o la macro de su secuencia.
async function resolveKey(key) {
  if (!key.sequence) return key;
  return { modifier: MACRO_MODIFIER, keycode: await macroForSequence(key.sequence) };
}

// Escribe una página completa en la que el teclado tenga puesta para ese
// perfil. Quien llama se encarga de ponerla (withPinnedPage).
export async function writePage(profileIdx, page, onStep = () => {}) {
  for (let s = 0; s < 20; s++) {
    await device.setLabel(profileIdx, s, page.labels?.[s] ?? '');
  }
  onStep();
  for (let s = 0; s < 24; s++) {
    const k = await resolveKey(page.keys?.[s] || { modifier: 0, keycode: 0 });
    await device.setKeymap(profileIdx, s, k.modifier, k.keycode);
  }
  onStep();
  for (let s = 0; s < 16; s++) {
    const r = page.rotary?.[s];
    if (r) await device.setRotary(profileIdx, s, r.type, r.modifier, r.keycode);
  }
  for (let layer = 0; layer < 2; layer++) {
    const sc = page.scroll?.[layer];
    if (sc?.detentsPerRev) await device.setProfileScroll(profileIdx, layer, sc.detentsPerRev, sc.invert);
  }
  onStep();
}

// Deja `count` páginas en el perfil, creando las que falten. El firmware solo
// deja escribir las páginas de un perfil que esté puesto, así que hay que
// activarlo antes (ver ensureEditTarget en store.js).
async function preparePages(profileIdx, count) {
  if (state.activeProfileIdx !== profileIdx) {
    await device.setProfile(profileIdx);
    await syncFromDevice();
  }
  while (pageCountOf(state.profiles[profileIdx]) < count) {
    const before = pageCountOf(state.profiles[profileIdx]);
    // addPage no lanza con un ERR: (lo devuelve como respuesta), así que si el
    // teclado no crea la página hay que verlo en el recuento o esto no acaba.
    await device.addPage(profileIdx, false);
    await syncFromDevice();
    if (pageCountOf(state.profiles[profileIdx]) <= before) {
      throw new Error('El teclado no admite más páginas en este perfil');
    }
  }
}

// Escribe todas las páginas de un perfil: lo usan la biblioteca y la
// restauración de copias de seguridad.
export async function writeAllPages(profileIdx, pages, onProgress = () => {}) {
  const wanted = Math.min(pages.length, state.maxPages || 1);
  await preparePages(profileIdx, wanted);
  for (let pg = 0; pg < wanted; pg++) {
    let step = 0;
    await withPinnedPage(profileIdx, pg, () => writePage(profileIdx, pages[pg], () => {
      step++;
      onProgress(`Página ${pg + 1} de ${wanted}… (${Math.round((step / 3) * 100)} %)`);
    }));
  }
}

// Añade un perfil de la biblioteca al final de la lista. Se queda activo: es el
// que el usuario quiere probar. Devuelve su índice.
export async function installProfile(def, onProgress = () => {}) {
  if (!state.connected) throw new Error('Teclado no conectado');
  if (state.profiles.length >= state.maxProfiles) {
    throw new Error(`El teclado admite como máximo ${state.maxProfiles} perfiles`);
  }
  if (def.pages.length > (state.maxPages || 1)) {
    throw new Error(`Este perfil trae ${def.pages.length} páginas y el firmware admite ${state.maxPages || 1}`);
  }

  // Las secuencias se guardan con las demás macros: hay que partir de la lista
  // de verdad, no de una vacía, o se reutilizarían ids ocupados.
  await macros.loadPCMacros();

  onProgress('Creando el perfil…');
  const ret = await device.addProfile();
  cache.invalidate();
  await syncFromDevice();
  const idx = Number.isInteger(ret) && ret < state.profiles.length ? ret : state.profiles.length - 1;

  await device.setName(idx, def.name);
  await writeAllPages(idx, def.pages, onProgress);

  await device.setPage(0);
  markDirty();
  await syncFromDevice();
  return idx;
}
