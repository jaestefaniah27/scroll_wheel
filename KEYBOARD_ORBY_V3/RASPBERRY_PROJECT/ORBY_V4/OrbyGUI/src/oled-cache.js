// Caché de los bitmaps que el teclado tiene guardados.
//
// La usan el editor de iconos y el de perfiles: leer los 20 huecos de un perfil
// por el CDC lleva su tiempo, así que se hace una sola vez y las dos vistas
// comparten el resultado. Una entrada a null significa "ese hueco usa la
// etiqueta de texto", que es distinto de "no lo hemos leído todavía".

import * as device from './device.js';
import { state } from './store.js';
import * as fb from './oled-fb.js';

const cache = new Map();        // "perfil:hueco" -> Uint8Array | null
const listeners = new Set();

let loadingProfile = null;      // perfil que se está descargando
let queued = null;              // perfil pedido mientras había otro en curso

export function cacheKey(profile, slot) {
  return `${profile}:${slot}`;
}

export function get(profile, slot) {
  return cache.get(cacheKey(profile, slot)) ?? null;
}

export function has(profile, slot) {
  return cache.has(cacheKey(profile, slot));
}

export function set(profile, slot, bytes) {
  cache.set(cacheKey(profile, slot), bytes);
  emit();
}

export function clearAll() {
  cache.clear();
  emit();
}

// Tras crear, duplicar o borrar un perfil los índices se desplazan, así que lo
// leído deja de ser válido.
export function invalidate() {
  cache.clear();
  loadingProfile = null;
  queued = null;
  emit();
}

export function isLoading() {
  return loadingProfile !== null;
}

export function onChange(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

function emit() {
  for (const fn of listeners) fn();
}

// Descarga los huecos ocupados de un perfil. Solo se piden los marcados en la
// máscara que llega con GET_PROFILE; el resto se marcan como vacíos sin gastar
// una petición.
export async function loadProfile(profileIdx) {
  if (!state.connected) return;
  if (loadingProfile !== null) { queued = profileIdx; return; }

  loadingProfile = profileIdx;
  emit();

  try {
    const prof = state.profiles[profileIdx];
    if (!prof) return;

    for (let slot = 0; slot < 20; slot++) {
      if (cache.has(cacheKey(profileIdx, slot))) continue;
      if (!(prof.oledMask & (1 << slot))) {
        cache.set(cacheKey(profileIdx, slot), null);
        continue;
      }
      cache.set(cacheKey(profileIdx, slot), await device.getOled(profileIdx, slot));
      emit();
    }
  } finally {
    loadingProfile = null;
    emit();
    if (queued !== null) {
      const next = queued;
      queued = null;
      loadProfile(next);
    }
  }
}

// Pinta en un <canvas> el bitmap de un hueco. Los canvas no se pueden serializar
// dentro de una plantilla, así que las vistas los rellenan tras insertar el HTML.
export function paintThumbs(root = document) {
  root.querySelectorAll('[data-bmp]').forEach((canvas) => {
    const bmp = cache.get(canvas.dataset.bmp);
    if (bmp) fb.renderToCanvas(bmp, canvas, { zoom: 1, grid: false });
  });
}
