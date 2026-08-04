// Caché de los bitmaps que el teclado tiene guardados.
//
// La usan el editor de iconos y el de perfiles: leer los 20 huecos de un perfil
// por el CDC lleva su tiempo, así que se hace una sola vez y las dos vistas
// comparten el resultado. Una entrada a null significa "ese hueco usa la
// etiqueta de texto", que es distinto de "no lo hemos leído todavía".

import * as device from './device.js';
import { state } from './store.js';
import * as fb from './oled-fb.js';

const cache = new Map();        // "perfil:pagina:hueco" -> Uint8Array | null
const listeners = new Set();

let loadingProfile = null;      // perfil que se está descargando
let queued = null;              // perfil pedido mientras había otro en curso

// La clave lleva la página: con páginas, el mismo (perfil, hueco) es un icono
// distinto en cada una, así que sin esto se enseñarían los de la página anterior.
// La página se toma del perfil, que es quien sabe cuál se está editando.
function pageOfProfile(profile) {
  const prof = state.profiles[profile];
  return prof ? (prof.pageIdx || 0) : 0;
}

export function cacheKey(profile, slot, page = pageOfProfile(profile)) {
  return `${profile}:${page}:${slot}`;
}

export function get(profile, slot, page) {
  return cache.get(cacheKey(profile, slot, page)) ?? null;
}

export function has(profile, slot, page) {
  return cache.has(cacheKey(profile, slot, page));
}

export function set(profile, slot, bytes, page) {
  cache.set(cacheKey(profile, slot, page), bytes);
  emit();
}

export function clearAll() {
  cache.clear();
  emit();
}

// --- Volcado para el espejo local ------------------------------------------
// Los iconos son lo único del teclado que no cabe en GET_PROFILE, así que sin
// guardarlos aparte la copia del PC no serviría para trabajar sin el Orby
// delante. Solo se vuelcan los huecos con bitmap: los que están a null son
// "ese hueco usa su etiqueta de texto", que es el estado por defecto.
function bytesToHex(bytes) {
  let hex = '';
  for (const b of bytes) hex += b.toString(16).padStart(2, '0');
  return hex;
}

function hexToBytes(hex) {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.substr(i * 2, 2), 16);
  return out;
}

export function dump() {
  const out = {};
  for (const [key, bytes] of cache) if (bytes) out[key] = bytesToHex(bytes);
  return out;
}

export function restore(dumped) {
  for (const [key, hex] of Object.entries(dumped || {})) {
    if (typeof hex === 'string' && hex.length) cache.set(key, hexToBytes(hex));
  }
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

// Tira lo leído de un perfil concreto, con todas sus páginas. Se usa para
// limpiar el espejo del disco de las entradas que pudo envenenar el fallo de
// carrera de loadProfile (ver mirror.js).
export function dropProfile(profileIdx) {
  const prefix = `${profileIdx}:`;
  for (const key of [...cache.keys()]) {
    if (key.startsWith(prefix)) cache.delete(key);
  }
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

    // Los iconos que se leen son los de la página que el teclado tiene puesta:
    // GET_OLED, como el resto de comandos sin número de página, actúa sobre ella.
    const page = prof.pageIdx || 0;

    // Leer los 20 huecos son 20 idas y vueltas por el CDC, y da tiempo de sobra
    // a cambiar de pestaña por el medio. En cuanto eso pasa, lo que conteste el
    // teclado ya es de la página nueva: guardarlo con la clave de la vieja la
    // deja con los iconos de la otra para siempre, porque `cache.has` la da por
    // leída y nadie vuelve a pedirla. Se corta y se encola la recarga, que ya
    // leerá la página que toque.
    const outOfDate = () => state.profiles[profileIdx] !== prof || (prof.pageIdx || 0) !== page;

    for (let slot = 0; slot < 20; slot++) {
      if (outOfDate()) { queued = profileIdx; return; }
      if (cache.has(cacheKey(profileIdx, slot, page))) continue;
      if (!(prof.oledMask & (1 << slot))) {
        cache.set(cacheKey(profileIdx, slot, page), null);
        continue;
      }
      const bytes = await device.getOled(profileIdx, slot);
      // Otra vez después de esperar: el cambio de página ocurre justo aquí.
      if (outOfDate()) { queued = profileIdx; return; }
      cache.set(cacheKey(profileIdx, slot, page), bytes);
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
