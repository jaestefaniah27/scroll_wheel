// Estado compartido de la aplicación.
//
// Los perfiles ya no se declaran aquí: se leen del firmware con GET_PROFILE al
// conectar. Antes estaban duplicados en el código de la app, así que cualquier
// cambio en el firmware hacía que la interfaz mostrase datos falsos.
//
// Desde el firmware 3.0 el número de perfiles es variable (se crean, duplican y
// borran desde la app) y la rueda de scroll es un ajuste más de cada perfil,
// con un valor por capa.

import * as device from './device.js';

const subscribers = new Set();

export const state = {
  connected: false,
  deviceInfo: null,
  activeProfileIdx: 0,
  maxProfiles: 8,
  deviceMode: 'NORMAL',
  superActive: false,
  brightness: 207,
  timeout: 5,
  scroll: { detentsPerRev: 60, invert: false, hires: false }, // lo que aplica ahora mismo
  profiles: [],       // rellenado desde el firmware
  dirty: false,       // hay cambios sin escribir en Flash
  syncing: false,
  // Páginas del perfil activo. Un firmware anterior al 4.0 no las manda y todo
  // se queda en una sola, que es como funcionaba.
  pageIdx: 0,
  pageCount: 1,
  maxPages: 1,
};

// La página que se está editando en la app es también la que el teclado tiene
// puesta: los comandos de edición del firmware actúan sobre la página activa, así
// que al elegir una pestaña se cambia también en el teclado. Tiene la ventaja de
// que las pantallas enseñan siempre lo que estás tocando.
export function pageOf(prof, idx = state.pageIdx) {
  if (!prof || !prof.pages) return null;
  return prof.pages[idx] || prof.pages[0] || null;
}

export function pageCountOf(prof) {
  return prof && prof.pages ? Math.max(1, prof.pageCount || prof.pages.length) : 1;
}

export function maxPages() {
  return Math.max(1, state.maxPages || 1);
}

export function hasPages() {
  return maxPages() > 1;
}

// Metadatos que solo viven en la app (el firmware no guarda iconos ni colores).
// La lista se recorre en bucle: con perfiles creados por el usuario ya no hay un
// número fijo de entradas.
const PROFILE_PALETTE = [
  { icon: 'profiles', accent: '#8b5cf6' },
  { icon: 'pencil',   accent: '#ec4899' },
  { icon: 'bolt',     accent: '#22d3ee' },
  { icon: 'oled',     accent: '#f59e0b' },
  { icon: 'key',      accent: '#10b981' },
  { icon: 'wheel',    accent: '#f43f5e' },
  { icon: 'text',     accent: '#6366f1' },
  { icon: 'fill',     accent: '#84cc16' },
];

export function profileMeta(idx) {
  return PROFILE_PALETTE[idx % PROFILE_PALETTE.length];
}

export function subscribe(fn) {
  subscribers.add(fn);
  return () => subscribers.delete(fn);
}

// Cada suscriptor va en su propio try/catch: si una vista revienta al pintar,
// no debe bloquear a las demás (en particular, que el indicador de "Guardar
// en Flash" siga actualizándose pase lo que pase en otra vista).
export function notify() {
  for (const fn of subscribers) {
    try {
      fn(state);
    } catch (err) {
      console.error('Error en un suscriptor de store.notify():', err);
    }
  }
}

export function markDirty() {
  state.dirty = true;
  notify();
}

export function profile(idx = state.activeProfileIdx) {
  return state.profiles[idx] || null;
}

// El firmware indexa 12 teclas y 10 pantallas. Las teclas 10 (menú) y 12
// (SUPER) no tienen pantalla asociada, y la tecla 11 usa la pantalla 10.
export const KEY_TO_SCREEN = [1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 10, 0];

// Las dos teclas de sistema, numeradas como las ve el usuario (1 a 12). Tienen
// que coincidir con KEY_IDX_SUPER y KEY_IDX_MENU del firmware, que van en
// base 0. Antes estaban al revés: SUPER en la 10 y el menú en la 12.
export const KEY_SUPER = 12;
export const KEY_MENU  = 10;

export function labelForKey(prof, keyIndex, layer) {
  const screen = KEY_TO_SCREEN[keyIndex];
  if (!screen || !prof) return null;
  return prof.labels[(screen - 1) + (layer === 'super' ? 10 : 0)] || '';
}

export function labelSlot(keyIndex, layer) {
  const screen = KEY_TO_SCREEN[keyIndex];
  if (!screen) return -1;
  return (screen - 1) + (layer === 'super' ? 10 : 0);
}

export function keymapSlot(keyIndex, layer) {
  return keyIndex + (layer === 'super' ? 12 : 0);
}

// Los mandos giratorios repiten sus ocho huecos en la capa SUPER, igual que las
// teclas: el 0-7 es la capa normal y el 8-15 la de SUPER.
export const ROTARY_LAYER_STRIDE = 8;

export function rotarySlot(baseSlot, layer) {
  return baseSlot + (layer === 'super' ? ROTARY_LAYER_STRIDE : 0);
}

export function layerIndex(layer) {
  return layer === 'super' ? 1 : 0;
}

// Configuración de rueda de un perfil y capa, con valores por defecto para los
// perfiles que todavía no se han leído del teclado.
export function scrollFor(prof, layer) {
  return prof?.scroll?.[layerIndex(layer)] || { detentsPerRev: 60, invert: false };
}

// Descarga estado y perfiles completos desde el dispositivo.
export async function syncFromDevice() {
  state.syncing = true;
  notify();
  try {
    const s = await device.getState();
    let count = state.profiles.length || 4;

    if (s) {
      if (Number.isInteger(s.profile))      state.activeProfileIdx = s.profile;
      if (Number.isInteger(s.profileCount)) count = s.profileCount;
      if (Number.isInteger(s.maxProfiles))  state.maxProfiles = s.maxProfiles;
      if (Number.isInteger(s.brightness))   state.brightness = s.brightness;
      if (Number.isInteger(s.timeout))      state.timeout = s.timeout;
      if (typeof s.superActive === 'boolean') state.superActive = s.superActive;
      if (s.mode)   state.deviceMode = s.mode;
      if (s.scroll) state.scroll = s.scroll;
      if (Number.isInteger(s.pageIdx))   state.pageIdx   = s.pageIdx;
      if (Number.isInteger(s.pageCount)) state.pageCount = s.pageCount;
      if (Number.isInteger(s.maxPages))  state.maxPages  = s.maxPages;
    }

    const profiles = [];
    for (let i = 0; i < count; i++) profiles.push(await device.getProfile(i));
    state.profiles = profiles;
    if (state.activeProfileIdx >= profiles.length) state.activeProfileIdx = 0;

    // Cada perfil se abre en la página que el teclado tiene puesta si es el
    // activo, y en la primera si no.
    for (const p of profiles) {
      p.pageIdx = (p.idx === state.activeProfileIdx)
        ? Math.min(state.pageIdx, pageCountOf(p) - 1) : 0;
    }
    state.pageCount = pageCountOf(profiles[state.activeProfileIdx]);
    state.dirty = false;
  } finally {
    state.syncing = false;
    notify();
  }
}

// ---------- Páginas ----------
// Cambiar de página en la app la cambia también en el teclado, porque los
// comandos de edición del firmware actúan sobre la página activa. Si el perfil
// que se está editando no es el activo, no hay nada que sincronizar: sus
// comandos van a su primera página.
export async function selectPage(idx) {
  const prof = profile();
  const count = pageCountOf(prof);
  if (idx < 0 || idx >= count) return;

  if (prof) prof.pageIdx = idx;
  state.pageIdx = idx;
  notify();

  try {
    await device.setPage(idx);
  } catch (e) {
    // Si el teclado no acepta el cambio, la app no puede seguir enseñando otra
    // página: lo que se editase iría a la equivocada.
    await syncFromDevice();
    throw e;
  }
}

export async function addPage() {
  const prof = profile();
  if (!prof || pageCountOf(prof) >= maxPages()) return false;
  await device.addPage(prof.idx, true);
  await syncFromDevice();
  markDirty();
  return true;
}

export async function removePage(idx) {
  const prof = profile();
  if (!prof || pageCountOf(prof) <= 1) return false;
  await device.delPage(prof.idx, idx);
  await syncFromDevice();
  markDirty();
  return true;
}

// La rueda que está aplicando el teclado ahora mismo: perfil activo y capa que
// se esté pulsando. Se usa para el estado en vivo del dashboard.
export function liveScroll() {
  const prof = profile();
  const s = scrollFor(prof, state.superActive ? 'super' : 'normal');
  return { ...s, hires: state.scroll.hires };
}
