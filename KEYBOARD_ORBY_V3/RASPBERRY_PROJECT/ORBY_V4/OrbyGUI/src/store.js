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
};

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

export function notify() {
  for (const fn of subscribers) fn(state);
}

export function markDirty() {
  state.dirty = true;
  notify();
}

export function profile(idx = state.activeProfileIdx) {
  return state.profiles[idx] || null;
}

// El firmware indexa 12 teclas y 10 pantallas. Las teclas 10 (SUPER) y 12
// (menú) no tienen pantalla asociada, y la tecla 11 usa la pantalla 10.
export const KEY_TO_SCREEN = [1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 10, 0];

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
    }

    const profiles = [];
    for (let i = 0; i < count; i++) profiles.push(await device.getProfile(i));
    state.profiles = profiles;
    if (state.activeProfileIdx >= profiles.length) state.activeProfileIdx = 0;
    state.dirty = false;
  } finally {
    state.syncing = false;
    notify();
  }
}

// La rueda que está aplicando el teclado ahora mismo: perfil activo y capa que
// se esté pulsando. Se usa para el estado en vivo del dashboard.
export function liveScroll() {
  const prof = profile();
  const s = scrollFor(prof, state.superActive ? 'super' : 'normal');
  return { ...s, hires: state.scroll.hires };
}
