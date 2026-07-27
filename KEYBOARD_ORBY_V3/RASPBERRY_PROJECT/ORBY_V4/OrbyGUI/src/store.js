// Estado compartido de la aplicación.
//
// Los perfiles ya no se declaran aquí: se leen del firmware con GET_PROFILE al
// conectar. Antes estaban duplicados en el código de la app, así que cualquier
// cambio en el firmware hacía que la interfaz mostrase datos falsos.

import * as device from './device.js';

const subscribers = new Set();

export const state = {
  connected: false,
  deviceInfo: null,
  activeProfileIdx: 0,
  deviceMode: 'NORMAL',
  superActive: false,
  brightness: 207,
  timeout: 5,
  scroll: { detentsPerRev: 60, invert: false, hires: false },
  profiles: [],       // rellenado desde el firmware
  dirty: false,       // hay cambios sin escribir en Flash
  syncing: false,
};

// Metadatos que solo viven en la app (el firmware no guarda iconos ni colores).
export const PROFILE_META = [
  { icon: 'profiles', accent: '#8b5cf6' },
  { icon: 'pencil',   accent: '#ec4899' },
  { icon: 'bolt',     accent: '#22d3ee' },
  { icon: 'oled',     accent: '#f59e0b' },
];

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

// Descarga estado y perfiles completos desde el dispositivo.
export async function syncFromDevice() {
  state.syncing = true;
  notify();
  try {
    const s = await device.getState();
    if (s) {
      if (Number.isInteger(s.profile))    state.activeProfileIdx = s.profile;
      if (Number.isInteger(s.brightness)) state.brightness = s.brightness;
      if (Number.isInteger(s.timeout))    state.timeout = s.timeout;
      if (s.mode)   state.deviceMode = s.mode;
      if (s.scroll) state.scroll = s.scroll;
    }

    const profiles = [];
    for (let i = 0; i < 4; i++) profiles.push(await device.getProfile(i));
    state.profiles = profiles;
    state.dirty = false;
  } finally {
    state.syncing = false;
    notify();
  }
}
