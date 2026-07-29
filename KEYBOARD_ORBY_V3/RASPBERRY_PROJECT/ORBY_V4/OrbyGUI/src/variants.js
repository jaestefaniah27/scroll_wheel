// Variaciones de perfil según la aplicación.
//
// Un perfil es el juego de atajos; una variación son solo las *diferencias*
// para una app concreta. Así, si en tu editor «seleccionar todo» es Ctrl+E en
// vez de Ctrl+A, cambias esa tecla y nada más, en lugar de duplicar el perfil
// entero y tener que mantener dos copias de todo.
//
// Se guardan en el PC (el teclado no sabe qué aplicación tienes delante) y se
// escriben en la RAM del teclado al vuelo:
//
//   aplicar   -> se mandan los huecos de la variación
//   revertir  -> se reescriben esos mismos huecos con el valor del perfil base
//
// El perfil base que guarda la app (store.state.profiles) NUNCA se toca con los
// valores de la variación: es la referencia para poder deshacer. Por eso, antes
// de escribir en Flash hay que revertir (withBase), o el teclado guardaría la
// variación como si fuese el perfil.

import * as device from './device.js';
import { state as store, labelSlot, keymapSlot, rotarySlot } from './store.js';

// { id, profile, name, matches:[texto], field, keys:{slot:{modifier,keycode}},
//   rotary:{slot:{type,modifier,keycode}}, labels:{slot:texto} }
//
// `matches` es una lista porque un mismo retoque suele valer para varias apps:
// los cambios que usas en el Bloc de notas también los quieres en el visor de
// texto o en cualquier otra aplicación de Windows que se comporte igual.
export const variants = [];

let applied = null;    // variación escrita ahora mismo en el teclado
let lastInfo = null;   // última ventana en primer plano
let enabled = false;   // el detector de aplicaciones está en marcha
const listeners = new Set();

export function onChange(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

// `reason` distingue un cambio de datos (editar una variación) de un cambio del
// motor (se ha aplicado o quitado). Las vistas que repintan entero solo deben
// reaccionar al segundo: si no, escribir una etiqueta perdería el foco a mitad.
function emit(reason = 'data') {
  for (const fn of listeners) fn(reason);
}

function persist() {
  return window.orby.setConfig({ profileVariants: variants });
}

// Las primeras variaciones guardaban una sola app en `match`; ahora es una
// lista. Se convierten al cargarlas para no perder lo que ya hubiera.
function normalize(v) {
  if (!Array.isArray(v.matches)) {
    v.matches = v.match ? [v.match] : [];
  }
  delete v.match;
  v.keys = v.keys || {};
  v.rotary = v.rotary || {};
  v.labels = v.labels || {};
  return v;
}

export async function init() {
  try {
    const cfg = await window.orby.getConfig();
    variants.push(...(cfg.profileVariants || []).map(normalize));
  } catch {
    // Sin configuración local: se empieza sin variaciones.
  }

  // Al desconectar, el teclado pierde la RAM: lo aplicado deja de estarlo.
  device.on('disconnected', () => {
    applied = null;
    emit('applied');
  });
}

// --- Consulta -----------------------------------------------------------

export function forProfile(profileIdx) {
  return variants.filter((v) => v.profile === profileIdx);
}

export function get(id) {
  return variants.find((v) => v.id === id) || null;
}

export function activeVariant() {
  return applied;
}

export function isApplied(id) {
  return applied?.id === id;
}

// Un hueco puede estar redefinido en teclas, mandos o etiquetas.
export function override(variant, kind, slot) {
  return variant?.[kind]?.[slot] ?? null;
}

export function countOverrides(variant) {
  if (!variant) return 0;
  return Object.keys(variant.keys || {}).length
       + Object.keys(variant.rotary || {}).length
       + Object.keys(variant.labels || {}).length;
}

// --- Alta, baja y edición ------------------------------------------------

export function create(profileIdx, { name, matches = [], field = 'any' } = {}) {
  const variant = {
    id: `v${Date.now()}${Math.random().toString(36).slice(2, 6)}`,
    profile: profileIdx,
    name: name || 'Variación',
    matches: matches.filter(Boolean),
    field,
    keys: {}, rotary: {}, labels: {},
  };
  variants.push(variant);
  persist();
  emit();
  return variant;
}

// Apps que disparan la variación. Se comparan en minúsculas y sin duplicados.
export function addMatch(id, text) {
  const v = get(id);
  const needle = (text || '').trim().toLowerCase();
  if (!v || !needle || v.matches.includes(needle)) return false;
  v.matches.push(needle);
  persist();
  emit();
  return true;
}

export function removeMatch(id, text) {
  const v = get(id);
  if (!v) return;
  const i = v.matches.indexOf(text);
  if (i >= 0) v.matches.splice(i, 1);
  persist();
  emit();
}

export function update(id, patch) {
  const v = get(id);
  if (!v) return;
  Object.assign(v, patch);
  persist();
  emit();
}

export async function remove(id) {
  if (isApplied(id)) await revert();
  const i = variants.findIndex((v) => v.id === id);
  if (i >= 0) variants.splice(i, 1);
  persist();
  emit();
}

export function setOverride(id, kind, slot, value) {
  const v = get(id);
  if (!v) return;
  v[kind] = v[kind] || {};
  v[kind][slot] = value;
  persist();
  emit();
}

export function clearOverride(id, kind, slot) {
  const v = get(id);
  if (!v?.[kind]) return;
  delete v[kind][slot];
  persist();
  emit();
}

// Borrar un perfil desplaza los índices de los siguientes; las variaciones del
// perfil borrado se van con él.
export function shiftProfiles(deletedIdx) {
  for (let i = variants.length - 1; i >= 0; i--) {
    if (variants[i].profile === deletedIdx) variants.splice(i, 1);
    else if (variants[i].profile > deletedIdx) variants[i].profile--;
  }
  persist();
  emit();
}

// --- Motor ---------------------------------------------------------------

export function setForeground(info) {
  lastInfo = info;
}

export function setEnabled(value) {
  enabled = value;
}

function haystack(info, field) {
  if (!info) return '';
  if (field === 'title') return (info.title || '').toLowerCase();
  if (field === 'process') return (info.process || '').toLowerCase();
  return `${info.process || ''} ${info.title || ''}`.toLowerCase();
}

// Gana la primera variación del perfil con alguna de sus apps en la ventana.
export function matchFor(profileIdx, info) {
  for (const v of variants) {
    if (v.profile !== profileIdx) continue;
    const hay = haystack(info, v.field);
    for (const needle of v.matches || []) {
      if (needle && hay.includes(needle)) return v;
    }
  }
  return null;
}

// Escribe los huecos de una variación, o los devuelve a su valor base.
async function writeSlots(variant, useBase) {
  const prof = store.profiles[variant.profile];
  if (!prof) return;

  for (const [slot, action] of Object.entries(variant.keys || {})) {
    const a = useBase ? (prof.keys[slot] || { modifier: 0, keycode: 0 }) : action;
    await device.setKeymap(variant.profile, Number(slot), a.modifier, a.keycode);
  }
  for (const [slot, action] of Object.entries(variant.rotary || {})) {
    const a = useBase ? (prof.rotary[slot] || { type: 0, modifier: 0, keycode: 0 }) : action;
    await device.setRotary(variant.profile, Number(slot), a.type, a.modifier, a.keycode);
  }
  for (const [slot, text] of Object.entries(variant.labels || {})) {
    const t = useBase ? (prof.labels[slot] || '') : text;
    await device.setLabel(variant.profile, Number(slot), t);
  }
}

export async function apply(id) {
  const v = get(id);
  if (!v || !store.connected) return;
  if (applied && applied.id !== v.id) await revert();
  try {
    await writeSlots(v, false);
    applied = v;
    emit('applied');
  } catch {
    // El teclado puede estar ocupado; se reintenta en la siguiente evaluación.
  }
}

// Reafirma un hueco de la variación aplicada. Se usa tras editar el perfil
// base: la escritura del valor base pisa el de la variación en el teclado, y
// hay que volver a poner el que toca.
export async function reassertSlot(kind, slot) {
  if (!applied) return;
  const value = applied[kind]?.[slot];
  if (!value) return;

  try {
    if (kind === 'keys')   await device.setKeymap(applied.profile, Number(slot), value.modifier, value.keycode);
    if (kind === 'rotary') await device.setRotary(applied.profile, Number(slot), value.type, value.modifier, value.keycode);
    if (kind === 'labels') await device.setLabel(applied.profile, Number(slot), value);
  } catch { /* se corrige en la siguiente evaluación */ }
}

export async function revert() {
  if (!applied) return;
  const v = applied;
  applied = null;
  if (store.connected) {
    try { await writeSlots(v, true); } catch { /* se recupera al reconectar */ }
  }
  emit('applied');
}

// Decide qué variación toca para el perfil activo y la ventana en primer plano.
export async function evaluate() {
  if (!store.connected) return;

  const target = enabled ? matchFor(store.activeProfileIdx, lastInfo) : null;
  if (target?.id === applied?.id) return;

  if (applied) await revert();
  if (target) await apply(target.id);
}

// Ejecuta algo con el perfil base intacto en el teclado y vuelve a dejar la
// variación como estaba. Lo usan «Guardar en Flash» y la edición del perfil
// base: sin esto, la Flash acabaría con los atajos de una app concreta.
export async function withBase(fn) {
  const restore = applied;
  if (restore) await revert();
  try {
    return await fn();
  } finally {
    if (restore) await apply(restore.id);
  }
}

// --- Valores efectivos ---------------------------------------------------
// Lo que hace el teclado ahora mismo = perfil base + variación seleccionada.

export function effectiveKey(prof, variant, keyIndex, layer) {
  const slot = keymapSlot(keyIndex, layer);
  return override(variant, 'keys', slot)
      ?? prof.keys[slot]
      ?? { modifier: 0, keycode: 0 };
}

export function effectiveRotary(prof, variant, baseSlot, layer) {
  const slot = rotarySlot(baseSlot, layer);
  return override(variant, 'rotary', slot)
      ?? prof.rotary?.[slot]
      ?? { type: 0, modifier: 0, keycode: 0 };
}

export function effectiveLabel(prof, variant, keyIndex, layer) {
  const slot = labelSlot(keyIndex, layer);
  if (slot < 0) return '';
  return override(variant, 'labels', slot) ?? prof.labels[slot] ?? '';
}
