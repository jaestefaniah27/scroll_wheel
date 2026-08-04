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

// Las variaciones son de un perfil Y de una página. No porque se declaren así,
// sino porque los comandos de edición del firmware no llevan número de página:
// escriben en la que el teclado tenga puesta. Una variación aplicada vive, por
// tanto, en la RAM de UNA página concreta, y hay que:
//
//   - leer los valores base de esa misma página (no de la que la app tenga
//     abierta, que puede ser otra),
//   - quitarla de esa misma página, aunque el teclado se haya ido a otra.
//
// Sin esto, aplicar y revertir copiaba los atajos de una página encima de otra,
// y el guardado a Flash los fijaba: los huecos de la variación acababan iguales
// en todas las páginas del perfil.

import * as device from './device.js';
import { state as store, labelSlot, keymapSlot, rotarySlot, pageOf,
         withPinnedPage } from './store.js';

// { id, profile, page, name, matches:[texto], field, keys:{slot:{modifier,keycode}},
//   rotary:{slot:{type,modifier,keycode}}, labels:{slot:texto} }
//
// `page` es parte de la identidad: los huecos son de una página concreta, así
// que la misma tecla en la página 2 no tiene nada que ver con la de la 1. Una
// variación solo se aplica cuando el teclado está en SU página.
//
// `matches` es una lista porque un mismo retoque suele valer para varias apps:
// los cambios que usas en el Bloc de notas también los quieres en el visor de
// texto o en cualquier otra aplicación de Windows que se comporte igual.
export const variants = [];

// Variación escrita ahora mismo en el teclado, con la página en la que se
// escribió: { v, page }. Sin la página no se puede quitar de donde está.
let applied = null;
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
  // Las variaciones anteriores a las páginas no la guardaban: son de la primera,
  // que es la única que había cuando se crearon.
  if (!Number.isInteger(v.page)) v.page = 0;
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

// Las de un perfil y una página. Sin página se devuelven todas, que es lo que
// hace falta al borrar un perfil.
export function forProfile(profileIdx, page) {
  return variants.filter((v) => v.profile === profileIdx
                             && (page === undefined || v.page === page));
}

export function get(id) {
  return variants.find((v) => v.id === id) || null;
}

export function activeVariant() {
  return applied?.v || null;
}

export function isApplied(id) {
  return applied?.v.id === id;
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

export function create(profileIdx, { page = 0, name, matches = [], field = 'any' } = {}) {
  const variant = {
    id: `v${Date.now()}${Math.random().toString(36).slice(2, 6)}`,
    profile: profileIdx,
    page,
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

// Borrar una página desplaza las siguientes (lo hace el firmware en page_remove),
// así que las variaciones que apuntaban a esas tienen que seguirlas. Las de la
// página borrada se van con ella: sus huecos ya no existen.
export function shiftPages(profileIdx, deletedIdx) {
  for (let i = variants.length - 1; i >= 0; i--) {
    const v = variants[i];
    if (v.profile !== profileIdx) continue;
    if (v.page === deletedIdx) variants.splice(i, 1);
    else if (v.page > deletedIdx) v.page--;
  }
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

// Gana la primera variación del perfil Y de la página con alguna de sus apps en
// la ventana. La página cuenta: los huecos que redefine son de esa página, y
// escribirlos en otra pisaría atajos que no tienen nada que ver.
export function matchFor(profileIdx, page, info) {
  for (const v of variants) {
    if (v.profile !== profileIdx || v.page !== page) continue;
    const hay = haystack(info, v.field);
    for (const needle of v.matches || []) {
      if (needle && hay.includes(needle)) return v;
    }
  }
  return null;
}

// Escribe los huecos de una variación, o los devuelve a su valor base.
//
// `page` es la página del teclado sobre la que se actúa, y de la que salen los
// valores base. Las dos cosas TIENEN que ser la misma página: leer la que la app
// tiene abierta (prof.keys, que es un acceso a prof.pages[prof.pageIdx]) y
// escribir en la que el teclado tenga puesta es justo lo que copiaba los atajos
// de una página encima de otra.
async function writeSlots(variant, page, useBase) {
  const prof = store.profiles[variant.profile];
  if (!prof) return;
  const base = pageOf(prof, page);
  if (!base) return;

  await withPinnedPage(variant.profile, page, async () => {
    for (const [slot, action] of Object.entries(variant.keys || {})) {
      const a = useBase ? (base.keys[slot] || { modifier: 0, keycode: 0 }) : action;
      await device.setKeymap(variant.profile, Number(slot), a.modifier, a.keycode);
    }
    for (const [slot, action] of Object.entries(variant.rotary || {})) {
      const a = useBase ? (base.rotary[slot] || { type: 0, modifier: 0, keycode: 0 }) : action;
      await device.setRotary(variant.profile, Number(slot), a.type, a.modifier, a.keycode);
    }
    for (const [slot, text] of Object.entries(variant.labels || {})) {
      const t = useBase ? (base.labels[slot] || '') : text;
      await device.setLabel(variant.profile, Number(slot), t);
    }
  });
}

// La página a la que se aplica una variación es la que el teclado tiene puesta:
// es la que está usando quien tiene esa aplicación delante.
function currentPage() {
  return store.pageIdx || 0;
}

export async function apply(id) {
  const v = get(id);
  if (!v || !store.connected) return;
  // Una variación es de un perfil y de una página concretos. Si no son los que
  // el teclado tiene puestos, no se escribe nada: sus huecos son de otra página
  // y aplicarlos aquí pisaría atajos ajenos.
  if (v.profile !== store.activeProfileIdx || v.page !== currentPage()) return;
  if (applied && applied.v.id !== v.id) await revert();

  const page = v.page;
  try {
    await writeSlots(v, page, false);
    applied = { v, page };
    emit('applied');
  } catch {
    // El teclado puede estar ocupado; se reintenta en la siguiente evaluación.
  }
}

// Reafirma un hueco de la variación aplicada. Se usa tras editar el perfil
// base: la escritura del valor base pisa el de la variación en el teclado, y
// hay que volver a poner el que toca. Solo vale si lo editado es la página
// donde está puesta la variación; en otra no hay nada suyo que pisar.
export async function reassertSlot(kind, slot) {
  if (!applied || applied.page !== currentPage()) return;
  const { v, page } = applied;
  const value = v[kind]?.[slot];
  if (!value) return;

  try {
    await withPinnedPage(v.profile, page, async () => {
      if (kind === 'keys')   await device.setKeymap(v.profile, Number(slot), value.modifier, value.keycode);
      if (kind === 'rotary') await device.setRotary(v.profile, Number(slot), value.type, value.modifier, value.keycode);
      if (kind === 'labels') await device.setLabel(v.profile, Number(slot), value);
    });
  } catch { /* se corrige en la siguiente evaluación */ }
}

// Escribe un hueco en la página donde está puesta la variación. Lo usa el
// editor: tocar una variación aplicada tiene que ir a su página, no a la que la
// app tenga abierta, que puede ser otra.
export async function writeAppliedSlot(id, kind, slot, value) {
  if (!isApplied(id) || !store.connected) return;
  const { v, page } = applied;
  await withPinnedPage(v.profile, page, async () => {
    if (kind === 'keys')   await device.setKeymap(v.profile, Number(slot), value.modifier, value.keycode);
    if (kind === 'rotary') await device.setRotary(v.profile, Number(slot), value.type, value.modifier, value.keycode);
    if (kind === 'labels') await device.setLabel(v.profile, Number(slot), value);
  });
}

// Devuelve un hueco al valor del perfil base, leído de la página donde está
// puesta la variación. Se usa al quitar una diferencia estando aplicada.
export async function revertSlot(id, kind, slot) {
  if (!isApplied(id) || !store.connected) return;
  const prof = store.profiles[applied.v.profile];
  const base = prof && pageOf(prof, applied.page);
  if (!base) return;

  const empty = kind === 'keys' ? { modifier: 0, keycode: 0 }
              : kind === 'rotary' ? { type: 0, modifier: 0, keycode: 0 } : '';
  const field = kind === 'labels' ? base.labels : base[kind];
  await writeAppliedSlot(id, kind, slot, field?.[slot] ?? empty);
}

export async function revert() {
  if (!applied) return;
  const { v, page } = applied;
  applied = null;
  // Se quita de la página en la que se escribió, aunque el teclado esté ahora en
  // otra: si se queda puesta, el guardado a Flash graba sus valores como si
  // fuesen los del perfil y la variación deja de poder deshacerse.
  //
  // De un perfil que ya no está puesto solo se alcanza su primera página (el
  // firmware no sabe apuntar a otra sin activarlo). Con la variación en otra,
  // se queda donde está hasta que ese perfil vuelva: forzar la escritura sería
  // peor, iría a la página equivocada.
  const reachable = v.profile === store.activeProfileIdx || page === 0;
  if (store.connected && reachable) {
    try { await writeSlots(v, page, true); } catch { /* se recupera al reconectar */ }
  }
  emit('applied');
}

// Decide qué variación toca para el perfil activo y la ventana en primer plano.
export async function evaluate() {
  if (!store.connected) return;

  // Solo entran las variaciones de la página que el teclado tiene puesta: al
  // cambiar de página, la anterior deja de valer y hay que quitarla.
  const target = enabled ? matchFor(store.activeProfileIdx, currentPage(), lastInfo) : null;
  if (target?.id === applied?.v.id && applied?.page === currentPage()) return;

  if (applied) await revert();
  if (target) await apply(target.id);
}

// El teclado ha cambiado de perfil o de página (EV:CTX). Lo que hubiera puesto
// se queda donde estaba, así que se quita de allí antes de decidir de nuevo.
export async function onContextChanged() {
  if (!store.connected) return;
  if (applied && (applied.page !== currentPage() || applied.v.profile !== store.activeProfileIdx)) {
    await revert();
  }
  await evaluate();
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
    if (restore) await apply(restore.v.id);
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
