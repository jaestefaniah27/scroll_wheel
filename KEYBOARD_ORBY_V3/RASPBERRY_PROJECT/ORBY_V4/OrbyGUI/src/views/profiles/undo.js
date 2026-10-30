// Deshacer en el editor de teclas y mandos.
//
// Cada toque del editor se escribe al momento en el teclado: no hay "Guardar".
// Está bien para cambiar algo a propósito, pero ir mirando qué ofrece un mando
// (pulsar un tipo de acción para ver sus opciones) ya lo cambiaba, y volver a
// como estaba obligaba a acordarse de lo que había. Aquí se apunta lo que había
// en cada hueco justo antes de tocarlo, y deshacer lo vuelve a escribir.
//
// Un paso es lo que hace un gesto del usuario (un clic, un cambio en un campo,
// soltar una tecla arrastrada), aunque escriba varias cosas: "Borrar" quita la
// acción, la etiqueta y el icono, y se deshace de una vez. Escribir una
// etiqueta letra a letra también es un paso solo.
//
// Lo que NO entra: los pasos de dentro de una secuencia (son de la macro, no
// del hueco) y lo que escribe Ajustes en todos los perfiles a la vez. Y la
// pila se tira si cambia la lista de perfiles (releer, crear, borrar): los
// índices apuntarían a otro perfil.

import * as device from '../../device.js';
import { state, markDirty, withPinnedPage } from '../../store.js';
import * as cache from '../../oled-cache.js';
import * as variants from '../../variants.js';
import { toast } from '../../ui.js';
import { view, rotaryPart, rerender } from './view-state.js';

const MAX_STEPS = 50;
const stack = [];
let gesture = 0;
let undoing = false;
const listeners = new Set();

// La llaman los manejadores de eventos del editor antes de hacer nada: lo que
// se apunte hasta el siguiente gesto va al mismo paso.
export function beginGesture() {
  gesture++;
}

export function canUndo() {
  dropStale();
  return stack.length > 0;
}

export function lastLabel() {
  return stack[stack.length - 1]?.label || '';
}

export function onChange(fn) {
  listeners.add(fn);
}

function emit() {
  for (const fn of listeners) fn();
}

// Nombre de lo que hay elegido, para el "Deshacer: …" del botón.
function selectionLabel() {
  const sel = view.selected;
  const layer = view.layer === 'super' ? ' (SUPER)' : '';
  if (sel?.kind === 'key') return `Tecla ${sel.index + 1}${layer}`;
  if (sel?.kind === 'rotary') {
    const found = rotaryPart(sel.slot);
    return found ? `${found.group.name} · ${found.part.label}${layer}` : `Mando${layer}`;
  }
  return 'el último cambio';
}

// Apunta lo que había en un hueco antes de escribirlo. `kind`: 'keys',
// 'rotary', 'labels' o 'icon'. `before` es el valor del modelo: para una
// variación, su diferencia (null si no redefinía el hueco); para un icono,
// { bytes, own }. Con `after`, si el valor no cambia no se apunta nada: pulsar
// el tipo de acción que ya estaba puesto no es un paso que deshacer.
export function record(kind, slot, before, { label = null, after } = {}) {
  if (undoing) return;
  if (after !== undefined && JSON.stringify(before ?? null) === JSON.stringify(after ?? null)) return;
  const profile = view.editingProfile;
  const prof = state.profiles[profile];
  if (!prof) return;
  const page = prof.pageIdx || 0;
  // El icono es del hueco, no de la variación (ver blankKeyScreen en keys.js):
  // va en el mismo paso, pero al deshacerlo se escribe en el perfil.
  const variantId = view.variantId && variants.get(view.variantId) ? view.variantId : null;

  let step = stack[stack.length - 1];
  const same = step && step.prof === prof && step.page === page && step.variantId === variantId;
  // Una etiqueta que se sigue escribiendo es el mismo paso aunque cada letra
  // llegue como un gesto nuevo.
  const typing = same && kind === 'labels' && step.writes.length === 1
    && step.writes[0].kind === 'labels' && step.writes[0].slot === slot;
  if (!(same && (step.gesture === gesture || typing))) {
    step = {
      gesture, prof, profile, page, variantId, writes: [],
      label: label || selectionLabel(),
      selected: view.selected ? { ...view.selected } : null,
      layer: view.layer, panelNav: view.panelNav ?? null, tab: view.tab,
    };
    stack.push(step);
    if (stack.length > MAX_STEPS) stack.shift();
  }
  // Del mismo hueco dentro de un paso vale lo primero que había, no lo que
  // dejó la escritura de en medio.
  if (step.writes.some((w) => w.kind === kind && w.slot === slot)) return;
  step.writes.push({ kind, slot, before: clone(before) });
  emit();
}

function clone(v) {
  if (v == null) return null;
  if (v instanceof Uint8Array) return Uint8Array.from(v);
  if (v.bytes !== undefined) return { bytes: v.bytes ? Uint8Array.from(v.bytes) : null, own: v.own };
  return typeof v === 'object' ? { ...v } : v;
}

// Pasos que ya no apuntan a nada: el perfil se ha releído o ya no existe.
function dropStale() {
  for (let i = stack.length - 1; i >= 0; i--) {
    const s = stack[i];
    if (state.profiles[s.profile] !== s.prof) stack.splice(i, 1);
  }
}

export async function undo() {
  dropStale();
  const step = stack.pop();
  emit();
  if (!step) return;
  if (!state.connected) {
    stack.push(step);
    toast('Conecta el Orvy para deshacer', 'error');
    return;
  }

  undoing = true;
  try {
    // Al revés de como se escribieron, por si dos escrituras dependían.
    const writes = [...step.writes].reverse();
    const icons = writes.filter((w) => w.kind === 'icon');
    if (step.variantId) {
      await undoVariant(step, writes.filter((w) => w.kind !== 'icon'));
      if (icons.length) await withPinnedPage(step.profile, step.page, () => undoBase(step, icons));
    } else {
      await withPinnedPage(step.profile, step.page, () => undoBase(step, writes));
    }
    markDirty();
    // De vuelta a lo que se tocó, para que se vea qué ha vuelto.
    view.editingProfile = step.profile;
    view.variantId = step.variantId;
    view.layer = step.layer;
    view.selected = step.selected;
    view.panelNav = step.panelNav;
    view.tab = step.tab;
    toast(`Deshecho: ${step.label}`);
  } catch {
    toast('El teclado no confirmó el deshacer', 'error');
  } finally {
    undoing = false;
    rerender();
    emit();
  }
}

async function undoBase(step, writes) {
  const pg = step.prof.pages[step.page];
  if (!pg) return;
  for (const { kind, slot, before } of writes) {
    if (kind === 'keys') {
      pg.keys[slot] = { ...before };
      await device.setKeymap(step.profile, slot, before.modifier, before.keycode);
    } else if (kind === 'rotary') {
      pg.rotary[slot] = { ...before };
      await device.setRotary(step.profile, slot, before.type, before.modifier, before.keycode);
    } else if (kind === 'labels') {
      pg.labels[slot] = before || '';
      await device.setLabel(step.profile, slot, before || '');
    } else if (kind === 'icon') {
      if (before.own && before.bytes) {
        await device.uploadOled(step.profile, slot, before.bytes);
        pg.oledMask |= (1 << slot);
      } else {
        await device.clearOled(step.profile, slot);
        pg.oledMask &= ~(1 << slot);
      }
      cache.set(step.profile, slot, before.own ? before.bytes : null, step.page);
    }
    // Si la variación puesta redefine el hueco, sigue mandando ella.
    if (kind !== 'icon') await variants.reassertSlot(kind, slot);
  }
}

async function undoVariant(step, writes) {
  const id = step.variantId;
  for (const { kind, slot, before } of writes) {
    if (before == null) {
      variants.clearOverride(id, kind, slot);
      if (variants.isApplied(id)) await variants.revertSlot(id, kind, slot);
    } else {
      variants.setOverride(id, kind, slot, before);
      if (variants.isApplied(id)) await variants.writeAppliedSlot(id, kind, slot, before);
    }
  }
}
