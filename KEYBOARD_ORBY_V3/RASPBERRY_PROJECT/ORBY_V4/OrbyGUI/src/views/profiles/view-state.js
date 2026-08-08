// Estado de la vista (qué perfil, capa, variación y hueco se están editando) y
// las lecturas derivadas de él.
//
// Aquí vive también el registro de repintado: los módulos que escriben algo
// necesitan repintar, pero render() lo monta profiles.js, que a su vez los
// importa a ellos. En vez de un ciclo de imports, profiles.js registra sus
// funciones al arrancar (setRenderers) y todos llaman a rerender().

import { state, scrollFor } from '../../store.js';
import * as variants from '../../variants.js';
import { MACRO_MODIFIER } from '../../hid-keys.js';
import { ROTARY_GROUPS, WHEEL_GROUP } from './constants.js';

export let renderers = { render: () => {}, renderKeyGrid: () => {} };

// La llama profiles.js en su init(), antes de que nada pueda repintar.
export function setRenderers(fns) {
  renderers = { ...renderers, ...fns };
}

export function rerender() {
  renderers.render();
}

export function rerenderKeyGrid() {
  renderers.renderKeyGrid();
}

export const view = {
  editingProfile: 0,
  layer: 'normal',
  variantId: null,  // null = perfil base; si no, la variación que se edita
  selected: null,   // { kind: 'key', index } | { kind: 'rotary', slot }
  tab: 'shortcut',  // pestaña del inspector de tecla: shortcut | sequence | text | record | app | media | pages
  capturing: false, // false | true (atajo) | 'sequence' (grabando una macro) | 'position' (capturando dónde poner el ratón)
  busy: false,      // hay un alta/baja de perfil en marcha
};

// Página del perfil que se está editando en esta vista.
export function currentPageIdx() {
  const prof = state.profiles[view.editingProfile];
  return prof ? (prof.pageIdx || 0) : 0;
}

// Variación que se está editando. Editar el perfil base es lo normal; al elegir
// una variación, cada cambio se guarda como diferencia respecto a ese base.
//
// Una variación es de una página concreta: si se está mirando otra, no pinta
// nada aquí (sus huecos son de la suya).
export function editingVariant() {
  const v = view.variantId ? variants.get(view.variantId) : null;
  if (!v) return null;
  return (v.profile === view.editingProfile && v.page === currentPageIdx()) ? v : null;
}

export function selectedKeyIndex() {
  return view.selected?.kind === 'key' ? view.selected.index : null;
}

export function selectedRotarySlot() {
  return view.selected?.kind === 'rotary' ? view.selected.slot : null;
}

export function rotaryPart(baseSlot) {
  for (const g of [...ROTARY_GROUPS, WHEEL_GROUP]) {
    const p = g.parts.find((x) => x.slot === baseSlot);
    if (p) return { group: g, part: p };
  }
  return null;
}

export function currentProfile() {
  return state.profiles[view.editingProfile] || null;
}

export function isActiveView() {
  return document.getElementById('view-profiles')?.classList.contains('active');
}

// Lo que hace la tecla o el mando con lo que hay seleccionado: el perfil base,
// o el base con la variación encima si se está editando una.
export function currentAction() {
  const prof = currentProfile();
  const index = selectedKeyIndex();
  if (!prof || index === null) return { modifier: 0, keycode: 0 };
  return variants.effectiveKey(prof, editingVariant(), index, view.layer);
}

export function currentRotary() {
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return { type: 0, modifier: 0, keycode: 0 };
  return variants.effectiveRotary(prof, editingVariant(), base, view.layer);
}

export function currentScroll() {
  return scrollFor(currentProfile(), view.layer);
}

// Macro asignada al hueco seleccionado (tecla o mando), o null si no lo es.
export function currentMacroId() {
  const a = view.selected?.kind === 'rotary' ? currentRotary() : currentAction();
  return a.modifier === MACRO_MODIFIER ? a.keycode : null;
}
