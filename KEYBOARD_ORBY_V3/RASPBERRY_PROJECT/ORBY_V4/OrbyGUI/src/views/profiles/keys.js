// Escritura de lo que hace una tecla o un mando.
//
// Todo pasa por aquí para que la regla sea una sola: sin teclado conectado no
// se toca nada (requireDevice), y con una variación en edición el cambio se
// guarda como diferencia en el PC en vez de ir al perfil.

import * as device from '../../device.js';
import { state, markDirty, keymapSlot, labelSlot, rotarySlot, KEY_TO_SCREEN, syncFromDevice } from '../../store.js';
import { MACRO_MODIFIER, isTwoWayType } from '../../hid-keys.js';
import { toast, requireDevice } from '../../ui.js';
import * as cache from '../../oled-cache.js';
import * as variants from '../../variants.js';
import { OLED_FRAME_BYTES, ROTARY_TWIN } from './constants.js';
import { view, currentProfile, selectedKeyIndex, selectedRotarySlot, editingVariant,
         currentAction, rerender, rerenderKeyGrid } from './view-state.js';
import { macroById, nextMacroId, ensureMacro, savePCMacros } from './macros-store.js';

// Escribe la etiqueta de un hueco, respetando variación (igual que applyKeymap):
// se usa tanto al escribir en el campo de texto como al pegar una tecla copiada.
// Los cambios se comprueban contra el teclado ANTES de tocar el modelo de la
// app: sin él conectado no se edita nada (ver requireDevice), porque la copia
// del PC y la del Orby tienen que seguir siendo la misma.
export async function writeLabel(slot, text) {
  if (!requireDevice()) return;
  const prof = currentProfile();
  if (!prof) return;
  const variant = editingVariant();

  if (variant) {
    variants.setOverride(variant.id, 'labels', slot, text);
    rerenderKeyGrid();
    if (variants.isApplied(variant.id)) {
      try { await variants.writeAppliedSlot(variant.id, 'labels', slot, text); }
      catch { toast('El teclado no confirmó la etiqueta', 'error'); }
    }
    return;
  }

  prof.labels[slot] = text;
  try {
    await device.setLabel(view.editingProfile, slot, text);
    markDirty();
    await variants.reassertSlot('labels', slot);
    rerenderKeyGrid();
  } catch {
    toast('El teclado no confirmó la etiqueta', 'error');
  }
}

export async function applyKeymap(modifier, keycode) {
  if (!requireDevice()) return;
  const prof = currentProfile();
  const index = selectedKeyIndex();
  if (!prof || index === null) return;

  const slot = keymapSlot(index, view.layer);
  const variant = editingVariant();

  // Editando una variación no se toca el perfil ni la Flash: se guarda la
  // diferencia en el PC, y solo se escribe en el teclado si esa variación es la
  // que está puesta ahora mismo.
  if (variant) {
    variants.setOverride(variant.id, 'keys', slot, { modifier, keycode });
    rerender();
    if (variants.isApplied(variant.id)) {
      try {
        await variants.writeAppliedSlot(variant.id, 'keys', slot, { modifier, keycode });
      } catch {
        toast('No se pudo escribir la asignación', 'error');
      }
    }
    return;
  }

  prof.keys[slot] = { modifier, keycode };
  rerender();

  try {
    await device.setKeymap(view.editingProfile, slot, modifier, keycode);
    markDirty();
    // Si la variación puesta redefine este hueco, manda ella.
    await variants.reassertSlot('keys', slot);
  } catch {
    toast('No se pudo escribir la asignación', 'error');
  }
}

// Deja en negro la pantalla de la tecla seleccionada (capa actual).
//
// OLED_CLEAR no sirve para esto: solo suelta el icono propio del hueco, y
// entonces el firmware vuelve a pintar la etiqueta de texto con su marco —y en
// el perfil 0, página 1 y capa normal, directamente el mapa de bits horneado de
// fábrica (ver refresh_single_screen en main.cpp)—, así que la tecla nunca se
// apagaba. Subir un icono todo a ceros sí lo respeta en todos los casos.
//
// El icono es del hueco (perfil, página, capa, tecla), no de la variación en
// edición: esa distinción no existe para el OLED. La etiqueta sí, así que se
// borra con writeLabel, que ya sabe si toca guardarla como diferencia.
export async function blankKeyScreen() {
  if (!requireDevice()) return;
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null) return;
  const lslot = labelSlot(i, view.layer);
  if (lslot < 0) return;

  // Sin esto, quitar el icono más adelante desde el editor descubriría el texto
  // de la tecla que había antes.
  await writeLabel(lslot, '');

  try {
    const blank = new Uint8Array(OLED_FRAME_BYTES);
    await device.uploadOled(view.editingProfile, lslot, blank);
    cache.set(view.editingProfile, lslot, blank);
    prof.oledMask |= (1 << lslot);
    markDirty();
    rerender();
  } catch {
    toast('No se pudo apagar la pantalla de la tecla', 'error');
  }
}

// --- Copiar y pegar teclas ---------------------------------------------------
// Copia el icono (si tiene uno propio), la etiqueta y la acción de la tecla
// seleccionada, para poder pegarlos en cualquier otra tecla: de otra capa, de
// otra página o de otro perfil. Vive solo en memoria (no sobrevive a cerrar la
// app), como el resto de estado transitorio de este editor.
export let keyClipboard = null;

export async function copyKey() {
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null) return;

  const action = currentAction();
  const lslot = labelSlot(i, view.layer);
  const label = lslot >= 0 ? variants.effectiveLabel(prof, editingVariant(), i, view.layer) : '';
  const iconBytes = lslot >= 0 ? cache.get(view.editingProfile, lslot) : null;

  // Una secuencia no viaja por su id (lo reutilizaría, enlazando las dos
  // teclas): se lleva una copia de sus pasos para duplicarla al pegar.
  let macroActions = null;
  if (action.modifier === MACRO_MODIFIER) {
    const m = macroById(action.keycode);
    if (m) macroActions = JSON.parse(JSON.stringify(m.actions || []));
  }

  keyClipboard = {
    action: { modifier: action.modifier, keycode: action.keycode },
    label,
    icon: iconBytes ? Uint8Array.from(iconBytes) : null,
    macroActions,
  };
  toast('Tecla copiada');
  rerender();
}

export async function pasteKey() {
  if (!keyClipboard || !requireDevice()) return;
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null) return;

  let { modifier, keycode } = keyClipboard.action;
  if (modifier === MACRO_MODIFIER && keyClipboard.macroActions) {
    const id = nextMacroId();
    const m = ensureMacro(id);
    m.actions = JSON.parse(JSON.stringify(keyClipboard.macroActions));
    savePCMacros(id);
    keycode = id;
  }

  await applyKeymap(modifier, keycode);

  const lslot = labelSlot(i, view.layer);
  if (lslot < 0) { toast('Tecla pegada'); return; }

  await writeLabel(lslot, keyClipboard.label || '');

  try {
    if (keyClipboard.icon) {
      await device.uploadOled(view.editingProfile, lslot, keyClipboard.icon);
      cache.set(view.editingProfile, lslot, Uint8Array.from(keyClipboard.icon));
      prof.oledMask |= (1 << lslot);
      markDirty();
    } else if (cache.get(view.editingProfile, lslot)) {
      // La copiada no tenía icono propio: si el destino sí, se limpia para
      // que enseñe la etiqueta de texto, igual que la tecla de origen.
      await device.clearOled(view.editingProfile, lslot);
      cache.set(view.editingProfile, lslot, null);
      prof.oledMask &= ~(1 << lslot);
      markDirty();
    }
    rerender();
  } catch {
    toast('No se pudo pegar el icono', 'error');
  }

  toast('Tecla pegada');
}

// --- Reordenar teclas arrastrando --------------------------------------------
// Las teclas que se pueden mover son las que tienen pantalla: la de SUPER y la
// del menú tienen un papel fijo en el firmware y se quedan en su sitio (las
// demás saltan por encima de ellas al hacer hueco).
export const MOVABLE_KEYS = KEY_TO_SCREEN.map((screen, i) => (screen ? i : -1)).filter((i) => i >= 0);

let reordering = false;

// Lleva la tecla que está en la posición `from` de MOVABLE_KEYS a la `to`, y
// las de en medio se corren una, como los iconos del iPhone. Se mueve todo lo
// que se ve de cada tecla en la capa en pantalla: acción, etiqueta e icono.
// La otra capa no se toca: cada capa se organiza por su cuenta.
//
// Primero se cambia el modelo y se repinta (el gesto tiene que quedar donde
// se soltó, sin esperar al puerto serie); luego se escribe en el teclado solo
// lo que ha cambiado. Si el teclado no confirma algo, se vuelve a leer todo de
// él: dejar la app con un orden que el teclado no tiene sería peor.
export async function reorderKeys(from, to) {
  if (from === to || reordering) return;
  if (!requireDevice()) return;
  const prof = currentProfile();
  if (!prof) return;
  if (editingVariant()) {
    toast('Las teclas se reordenan en el perfil base, no dentro de una variación', 'error');
    rerenderKeyGrid();
    return;
  }

  const profileIdx = view.editingProfile;
  const layer = view.layer;
  const content = MOVABLE_KEYS.map((k) => {
    const lslot = labelSlot(k, layer);
    const own = Boolean(prof.oledMask & (1 << lslot));
    const bmp = cache.get(profileIdx, lslot);
    return {
      action: { ...(prof.keys[keymapSlot(k, layer)] || { modifier: 0, keycode: 0 }) },
      label: prof.labels[lslot] || '',
      icon: own && bmp ? Uint8Array.from(bmp) : null,
      own,
      loaded: !own || (cache.has(profileIdx, lslot) && bmp),
    };
  });
  // Sin el icono en la app no hay qué llevar a la tecla de destino: moverla
  // así la dejaría con la pantalla vacía.
  if (content.some((c) => !c.loaded)) {
    toast('Espera a que terminen de leerse los iconos del teclado', 'error');
    rerenderKeyGrid();
    return;
  }

  const order = content.map((_, i) => i);
  order.splice(from, 1);
  order.splice(to, 0, from);

  const changed = [];
  order.forEach((orig, pos) => {
    if (orig === pos) return;
    const k = MOVABLE_KEYS[pos];
    const c = content[orig];
    const lslot = labelSlot(k, layer);
    const kslot = keymapSlot(k, layer);
    changed.push({ kslot, lslot, c, hadOwn: content[pos].own });
    prof.keys[kslot] = { ...c.action };
    prof.labels[lslot] = c.label;
    if (c.icon) prof.oledMask |= (1 << lslot);
    else prof.oledMask &= ~(1 << lslot);
    cache.set(profileIdx, lslot, c.icon ? Uint8Array.from(c.icon) : null);
  });

  // La selección se va con la tecla, no se queda en el hueco.
  const sel = selectedKeyIndex();
  const selPos = MOVABLE_KEYS.indexOf(sel);
  if (selPos >= 0) view.selected = { kind: 'key', index: MOVABLE_KEYS[order.indexOf(selPos)] };
  rerender();

  reordering = true;
  try {
    for (const { kslot, lslot, c, hadOwn } of changed) {
      await device.setKeymap(profileIdx, kslot, c.action.modifier, c.action.keycode);
      await device.setLabel(profileIdx, lslot, c.label);
      if (c.icon) await device.uploadOled(profileIdx, lslot, c.icon);
      else if (hadOwn) await device.clearOled(profileIdx, lslot);
    }
    markDirty();
    for (const { kslot, lslot } of changed) {
      await variants.reassertSlot('keys', kslot);
      await variants.reassertSlot('labels', lslot);
    }
  } catch {
    toast('El teclado no confirmó el cambio de orden: se vuelve a leer de él', 'error');
    try { await syncFromDevice(); } catch { /* sin teclado no hay de dónde releer */ }
    rerender();
  } finally {
    reordering = false;
  }
}

// Núcleo compartido de escritura de mandos: aplica una lista de [hueco, acción]
// a la vez, respetando variación. Lo usa applyRotary (un hueco, con espejo si
// es desplazamiento) y applyRotaryPair (dos huecos con acciones distintas, ver
// más abajo).
async function writeRotarySlots(writes) {
  const prof = currentProfile();
  if (!prof) return;
  const variant = editingVariant();

  // Editando una variación se guardan diferencias en el PC; el teclado solo se
  // toca si esa variación es la que está puesta ahora mismo.
  if (variant) {
    for (const [s, a] of writes) variants.setOverride(variant.id, 'rotary', s, a);
    rerender();
    if (variants.isApplied(variant.id)) {
      try {
        for (const [s, a] of writes) {
          await variants.writeAppliedSlot(variant.id, 'rotary', s, a);
        }
      } catch {
        toast('No se pudo escribir la acción del mando', 'error');
      }
    }
    return;
  }

  for (const [s, a] of writes) prof.rotary[s] = a;

  rerender();
  try {
    for (const [s, a] of writes) {
      await device.setRotary(view.editingProfile, s, a.type, a.modifier, a.keycode);
      await variants.reassertSlot('rotary', s);
    }
    markDirty();
  } catch {
    toast('No se pudo escribir la acción del mando', 'error');
  }
}

export async function applyRotary(action) {
  if (!requireDevice()) return;
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return;

  const slot = rotarySlot(base, view.layer);

  // Los desplazamientos son bidireccionales, así que el firmware ignora la
  // acción del sentido contrario. Espejamos el par para que la interfaz no
  // enseñe una configuración que no se va a aplicar.
  const twinBase = ROTARY_TWIN[base];
  const mirror = twinBase !== undefined && isTwoWayType(action.type);
  const writes = [[slot, action]];
  if (mirror) writes.push([rotarySlot(twinBase, view.layer), { ...action }]);

  await writeRotarySlots(writes);
}

// Escribe los dos sentidos de un mismo mando a la vez, cada uno con su propia
// acción. Lo usa el giro de un complemento (ver setRotaryPluginAction en
// macro-tabs.js): los dos huecos físicos tienen que cambiar juntos, o pueden
// quedar con el mismo signo si el otro sentido no estaba configurado todavía.
export async function applyRotaryPair(slotA, actionA, slotB, actionB) {
  if (!requireDevice()) return;
  if (!currentProfile()) return;
  await writeRotarySlots([
    [rotarySlot(slotA, view.layer), actionA],
    [rotarySlot(slotB, view.layer), actionB],
  ]);
}
