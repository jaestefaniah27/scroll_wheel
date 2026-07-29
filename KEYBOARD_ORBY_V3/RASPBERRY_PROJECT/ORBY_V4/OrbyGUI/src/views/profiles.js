// Editor de perfiles.
//
// Los datos vienen del firmware (GET_PROFILE) y cada cambio se escribe con
// SET_LABEL / SET_KEYMAP / SET_ROTARY / SET_PSCROLL / SET_NAME. Los cambios
// viven en RAM del microcontrolador hasta que se pulsa "Guardar en Flash", igual
// que en el menú físico del teclado.
//
// La réplica del teclado es la misma que la del editor de iconos —cada tecla
// enseña su icono real— para no tener dos representaciones distintas de lo
// mismo. El botón "Editar icono" salta al editor con esa tecla ya elegida.
//
// El interruptor NORMAL/SUPER afecta a todo el perfil: teclas, mandos giratorios
// y rueda de scroll. Cada capa guarda sus propias acciones, así que un mismo
// perfil puede hacer dos cosas distintas según se mantenga SUPER o no.

import * as device from '../device.js';
import { state, notify, markDirty, subscribe, syncFromDevice, profileMeta, labelSlot, keymapSlot,
         rotarySlot, layerIndex, scrollFor, KEY_TO_SCREEN } from '../store.js';
import { MODIFIERS, CONSUMER_MODIFIER, CONSUMER_ACTIONS, KEY_GROUPS, describeAction, eventToAction,
         ROTARY_TYPES, ROTARY_SLOTS, isScrollType, describeRotary } from '../hid-keys.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';
import { goTo } from '../nav.js';
import * as cache from '../oled-cache.js';
import * as wheelDial from '../wheel-dial.js';
import * as variants from '../variants.js';
import * as dashboard from './dashboard.js';

// Mandos giratorios agrupados como se ven en el teclado.
const ROTARY_GROUPS = [
  {
    name: 'Encoder izquierdo', icon: 'reset',
    parts: [
      { slot: ROTARY_SLOTS.ENC1_CW,    label: 'Giro horario',     short: '↻' },
      { slot: ROTARY_SLOTS.ENC1_CCW,   label: 'Giro antihorario', short: '↺' },
      { slot: ROTARY_SLOTS.ENC1_CLICK, label: 'Pulsación',        short: '⏺', discrete: true },
    ],
  },
  {
    name: 'Rueda de scroll', icon: 'wheel',
    parts: [
      { slot: ROTARY_SLOTS.WHEEL_CW,  label: 'Hacia abajo', short: '↓' },
      { slot: ROTARY_SLOTS.WHEEL_CCW, label: 'Hacia arriba', short: '↑' },
    ],
  },
  {
    name: 'Encoder derecho', icon: 'reset',
    parts: [
      { slot: ROTARY_SLOTS.ENC2_CW,    label: 'Giro horario',     short: '↻' },
      { slot: ROTARY_SLOTS.ENC2_CCW,   label: 'Giro antihorario', short: '↺' },
      { slot: ROTARY_SLOTS.ENC2_CLICK, label: 'Pulsación',        short: '⏺', discrete: true },
    ],
  },
];

// Tipos ofrecidos en el inspector. Los de desplazamiento no tienen sentido en
// una pulsación, que es un evento suelto sin dirección.
const ROTARY_TYPE_OPTIONS = [
  { type: ROTARY_TYPES.NONE,     label: 'Nada' },
  { type: ROTARY_TYPES.CONSUMER, label: 'Multimedia / sistema' },
  { type: ROTARY_TYPES.KEY,      label: 'Atajo de teclado' },
  { type: ROTARY_TYPES.SCROLL_V, label: 'Desplazar vertical', turnOnly: true },
  { type: ROTARY_TYPES.SCROLL_H, label: 'Desplazar horizontal', turnOnly: true },
  { type: ROTARY_TYPES.ZOOM,     label: 'Zoom (Ctrl + rueda)', turnOnly: true },
];

const SCROLL_PRESETS = [
  { value: 12,  name: 'Preciso',  desc: 'Timeline, PCB, edición fina' },
  { value: 30,  name: 'Suave',    desc: 'Lectura y navegación lenta' },
  { value: 60,  name: 'Estándar', desc: 'Equivalente a rueda de ratón' },
  { value: 120, name: 'Rápido',   desc: 'Documentos y logs largos' },
];

const view = {
  editingProfile: 0,
  layer: 'normal',
  variantId: null,  // null = perfil base; si no, la variación que se edita
  selected: null,   // { kind: 'key', index } | { kind: 'rotary', slot }
  capturing: false,
  busy: false,      // hay un alta/baja de perfil en marcha
};

// Variación que se está editando. Editar el perfil base es lo normal; al elegir
// una variación, cada cambio se guarda como diferencia respecto a ese base.
function editingVariant() {
  return view.variantId ? variants.get(view.variantId) : null;
}

// Giro acumulado de la rueda para la prueba en vivo.
let liveTotal = 0;
let liveDecay = null;

function selectedKeyIndex() {
  return view.selected?.kind === 'key' ? view.selected.index : null;
}

function selectedRotarySlot() {
  return view.selected?.kind === 'rotary' ? view.selected.slot : null;
}

function rotaryPart(baseSlot) {
  for (const g of ROTARY_GROUPS) {
    const p = g.parts.find((x) => x.slot === baseSlot);
    if (p) return { group: g, part: p };
  }
  return null;
}

function currentProfile() {
  return state.profiles[view.editingProfile] || null;
}

function isActiveView() {
  return document.getElementById('view-profiles')?.classList.contains('active');
}

export function init() {
  const body = document.getElementById('profiles-body');
  body.addEventListener('click', onClick);
  body.addEventListener('change', onChange);
  body.addEventListener('input', onInput);
  window.addEventListener('keydown', onCapture, true);

  // Las miniaturas llegan por el puerto serie con retraso: cuando la caché se
  // completa hay que repintar la rejilla, no la vista entera.
  cache.onChange(() => { if (isActiveView()) renderKeyGrid(); });

  device.on('telemetry', (line) => {
    if (line.startsWith('WHEEL:') && isActiveView()) onWheelTelemetry(parseInt(line.slice(6), 10));
  });

  // El dibujo de la rueda lo lleva wheel-dial.js con el ángulo absoluto del
  // sensor, así que la aguja se queda donde esté la rueda de verdad.
  wheelDial.onUpdate(paintDialMarker);

  // Cuando el detector de aplicaciones aplica o quita una variación hay que
  // reflejarlo aquí (el distintivo de "aplicada ahora").
  variants.onChange((reason) => {
    if (reason === 'applied' && isActiveView()) render();
  });

  // Repintado solo cuando cambia algo que se ve aquí: si repintásemos con cada
  // notificación, escribir un nombre o una etiqueta perdería el foco al vuelo,
  // porque cada escritura marca la configuración como pendiente de guardar.
  let lastCount = -1;
  let lastActive = -1;
  subscribe(() => {
    if (state.profiles.length === lastCount && state.activeProfileIdx === lastActive) return;
    lastCount = state.profiles.length;
    lastActive = state.activeProfileIdx;
    if (isActiveView()) render();
  });
}

// --- Interacción -----------------------------------------------------------

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;
  const act = el.dataset.act;

  if (act === 'pick-profile') {
    view.editingProfile = Number(el.dataset.idx);
    view.variantId = null;
    view.selected = null;
    render();
  } else if (act === 'pick-variant') {
    view.variantId = el.dataset.id || null;
    view.selected = null;
    render();
  } else if (act === 'new-variant') {
    createVariant();
  } else if (act === 'del-variant') {
    removeVariant();
  } else if (act === 'clear-override') {
    clearOverrideOfSelection();
  } else if (act === 'layer') {
    view.layer = el.dataset.layer;
    view.selected = null;
    render();
  } else if (act === 'pick-key') {
    view.selected = { kind: 'key', index: Number(el.dataset.key) };
    render();
  } else if (act === 'pick-rotary') {
    view.selected = { kind: 'rotary', slot: Number(el.dataset.slot) };
    render();
  } else if (act === 'edit-icon') {
    // Salta al editor de iconos con este perfil, esta tecla y esta capa.
    goTo('view-oled', { profile: view.editingProfile, key: Number(el.dataset.key), layer: view.layer });
  } else if (act === 'rotary-type') {
    applyRotary({ type: Number(el.dataset.type), modifier: 0, keycode: 0 });
  } else if (act === 'rotary-consumer') {
    applyRotary({ type: ROTARY_TYPES.CONSUMER, modifier: 0, keycode: Number(el.dataset.index) });
  } else if (act === 'rotary-mod') {
    const cur = currentRotary();
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: cur.modifier ^ Number(el.dataset.bit), keycode: cur.keycode });
  } else if (act === 'activate') {
    device.setProfile(view.editingProfile)
      .then(() => {
        state.activeProfileIdx = view.editingProfile;
        notify();
        render();
      })
      .catch(() => toast('El teclado no confirmó el cambio de perfil', 'error'));
  } else if (act === 'capture') {
    view.capturing = !view.capturing;
    render();
  } else if (act === 'clear-action') {
    applyKeymap(0, 0);
  } else if (act === 'set-consumer') {
    applyKeymap(CONSUMER_MODIFIER, Number(el.dataset.index));
  } else if (act === 'toggle-mod') {
    const bit = Number(el.dataset.bit);
    const current = currentAction();
    if (current.modifier === CONSUMER_MODIFIER) applyKeymap(bit, 0);
    else applyKeymap(current.modifier ^ bit, current.keycode);
  } else if (act === 'profile-new') {
    createProfile(null);
  } else if (act === 'profile-dup') {
    createProfile(view.editingProfile);
  } else if (act === 'profile-del') {
    removeProfile();
  } else if (act === 'scroll-preset') {
    applyScroll({ detentsPerRev: Number(el.dataset.value) });
  } else if (act === 'scroll-invert') {
    applyScroll({ invert: !currentScroll().invert });
  } else if (act === 'dial-marker') {
    applyDialSetting({ marker: el.dataset.marker });
  } else if (act === 'dial-invert') {
    applyDialSetting({ invert: !wheelDial.dial.invert });
  } else if (act === 'dial-nudge') {
    applyDialSetting({ offsetDeg: wheelDial.normalize(wheelDial.dial.offsetDeg + Number(el.dataset.d)) });
  } else if (act === 'dial-align') {
    wheelDial.alignHere();
    render();
    dashboard.refreshMarker();
  }
}

function onChange(e) {
  const act = e.target.dataset.act;
  if (act === 'pick-keycode') {
    const current = currentAction();
    const mod = current.modifier === CONSUMER_MODIFIER ? 0 : current.modifier;
    applyKeymap(mod, Number(e.target.value));
  } else if (act === 'rotary-keycode') {
    const cur = currentRotary();
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: cur.modifier, keycode: Number(e.target.value) });
  } else if (act === 'scroll-slider') {
    applyScroll({ detentsPerRev: Number(e.target.value) });
  } else if (act === 'variant-field') {
    variants.update(view.variantId, { field: e.target.value });
  } else if (act === 'variant-name' || act === 'variant-match') {
    // `change` salta al salir del campo: repintar aquí no corta la escritura.
    variants.update(view.variantId, {
      [act === 'variant-name' ? 'name' : 'match']: e.target.value.trim(),
    });
    render();
  }
}

let labelDebounce = null;
function onInput(e) {
  const act = e.target.dataset.act;

  if (act === 'edit-label') {
    const slot = Number(e.target.dataset.slot);
    const text = e.target.value.slice(0, 7);
    clearTimeout(labelDebounce);
    labelDebounce = setTimeout(async () => {
      const prof = currentProfile();
      if (!prof) return;
      const variant = editingVariant();

      if (variant) {
        variants.setOverride(variant.id, 'labels', slot, text);
        renderKeyGrid();
        if (variants.isApplied(variant.id)) {
          try { await device.setLabel(view.editingProfile, slot, text); }
          catch { toast('El teclado no confirmó la etiqueta', 'error'); }
        }
        return;
      }

      prof.labels[slot] = text;
      try {
        await device.setLabel(view.editingProfile, slot, text);
        markDirty();
        await variants.reassertSlot('labels', slot);
        renderKeyGrid();
      } catch {
        toast('El teclado no confirmó la etiqueta', 'error');
      }
    }, 250);

  } else if (act === 'edit-name') {
    const text = e.target.value.slice(0, 7);
    clearTimeout(labelDebounce);
    labelDebounce = setTimeout(async () => {
      const prof = currentProfile();
      if (!prof) return;
      prof.name = text;
      try {
        await device.setName(view.editingProfile, text);
        markDirty();
        notify();
        renderTabs();
      } catch {
        toast('El teclado no confirmó el nombre', 'error');
      }
    }, 250);

  } else if (act === 'scroll-slider') {
    // Deslizar solo actualiza el número: el comando se manda al soltar, para no
    // inundar el CDC con un SET_PSCROLL por cada píxel.
    const val = Number(e.target.value);
    const readout = document.getElementById('scroll-value');
    if (readout) readout.textContent = val;
    updateDerived(val);

  } else if (act === 'dial-offset') {
    // El desfase se ve al momento en la aguja, sin repintar la vista entera.
    const deg = Number(e.target.value);
    wheelDial.setDial({ offsetDeg: deg });
    const label = document.getElementById('dial-offset-val');
    if (label) label.textContent = `${deg}°`;
  }
}

// Captura un atajo pulsándolo físicamente en el teclado del PC.
function onCapture(e) {
  if (!view.capturing || !view.selected) return;
  e.preventDefault();
  e.stopPropagation();

  if (e.key === 'Escape') { view.capturing = false; render(); return; }

  const action = eventToAction(e);
  if (!action.keycode) return; // solo modificadores: seguimos esperando

  view.capturing = false;
  if (view.selected.kind === 'rotary') {
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: action.modifier, keycode: action.keycode });
  } else {
    applyKeymap(action.modifier, action.keycode);
  }
}

// --- Variaciones ------------------------------------------------------------

// Se propone la app que tengas delante: es casi siempre para la que quieres la
// variación, y ahorra escribir el nombre del ejecutable a mano.
async function createVariant() {
  let match = '';
  try {
    const info = await window.orby.foreground.current();
    match = (info?.process || '').toLowerCase();
  } catch { /* sin detector: se rellena a mano */ }

  const variant = variants.create(view.editingProfile, {
    name: match ? match.replace(/\.exe$/, '') : 'Variación',
    match,
  });
  view.variantId = variant.id;
  view.selected = null;
  render();
  toast(match ? `Variación creada para "${match}"` : 'Variación creada: indícale a qué app se aplica');
}

async function removeVariant() {
  const variant = editingVariant();
  if (!variant) return;
  if (!confirm(`Se borrará la variación "${variant.name}" y sus ${variants.countOverrides(variant)} cambios.\n\n`
             + 'El perfil base no se toca.\n\n¿Continuar?')) return;

  await variants.remove(variant.id);
  view.variantId = null;
  view.selected = null;
  render();
  toast('Variación eliminada');
}

// Devuelve la tecla o el mando seleccionados al valor del perfil base.
async function clearOverrideOfSelection() {
  const variant = editingVariant();
  if (!variant || !view.selected) return;

  const kind = view.selected.kind === 'rotary' ? 'rotary' : 'keys';
  const slot = view.selected.kind === 'rotary'
    ? rotarySlot(view.selected.slot, view.layer)
    : keymapSlot(view.selected.index, view.layer);

  variants.clearOverride(variant.id, kind, slot);
  if (view.selected.kind === 'key') {
    variants.clearOverride(variant.id, 'labels', labelSlot(view.selected.index, view.layer));
  }

  // Si la variación está puesta, el teclado tiene que volver al valor base.
  if (variants.isApplied(variant.id)) {
    const prof = currentProfile();
    try {
      if (kind === 'keys') {
        const a = prof.keys[slot] || { modifier: 0, keycode: 0 };
        await device.setKeymap(view.editingProfile, slot, a.modifier, a.keycode);
        const ls = labelSlot(view.selected.index, view.layer);
        if (ls >= 0) await device.setLabel(view.editingProfile, ls, prof.labels[ls] || '');
      } else {
        const a = prof.rotary[slot] || { type: 0, modifier: 0, keycode: 0 };
        await device.setRotary(view.editingProfile, slot, a.type, a.modifier, a.keycode);
      }
    } catch {
      toast('El teclado no confirmó la vuelta al valor base', 'error');
    }
  }
  render();
}

// --- Alta y baja de perfiles ------------------------------------------------

async function createProfile(copyFrom) {
  if (view.busy) return;
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (state.profiles.length >= state.maxProfiles) {
    toast(`El teclado admite como máximo ${state.maxProfiles} perfiles`, 'error');
    return;
  }

  view.busy = true;
  render();
  try {
    const idx = copyFrom === null ? await device.addProfile() : await device.dupProfile(copyFrom);
    // Los índices del banco de iconos se desplazan con la lista, así que lo
    // leído deja de valer.
    cache.invalidate();
    await syncFromDevice();
    view.editingProfile = Number.isInteger(idx) ? idx : state.profiles.length - 1;
    view.selected = null;
    markDirty();
    toast(copyFrom === null ? 'Perfil creado' : 'Perfil duplicado');
  } catch (err) {
    toast(`No se pudo crear el perfil: ${err.message}`, 'error');
  } finally {
    view.busy = false;
    render();
  }
}

async function removeProfile() {
  if (view.busy) return;
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (state.profiles.length <= 1) { toast('Tiene que quedar al menos un perfil', 'error'); return; }

  const prof = currentProfile();
  if (!confirm(`Se borrará el perfil "${prof?.name || ''}" con sus atajos, mandos e iconos.\n\n`
             + 'El cambio no será permanente hasta que pulses "Guardar en Flash".\n\n¿Continuar?')) return;

  view.busy = true;
  render();
  try {
    await device.delProfile(view.editingProfile);
    // Los índices de los perfiles siguientes bajan uno: las variaciones tienen
    // que seguirlos, y las del perfil borrado se van con él.
    variants.shiftProfiles(view.editingProfile);
    view.variantId = null;
    cache.invalidate();
    await syncFromDevice();
    view.editingProfile = Math.min(view.editingProfile, state.profiles.length - 1);
    view.selected = null;
    markDirty();
    toast('Perfil eliminado');
  } catch (err) {
    toast(`No se pudo eliminar: ${err.message}`, 'error');
  } finally {
    view.busy = false;
    render();
  }
}

// --- Escritura de acciones --------------------------------------------------

// Lo que hace la tecla o el mando con lo que hay seleccionado: el perfil base,
// o el base con la variación encima si se está editando una.
function currentAction() {
  const prof = currentProfile();
  const index = selectedKeyIndex();
  if (!prof || index === null) return { modifier: 0, keycode: 0 };
  return variants.effectiveKey(prof, editingVariant(), index, view.layer);
}

function currentRotary() {
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return { type: 0, modifier: 0, keycode: 0 };
  return variants.effectiveRotary(prof, editingVariant(), base, view.layer);
}

function currentScroll() {
  return scrollFor(currentProfile(), view.layer);
}

async function applyKeymap(modifier, keycode) {
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
    render();
    if (variants.isApplied(variant.id)) {
      try {
        await device.setKeymap(view.editingProfile, slot, modifier, keycode);
      } catch {
        toast('No se pudo escribir la asignación', 'error');
      }
    }
    return;
  }

  prof.keys[slot] = { modifier, keycode };
  render();

  try {
    await device.setKeymap(view.editingProfile, slot, modifier, keycode);
    markDirty();
    // Si la variación puesta redefine este hueco, manda ella.
    await variants.reassertSlot('keys', slot);
  } catch {
    toast('No se pudo escribir la asignación', 'error');
  }
}

async function applyRotary(action) {
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return;

  const slot = rotarySlot(base, view.layer);
  const variant = editingVariant();

  // Los desplazamientos son bidireccionales, así que el firmware ignora la
  // acción del sentido contrario. Espejamos el par para que la interfaz no
  // enseñe una configuración que no se va a aplicar.
  const twinBase = { [ROTARY_SLOTS.ENC1_CW]: ROTARY_SLOTS.ENC1_CCW,
                     [ROTARY_SLOTS.ENC1_CCW]: ROTARY_SLOTS.ENC1_CW,
                     [ROTARY_SLOTS.ENC2_CW]: ROTARY_SLOTS.ENC2_CCW,
                     [ROTARY_SLOTS.ENC2_CCW]: ROTARY_SLOTS.ENC2_CW,
                     [ROTARY_SLOTS.WHEEL_CW]: ROTARY_SLOTS.WHEEL_CCW,
                     [ROTARY_SLOTS.WHEEL_CCW]: ROTARY_SLOTS.WHEEL_CW }[base];
  const mirror = twinBase !== undefined && isScrollType(action.type);
  const writes = [[slot, action]];
  if (mirror) writes.push([rotarySlot(twinBase, view.layer), { ...action }]);

  // Editando una variación se guardan diferencias en el PC; el teclado solo se
  // toca si esa variación es la que está puesta ahora mismo.
  if (variant) {
    for (const [s, a] of writes) variants.setOverride(variant.id, 'rotary', s, a);
    render();
    if (variants.isApplied(variant.id)) {
      try {
        for (const [s, a] of writes) {
          await device.setRotary(view.editingProfile, s, a.type, a.modifier, a.keycode);
        }
      } catch {
        toast('No se pudo escribir la acción del mando', 'error');
      }
    }
    return;
  }

  for (const [s, a] of writes) prof.rotary[s] = a;

  render();
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

// Sensibilidad e inversión de la rueda: un ajuste más del perfil y de la capa.
async function applyScroll(patch) {
  const prof = currentProfile();
  if (!prof) return;

  const li = layerIndex(view.layer);
  const next = { ...scrollFor(prof, view.layer), ...patch };
  prof.scroll[li] = next;

  // Si es lo que el teclado está usando ahora mismo, el resto de la app tiene
  // que enterarse (el dashboard enseña la sensibilidad en vivo).
  if (view.editingProfile === state.activeProfileIdx && li === (state.superActive ? 1 : 0)) {
    state.scroll = { ...state.scroll, ...next };
  }

  render();
  try {
    await device.setProfileScroll(view.editingProfile, li, next.detentsPerRev, next.invert);
    markDirty();
  } catch {
    toast('El teclado no confirmó la calibración de la rueda', 'error');
  }
}

function paintDialMarker(deg) {
  const marker = document.getElementById('scroll-live-needle');
  if (marker) marker.style.transform = `rotate(${deg}deg)`;
}

function onWheelTelemetry(delta) {
  if (!Number.isFinite(delta)) return;
  liveTotal += delta;

  const readout = document.getElementById('scroll-live-readout');
  if (!readout) return;

  const detents = (liveTotal * currentScroll().detentsPerRev) / 4096;
  readout.textContent =
    `Rueda en ${wheelDial.normalize(wheelDial.angle()).toFixed(0)}°  ·  `
    + `${detents >= 0 ? '+' : ''}${detents.toFixed(2)} clics en este giro`;

  clearTimeout(liveDecay);
  liveDecay = setTimeout(() => { liveTotal = 0; }, 1200);
}

// Ajustes de cómo se dibuja la rueda: sentido de giro, desfase del marcador de
// la tapa y forma. Son de montaje, así que se guardan en el PC.
function applyDialSetting(patch) {
  wheelDial.setDial(patch);
  render();
  // El dashboard tiene el mismo marcador y también hay que rehacerlo.
  dashboard.refreshMarker();
}

// --- Render ----------------------------------------------------------------

export function render() {
  const body = document.getElementById('profiles-body');
  if (!body) return;

  if (!state.profiles.length) {
    body.innerHTML = `<div class="empty-panel glass-panel">
      ${icon('plug', 40)}
      <h3>Sin perfiles cargados</h3>
      <p>Conecta el Orby: los perfiles se leen directamente del firmware.</p>
    </div>`;
    return;
  }

  if (view.editingProfile >= state.profiles.length) view.editingProfile = 0;
  const prof = currentProfile();

  body.innerHTML = `
    <div class="profile-bar" id="profile-bar">${renderTabsInner()}</div>
    ${renderVariantBar()}

    <div class="editor-layout">
      <div class="editor-main">
        ${renderVariantSettings()}
        <div class="editor-board glass-panel">
          <div class="editor-board-head">
            <label class="field-inline">
              <span>Nombre</span>
              <input type="text" maxlength="7" value="${escape(prof.name)}"
                     data-act="edit-name" class="text-input compact">
            </label>
            <div class="layer-toggle">
              <button class="toggle-btn ${view.layer === 'normal' ? 'active' : ''}" data-act="layer" data-layer="normal">NORMAL</button>
              <button class="toggle-btn ${view.layer === 'super' ? 'active' : ''}" data-act="layer" data-layer="super">SUPER</button>
            </div>
            ${view.editingProfile === state.activeProfileIdx
              ? '<span class="pill pill-live">Perfil activo</span>'
              : '<button class="secondary-btn" data-act="activate">Activar en el teclado</button>'}
          </div>

          <div class="okey-grid" id="profile-key-grid">${renderKeyGridInner()}</div>
          <p class="grid-status" id="profile-grid-status"></p>
        </div>

        <div class="glass-panel oled-card">
          <div class="card-header">${icon('reset', 20)}<h2>Mandos giratorios</h2></div>
          <div class="rotary-groups">${renderRotaryGroups()}</div>
          <p class="setting-desc mt-4">
            Cada capa guarda sus propias acciones: con <strong>SUPER</strong> mantenida los encoders
            y la rueda hacen lo que configures aquí en la capa SUPER. Dentro del menú del teclado
            los encoders siguen sirviendo para navegar.
          </p>
        </div>

        ${renderWheelCard()}
      </div>

      ${renderInspector()}
    </div>`;

  // Los <canvas> de las miniaturas no se pueden serializar en la plantilla: se
  // pintan después de insertar el HTML, tanto en la rejilla como en el inspector.
  cache.paintThumbs(body);
  paintKeyGrid();
  updateDerived(currentScroll().detentsPerRev);
  paintDialMarker(wheelDial.angle());
  cache.loadProfile(view.editingProfile);
}

function renderTabsInner() {
  const full = state.profiles.length >= state.maxProfiles;
  return `
    <div class="profile-tabs">
      ${state.profiles.map((p, i) => `
        <button class="profile-tab ${i === view.editingProfile ? 'active' : ''} ${i === state.activeProfileIdx ? 'is-live' : ''}"
                data-act="pick-profile" data-idx="${i}">
          <span class="tab-icon" style="--accent:${profileMeta(i).accent}">${icon(profileMeta(i).icon, 18)}</span>
          <span class="tab-name">${escape(p.name)}</span>
          ${i === state.activeProfileIdx ? '<span class="tab-live">EN USO</span>' : ''}
        </button>`).join('')}
    </div>
    <div class="profile-bar-actions">
      <button class="secondary-btn" data-act="profile-new" ${full || view.busy ? 'disabled' : ''}
              title="${full ? `Máximo ${state.maxProfiles} perfiles` : 'Crear un perfil vacío'}">
        ${icon('plus', 16)} Nuevo
      </button>
      <button class="secondary-btn" data-act="profile-dup" ${full || view.busy ? 'disabled' : ''}
              title="Copiar el perfil actual con sus iconos">
        ${icon('profiles', 16)} Duplicar
      </button>
      <button class="secondary-btn danger" data-act="profile-del"
              ${state.profiles.length <= 1 || view.busy ? 'disabled' : ''}>
        ${icon('trash', 16)} Eliminar
      </button>
      <span class="profile-count">${state.profiles.length} / ${state.maxProfiles}</span>
    </div>`;
}

function renderTabs() {
  const bar = document.getElementById('profile-bar');
  if (bar) bar.innerHTML = renderTabsInner();
}

// Variaciones del perfil: el mismo juego de atajos con un par de teclas
// distintas para una aplicación concreta, sin duplicar el perfil entero.
function renderVariantBar() {
  const list = variants.forProfile(view.editingProfile);

  return `
    <div class="variant-bar">
      <span class="variant-bar-label">${icon('bolt', 14)} Según la app</span>
      <div class="chip-row">
        <button class="chip ${!view.variantId ? 'on' : ''}" data-act="pick-variant" data-id="">
          Perfil base
        </button>
        ${list.map((v) => `
          <button class="chip ${view.variantId === v.id ? 'on' : ''} ${variants.isApplied(v.id) ? 'is-live' : ''}"
                  data-act="pick-variant" data-id="${v.id}"
                  title="${escape(v.match ? `Se aplica con: ${v.match}` : 'Sin aplicación asignada')}">
            ${escape(v.name)}
            <em class="chip-count">${variants.countOverrides(v)}</em>
          </button>`).join('')}
        <button class="chip ghost" data-act="new-variant">${icon('plus', 14)} Nueva variación</button>
      </div>
    </div>`;
}

function renderVariantSettings() {
  const variant = editingVariant();
  if (!variant) return '';

  const applied = variants.isApplied(variant.id);
  const count = variants.countOverrides(variant);

  return `
    <div class="glass-panel oled-card variant-card">
      <div class="card-header">
        ${icon('bolt', 20)}<h2>Variación de ${escape(currentProfile().name)}</h2>
        <span class="pill ${applied ? 'pill-live' : ''}">${applied ? 'Aplicada ahora' : `${count} cambios`}</span>
      </div>

      <div class="row-inline">
        <label class="field">
          <span class="field-label">Nombre</span>
          <input type="text" class="text-input" value="${escape(variant.name)}" data-act="variant-name">
        </label>
        <label class="field">
          <span class="field-label">Se aplica cuando la ventana contiene</span>
          <input type="text" class="text-input" placeholder="p. ej. notepad" value="${escape(variant.match)}" data-act="variant-match">
        </label>
        <label class="field">
          <span class="field-label">Comparar contra</span>
          <select class="select-input" data-act="variant-field">
            <option value="any"     ${variant.field === 'any' ? 'selected' : ''}>Programa o título</option>
            <option value="process" ${variant.field === 'process' ? 'selected' : ''}>Solo programa</option>
            <option value="title"   ${variant.field === 'title' ? 'selected' : ''}>Solo título</option>
          </select>
        </label>
      </div>

      <p class="setting-desc">
        Estás editando <strong>solo las diferencias</strong>: lo que cambies aquí se
        guarda como excepción de este perfil para esa app, y todo lo demás lo sigue
        mandando el perfil base. Las teclas y mandos redefinidos salen marcados abajo.
        ${count === 0 ? '<br>Todavía no hay ninguna diferencia: cambia una tecla para empezar.' : ''}
      </p>

      <div class="row-inline">
        <button class="secondary-btn danger" data-act="del-variant">${icon('trash', 16)} Eliminar variación</button>
      </div>
    </div>`;
}

// Réplica del teclado con el icono real de cada tecla, igual que en el editor
// de iconos: es la representación que ya conoce el usuario.
function renderKeyGridInner() {
  const prof = currentProfile();
  const variant = editingVariant();
  const selected = selectedKeyIndex();
  let html = '';

  for (let i = 0; i < 12; i++) {
    const screen = KEY_TO_SCREEN[i];
    const lslot = labelSlot(i, view.layer);
    const action = variants.effectiveKey(prof, variant, i, view.layer);
    const label = lslot >= 0 ? variants.effectiveLabel(prof, variant, i, view.layer) : '';
    const bmp = lslot >= 0 ? cache.get(view.editingProfile, lslot) : null;
    const assigned = action.modifier || action.keycode;
    // Marca las teclas que la variación redefine respecto al perfil base.
    const changed = variant && (variants.override(variant, 'keys', keymapSlot(i, view.layer))
                             || variants.override(variant, 'labels', lslot));

    html += `
      <button class="okey ${selected === i ? 'selected' : ''} ${bmp ? 'has-icon' : ''} ${screen ? '' : 'roleless'} ${changed ? 'is-override' : ''}"
              data-act="pick-key" data-key="${i}">
        <span class="okey-num">T${i + 1}${screen ? `<em>P${screen}</em>` : ''}${changed ? '<i class="okey-dot" title="Cambiado en esta variación"></i>' : ''}</span>
        <span class="okey-screen">
          ${screen
            ? (bmp ? `<canvas class="okey-canvas" data-bmp="${cache.cacheKey(view.editingProfile, lslot)}"></canvas>`
                   : `<span class="okey-text">${escape(label || '—')}</span>`)
            : `<span class="okey-role">${i === 9 ? 'SUPER' : 'MENÚ'}</span>`}
        </span>
        <span class="okey-action ${assigned ? 'assigned' : ''}">${escape(describeAction(action.modifier, action.keycode))}</span>
      </button>`;
  }
  return html;
}

function paintKeyGrid() {
  const grid = document.getElementById('profile-key-grid');
  if (grid) cache.paintThumbs(grid);

  const status = document.getElementById('profile-grid-status');
  if (status) {
    status.textContent = cache.isLoading()
      ? 'Leyendo iconos del teclado…'
      : `Capa ${view.layer === 'super' ? 'SUPER' : 'NORMAL'} · las teclas 10 y 12 no tienen pantalla: `
        + 'la 10 es el modificador SUPER y la 12 abre el menú al mantenerla.';
  }
}

function renderKeyGrid() {
  const grid = document.getElementById('profile-key-grid');
  if (!grid) return;
  grid.innerHTML = renderKeyGridInner();
  paintKeyGrid();
}

function renderRotaryGroups() {
  const prof = currentProfile();
  const variant = editingVariant();
  const selected = selectedRotarySlot();

  return ROTARY_GROUPS.map((group) => `
    <div class="rotary-group">
      <span class="rotary-group-name">${icon(group.icon, 14)} ${group.name}</span>
      ${group.parts.map((part) => {
        const action = variants.effectiveRotary(prof, variant, part.slot, view.layer);
        const changed = variant && variants.override(variant, 'rotary', rotarySlot(part.slot, view.layer));
        return `
          <button class="rotary-part ${selected === part.slot ? 'selected' : ''} ${action.type ? 'assigned' : ''} ${changed ? 'is-override' : ''}"
                  data-act="pick-rotary" data-slot="${part.slot}">
            <span class="rp-dir">${part.short}</span>
            <span class="rp-body">
              <em>${part.label}</em>
              <strong>${escape(describeRotary(action))}</strong>
            </span>
          </button>`;
      }).join('')}
    </div>`).join('');
}

function renderWheelCard() {
  const s = currentScroll();
  const layerName = view.layer === 'super' ? 'SUPER' : 'normal';

  return `
    <div class="glass-panel oled-card">
      <div class="card-header">
        ${icon('wheel', 20)}<h2>Rueda de scroll</h2>
        <span class="pill">Capa ${layerName}</span>
      </div>

      ${editingVariant() ? `
        <p class="setting-desc override-note">
          La sensibilidad de la rueda es siempre la del <strong>perfil base</strong>:
          las variaciones cambian atajos y etiquetas, no la calibración.
        </p>` : ''}

      <div class="wheel-tune">
        <div class="wheel-tune-main">
          <div class="wheel-readout">
            <span id="scroll-value">${s.detentsPerRev}</span>
            <small>clics por vuelta completa</small>
          </div>

          <input type="range" id="scroll-slider" class="premium-slider" data-act="scroll-slider"
                 min="6" max="240" step="1" value="${s.detentsPerRev}">

          <div class="preset-row">
            ${SCROLL_PRESETS.map((p) => `
              <button class="preset-btn ${s.detentsPerRev === p.value ? 'on' : ''}" data-act="scroll-preset" data-value="${p.value}">
                <strong>${p.name}</strong>
                <span>${p.value} clics</span>
                <em>${p.desc}</em>
              </button>`).join('')}
          </div>

          <ul class="info-list" id="scroll-derived"></ul>

          <div class="row-inline mt-4">
            <button class="secondary-btn ${s.invert ? 'is-on' : ''}" data-act="scroll-invert">
              ${icon('reset', 16)} ${s.invert ? 'Dirección invertida' : 'Invertir dirección'}
            </button>
          </div>
        </div>

        <div class="wheel-tune-side">
          <div class="hires-state ${state.scroll.hires ? 'ok' : 'off'}">
            <span class="hires-dot"></span>
            <strong>${state.scroll.hires ? 'Alta resolución activa' : 'Alta resolución no negociada'}</strong>
          </div>
          <div class="wheel-dial">
            <div class="wheel-dial-face">${wheelDial.markerHtml('scroll-live-needle')}</div>
          </div>
          <p class="wheel-live-readout" id="scroll-live-readout">Gira la rueda para comprobar el recorrido</p>

          ${renderDialCalibration()}

          <p class="setting-desc">
            La sensibilidad pertenece a <strong>${escape(currentProfile().name)}</strong> en la capa
            ${layerName}: cada perfil —y cada capa— puede tener la suya.
            ${state.scroll.hires
              ? 'Con el multiplicador negociado el desplazamiento viaja en unidades de 1/120 de clic.'
              : 'Hasta que Windows pida el multiplicador, las aplicaciones antiguas saltarán de tres en tres líneas.'}
          </p>
        </div>
      </div>
    </div>`;
}

// Cómo se dibuja la rueda en la app. No toca el teclado: solo hace que el
// dibujo coincida con el marcador que lleve pegado la tapa.
function renderDialCalibration() {
  const d = wheelDial.dial;

  return `
    <div class="dial-calib">
      <span class="field-label">Marcador en pantalla</span>

      <div class="chip-row">
        <button class="chip ${d.marker === 'dot' ? 'on' : ''}" data-act="dial-marker" data-marker="dot">Círculo</button>
        <button class="chip ${d.marker === 'line' ? 'on' : ''}" data-act="dial-marker" data-marker="line">Raya</button>
        <button class="chip ${d.invert ? 'on' : ''}" data-act="dial-invert"
                title="Si el dibujo gira al revés que la rueda">Invertir giro</button>
      </div>

      <div class="dial-offset">
        <button class="tool-btn small" data-act="dial-nudge" data-d="-1" title="1° menos">${icon('minus', 14)}</button>
        <input type="range" class="premium-slider" data-act="dial-offset"
               min="0" max="359" step="1" value="${Math.round(d.offsetDeg)}">
        <button class="tool-btn small" data-act="dial-nudge" data-d="1" title="1° más">${icon('plus', 14)}</button>
        <b id="dial-offset-val">${Math.round(d.offsetDeg)}°</b>
      </div>

      <button class="secondary-btn full" data-act="dial-align">
        ${icon('fit', 16)} El marcador está arriba: alinear aquí
      </button>
      <p class="setting-desc">
        Pon el circulito de la tapa mirando hacia arriba y pulsa el botón: el de la
        pantalla se coloca en el mismo sitio. Con la barra afinas grado a grado.
      </p>
    </div>`;
}

function updateDerived(detents) {
  const el = document.getElementById('scroll-derived');
  if (!el) return;
  const degPerDetent = 360 / detents;
  const unitsPerCount = (detents * 120) / 4096;
  el.innerHTML = `
    <li><span class="lbl">Giro por clic</span><span class="val">${degPerDetent.toFixed(1)}°</span></li>
    <li><span class="lbl">Unidades HID por cuenta</span><span class="val">${unitsPerCount.toFixed(3)}</span></li>
    <li><span class="lbl">Líneas por vuelta (Windows)</span><span class="val">${(detents * 3).toFixed(0)}</span></li>`;
}

// Lista de teclas HID reutilizada por los dos editores.
function keycodeOptions(selectedCode) {
  return KEY_GROUPS.map((g) => `
    <optgroup label="${g.name}">
      ${g.keys.map((k) => `<option value="${k.code}" ${selectedCode === k.code ? 'selected' : ''}>${escape(k.label)}</option>`).join('')}
    </optgroup>`).join('');
}

function renderInspector() {
  if (!view.selected) {
    return `<div class="editor-inspector glass-panel empty-panel">
      ${icon('key', 36)}
      <h3>Elige una tecla o un mando</h3>
      <p>Selecciona cualquier tecla para cambiar su icono, su etiqueta y su atajo, o un giro
         de los encoders y la rueda para reasignarlo.</p>
    </div>`;
  }

  return view.selected.kind === 'rotary' ? renderRotaryInspector() : renderKeyInspector();
}

function renderRotaryInspector() {
  const base = selectedRotarySlot();
  const found = rotaryPart(base);
  const action = currentRotary();
  const isClick = Boolean(found?.part.discrete);
  const types = ROTARY_TYPE_OPTIONS.filter((t) => !(isClick && t.turnOnly));

  const variant = editingVariant();
  const changed = Boolean(variant && variants.override(variant, 'rotary', rotarySlot(base, view.layer)));
  const baseAction = currentProfile().rotary?.[rotarySlot(base, view.layer)] || { type: 0, modifier: 0, keycode: 0 };

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>${escape(found?.group.name || 'Mando')}</h3>
        <span class="pill">${view.layer === 'super' ? 'Capa SUPER' : 'Capa normal'}</span>
      </div>
      <p class="setting-desc inspector-sub">${escape(found?.part.label || '')}</p>

      ${variant ? `
        <div class="override-note ${changed ? 'is-override' : ''}">
          <span>${changed
            ? `Cambiado en <strong>${escape(variant.name)}</strong> · el perfil base hace
               <code>${escape(describeRotary(baseAction))}</code>`
            : `Editando <strong>${escape(variant.name)}</strong>: solo valdrá con esa app`}</span>
          ${changed ? `<button class="secondary-btn" data-act="clear-override">
                         ${icon('reset', 14)} Volver al valor base
                       </button>` : ''}
        </div>` : ''}

      <div class="field">
        <span class="field-label">Tipo de acción</span>
        <div class="type-grid">
          ${types.map((t) => `
            <button class="type-chip ${action.type === t.type ? 'on' : ''}"
                    data-act="rotary-type" data-type="${t.type}">${t.label}</button>`).join('')}
        </div>
      </div>

      ${action.type === ROTARY_TYPES.CONSUMER ? `
        <div class="field">
          <span class="field-label">Acción</span>
          <div class="consumer-grid">
            ${CONSUMER_ACTIONS.map((c) => `
              <button class="consumer-chip ${action.keycode === c.index ? 'on' : ''}"
                      data-act="rotary-consumer" data-index="${c.index}">${c.label}</button>`).join('')}
          </div>
        </div>` : ''}

      ${action.type === ROTARY_TYPES.KEY ? `
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${MODIFIERS.map((m) => `
              <button class="mod-chip ${action.modifier & m.bit ? 'on' : ''}"
                      data-act="rotary-mod" data-bit="${m.bit}">${m.label}</button>`).join('')}
          </div>
        </div>
        <label class="field">
          <span class="field-label">Tecla</span>
          <select class="select-input" data-act="rotary-keycode">
            <option value="0" ${!action.keycode ? 'selected' : ''}>— ninguna —</option>
            ${keycodeOptions(action.keycode)}
          </select>
        </label>
        <button class="primary-btn full ${view.capturing ? 'is-capturing' : ''}" data-act="capture">
          ${icon('key', 16)} ${view.capturing ? 'Pulsa el atajo… (Esc cancela)' : 'Capturar atajo del teclado'}
        </button>` : ''}

      ${isScrollType(action.type) ? `
        <p class="setting-desc">
          El sentido va implícito en el giro, así que esta acción cubre las dos direcciones.
          ${action.type === ROTARY_TYPES.SCROLL_V
            ? 'En la rueda magnética es la única opción que aprovecha la alta resolución; el resto trabajan por clics completos.'
            : ''}
        </p>` : ''}

      <div class="inspector-summary">
        <span class="field-label">Resultado</span>
        <code>${escape(describeRotary(action))}</code>
      </div>
    </div>`;
}

function renderKeyInspector() {
  const i = selectedKeyIndex();
  const prof = currentProfile();
  const variant = editingVariant();
  const lslot = labelSlot(i, view.layer);
  const action = currentAction();
  const isConsumer = action.modifier === CONSUMER_MODIFIER;
  const keyOptions = keycodeOptions(isConsumer ? 0 : action.keycode);
  const hasIcon = lslot >= 0 && Boolean(cache.get(view.editingProfile, lslot));
  const baseAction = prof.keys[keymapSlot(i, view.layer)] || { modifier: 0, keycode: 0 };
  const changed = Boolean(variant && (variants.override(variant, 'keys', keymapSlot(i, view.layer))
                                   || variants.override(variant, 'labels', lslot)));

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>Tecla ${i + 1}</h3>
        <span class="pill">${view.layer === 'super' ? 'Capa SUPER' : 'Capa normal'}</span>
      </div>

      ${variant ? `
        <div class="override-note ${changed ? 'is-override' : ''}">
          <span>${changed
            ? `Cambiada en <strong>${escape(variant.name)}</strong> · el perfil base hace
               <code>${escape(describeAction(baseAction.modifier, baseAction.keycode))}</code>`
            : `Editando <strong>${escape(variant.name)}</strong>: lo que cambies aquí solo
               valdrá con esa app`}</span>
          ${changed ? `<button class="secondary-btn" data-act="clear-override">
                         ${icon('reset', 14)} Volver al valor base
                       </button>` : ''}
        </div>` : ''}

      ${lslot >= 0 ? `
        <div class="field">
          <span class="field-label">Pantalla OLED</span>
          <div class="inspector-screen">
            <span class="okey-screen">
              ${hasIcon ? `<canvas class="okey-canvas" data-bmp="${cache.cacheKey(view.editingProfile, lslot)}"></canvas>`
                        : `<span class="okey-text">${escape(prof.labels[lslot] || '—')}</span>`}
            </span>
            <button class="secondary-btn" data-act="edit-icon" data-key="${i}">
              ${icon('pencil', 16)} ${hasIcon ? 'Editar icono' : 'Dibujar icono'}
            </button>
          </div>
          <p class="setting-desc">
            ${hasIcon
              ? 'La pantalla muestra este icono; la etiqueta de texto queda de reserva.'
              : 'Sin icono, la pantalla muestra la etiqueta de texto.'}
          </p>
        </div>

        <label class="field">
          <span class="field-label">Etiqueta OLED (máx. 7 caracteres)</span>
          <input type="text" class="text-input" maxlength="7"
                 value="${escape(variants.effectiveLabel(prof, variant, i, view.layer))}"
                 data-act="edit-label" data-slot="${lslot}">
        </label>` : `
        <p class="setting-desc">
          Esta tecla no tiene pantalla: la 10 es el modificador SUPER y la 12 abre el menú
          del teclado al mantenerla, así que el firmware no ejecuta su atajo.
        </p>`}

      <div class="field">
        <span class="field-label">Modificadores</span>
        <div class="mod-grid">
          ${MODIFIERS.map((m) => `
            <button class="mod-chip ${!isConsumer && (action.modifier & m.bit) ? 'on' : ''}"
                    data-act="toggle-mod" data-bit="${m.bit}">${m.label}</button>`).join('')}
        </div>
      </div>

      <label class="field">
        <span class="field-label">Tecla</span>
        <select class="select-input" data-act="pick-keycode">
          <option value="0" ${!isConsumer && !action.keycode ? 'selected' : ''}>— ninguna —</option>
          ${keyOptions}
        </select>
      </label>

      <div class="field">
        <span class="field-label">Acción multimedia</span>
        <div class="consumer-grid">
          ${CONSUMER_ACTIONS.map((c) => `
            <button class="consumer-chip ${isConsumer && action.keycode === c.index ? 'on' : ''}"
                    data-act="set-consumer" data-index="${c.index}">${c.label}</button>`).join('')}
        </div>
      </div>

      <div class="inspector-actions">
        <button class="primary-btn ${view.capturing ? 'is-capturing' : ''}" data-act="capture">
          ${icon('key', 16)} ${view.capturing ? 'Pulsa el atajo… (Esc cancela)' : 'Capturar atajo del teclado'}
        </button>
        <button class="secondary-btn" data-act="clear-action">${icon('trash', 16)} Quitar</button>
      </div>

      <div class="inspector-summary">
        <span class="field-label">Resultado</span>
        <code>${escape(describeAction(action.modifier, action.keycode))}</code>
      </div>
    </div>`;
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
