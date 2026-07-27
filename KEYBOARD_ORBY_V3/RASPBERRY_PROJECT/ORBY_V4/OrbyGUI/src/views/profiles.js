// Editor de perfiles.
//
// Sustituye a la lista de solo lectura anterior: los datos vienen del firmware
// (GET_PROFILE) y cada cambio se escribe con SET_LABEL / SET_KEYMAP / SET_NAME.
// Los cambios viven en RAM del microcontrolador hasta que se pulsa "Guardar en
// Flash", igual que en el menú físico del teclado.

import * as device from '../device.js';
import { state, notify, markDirty, PROFILE_META, labelSlot, keymapSlot, KEY_TO_SCREEN } from '../store.js';
import { MODIFIERS, CONSUMER_MODIFIER, CONSUMER_ACTIONS, KEY_GROUPS, describeAction, eventToAction,
         ROTARY_TYPES, ROTARY_SLOTS, isScrollType, describeRotary } from '../hid-keys.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';

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

const view = {
  editingProfile: 0,
  layer: 'normal',
  selected: null,   // { kind: 'key', index } | { kind: 'rotary', slot }
  capturing: false,
};

function selectedKeyIndex() {
  return view.selected?.kind === 'key' ? view.selected.index : null;
}

function selectedRotarySlot() {
  return view.selected?.kind === 'rotary' ? view.selected.slot : null;
}

function rotaryPart(slot) {
  for (const g of ROTARY_GROUPS) {
    const p = g.parts.find((x) => x.slot === slot);
    if (p) return { group: g, part: p };
  }
  return null;
}

export function init() {
  document.getElementById('profiles-body').addEventListener('click', onClick);
  document.getElementById('profiles-body').addEventListener('change', onChange);
  document.getElementById('profiles-body').addEventListener('input', onInput);
  window.addEventListener('keydown', onCapture, true);
}

// --- Interacción -----------------------------------------------------------

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;
  const act = el.dataset.act;

  if (act === 'pick-profile') {
    view.editingProfile = Number(el.dataset.idx);
    view.selected = null;
    render();
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
  }
}

function onChange(e) {
  if (e.target.dataset.act === 'pick-keycode') {
    const current = currentAction();
    const mod = current.modifier === CONSUMER_MODIFIER ? 0 : current.modifier;
    applyKeymap(mod, Number(e.target.value));
  } else if (e.target.dataset.act === 'rotary-keycode') {
    const cur = currentRotary();
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: cur.modifier, keycode: Number(e.target.value) });
  }
}

let labelDebounce = null;
function onInput(e) {
  if (e.target.dataset.act === 'edit-label') {
    const slot = Number(e.target.dataset.slot);
    const text = e.target.value.slice(0, 7);
    clearTimeout(labelDebounce);
    labelDebounce = setTimeout(async () => {
      const prof = state.profiles[view.editingProfile];
      if (!prof) return;
      prof.labels[slot] = text;
      try {
        await device.setLabel(view.editingProfile, slot, text);
        markDirty();
        renderPreview();
      } catch {
        toast('El teclado no confirmó la etiqueta', 'error');
      }
    }, 250);
  } else if (e.target.dataset.act === 'edit-name') {
    const text = e.target.value.slice(0, 7);
    clearTimeout(labelDebounce);
    labelDebounce = setTimeout(async () => {
      const prof = state.profiles[view.editingProfile];
      if (!prof) return;
      prof.name = text;
      try {
        await device.setName(view.editingProfile, text);
        markDirty();
        notify();
      } catch {
        toast('El teclado no confirmó el nombre', 'error');
      }
    }, 250);
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

function currentAction() {
  const prof = state.profiles[view.editingProfile];
  const index = selectedKeyIndex();
  if (!prof || index === null) return { modifier: 0, keycode: 0 };
  return prof.keys[keymapSlot(index, view.layer)] || { modifier: 0, keycode: 0 };
}

function currentRotary() {
  const prof = state.profiles[view.editingProfile];
  const slot = selectedRotarySlot();
  if (!prof || slot === null) return { type: 0, modifier: 0, keycode: 0 };
  return prof.rotary?.[slot] || { type: 0, modifier: 0, keycode: 0 };
}

async function applyKeymap(modifier, keycode) {
  const prof = state.profiles[view.editingProfile];
  const index = selectedKeyIndex();
  if (!prof || index === null) return;

  const slot = keymapSlot(index, view.layer);
  prof.keys[slot] = { modifier, keycode };
  render();

  try {
    await device.setKeymap(view.editingProfile, slot, modifier, keycode);
    markDirty();
  } catch {
    toast('No se pudo escribir la asignación', 'error');
  }
}

async function applyRotary(action) {
  const prof = state.profiles[view.editingProfile];
  const slot = selectedRotarySlot();
  if (!prof || slot === null) return;

  prof.rotary[slot] = action;

  // Los desplazamientos son bidireccionales, así que el firmware ignora la
  // acción del sentido contrario. Espejamos el par para que la interfaz no
  // enseñe una configuración que no se va a aplicar.
  const twin = { [ROTARY_SLOTS.ENC1_CW]: ROTARY_SLOTS.ENC1_CCW,
                 [ROTARY_SLOTS.ENC1_CCW]: ROTARY_SLOTS.ENC1_CW,
                 [ROTARY_SLOTS.ENC2_CW]: ROTARY_SLOTS.ENC2_CCW,
                 [ROTARY_SLOTS.ENC2_CCW]: ROTARY_SLOTS.ENC2_CW,
                 [ROTARY_SLOTS.WHEEL_CW]: ROTARY_SLOTS.WHEEL_CCW,
                 [ROTARY_SLOTS.WHEEL_CCW]: ROTARY_SLOTS.WHEEL_CW }[slot];

  const writes = [[slot, action]];
  if (twin !== undefined && isScrollType(action.type)) {
    prof.rotary[twin] = { ...action };
    writes.push([twin, action]);
  }

  render();
  try {
    for (const [s, a] of writes) {
      await device.setRotary(view.editingProfile, s, a.type, a.modifier, a.keycode);
    }
    markDirty();
  } catch {
    toast('No se pudo escribir la acción del mando', 'error');
  }
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

  body.innerHTML = `
    <div class="profile-tabs">
      ${state.profiles.map((p, i) => `
        <button class="profile-tab ${i === view.editingProfile ? 'active' : ''} ${i === state.activeProfileIdx ? 'is-live' : ''}"
                data-act="pick-profile" data-idx="${i}">
          <span class="tab-icon" style="--accent:${PROFILE_META[i].accent}">${icon(PROFILE_META[i].icon, 18)}</span>
          <span class="tab-name">${escape(p.name)}</span>
          ${i === state.activeProfileIdx ? '<span class="tab-live">EN USO</span>' : ''}
        </button>`).join('')}
    </div>

    <div class="editor-layout">
      <div class="editor-board glass-panel">
        <div class="editor-board-head">
          <label class="field-inline">
            <span>Nombre</span>
            <input type="text" maxlength="7" value="${escape(state.profiles[view.editingProfile].name)}"
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
        <div class="editor-keys" id="editor-keys">${renderKeys()}</div>
        <p class="setting-desc">
          Las teclas 10 y 12 no tienen pantalla: la 10 es el modificador SUPER y la 12 abre el menú al mantenerla.
        </p>

        <div class="rotary-section">
          <h3 class="rotary-title">${icon('wheel', 16)} Mandos giratorios</h3>
          <div class="rotary-groups">${renderRotaryGroups()}</div>
          <p class="setting-desc">
            Los encoders solo ejecutan estas acciones en modo normal: dentro del menú del teclado
            siguen sirviendo para navegar.
          </p>
        </div>
      </div>

      ${renderInspector()}
    </div>`;
}

function renderRotaryGroups() {
  const prof = state.profiles[view.editingProfile];
  const selected = selectedRotarySlot();

  return ROTARY_GROUPS.map((group) => `
    <div class="rotary-group">
      <span class="rotary-group-name">${icon(group.icon, 14)} ${group.name}</span>
      ${group.parts.map((part) => {
        const action = prof.rotary?.[part.slot] || { type: 0, modifier: 0, keycode: 0 };
        return `
          <button class="rotary-part ${selected === part.slot ? 'selected' : ''} ${action.type ? 'assigned' : ''}"
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

function renderKeys() {
  const prof = state.profiles[view.editingProfile];
  const selected = selectedKeyIndex();
  let html = '';

  for (let i = 0; i < 12; i++) {
    const screen = KEY_TO_SCREEN[i];
    const lslot = labelSlot(i, view.layer);
    const action = prof.keys[keymapSlot(i, view.layer)] || { modifier: 0, keycode: 0 };
    const label = lslot >= 0 ? prof.labels[lslot] : '';
    const assigned = action.modifier || action.keycode;

    html += `
      <button class="editor-key ${selected === i ? 'selected' : ''} ${screen ? '' : 'no-screen'} ${assigned ? 'assigned' : ''}"
              data-act="pick-key" data-key="${i}">
        <span class="ek-num">T${i + 1}</span>
        <span class="ek-label">${screen ? escape(label || '—') : (i === 9 ? 'SUPER' : 'MENU')}</span>
        <span class="ek-action">${escape(describeAction(action.modifier, action.keycode))}</span>
      </button>`;
  }
  return html;
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
      <p>Selecciona cualquier tecla para cambiar su etiqueta OLED y su atajo, o un giro
         de los encoders y la rueda para reasignarlo.</p>
    </div>`;
  }

  return view.selected.kind === 'rotary' ? renderRotaryInspector() : renderKeyInspector();
}

function renderRotaryInspector() {
  const slot = selectedRotarySlot();
  const found = rotaryPart(slot);
  const action = currentRotary();
  const isClick = Boolean(found?.part.discrete);
  const types = ROTARY_TYPE_OPTIONS.filter((t) => !(isClick && t.turnOnly));

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>${escape(found?.group.name || 'Mando')}</h3>
        <span class="pill">${escape(found?.part.label || '')}</span>
      </div>

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
  const prof = state.profiles[view.editingProfile];
  const lslot = labelSlot(i, view.layer);
  const action = currentAction();
  const isConsumer = action.modifier === CONSUMER_MODIFIER;
  const keyOptions = keycodeOptions(isConsumer ? 0 : action.keycode);

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>Tecla ${i + 1}</h3>
        <span class="pill">${view.layer === 'super' ? 'Capa SUPER' : 'Capa normal'}</span>
      </div>

      ${lslot >= 0 ? `
        <label class="field">
          <span class="field-label">Etiqueta OLED (máx. 7 caracteres)</span>
          <input type="text" class="text-input" maxlength="7" value="${escape(prof.labels[lslot] || '')}"
                 data-act="edit-label" data-slot="${lslot}">
        </label>` : `
        <p class="setting-desc">Esta tecla no tiene pantalla asociada.</p>`}

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

function renderPreview() {
  const keys = document.getElementById('editor-keys');
  if (keys) keys.innerHTML = renderKeys();
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
