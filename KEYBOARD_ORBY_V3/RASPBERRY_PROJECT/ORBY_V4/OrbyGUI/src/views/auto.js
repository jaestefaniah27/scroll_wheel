// Cambio automático de perfil según la aplicación en primer plano.
//
// El proceso principal vigila la ventana activa de Windows y nos manda
// { process, title }. Aquí se comparan esas cadenas con las reglas del usuario
// y, al primer encaje, se manda SET_PROFILE al teclado.

import * as device from '../device.js';
import { state, notify, subscribe, PROFILE_META } from '../store.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';

const view = {
  available: false,
  enabled: false,
  rules: [],
  fallback: null,
  current: null,     // { process, title }
  lastApplied: null, // id de la última regla aplicada
  error: null,
};

// Sugerencias para el botón de añadir rápido.
const SUGGESTIONS = [
  { match: 'photoshop', label: 'Photoshop' },
  { match: 'altium',    label: 'Altium Designer' },
  { match: 'premiere',  label: 'Premiere Pro' },
  { match: 'code',      label: 'VS Code' },
  { match: 'excel',     label: 'Excel' },
  { match: 'winword',   label: 'Word' },
  { match: 'chrome',    label: 'Chrome' },
  { match: 'blender',   label: 'Blender' },
];

export async function init() {
  const root = document.getElementById('view-auto');
  root.addEventListener('click', onClick);
  root.addEventListener('change', onChange);

  view.available = await window.orby.foreground.available();

  const cfg = await window.orby.getConfig();
  Object.assign(view, {
    enabled: cfg.autoProfile.enabled,
    rules: cfg.autoProfile.rules || [],
    fallback: cfg.autoProfile.fallback,
  });

  window.orby.foreground.onChange((info) => {
    view.current = info;
    evaluate(info);
    renderStatus();
  });

  window.orby.foreground.onError((err) => {
    view.error = err;
    renderStatus();
  });

  // Al conectar el teclado hay que reevaluar: la ventana activa puede llevar
  // rato siendo la misma, así que no llegará ningún evento de cambio.
  device.on('connected', () => setTimeout(() => evaluate(view.current), 1500));

  // Los nombres de los perfiles llegan del firmware; hay que repintar los
  // desplegables cuando terminan de sincronizarse.
  let lastProfileCount = 0;
  subscribe(() => {
    if (state.profiles.length !== lastProfileCount) {
      lastProfileCount = state.profiles.length;
      render();
    } else {
      renderStatus();
    }
  });

  if (view.enabled) view.current = await window.orby.foreground.current();
  render();
}

// --- Motor de reglas --------------------------------------------------------

// El campo `field` decide contra qué se compara: el ejecutable, el título de la
// ventana o ambos. Comparar solo el ejecutable falla con apps como los
// navegadores, donde lo que distingue el contexto está en el título.
function haystack(info, field) {
  if (!info) return '';
  if (field === 'title') return (info.title || '').toLowerCase();
  if (field === 'process') return (info.process || '').toLowerCase();
  return `${info.process || ''} ${info.title || ''}`.toLowerCase();
}

function matchRule(info) {
  for (const rule of view.rules) {
    const needle = (rule.match || '').trim().toLowerCase();
    if (!needle) continue;
    if (haystack(info, rule.field).includes(needle)) return rule;
  }
  return null;
}

async function evaluate(info) {
  if (!view.enabled || !state.connected) return;

  const rule = matchRule(info);
  const target = rule ? rule.profile : view.fallback;

  if (target === null || target === undefined) return;
  if (target === state.activeProfileIdx) { view.lastApplied = rule?.id ?? 'fallback'; return; }

  try {
    await device.setProfile(target);
    state.activeProfileIdx = target;
    view.lastApplied = rule?.id ?? 'fallback';
    notify();
    renderStatus();
  } catch {
    // Silencioso: el teclado puede estar ocupado o recién desconectado, y no
    // tiene sentido escupir un aviso por cada cambio de ventana.
  }
}

// --- Persistencia -----------------------------------------------------------

function persist() {
  return window.orby.setConfig({
    autoProfile: { enabled: view.enabled, rules: view.rules, fallback: view.fallback },
  });
}

// --- Interacción ------------------------------------------------------------

async function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;
  const act = el.dataset.act;

  if (act === 'toggle') {
    view.enabled = !view.enabled;
    await persist();
    if (view.enabled) {
      const ok = await window.orby.foreground.start();
      if (!ok) { toast('No se pudo iniciar el detector de aplicaciones', 'error'); view.enabled = false; await persist(); }
      else { view.current = await window.orby.foreground.current(); evaluate(view.current); }
    } else {
      await window.orby.foreground.stop();
    }
    render();

  } else if (act === 'add-current') {
    const proc = view.current?.process;
    if (!proc) { toast('Aún no se ha detectado ninguna ventana', 'error'); return; }
    addRule(proc.toLowerCase(), state.activeProfileIdx);

  } else if (act === 'add-suggestion') {
    addRule(el.dataset.match, 0);

  } else if (act === 'add-blank') {
    addRule('', 0);

  } else if (act === 'remove') {
    view.rules = view.rules.filter((r) => r.id !== el.dataset.id);
    persist();
    render();

  } else if (act === 'move-up' || act === 'move-down') {
    const i = view.rules.findIndex((r) => r.id === el.dataset.id);
    const j = act === 'move-up' ? i - 1 : i + 1;
    if (i < 0 || j < 0 || j >= view.rules.length) return;
    [view.rules[i], view.rules[j]] = [view.rules[j], view.rules[i]];
    persist();
    render();
  }
}

function addRule(match, profile) {
  view.rules.push({ id: `r${Date.now()}${Math.random().toString(36).slice(2, 6)}`, match, profile, field: 'any' });
  persist();
  render();
}

function onChange(e) {
  const el = e.target;
  const id = el.dataset.id;

  if (el.dataset.act === 'rule-match' || el.dataset.act === 'rule-profile' || el.dataset.act === 'rule-field') {
    const rule = view.rules.find((r) => r.id === id);
    if (!rule) return;
    if (el.dataset.act === 'rule-match')   rule.match = el.value.trim();
    if (el.dataset.act === 'rule-profile') rule.profile = Number(el.value);
    if (el.dataset.act === 'rule-field')   rule.field = el.value;
    persist();
    renderStatus();

  } else if (el.dataset.act === 'fallback') {
    view.fallback = el.value === '' ? null : Number(el.value);
    persist();
  }
}

// --- Render -----------------------------------------------------------------

function profileNames() {
  return state.profiles.length ? state.profiles.map((p) => p.name) : ['P1', 'P2', 'P3', 'P4'];
}

function profileOptions(selected, includeNone = false) {
  const names = profileNames();
  const none = includeNone ? `<option value="" ${selected === null ? 'selected' : ''}>— no cambiar —</option>` : '';
  return none + names
    .map((n, i) => `<option value="${i}" ${selected === i ? 'selected' : ''}>${escape(n)}</option>`)
    .join('');
}

function renderStatus() {
  const el = document.getElementById('auto-status');
  if (!el) return;

  const info = view.current;
  const rule = info ? matchRule(info) : null;
  const names = profileNames();

  el.innerHTML = `
    <div class="auto-now ${rule ? 'matched' : ''}">
      <span class="field-label">Ventana activa</span>
      <strong>${escape(info?.process || '—')}</strong>
      <em>${escape(info?.title || 'Sin detección todavía')}</em>
      <span class="auto-verdict">
        ${rule
          ? `Encaja con <code>${escape(rule.match)}</code> → <b>${escape(names[rule.profile] ?? '?')}</b>`
          : (view.fallback !== null && view.fallback !== undefined
              ? `Sin regla → vuelve a <b>${escape(names[view.fallback] ?? '?')}</b>`
              : 'Sin regla que encaje')}
      </span>
    </div>
    ${view.error ? `<p class="auto-error">${escape(view.error)}</p>` : ''}`;
}

export function render() {
  const root = document.getElementById('view-auto');
  if (!root) return;

  if (!view.available) {
    root.innerHTML = `
      <header class="view-header">
        <h1>Cambio automático</h1>
        <p>Perfil según la aplicación en primer plano</p>
      </header>
      <div class="empty-panel glass-panel">
        ${icon('info', 40)}
        <h3>Solo disponible en Windows</h3>
        <p>La detección de la ventana activa usa la API de Windows.</p>
      </div>`;
    return;
  }

  root.innerHTML = `
    <header class="view-header">
      <h1>Cambio automático</h1>
      <p>El teclado cambia de perfil solo, según la aplicación que tengas delante</p>
    </header>

    <div class="auto-grid">
      <div class="glass-panel auto-main">
        <div class="auto-head">
          <div class="card-header">${icon('bolt', 22)}<h2>Reglas</h2></div>
          <button class="switch ${view.enabled ? 'on' : ''}" data-act="toggle" title="Activar o desactivar">
            <span class="switch-knob"></span>
          </button>
        </div>

        <p class="setting-desc">
          Se evalúan de arriba abajo y gana la primera que encaje, así que pon las
          más específicas primero. La comparación no distingue mayúsculas.
        </p>

        ${view.rules.length ? `
          <div class="rule-list">
            ${view.rules.map((r, i) => `
              <div class="rule-row ${view.lastApplied === r.id ? 'is-active' : ''}">
                <span class="rule-order">${i + 1}</span>
                <input type="text" class="text-input" placeholder="p. ej. photoshop"
                       value="${escape(r.match)}" data-act="rule-match" data-id="${r.id}">
                <select class="select-input compact" data-act="rule-field" data-id="${r.id}">
                  <option value="any"     ${r.field === 'any' ? 'selected' : ''}>Programa o título</option>
                  <option value="process" ${r.field === 'process' ? 'selected' : ''}>Solo programa</option>
                  <option value="title"   ${r.field === 'title' ? 'selected' : ''}>Solo título</option>
                </select>
                <span class="rule-arrow">${icon('check', 14)}</span>
                <select class="select-input compact" data-act="rule-profile" data-id="${r.id}">
                  ${profileOptions(r.profile)}
                </select>
                <button class="tool-btn small" data-act="move-up"   data-id="${r.id}" title="Subir">↑</button>
                <button class="tool-btn small" data-act="move-down" data-id="${r.id}" title="Bajar">↓</button>
                <button class="tool-btn small danger" data-act="remove" data-id="${r.id}" title="Eliminar">${icon('trash', 14)}</button>
              </div>`).join('')}
          </div>` : `
          <div class="rule-empty">Todavía no hay reglas. Añade una para empezar.</div>`}

        <div class="row-inline mt-4">
          <button class="primary-btn" data-act="add-current">${icon('plug', 16)} Añadir la app actual</button>
          <button class="secondary-btn" data-act="add-blank">Regla vacía</button>
        </div>

        <div class="field mt-4">
          <span class="field-label">Sugerencias rápidas</span>
          <div class="chip-row">
            ${SUGGESTIONS.map((s) => `
              <button class="chip" data-act="add-suggestion" data-match="${s.match}">${s.label}</button>`).join('')}
          </div>
        </div>

        <label class="field mt-4">
          <span class="field-label">Cuando no encaje ninguna regla</span>
          <select class="select-input" data-act="fallback">${profileOptions(view.fallback ?? null, true)}</select>
        </label>
      </div>

      <div class="glass-panel auto-side">
        <div class="card-header">${icon('dashboard', 22)}<h2>Estado</h2></div>
        <div id="auto-status"></div>

        <div class="auto-legend">
          <span class="field-label">Perfiles</span>
          ${profileNames().map((n, i) => `
            <div class="legend-row ${state.activeProfileIdx === i ? 'on' : ''}">
              <span class="legend-dot" style="background:${PROFILE_META[i].accent}"></span>
              <span>${escape(n)}</span>
              ${state.activeProfileIdx === i ? '<em>activo</em>' : ''}
            </div>`).join('')}
        </div>

        <p class="setting-desc mt-4">
          La detección corre en un proceso de PowerShell que consulta la ventana
          activa cada 400 ms. Solo se manda un comando al teclado cuando el
          perfil realmente tiene que cambiar.
        </p>
        <p class="setting-desc">
          Estas reglas se guardan en tu PC, no en el teclado: no necesitan
          «Guardar en Flash».
        </p>
      </div>
    </div>`;

  renderStatus();
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
