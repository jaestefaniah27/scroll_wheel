// Cambio automático de perfil según la aplicación en primer plano.
//
// El proceso principal vigila la ventana activa de Windows y nos manda
// { process, title }. Aquí se comparan esas cadenas con las reglas del usuario
// y, al primer encaje, se manda SET_PROFILE al teclado.
//
// Cada regla es una tarjeta: programa + perfil, que es todo lo que hace falta
// para el caso normal. Los ajustes finos (contra qué se compara y el orden de
// prioridad) viven dentro de la propia tarjeta, no en una fila de columnas.

import * as device from '../device.js';
import { state, notify, subscribe, profileMeta } from '../store.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';
import * as variants from '../variants.js';

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
  variants.setEnabled(view.enabled);
  variants.onChange(() => renderStatus());

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

  // Los nombres de los perfiles llegan del firmware; hay que repintar las
  // tarjetas cuando terminan de sincronizarse o cuando se crea o borra uno.
  let lastProfileCount = 0;
  subscribe(() => {
    if (state.profiles.length !== lastProfileCount) {
      lastProfileCount = state.profiles.length;
      clampTargets();
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

// Borrar un perfil desplaza los índices, así que una regla puede quedar
// apuntando a un perfil que ya no existe.
function clampTargets() {
  const max = state.profiles.length - 1;
  if (max < 0) return;
  let changed = false;
  for (const rule of view.rules) {
    if (rule.profile > max) { rule.profile = max; changed = true; }
  }
  if (view.fallback !== null && view.fallback !== undefined && view.fallback > max) {
    view.fallback = max;
    changed = true;
  }
  if (changed) persist();
}

async function evaluate(info) {
  if (!view.enabled || !state.connected) return;

  // El detector alimenta dos cosas distintas: qué perfil se activa (esto) y qué
  // variación de ese perfil se aplica encima (variants.js).
  variants.setForeground(info);

  const rule = matchRule(info);
  const target = rule ? rule.profile : view.fallback;

  try {
    if (target !== null && target !== undefined && target !== state.activeProfileIdx) {
      await device.setProfile(target);
      state.activeProfileIdx = target;
      notify();
    }
    view.lastApplied = rule?.id ?? 'fallback';
    await variants.evaluate();
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
    variants.setEnabled(view.enabled);
    await persist();
    if (view.enabled) {
      const ok = await window.orby.foreground.start();
      if (!ok) {
        toast('No se pudo iniciar el detector de aplicaciones', 'error');
        view.enabled = false;
        variants.setEnabled(false);
        await persist();
      } else {
        view.current = await window.orby.foreground.current();
        evaluate(view.current);
      }
    } else {
      await window.orby.foreground.stop();
      // Al apagar el detector el teclado se queda con la variación puesta.
      await variants.revert();
    }
    render();

  } else if (act === 'add-current') {
    const proc = view.current?.process;
    if (!proc) { toast('Aún no se ha detectado ninguna ventana', 'error'); return; }
    if (view.rules.some((r) => r.match === proc.toLowerCase())) {
      toast('Esa aplicación ya tiene tarjeta', 'info');
      return;
    }
    addRule(proc.toLowerCase(), state.activeProfileIdx);

  } else if (act === 'add-suggestion') {
    addRule(el.dataset.match, state.activeProfileIdx);

  } else if (act === 'add-blank') {
    addRule('', state.activeProfileIdx);

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
  view.rules.push({
    id: `r${Date.now()}${Math.random().toString(36).slice(2, 6)}`,
    match, profile: profile ?? 0, field: 'any',
  });
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
    // `change` en el campo de texto salta al salir de él, así que repintar aquí
    // no interrumpe la escritura y actualiza el título de la tarjeta.
    render();

  } else if (el.dataset.act === 'fallback') {
    view.fallback = el.value === '' ? null : Number(el.value);
    persist();
    render();
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
  const variant = variants.activeVariant();

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
      ${variant ? `
        <span class="auto-verdict">
          Variación <b>${escape(variant.name)}</b> aplicada sobre
          ${escape(names[variant.profile] ?? '?')}
          (${variants.countOverrides(variant)}
           ${variants.countOverrides(variant) === 1 ? 'cambio' : 'cambios'})
        </span>` : ''}
    </div>
    ${view.error ? `<p class="auto-error">${escape(view.error)}</p>` : ''}`;
}

// Una tarjeta por aplicación: nombre del programa, perfil que se aplica y, en
// pequeño, contra qué se compara y su prioridad.
function renderRuleCard(rule, index) {
  const meta = profileMeta(rule.profile);
  const names = profileNames();
  const title = rule.match || 'Sin programa';

  return `
    <div class="rule-card ${view.lastApplied === rule.id ? 'is-active' : ''}" style="--accent:${meta.accent}">
      <div class="rule-card-head">
        <span class="rule-app-icon">${icon(meta.icon, 18)}</span>
        <span class="rule-app-name" title="${escape(title)}">${escape(title)}</span>
        <span class="rule-priority">#${index + 1}</span>
      </div>

      <label class="field">
        <span class="field-label">Programa o texto de la ventana</span>
        <input type="text" class="text-input" placeholder="p. ej. photoshop"
               value="${escape(rule.match)}" data-act="rule-match" data-id="${rule.id}">
      </label>

      <label class="field">
        <span class="field-label">Usa el perfil</span>
        <select class="select-input" data-act="rule-profile" data-id="${rule.id}">
          ${profileOptions(rule.profile)}
        </select>
      </label>

      <div class="rule-card-foot">
        <select class="select-input compact" data-act="rule-field" data-id="${rule.id}">
          <option value="any"     ${rule.field === 'any' ? 'selected' : ''}>Programa o título</option>
          <option value="process" ${rule.field === 'process' ? 'selected' : ''}>Solo programa</option>
          <option value="title"   ${rule.field === 'title' ? 'selected' : ''}>Solo título</option>
        </select>
        <div class="rule-card-btns">
          <button class="tool-btn small" data-act="move-up"   data-id="${rule.id}" title="Más prioridad">↑</button>
          <button class="tool-btn small" data-act="move-down" data-id="${rule.id}" title="Menos prioridad">↓</button>
          <button class="tool-btn small danger" data-act="remove" data-id="${rule.id}" title="Eliminar">${icon('trash', 14)}</button>
        </div>
      </div>

      ${view.lastApplied === rule.id
        ? `<span class="rule-card-live">Aplicando ${escape(names[rule.profile] ?? '?')}</span>` : ''}
    </div>`;
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

  const fallbackName = (view.fallback === null || view.fallback === undefined)
    ? null : (profileNames()[view.fallback] ?? '?');

  root.innerHTML = `
    <header class="view-header">
      <h1>Cambio automático</h1>
      <p>El teclado cambia de perfil solo, según la aplicación que tengas delante</p>
    </header>

    <div class="auto-grid">
      <div class="auto-main-col">
        <div class="glass-panel auto-main">
          <div class="auto-head">
            <div class="card-header">${icon('bolt', 22)}<h2>Aplicaciones</h2></div>
            <button class="switch ${view.enabled ? 'on' : ''}" data-act="toggle" title="Activar o desactivar">
              <span class="switch-knob"></span>
            </button>
          </div>

          <div class="row-inline">
            <button class="primary-btn" data-act="add-current">${icon('plug', 16)} Añadir la app actual</button>
            <button class="secondary-btn" data-act="add-blank">${icon('plus', 16)} Tarjeta vacía</button>
          </div>

          <div class="field mt-4">
            <span class="field-label">Añadir rápido</span>
            <div class="chip-row">
              ${SUGGESTIONS.map((s) => `
                <button class="chip" data-act="add-suggestion" data-match="${s.match}">${s.label}</button>`).join('')}
            </div>
          </div>

          <div class="rule-cards">
            ${view.rules.length
              ? view.rules.map(renderRuleCard).join('')
              : `<div class="rule-empty">
                   Todavía no hay tarjetas. Añade la aplicación que tengas abierta y elige su perfil.
                 </div>`}
          </div>

          ${view.rules.length > 1 ? `
            <p class="setting-desc mt-4">
              Si dos tarjetas encajan a la vez gana la de más arriba: usa las flechas para
              poner primero las más específicas. La comparación no distingue mayúsculas.
            </p>` : ''}
        </div>

        <div class="glass-panel auto-main fallback-card">
          <div class="card-header">${icon('profiles', 22)}<h2>Perfil por defecto</h2></div>
          <div class="fallback-row">
            <label class="field">
              <span class="field-label">Cuando no se detecta ninguna de las apps añadidas</span>
              <select class="select-input" data-act="fallback">${profileOptions(view.fallback ?? null, true)}</select>
            </label>
            <span class="fallback-pill ${fallbackName ? 'on' : ''}">
              ${fallbackName ? escape(fallbackName) : 'No cambiar'}
            </span>
          </div>
        </div>
      </div>

      <div class="glass-panel auto-side">
        <div class="card-header">${icon('dashboard', 22)}<h2>Estado</h2></div>
        <div id="auto-status"></div>

        <div class="auto-legend">
          <span class="field-label">Perfiles</span>
          ${profileNames().map((n, i) => `
            <div class="legend-row ${state.activeProfileIdx === i ? 'on' : ''}">
              <span class="legend-dot" style="background:${profileMeta(i).accent}"></span>
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
          Estas tarjetas se guardan en tu PC, no en el teclado: no necesitan
          «Guardar en Flash».
        </p>

        <div class="divider"></div>
        <p class="setting-desc">
          <strong>¿Y si solo cambia un atajo?</strong> Aquí eliges <em>qué perfil</em>
          se activa con cada app. Si lo que quieres es que un perfil concreto tenga
          un par de teclas distintas en una app —«seleccionar todo» con Ctrl+E en vez
          de Ctrl+A, por ejemplo— no hace falta duplicar el perfil: crea una
          <strong>variación</strong> desde <em>Perfiles y macros</em>. Guarda solo las
          diferencias y se aplica sola encima del perfil cuando esa app está delante.
        </p>
      </div>
    </div>`;

  renderStatus();
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
