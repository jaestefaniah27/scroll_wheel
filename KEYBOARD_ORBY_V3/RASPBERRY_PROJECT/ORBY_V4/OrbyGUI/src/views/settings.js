// Ajustes: dispositivo, persistencia y la propia aplicación.

import * as device from '../device.js';
import * as updater from '../updater.js';
import { state, markDirty, syncFromDevice } from '../store.js';
import { toast, requireDevice } from '../ui.js';
import { runExport, runImport } from '../backup.js';
import * as cache from '../oled-cache.js';
import * as wheelDial from '../wheel-dial.js';
import * as plugins from '../plugins.js';
import * as compat from '../compat.js';
import * as firmware from '../firmware.js';
import { icon } from '../icons.js';
import * as dashboard from './dashboard.js';

// Giro acumulado de la rueda para la prueba en vivo de esta tarjeta.
let liveTotal = 0;
let liveDecay = null;

// Deshabilita los botones mientras hay una copia en marcha: leer los 20 huecos
// de icono de cada perfil lleva su tiempo y pulsar dos veces solaparía comandos
// en el CDC.
function backupBusy(message) {
  const status = document.getElementById('backup-status');
  const save = document.getElementById('btn-backup-save');
  const load = document.getElementById('btn-backup-load');
  if (status) status.textContent = message || '';
  if (save) save.disabled = Boolean(message);
  if (load) load.disabled = Boolean(message);
}

export function init() {
  document.getElementById('btn-backup-save').addEventListener('click', () => runExport(backupBusy));
  document.getElementById('btn-backup-load').addEventListener('click', () => runImport(backupBusy));

  document.querySelectorAll('#timeout-selector .opt-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const val = Number(btn.dataset.val);
      if (!requireDevice()) return;
      try {
        await device.setTimeout_(val);
        state.timeout = val;
        markDirty();
        render();
      } catch {
        toast('El teclado no confirmó el tiempo de reposo', 'error');
      }
    });
  });

  initAutostart();
  initPlugins();

  document.getElementById('btn-reconnect').addEventListener('click', () => {
    window.orby.reconnect();
    toast('Buscando el dispositivo…', 'info');
  });

  document.getElementById('btn-resync').addEventListener('click', async () => {
    try {
      await syncFromDevice();
      toast('Configuración releída del teclado');
    } catch {
      toast('No se pudo leer la configuración', 'error');
    }
  });

  document.getElementById('btn-factory-reset').addEventListener('click', async () => {
    if (!requireDevice()) return;
    if (!confirm('Se restaurarán los perfiles, iconos y la calibración de fábrica en la memoria del teclado.\n\nEl cambio no será permanente hasta que pulses "Guardar en Flash".\n\n¿Continuar?')) return;
    try {
      await device.resetDefaults();
      // Vuelven los cuatro perfiles de fábrica: los índices y los iconos que
      // tuviéramos leídos ya no valen.
      cache.invalidate();
      await syncFromDevice();
      markDirty();
      toast('Valores de fábrica restaurados (sin guardar todavía)', 'info');
    } catch {
      toast('No se pudo restaurar la configuración', 'error');
    }
  });

  initWheelCalib();
  initAppCard();
  initFirmwareCard();
}

// ================= Aplicación (versión y actualizaciones) =================
function initAppCard() {
  const check = document.getElementById('btn-check-update');
  const install = document.getElementById('btn-install-update');
  if (!check || !install) return;

  check.addEventListener('click', async () => {
    if (updater.update.status === 'dev') {
      toast('En modo desarrollo no hay actualizaciones que buscar', 'info');
      return;
    }
    await updater.check();
    // El resultado llega por el canal de estado; esto solo confirma el clic.
    toast('Buscando actualizaciones…', 'info');
  });

  install.addEventListener('click', () => updater.install());

  updater.onChange(renderAppCard);
  renderAppCard();
}

function renderAppCard() {
  const version = document.getElementById('app-version');
  const status = document.getElementById('app-update-status');
  const install = document.getElementById('btn-install-update');
  if (!version || !status || !install) return;

  version.textContent = updater.update.version || '—';
  status.textContent = updater.describe();
  status.style.color = updater.update.status === 'error' ? 'var(--danger)' : '';
  install.classList.toggle('hidden', updater.update.status !== 'downloaded');
}

// ================= Firmware del teclado =================
// Actualizar el teclado no es como actualizar la app: hay que reiniciarlo en el
// cargador de la ROM, y mientras dura la copia el teclado *no existe* (aparece
// una unidad USB en su lugar). Por eso esta tarjeta avisa antes de empezar y
// deja el estado a la vista durante todo el proceso.
function initFirmwareCard() {
  const card = document.getElementById('settings-firmware');
  if (!card) return;

  // Sin preload (la interfaz abierta en un navegador para maquetar) no hay
  // manera de flashear nada: la tarjeta sobra.
  if (!firmware.available()) {
    card.classList.add('hidden');
    return;
  }

  document.getElementById('btn-fw-check').addEventListener('click', async () => {
    const st = await firmware.check();
    // Un fallo al consultar GitHub no es "no hay firmware": decir eso mandaba a
    // buscar un problema que no existe (la release estaba publicada).
    if (st?.status === 'error') { toast(st.error, 'error', 6000); return; }
    if (st?.status === 'idle' && !st.available) {
      toast(st.latest
        ? `El teclado ya tiene el último firmware (${st.latest.version})`
        : 'No hay firmware publicado que esta versión de la app sepa instalar', 'info');
    }
  });

  document.getElementById('btn-fw-update').addEventListener('click', async () => {
    if (!requireDevice()) return;

    // Guardar primero, y no después: el reinicio en el cargador se lleva por
    // delante lo que hubiera solo en RAM, y las variaciones por aplicación
    // viven exactamente ahí (ver variants.js).
    if (state.dirty) {
      toast('Guarda los cambios en Flash antes de actualizar el firmware', 'error', 6000);
      return;
    }

    const manual = !compat.supports(state.deviceInfo, 'bootsel');
    const aviso = manual
      ? 'Este firmware no sabe reiniciarse solo: cuando te lo pida, desenchufa el teclado y vuelve a enchufarlo con BOOTSEL pulsado.\n\n'
      : '';
    if (!confirm(`${aviso}El teclado dejará de funcionar durante la copia (unos segundos). No lo desconectes.\n\n¿Actualizar a la ${firmware.fw.latest?.version}?`)) return;

    await firmware.update();
    if (firmware.fw.status === 'done') toast('Firmware actualizado');
    else if (firmware.fw.status === 'error') toast(firmware.fw.error, 'error', 9000);
  });

  document.getElementById('btn-fw-cancel').addEventListener('click', () => {
    firmware.cancel();
    toast('Actualización cancelada', 'info');
  });

  firmware.onChange(renderFirmwareCard);
  firmware.init().then(renderFirmwareCard);
}

function renderFirmwareCard() {
  const current = document.getElementById('fw-current');
  if (!current) return;

  const st = firmware.fw;
  const latest = document.getElementById('fw-latest');
  const status = document.getElementById('fw-status');
  const btnUpdate = document.getElementById('btn-fw-update');
  const btnCheck = document.getElementById('btn-fw-check');
  const btnCancel = document.getElementById('btn-fw-cancel');

  // La versión que manda es la del teclado que hay delante ahora mismo, no la
  // que guardó el proceso principal la última vez que se comprobó.
  current.textContent = state.connected ? (state.deviceInfo?.fw || '?') : 'sin teclado';
  latest.textContent = st.latest?.version || '—';
  status.textContent = firmware.describe();
  status.style.color = st.status === 'error' ? 'var(--danger)' : '';

  btnCheck.disabled = firmware.busy();
  btnCancel.classList.toggle('hidden', !['downloading', 'bootsel'].includes(st.status));
  // Se ofrece cuando hay algo que instalar y hay teclado al que instalárselo.
  btnUpdate.classList.toggle('hidden', !st.latest || !state.connected || firmware.busy());
  // Solo la etiqueta: escribir sobre el botón entero se llevaría por delante el
  // <span data-icon>, que ya está hidratado y no se vuelve a pintar.
  document.getElementById('fw-update-label').textContent =
    st.available ? `Actualizar a ${st.latest.version}` : 'Reinstalar firmware';
}

// ================= Autoarranque con Windows =================
// El estado real vive en el registro de Windows (app.setLoginItemSettings), no
// en config.json, así que se pregunta al proceso principal en vez de guardarlo
// aparte: así el botón nunca puede desincronizarse de lo que Windows hace.
async function initAutostart() {
  const btn = document.getElementById('btn-autostart');
  if (!btn) return;

  btn.classList.toggle('on', await window.orby.autostart.get());

  btn.addEventListener('click', async () => {
    const enabled = await window.orby.autostart.set(!btn.classList.contains('on'));
    btn.classList.toggle('on', enabled);
    toast(enabled ? 'OrbyGUI arrancará con Windows' : 'Autoarranque desactivado');
  });
}

// ================= Complementos =================
// La lista de instalados y, debajo, una tarjeta por cada uno que pida ajustes
// propios. Los ajustes se declaran (nombre, tipo, valor por defecto) y se
// pintan solos: la app no sabe qué es una "dirección de lámpara", solo que ese
// complemento pide un texto llamado así.
//
// Las acciones de los complementos no pasan por esta vista: las dispara el
// firmware con MACRO:<id> y las ejecuta el proceso principal (ver
// electron/plugins.js).
function initPlugins() {
  const install = document.getElementById('btn-plugin-install');
  const folder = document.getElementById('btn-plugin-folder');
  const list = document.getElementById('plugin-list');
  const cards = document.getElementById('plugin-settings-cards');
  if (!install || !list || !cards) return;

  install.addEventListener('click', async () => {
    install.disabled = true;
    const res = await window.orby.plugins.install();
    install.disabled = false;
    if (res.canceled) return;
    if (!res.ok) {
      toast(`No se pudo instalar: ${res.error}`, 'error', 6000);
      return;
    }
    await plugins.refresh();
    toast(`Complemento «${res.plugin.name}» instalado`);
  });

  folder?.addEventListener('click', () => window.orby.plugins.openFolder());

  list.addEventListener('click', async (e) => {
    const el = e.target.closest('[data-act]');
    if (!el) return;
    const id = el.dataset.plugin;

    if (el.dataset.act === 'plugin-enable') {
      const p = plugins.byId(id);
      await window.orby.plugins.setEnabled(id, !p?.enabled);
      await plugins.refresh();
    } else if (el.dataset.act === 'plugin-remove') {
      const p = plugins.byId(id);
      if (!confirm(`Se desinstalará «${p?.name || id}».\n\n`
                 + 'Las teclas y mandos que lo usaran dejarán de hacer nada, pero conservarán su '
                 + 'ajuste: si vuelves a instalarlo, volverán a funcionar.\n\n¿Continuar?')) return;
      const res = await window.orby.plugins.uninstall(id);
      if (!res.ok) {
        toast(`No se pudo desinstalar: ${res.error}`, 'error', 5000);
        return;
      }
      await plugins.refresh();
      toast('Complemento desinstalado');
    }
  });

  // Cada campo se guarda al salir de él, como la dirección de la lámpara hacía
  // antes: escribir a cada tecla dispararía una escritura del archivo por letra.
  cards.addEventListener('change', (e) => {
    const el = e.target.closest('[data-plugin][data-key]');
    if (!el) return;
    const value = el.type === 'number' ? Number(el.value) : el.value;
    window.orby.plugins.setSettings(el.dataset.plugin, { [el.dataset.key]: value });
  });

  cards.addEventListener('click', async (e) => {
    const el = e.target.closest('[data-act]');
    if (!el) return;

    if (el.dataset.act === 'plugin-toggle-field') {
      const on = !el.classList.contains('on');
      el.classList.toggle('on', on);
      window.orby.plugins.setSettings(el.dataset.plugin, { [el.dataset.key]: on });
    } else if (el.dataset.act === 'plugin-test') {
      await runPluginTest(el.dataset.plugin, el);
    }
  });

  plugins.onChange(renderPlugins);
  renderPlugins();
}

// La comprobación se hace con lo que hay escrito en los campos ahora mismo, no
// con lo guardado: probar una dirección nueva sin haber salido antes del campo
// es justo lo que uno intenta hacer cuando algo no responde.
async function runPluginTest(id, btn) {
  const status = document.getElementById(`plugin-status-${id}`);
  const values = {};
  document.querySelectorAll(`#plugin-settings-cards [data-plugin="${id}"][data-key]`).forEach((el) => {
    values[el.dataset.key] = el.classList?.contains('switch')
      ? el.classList.contains('on')
      : (el.type === 'number' ? Number(el.value) : el.value);
  });
  window.orby.plugins.setSettings(id, values);

  if (status) status.textContent = 'Probando…';
  btn.disabled = true;
  const res = await window.orby.plugins.test(id, values);
  btn.disabled = false;

  if (status) status.textContent = res.ok ? res.detail : `Sin respuesta (${res.error})`;
  toast(res.ok ? 'El complemento responde' : 'El complemento no responde', res.ok ? 'success' : 'error');
}

function renderPlugins() {
  renderPluginList();
  renderPluginCards();
}

function renderPluginList() {
  const list = document.getElementById('plugin-list');
  if (!list) return;
  const all = plugins.all();

  if (!all.length) {
    list.innerHTML = '<p class="plugin-empty">Todavía no hay ninguno instalado.</p>';
    return;
  }

  list.innerHTML = all.map((p) => `
    <div class="plugin-row ${p.error ? 'has-error' : ''}">
      <span class="plugin-row-icon">${icon(p.icon, 18)}</span>
      <div class="plugin-row-main">
        <span class="plugin-row-name">${escape(p.name)} <b>${escape(p.version)}</b></span>
        <span class="plugin-row-desc">${escape(p.error ? `No se pudo cargar: ${p.error}` : p.description)}</span>
      </div>
      <div class="plugin-row-actions">
        ${p.error ? '' : `
          <button class="switch ${p.enabled ? 'on' : ''}" data-act="plugin-enable" data-plugin="${p.id}"
                  title="${p.enabled ? 'Desactivar' : 'Activar'}">
            <span class="switch-knob"></span>
          </button>`}
        <button class="tool-btn small danger" data-act="plugin-remove" data-plugin="${p.id}"
                title="Desinstalar">${icon('trash', 14)}</button>
      </div>
    </div>`).join('');
}

function renderPluginCards() {
  const host = document.getElementById('plugin-settings-cards');
  if (!host) return;

  // Solo los activos y con ajustes que pedir: uno desactivado no debe dejar una
  // tarjeta que parezca que sigue haciendo algo.
  const conAjustes = plugins.active().filter((p) => p.settings?.fields?.length || p.settings?.hasTest);

  host.innerHTML = conAjustes.map((p) => `
    <div class="settings-card glass-panel">
      <div class="card-header">
        <span>${icon(p.icon, 22)}</span>
        <h2>${escape(p.name)}</h2>
      </div>
      <div class="setting-body">
        ${(p.settings.fields || []).map((f) => renderPluginField(p, f)).join('')}
        ${p.settings.hasTest ? `
          <div class="row-inline">
            <button class="secondary-btn" data-act="plugin-test" data-plugin="${p.id}">Probar</button>
            <span class="setting-desc" id="plugin-status-${p.id}"></span>
          </div>` : ''}
        ${p.settings.description ? `<p class="setting-desc">${escape(p.settings.description)}</p>` : ''}
      </div>
    </div>`).join('');
}

function renderPluginField(p, f) {
  const value = p.values?.[f.key];
  const attrs = `data-plugin="${p.id}" data-key="${escape(f.key)}"`;

  if (f.type === 'toggle') {
    return `
      <div class="row-inline">
        <button class="switch ${value ? 'on' : ''}" data-act="plugin-toggle-field" ${attrs}>
          <span class="switch-knob"></span>
        </button>
        <span class="switch-label">${escape(f.label)}</span>
      </div>
      ${f.hint ? `<p class="setting-desc">${escape(f.hint)}</p>` : ''}`;
  }

  const control = f.type === 'select'
    ? `<select class="select-input" ${attrs}>
         ${f.options.map((o) => `
           <option value="${escape(o.value)}" ${String(value) === o.value ? 'selected' : ''}>${escape(o.label)}</option>`).join('')}
       </select>`
    : `<input class="text-input" type="${f.type === 'number' ? 'number' : f.type}" ${attrs}
              value="${escape(value ?? '')}" placeholder="${escape(f.placeholder)}" spellcheck="false">`;

  return `
    <label class="field">
      <span class="field-label">${escape(f.label)}</span>
      ${control}
    </label>
    ${f.hint ? `<p class="setting-desc">${escape(f.hint)}</p>` : ''}`;
}

// Lo que llega de un complemento es texto de terceros: acaba en innerHTML, así
// que no puede entrar sin escapar.
function escape(s) {
  return String(s ?? '').replace(/[&<>"']/g, (c) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// ================= Calibración de la rueda =================
// Cómo se dibuja la rueda magnética en pantalla: no toca el teclado, solo hace
// que el dibujo (forma del marcador, sentido, desfase) coincida con el
// marcador físico que lleve pegado la tapa. Vive aquí, no en Perfiles, porque
// es un ajuste de montaje del PC, no de un perfil ni de una capa.
function initWheelCalib() {
  const box = document.getElementById('settings-wheel-calib-body');
  if (!box) return;

  box.addEventListener('click', (e) => {
    const el = e.target.closest('[data-act]');
    if (!el) return;
    const act = el.dataset.act;

    if (act === 'dial-marker') {
      applyDialSetting({ marker: el.dataset.marker });
    } else if (act === 'dial-invert') {
      applyDialSetting({ invert: !wheelDial.dial.invert });
    } else if (act === 'dial-nudge') {
      applyDialSetting({ offsetDeg: wheelDial.normalize(wheelDial.dial.offsetDeg + Number(el.dataset.d)) });
    } else if (act === 'dial-align') {
      wheelDial.alignHere();
      renderWheelCalib();
      dashboard.refreshMarker();
    }
  });

  // El desfase se ve al momento en la aguja, sin repintar la tarjeta entera.
  box.addEventListener('input', (e) => {
    if (e.target.dataset.act !== 'dial-offset') return;
    const deg = Number(e.target.value);
    wheelDial.setDial({ offsetDeg: deg });
    const label = document.getElementById('dial-offset-val');
    if (label) label.textContent = `${deg}°`;
  });

  // El dibujo de la rueda lo lleva wheel-dial.js con el ángulo absoluto del
  // sensor, así que la aguja se queda donde esté la rueda de verdad.
  wheelDial.onUpdate(paintDialMarker);

  device.on('telemetry', (line) => {
    if (line.startsWith('WHEEL:')) onWheelTelemetry(parseInt(line.slice(6), 10));
  });

  renderWheelCalib();
}

function paintDialMarker(deg) {
  const marker = document.getElementById('settings-wheel-needle');
  if (marker) marker.style.transform = `rotate(${deg}deg)`;
}

function onWheelTelemetry(delta) {
  if (!Number.isFinite(delta)) return;
  liveTotal += delta;

  const readout = document.getElementById('wheel-calib-readout');
  if (!readout) return;

  const detentsPerRev = state.scroll.detentsPerRev || 60;
  const detents = (liveTotal * detentsPerRev) / 4096;
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
  renderWheelCalib();
  // El dashboard tiene el mismo marcador y también hay que rehacerlo.
  dashboard.refreshMarker();
}

function renderWheelCalib() {
  const box = document.getElementById('settings-wheel-calib-body');
  if (!box) return;
  const d = wheelDial.dial;

  box.innerHTML = `
    <p class="setting-desc">
      Cómo se dibuja la rueda magnética en pantalla. No afecta al teclado: solo hace que el
      dibujo coincida con el marcador que lleve pegado la tapa.
    </p>

    <div class="wheel-dial">
      <div class="wheel-dial-face">${wheelDial.markerHtml('settings-wheel-needle')}</div>
    </div>
    <p class="wheel-live-readout" id="wheel-calib-readout">Gira la rueda para comprobar el recorrido</p>

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

  paintDialMarker(wheelDial.angle());
}

export function render() {
  renderAppCard();
  renderFirmwareCard();

  document.querySelectorAll('#timeout-selector .opt-btn').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.val) === state.timeout);
  });

  const list = document.getElementById('device-info-list');
  if (!list) return;

  if (state.connected) {
    const info = state.deviceInfo || {};
    // La versión de firmware sola no dice nada: lo que importa es si la app
    // puede con ella. El veredicto sale de compat.js, el mismo que decide qué
    // comandos se mandan, para que la pantalla no pueda contar otra cosa.
    const verdict = compat.check(info);
    const verdictColor = verdict.level === 'ok' ? 'var(--ok, inherit)' : 'var(--danger)';
    list.innerHTML = `
      <li><span class="lbl">Dispositivo</span><span class="val">${info.device || 'ORBY_V4'}</span></li>
      <li><span class="lbl">Firmware</span><span class="val">${info.fw || '?'}</span></li>
      <li><span class="lbl">Compatibilidad</span><span class="val" style="color:${verdictColor}"
          title="${verdict.detail}">${verdict.level === 'ok' ? 'al día' : verdict.title}</span></li>
      <li><span class="lbl">Puerto</span><span class="val">${info.port || '—'}</span></li>
      <li><span class="lbl">Teclas / OLEDs</span><span class="val">${info.keys || 12} / ${info.oleds || 10}</span></li>
      <li><span class="lbl">Perfiles</span><span class="val">${state.profiles.length} / ${state.maxProfiles}</span></li>
      <li><span class="lbl">Modo</span><span class="val">${state.deviceMode}</span></li>
      <li><span class="lbl">Scroll alta res.</span><span class="val">${state.scroll.hires ? 'sí' : 'no'}</span></li>`;
  } else {
    list.innerHTML = '<li><span class="lbl">Estado</span><span class="val" style="color:var(--danger)">Desconectado</span></li>';
  }

  renderWheelCalib();
}
