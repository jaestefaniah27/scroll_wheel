// Ajustes de hardware y persistencia.

import * as device from '../device.js';
import { state, markDirty, syncFromDevice } from '../store.js';
import { toast } from '../ui.js';
import { runExport, runImport } from '../backup.js';
import * as cache from '../oled-cache.js';
import * as wheelDial from '../wheel-dial.js';
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
  document.querySelectorAll('#timeout-selector .opt-btn').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.val) === state.timeout);
  });

  const list = document.getElementById('device-info-list');
  if (!list) return;

  if (state.connected) {
    const info = state.deviceInfo || {};
    list.innerHTML = `
      <li><span class="lbl">Dispositivo</span><span class="val">${info.device || 'ORBY_V4'}</span></li>
      <li><span class="lbl">Firmware</span><span class="val">${info.fw || '?'}</span></li>
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
