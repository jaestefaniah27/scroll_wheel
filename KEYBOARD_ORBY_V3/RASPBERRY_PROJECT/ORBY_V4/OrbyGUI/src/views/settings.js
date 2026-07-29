// Ajustes de hardware y persistencia.

import * as device from '../device.js';
import { state, markDirty, syncFromDevice } from '../store.js';
import { toast } from '../ui.js';
import { runExport, runImport } from '../backup.js';
import * as cache from '../oled-cache.js';

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
}
