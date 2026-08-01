import './styles/index.css';

import * as device from './device.js';
import { state, subscribe, notify, syncFromDevice } from './store.js';
import { hydrateIcons } from './icons.js';
import { toast } from './ui.js';
import { setNavigator } from './nav.js';
import * as wheelDial from './wheel-dial.js';
import * as variants from './variants.js';

import * as dashboard from './views/dashboard.js';
import * as profiles from './views/profiles.js';
import * as oled from './views/oled.js';
import * as auto from './views/auto.js';
import * as settings from './views/settings.js';
import * as consoleView from './views/console.js';

const VIEWS = {
  'view-dashboard': dashboard,
  'view-profiles': profiles,
  'view-oled': oled,
  'view-auto': auto,
  'view-settings': settings,
  'view-console': consoleView,
};

let activeView = 'view-dashboard';

function initChrome() {
  document.getElementById('btn-minimize').addEventListener('click', () => window.orby.minimize());
  document.getElementById('btn-maximize').addEventListener('click', () => window.orby.maximize());
  document.getElementById('btn-close').addEventListener('click', () => window.orby.close());

  document.querySelectorAll('.nav-item').forEach((item) => {
    item.addEventListener('click', () => switchView(item.dataset.target));
  });

  document.getElementById('btn-save-flash').addEventListener('click', saveToFlash);
}

// `params` permite abrir una vista apuntando a algo concreto: es lo que usa el
// botón "Editar icono" del editor de perfiles para caer en la tecla correcta.
function switchView(target, params) {
  activeView = target;
  document.querySelectorAll('.nav-item').forEach((n) => n.classList.toggle('active', n.dataset.target === target));
  document.querySelectorAll('.view').forEach((v) => v.classList.toggle('active', v.id === target));

  if (params && target === 'view-oled') oled.openTarget(params);
  else VIEWS[target]?.render?.();
}

async function saveToFlash() {
  const btn = document.getElementById('btn-save-flash');
  if (!state.connected) return;
  if (autoSaveTimer) { clearTimeout(autoSaveTimer); autoSaveTimer = null; }

  btn.disabled = true;
  try {
    // Con una variación aplicada, la RAM del teclado tiene los atajos de esa
    // app concreta. Se revierte antes de escribir para que a la Flash vaya el
    // perfil base, y se vuelve a poner después.
    await variants.withBase(async () => {
      await device.saveToFlash();
      state.dirty = false;
      notify();
    });
  } catch {
    // No se ha podido escribir todavía (p.ej. el teclado se desenchufó a
    // mitad). state.dirty sigue en true: se reintenta solo, sin que el
    // usuario tenga que volver a pulsar nada.
    toast('El teclado no confirmó el guardado, reintentando…', 'error');
    scheduleAutoSave();
  } finally {
    btn.disabled = false;
  }
}

// Guardado automático: cada cambio (marcado con markDirty) programa una
// escritura a Flash sola, sin que el usuario tenga que acordarse de pulsar
// nada. El retardo agrupa varios cambios seguidos (p.ej. reasignar varias
// teclas seguidas) en una sola escritura en vez de una por tecla.
const AUTO_SAVE_DELAY_MS = 1500;
let autoSaveTimer = null;

function scheduleAutoSave() {
  if (autoSaveTimer) clearTimeout(autoSaveTimer);
  autoSaveTimer = setTimeout(() => {
    autoSaveTimer = null;
    saveToFlash();
  }, AUTO_SAVE_DELAY_MS);
}

// El indicador de "cambios sin guardar" evita el fallo más molesto de la
// versión anterior: ajustar todo y perderlo al desenchufar. Ahora el propio
// guardado automático se encarga de que ese hueco dure como mucho
// AUTO_SAVE_DELAY_MS; el indicador es solo para que se vea que está en curso.
function renderChrome() {
  const badge = document.getElementById('connection-status-badge');
  const text = badge.querySelector('.status-text');

  if (state.connected) {
    badge.className = 'status-badge connected';
    text.textContent = state.syncing ? 'Sincronizando…' : 'Orby V4 conectado';
  } else {
    badge.className = 'status-badge disconnected';
    text.textContent = 'Desconectado';
  }

  if (state.dirty && state.connected) scheduleAutoSave();

  const save = document.getElementById('btn-save-flash');
  save.classList.toggle('has-changes', state.dirty);
  save.querySelector('.save-label').textContent = state.dirty ? 'Guardando…' : 'Guardado';
  save.disabled = !state.connected;
}

function wireDevice() {
  device.on('connected', async (info) => {
    state.connected = true;
    state.deviceInfo = info;
    notify();

    // El firmware 1.0 no conoce GET_STATE ni GET_PROFILE, así que la sincronía
    // acabaría en un tiempo de espera agotado poco explicativo.
    const fw = parseFloat(info?.fw ?? '0');
    if (fw < 2) {
      toast('Firmware antiguo detectado: flashea la versión 3.0 para usar el editor y el scroll suave', 'error', 9000);
      return;
    }
    if (fw < 3) {
      toast('Firmware 2.x: no admite crear perfiles, mandos en capa SUPER ni rueda por perfil. '
          + 'Flashea la versión 3.0 para esas funciones', 'error', 9000);
    }

    try {
      await syncFromDevice();
      VIEWS[activeView]?.render?.();
      toast('Perfiles cargados desde el teclado');
    } catch (err) {
      toast(`No se pudo leer la configuración: ${err.message}`, 'error');
    }

    // No bloquea el arranque: si el teclado lleva firmware con secuencias
    // propias, reenvía las que haya (cubre un teclado recién reflasheado, cuya
    // Flash de secuencias está vacía aunque la app ya las tuviera guardadas).
    profiles.syncAllMacrosToDevice?.().catch(() => {});
  });

  device.on('disconnected', () => {
    state.connected = false;
    state.deviceInfo = null;
    notify();
  });

  device.on('searching', () => {
    if (state.connected) return;
    const badge = document.getElementById('connection-status-badge');
    badge.className = 'status-badge searching';
    badge.querySelector('.status-text').textContent = 'Buscando USB…';
  });
}

document.addEventListener('DOMContentLoaded', () => {
  hydrateIcons();
  initChrome();
  device.init();
  wireDevice();
  setNavigator(switchView);

  // La calibración del dibujo de la rueda se lee del disco, así que llega
  // después del primer pintado: hay que rehacer los marcadores al tenerla.
  wheelDial.init().then(() => {
    dashboard.refreshMarker();
    if (activeView === 'view-profiles') profiles.render();
    if (activeView === 'view-settings') settings.render();
  });

  // Variaciones de perfil por aplicación: se leen del disco antes de que el
  // detector de ventanas empiece a pedir evaluaciones.
  variants.init().then(() => {
    if (activeView === 'view-profiles') profiles.render();
  });

  dashboard.init();
  profiles.init();
  oled.init();
  settings.init();
  consoleView.init();
  auto.init(); // asíncrono: lee la configuración local antes de pintarse

  subscribe(() => {
    renderChrome();
    settings.render();
    if (activeView === 'view-dashboard') dashboard.render();
  });

  renderChrome();
  switchView('view-dashboard');
});
