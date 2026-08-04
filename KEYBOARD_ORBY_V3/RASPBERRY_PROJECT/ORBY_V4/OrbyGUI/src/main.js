import './styles/index.css';

import * as device from './device.js';
import { state, subscribe, notify, syncFromDevice } from './store.js';
import { hydrateIcons } from './icons.js';
import { toast } from './ui.js';
import { setNavigator } from './nav.js';
import * as wheelDial from './wheel-dial.js';
import * as variants from './variants.js';
import * as mirror from './mirror.js';
import * as updater from './updater.js';

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
  initUpdateBadge();
}

// El aviso de actualización vive al lado del estado de conexión y del guardado:
// es lo que el usuario ya mira, y sin él la versión descargada se quedaría
// esperando a un cierre real de la app que puede no llegar nunca.
function initUpdateBadge() {
  const btn = document.getElementById('btn-update');

  btn.addEventListener('click', async () => {
    if (updater.update.status !== 'downloaded') return;
    if (!confirm(`Se instalará OrbyGUI ${updater.update.newVersion}.\n\n`
               + 'La app se cerrará y volverá a abrirse sola. ¿Continuar?')) return;
    await updater.install();
  });

  updater.onChange(renderUpdateBadge);
  updater.init();
  renderUpdateBadge();
}

function renderUpdateBadge() {
  const btn = document.getElementById('btn-update');
  const label = btn.querySelector('.update-label');
  const { status, newVersion, percent } = updater.update;

  // 'checking' e 'idle' no se enseñan: comprobar sola cada seis horas no es
  // algo que el usuario tenga que ver, solo el resultado cuando hay versión.
  btn.classList.toggle('hidden', status !== 'downloading' && status !== 'downloaded');
  btn.classList.toggle('ready', status === 'downloaded');

  if (status === 'downloading') {
    label.textContent = `Descargando ${percent}%`;
    btn.title = `Bajando OrbyGUI ${newVersion}`;
  } else if (status === 'downloaded') {
    label.textContent = `Actualizar a ${newVersion}`;
    btn.title = `OrbyGUI ${newVersion} lista: haz clic para instalarla`;
  }
}

// `params` permite abrir una vista apuntando a algo concreto: es lo que usa el
// botón "Editar icono" del editor de perfiles para caer en la tecla correcta.
//
// El editor de iconos no tiene botón en la barra lateral (solo se entra desde
// una tecla concreta), así que mientras esté abierto se deja encendido el de
// Perfiles: es de donde se viene y a donde se vuelve al guardar o salir.
const NAV_FOR_VIEW = { 'view-oled': 'view-profiles' };

function switchView(target, params) {
  activeView = target;
  const navTarget = NAV_FOR_VIEW[target] || target;
  document.querySelectorAll('.nav-item').forEach((n) => n.classList.toggle('active', n.dataset.target === navTarget));
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
    // Antes de tocar el estado: en cuanto `connected` valga true, cualquier
    // notify() daría por buenos los cambios hechos sin el teclado.
    const offlineEdits = mirror.takeOfflineEdits();

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

    // Se editó algo con el teclado desenchufado: o se sube al Orby, o se
    // descarta leyendo lo que él tenga. Preguntar es obligado, porque las dos
    // opciones pisan datos reales del usuario.
    if (offlineEdits && confirm(
        'Editaste la configuración con el teclado desconectado.\n\n'
      + '¿Quieres volcar esos cambios al Orby ahora?\n\n'
      + 'Si dices que no, se descartan y se lee lo que tenga el teclado.')) {
      try {
        await mirror.push((msg) => toast(msg, 'info', 1200));
        toast('Cambios volcados al teclado');
      } catch (err) {
        toast(`No se pudieron volcar los cambios: ${err.message}`, 'error', 9000);
      }
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

  // Espejo local de los perfiles: sin él, abrir la app sin el teclado enchufado
  // no enseñaba nada y no había nada que editar. Se registra el guardado antes
  // de cargarlo para que cualquier cambio posterior quede recogido.
  mirror.watch();
  mirror.init().then((hydrated) => {
    if (!hydrated) return;
    VIEWS[activeView]?.render?.();
    toast('Perfiles cargados de la copia del PC (teclado no conectado)', 'info', 4000);
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
