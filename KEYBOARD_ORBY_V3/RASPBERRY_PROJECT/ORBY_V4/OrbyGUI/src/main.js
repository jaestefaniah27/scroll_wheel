import './styles/index.css';

import * as device from './device.js';
import { state, subscribe, notify, syncFromDevice, applyDeviceContext } from './store.js';
import * as cache from './oled-cache.js';
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

// Se resuelve cuando la copia local ya está en memoria. El teclado puede
// conectarse antes de que termine de leerse del disco, y comparar huellas
// contra una lista de perfiles vacía haría descargarlo todo sin motivo.
let mirrorReady = Promise.resolve();

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

  // Sin teclado la app enseña la copia local, pero no deja tocarla (ver
  // requireDevice en ui.js). Hay que decirlo, o parece que la app se ha quedado
  // tonta cuando un clic no hace nada.
  document.getElementById('readonly-banner').classList.toggle('hidden', state.connected);
  document.body.classList.toggle('read-only', !state.connected);

  if (state.dirty && state.connected) scheduleAutoSave();

  const save = document.getElementById('btn-save-flash');
  save.classList.toggle('has-changes', state.dirty);
  save.querySelector('.save-label').textContent = state.dirty ? 'Guardando…' : 'Guardado';
  save.disabled = !state.connected;
}

// Todos los iconos de todas las páginas de todos los perfiles, de una vez y en
// segundo plano. La app ya se puede usar mientras tanto; lo que llega se va
// pintando. Así cambiar de perfil o de pestaña no dispara veinte peticiones ni
// deja los huecos en blanco durante un segundo.
//
// Sin GET_OLED_PG (firmware anterior al 4.1) no se puede: hay que conformarse
// con los del perfil activo, que es lo que había.
function preloadIcons(canHash) {
  if (!canHash) {
    cache.loadProfile(state.activeProfileIdx);
    return;
  }

  cache.preloadAll((done, total) => {
    if (done === total) toast('Iconos descargados', 'info', 1500);
  }).then(() => {
    VIEWS[activeView]?.render?.();
    // El espejo del disco se guarda con los iconos ya dentro: así la próxima
    // apertura sin teclado los enseña, y la próxima conexión puede compararlos.
    mirror.save();
  }).catch((err) => {
    console.error('No se pudieron precargar los iconos:', err);
  });
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

    // La copia del PC tiene que estar cargada antes de comparar huellas: es
    // contra ella contra lo que se decide qué hace falta descargar.
    await mirrorReady;

    // GET_HASH y GET_OLED_PG los añadió el firmware 4.1. Con uno anterior se
    // sigue haciendo lo de siempre: descargarlo todo y leer los iconos de la
    // página activa según hagan falta.
    const canHash = info?.hash === '1';

    try {
      let expected = null;
      if (canHash) {
        const hashes = await device.getHash();
        if (hashes?.all) expected = hashes.per;
      }

      const reloaded = await syncFromDevice({
        expected,
        iconOf: cache.get,
        onProgress: (i, total) => toast(`Leyendo perfil ${i + 1} de ${total}…`, 'info', 900),
      });

      // Un perfil que se ha vuelto a leer es un perfil que había cambiado: sus
      // iconos en caché son los de antes. Y los de los perfiles que la copia
      // traía de más ya no valen para nada.
      for (const idx of reloaded) cache.dropProfile(idx);
      cache.pruneBeyond(state.profiles.length);

      VIEWS[activeView]?.render?.();
      toast(reloaded.length
        ? 'Perfiles cargados desde el teclado'
        : 'La copia del PC está al día: no ha hecho falta descargar nada');
    } catch (err) {
      toast(`No se pudo leer la configuración: ${err.message}`, 'error');
      return;
    }

    preloadIcons(canHash);

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

  // El teclado anuncia con EV:CTX el perfil y la página que tiene puestos. Los
  // cambia él solo (su menú de perfiles, el gestor de páginas, la pulsación
  // corta del botón de menú, una tecla de salto a página) y hasta que lo dijo,
  // la app se quedaba enseñando lo anterior; peor todavía, seguía editando lo
  // que ella creía activo y los iconos acababan en otra página.
  device.on('telemetry', (line) => {
    if (!line.startsWith('EV:CTX:')) return;
    const [profileIdx, pageIdx, pageCount] = line.slice(7).split(':').map(Number);

    if (applyDeviceContext(profileIdx, pageIdx, pageCount)) {
      // Los iconos son de cada página, así que los de la nueva hay que leerlos.
      cache.loadProfile(profileIdx);
      VIEWS[activeView]?.render?.();
    }

    // Siempre, venga el cambio del teclado o de la propia app (en cuyo caso el
    // estado ya estaba puesto y lo de arriba no hace nada): una variación
    // aplicada solo existe en la RAM de la página en la que se escribió, y hay
    // que quitarla de allí antes de que el guardado a Flash se la lleve como si
    // fuese el perfil base.
    variants.onContextChanged().catch(() => {});
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
  // no enseñaba nada. Se registra el guardado antes de cargarlo para que
  // cualquier cambio posterior quede recogido.
  //
  // La promesa se guarda porque el enganche de conexión la espera: hasta que la
  // copia no está cargada no se puede comparar con las huellas del teclado, y
  // sin eso volvería a descargarse todo en cada arranque.
  mirror.watch();
  mirrorReady = mirror.init().then((hydrated) => {
    if (!hydrated) return;
    VIEWS[activeView]?.render?.();
    if (!state.connected) toast('Configuración cargada de la copia del PC; conecta el Orby para editarla', 'info', 5000);
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
