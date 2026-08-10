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
import * as splash from './splash.js';
import * as updater from './updater.js';
import * as plugins from './plugins.js';
import * as compat from './compat.js';
import * as firmware from './firmware.js';
import * as platform from './platform.js';
import * as liveOled from './live-oled.js';

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

// Arranque: la pantalla de carga tapa hasta que la app está montada Y el
// teclado ha tenido su oportunidad de contestar. Sin esto lo primero que se veía
// era la interfaz vacía (o sin estilos) rellenándose a trozos mientras llegaban
// los perfiles.
let bootDone = false;

// Margen para el teclado cuando no ha dicho nada todavía. El proceso principal
// escanea los puertos cada 3 s y el ACK tarda ~2 s en confirmarse, así que un
// arranque con el Orby ya enchufado necesita esperar un poco antes de darlo por
// ausente; si no aparece, se abre en solo lectura con la copia del PC.
const BOOT_GRACE_MS = 2500;

function finishBoot() {
  if (bootDone) return;
  bootDone = true;
  splash.hide();
}

// Durante el arranque los mensajes de progreso van al texto de la pantalla de
// carga: forman parte de "todavía me estoy montando", no son avisos que el
// usuario tenga que leer encima de una app ya pintada.
function progress(msg, kind = 'info', ms = 900) {
  if (splash.visible()) splash.status(msg);
  else toast(msg, kind, ms);
}

function initChrome() {
  // En el navegador la pestaña ya tiene su marco y su botón de cerrar, y no hay
  // proceso que vigile la ventana en primer plano: el cambio automático de perfil
  // no puede funcionar, así que su entrada no se enseña en vez de dejarla llevando
  // a una pantalla que no hace nada.
  if (!platform.can('windowChrome')) {
    document.querySelector('.titlebar-controls')?.classList.add('hidden');
  }
  if (!platform.can('autoProfile')) {
    document.querySelector('.nav-item[data-target="view-auto"]')?.classList.add('hidden');
  }
  if (!platform.can('appUpdate')) {
    document.getElementById('btn-update')?.classList.add('hidden');
  }

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
  // Sin autoactualización no hay insignia que pintar, y updater.init() llamaría a
  // un window.orby.updater que en navegador no hace nada.
  if (!platform.can('appUpdate')) return;

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
  if (!platform.can('appUpdate')) return;

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
    // La versión de firmware va aquí y no solo en Ajustes: es el primer dato
    // que hace falta para saber si un fallo es del teclado o de la app, y
    // tenerla a la vista ahorra tener que ir a buscarla.
    const fw = state.deviceInfo?.fw ? ` · fw ${state.deviceInfo.fw}` : '';
    text.textContent = state.syncing ? 'Sincronizando…' : `Orby V4 conectado${fw}`;
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
// Cada cuántos iconos se vuelca el espejo mientras se descargan. Guardar solo al
// final salía carísimo: si el teclado se iba (o se cerraba la app) a mitad, lo
// descargado se perdía entero, la copia del PC se quedaba sin esos iconos y la
// conexión siguiente no podía comparar huellas —profileHash necesita los
// bitmaps— así que volvía a leer el perfil entero y sus veinte iconos. Con la
// app abriéndose y cerrándose a menudo, eso no convergía nunca.
const ICON_SAVE_EVERY = 20;

function preloadIcons(canHash) {
  if (!canHash) {
    cache.loadProfile(state.activeProfileIdx);
    return;
  }

  cache.preloadAll((done, total) => {
    if (done % ICON_SAVE_EVERY === 0) mirror.save();
    if (done === total) progress('Iconos descargados', 'info', 1500);
  }).then(() => {
    VIEWS[activeView]?.render?.();
    // El espejo del disco se guarda con los iconos ya dentro: así la próxima
    // apertura sin teclado los enseña, y la próxima conexión puede compararlos.
    mirror.save();
  }).catch((err) => {
    console.error('No se pudieron precargar los iconos:', err);
    mirror.save();   // lo que sí llegó se conserva
  });
}

// Las huellas por perfil, con reintentos.
//
// Un GET_HASH perdido no es un detalle: sin huellas la app no puede saber qué
// ha cambiado y se descarga los perfiles enteros con sus iconos. Y perderse se
// pierde: al enchufar el teclado, el CDC acaba de enumerarse y la primera
// pregunta se puede quedar por el camino. Antes bastaba con eso para que la
// conexión siguiente volviera a leerlo todo.
async function askHashes(attempts = 3) {
  for (let i = 0; i < attempts; i++) {
    const hashes = await device.getHash();
    if (hashes?.all) return hashes.per;
    if (!device.isConnected()) return null;
    await new Promise((r) => setTimeout(r, 400));
  }
  return null;
}

async function onDeviceConnected(info) {
  state.connected = true;
  state.deviceInfo = info;
  notify();

  // Qué sabe hacer el teclado que hay delante. La tabla vive en compat.js
  // para que no haya un número de versión suelto por cada vista; aquí solo
  // se decide si se sigue adelante y qué se le cuenta al usuario.
  const verdict = compat.check(info);
  if (verdict.level === 'blocked') {
    toast(`${verdict.title}. ${verdict.detail}`, 'error', 9000);
    return;
  }
  if (verdict.level !== 'ok') {
    toast(`${verdict.title}. ${verdict.detail}`, 'error', 9000);
  }

  // La copia del PC tiene que estar cargada antes de comparar huellas: es
  // contra ella contra lo que se decide qué hace falta descargar.
  await mirrorReady;

  // GET_HASH y GET_OLED_PG los añadió el firmware 4.1. Con uno anterior se
  // sigue haciendo lo de siempre: descargarlo todo y leer los iconos de la
  // página activa según hagan falta.
  const canHash = compat.supports(info, 'hash');

  try {
    let expected = null;
    if (canHash) {
      progress('Comprobando si la copia del PC sigue valiendo…');
      expected = await askHashes();
      if (!expected) {
        // Sin huellas no queda otra que descargarlo todo, así que conviene
        // dejar dicho por qué: es la diferencia entre una conexión instantánea
        // y una que se pasa medio minuto leyendo perfiles e iconos.
        console.warn('[sync] el teclado no ha dado las huellas: toca releerlo todo');
        progress('El teclado no ha dado las huellas: hay que releerlo todo', 'error', 5000);
      }
    }

    const reloaded = await syncFromDevice({
      expected,
      iconOf: cache.get,
      onProgress: (i, total) => progress(`Leyendo perfil ${i + 1} de ${total}…`),
    });

    // Un perfil que se ha vuelto a leer es un perfil que había cambiado: sus
    // iconos en caché son los de antes. Y los de los perfiles que la copia
    // traía de más ya no valen para nada.
    for (const idx of reloaded) cache.dropProfile(idx);
    cache.pruneBeyond(state.profiles.length);

    VIEWS[activeView]?.render?.();
    // Cuando no ha hecho falta descargar nada no se dice nada: es el caso
    // normal, y un aviso en cada conexión para contar que no ha pasado nada
    // solo es ruido.
    if (reloaded.length) progress('Perfiles cargados desde el teclado', 'success', 2600);
    else splash.status('Todo al día');
  } catch (err) {
    toast(`No se pudo leer la configuración: ${err.message}`, 'error');
    return;
  }

  preloadIcons(canHash);

  // No bloquea el arranque: si el teclado lleva firmware con secuencias
  // propias, sube las que le falten (cubre un teclado recién reflasheado, cuya
  // Flash de secuencias está vacía aunque la app ya las tuviera guardadas).
  profiles.syncAllMacrosToDevice?.().catch(() => {});

  // Qué firmware hay publicado para este teclado. No bloquea: si GitHub no
  // contesta, la tarjeta de Ajustes lo dirá y no pasa nada más.
  //
  // Actualizar el firmware exige copiar un .uf2 en la unidad USB que aparece al
  // reiniciar en modo carga: eso el navegador no puede hacerlo.
  if (platform.can('firmwareUpdate')) {
    firmware.check()?.then((st) => {
      if (st?.available) {
        toast(`Hay firmware nuevo para el teclado (${st.latest.version}). Ajustes → Firmware del teclado`, 'info', 8000);
      }
    }).catch(() => {});
  }
}

function wireDevice() {
  // El arranque no se da por terminado hasta que esto acaba, salga bien o mal:
  // así la pantalla de carga se levanta con la app ya cuadrada con el teclado,
  // en vez de enseñar los perfiles apareciendo de uno en uno.
  device.on('connected', (info) => {
    onDeviceConnected(info).catch((err) => {
      console.error('Fallo al preparar la conexión con el teclado:', err);
    }).finally(finishBoot);
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

// main.js llega tanto por <script type="module"> directo (Electron, si algún día
// se deja de pasar por entry.js) como por import() dinámico desde entry.js/web-main.js
// después de montar window.orby en el navegador. En ese segundo camino el documento
// ya ha disparado DOMContentLoaded para cuando este módulo se evalúa: escuchar el
// evento sin más deja el arranque colgado para siempre, pantalla de carga incluida.
function arrancarApp() {
  splash.init();
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

  // Complementos instalados: la lista llega del proceso principal, así que el
  // primer pintado del editor puede no tenerla todavía. plugins.onChange hace
  // que las vistas se repinten al llegar (ver profiles.init y settings.init).
  plugins.init();

  // Visor en vivo (brillo/color actual...) en la pantalla de la tecla que se le
  // asigne: sondea el complemento y empuja el bitmap sin tocar Flash.
  liveOled.init();

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
  splash.status('Leyendo la copia del PC…');
  mirrorReady = mirror.init().then((hydrated) => {
    if (hydrated) VIEWS[activeView]?.render?.();

    // A partir de aquí la app ya se puede enseñar entera. Solo falta saber si
    // hay teclado: se le da un margen y, si no aparece, se abre en solo lectura
    // con esta copia. El aviso se deja para entonces, no antes, para no soltarlo
    // por encima de una app que aún estaba montándose.
    splash.status(state.connected ? 'Hablando con el teclado…' : 'Buscando el teclado…');
    setTimeout(() => {
      if (state.connected) return;   // ya lo terminará el manejador de conexión
      finishBoot();
      if (hydrated) toast('Configuración cargada de la copia del PC; conecta el Orby para editarla', 'info', 5000);
    }, BOOT_GRACE_MS);
  });

  dashboard.init();
  profiles.init();
  oled.init();
  settings.init();
  consoleView.init();
  // cfg.autoProfile no existe en la WebGUI (config-merge.mjs lo quita de DEFAULTS
  // a propósito: no hay proceso que vigile la ventana en primer plano), y auto.init
  // lo lee sin comprobar nada. Coherente con el hueco de nav de initChrome de arriba.
  if (platform.can('autoProfile')) auto.init(); // asíncrono: lee la configuración local antes de pintarse

  subscribe(() => {
    renderChrome();
    settings.render();
    if (activeView === 'view-dashboard') dashboard.render();
  });

  renderChrome();
  switchView('view-dashboard');
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', arrancarApp);
} else {
  arrancarApp();
}
