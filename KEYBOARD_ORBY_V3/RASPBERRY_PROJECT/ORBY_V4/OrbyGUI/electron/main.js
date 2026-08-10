const { app, BrowserWindow, ipcMain, dialog, Tray, Menu, nativeImage, Notification } = require('electron');
const path = require('path');
const fs = require('fs');
const { autoUpdater } = require('electron-updater');
const { OrbySerial } = require('./serial');
const { ForegroundWatcher } = require('./foreground');
const config = require('./config');
const { executeMacro, getMousePosition, warmup } = require('./macros');
const { listInstalledApps } = require('./apps');
const recorder = require('./recorder');
const plugins = require('./plugins');
const { FirmwareUpdater } = require('./firmware');
const { log } = require('./log');

let mainWindow = null;
let serial = null;
let foreground = null;
let tray = null;
let firmware = null;

// Solo puede haber una instancia: si arranca sola (autostart) y luego el
// usuario hace doble clic en el icono, la segunda instancia no debe abrir un
// segundo tray ni pelear por el puerto serie con la primera.
const gotSingleInstanceLock = app.requestSingleInstanceLock();
if (!gotSingleInstanceLock) {
  app.quit();
}

// Cerrar la ventana (la X de la barra de título propia) esconde la app en
// vez de matarla: las secuencias las ejecuta este proceso al recibir
// MACRO:<id> por CDC, así que si se cerrara de verdad dejarían de funcionar
// en cuanto no hubiera una ventana abierta. Solo el menú de la bandeja (o un
// cierre real del sistema) pone `quitting` a true y deja pasar el cierre.
let quitting = false;

// Un fallo aquí arriba no se ve en ningún sitio: la app instalada no tiene
// consola y Electron se traga la excepción dejando la app a medio montar (fue
// justo lo que pasó con `pkg.build.publish`). Al registro, que sí se puede leer.
process.on('uncaughtException', (err) => {
  log(`[app] excepción sin capturar: ${err?.stack || err}`);
});
process.on('unhandledRejection', (err) => {
  log(`[app] promesa rechazada sin capturar: ${err?.stack || err}`);
});

// El modo desarrollo se pide explícitamente con --dev. Deducirlo de
// app.isPackaged hacía que un `electron .` normal intentara conectarse al
// servidor de Vite y se quedara en pantalla en blanco.
const isDev = process.argv.includes('--dev');
const DEV_URL = 'http://localhost:5173';

// Dónde se publican las releases: las de la app (`v*`) y las del firmware
// (`fw-v*`), dos feeds sobre el mismo repositorio.
//
// Escrito aquí y no leído de `pkg.build.publish`: electron-builder SE LLEVA el
// campo `build` del package.json al empaquetar, así que en la app instalada no
// existe. Leerlo reventaba createWindow() con "Cannot read properties of
// undefined (reading 'publish')" justo después de crear el puerto serie, y todo
// lo que venía detrás —el reenvío de líneas al renderer, el arranque del
// escaneo, el disparo de macros, el cierre a la bandeja— no llegaba a
// registrarse: la app instalada no detectaba el teclado, no se quedaba en la
// bandeja y no ejecutaba secuencias, y en desarrollo no se reproducía nada de
// eso porque ahí el package.json sí tiene su `build`.
//
// Tiene que coincidir con "build.publish" del package.json.
const GITHUB_REPO = 'jaestefaniah27/scroll_wheel';

// Autoarranque relanza con este flag (ver autostart:set) para que la ventana
// no se muestre sola al iniciar sesión: el usuario la abre desde la bandeja
// si quiere. Sin esto, ready-to-show la enseña siempre.
const startHidden = process.argv.includes('--hidden');

// La ventana se abre maximizada. No se puede maximizar al crearla porque
// `maximize()` también la muestra, y con --hidden tiene que quedarse en la
// bandeja; así que queda pendiente hasta la primera vez que se enseñe.
// Es de una sola vez a propósito: si luego se restaura a tamaño ventana y se
// esconde en la bandeja, al volver a sacarla se respeta como estaba. El
// 1280x820 de createWindow() sigue siendo el tamaño al que restaura.
let pendingMaximize = true;

// Único punto por el que se saca la ventana (bandeja, aviso de conexión,
// segunda instancia, ready-to-show): así el maximizado inicial no depende de
// por dónde se haya abierto.
function revealWindow() {
  if (!mainWindow) return;
  if (mainWindow.isMinimized()) mainWindow.restore();
  if (pendingMaximize) {
    pendingMaximize = false;
    mainWindow.maximize();
  }
  mainWindow.show();
  mainWindow.focus();
}

// En desarrollo Electron y Vite arrancan a la vez, así que la primera carga
// puede llegar antes de que el servidor escuche. Reintentamos en vez de
// depender de wait-on.
async function loadRenderer(win) {
  if (!isDev) {
    return win.loadFile(path.join(__dirname, '..', 'dist', 'index.html'));
  }
  for (let attempt = 0; attempt < 40; attempt++) {
    try {
      await win.loadURL(DEV_URL);
      return;
    } catch {
      await new Promise((r) => setTimeout(r, 400));
    }
  }
  console.error(`No se pudo conectar con ${DEV_URL}. ¿Está corriendo "vite"?`);
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 820,
    minWidth: 1024,
    minHeight: 700,
    backgroundColor: '#0a0a0f',
    frame: false,
    titleBarStyle: 'hidden',
    show: false,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
    icon: path.join(__dirname, '..', 'assets', 'orby-icon.png'),
  });

  mainWindow.once('ready-to-show', () => {
    if (!startHidden) revealWindow();
  });

  // En desarrollo, lo que la app escribe en su consola sale también por el
  // terminal. Sin esto había que abrir las herramientas de desarrollo a mano
  // para ver por qué la app decide releer la configuración del teclado
  // (los avisos "[sync] …" de store.js), y en una ventana sin menú eso no
  // siempre es cómodo.
  if (isDev) {
    mainWindow.webContents.on('console-message', (_e, level, message) => {
      console.log(`[renderer] ${message}`);
    });
  }
  loadRenderer(mainWindow);

  // La X de la barra propia manda 'window:close', que llama a .close() más
  // abajo: se intercepta aquí para esconder en vez de cerrar de verdad.
  let balloonShown = false;
  mainWindow.on('close', (e) => {
    if (quitting) return;
    e.preventDefault();
    mainWindow.hide();
    if (!balloonShown && tray) {
      balloonShown = true;
      tray.displayBalloon({
        title: 'OrbyGUI sigue activo',
        content: 'Se quedó en la bandeja del sistema para poder seguir ejecutando secuencias. Para cerrarlo del todo, usa "Salir" en el icono de la bandeja.',
      });
    }
  });

  mainWindow.on('closed', () => {
    mainWindow = null;
    serial?.stopAutoScan();
    serial?.disconnect();
    foreground?.stop();
  });


  // --- Serie ---
  serial = new OrbySerial();
  const forward = (event, channel) =>
    serial.on(event, (payload) => mainWindow?.webContents.send(channel, payload));

  forward('connected', 'serial:connected');
  forward('disconnected', 'serial:disconnected');
  forward('error', 'serial:error');
  forward('searching', 'serial:searching');

  // --- Firmware del teclado ---
  // Necesita el puerto serie para pedirle al teclado que se reinicie en el
  // cargador, así que se crea aquí y no arriba con el resto de módulos.
  //
  // Va en su try/catch, como todo lo que no es imprescindible para hablar con el
  // teclado: que falle la tarjeta de firmware es un incordio, pero que se lleve
  // por delante el resto de esta función deja la app sin puerto serie, sin
  // bandeja y sin macros (ver GITHUB_REPO).
  try {
    firmware = new FirmwareUpdater({ repo: GITHUB_REPO, serial });
    firmware.on('state', (st) => mainWindow?.webContents.send('firmware:state', st));
    log(`[app] actualización de firmware lista (repo ${GITHUB_REPO})`);
  } catch (err) {
    log(`[app] no se pudo preparar la actualización de firmware: ${err.message}`);
  }

  // Aviso de "acaba de enchufarse": sustituye al popup de instalación de
  // hardware nuevo tipo Razer Synapse (eso exigiría Device Metadata Authoring +
  // firma WHQL + ficha en Microsoft Store, ver docs/TODO_BACKGROUND_AUTOSTART.md).
  // Se engancha al mismo evento 'connected' que ya dispara el escaneo por
  // puerto serie cada 3s (ver serial.js) en vez de usar usb-detection: ese
  // módulo requiere compilar código nativo con node-gyp, y este equipo no
  // tiene Visual Studio Build Tools instalado. El resultado para el usuario
  // es idéntico (toast al conectar, clic abre/enfoca la ventana) sin añadir
  // esa dependencia nativa.
  serial.on('connected', () => {
    if (!Notification.isSupported()) return;
    const notif = new Notification({
      title: 'Orby conectado',
      body: 'Se detectó el teclado. Haz clic para abrir OrbyGUI.',
      icon: path.join(__dirname, '..', 'assets', 'orby-icon.png'),
    });
    notif.on('click', revealWindow);
    notif.show();
  });

  // Las líneas MACRO:<id> las dispara el firmware pero las ejecuta el PC; se
  // interceptan aquí además de seguir reenviándolas al renderer como siempre,
  // para que la consola de la app también las vea.
  //
  // El id y NADA MÁS detrás. El prefijo "MACRO:" lo comparten todas las
  // respuestas del protocolo de secuencias (MACRO:OK:…, MACRO:<id>:STEP:…,
  // MACRO:<id>:END), y con un parseInt suelto el volcado de GET_MACRO se leía
  // como una pulsación: al conectar, la app se ponía a ejecutar en el PC todas
  // las secuencias que estaba leyendo del teclado.
  const MACRO_TRIGGER = /^MACRO:(\d+)$/;

  serial.on('data', (line) => {
    const trigger = MACRO_TRIGGER.exec(line);
    if (trigger) {
      triggerMacro(parseInt(trigger[1], 10));
    } else if (line.startsWith('KEY_EV:') && line.endsWith(':0')) {
      // Reproducción "mientras se mantenga pulsada": el firmware solo avisa de
      // la macro al pulsar, así que la suelta se saca de la telemetría de
      // teclas. Cualquier tecla vale para cortar: soltar es soltar.
      if (recorder.playingMode() === 'hold') recorder.stopPlayback();
    }
    mainWindow?.webContents.send('serial:data', line);
  });

  serial.startAutoScan();

  // --- Ventana en primer plano ---
  foreground = new ForegroundWatcher();
  foreground.on('change', (info) => mainWindow?.webContents.send('foreground:change', info));
  foreground.on('error', (err) => mainWindow?.webContents.send('foreground:error', err));

  if (config.load().autoProfile.enabled) foreground.start();

}

// --- Grabar operación -------------------------------------------------------
// Una tecla de tipo "grabación" hace tres cosas distintas según en qué punto
// esté, y siempre con la misma pulsación:
//   sin nada grabado  -> empieza a grabar
//   grabando          -> para y guarda
//   con algo grabado  -> lo reproduce (y, en bucle, la siguiente pulsación lo para)
// El resto de macros siguen yendo al reproductor de secuencias de siempre.

function macroById(id) {
  return (config.load().macros || []).find((m) => m.id === id) || null;
}

function notifyRecorder(id, phase = {}) {
  mainWindow?.webContents.send('recorder:state', { id, phase });
}

function saveRecording(id, events) {
  const macros = (config.load().macros || []).map(
    (m) => (m.id === id ? { ...m, events } : m));
  config.save({ macros });
}

function triggerMacro(id) {
  const macro = macroById(id);

  // Una tecla que manda un id que la configuración no conoce no hace nada y no
  // se queja: es el síntoma de una tecla que quedó apuntando a una macro
  // borrada, y sin esta línea no hay forma de verlo.
  if (!macro) {
    console.warn(`MACRO:${id} llegó del teclado pero no hay ninguna macro con ese id`);
    return;
  }

  // SUPER + la tecla de una grabación: tira lo grabado y la deja lista para
  // volver a grabar. Es la pareja que monta sola la pestaña "Grabar".
  if (macro?.kind === 'recording-reset') {
    if (recorder.isRecording()) recorder.stopRecording();
    recorder.stopPlayback(true);   // sin aviso de final: el reset ya repinta la tecla
    saveRecording(macro.target, []);
    notifyRecorder(macro.target, 'reset');
    return;
  }

  if (macro?.kind !== 'recording') {
    executeMacro(id).catch((err) => console.error('Macro:', err.message));
    return;
  }

  if (recorder.isRecording()) {
    const events = recorder.stopRecording();
    saveRecording(id, events);
    notifyRecorder(id, events.length ? 'saved' : 'empty');
    return;
  }

  if (recorder.playingId() === id) {
    recorder.stopPlayback();
    return;
  }

  if (macro.events?.length) {
    notifyRecorder(id, 'playing');
    recorder.startPlayback(id, macro.events, macro.mode || 'once', macro.speed || 3,
                           () => notifyRecorder(id, 'idle'));
    return;
  }

  recorder.startRecording();
  notifyRecorder(id, 'recording');
}

// Icono en la bandeja del sistema: es lo que permite que la app siga viva
// (y por tanto ejecutando secuencias) después de "cerrar" la ventana.
function createTray() {
  const iconPath = path.join(__dirname, '..', 'assets', 'orby-icon.png');
  const trayIcon = nativeImage.createFromPath(iconPath).resize({ width: 16, height: 16 });
  tray = new Tray(trayIcon);
  tray.setToolTip('OrbyGUI');

  tray.setContextMenu(Menu.buildFromTemplate([
    { label: 'Abrir OrbyGUI', click: revealWindow },
    { type: 'separator' },
    { label: 'Salir', click: () => { quitting = true; app.quit(); } },
  ]));

  tray.on('click', () => {
    if (!mainWindow) return;
    if (mainWindow.isVisible()) mainWindow.hide();
    else revealWindow();
  });
  tray.on('double-click', revealWindow);
}

// --- Actualizaciones automáticas ---
// Se comprueba contra las Releases de GitHub (ver "publish" en package.json).
// Solo tiene sentido con la app empaquetada e instalada: en dev no hay
// instalador que sustituir ni feed de actualizaciones que consultar.
//
// El estado se replica al renderer (canal 'updater:state') porque
// `autoInstallOnAppQuit` no basta en esta app: la X esconde la ventana en vez
// de cerrar, así que el proceso puede pasar semanas vivo con la actualización
// descargada y sin instalar. La forma de instalarla es el aviso de la barra
// de título, que sí se ve.
const UPDATE_CHECK_INTERVAL_MS = 6 * 60 * 60 * 1000;

// status: 'dev' | 'idle' | 'checking' | 'downloading' | 'downloaded' | 'error'
let updateState = {
  status: 'idle',
  version: app.getVersion(),
  newVersion: null,
  percent: 0,
  error: null,
};

// electron-updater mete el cuerpo entero de la respuesta HTTP en el mensaje
// cuando no puede leer el feed de releases: cabeceras, cookies y el XML de
// GitHub. Eso no cabe en una tarjeta, así que solo viaja la primera línea
// recortada; el detalle completo queda en la consola del proceso principal.
function shortUpdateError(err) {
  const raw = String(err?.message ?? err ?? 'desconocido');
  const firstLine = raw.split('\n', 1)[0].trim();
  return firstLine.length > 140 ? `${firstLine.slice(0, 140)}…` : firstLine;
}

function setUpdateState(patch) {
  updateState = { ...updateState, ...patch };
  mainWindow?.webContents.send('updater:state', updateState);
}

function checkForUpdates() {
  if (!app.isPackaged) return;
  autoUpdater.checkForUpdates().catch((err) => {
    console.error('AutoUpdater:', err);
    setUpdateState({ status: 'error', error: shortUpdateError(err) });
  });
}

function setupAutoUpdater() {
  if (!app.isPackaged) {
    updateState = { ...updateState, status: 'dev' };
    return;
  }

  autoUpdater.autoDownload = true;
  autoUpdater.autoInstallOnAppQuit = true;

  autoUpdater.on('checking-for-update', () => setUpdateState({ status: 'checking', error: null }));
  autoUpdater.on('update-not-available', () => setUpdateState({ status: 'idle', newVersion: null }));
  autoUpdater.on('update-available', (info) =>
    setUpdateState({ status: 'downloading', newVersion: info.version, percent: 0, error: null }));
  autoUpdater.on('download-progress', (p) =>
    setUpdateState({ status: 'downloading', percent: Math.round(p.percent) }));

  autoUpdater.on('error', (err) => {
    console.error('AutoUpdater:', err);
    setUpdateState({ status: 'error', error: shortUpdateError(err) });
  });

  autoUpdater.on('update-downloaded', (info) => {
    setUpdateState({ status: 'downloaded', newVersion: info.version, percent: 100 });
    if (!Notification.isSupported()) return;
    const notif = new Notification({
      title: `OrbyGUI ${info.version} disponible`,
      body: 'Haz clic para instalarla ahora, o usa el aviso de la barra superior.',
      icon: path.join(__dirname, '..', 'assets', 'orby-icon.png'),
    });
    notif.on('click', installUpdate);
    notif.show();
  });

  checkForUpdates();
  // Un proceso que vive en la bandeja puede no reiniciarse en semanas: sin este
  // repaso solo se enteraría de una versión nueva al arrancar.
  setInterval(checkForUpdates, UPDATE_CHECK_INTERVAL_MS);
}

function installUpdate() {
  if (updateState.status !== 'downloaded') return false;
  quitting = true;
  // Fuera del manejador que lo pidió: quitAndInstall cierra las ventanas al
  // vuelo y dejaría colgada la respuesta del IPC (o el clic de la notificación).
  setImmediate(() => autoUpdater.quitAndInstall(false, true));
  return true;
}

ipcMain.handle('updater:get', () => updateState);
ipcMain.handle('updater:check', () => { checkForUpdates(); return updateState; });
ipcMain.handle('updater:install', () => installUpdate());

// --- IPC: firmware del teclado ---
// `maxFw` lo pone el renderer desde compat.js: es la app la que sabe hasta qué
// firmware entiende, y no tiene sentido ofrecer uno que no sabría manejar.
ipcMain.handle('firmware:get', () => firmware?.state ?? null);
ipcMain.handle('firmware:check', (_e, opts) => firmware?.check(opts) ?? null);
ipcMain.handle('firmware:update', (_e, opts) => firmware?.update(opts) ?? null);
ipcMain.handle('firmware:cancel', () => firmware?.cancel() ?? null);

// --- IPC: serie ---
// Comandos que cambian algo en el teclado. Los de lectura (GET_*) no entran:
// son cientos en una sincronización y no dicen nada que no se sepa ya.
//
// Van al registro para que "se me han borrado los perfiles" tenga respuesta.
// Cuando pasó, no había forma de saber si lo había mandado la app, el menú del
// propio teclado o un clic del usuario.
const CAMBIA_ALGO = /^(SET_|DEL_|ADD_|SAVE_STATE|RESET_DEFAULTS|OLED_CLEAR|MACRO_CLEAR|MACRO_TRUNC|BOOTSEL)/;

ipcMain.handle('serial:send', async (_e, cmd) => {
  // En desarrollo sale TODO por el terminal: es la única forma cómoda de
  // comprobar que una conexión no está releyendo la configuración (si aparecen
  // GET_PROFILE o GET_OLED_PG, es que sí).
  if (isDev) console.log(`[serie] > ${cmd}`);
  else if (CAMBIA_ALGO.test(cmd)) log(`[serie] > ${cmd}`);
  return serial ? serial.sendCommand(cmd) : false;
});
ipcMain.handle('serial:getInfo', async () => serial?.getDeviceInfo() ?? null);
ipcMain.handle('serial:getStatus', async () => serial?.getStatus() ?? 'disconnected');
ipcMain.handle('serial:reconnect', async () => {
  serial?.forceRescan();
  return Boolean(serial);
});

// --- IPC: ratón (para capturar posiciones al editar una secuencia) ---
// Se pregunta a nut.js, no al módulo screen de Electron: tiene que devolver
// coordenadas en el mismo sistema que usa mouse.setPosition al reproducir la
// secuencia, o la posición capturada queda desplazada en pantallas con escalado.
ipcMain.handle('mouse:getPosition', async () => getMousePosition());

// --- IPC: configuración local ---
ipcMain.handle('config:get', async () => config.load());
ipcMain.handle('config:set', async (_e, patch) => config.save(patch));

// --- IPC: complementos ---
// Solo la gestión (instalar, activar, ajustar, comprobar) y la lista para
// pintarla. Las acciones asignadas a teclas y mandos no pasan por aquí: las
// dispara el firmware con MACRO:<id> y las ejecuta electron/macros.js contra
// electron/plugins.js, sin tocar el renderer.
ipcMain.handle('plugins:list', async () => plugins.list());
ipcMain.handle('plugins:install', async () => plugins.install(mainWindow));
ipcMain.handle('plugins:uninstall', async (_e, id) => plugins.uninstall(id));
ipcMain.handle('plugins:setEnabled', async (_e, id, enabled) => plugins.setEnabled(id, enabled));
ipcMain.handle('plugins:getSettings', async (_e, id) => plugins.settingsOf(id));
ipcMain.handle('plugins:setSettings', async (_e, id, patch) => plugins.saveSettings(id, patch));
ipcMain.handle('plugins:test', async (_e, id, values) => plugins.test(id, values));
ipcMain.handle('plugins:openFolder', async () => plugins.openFolder());

// --- IPC: autoarranque con Windows ---
// app.setLoginItemSettings ya persiste el valor en el registro de Windows por
// su cuenta (no hace falta guardarlo también en config.json): consultamos
// siempre el estado real en vez de duplicarlo.
// En Windows, getLoginItemSettings() compara path+args con lo registrado: hay
// que pasarle los mismos args que usa setLoginItemSettings o siempre devuelve
// openAtLogin:false aunque el autoarranque esté activo (no encuentra el item
// porque busca uno sin args).
const AUTOSTART_ARGS = ['--hidden'];
ipcMain.handle('autostart:get', async () => app.getLoginItemSettings({ args: AUTOSTART_ARGS }).openAtLogin);
ipcMain.handle('autostart:set', async (_e, enabled) => {
  // --hidden le dice a este mismo main.js (ver startHidden más arriba) que no
  // muestre la ventana al arrancar por login; sin el flag se abriría sola.
  app.setLoginItemSettings({ openAtLogin: enabled, args: enabled ? AUTOSTART_ARGS : [] });
  return app.getLoginItemSettings({ args: AUTOSTART_ARGS }).openAtLogin;
});

// --- IPC: copias de seguridad ---
// Guardar los perfiles y los iconos en un archivo del PC es el único respaldo
// que sobrevive a un cambio de formato en la memoria del teclado.
ipcMain.handle('backup:save', async (_e, data) => {
  const stamp = new Date().toISOString().slice(0, 16).replace(/[:T]/g, '-');
  const { canceled, filePath } = await dialog.showSaveDialog(mainWindow, {
    title: 'Guardar copia de seguridad',
    defaultPath: path.join(app.getPath('documents'), `orby-backup-${stamp}.json`),
    filters: [{ name: 'Copia de seguridad Orby', extensions: ['json'] }],
  });
  if (canceled || !filePath) return { ok: false, canceled: true };

  try {
    fs.writeFileSync(filePath, JSON.stringify(data, null, 2), 'utf8');
    return { ok: true, path: filePath };
  } catch (err) {
    return { ok: false, error: err.message };
  }
});

ipcMain.handle('backup:load', async () => {
  const { canceled, filePaths } = await dialog.showOpenDialog(mainWindow, {
    title: 'Restaurar copia de seguridad',
    properties: ['openFile'],
    filters: [{ name: 'Copia de seguridad Orby', extensions: ['json'] }],
  });
  if (canceled || !filePaths?.length) return { ok: false, canceled: true };

  try {
    return { ok: true, data: JSON.parse(fs.readFileSync(filePaths[0], 'utf8')) };
  } catch (err) {
    return { ok: false, error: err.message };
  }
});

// --- IPC: elegir app o archivo para un paso "Abrir…" de una secuencia ---
ipcMain.handle('dialog:pickAppOrFile', async (_e, kind) => {
  // El filtro que va primero es el que el diálogo enseña seleccionado; el
  // otro sigue disponible en el desplegable, así que "Aplicación" y "Archivo"
  // solo cambian la sugerencia inicial, no lo que se puede elegir.
  const programFilter = { name: 'Programas', extensions: ['exe', 'lnk', 'bat', 'cmd'] };
  const allFilter = { name: 'Todos los archivos', extensions: ['*'] };
  const filters = kind === 'file' ? [allFilter, programFilter] : [programFilter, allFilter];

  const { canceled, filePaths } = await dialog.showOpenDialog(mainWindow, {
    title: kind === 'file' ? 'Elegir archivo' : 'Elegir aplicación',
    properties: ['openFile'],
    filters,
  });
  if (canceled || !filePaths?.length) return { ok: false, canceled: true };
  return { ok: true, path: filePaths[0] };
});

// --- IPC: apps instaladas (pestaña "App" del inspector de tecla) ---
ipcMain.handle('apps:listInstalled', async () => listInstalledApps());

// --- IPC: grabar operación ---
// Los mismos tres estados que la pulsación de la tecla, para poder empezar y
// parar la grabación desde el editor sin tener el Orby delante.
ipcMain.handle('recorder:toggle', async (_e, id) => {
  triggerMacro(id);
  return { recording: recorder.isRecording(), playing: recorder.playingId() === id };
});
ipcMain.handle('recorder:stop', async () => { recorder.stopPlayback(); return true; });
ipcMain.handle('recorder:status', async () => ({
  recording: recorder.isRecording(),
  playingId: recorder.playingId(),
}));

// --- IPC: detector de aplicaciones ---
ipcMain.handle('foreground:start', async () => foreground?.start() ?? false);
ipcMain.handle('foreground:stop', async () => { foreground?.stop(); return true; });
ipcMain.handle('foreground:current', async () => foreground?.current ?? null);
ipcMain.handle('foreground:available', async () => foreground?.available ?? false);

// --- IPC: ventana ---
ipcMain.on('window:minimize', () => mainWindow?.minimize());
ipcMain.on('window:maximize', () => {
  if (mainWindow?.isMaximized()) mainWindow.unmaximize();
  else mainWindow?.maximize();
});
ipcMain.on('window:close', () => mainWindow?.close());

if (gotSingleInstanceLock) {
  // Lanzar la app de nuevo (autostart ya corriendo + doble clic manual, o
  // el propio instalador) llega aquí en vez de abrir una segunda instancia:
  // simplemente se muestra la ventana que ya existe.
  app.on('second-instance', revealWindow);

  app.whenReady().then(() => {
    // Cada pieza en su try/catch y con su nombre en el registro. Antes iban
    // seguidas: la primera que fallara se llevaba a las demás por delante y la
    // app se quedaba a medio montar sin decir nada (así estuvo la app instalada
    // sin bandeja y sin puerto serie, ver GITHUB_REPO).
    const paso = (nombre, fn) => {
      try {
        fn();
      } catch (err) {
        log(`[app] fallo al arrancar "${nombre}": ${err?.stack || err}`);
      }
    };

    // Antes de la ventana y del puerto serie: el teclado puede mandar un
    // MACRO:<id> en cuanto se conecta, y si el complemento al que apunta no
    // estuviera cargado todavía ese primer disparo se perdería sin más.
    paso('complementos', () => plugins.loadAll());

    paso('ventana', createWindow);
    paso('bandeja', createTray);
    paso('actualizaciones', setupAutoUpdater);

    // Después de la ventana, no antes: requiere el binario nativo de nut.js y
    // puede tardar (ver warmup() en macros.js), no debe retrasar el arranque.
    setImmediate(warmup);

    // Migra entradas de autoarranque creadas antes de que existiera el flag
    // --hidden (se abrirían solas igualmente sin esto).
    if (app.getLoginItemSettings().openAtLogin) {
      app.setLoginItemSettings({ openAtLogin: true, args: ['--hidden'] });
    }
  });

  app.on('window-all-closed', () => app.quit());
  app.on('before-quit', () => {
    quitting = true;
    foreground?.stop();
    // El hook global de entrada corre en un hilo nativo: sin pararlo, el
    // proceso no llega a terminar nunca.
    recorder.shutdown();
  });

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
    else revealWindow();
  });
}
