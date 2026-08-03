const { app, BrowserWindow, ipcMain, dialog, Tray, Menu, nativeImage, Notification } = require('electron');
const path = require('path');
const fs = require('fs');
const { autoUpdater } = require('electron-updater');
const { OrbySerial } = require('./serial');
const { ForegroundWatcher } = require('./foreground');
const config = require('./config');
const { executeMacro, getMousePosition, warmup } = require('./macros');

let mainWindow = null;
let serial = null;
let foreground = null;
let tray = null;

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

// El modo desarrollo se pide explícitamente con --dev. Deducirlo de
// app.isPackaged hacía que un `electron .` normal intentara conectarse al
// servidor de Vite y se quedara en pantalla en blanco.
const isDev = process.argv.includes('--dev');
const DEV_URL = 'http://localhost:5173';

// Autoarranque relanza con este flag (ver autostart:set) para que la ventana
// no se muestre sola al iniciar sesión: el usuario la abre desde la bandeja
// si quiere. Sin esto, ready-to-show la enseña siempre.
const startHidden = process.argv.includes('--hidden');

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
    if (!startHidden) mainWindow.show();
  });
  loadRenderer(mainWindow);

  // --- Serie ---
  serial = new OrbySerial();
  const forward = (event, channel) =>
    serial.on(event, (payload) => mainWindow?.webContents.send(channel, payload));

  forward('connected', 'serial:connected');
  forward('disconnected', 'serial:disconnected');
  forward('error', 'serial:error');
  forward('searching', 'serial:searching');

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
    notif.on('click', () => {
      mainWindow?.show();
      mainWindow?.focus();
    });
    notif.show();
  });

  // Las líneas MACRO:<id> las dispara el firmware pero las ejecuta el PC; se
  // interceptan aquí además de seguir reenviándolas al renderer como siempre,
  // para que la consola de la app también las vea.
  serial.on('data', (line) => {
    if (line.startsWith('MACRO:')) {
      const id = parseInt(line.slice(6), 10);
      if (Number.isInteger(id)) executeMacro(id).catch((err) => console.error('Macro:', err.message));
    }
    mainWindow?.webContents.send('serial:data', line);
  });

  serial.startAutoScan();

  // --- Ventana en primer plano ---
  foreground = new ForegroundWatcher();
  foreground.on('change', (info) => mainWindow?.webContents.send('foreground:change', info));
  foreground.on('error', (err) => mainWindow?.webContents.send('foreground:error', err));

  if (config.load().autoProfile.enabled) foreground.start();

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
}

// Icono en la bandeja del sistema: es lo que permite que la app siga viva
// (y por tanto ejecutando secuencias) después de "cerrar" la ventana.
function createTray() {
  const iconPath = path.join(__dirname, '..', 'assets', 'orby-icon.png');
  const trayIcon = nativeImage.createFromPath(iconPath).resize({ width: 16, height: 16 });
  tray = new Tray(trayIcon);
  tray.setToolTip('OrbyGUI');

  const showWindow = () => { mainWindow?.show(); mainWindow?.focus(); };

  tray.setContextMenu(Menu.buildFromTemplate([
    { label: 'Abrir OrbyGUI', click: showWindow },
    { type: 'separator' },
    { label: 'Salir', click: () => { quitting = true; app.quit(); } },
  ]));

  tray.on('click', () => {
    if (!mainWindow) return;
    if (mainWindow.isVisible()) mainWindow.hide();
    else showWindow();
  });
  tray.on('double-click', showWindow);
}

// --- Actualizaciones automáticas ---
// Se comprueba contra las Releases de GitHub (ver "publish" en package.json).
// Solo tiene sentido con la app empaquetada e instalada: en dev no hay
// instalador que sustituir ni feed de actualizaciones que consultar.
function setupAutoUpdater() {
  if (!app.isPackaged) return;

  autoUpdater.autoDownload = true;
  autoUpdater.autoInstallOnAppQuit = true;

  autoUpdater.on('error', (err) => console.error('AutoUpdater:', err.message));

  autoUpdater.on('update-downloaded', (info) => {
    if (!Notification.isSupported()) return;
    const notif = new Notification({
      title: `OrbyGUI ${info.version} disponible`,
      body: 'Se instalará al cerrar la app. Haz clic para instalar ahora.',
      icon: path.join(__dirname, '..', 'assets', 'orby-icon.png'),
    });
    notif.on('click', () => {
      quitting = true;
      autoUpdater.quitAndInstall();
    });
    notif.show();
  });

  autoUpdater.checkForUpdates().catch((err) => console.error('AutoUpdater:', err.message));
}

// --- IPC: serie ---
ipcMain.handle('serial:send', async (_e, cmd) => (serial ? serial.sendCommand(cmd) : false));
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
  app.on('second-instance', () => {
    if (!mainWindow) return;
    if (mainWindow.isMinimized()) mainWindow.restore();
    mainWindow.show();
    mainWindow.focus();
  });

  app.whenReady().then(() => {
    createWindow();
    createTray();
    setupAutoUpdater();

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
  });

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
    else mainWindow?.show();
  });
}
