const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const path = require('path');
const fs = require('fs');
const { OrbySerial } = require('./serial');
const { ForegroundWatcher } = require('./foreground');
const config = require('./config');
const { executeMacro } = require('./macros');

let mainWindow = null;
let serial = null;
let foreground = null;

// El modo desarrollo se pide explícitamente con --dev. Deducirlo de
// app.isPackaged hacía que un `electron .` normal intentara conectarse al
// servidor de Vite y se quedara en pantalla en blanco.
const isDev = process.argv.includes('--dev');
const DEV_URL = 'http://localhost:5173';

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

  mainWindow.once('ready-to-show', () => mainWindow.show());
  loadRenderer(mainWindow);

  // --- Serie ---
  serial = new OrbySerial();
  const forward = (event, channel) =>
    serial.on(event, (payload) => mainWindow?.webContents.send(channel, payload));

  forward('connected', 'serial:connected');
  forward('disconnected', 'serial:disconnected');
  forward('error', 'serial:error');
  forward('searching', 'serial:searching');

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

  mainWindow.on('closed', () => {
    mainWindow = null;
    serial?.stopAutoScan();
    serial?.disconnect();
    foreground?.stop();
  });
}

// --- IPC: serie ---
ipcMain.handle('serial:send', async (_e, cmd) => (serial ? serial.sendCommand(cmd) : false));
ipcMain.handle('serial:getInfo', async () => serial?.getDeviceInfo() ?? null);
ipcMain.handle('serial:getStatus', async () => serial?.getStatus() ?? 'disconnected');
ipcMain.handle('serial:reconnect', async () => {
  serial?.startAutoScan();
  return Boolean(serial);
});

// --- IPC: configuración local ---
ipcMain.handle('config:get', async () => config.load());
ipcMain.handle('config:set', async (_e, patch) => config.save(patch));

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

app.whenReady().then(createWindow);

app.on('window-all-closed', () => app.quit());
app.on('before-quit', () => foreground?.stop());

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
