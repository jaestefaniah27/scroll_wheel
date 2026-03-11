const { app, BrowserWindow } = require('electron');
const { SerialManager } = require('./serialManager');
const { ActiveWindowTracker } = require('./activeWindowTracker');
const { setupIpcHandlers } = require('./ipcHandlers');

const VITE_URL = 'http://localhost:5173';
let mainWindow;
let serialManager;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1340,
    height: 860,
    minWidth: 900,
    minHeight: 600,
    backgroundColor: '#0d1117',
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      webSecurity: false
    }
  });

  mainWindow.webContents.openDevTools({ mode: 'detach' });

  mainWindow.webContents.on('did-fail-load', (_e, code, desc) => {
    console.log(`[Electron] Load failed (${code}: ${desc}). Retrying in 500ms...`);
    setTimeout(() => mainWindow.loadURL(VITE_URL), 500);
  });

  mainWindow.webContents.on('did-finish-load', () => {
    console.log('[Electron] Vite page loaded!');
    // Push current connection status immediately on page load
    if (serialManager) {
      mainWindow.webContents.send('device-status', {
        connected: serialManager.isConnected,
        port: serialManager._portPath || null
      });
    }
  });

  mainWindow.loadURL(VITE_URL);
}

app.whenReady().then(() => {
  createWindow();

  serialManager = new SerialManager();
  const windowTracker = new ActiveWindowTracker();

  // Setup IPC handlers (profile save/load, device commands)
  setupIpcHandlers(serialManager);

  // Push real-time device status changes to renderer
  serialManager.on('statusChange', (connected, port) => {
    console.log(`[Electron] Device ${connected ? 'connected' : 'disconnected'}`);
    mainWindow?.webContents.send('device-status', { connected, port });
  });

  // Push active app changes to renderer for auto-profile switching
  serialManager.startAutoConnect();
  windowTracker.startTracking((appName) => {
    serialManager.sendCommand(`APP:${appName}`);
    mainWindow?.webContents.send('app-changed', appName);
  });

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
