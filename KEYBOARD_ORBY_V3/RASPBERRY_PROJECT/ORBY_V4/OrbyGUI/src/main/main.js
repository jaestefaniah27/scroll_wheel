const { app, BrowserWindow } = require('electron');
const { SerialManager } = require('./serialManager');
const { ActiveWindowTracker } = require('./activeWindowTracker');
const { setupIpcHandlers } = require('./ipcHandlers');

const VITE_URL = 'http://localhost:5173';
let mainWindow;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 800,
    backgroundColor: '#0d1117',
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      webSecurity: false
    }
  });

  mainWindow.webContents.openDevTools();

  mainWindow.webContents.on('did-fail-load', (_e, code, desc) => {
    console.log(`Load failed (${code}: ${desc}). Retrying in 500ms...`);
    setTimeout(() => mainWindow.loadURL(VITE_URL), 500);
  });

  mainWindow.webContents.on('did-finish-load', () => {
    console.log('Vite page loaded successfully!');
  });

  mainWindow.loadURL(VITE_URL);
}

app.whenReady().then(() => {
  createWindow();

  const serialManager = new SerialManager();
  const windowTracker = new ActiveWindowTracker();

  setupIpcHandlers(serialManager);

  serialManager.startAutoConnect();
  windowTracker.startTracking((appName) => {
    serialManager.sendCommand(`APP:${appName}`);
  });

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
