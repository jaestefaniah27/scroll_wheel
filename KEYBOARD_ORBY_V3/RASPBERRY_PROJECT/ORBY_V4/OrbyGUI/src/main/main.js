const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const { SerialManager } = require('./serialManager');
const { ActiveWindowTracker } = require('./activeWindowTracker');

let mainWindow;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 900,
    height: 700,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false
    }
  });

  // En modo desarrollo, cargamos desde el servidor de Vite
  mainWindow.loadURL('http://localhost:5173');
}

app.whenReady().then(() => {
  createWindow();

  // Iniciar Hardware Backend
  const serialManager = new SerialManager();
  const windowTracker = new ActiveWindowTracker();

  serialManager.startAutoConnect();
  windowTracker.startTracking((appName) => {
    serialManager.sendCommand(`APP:${appName}`);
  });

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
