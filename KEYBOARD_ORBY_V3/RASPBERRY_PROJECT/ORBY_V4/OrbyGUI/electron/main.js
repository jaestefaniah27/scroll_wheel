const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const { OrbySerial } = require('./serial');

let mainWindow = null;
let serial = null;

const isDev = !app.isPackaged;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 820,
    minWidth: 1024,
    minHeight: 700,
    backgroundColor: '#0a0a0f',
    frame: false,
    titleBarStyle: 'hidden',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
    icon: path.join(__dirname, '..', 'assets', 'orby-icon.png'),
  });

  if (isDev) {
    mainWindow.loadURL('http://localhost:5173');
  } else {
    mainWindow.loadFile(path.join(__dirname, '..', 'dist', 'index.html'));
  }

  // Initialize serial communication
  serial = new OrbySerial();

  // Forward serial events to the renderer
  serial.on('connected', (info) => {
    mainWindow?.webContents.send('serial:connected', info);
  });

  serial.on('disconnected', () => {
    mainWindow?.webContents.send('serial:disconnected');
  });

  serial.on('data', (line) => {
    mainWindow?.webContents.send('serial:data', line);
  });

  serial.on('error', (err) => {
    mainWindow?.webContents.send('serial:error', err);
  });

  serial.on('searching', () => {
    mainWindow?.webContents.send('serial:searching');
  });

  // Start auto-scanning for the device
  serial.startAutoScan();

  mainWindow.on('closed', () => {
    mainWindow = null;
    if (serial) {
      serial.stopAutoScan();
      serial.disconnect();
    }
  });
}

// IPC Handlers
ipcMain.handle('serial:send', async (_event, cmd) => {
  if (serial) {
    return serial.sendCommand(cmd);
  }
  return false;
});

ipcMain.handle('serial:getInfo', async () => {
  if (serial) {
    return serial.getDeviceInfo();
  }
  return null;
});

ipcMain.handle('serial:getStatus', async () => {
  if (serial) {
    return serial.getStatus();
  }
  return 'disconnected';
});

ipcMain.handle('serial:reconnect', async () => {
  if (serial) {
    serial.startAutoScan();
    return true;
  }
  return false;
});

// Window control IPC
ipcMain.on('window:minimize', () => mainWindow?.minimize());
ipcMain.on('window:maximize', () => {
  if (mainWindow?.isMaximized()) {
    mainWindow.unmaximize();
  } else {
    mainWindow?.maximize();
  }
});
ipcMain.on('window:close', () => mainWindow?.close());

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});
