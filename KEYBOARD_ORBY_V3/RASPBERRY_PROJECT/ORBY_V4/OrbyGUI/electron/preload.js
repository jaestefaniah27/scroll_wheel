const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('orby', {
  // Serial communication
  sendCommand: (cmd) => ipcRenderer.invoke('serial:send', cmd),
  getDeviceInfo: () => ipcRenderer.invoke('serial:getInfo'),
  getStatus: () => ipcRenderer.invoke('serial:getStatus'),
  reconnect: () => ipcRenderer.invoke('serial:reconnect'),

  // Serial event listeners
  onConnected: (callback) => {
    ipcRenderer.on('serial:connected', (_event, info) => callback(info));
  },
  onDisconnected: (callback) => {
    ipcRenderer.on('serial:disconnected', () => callback());
  },
  onData: (callback) => {
    ipcRenderer.on('serial:data', (_event, line) => callback(line));
  },
  onError: (callback) => {
    ipcRenderer.on('serial:error', (_event, err) => callback(err));
  },
  onSearching: (callback) => {
    ipcRenderer.on('serial:searching', () => callback());
  },

  // Window controls
  minimize: () => ipcRenderer.send('window:minimize'),
  maximize: () => ipcRenderer.send('window:maximize'),
  close: () => ipcRenderer.send('window:close'),
});
