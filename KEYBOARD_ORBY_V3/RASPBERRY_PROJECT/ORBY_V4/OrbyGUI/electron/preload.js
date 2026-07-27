const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('orby', {
  // Comunicación serie
  sendCommand: (cmd) => ipcRenderer.invoke('serial:send', cmd),
  getDeviceInfo: () => ipcRenderer.invoke('serial:getInfo'),
  getStatus: () => ipcRenderer.invoke('serial:getStatus'),
  reconnect: () => ipcRenderer.invoke('serial:reconnect'),

  onConnected: (cb) => ipcRenderer.on('serial:connected', (_e, info) => cb(info)),
  onDisconnected: (cb) => ipcRenderer.on('serial:disconnected', () => cb()),
  onData: (cb) => ipcRenderer.on('serial:data', (_e, line) => cb(line)),
  onError: (cb) => ipcRenderer.on('serial:error', (_e, err) => cb(err)),
  onSearching: (cb) => ipcRenderer.on('serial:searching', () => cb()),

  // Configuración local (solo del PC, no del firmware)
  getConfig: () => ipcRenderer.invoke('config:get'),
  setConfig: (patch) => ipcRenderer.invoke('config:set', patch),

  // Copias de seguridad en un archivo del PC
  saveBackup: (data) => ipcRenderer.invoke('backup:save', data),
  loadBackup: () => ipcRenderer.invoke('backup:load'),

  // Detección de la aplicación en primer plano
  foreground: {
    start: () => ipcRenderer.invoke('foreground:start'),
    stop: () => ipcRenderer.invoke('foreground:stop'),
    current: () => ipcRenderer.invoke('foreground:current'),
    available: () => ipcRenderer.invoke('foreground:available'),
    onChange: (cb) => ipcRenderer.on('foreground:change', (_e, info) => cb(info)),
    onError: (cb) => ipcRenderer.on('foreground:error', (_e, err) => cb(err)),
  },

  // Controles de ventana
  minimize: () => ipcRenderer.send('window:minimize'),
  maximize: () => ipcRenderer.send('window:maximize'),
  close: () => ipcRenderer.send('window:close'),
});
