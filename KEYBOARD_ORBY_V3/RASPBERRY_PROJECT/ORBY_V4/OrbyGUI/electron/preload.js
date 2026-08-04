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

  // Ratón: posición actual del cursor en pantalla (captura de posiciones al editar una secuencia)
  getMousePosition: () => ipcRenderer.invoke('mouse:getPosition'),

  // Configuración local (solo del PC, no del firmware)
  getConfig: () => ipcRenderer.invoke('config:get'),
  setConfig: (patch) => ipcRenderer.invoke('config:set', patch),

  // Autoarranque con Windows (app.setLoginItemSettings)
  autostart: {
    get: () => ipcRenderer.invoke('autostart:get'),
    set: (enabled) => ipcRenderer.invoke('autostart:set', enabled),
  },

  // Copias de seguridad en un archivo del PC
  saveBackup: (data) => ipcRenderer.invoke('backup:save', data),
  loadBackup: () => ipcRenderer.invoke('backup:load'),

  // Elegir una app o archivo (pestaña "App" y paso "Abrir…" de una secuencia)
  pickAppOrFile: (kind) => ipcRenderer.invoke('dialog:pickAppOrFile', kind),

  // Apps instaladas (menú Inicio), para elegir una sin ir a buscar el .exe a mano
  listInstalledApps: () => ipcRenderer.invoke('apps:listInstalled'),

  // Grabar operación: capturar ratón y teclado del PC y repetirlo
  recorder: {
    toggle: (id) => ipcRenderer.invoke('recorder:toggle', id),
    stop: () => ipcRenderer.invoke('recorder:stop'),
    status: () => ipcRenderer.invoke('recorder:status'),
    onState: (cb) => ipcRenderer.on('recorder:state', (_e, info) => cb(info)),
  },

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
