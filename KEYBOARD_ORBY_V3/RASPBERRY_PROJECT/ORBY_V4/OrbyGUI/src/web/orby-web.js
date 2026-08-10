// El equivalente de electron/preload.js para el navegador.
//
// El renderer entero está escrito contra window.orby: 55 llamadas repartidas por
// quince ficheros. Reimplementar esa superficie sale infinitamente más barato que
// reescribir las vistas, y sobre todo evita tener dos interfaces que divergen desde
// el primer commit.
//
// Lo que aquí no puede existir (abrir programas, mover el ratón, complementos,
// detectar la ventana en primer plano) devuelve un valor vacío en vez de reventar:
// las vistas ya lo esconden preguntando a src/platform.js, y esto es la segunda red
// por si alguna se deja.

import { initConfig, getConfig, setConfig } from './config-store.js';
import { saveBackup, loadBackup } from './backup-file.js';
import * as serie from './transport-serial.js';

// Los eventos que en Electron llegan por ipcRenderer.on aquí son suscripciones al
// transporte. La firma se mantiene: un único callback, sin objeto de evento.
function puente(evento) {
  return (cb) => serie.on(evento, cb);
}

export async function instalarOrbyWeb() {
  await initConfig();

  window.orby = {
    platform: 'web',

    // Comunicación serie
    sendCommand: (cmd) => serie.enviar(cmd),
    getDeviceInfo: async () => serie.info(),
    getStatus: async () => serie.estado(),
    reconnect: () => serie.reconectar(),

    onConnected: puente('connected'),
    onDisconnected: puente('disconnected'),
    onData: puente('data'),
    onError: puente('error'),
    onSearching: puente('searching'),

    // Configuración local
    getConfig,
    setConfig,

    // Copias de seguridad
    saveBackup,
    loadBackup,

    // Nada de esto existe sin un proceso con permisos de escritorio. Devuelven
    // valores vacíos, no excepciones: una vista que se deje una comprobación tiene
    // que quedarse sin la función, no romper la página entera.
    getMousePosition: async () => null,
    pickAppOrFile: async () => ({ ok: false, canceled: true }),
    listInstalledApps: async () => [],

    plugins: {
      list: async () => [],
      install: async () => ({ ok: false }),
      uninstall: async () => ({ ok: false }),
      setEnabled: async () => ({ ok: false }),
      getSettings: async () => ({}),
      setSettings: async () => ({ ok: false }),
      test: async () => ({ ok: false, error: 'Los complementos necesitan la app de escritorio' }),
      openFolder: async () => ({ ok: false }),
    },

    autostart: {
      get: async () => false,
      set: async () => false,
    },

    recorder: {
      toggle: async () => ({ ok: false }),
      stop: async () => ({ ok: false }),
      status: async () => ({ recording: false }),
      onState: () => {},
    },

    foreground: {
      start: async () => ({ ok: false }),
      stop: async () => ({ ok: false }),
      current: async () => null,
      available: async () => false,
      onChange: () => {},
      onError: () => {},
    },

    updater: {
      get: async () => ({ status: 'idle' }),
      check: async () => null,
      install: async () => false,
      onState: () => {},
    },

    firmware: {
      get: async () => ({ status: 'idle' }),
      check: async () => null,
      update: async () => ({ ok: false }),
      cancel: async () => null,
      onState: () => {},
    },

    // No hay marco de ventana que controlar: la pestaña es del navegador.
    minimize: () => {},
    maximize: () => {},
    close: () => {},
  };
}
