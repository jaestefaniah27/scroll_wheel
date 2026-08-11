// El equivalente de electron/preload.js para Tauri.
//
// Tercera implementación de la misma superficie: escritorio con Electron, navegador con
// Web Serial (src/web/orby-web.js) y esto. El renderer entero está escrito contra
// `window.orby`, así que reimplementar el objeto sale infinitamente más barato que
// reescribir las vistas, y sobre todo evita tener interfaces que divergen.
//
// Los nombres de los EVENTOS se conservan con dos puntos (`serial:connected`) porque son
// los que ya conoce el renderer. Los nombres de los COMANDOS no pueden llevarlos —Rust
// no admite `:` en un identificador— y van en snake_case. Es la única diferencia con
// Electron, y queda encerrada en este fichero.

// La API se coge del global que Tauri inyecta (`withGlobalTauri: true` en
// tauri.conf.json) en vez de del paquete `@tauri-apps/api`. A propósito: un `import` de
// ese paquete lo tendría que resolver vite **en los tres builds**, incluido el de
// navegador, así que una dependencia que solo usa una de las vías obligaría a instalarla
// para compilar cualquiera de las otras. Así el frontend sigue compilando sin nada nuevo.
const { core, event } = window.__TAURI__ ?? {};

// Los `on*` de Electron entregan solo la carga, nunca el objeto de evento, y no tienen
// forma de darse de baja. Se replica igual: inventar aquí un `off` daría una interfaz
// que las otras dos vías no tienen.
function puente(evento) {
  return (cb) => { event.listen(evento, (e) => cb(e.payload)); };
}

const invoke = (cmd, args) => core.invoke(cmd, args);

export async function instalarOrbyTauri() {
  window.orby = {
    platform: 'tauri',

    // Comunicación serie
    sendCommand: (cmd) => invoke('serial_send', { cmd }),
    getDeviceInfo: () => invoke('serial_get_info'),
    getStatus: () => invoke('serial_get_status'),
    reconnect: () => invoke('serial_reconnect'),

    onConnected: puente('serial:connected'),
    onDisconnected: puente('serial:disconnected'),
    onData: puente('serial:data'),
    onError: puente('serial:error'),
    onSearching: puente('serial:searching'),

    // Ratón: posición actual del cursor (captura de posiciones al editar una secuencia).
    // Tiene que salir del mismo espacio de coordenadas con DPI que usa la reproducción,
    // o en una pantalla con escalado se graba un punto y se pincha en otro.
    getMousePosition: () => invoke('mouse_get_position'),

    // Configuración local (solo del PC, no del firmware)
    getConfig: () => invoke('config_get'),
    setConfig: (patch) => invoke('config_set', { patch }),

    // Complementos: solo su gestión. Las acciones las ejecuta el backend.
    plugins: {
      list: () => invoke('plugins_list'),
      install: () => invoke('plugins_install'),
      uninstall: (id) => invoke('plugins_uninstall', { id }),
      setEnabled: (id, enabled) => invoke('plugins_set_enabled', { id, enabled }),
      getSettings: (id) => invoke('plugins_get_settings', { id }),
      setSettings: (id, patch) => invoke('plugins_set_settings', { id, patch }),
      test: (id, values) => invoke('plugins_test', { id, values }),
      read: (id, op) => invoke('plugins_read', { id, op }),
      openFolder: () => invoke('plugins_open_folder'),
    },

    // Autoarranque con Windows
    autostart: {
      get: () => invoke('autostart_get'),
      set: (enabled) => invoke('autostart_set', { enabled }),
    },

    // Copias de seguridad en un archivo del PC
    saveBackup: (data) => invoke('backup_save', { data }),
    loadBackup: () => invoke('backup_load'),

    // Elegir una app o archivo (pestaña "App" y paso "Abrir…" de una secuencia)
    pickAppOrFile: (kind) => invoke('dialog_pick_app_or_file', { kind }),

    // Apps instaladas (menú Inicio), para elegir una sin ir a buscar el .exe a mano
    listInstalledApps: () => invoke('apps_list_installed'),

    // Grabar operación: capturar ratón y teclado del PC y repetirlo
    recorder: {
      toggle: (id) => invoke('recorder_toggle', { id }),
      stop: () => invoke('recorder_stop'),
      status: () => invoke('recorder_status'),
      onState: puente('recorder:state'),
    },

    // Detección de la aplicación en primer plano
    foreground: {
      start: () => invoke('foreground_start'),
      stop: () => invoke('foreground_stop'),
      current: () => invoke('foreground_current'),
      available: () => invoke('foreground_available'),
      onChange: puente('foreground:change'),
      onError: puente('foreground:error'),
    },

    // Actualizaciones de la propia app
    updater: {
      get: () => invoke('updater_get'),
      check: () => invoke('updater_check'),
      install: () => invoke('updater_install'),
      onState: puente('updater:state'),
    },

    // Firmware del teclado (releases `fw-v*` del mismo repositorio)
    firmware: {
      get: () => invoke('firmware_get'),
      check: (opts) => invoke('firmware_check', { opts }),
      update: (opts) => invoke('firmware_update', { opts }),
      cancel: () => invoke('firmware_cancel'),
      onState: puente('firmware:state'),
    },

    // Controles de ventana. El marco lo dibuja la app, así que estos tres son la única
    // forma de minimizar, maximizar y cerrar. `close` esconde a la bandeja, no cierra.
    minimize: () => invoke('window_minimize'),
    maximize: () => invoke('window_maximize'),
    close: () => invoke('window_close'),
  };
}
