// Estado de las actualizaciones de la propia app.
//
// Copia local de lo que informa electron-updater en el proceso principal (ver
// setupAutoUpdater en electron/main.js). Lo consumen dos sitios: el aviso de la
// barra de título y la tarjeta "Aplicación" de Ajustes.

const listeners = new Set();

export const update = {
  status: 'idle',   // 'dev' | 'idle' | 'checking' | 'downloading' | 'downloaded' | 'error'
  version: '',      // versión instalada
  newVersion: null, // versión que se está bajando o ya está lista
  percent: 0,
  error: null,
};

function apply(next) {
  if (!next) return;
  Object.assign(update, next);
  listeners.forEach((cb) => cb(update));
}

export function onChange(cb) {
  listeners.add(cb);
  return () => listeners.delete(cb);
}

export async function init() {
  // Sin preload (p. ej. abriendo la interfaz en un navegador para maquetar) no
  // hay actualizador: se queda en 'dev' y ni el aviso ni la tarjeta molestan.
  if (!window.orby?.updater) {
    apply({ status: 'dev' });
    return;
  }
  window.orby.updater.onState(apply);
  apply(await window.orby.updater.get());
}

export function check() {
  return window.orby?.updater?.check?.() ?? null;
}

export function install() {
  return window.orby?.updater?.install?.() ?? false;
}

// Texto para la tarjeta de Ajustes. El aviso de la barra usa el suyo, más corto.
export function describe() {
  switch (update.status) {
    case 'dev':         return 'No disponible en modo desarrollo';
    case 'checking':    return 'Comprobando…';
    case 'downloading': return `Descargando ${update.newVersion} (${update.percent}%)`;
    case 'downloaded':  return `${update.newVersion} lista para instalar`;
    case 'error':       return `Error: ${update.error}`;
    default:            return 'Estás en la última versión';
  }
}
