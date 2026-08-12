// Estado de las actualizaciones de la propia app.
//
// Copia local de lo que informa el proceso principal. Lo consumen dos sitios: el
// aviso de la barra de título y la tarjeta "Aplicación" de Ajustes.
//
// **La app se actualiza sola.** Con `auto` puesto —que es lo de fábrica— el backend
// encadena `available` → `downloading` → `downloaded` → reinicio sin que nadie apriete
// nada, y aquí no hay más que informar de por dónde va. Con `auto` apagado se para en
// `available` y espera al botón de Ajustes. Esta capa no decide: pinta el estado que le
// llegue (ver src-tauri/src/updater.rs).

const listeners = new Set();

export const update = {
  // 'dev' | 'idle' | 'checking' | 'downloading' | 'downloaded' | 'available' | 'error'
  status: 'idle',
  version: '',      // versión instalada (puede ser una preversión: «1.0.0-alpha»)
  newVersion: null, // versión que se está bajando o ya está lista
  percent: 0,
  error: null,
  auto: true,       // si se instala sola; el interruptor de Ajustes
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

export function setAuto(enabled) {
  return window.orby?.updater?.setAuto?.(enabled) ?? null;
}

// Texto para la tarjeta de Ajustes. El aviso de la barra usa el suyo, más corto.
export function describe() {
  switch (update.status) {
    case 'dev':         return 'No disponible en modo desarrollo';
    case 'checking':    return 'Comprobando…';
    case 'downloading': return `Descargando la ${update.newVersion}… (${update.percent}%)`;
    // Con el automático puesto, `downloaded` dura lo que tarde el reinicio, salvo que
    // haya algo en marcha que no se puede interrumpir (un firmware instalándose, una
    // grabación). Sin él, es el estado en el que se queda esperando al botón.
    case 'downloaded':  return update.auto
      ? `${update.newVersion} instalada: la app se va a reiniciar`
      : `${update.newVersion} lista para instalar`;
    case 'available':   return update.auto
      ? `Hay una versión nueva (${update.newVersion}): se está instalando`
      : `Hay una versión nueva: ${update.newVersion}`;
    case 'error':       return `Error: ${update.error}`;
    default:            return 'Estás en la última versión';
  }
}
