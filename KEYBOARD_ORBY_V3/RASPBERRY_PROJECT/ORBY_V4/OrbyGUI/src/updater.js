// Estado de las actualizaciones de la propia app.
//
// Copia local de lo que informa el proceso principal. Lo consumen dos sitios: el
// aviso de la barra de título y la tarjeta "Aplicación" de Ajustes.
//
// Los dos backends hacen ya lo mismo —descargar e instalar solos—, así que esta capa
// solo pinta el estado que le llegue: `checking` → `downloading` → `downloaded`, y de
// ahí el proceso se cierra para instalarse. Hubo un tercer estado, `available`, de
// cuando la vía de Tauri se limitaba a avisar y abrir la página de la release; se quitó
// al firmar las actualizaciones (ver src-tauri/src/updater.rs).
//
// `downloaded` ya no significa "listo, pulsa para instalar" en el caso normal: la
// instalación va sola. Se queda ahí, con la insignia encendida, solo cuando el
// actualizador se ha retirado por haber cambios sin escribir en la Flash del teclado.

import { state, subscribe } from './store.js';

const listeners = new Set();

export const update = {
  // 'dev' | 'idle' | 'checking' | 'downloading' | 'downloaded' | 'error'
  status: 'idle',
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
  vigilarCambiosSinGuardar();
  apply(await window.orby.updater.get());
}

// Instalar cierra el proceso, y el guardado a Flash es un temporizador de 1500 ms
// (scheduleAutoSave en main.js): morirse dentro de esa ventana pierde una escritura que
// nadie va a repetir. El backend necesita saberlo para retirarse, y `state.dirty` solo
// existe aquí, así que se le va contando cada vez que cambia.
//
// Se manda solo cuando cambia, y no en cada notify(): notify() se dispara con cada tecla
// que se toca, y esto cruza el puente al backend.
function vigilarCambiosSinGuardar() {
  let ultimo = null;
  const contar = () => {
    if (state.dirty === ultimo) return;
    ultimo = state.dirty;
    window.orby.updater.ocupado(state.dirty);
  };
  subscribe(contar);
  contar();
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
    // Se llega aquí con la descarga hecha y la instalación retirada por haber cambios
    // sin guardar: decir "lista para instalar" a secas dejaba al usuario sin saber por
    // qué no se instalaba sola, que es lo que la app hace el resto del tiempo.
    case 'downloaded':  return `${update.newVersion} lista: se instalará al guardar los cambios`;
    case 'error':       return `Error: ${update.error}`;
    default:            return 'Estás en la última versión';
  }
}
