// Estado de la actualización del firmware del teclado.
//
// Espejo en el renderer de electron/firmware.js, igual que updater.js lo es de
// las actualizaciones de la propia app. Lo consume la tarjeta "Firmware del
// teclado" de Ajustes.
//
// La diferencia con updater.js: aquí la app decide *hasta qué versión* se puede
// actualizar. El firmware y OrbyGUI se publican por separado, y ofrecer uno más
// nuevo que el que la app sabe manejar dejaría el teclado hablando un idioma
// que nadie entiende. Ese tope es FW_RECOMMENDED, y sale de compat.js.

import * as compat from './compat.js';
import { state } from './store.js';

const listeners = new Set();

export const fw = {
  status: 'idle',   // 'idle' | 'checking' | 'downloading' | 'bootsel' | 'flashing' | 'done' | 'error'
  current: null,    // la del teclado conectado
  latest: null,     // { version, asset, notes } de la última release compatible
  available: false,
  percent: 0,
  manual: false,    // esperando a que el usuario entre en BOOTSEL a mano
  error: null,
};

function apply(next) {
  if (!next) return;
  Object.assign(fw, next);
  listeners.forEach((cb) => cb(fw));
}

export function onChange(cb) {
  listeners.add(cb);
  return () => listeners.delete(cb);
}

export function available() {
  return Boolean(window.orby?.firmware);
}

export async function init() {
  if (!available()) return;
  window.orby.firmware.onState(apply);
  apply(await window.orby.firmware.get());
}

// El resultado se aplica aquí mismo, además de llegar por el evento de estado.
//
// Los dos caminos son mensajes distintos: la respuesta de la petición y el
// aviso 'firmware:state'. Nada garantiza cuál llega antes, y quien hacía
// `await firmware.check()` y miraba `fw.latest` se encontraba el estado de
// antes: en el primer arranque eso es `latest: null`, y Ajustes decía "No hay
// firmware publicado" justo después de una comprobación que había ido bien.
export async function check() {
  if (!available()) return null;
  const st = await window.orby.firmware.check({
    maxFw: compat.FW_RECOMMENDED,
    currentFw: state.deviceInfo?.fw ?? null,
  });
  apply(st);
  return st;
}

// `viaBootsel` decide qué se le pide al usuario: si el teclado sabe reiniciarse
// solo (bandera BOOTSEL=1), nada; si no, que lo enchufe con el botón pulsado.
export async function update() {
  if (!available()) return null;
  const st = await window.orby.firmware.update({
    viaBootsel: compat.supports(state.deviceInfo, 'bootsel'),
  });
  apply(st);
  return st;
}

export function cancel() {
  return window.orby?.firmware?.cancel?.() ?? null;
}

export function describe() {
  switch (fw.status) {
    case 'checking':    return 'Comprobando…';
    case 'downloading': return `Descargando ${fw.latest?.version ?? ''} (${fw.percent}%)`;
    case 'bootsel':     return fw.manual
      ? 'Desenchufa el teclado, enchúfalo con BOOTSEL pulsado y suéltalo'
      : 'Reiniciando el teclado en modo de actualización…';
    case 'flashing':    return 'Copiando el firmware… no desconectes el teclado';
    case 'done':        return 'Firmware actualizado. El teclado se está reiniciando';
    case 'error':       return `Error: ${fw.error}`;
    default:
      if (!fw.latest) return 'Sin comprobar';
      if (fw.available) return `Disponible la ${fw.latest.version}`;
      return 'El teclado está al día';
  }
}

// Si tiene sentido enseñar el botón de actualizar. Se deja pulsar aunque no
// haya versión nueva —reinstalar el mismo firmware es la forma de recuperar un
// teclado que se quedó a medias— pero no mientras hay algo en marcha.
export function busy() {
  return ['checking', 'downloading', 'bootsel', 'flashing'].includes(fw.status);
}
