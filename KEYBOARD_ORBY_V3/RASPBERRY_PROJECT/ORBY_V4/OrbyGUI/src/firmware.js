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
import * as device from './device.js';
import { motivoParaEsperar, esActividadDelTeclado, VUELTA_MS, RECOMPROBAR_MS } from './fw-auto.mjs';

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
  ultimaComprobacion = Date.now();
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

// ================= Actualización automática =================
// El firmware se instala solo cuando el teclado lleva un rato sin que nadie lo
// toque (ver fw-auto.mjs para las reglas). Se activa por defecto: la clave
// `autoFirmware` de la configuración solo existe si alguien lo ha apagado, así
// que una instalación vieja sin ella queda activada.

let autoOn = true;
let fallida = null;          // versión que falló sola en esta sesión
let ultimaComprobacion = 0;
let enVuelta = false;
let avisar = null;
// Última vez que el teclado dio señales de uso. Al conectar vale 0 (nadie lo ha
// tocado): un teclado recién enchufado se actualiza sin esperar, ver alConectar.
let ultimaActividad = 0;

export function autoEnabled() {
  return autoOn;
}

export function setAutoEnabled(on) {
  autoOn = Boolean(on);
  return window.orby.setConfig({ autoFirmware: autoOn });
}

// `aviso(texto, tipo)` es el toast de ui.js: se recibe y no se importa para que
// este módulo no arrastre la interfaz.
export async function iniciarAuto(aviso) {
  if (!available()) return;
  avisar = aviso;
  try {
    autoOn = (await window.orby.getConfig())?.autoFirmware !== false;
  } catch { /* sin configuración legible se queda el valor por defecto */ }
  device.on('telemetry', (linea) => {
    if (esActividadDelTeclado(linea)) ultimaActividad = Date.now();
  });
  device.on('connected', () => { ultimaActividad = 0; });
  setInterval(vuelta, VUELTA_MS);
}

// Lo llama main.js cuando la conexión ya está preparada (perfiles leídos,
// macros sincronizadas y versión publicada comprobada): si hay firmware nuevo y
// nadie ha tocado el teclado, se instala ahora, sin esperar a la siguiente vuelta.
export function alConectar() {
  return vuelta();
}

async function vuelta() {
  // Una actualización dura más que una vuelta: sin esto la siguiente vuelta la
  // vería `busy()` y no haría nada, pero la comprobación de GitHub sí se pisaría.
  if (enVuelta) return;
  enVuelta = true;
  try {
    if (autoOn && state.connected && !busy() && Date.now() - ultimaComprobacion > RECOMPROBAR_MS) {
      await check();
    }
    const motivo = motivoParaEsperar({
      activado: autoOn,
      conectado: state.connected,
      disponible: fw.available,
      version: fw.latest?.version,
      ocupado: busy(),
      sinGuardar: state.dirty,
      sincronizando: state.syncing,
      bootsel: compat.supports(state.deviceInfo, 'bootsel'),
      inactivoMs: Date.now() - ultimaActividad,
      fallida,
    });
    if (motivo) return;

    const version = fw.latest.version;
    await update();
    if (fw.status === 'done') {
      avisar?.(`Firmware del teclado actualizado a la ${version}`, 'success', 8000);
    } else if (fw.status === 'error') {
      fallida = version;
      avisar?.(`No se pudo actualizar el firmware solo: ${fw.error}. Puedes reintentarlo desde Ajustes`, 'error', 9000);
    }
  } catch {
    // Un fallo de una vuelta (GitHub caído, el puente sin responder) no es
    // motivo para parar las siguientes.
  } finally {
    enVuelta = false;
  }
}
