// Posición dibujada de la rueda magnética.
//
// El firmware 3.0 manda `WHEEL:<incremento>:<ángulo 0-4095>`, así que la app
// puede pintar la rueda donde está de verdad en la mesa. Con firmware anterior
// solo llega el incremento y se acumula, que es lo que hacía antes.
//
// Tres ajustes, guardados en el PC porque son de montaje, no del teclado:
//   - sentido: el AS5600 lee el imán por el otro lado, y según cómo esté puesto
//     el giro puede salir espejado.
//   - desfase: el marcador físico de la tapa está pegado en un ángulo
//     cualquiera respecto al cero del sensor.
//   - forma del marcador: punto o raya.

import * as device from './device.js';

const COUNTS_PER_REV = 4096;

// Valores medidos sobre el Orby montado: con este imán y este marcador de tapa,
// el giro va espejado y el circulito queda 62° adelantado respecto al cero del
// sensor. Quien tenga otro montaje lo recalibra desde la tarjeta de la rueda.
export const dial = {
  invert: true,      // true = gira al contrario que el sentido natural
  offsetDeg: 62,     // desfase del marcador de la tapa, en grados
  marker: 'dot',     // 'dot' | 'line'
};

let rawDeg = null;     // último ángulo absoluto recibido, en grados
let fallbackDeg = 0;   // acumulado, solo para firmware sin ángulo absoluto
let shownDeg = 0;      // ángulo continuo que se pinta (sin saltos al dar la vuelta)

const listeners = new Set();

export function onUpdate(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

export async function init() {
  try {
    const cfg = await window.orby.getConfig();
    Object.assign(dial, cfg.wheelDial || {});
  } catch {
    // Sin configuración local nos quedamos con los valores por defecto.
  }
  device.on('telemetry', (line) => { if (line.startsWith('WHEEL:')) onTelemetry(line); });
}

export function setDial(patch) {
  Object.assign(dial, patch);
  window.orby.setConfig({ wheelDial: { ...dial } });
  recompute();
}

// El CDC puede entregar una línea partida pegada a la siguiente
// ("WHEEL:5:WHEEL:-3:2100"), y entonces el ángulo no se puede leer. La línea se
// descarta entera en vez de dejar pasar un NaN; la próxima llega a los 50 ms.
function onTelemetry(line) {
  const [deltaStr, angleStr] = line.slice(6).split(':');
  const delta = parseInt(deltaStr, 10);

  if (angleStr !== undefined) {
    const counts = parseInt(angleStr, 10);
    if (!Number.isFinite(counts)) return;
    rawDeg = (normalizeCounts(counts) / COUNTS_PER_REV) * 360;
  } else if (Number.isFinite(delta)) {
    fallbackDeg += (delta / COUNTS_PER_REV) * 360;
  } else {
    return;
  }
  recompute();
}

function normalizeCounts(counts) {
  return ((counts % COUNTS_PER_REV) + COUNTS_PER_REV) % COUNTS_PER_REV;
}

// El AS5600 cuenta en el sentido contrario al que se ve girar la rueda, así que
// el natural es el negativo; el interruptor cubre los montajes espejados.
function targetDeg() {
  const base = rawDeg !== null ? rawDeg : fallbackDeg;
  return (dial.invert ? base : -base) + dial.offsetDeg;
}

// Se lleva un ángulo continuo: pasar de 359° a 1° con un `rotate` normal daría
// una vuelta entera hacia atrás en la animación.
function recompute() {
  let diff = (targetDeg() - shownDeg) % 360;
  // `shownDeg` se realimenta de sí mismo: si una sola vez suma NaN se queda en
  // NaN para siempre y el marcador no se mueve más, aunque todo lo que llegue
  // después esté bien. Mejor no moverse esta vez que no moverse nunca más.
  if (!Number.isFinite(diff)) return;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  shownDeg += diff;
  for (const fn of listeners) fn(shownDeg);
}

export function angle() {
  return shownDeg;
}

// Deja el marcador de pantalla justo donde está el de la tapa: se gira la rueda
// hasta poner el circulito arriba del todo y se llama a esto.
export function alignHere() {
  const base = rawDeg !== null ? rawDeg : fallbackDeg;
  const natural = dial.invert ? base : -base;
  setDial({ offsetDeg: normalize(-natural) });
}

export function normalize(deg) {
  return ((deg % 360) + 360) % 360;
}

// Marcador de la rueda: el mismo elemento en el dashboard y en el editor, para
// que los dos respondan igual a los ajustes.
export function markerHtml(id) {
  return `<div class="dial-marker ${dial.marker === 'line' ? 'as-line' : 'as-dot'}" id="${id}"></div>`;
}
