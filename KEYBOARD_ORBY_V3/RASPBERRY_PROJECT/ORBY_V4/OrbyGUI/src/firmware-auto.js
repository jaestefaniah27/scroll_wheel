// Cuándo se puede flashear el teclado solo, sin decirle nada al usuario.
//
// El *cómo* ya estaba entero en `src-tauri/src/firmware.rs`: descargar el `.uf2`,
// comprobar su tamaño, pedirle al teclado que se reinicie en el cargador (`BOOTSEL`),
// esperar a que aparezca la unidad USB, copiar y esperar a que vuelva. Lo que faltaba —y
// es lo único que hay aquí— es **quién aprieta el botón y cuándo**.
//
// ## Por qué vive en el renderer y no en Rust
//
// Porque las condiciones que hacen que un momento sea bueno solo se conocen aquí: si hay
// cambios sin escribir en Flash (`state.dirty`), si la grabadora está en marcha, y sobre
// todo si el teclado lleva un rato sin que nadie lo toque. Bajarlo a Rust obligaría a
// duplicar media `store.js` al otro lado del puente.
//
// ## Qué cuenta como «tocar el teclado»
//
// `device.js` ya separa las líneas que el teclado manda **por su cuenta** (`telemetry`:
// una tecla, un giro del mando, un cambio de contexto) de las que son **respuesta a algo
// que preguntó la app** (`response`). Solo las primeras reinician el reloj de ocio. Esa
// distinción es la que hace que esto funcione: el latido `HOST_APP` que la app manda cada
// ocho segundos genera tráfico continuo, y contándolo como actividad el teclado no estaría
// ocioso jamás.
//
// ## Lo que se pierde si esto se equivoca
//
// Durante la copia el teclado **no existe**: se reinicia en el cargador de la ROM y
// aparece una unidad USB en su lugar. Son unos segundos en los que no responde a nada. Por
// eso las guardas son conservadoras y por eso un fallo **no se reintenta en bucle**: se
// deja anotado, la tarjeta de Ajustes recupera su botón manual y se vuelve a intentar en la
// siguiente comprobación, que es al reconectar el teclado.

import * as device from './device.js';
import * as firmware from './firmware.js';
import { state } from './store.js';
import * as platform from './platform.js';

// Cuánto tiene que llevar el teclado sin actividad. Cinco minutos no es un número
// afinado: es «se ha ido a por café», que es exactamente el momento que se busca.
const OCIO_MS = 5 * 60 * 1000;

// Cada cuánto se mira si ya se dan las condiciones. Un minuto: esto no tiene ninguna
// prisa, y sondear más a menudo solo gastaría vueltas para llegar a la misma respuesta.
const SONDEO_MS = 60 * 1000;

let ultimaActividad = Date.now();
let temporizador = null;
let enMarcha = false;
// La versión cuyo intento ya falló. Se guarda para no volver a intentarla en esta sesión:
// si el `.uf2` de la 4.7 no se pudo copiar, reintentarlo cada minuto no lo va a arreglar.
let versionFallida = null;
let avisar = () => {};

function tocado() {
  ultimaActividad = Date.now();
}

// Si el usuario lo ha apagado en Ajustes. Se lee de la configuración local en cada vuelta
// —y no una vez al arrancar— para que apagar el interruptor tenga efecto inmediato, sin
// esperar a reiniciar la app.
async function automatico() {
  try {
    const cfg = await window.orby.getConfig();
    return cfg?.autoFirmware !== false;
  } catch {
    // Sin poder leer la configuración, no se flashea: ante la duda, no tocar el teclado.
    return false;
  }
}

// Lo mismo que `malMomento`, pero para lo que hay que preguntarle al backend. Se separa
// porque es asíncrono y `malMomento` se usa también para releer justo antes de disparar.
///
// La grabadora se pregunta al proceso principal y no a `store.js` porque es él quien
// tiene los ganchos del ratón y el teclado puestos: es el único que sabe de verdad si hay
// una captura en marcha o una secuencia reproduciéndose.
async function malMomentoAsync() {
  try {
    const grabadora = await window.orby.recorder.status();
    if (grabadora?.recording) return 'la grabadora está grabando';
    if (grabadora?.playingId != null) return 'hay una secuencia reproduciéndose';
  } catch {
    return 'no se pudo consultar la grabadora';
  }
  if (!(await automatico())) return 'el automático está apagado';
  return null;
}

// Por qué **no** se puede flashear ahora, o `null` si se puede.
///
// Devuelve el motivo en texto y no un booleano porque acaba en la consola: con algo que
// pasa solo, «no ha pasado» sin explicación es imposible de diagnosticar.
function malMomento() {
  if (!state.connected || !state.deviceInfo) return 'no hay teclado conectado';
  if (!firmware.fw.available || !firmware.fw.latest) return 'no hay firmware nuevo';
  if (firmware.busy()) return 'ya hay algo en marcha';
  if (firmware.fw.latest.version === versionFallida) return 'esta versión ya falló en esta sesión';

  // El reinicio en el cargador se lleva por delante lo que solo esté en RAM, y las
  // variaciones por aplicación viven exactamente ahí (ver variants.js). Es el mismo motivo
  // por el que el botón manual de Ajustes obliga a guardar antes.
  if (state.dirty) return 'hay cambios sin guardar en Flash';

  const quieto = Date.now() - ultimaActividad;
  if (quieto < OCIO_MS) {
    return `el teclado se ha usado hace ${Math.round(quieto / 1000)} s`;
  }
  return null;
}

async function vuelta() {
  if (enMarcha) return;

  if (malMomento()) return;
  if (await malMomentoAsync()) return;

  // Se relee después de los `await`: mientras se consultaba el backend el usuario ha
  // podido ponerse a teclear, y entonces el momento ya no es bueno.
  if (malMomento()) return;

  const version = firmware.fw.latest.version;
  enMarcha = true;
  try {
    const st = await firmware.update();
    if (st?.status === 'done') {
      avisar(`Firmware actualizado a ${version}`);
    } else {
      versionFallida = version;
      avisar(`No se pudo actualizar el firmware: ${st?.error ?? 'sin detalle'}`, 'error', 9000);
    }
  } catch (err) {
    versionFallida = version;
    avisar(`No se pudo actualizar el firmware: ${err.message}`, 'error', 9000);
  } finally {
    enMarcha = false;
    // Después de un flasheo el teclado se reinicia y vuelve a enumerar: eso es actividad
    // suya, no del usuario, pero da igual — reiniciar el reloj aquí evita encadenar dos
    // intentos seguidos si algo saliera raro.
    tocado();
  }
}

// Arranca la vigilancia. Idempotente: llamarla dos veces no duplica el temporizador.
export function init(notificar) {
  if (!platform.can('firmwareUpdate') || temporizador) return;
  if (typeof notificar === 'function') avisar = notificar;

  // Lo que el teclado manda por su cuenta. `response` NO cuenta: son contestaciones a lo
  // que pregunta la app, incluido el latido de cada ocho segundos.
  device.on('telemetry', tocado);
  // Conectar y desconectar también cuentan: enchufar el teclado es tocarlo, y arrancar el
  // reloj desde cero evita flashear a los dos segundos de haberlo puesto.
  device.on('connected', tocado);

  tocado();
  temporizador = setInterval(() => { vuelta().catch(() => {}); }, SONDEO_MS);
}

// Solo para los tests y para el interruptor: si se apaga el automático, no hace falta
// seguir sondeando.
export function stop() {
  if (temporizador) clearInterval(temporizador);
  temporizador = null;
}
