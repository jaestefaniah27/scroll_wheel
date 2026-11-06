// El motor de una partida: lleva la sesión con el teclado y hace girar el juego.
//
// El teclado es solo pantallas y entradas (modo GAME del firmware, 4.29): la lógica del
// juego corre aquí. Este módulo no sabe a qué se juega. Se ocupa de lo que es común:
//
//   · entrar y salir del modo juego, y salir también si el teclado lo pide solo
//   · llamar al juego a un ritmo fijo, pintar lo que dibuje y mandar al teclado solo lo
//     que ha cambiado (proto.mjs)
//   · no pasarse de fotogramas por delante del teclado (control de flujo con acuses)
//   · repartir al juego las teclas, los mandos y la rueda
//
// Un juego es un objeto con { id, nombre, crear(ctx) }; crear devuelve la partida:
//   tick(dt)        avanza la lógica `dt` milisegundos
//   dibujar(lienzo) pinta el estado en las diez pantallas (se llama con ellas en negro)
//   tecla(ev) · mando(ev) · mandoBoton(ev) · rueda(ev)   entradas, todas opcionales
//   enviado(n)      opcional: el último dibujar() salió al teclado como fotograma `n`
//   acuse(ev)       opcional: el teclado ha pintado hasta el fotograma `ev.n` (con `ev.us`)
//   terminado       true cuando la partida ha acabado y ya se ha enseñado el final
//   puntos          el resultado
//   detalle         opcional: lo que se guarda además de los puntos (ver scores.mjs)
//
// Toda entrada lleva `t`, el instante en el reloj del PC al llegar, y las teclas y los
// acuses `us`, el del reloj del teclado (null si el firmware no lo manda). Para medir
// tiempos de reacción vale el segundo: no le afecta lo que tarde el USB ni el PC.
//
// La tecla 10 (la de menú) está reservada: salir. Un juego no la recibe.

import * as device from '../device.js';
import {
  lienzo, copiarLienzo, limpiarLienzo, armarFotograma, leerEvento, puedeEnviar, EN_VUELO,
} from './proto.mjs';

// Cada cuánto despierta el reloj. El ritmo real lo marca el control de flujo, no esto:
// 4 ms es solo la rapidez con la que se entera el motor de que puede mandar otro.
const TIC_MS = 4;

// El firmware sale solo del juego si pasan 3 s sin oír a la app (GAME_TIMEOUT_MS). Con
// la pantalla quieta —el jugador pensando— hay que decirle que seguimos aquí.
const LATIDO_MS = 1000;

// Si un acuse se pierde, el control de flujo se quedaría esperando para siempre. Pasado
// esto sin novedad se da por entregado lo enviado.
const ACUSE_PERDIDO_MS = 500;

// Un tic muy tardío (el equipo se quedó colgado) no debe traducirse en que el juego
// salte medio segundo de golpe.
const DT_MAX_MS = 100;

const TECLA_SALIR = 10;

// El reloj va en un Worker (ver metronomo.worker.js). Donde no hay Workers —los tests en
// Node—, un setInterval corriente.
function metronomo(cb, ms) {
  try {
    const w = new Worker(new URL('./metronomo.worker.js', import.meta.url), { type: 'module' });
    w.onmessage = cb;
    w.postMessage({ ms });
    return () => { w.postMessage({}); w.terminate(); };
  } catch {
    const id = setInterval(cb, ms);
    return () => clearInterval(id);
  }
}

// El número de fotograma sigue entre sesiones y no vuelve a cero en cada una: el firmware
// acusa cuando cambia el último número pintado, y reutilizar uno ya visto dejaría un
// fotograma sin acuse hasta que caducara el control de flujo.
let secuencia = 0;

// Lanza una partida. Devuelve { resultado, cancelar }: `resultado` se cumple con
// { motivo, puntos, completa, valor } cuando acaba, y `cancelar()` la corta desde fuera (el
// botón de la vista). Rechaza si el teclado no deja entrar (sin conexión, un menú abierto).
//
// Por defecto, al acabar devuelve las pantallas al teclado (GAME:0). El menú del teclado
// encadena varias sesiones seguidas —menú, partida, pantalla final— y entre una y otra
// no debe salir del modo juego: quien las encadena pasa `cerrar: false` y cierra al final.
export function jugar(juego, { alFotograma = null, alEntrar = null, cerrar = true } = {}) {
  let terminar = () => {};
  const fin = new Promise((resolver) => { terminar = resolver; });

  const resultado = (async () => {
    await device.gameStart();

    const actual = lienzo();
    const enviado = lienzo();
    const ahora = () => performance.now();
    let primero = true;
    let n = secuencia;
    let acusado = n - 1;
    let ultimoEnvio = ahora();
    let ultimoTic = ahora();

    const partida = juego.crear({
      ahora,
      aleatorio: (tope) => Math.floor(Math.random() * tope),
    });

    function tic() {
      const t = ahora();
      const dt = Math.min(t - ultimoTic, DT_MAX_MS);
      ultimoTic = t;

      partida.tick(dt);
      limpiarLienzo(actual);
      partida.dibujar(actual);

      // Los acuses coalescen en el firmware (si dos fotogramas se pintan de una vez,
      // llega el del último), así que lo enviado y lo acusado nunca se desfasan más de
      // EN_VUELO; pero un acuse perdido dejaría al motor esperando para siempre.
      if (!puedeEnviar(n - 1, acusado) && t - ultimoEnvio > ACUSE_PERDIDO_MS) acusado = n - 1;

      if (puedeEnviar(n - 1, acusado)) {
        const f = armarFotograma(primero ? null : enviado, actual, n);
        if (f) {
          if (device.gameFrame(f.bytes)) {
            copiarLienzo(enviado, actual);
            partida.enviado?.(n);
            primero = false;
            n++;
            ultimoEnvio = t;
            alFotograma?.(actual, partida);
          }
        }
      }
      if (t - ultimoEnvio > LATIDO_MS) {
        device.gameKeepAlive();
        ultimoEnvio = t;
      }

      if (partida.terminado) terminar('fin');
    }

    const quitar = [
      device.on('telemetry', (linea) => {
        // El teclado ha tirado un fotograma a medias: lo que traía no se ha pintado,
        // y como solo se mandan las pantallas que cambian, no se repintaría nunca. El
        // siguiente va con las diez.
        if (linea.startsWith('ERR:GAME_BIN_TIMEOUT')) { primero = true; return; }
        const ev = leerEvento(linea);
        if (!ev) return;
        ev.t = ahora();
        switch (ev.tipo) {
          case 'acuse': acusado = Math.max(acusado, ev.n); partida.acuse?.(ev); break;
          case 'fin': terminar(ev.motivo); break;
          case 'tecla':
            if (ev.tecla === TECLA_SALIR) { if (ev.pulsada) terminar('salir'); }
            else partida.tecla?.(ev);
            break;
          case 'mando': partida.mando?.(ev); break;
          case 'mando-boton': partida.mandoBoton?.(ev); break;
          case 'rueda': partida.rueda?.(ev); break;
        }
      }),
      device.on('disconnected', () => terminar('desconectado')),
    ];

    const pararReloj = metronomo(tic, TIC_MS);
    alEntrar?.(partida);

    const motivo = await fin;

    pararReloj();
    quitar.forEach((f) => f());
    secuencia = n;
    // Si el teclado ya ha salido solo (menú, silencio, desconexión) no hay nada que
    // cerrar; si no, se le devuelven las pantallas.
    if (cerrar && (motivo === 'fin' || motivo === 'salir' || motivo === 'cancelado')) {
      await device.gameStop().catch(() => {});
    }
    return { motivo, puntos: partida.puntos, completa: motivo === 'fin', valor: partida.valor, detalle: partida.detalle ?? null };
  })();

  return { resultado, cancelar: () => terminar('cancelado') };
}

export { EN_VUELO };
