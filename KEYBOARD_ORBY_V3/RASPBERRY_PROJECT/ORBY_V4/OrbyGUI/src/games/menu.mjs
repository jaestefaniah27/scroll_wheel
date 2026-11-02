// El menú de juegos del teclado: el icono de cada juego en una pantalla y la puerta de
// «salir» en la décima.
//
// Es un juego más para el motor (engine.js): dibuja y recibe teclas como cualquiera, y lo
// que elige el jugador sale por `valor`. Así el menú, las partidas y la pantalla final
// comparten el mismo reloj, el mismo control de flujo y la misma salida con la tecla 10.

import { texto, icono, invertir } from './dibujo.mjs';

// La tecla que se acaba de pulsar para entrar al menú (o para salir de una partida) puede
// seguir pulsada o repetirse sin querer: no se hace caso hasta pasado esto.
const ARMA_MS = 350;

// Cuánto se ve la opción elegida en negativo antes de pasar a ella.
const SELECCION_MS = 160;

export const MAX_JUEGOS = 9;

export function crearMenu(juegos) {
  const lista = juegos.slice(0, MAX_JUEGOS);
  return {
    id: 'menu',
    nombre: 'Menú de juegos',
    crear() {
      let tiempo = 0;
      let elegido = null;       // { accion: 'jugar', id } | { accion: 'salir' }
      let pantalla = 0;         // la que se pulsó, para enseñarla en negativo
      let desde = 0;

      return {
        get terminado() { return elegido !== null && tiempo - desde >= SELECCION_MS; },
        get puntos() { return 0; },
        get valor() { return elegido; },
        get estado() { return 'Elige un juego en el teclado'; },

        tick(dt) { tiempo += dt; },

        tecla(ev) {
          if (elegido || !ev.pulsada || !ev.pantalla || tiempo < ARMA_MS) return;
          if (ev.pantalla === 10) {
            elegido = { accion: 'salir' };
          } else if (lista[ev.pantalla - 1]) {
            elegido = { accion: 'jugar', id: lista[ev.pantalla - 1].id };
          } else {
            return;
          }
          pantalla = ev.pantalla;
          desde = tiempo;
        },

        dibujar(l) {
          // Cada juego es su icono; uno sin icono sale con su nombre.
          lista.forEach((j, i) => (j.icono ? icono(l[i], j.icono) : texto(l[i], j.nombre.toUpperCase())));
          icono(l[9], '#SALIR');
          if (elegido) invertir(l[pantalla - 1]);
        },
      };
    },
  };
}
