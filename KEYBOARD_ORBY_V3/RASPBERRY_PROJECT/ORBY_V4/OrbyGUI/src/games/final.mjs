// La pantalla de después de una partida, en el teclado:
//
//   PARTIDAS │ PUNTOS │ RECORD        lo que ha salido y cómo va
//            │ (juego)│               el icono del juego, para saber de qué es
//   OTRA VEZ │ MENU   │ SALIR         los tres botones, abajo, con la tecla bajo cada icono
//
// Los botones abajo porque es donde caen los dedos al acabar, y con icono porque se
// reconocen sin leer. Como el menú, es un juego más para el motor y lo que se elige sale
// por `valor`.

import { texto, icono, etiquetaValor, invertir } from './dibujo.mjs';

// Quien juega con prisa sigue pulsando teclas cuando se acaba la partida, y esas pulsaciones
// no son una elección: sin esta guarda, el último topo fallado podía darle a «OTRA VEZ».
const ARMA_MS = 900;
const SELECCION_MS = 160;
const PARPADEO_MS = 350;

// La pantalla (1-10) de cada botón: la fila de abajo de la rejilla.
const ACCIONES = { 7: 'otra', 8: 'menu', 9: 'salir' };

// `datos`: { juego, textoPuntos, textoMejor, partidas, record }; el texto ya formateado.
export function crearFinal(datos) {
  return {
    id: 'final',
    nombre: 'Fin de partida',
    crear() {
      let tiempo = 0;
      let elegido = null;
      let pantalla = 0;
      let desde = 0;

      return {
        get terminado() { return elegido !== null && tiempo - desde >= SELECCION_MS; },
        get puntos() { return 0; },
        get valor() { return elegido; },
        get estado() { return `${datos.juego.nombre}: ${datos.textoPuntos} (récord ${datos.textoMejor})`; },

        tick(dt) { tiempo += dt; },

        tecla(ev) {
          if (elegido || !ev.pulsada || tiempo < ARMA_MS) return;
          const accion = ACCIONES[ev.pantalla];
          if (!accion) return;
          elegido = { accion };
          pantalla = ev.pantalla;
          desde = tiempo;
        },

        dibujar(l) {
          etiquetaValor(l[0], 'PARTIDAS', datos.partidas);
          etiquetaValor(l[1], 'PUNTOS', datos.textoPuntos);
          etiquetaValor(l[2], datos.record ? 'RECORD!' : 'RECORD', datos.textoMejor);
          if (datos.juego.icono) icono(l[4], datos.juego.icono);
          else texto(l[4], datos.juego.nombre.toUpperCase());
          icono(l[6], '#OTRA');
          icono(l[7], '#MENU');
          icono(l[8], '#SALIR');
          // Un récord nuevo parpadea un rato: es lo único que merece llamar la atención.
          if (datos.record && tiempo < 3000 && Math.floor(tiempo / PARPADEO_MS) % 2 === 1) invertir(l[2]);
          if (elegido) invertir(l[pantalla - 1]);
        },
      };
    },
  };
}
