// Simon: el teclado enciende una secuencia de teclas y hay que repetirla. Cada ronda
// añade una más.
//
// Se juega con las nueve teclas de la rejilla; la décima pantalla lleva la ronda.

import { rect, linea, px, texto, pastilla } from './dibujo.mjs';

// Apagado: el borde del pulsador y una sombra debajo, como si tuviera recorrido.
function apagado(fb) {
  pastilla(fb, 14, 8, 44, 24, 7, 1);
  pastilla(fb, 16, 10, 40, 19, 5, 0);
  for (let x = 22; x < 50; x += 2) px(fb, x, 34, 1);
}

// Encendido: macizo, hundido dos píxeles y con un halo de rayos alrededor.
function encendido(fb) {
  pastilla(fb, 14, 10, 44, 24, 7, 1);
  pastilla(fb, 20, 14, 32, 4, 2, 0);   // el brillo de arriba
  for (const [x0, y0, x1, y1] of [[2, 4, 8, 8], [70, 4, 64, 8], [2, 38, 8, 33], [70, 38, 64, 33], [2, 21, 8, 21], [64, 21, 70, 21], [36, 1, 36, 6]]) {
    linea(fb, x0, y0, x1, y1, 1);
  }
}

const ENCENDIDA_MS = 420;
const APAGADA_MS = 180;
const PRIMERA_PAUSA_MS = 700;
const ENTRE_RONDAS_MS = 650;
const PULSACION_MS = 160;      // cuánto se queda encendida una tecla al pulsarla
const ERROR_MS = 1400;

export default {
  id: 'simon',
  nombre: 'Simon',
  descripcion: 'Memoriza la secuencia de teclas y repítela. Cada ronda añade una más.',
  icono: '#SIMON',
  unidad: 'rondas',
  mejorSiMenor: false,

  crear({ aleatorio }) {
    const secuencia = [1 + aleatorio(9)];
    let tiempo = 0;
    // 'mostrar' | 'repetir' | 'bien' | 'error' | 'final'
    let fase = 'mostrar';
    let desde = 0;
    let indice = 0;          // por dónde va la secuencia (mostrándola o repitiéndola)
    let encendida = null;    // pantalla encendida ahora mismo, o null
    let pulsada = null;      // { pantalla, hasta } al pulsar
    let rondas = 0;          // rondas completadas
    let fallada = null;      // la tecla que se pulsó mal

    // Se acelera un poco con cada ronda: el primer día cuesta menos.
    const factor = () => Math.max(0.55, 1 - rondas * 0.04);

    function empezarMostrar() {
      fase = 'mostrar';
      desde = tiempo;
      indice = -1;           // -1 = la pausa antes del primer destello
      encendida = null;
    }
    empezarMostrar();

    return {
      // Al acabar se pasa directo a la pantalla de resultados y botones (final.mjs): sin un «FIN» propio.
      get terminado() { return fase === 'final'; },
      get puntos() { return rondas; },
      get estado() { return `${rondas} ${rondas === 1 ? 'ronda' : 'rondas'}`; },

      tick(dt) {
        tiempo += dt;
        const en = tiempo - desde;
        if (pulsada && tiempo >= pulsada.hasta) pulsada = null;

        if (fase === 'mostrar') {
          // Cada paso: pausa, destello, pausa. El índice -1 es el silencio inicial.
          const unidad = (ENCENDIDA_MS + APAGADA_MS) * factor();
          const arranque = indice < 0 ? PRIMERA_PAUSA_MS : 0;
          if (indice < 0 && en >= arranque) { indice = 0; desde = tiempo; return; }
          if (indice >= 0) {
            encendida = en < ENCENDIDA_MS * factor() ? secuencia[indice] : null;
            if (en >= unidad) {
              indice++;
              desde = tiempo;
              if (indice >= secuencia.length) { fase = 'repetir'; indice = 0; desde = tiempo; encendida = null; }
            }
          }
        } else if (fase === 'bien' && en >= ENTRE_RONDAS_MS) {
          secuencia.push(1 + aleatorio(9));
          empezarMostrar();
        } else if (fase === 'error' && en >= ERROR_MS) {
          fase = 'final';
          desde = tiempo;
        }
      },

      tecla(ev) {
        if (fase !== 'repetir' || !ev.pulsada || !ev.pantalla || ev.pantalla > 9) return;
        pulsada = { pantalla: ev.pantalla, hasta: tiempo + PULSACION_MS };
        if (ev.pantalla === secuencia[indice]) {
          indice++;
          if (indice >= secuencia.length) {
            rondas++;
            fase = 'bien';
            desde = tiempo;
          }
        } else {
          fallada = ev.pantalla;
          fase = 'error';
          desde = tiempo;
        }
      },

      dibujar(lienzo) {
        for (let p = 1; p <= 9; p++) {
          const fb = lienzo[p - 1];
          const luz = encendida === p || (pulsada && pulsada.pantalla === p);
          // Al fallar, la que tocaba parpadea encendida, para ver qué se esperaba.
          const esperada = fase === 'error' && secuencia[indice] === p;
          if (fase === 'error' && fallada === p) {
            // La que se pulsó mal, tachada: así se ve el error además de la buena.
            apagado(fb);
            for (let g = -1; g <= 1; g++) { linea(fb, 24 + g, 12, 48 + g, 28, 1); linea(fb, 48 + g, 12, 24 + g, 28, 1); }
          } else if (luz || (esperada && Math.floor((tiempo - desde) / 200) % 2 === 0)) encendido(fb);
          else apagado(fb);
        }
        if (fase === 'final') texto(lienzo[4], 'FIN');
        // El marcador cuenta las rondas superadas, antes y después de fallar: enseñar la
        // ronda en curso y, al fallar, una menos, parecía que el error quitaba un punto.
        texto(lienzo[9], rondas);
      },
    };
  },
};
