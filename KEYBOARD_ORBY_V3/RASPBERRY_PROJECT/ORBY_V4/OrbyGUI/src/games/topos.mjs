// Topos: asoma un topo en una de las nueve teclas y hay que pulsarla antes de que se esconda.
//
// Las nueve primeras pantallas son los agujeros y la décima lleva el marcador: es la única
// que no cabe en la rejilla de 3x3, así que se aprovecha para lo que no es un blanco.

import { rect, disco, elipse, linea, destello, px, texto } from './dibujo.mjs';

const DURACION_MS = 30_000;
const FINAL_MS = 3_000;
const SUBIDA_MS = 110;       // lo que tarda el topo en asomar del todo
const VIDA_INICIAL_MS = 1100;
const VIDA_MINIMA_MS = 420;
const VIDA_BAJA_POR_ACIERTO_MS = 28;
const PAUSA_MIN_MS = 140;
const PAUSA_VARIABLE_MS = 330;
const GOLPE_MS = 220;        // cuánto se ve el topo mareado tras el acierto
const BORDE = 30;            // la fila de la boca del agujero
const ARRIBA = 15;           // dónde queda la cabeza del topo asomado del todo

export default {
  id: 'topos',
  nombre: 'Topos',
  descripcion: 'Pulsa la tecla donde asoma el topo antes de que se esconda. Treinta segundos; fallar resta.',
  icono: '#TOPOS',
  unidad: 'aciertos',
  mejorSiMenor: false,

  crear({ aleatorio }) {
    let tiempo = 0;
    let aciertos = 0;
    let fallos = 0;
    let fase = 'jugando';
    let faseDesde = 0;
    // El topo: en qué pantalla (1-9), desde cuándo asoma y cuánto aguanta. null = no hay.
    let topo = null;
    let proximo = 600;      // cuándo sale el siguiente
    let golpe = null;       // { pantalla, hasta }: el acierto que se está enseñando
    let erroneo = null;     // { pantalla, hasta }: la tecla mala

    const vida = () => Math.max(VIDA_MINIMA_MS, VIDA_INICIAL_MS - aciertos * VIDA_BAJA_POR_ACIERTO_MS);

    function nuevoTopo() {
      let p;
      do { p = 1 + aleatorio(9); } while (topo && p === topo.pantalla);
      topo = { pantalla: p, desde: tiempo, vida: vida() };
    }

    return {
      get terminado() { return fase === 'final' && tiempo - faseDesde >= FINAL_MS; },
      get puntos() { return Math.max(0, aciertos - fallos); },
      get estado() { return `${Math.max(0, aciertos - fallos)} puntos · ${Math.max(0, Math.ceil((DURACION_MS - tiempo) / 1000))} s`; },

      tick(dt) {
        tiempo += dt;
        if (fase === 'jugando') {
          if (tiempo >= DURACION_MS) { fase = 'final'; faseDesde = tiempo; topo = null; return; }
          if (topo && tiempo - topo.desde >= topo.vida) {
            // Se escondió sin que lo pulsaran: no resta, solo no suma.
            topo = null;
            proximo = tiempo + PAUSA_MIN_MS + aleatorio(PAUSA_VARIABLE_MS);
          } else if (!topo && tiempo >= proximo) {
            nuevoTopo();
          }
        }
        if (golpe && tiempo >= golpe.hasta) golpe = null;
        if (erroneo && tiempo >= erroneo.hasta) erroneo = null;
      },

      tecla(ev) {
        if (fase !== 'jugando' || !ev.pulsada || !ev.pantalla || ev.pantalla > 9) return;
        if (topo && ev.pantalla === topo.pantalla) {
          aciertos++;
          golpe = { pantalla: topo.pantalla, hasta: tiempo + GOLPE_MS };
          topo = null;
          proximo = tiempo + PAUSA_MIN_MS + aleatorio(PAUSA_VARIABLE_MS);
        } else {
          fallos++;
          erroneo = { pantalla: ev.pantalla, hasta: tiempo + GOLPE_MS };
        }
      },

      dibujar(lienzo) {
        for (let p = 1; p <= 9; p++) dibujarHueco(lienzo[p - 1], p);
        dibujarMarcador(lienzo[9]);
        if (fase === 'final') {
          texto(lienzo[4], 'FIN');
        }
      },
    };

    // El agujero visto de frente y un poco desde arriba: la boca es un óvalo con su borde
    // y alrededor la tierra que ha sacado el topo, punteada para que se lea como tierra y
    // no como otra raya. `parte` deja pintar solo el labio de atrás o el de delante, que es
    // lo que hace falta para que el topo quede entre los dos.
    function borde(fb, parte) {
      for (let j = -4; j <= 4; j++) {
        if ((parte === 'atras' && j > 0) || (parte === 'delante' && j <= 0)) continue;
        for (let i = -20; i <= 20; i++) {
          const fuera = (i * i) / 420 + (j * j) / 18 <= 1;
          const dentro = (i * i) / 289 + (j * j) / 6.5 <= 1;
          if (fuera && !dentro) px(fb, 36 + i, BORDE + j, 1);
        }
      }
    }

    function tierra(fb) {
      for (let x = 1; x < 72; x += 3) if (x < 14 || x > 58) px(fb, x, BORDE + 1, 1);
      for (let x = 3; x < 70; x += 4) px(fb, x, BORDE + 7, 1);
      for (let x = 5; x < 68; x += 6) px(fb, x, BORDE + 9, 1);
    }

    // El topo, de perfil y mirando a la derecha, con la cabeza en (34, y). De frente, con
    // orejas, parecía un osito, y sin ellas una foca: lo que dice «topo» a este tamaño es
    // el perfil, con el hocico largo en punta y la nariz al final, un ojillo de nada, los
    // bigotes y la manaza de cavar con sus garras.
    function cuerpo(fb, y, mareado) {
      elipse(fb, 32, y, 11, 9, 1);
      rect(fb, 21, y, 23, 20, 1);                    // el cuerpo, recto hacia abajo
      // El hocico: se estrecha hacia la punta y cae un poco, como el de verdad.
      for (let x = 42; x <= 53; x++) {
        const k = (x - 42) / 11;
        const media = Math.round(5 - 3.2 * k);
        const centro = Math.round(y + 1 + 2 * k);
        rect(fb, x, centro - media, 1, 2 * media + 1, 1);
      }
      elipse(fb, 54, y + 3, 2, 2, 1);                // la nariz, en la punta
      if (mareado) {
        linea(fb, 36, y - 5, 39, y - 2, 0);
        linea(fb, 36, y - 2, 39, y - 5, 0);
      } else {
        rect(fb, 37, y - 4, 2, 2, 0);                // el ojillo
      }
      // Los bigotes, saliendo del hocico.
      linea(fb, 49, y, 56, y - 5, 1);
      linea(fb, 50, y + 1, 59, y - 2, 1);
    }

    // Las manos apoyadas en el borde: la de delante, ancha y con tres garras hacia
    // delante; la de atrás asoma menos.
    function patas(fb) {
      elipse(fb, 46, BORDE - 2, 5, 3, 1);
      for (const dy of [-3, -1, 1]) linea(fb, 50, BORDE - 2 + dy, 54, BORDE - 4 + dy, 1);
      elipse(fb, 22, BORDE - 1, 4, 2, 1);
    }

    function dibujarHueco(fb, p) {
      tierra(fb);
      borde(fb, 'atras');
      let y = null;
      let mareado = false;
      if (golpe && golpe.pantalla === p) {
        y = ARRIBA;
        mareado = true;
      } else if (topo && topo.pantalla === p) {
        const t = Math.min(1, (tiempo - topo.desde) / SUBIDA_MS);
        y = BORDE + 12 - Math.round((BORDE + 12 - ARRIBA) * t);   // sube desde dentro
      }
      if (y !== null) {
        cuerpo(fb, y, mareado);
        rect(fb, 16, BORDE, 41, 5, 0);               // lo que queda dentro del agujero no se ve
        rect(fb, 0, BORDE + 1, 72, 40, 0);
        tierra(fb);
        if (y <= ARRIBA + 3) patas(fb);
      }
      borde(fb, 'delante');
      if (mareado) {
        destello(fb, 8, 8, 5);
        destello(fb, 63, 7, 4);
        destello(fb, 60, 21, 3);
        destello(fb, 11, 22, 3);
      }
      if (erroneo && erroneo.pantalla === p) {
        // Un aspa sobre el agujero vacío: ahí no había nada que pulsar.
        for (let g = -1; g <= 1; g++) {
          linea(fb, 26 + g, 3, 46 + g, 23, 1);
          linea(fb, 46 + g, 3, 26 + g, 23, 1);
        }
      }
    }

    function dibujarMarcador(fb) {
      texto(fb, Math.max(0, aciertos - fallos));
      if (fase === 'jugando') {
        // La barra de tiempo, por debajo del número.
        const resto = Math.max(0, 1 - tiempo / DURACION_MS);
        rect(fb, 4, 37, Math.round(64 * resto), 2, 1);
      }
    }
  },
};
