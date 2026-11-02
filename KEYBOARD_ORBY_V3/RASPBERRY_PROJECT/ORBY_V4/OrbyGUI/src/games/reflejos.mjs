// Reflejos: se enciende una tecla en blanco y hay que pulsarla lo antes posible.
//
// Tres niveles de tres pulsaciones. El primero se juega con la fila del medio; el segundo
// suma la de arriba y el tercero la de abajo, así que cada nivel tiene más sitios donde
// puede salir y se nota lo que cuesta elegir entre más (la ley de Hick). Por eso el
// resultado se da por nivel, además de la media de todo: se ve dónde se es más lento.
//
// La medida es lo que importa aquí, y por eso no sale del reloj del juego, que va a
// trompicones de 4 ms y llega tarde por el USB. El firmware (4.29) dice cuándo terminó de
// pintar cada fotograma y cuándo empezó cada pulsación, los dos en µs de su propio reloj:
// la reacción es la resta, de pantalla a dedo, sin el viaje al PC. Si el firmware no trae
// esos instantes se mide con el reloj del PC al llegar cada línea (los dos viajes se
// parecen y se compensan casi del todo), y en los tests, sin motor, con el del juego.
//
// Las esperas entre una tecla y la siguiente son aleatorias en el PC: lo que tiene que ser
// preciso es la medida, no cuándo sale; con que no se pueda adivinar basta.

import { rect, texto, etiquetaValor, pastilla } from './dibujo.mjs';
import { msEntre } from './proto.mjs';

export const NIVELES = [
  { nombre: '3 teclas', pantallas: [4, 5, 6] },
  { nombre: '6 teclas', pantallas: [1, 2, 3, 4, 5, 6] },
  { nombre: '9 teclas', pantallas: [1, 2, 3, 4, 5, 6, 7, 8, 9] },
];
export const POR_NIVEL = 3;
export const CASTIGO_MS = 100;   // cada tecla equivocada suma esto a la reacción en curso

const ESPERA_MIN_MS = 900;
const ESPERA_VARIABLE_MS = 1700;
const PRESENTA_MS = 1300;        // lo que se enseñan las teclas nuevas al cambiar de nivel
const RESULTADOS_MS = 6000;
const PARPADEO_MS = 160;

export default {
  id: 'reflejos',
  nombre: 'Reflejos',
  descripcion: 'Pulsa la tecla del centro para empezar y luego cada tecla que se ponga en blanco, lo antes que puedas. Tres niveles: 3, 6 y 9 teclas posibles. Cada fallo suma 0,1 s.',
  icono: '#REFLEJO',
  unidad: 'reaccion',
  mejorSiMenor: true,
  detalle: NIVELES.map((n) => n.nombre),

  crear({ aleatorio }) {
    let tiempo = 0;
    // 'inicio' | 'presenta' | 'espera' | 'objetivo' | 'resultados'
    let fase = 'inicio';
    let desde = 0;
    let nivel = 0;
    let espera = 0;
    let castigo = 0;            // lo acumulado por fallos para la tecla en curso
    let objetivo = null;        // { pantalla, desde, frame, us, t }
    let pulsada = null;         // { pantalla, hasta }: la tecla buena, un momento en contorno
    let fallo = null;           // { pantalla, hasta }
    let preciso = true;         // todas las medidas salieron del reloj del teclado
    const tiempos = NIVELES.map(() => []);

    const media = (xs) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : null);
    const todas = () => tiempos.flat();

    function esperar() {
      fase = 'espera';
      desde = tiempo;
      espera = ESPERA_MIN_MS + aleatorio(ESPERA_VARIABLE_MS);
    }

    function lanzar() {
      const opciones = NIVELES[nivel].pantallas;
      let p;
      // Repetir la misma tecla seguida es legal, pero con tres opciones sale demasiado y
      // deja de medir la reacción: el dedo ya está ahí.
      do { p = opciones[aleatorio(opciones.length)]; } while (opciones.length > 1 && objetivo && p === objetivo.pantalla);
      objetivo = { pantalla: p, desde: tiempo, frame: null, us: null, t: null };
      fase = 'objetivo';
    }

    // La reacción de la pulsación `ev` al objetivo en curso. null si la tecla llegó antes
    // de que el objetivo estuviera en el panel (se ha adelantado: es un fallo).
    function reaccion(ev) {
      if (objetivo.us != null && ev.us != null) {
        const ms = msEntre(objetivo.us, ev.us);
        // Una resta «negativa» da la vuelta al contador y sale enorme: pulsó antes.
        return ms > 60_000 ? null : ms;
      }
      preciso = false;
      if (objetivo.t != null && ev.t != null) return ev.t >= objetivo.t ? ev.t - objetivo.t : null;
      return tiempo - objetivo.desde;
    }

    function acertar(ms) {
      tiempos[nivel].push(ms + castigo);
      castigo = 0;
      pulsada = { pantalla: objetivo.pantalla, hasta: tiempo + PARPADEO_MS };
      if (tiempos[nivel].length < POR_NIVEL) { esperar(); return; }
      nivel++;
      if (nivel >= NIVELES.length) {
        fase = 'resultados';
        desde = tiempo;
        objetivo = null;
      } else {
        fase = 'presenta';
        desde = tiempo;
      }
    }

    function fallar(pantalla) {
      castigo += CASTIGO_MS;
      fallo = { pantalla, hasta: tiempo + PARPADEO_MS * 2 };
    }

    return {
      get terminado() { return fase === 'resultados' && tiempo - desde >= RESULTADOS_MS; },
      // La media de todas, en ms enteros. Antes de acabar no hay resultado.
      get puntos() { const m = media(todas()); return m == null ? 0 : Math.round(m); },
      get detalle() {
        return { niveles: tiempos.map((xs) => (xs.length ? Math.round(media(xs)) : null)), preciso };
      },
      get estado() {
        if (fase === 'inicio') return 'Pulsa la tecla del centro para empezar';
        if (fase === 'resultados') return `Media ${Math.round(media(todas()))} ms`;
        return `Nivel ${nivel + 1} de ${NIVELES.length} · ${tiempos[nivel].length} de ${POR_NIVEL}`;
      },

      tick(dt) {
        tiempo += dt;
        if (pulsada && tiempo >= pulsada.hasta) pulsada = null;
        if (fallo && tiempo >= fallo.hasta) fallo = null;
        if (fase === 'presenta' && tiempo - desde >= PRESENTA_MS) esperar();
        else if (fase === 'espera' && tiempo - desde >= espera) lanzar();
      },

      enviado(n) {
        if (objetivo && objetivo.frame === null && fase === 'objetivo') objetivo.frame = n;
      },

      acuse(ev) {
        // El primer acuse que cubre el fotograma del objetivo: ahí se encendió.
        if (!objetivo || objetivo.frame === null || ev.n < objetivo.frame) return;
        if (objetivo.us === null && ev.us != null) objetivo.us = ev.us;
        if (objetivo.t === null) objetivo.t = ev.t ?? null;
      },

      tecla(ev) {
        if (!ev.pulsada || !ev.pantalla || ev.pantalla > 9) return;
        if (fase === 'inicio') {
          if (ev.pantalla === 5) esperar();
          return;
        }
        if (fase === 'presenta' || fase === 'resultados') return;
        // Pulsar mientras se espera también es fallar: si no, aporrear las teclas a ciegas
        // acabaría acertando con tiempos imposibles.
        if (fase === 'espera' || ev.pantalla !== objetivo.pantalla) { fallar(ev.pantalla); return; }
        const ms = reaccion(ev);
        if (ms === null) { fallar(ev.pantalla); return; }
        acertar(ms);
      },

      dibujar(l) {
        if (fase === 'resultados') {
          NIVELES.forEach((n, i) => etiquetaValor(l[i], n.nombre.toUpperCase(), `${tiempos[i].length ? Math.round(media(tiempos[i])) : '-'}`));
          etiquetaValor(l[4], 'MEDIA MS', Math.round(media(todas())));
          texto(l[9], 'FIN');
          return;
        }
        const opciones = NIVELES[Math.min(nivel, NIVELES.length - 1)].pantallas;
        const nuevas = fase === 'presenta' ? opciones.filter((p) => !NIVELES[nivel - 1].pantallas.includes(p)) : [];
        for (const p of opciones) {
          const fb = l[p - 1];
          // Las teclas que se suman al nivel parpadean al presentarse.
          if (nuevas.includes(p) && Math.floor((tiempo - desde) / 220) % 2 === 1) continue;
          if (fase === 'objetivo' && objetivo.pantalla === p) rect(fb, 0, 0, 72, 40, 1);
          else if (pulsada && pulsada.pantalla === p) { rect(fb, 0, 0, 72, 40, 1); rect(fb, 4, 4, 64, 32, 0); }
          else { pastilla(fb, 20, 11, 32, 18, 6, 1); pastilla(fb, 22, 13, 28, 14, 4, 0); }
          if (fallo && fallo.pantalla === p) {
            // Un aspa: esa no era. Se dibuja en negro sobre blanco si cayó en la buena tarde.
            const v = fase === 'objetivo' && objetivo.pantalla === p ? 0 : 1;
            for (let g = -1; g <= 1; g++) {
              for (let k = 0; k <= 20; k++) {
                rect(fb, 26 + k + g, 10 + k, 1, 1, v);
                rect(fb, 46 - k + g, 10 + k, 1, 1, v);
              }
            }
          }
        }
        if (fase === 'inicio') {
          // La del centro encendida con un «play» en negro: púlsala para empezar.
          const fb = l[4];
          rect(fb, 0, 0, 72, 40, 1);
          for (let x = 28; x <= 48; x++) {
            const h = Math.round((48 - x) * 0.6);
            rect(fb, x, 20 - h, 1, 2 * h + 1, 0);
          }
        }
        if (fase === 'presenta') texto(l[9], `NIVEL ${nivel + 1}`);
        else texto(l[9], fase === 'inicio' ? 'LISTO?' : `${nivel + 1}: ${tiempos[nivel].length}/${POR_NIVEL}`);
      },
    };
  },
};
