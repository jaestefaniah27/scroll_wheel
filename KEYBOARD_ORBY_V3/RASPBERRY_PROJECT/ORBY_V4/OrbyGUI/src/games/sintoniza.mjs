// Sintoniza: como el dial de una radio antigua, gira la rueda hasta meter la aguja en la
// emisora (la zona rellena de la barra) y mantenla ahí. Cinco emisoras, cada una con una
// banda más estrecha que la anterior; gana quien tarda menos en conquistar las cinco.
//
// Se llamó «Cerradura» mientras era un prototipo, pero nada en él se parece a abrir una:
// girar un dial hasta una marca es sintonizar.
//
// Tiene que explicarse solo, en el propio teclado, porque nadie lee una descripción antes
// de jugar. Por eso:
//   · las tres teclas de arriba son una barra continua y sin huecos (Malla): la línea es el
//     cursor y el bloque relleno, la zona a la que hay que llevarlo. Sin huecos porque un
//     cursor o una zona que se esconden entre dos teclas no son un reto, son un fallo
//   · la tecla 5 es la barra de sintonía: una línea vertical fija marca el centro, y
//     mientras el cursor está dentro de la zona el vaciado crece desde esa línea hacia los
//     dos lados a la vez. Si el cursor se sale, la barra vuelve a llena: no hay margen para
//     entrar y salir sin perder el progreso
//   · la fila del medio guía con una flecha hacia el lado en que está la zona (no hace
//     falta cuando el cursor ya está dentro: ahí manda la barra de sintonía)
//   · la primera emisora escribe las instrucciones en la fila de abajo, y las quita en
//     cuanto se empieza a girar; después esas teclas enseñan la frecuencia del dial, de
//     puro adorno
//   · el tiempo no corre mientras se lee: cuenta desde el primer giro

import { Malla, punta, texto, rect } from './dibujo.mjs';

const EMISORAS = 5;
const ZONA_INICIAL = 30;          // ancho de la primera zona, en píxeles de la barra
const ZONA_PASO = 4;              // cuánto se estrecha la zona en cada emisora siguiente
const PASO_RUEDA = 1 / 22;        // píxeles de barra por unidad de giro de la rueda
// El giro positivo de la rueda es el que el firmware cuenta de fábrica, y al jugar se
// sentía al revés: la línea iba al lado contrario de la mano. Se invierte aquí, solo
// para este juego, sin tocar el sentido que cada perfil da a la rueda.
const SENTIDO = -1;
const DURACION_MS = 600;          // cuánto hay que mantener el cursor dentro para conquistar
const FINAL_MS = 3500;
const MARGEN_ZONA = 14;           // la zona nunca sale tan pegada a un borde
const MHZ_MIN = 88;
const MHZ_MAX = 108;              // banda FM, de puro adorno: no significa nada para el juego

function flecha(fb, haciaIzquierda) {
  const dir = haciaIzquierda ? -1 : 1;
  for (const x of [36 - dir * 9, 36 + dir * 9]) punta(fb, x, 20, 13, dir, 5);
}

export default {
  id: 'sintoniza',
  nombre: 'Sintoniza',
  descripcion: 'Gira la rueda hasta meter la línea de las tres teclas de arriba en la zona rellena (la flecha te guía) y mantenla dentro hasta que la barra de la tecla 5 se vacíe entera. Cinco emisoras, cada vez más estrechas: cuanto menos tardes, mejor.',
  icono: '#SINT',
  unidad: 'ms',
  mejorSiMenor: true,

  crear({ aleatorio }) {
    let malla = null;
    let tiempo = 0;
    let cursor = null;            // posición en la barra; se centra al empezar
    let zona = 0;                 // inicio de la zona
    let anchoZona = ZONA_INICIAL;
    let numero = 0;               // emisora en la que va (0-4)
    let inicio = 0;               // cuándo empezó a girar (el reloj solo cuenta desde ahí)
    let girado = false;           // ya ha tocado la rueda alguna vez
    let progreso = 0;             // ms mantenidos dentro de la zona en esta emisora
    let fase = 'jugando';         // 'jugando' | 'final'
    let hasta = 0;
    let total = 0;

    function nuevaZona(ancho) {
      anchoZona = ZONA_INICIAL - numero * ZONA_PASO;
      // Nunca sale ya con el cursor dentro: no habría nada que girar.
      do {
        zona = MARGEN_ZONA + aleatorio(ancho - anchoZona - 2 * MARGEN_ZONA);
      } while (cursor >= zona - 4 && cursor <= zona + anchoZona + 4);
      progreso = 0;
    }

    function girar(delta) {
      if (fase !== 'jugando' || !malla || !delta) return;
      if (!girado) { girado = true; inicio = tiempo; }
      cursor = Math.max(0, Math.min(malla.ancho - 1, cursor + SENTIDO * delta * PASO_RUEDA));
    }

    const dentro = () => cursor >= zona && cursor <= zona + anchoZona;

    return {
      get terminado() { return fase === 'final' && tiempo - hasta >= FINAL_MS; },
      get puntos() { return Math.round(total); },
      get estado() { return `Emisora ${Math.min(numero + 1, EMISORAS)} de ${EMISORAS}`; },

      tick(dt) {
        tiempo += dt;
        if (fase !== 'jugando' || !girado) return;
        if (dentro()) {
          progreso = Math.min(DURACION_MS, progreso + dt);
          if (progreso >= DURACION_MS) {
            numero++;
            if (numero >= EMISORAS) {
              total = tiempo - inicio;
              fase = 'final';
              hasta = tiempo;
            } else {
              nuevaZona(malla.ancho);
            }
          }
        } else {
          progreso = 0;
        }
      },

      rueda(ev) { girar(ev.delta); },
      // Los mandos valen igual que la rueda: no todo el mundo la tiene a mano.
      mando(ev) { girar(ev.delta * 12); },

      dibujar(lienzo) {
        malla ??= new Malla(lienzo, [[1, 2, 3]], 0, 0);
        if (cursor === null) {
          cursor = malla.ancho / 2;
          nuevaZona(malla.ancho);
        }

        if (fase === 'final') {
          texto(lienzo[0], 'FIN');
          texto(lienzo[1], (total / 1000).toFixed(1));
          texto(lienzo[2], 's');
          return;
        }

        // La barra es la escala del dial: una línea con marcas cada pocos píxeles
        // (más largas cada cuatro), la zona encima y la aguja del cursor.
        malla.rect(0, 19, malla.ancho, 2, 1);
        for (let x = 3; x < malla.ancho; x += 9) {
          const larga = Math.round((x - 3) / 9) % 4 === 0;
          malla.rect(x, larga ? 13 : 16, 1, larga ? 14 : 8, 1);
        }
        // Fuera, la zona es un bloque relleno; dentro se queda hueca y la aguja se ve
        // dentro de ella, así que «encajar» se nota también en la barra.
        malla.rect(zona, 8, anchoZona, 24, 1);
        if (dentro()) malla.rect(zona + 3, 11, anchoZona - 6, 18, 0);
        const c = Math.round(cursor);
        malla.rect(c - 1, 5, 3, 30, 1);
        // Puntas arriba y abajo: sobresalen de la zona y la aguja no se pierde dentro de ella.
        for (let k = 0; k < 4; k++) {
          malla.rect(c - k, 4 - k, 2 * k + 1, 1, 1);
          malla.rect(c - k, 36 + k, 2 * k + 1, 1, 1);
        }

        // La barra de sintonía (tecla 5): una línea vertical fija en el centro, y el
        // vaciado crece hacia los dos lados a la vez según se mantiene el cursor dentro.
        const barra = lienzo[4];
        const anchoBarra = 72;
        const centro = anchoBarra / 2;
        const vaciado = Math.round(centro * (progreso / DURACION_MS));
        rect(barra, 0, 8, anchoBarra, 24, 1);
        if (vaciado > 0) {
          rect(barra, centro - vaciado, 8, vaciado, 24, 0);
          rect(barra, centro, 8, vaciado, 24, 0);
        }
        rect(barra, centro - 1, 5, 3, 30, 1);

        // La fila del medio guía: solo una flecha hacia la zona cuando el cursor está
        // fuera. Dentro ya manda la barra de sintonía.
        if (!dentro()) {
          flecha(lienzo[cursor < zona ? 5 : 3], cursor >= zona);
        }
        // Las instrucciones, solo en la primera emisora y hasta que se empieza a girar;
        // después, la frecuencia del dial, de puro adorno.
        if (numero === 0 && !girado) {
          texto(lienzo[6], 'GIRA');
          texto(lienzo[7], 'LA');
          texto(lienzo[8], 'RUEDA');
        } else {
          const mhz = MHZ_MIN + (cursor / malla.ancho) * (MHZ_MAX - MHZ_MIN);
          const entero = Math.floor(mhz);
          const decimal = Math.floor((mhz - entero) * 10);
          texto(lienzo[6], String(entero));
          texto(lienzo[7], '.');
          texto(lienzo[8], String(decimal));
        }

        // En qué emisora va.
        texto(lienzo[9], `${Math.min(numero + 1, EMISORAS)}/${EMISORAS}`);
      },
    };
  },
};
