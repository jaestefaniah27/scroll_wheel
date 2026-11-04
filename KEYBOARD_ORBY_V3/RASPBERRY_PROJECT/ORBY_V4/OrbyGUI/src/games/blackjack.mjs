// Blackjack contra la banca: diez manos y se empieza con 100 fichas. Los puntos son las
// fichas con las que se acaba.
//
//   [1][2][3]   las cartas de la banca (la segunda, boca abajo hasta su turno)
//   [4][5][6]   las tuyas
//   [7][8][9]   PEDIR · PLANTO · DOBLAR      (al apostar: -  APOSTAR  +)
//      [10]     fichas, o el total de tu mano mientras juegas
//
// Reglas: zapato de seis barajas, la banca se planta en 17 (también blando), el blackjack
// paga 3:2, doblar solo con las dos primeras cartas y sin dividir ni seguro.

import { rect, px, pastilla, texto, etiquetaValor, invertir } from './dibujo.mjs';

const FICHAS_INICIALES = 100;
const MANOS = 10;
const PASO_APUESTA = 10;       // múltiplos de 10: así el 3:2 siempre da fichas enteras
const BARAJAS = 6;
const BARAJAR_BAJO = 52;       // se rehace el zapato si quedan menos cartas que esto

const REPARTO_MS = 280;        // entre carta y carta al repartir
const BANCA_MS = 650;          // entre carta y carta de la banca, para que se lea
const RESULTADO_MS = 2200;     // lo que se ve el resultado antes de la siguiente mano
const RESULTADO_ARMA_MS = 600; // antes de esto, una tecla no salta el resultado
const PULSACION_MS = 140;
const RUEDA_PASO = 40;         // unidades de giro de la rueda por cada paso de apuesta

// --- Reglas (sin estado: los tests las usan directamente) -----------------------------

// Carta: { r: 1-13 (1 = as), s: 0-3 (picas, corazones, diamantes, tréboles) }.
export function valorMano(cartas) {
  let total = 0;
  let ases = 0;
  for (const c of cartas) {
    if (c.r === 1) { ases++; total += 11; } else total += Math.min(c.r, 10);
  }
  while (total > 21 && ases > 0) { total -= 10; ases--; }
  return total;
}

export function esBlackjack(cartas) {
  return cartas.length === 2 && valorMano(cartas) === 21;
}

// Lo que se devuelve al jugador por una mano en la que apostó `apuesta` (ya descontada de
// sus fichas). `resultado` es para el rótulo.
export function liquidar(jugador, banca, apuesta) {
  const vj = valorMano(jugador);
  const vb = valorMano(banca);
  const bjJ = esBlackjack(jugador);
  const bjB = esBlackjack(banca);
  if (vj > 21) return { resultado: 'pasa', devuelve: 0 };
  if (bjJ && bjB) return { resultado: 'empate', devuelve: apuesta };
  if (bjJ) return { resultado: 'blackjack', devuelve: apuesta + (apuesta * 3) / 2 };
  if (bjB) return { resultado: 'pierde', devuelve: 0 };
  if (vb > 21 || vj > vb) return { resultado: 'gana', devuelve: apuesta * 2 };
  if (vj === vb) return { resultado: 'empate', devuelve: apuesta };
  return { resultado: 'pierde', devuelve: 0 };
}

export function nuevoZapato(aleatorio, barajas = BARAJAS) {
  const z = [];
  for (let b = 0; b < barajas; b++) for (let s = 0; s < 4; s++) for (let r = 1; r <= 13; r++) z.push({ r, s });
  for (let i = z.length - 1; i > 0; i--) {   // Fisher–Yates
    const j = aleatorio(i + 1);
    [z[i], z[j]] = [z[j], z[i]];
  }
  return z;
}

// --- Dibujo de las cartas --------------------------------------------------------------

// Letras de 3x5 para el índice de la carta (se pintan a doble tamaño). Cada fila, de
// izquierda a derecha, en los tres bits bajos.
const LETRAS = {
  A: [2, 5, 7, 5, 5],
  J: [1, 1, 1, 5, 2],
  Q: [2, 5, 5, 6, 3],
  K: [5, 6, 4, 6, 5],
  0: [7, 5, 5, 5, 7],
  1: [2, 6, 2, 2, 7],
  2: [6, 1, 2, 4, 7],
  3: [6, 1, 2, 1, 6],
  4: [5, 5, 7, 1, 1],
  5: [7, 4, 6, 1, 6],
  6: [3, 4, 6, 5, 2],
  7: [7, 1, 2, 2, 2],
  8: [2, 5, 2, 5, 2],
  9: [2, 5, 3, 1, 6],
};

// Palos de 7x7, en el mismo orden que `s`.
const PALOS = [
  [0x08, 0x1c, 0x3e, 0x7f, 0x7f, 0x2a, 0x1c],   // picas
  [0x36, 0x7f, 0x7f, 0x7f, 0x3e, 0x1c, 0x08],   // corazones
  [0x08, 0x1c, 0x3e, 0x7f, 0x3e, 0x1c, 0x08],   // diamantes
  [0x1c, 0x1c, 0x6b, 0x7f, 0x6b, 0x08, 0x1c],   // tréboles
];

const NOMBRE = (r) => (r === 1 ? 'A' : r === 11 ? 'J' : r === 12 ? 'Q' : r === 13 ? 'K' : String(r));

function letra(fb, ch, x, y, escala, valor) {
  const filas = LETRAS[ch];
  for (let j = 0; j < 5; j++) {
    for (let i = 0; i < 3; i++) {
      if ((filas[j] >> (2 - i)) & 1) rect(fb, x + i * escala, y + j * escala, escala, escala, valor);
    }
  }
}

function palo(fb, s, x, y, escala, valor) {
  const filas = PALOS[s];
  for (let j = 0; j < 7; j++) {
    for (let i = 0; i < 7; i++) {
      if ((filas[j] >> (6 - i)) & 1) rect(fb, x + i * escala, y + j * escala, escala, escala, valor);
    }
  }
}

const CARTA_ALTO = 36;
const CARTA_Y = 2;

// Una carta en (x, CARTA_Y) de ancho `w`: cara blanca con el índice y el palo en negro, como
// una carta de verdad. Boca abajo, el dorso a cuadros.
function carta(fb, c, x, w, oculta) {
  pastilla(fb, x, CARTA_Y, w, CARTA_ALTO, 3, 1);
  if (oculta) {
    pastilla(fb, x + 2, CARTA_Y + 2, w - 4, CARTA_ALTO - 4, 2, 0);
    for (let y = CARTA_Y + 4; y < CARTA_Y + CARTA_ALTO - 4; y++) {
      for (let i = x + 4; i < x + w - 4; i++) if ((i + y) % 4 === 0) px(fb, i, y, 1);
    }
    return;
  }
  const nombre = NOMBRE(c.r);
  let lx = x + 3;
  for (const ch of nombre) { letra(fb, ch, lx, CARTA_Y + 3, 2, 0); lx += 8; }
  palo(fb, c.s, x + Math.floor((w - 14) / 2), CARTA_Y + 18, 2, 0);
}

// Las cartas de una mano en sus tres pantallas: dos por pantalla, y tres si no caben.
function mano(lienzo, pantallas, cartas, visibles, ocultaSegunda) {
  const n = Math.min(cartas.length, visibles);
  const porPantalla = n > 6 ? 3 : 2;
  const w = porPantalla === 2 ? 28 : 22;
  const xs = porPantalla === 2 ? [6, 38] : [1, 25, 49];
  for (let i = 0; i < n && i < porPantalla * 3; i++) {
    const fb = lienzo[pantallas[Math.floor(i / porPantalla)] - 1];
    carta(fb, cartas[i], xs[i % porPantalla], w, ocultaSegunda && i === 1);
  }
}

const ROTULOS = {
  gana: 'GANAS',
  blackjack: 'BJ!',
  empate: 'EMPATE',
  pierde: 'PIERDES',
  pasa: 'TE PASAS',
};

// --- El juego ----------------------------------------------------------------------------

export default {
  id: 'blackjack',
  nombre: 'Blackjack',
  descripcion: 'Diez manos contra la banca empezando con 100 fichas. Apuesta con la rueda o las teclas 7 y 9 y reparte con la 8; luego pide (7), plántate (8) o dobla (9). Acércate a 21 sin pasarte.',
  icono: '#BLACKJ',
  unidad: 'fichas',
  mejorSiMenor: false,

  crear({ aleatorio }) {
    let tiempo = 0;
    let zapato = nuevoZapato(aleatorio);
    let fichas = FICHAS_INICIALES;
    let apuesta = PASO_APUESTA;   // la que se elige; se mantiene de una mano a otra
    let enJuego = 0;              // lo apostado en la mano en curso (doble si se dobló)
    let manoN = 0;                // manos jugadas (terminadas)
    // 'apuesta' | 'reparto' | 'jugador' | 'banca' | 'resultado' | 'final'
    let fase = 'apuesta';
    let desde = 0;
    let jugador = [];
    let banca = [];
    let visJ = 0;                 // cartas ya enseñadas de cada mano (el reparto va de una en una)
    let visB = 0;
    let ocultaB = true;
    let siguienteCarta = 0;       // cuándo se enseña la siguiente carta pendiente
    let resultado = null;         // { resultado, devuelve, apostado }
    let pulsada = null;           // { pantalla, hasta }
    let giro = 0;                 // giro de rueda acumulado sin convertir en pasos

    const sacar = () => {
      if (!zapato.length) zapato = nuevoZapato(aleatorio);
      return zapato.pop();
    };
    const animando = () => visJ < jugador.length || visB < banca.length;

    function cambiarApuesta(pasos) {
      if (fase !== 'apuesta') return;
      apuesta = Math.max(PASO_APUESTA, Math.min(fichas - (fichas % PASO_APUESTA), apuesta + pasos * PASO_APUESTA));
    }

    function repartir() {
      if (zapato.length < BARAJAR_BAJO) zapato = nuevoZapato(aleatorio);
      fichas -= apuesta;
      enJuego = apuesta;
      jugador = [sacar()];
      banca = [sacar()];
      jugador.push(sacar());
      banca.push(sacar());
      visJ = 0;
      visB = 0;
      ocultaB = true;
      resultado = null;
      siguienteCarta = tiempo + REPARTO_MS;
      fase = 'reparto';
      desde = tiempo;
    }

    // Enseña la siguiente carta pendiente, alternando como se reparte de verdad.
    function enseñarUna() {
      if (visJ < jugador.length && (visJ <= visB || visB >= banca.length)) visJ++;
      else if (visB < banca.length) visB++;
    }

    function cerrar() {
      ocultaB = false;
      const r = liquidar(jugador, banca, enJuego);
      fichas += r.devuelve;
      resultado = { ...r, apostado: enJuego };
      enJuego = 0;
      fase = 'resultado';
      desde = tiempo;
    }

    function turnoBanca() {
      ocultaB = false;
      fase = 'banca';
      desde = tiempo;
      siguienteCarta = tiempo + BANCA_MS;
    }

    function siguienteMano() {
      manoN++;
      if (manoN >= MANOS || fichas < PASO_APUESTA) {
        fase = 'final';
        desde = tiempo;
        return;
      }
      apuesta = Math.max(PASO_APUESTA, Math.min(apuesta, fichas - (fichas % PASO_APUESTA)));
      jugador = [];
      banca = [];
      visJ = visB = 0;
      fase = 'apuesta';
      desde = tiempo;
    }

    return {
      get terminado() { return fase === 'final'; },
      get puntos() { return fichas + enJuego; },
      get estado() {
        const m = Math.min(manoN + 1, MANOS);
        return `Mano ${m}/${MANOS} · ${fichas} fichas`;
      },

      tick(dt) {
        tiempo += dt;
        if (pulsada && tiempo >= pulsada.hasta) pulsada = null;

        if (animando() && tiempo >= siguienteCarta) {
          enseñarUna();
          siguienteCarta = tiempo + (fase === 'banca' ? BANCA_MS : REPARTO_MS);
        }

        if (fase === 'reparto' && !animando()) {
          if (esBlackjack(jugador) || esBlackjack(banca)) cerrar();
          else { fase = 'jugador'; desde = tiempo; }
        } else if (fase === 'banca' && !animando() && tiempo >= siguienteCarta) {
          if (valorMano(banca) < 17) {
            banca.push(sacar());
            siguienteCarta = tiempo;   // la enseña el bloque de arriba en el próximo tic
          } else cerrar();
        } else if (fase === 'resultado' && tiempo - desde >= RESULTADO_MS) {
          siguienteMano();
        }
      },

      tecla(ev) {
        if (!ev.pulsada || !ev.pantalla || ev.pantalla > 9) return;
        const p = ev.pantalla;

        if (fase === 'apuesta') {
          if (p === 7) cambiarApuesta(-1);
          else if (p === 9) cambiarApuesta(1);
          else if (p === 8) repartir();
          else return;
        } else if (fase === 'jugador' && !animando()) {
          if (p === 7) {
            jugador.push(sacar());
            siguienteCarta = tiempo;
            // Pasarse acaba la mano; llegar a 21 planta solo, no hay nada mejor que pedir.
            const v = valorMano(jugador);
            if (v > 21) { visJ = jugador.length; cerrar(); }
            else if (v === 21) { visJ = jugador.length; turnoBanca(); }
          } else if (p === 8) {
            turnoBanca();
          } else if (p === 9 && jugador.length === 2 && fichas >= enJuego) {
            fichas -= enJuego;
            enJuego *= 2;
            jugador.push(sacar());
            visJ = jugador.length;
            if (valorMano(jugador) > 21) cerrar(); else turnoBanca();
          } else return;
        } else if (fase === 'resultado' && tiempo - desde >= RESULTADO_ARMA_MS) {
          siguienteMano();
        } else return;

        pulsada = { pantalla: p, hasta: tiempo + PULSACION_MS };
      },

      rueda(ev) {
        giro += ev.delta;
        while (Math.abs(giro) >= RUEDA_PASO) {
          const paso = Math.sign(giro);
          giro -= paso * RUEDA_PASO;
          cambiarApuesta(paso);
        }
      },

      mando(ev) { cambiarApuesta(Math.sign(ev.delta)); },

      dibujar(l) {
        mano(l, [1, 2, 3], banca, visB, ocultaB);
        mano(l, [4, 5, 6], jugador, visJ, false);

        if (fase === 'apuesta') {
          etiquetaValor(l[4], 'MANO', `${manoN + 1}/${MANOS}`);
          texto(l[6], '-');
          etiquetaValor(l[7], 'APOSTAR', apuesta);
          texto(l[8], '+');
          etiquetaValor(l[9], 'FICHAS', fichas);
        } else if (fase === 'jugador') {
          texto(l[6], 'PEDIR');
          texto(l[7], 'PLANTO');
          if (jugador.length === 2 && fichas >= enJuego) texto(l[8], 'DOBLAR');
          etiquetaValor(l[9], 'TIENES', valorMano(jugador));
        } else if (fase === 'reparto') {
          etiquetaValor(l[9], 'APUESTA', enJuego);
        } else if (fase === 'banca') {
          etiquetaValor(l[9], 'BANCA', valorMano(banca.slice(0, visB)));
        } else if (fase === 'resultado' && resultado) {
          etiquetaValor(l[6], 'TU', valorMano(jugador));
          texto(l[7], ROTULOS[resultado.resultado]);
          etiquetaValor(l[8], 'BANCA', valorMano(banca));
          const neto = resultado.devuelve - resultado.apostado;
          etiquetaValor(l[9], 'FICHAS', neto > 0 ? `+${neto}` : neto);
          // El rótulo parpadea al ganar, como una máquina que paga.
          if (neto > 0 && Math.floor((tiempo - desde) / 300) % 2 === 1) invertir(l[7]);
        } else if (fase === 'final') {
          etiquetaValor(l[9], 'FICHAS', fichas);
        }

        if (pulsada && fase !== 'resultado') invertir(l[pulsada.pantalla - 1]);
      },
    };
  },
};
