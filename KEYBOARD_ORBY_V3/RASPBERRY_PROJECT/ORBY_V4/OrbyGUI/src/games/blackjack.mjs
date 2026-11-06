// Blackjack contra la banca, sin tope de manos. Las fichas se conservan de una partida a
// otra (el PC guarda el saldo de cada jugador) y el récord es el máximo de fichas que se
// llegó a tener, aunque luego se pierdan. Si te quedas sin fichas se te dan 100 más.
//
//   [1][2][3]   las cartas de la banca (la segunda, boca abajo hasta su turno)
//   [4][5][6]   las tuyas
//   [7][8][9]   PEDIR · PLANTO · DOBLAR      (al apostar: -  APOSTAR  +)
//      [10]     fichas, o el total de tu mano mientras juegas
//
// Reglas: zapato de seis barajas, la banca se planta en 17 (también blando), el blackjack
// paga 3:2, doblar solo con las dos primeras cartas y sin dividir ni seguro.
//
// No acaba solo: se sale con la tecla 10. Cada cambio de saldo se guarda al momento (por
// `progreso`) para que salir, o que se caiga la conexión, no devuelva las fichas apostadas.

import { rect, px, pastilla, texto, etiquetaValor, invertir } from './dibujo.mjs';

const FICHAS_INICIALES = 100;   // al empezar y cada vez que te quedas sin fichas
const PASO_APUESTA = 10;       // múltiplos de 10: así el 3:2 siempre da fichas enteras
const BARAJAS = 6;
const BARAJAR_BAJO = 52;       // se rehace el zapato si quedan menos cartas que esto

const REPARTO_MS = 650;        // entre carta y carta al repartir
const BANCA_MS = 1200;         // entre carta y carta de la banca (también lo que tarda en destapar la suya)
const RESULTADO_MS = 5000;     // lo que se ve el resultado antes de la siguiente mano
const RESULTADO_ARMA_MS = 1500; // antes de esto, una tecla no salta el resultado: da tiempo a ver por qué
const ABRIR_MS = 220;          // una carta nueva se abre desde el centro
const VOLTEAR_MS = 450;        // la carta tapada de la banca se da la vuelta al destaparse
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

// El dorso de una carta de ancho `w` en x: marco blanco y cuadros dentro.
function dorso(fb, x, w) {
  pastilla(fb, x, CARTA_Y, w, CARTA_ALTO, 3, 1);
  if (w < 9) return;
  pastilla(fb, x + 2, CARTA_Y + 2, w - 4, CARTA_ALTO - 4, 2, 0);
  for (let y = CARTA_Y + 4; y < CARTA_Y + CARTA_ALTO - 4; y++) {
    for (let i = x + 4; i < x + w - 4; i++) if ((i + y) % 4 === 0) px(fb, i, y, 1);
  }
}

// Una carta en (x, CARTA_Y) de ancho `w`: cara blanca con el índice y el palo en negro, como
// una carta de verdad. Boca abajo, el dorso a cuadros.
//
// `anim` es una animación en curso: { tipo: 'abrir' | 'voltear', p: 0..1 }. Abrir hace crecer
// la carta desde el centro (lo mismo boca arriba que boca abajo). Voltear la estrecha hasta
// una línea con el dorso, y la vuelve a ensanchar ya con la cara: el contenido de la cara
// entra al final, cuando la carta casi está entera, para no tener que reescalar el dibujo.
function carta(fb, c, x, w, oculta, anim = null) {
  if (anim) {
    const ancho = anim.tipo === 'voltear' ? Math.abs(1 - 2 * anim.p) : anim.p;
    const ww = Math.max(2, Math.round(w * ancho));
    const xx = x + Math.floor((w - ww) / 2);
    const mostrarDorso = anim.tipo === 'voltear' ? anim.p < 0.5 : oculta;
    if (mostrarDorso) dorso(fb, xx, ww);
    else pastilla(fb, xx, CARTA_Y, ww, CARTA_ALTO, ww < 8 ? 1 : 3, 1);
    // Con la carta casi entera el índice y el palo ya caben.
    if (mostrarDorso || ancho <= 0.85) return;
  } else if (oculta) {
    dorso(fb, x, w);
    return;
  } else {
    pastilla(fb, x, CARTA_Y, w, CARTA_ALTO, 3, 1);
  }
  const nombre = NOMBRE(c.r);
  let lx = x + 3;
  for (const ch of nombre) { letra(fb, ch, lx, CARTA_Y + 3, 2, 0); lx += 8; }
  palo(fb, c.s, x + Math.floor((w - 14) / 2), CARTA_Y + 18, 2, 0);
}

// Las cartas de una mano en sus tres pantallas: dos por pantalla, y tres si no caben.
function mano(lienzo, pantallas, cartas, visibles, ocultaSegunda, anima = () => null) {
  const n = Math.min(cartas.length, visibles);
  const porPantalla = n > 6 ? 3 : 2;
  const w = porPantalla === 2 ? 28 : 22;
  const xs = porPantalla === 2 ? [6, 38] : [1, 25, 49];
  for (let i = 0; i < n && i < porPantalla * 3; i++) {
    const fb = lienzo[pantallas[Math.floor(i / porPantalla)] - 1];
    carta(fb, cartas[i], xs[i % porPantalla], w, ocultaSegunda && i === 1, anima(i));
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
  descripcion: 'Blackjack contra la banca sin límite de manos. Tus fichas se guardan de una partida a otra y, si te quedas sin ellas, recibes 100 más. El récord es el máximo de fichas que has llegado a tener. Apuesta con la rueda o las teclas 7 y 9 y reparte con la 8; luego pide (7), plántate (8) o dobla (9). Acércate a 21 sin pasarte.',
  icono: '#BLACKJ',
  unidad: 'fichas',
  mejorSiMenor: false,
  persistente: true,   // guarda el saldo (ver gestor.js)
  sinFin: true,        // no termina solo: se sale con la tecla 10

  // `saldo`: las fichas guardadas del jugador; `progreso({ saldo, pico })` avisa de cada cambio.
  crear({ aleatorio, saldo = null, progreso = null }) {
    let tiempo = 0;
    let zapato = nuevoZapato(aleatorio);
    let fichas = Number.isFinite(saldo) && saldo >= PASO_APUESTA ? Math.floor(saldo) : FICHAS_INICIALES;
    let pico = fichas;            // el máximo de fichas alcanzado en esta partida (el récord)
    let recargado = false;        // se acaba de dar la recarga de 100
    let apuesta = PASO_APUESTA;   // la que se elige; se mantiene de una mano a otra
    let enJuego = 0;              // lo apostado en la mano en curso (doble si se dobló)
    let manoN = 0;                // manos jugadas (terminadas)
    // 'apuesta' | 'reparto' | 'jugador' | 'banca' | 'resultado'
    let fase = 'apuesta';
    let desde = 0;
    let jugador = [];
    let banca = [];
    let visJ = 0;                 // cartas ya enseñadas de cada mano (el reparto va de una en una)
    let visB = 0;
    let ocultaB = true;
    let tJ = [];                  // cuándo se enseñó cada carta (para abrirla con animación)
    let tB = [];
    let tVoltear = null;          // cuándo se destapó la carta oculta de la banca
    let siguienteCarta = 0;       // cuándo se enseña la siguiente carta pendiente
    let resultado = null;         // { resultado, devuelve, apostado }
    let pulsada = null;           // { pantalla, hasta }
    let giro = 0;                 // giro de rueda acumulado sin convertir en pasos

    const sacar = () => {
      if (!zapato.length) zapato = nuevoZapato(aleatorio);
      return zapato.pop();
    };
    const animando = () => visJ < jugador.length || visB < banca.length;

    // Enseña de golpe las cartas pendientes del jugador (al pasarse o doblar, que no esperan).
    function verTodoJugador() {
      for (let i = visJ; i < jugador.length; i++) tJ[i] = tiempo;
      visJ = jugador.length;
    }

    // Destapa la carta oculta de la banca, y se da la vuelta.
    function destapar() {
      if (!ocultaB) return;
      ocultaB = false;
      tVoltear = tiempo;
    }

    // La animación de una carta en este instante, o null si ya está entera.
    function animar(desdeT, tipo) {
      if (desdeT == null) return null;
      const p = (tiempo - desdeT) / (tipo === 'voltear' ? VOLTEAR_MS : ABRIR_MS);
      return p >= 1 ? null : { tipo, p: Math.max(0, p) };
    }

    function cambiarApuesta(pasos) {
      if (fase !== 'apuesta') return;
      apuesta = Math.max(PASO_APUESTA, Math.min(fichas - (fichas % PASO_APUESTA), apuesta + pasos * PASO_APUESTA));
    }

    // Cada cambio de saldo se guarda al momento: lo apostado ya no es tuyo aunque se salga a
    // media mano, y un récord no se pierde porque se cierre el juego sin avisar.
    const guardar = () => progreso?.({ saldo: fichas, pico });

    function repartir() {
      if (zapato.length < BARAJAR_BAJO) zapato = nuevoZapato(aleatorio);
      fichas -= apuesta;
      enJuego = apuesta;
      recargado = false;
      guardar();
      jugador = [sacar()];
      banca = [sacar()];
      jugador.push(sacar());
      banca.push(sacar());
      visJ = 0;
      visB = 0;
      tJ = [];
      tB = [];
      tVoltear = null;
      ocultaB = true;
      resultado = null;
      siguienteCarta = tiempo + REPARTO_MS;
      fase = 'reparto';
      desde = tiempo;
    }

    // Enseña la siguiente carta pendiente, alternando como se reparte de verdad.
    function enseñarUna() {
      if (visJ < jugador.length && (visJ <= visB || visB >= banca.length)) tJ[visJ++] = tiempo;
      else if (visB < banca.length) tB[visB++] = tiempo;
    }

    function cerrar() {
      destapar();
      const r = liquidar(jugador, banca, enJuego);
      fichas += r.devuelve;
      resultado = { ...r, apostado: enJuego };
      enJuego = 0;
      manoN++;
      // El récord se mide antes de la recarga: las 100 fichas de regalo no cuentan como ganadas.
      if (fichas > pico) pico = fichas;
      if (fichas < PASO_APUESTA) { fichas += FICHAS_INICIALES; recargado = true; }
      guardar();
      fase = 'resultado';
      desde = tiempo;
    }

    function turnoBanca() {
      destapar();
      fase = 'banca';
      desde = tiempo;
      siguienteCarta = tiempo + BANCA_MS;
    }

    function siguienteMano() {
      apuesta = Math.max(PASO_APUESTA, Math.min(apuesta, fichas - (fichas % PASO_APUESTA)));
      jugador = [];
      banca = [];
      visJ = visB = 0;
      tJ = [];
      tB = [];
      tVoltear = null;
      fase = 'apuesta';
      desde = tiempo;
    }

    return {
      get terminado() { return false; },
      get puntos() { return pico; },
      get manos() { return manoN; },
      get estado() {
        return `Mano ${manoN + 1} · ${fichas + enJuego} fichas · máximo ${pico}`;
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
            if (v > 21) { verTodoJugador(); cerrar(); }
            else if (v === 21) { verTodoJugador(); turnoBanca(); }
          } else if (p === 8) {
            turnoBanca();
          } else if (p === 9 && jugador.length === 2 && fichas >= enJuego) {
            fichas -= enJuego;
            enJuego *= 2;
            guardar();
            jugador.push(sacar());
            verTodoJugador();
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
        mano(l, [1, 2, 3], banca, visB, ocultaB,
          (i) => (i === 1 && tVoltear != null ? animar(tVoltear, 'voltear') : animar(tB[i], 'abrir')));
        mano(l, [4, 5, 6], jugador, visJ, false, (i) => animar(tJ[i], 'abrir'));

        if (fase === 'apuesta') {
          if (recargado) etiquetaValor(l[4], 'REGALO', `+${FICHAS_INICIALES}`);
          else etiquetaValor(l[4], 'MAXIMO', pico);
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
        }

        if (pulsada && fase !== 'resultado') invertir(l[pulsada.pantalla - 1]);
      },
    };
  },
};
