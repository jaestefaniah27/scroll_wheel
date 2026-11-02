// Misil: vista de satélite sobre las nueve teclas, un objetivo en el suelo y un misil que
// ya viene cayendo. La mira se mueve con los dos mandos (el izquierdo en horizontal, el
// derecho en vertical) y el círculo que se cierra sobre ella es el misil: cuando llega al
// centro, explota donde esté la mira. Si el objetivo estaba debajo, puntos; si no, fallo.
// Tres fallos y se acaba. Cada objetivo hace caer el siguiente misil algo más deprisa.
//
// Es la fase 1 de un juego más grande (objetivos que se mueven, luego el VTOL con el
// terreno pasando y defensas que disparan): lo que se decide aquí —mira, sprites,
// puntuación por dificultad— es lo que reutilizarán las otras dos.
//
// Decisiones:
//   · las nueve pantallas son un solo suelo (Malla) con huecos cortos entre ellas. Con el
//     hueco «real» de los otros juegos la mira tardaba demasiadas vueltas de mando en
//     cruzarlo; aquí el hueco solo tiene que hacer que la mira desaparezca un instante
//   · el objetivo nunca cae en un hueco ni pegado al borde de su pantalla: un objetivo que
//     no se ve no es un reto, es un fallo seguro
//   · la cruz se corta alrededor del centro, como la de un visor: si llegara hasta el
//     centro taparía justo lo que hay que ver
//   · el misil revienta un círculo, no un píxel: basta con que el objetivo quede dentro del
//     círculo cerrado (RADIO_EXPLOSION). Pedir el píxel exacto era más frustrante que difícil
//   · los puntos salen solo del tamaño del objetivo: cuanto más pequeño, más vale, porque es
//     lo que cuesta meterlo en el círculo
//   · pulsar los mandos acelera la caída (uno, algo; los dos, mucho). Acelerar se paga en
//     riesgo y se cobra en puntos: el premio se multiplica por lo que se ha adelantado
//   · el misil cae desde el principio, sin esperar a que se toque nada: sin teclado, la
//     partida acaba sola por fallos
//   · la pantalla 10 es el marcador: puntos y, encima, los misiles que quedan por fallar

import { Malla, W, H, rect, destello, etiquetaValor, texto } from './dibujo.mjs';

const FILAS = [[1, 2, 3], [4, 5, 6], [7, 8, 9]];
const HUECO_X = 16;
const HUECO_Y = 14;

const VIDAS = 3;
const CAIDA_INICIAL_MS = 7000;    // el primero da tiempo a entender qué pasa
const CAIDA_MINIMA_MS = 1800;
const ACELERA = 0.9;              // cada objetivo, el siguiente tarda esto por lo anterior
// Cuánto corre el misil con los mandos pulsados: ninguno, uno, los dos.
const PRISA = [1, 1.8, 3];
const PASO = 4;                   // píxeles de mira por paso del mando
const PASO_RAPIDO = 9;            // si se gira deprisa: cruzar el suelo no puede costar 60 pasos
const RAPIDO_MS = 45;             // dos pasos más juntos que esto son girar deprisa
// El positivo del mando, en cada eje. Se deja aquí para darle la vuelta sin buscar.
const SENTIDO_X = 1;
const SENTIDO_Y = 1;

const RADIO_INICIAL = 34;         // el círculo del misil al empezar a caer
// Lo que alcanza la explosión. El círculo de la mira se cierra hasta este radio, así que lo
// que se ve dentro al final es justo lo que se lleva por delante.
const RADIO_EXPLOSION = 7;
const RADIO_FINAL = RADIO_EXPLOSION;
const HUECO_CRUZ = 3;             // la cruz no se dibuja a menos de esto del centro
const MARCA_CADA = 8;             // las rayitas de la cruz
const EXPLOSION_MS = 650;
const AVISO_MS = 1200;
const FINAL_MS = 3500;
const DISTANCIA_MINIMA = 60;      // el objetivo nuevo no sale ya debajo de la mira

// Los objetivos, vistos desde arriba. Lo que vale cada uno sale de su tamaño, la caja que
// ocupa: es lo que hay que meter en el círculo. Contar los píxeles pintados no vale: la
// tienda, hueca, salía más pequeña que el camión y se ve mucho más grande. Sin redondear,
// para que dos objetivos de tamaño parecido no empaten.
const PUNTOS_POR_AREA = 2000;
function sprite(filas) {
  const ancho = Math.max(...filas.map((f) => f.length));
  const valor = Math.round(PUNTOS_POR_AREA / (ancho * filas.length));
  return { w: ancho, h: filas.length, valor, filas: filas.map((f) => f.padEnd(ancho, '.')) };
}

export const OBJETIVOS = [
  {
    nombre: 'tienda',
    ...sprite([
      '###########',
      '##.......##',
      '#.#.....#.#',
      '#..#...#..#',
      '#...#.#...#',
      '#....#....#',
      '#...#.#...#',
      '#..#...#..#',
      '#.#.....#.#',
      '##.......##',
      '###########',
    ]),
  },
  {
    // Cabina delante y caja larga detrás: lo que lo distingue del tanque es que es fino.
    nombre: 'camion',
    ...sprite([
      '...###########',
      '##############',
      '###.##########',
      '##############',
      '...###########',
    ]),
  },
  {
    // Las orugas a rayas y el cañón que sobresale: sin cañón se confunde con el camión.
    nombre: 'tanque',
    ...sprite([
      '###########',
      '#.#.#.#.#.#',
      '###########',
      '...#####',
      '...#...#########',
      '...#####',
      '###########',
      '#.#.#.#.#.#',
      '###########',
    ]),
  },
  {
    nombre: 'soldado',
    ...sprite([
      '.#.',
      '###',
      '.#.',
      '#.#',
    ]),
  },
];

function pintarSprite(malla, s, x, y) {
  for (let j = 0; j < s.h; j++) {
    for (let i = 0; i < s.w; i++) if (s.filas[j][i] === '#') malla.px(x + i, y + j);
  }
}

function anillo(malla, cx, cy, r) {
  const dentro = (r - 1) * (r - 1);
  for (let j = -r; j <= r; j++) {
    for (let i = -r; i <= r; i++) {
      const d = i * i + j * j;
      if (d <= r * r + r * 0.5 && d >= dentro) malla.px(cx + i, cy + j);
    }
  }
}

// La cruz del visor: dos líneas que cruzan todo el suelo, con rayitas cada pocos píxeles
// y cortadas alrededor del centro.
function cruz(malla, cx, cy) {
  for (let x = 0; x < malla.ancho; x++) {
    const d = x - cx;
    if (Math.abs(d) < HUECO_CRUZ) continue;
    malla.px(x, cy);
    if (d % MARCA_CADA === 0) { malla.px(x, cy - 1); malla.px(x, cy + 1); }
  }
  for (let y = 0; y < malla.alto; y++) {
    const d = y - cy;
    if (Math.abs(d) < HUECO_CRUZ) continue;
    malla.px(cx, y);
    if (d % MARCA_CADA === 0) { malla.px(cx - 1, y); malla.px(cx + 1, y); }
  }
  malla.px(cx, cy);
}

export default {
  id: 'misil',
  nombre: 'Misil',
  descripcion: 'Mueve la mira con los dos mandos (izquierdo: horizontal, derecho: vertical) y pon el objetivo dentro del círculo antes de que se cierre y caiga el misil. Cuanto más pequeño el objetivo, más puntos. Pulsa un mando para que caiga antes, o los dos para que caiga aún más rápido: vale más puntos. Tres fallos y se acaba.',
  icono: '#MISIL',
  unidad: 'puntos',
  mejorSiMenor: false,

  crear({ aleatorio }) {
    let malla = null;
    let tiempo = 0;
    let mira = null;              // { x, y } en el suelo; se centra al empezar
    let objetivo = null;          // { tipo, x, y }: esquina superior izquierda en el suelo
    let caida = CAIDA_INICIAL_MS; // lo que tarda este misil sin prisa
    let progreso = 0;             // 0 al soltarse, 1 al llegar
    let empezado = 0;             // cuándo se soltó este misil
    let fase = 'cayendo';         // 'cayendo' | 'explosion' | 'final'
    let hasta = 0;
    let impacto = null;           // { x, y }: dónde estalló el último
    let aviso = null;             // { texto, hasta }
    let vidas = VIDAS;
    let puntos = 0;
    let aciertos = 0;
    const pulsados = new Set();
    const ultimoPaso = { 1: -Infinity, 2: -Infinity };

    function nuevoObjetivo() {
      const tipo = OBJETIVOS[aleatorio(OBJETIVOS.length)];
      // En una pantalla al azar, entero dentro de ella y con un margen al borde.
      for (let intento = 0; ; intento++) {
        const p = aleatorio(9);
        const col = p % 3;
        const fila = Math.floor(p / 3);
        const x = col * (W + HUECO_X) + 3 + aleatorio(W - tipo.w - 6);
        const y = fila * (H + HUECO_Y) + 3 + aleatorio(H - tipo.h - 6);
        const lejos = Math.hypot(x + tipo.w / 2 - mira.x, y + tipo.h / 2 - mira.y) >= DISTANCIA_MINIMA;
        if (lejos || intento > 20) { objetivo = { tipo, x, y }; break; }
      }
      progreso = 0;
      empezado = tiempo;
      fase = 'cayendo';
    }

    function estallar() {
      impacto = { x: Math.round(mira.x), y: Math.round(mira.y) };
      const { tipo, x, y } = objetivo;
      // Acierta si la explosión toca el dibujo: el punto de la caja más cercano al impacto
      // está dentro del radio.
      const px = Math.max(x, Math.min(impacto.x, x + tipo.w - 1));
      const py = Math.max(y, Math.min(impacto.y, y + tipo.h - 1));
      if (Math.hypot(px - impacto.x, py - impacto.y) <= RADIO_EXPLOSION) {
        // Lo que se adelantó pulsando: un misil que cayó en un tercio del tiempo vale el triple.
        const prisa = Math.max(1, caida / Math.max(1, tiempo - empezado));
        const ganados = Math.round(tipo.valor * prisa);
        puntos += ganados;
        aciertos++;
        aviso = { texto: `+${ganados}`, hasta: tiempo + AVISO_MS };
        caida = Math.max(CAIDA_MINIMA_MS, caida * ACELERA);
      } else {
        vidas--;
        aviso = { texto: 'FALLO', hasta: tiempo + AVISO_MS };
      }
      fase = 'explosion';
      hasta = tiempo + EXPLOSION_MS;
    }

    function mover(mando, delta) {
      if (fase === 'final' || !mira || !delta) return;
      const rapido = tiempo - ultimoPaso[mando] < RAPIDO_MS;
      ultimoPaso[mando] = tiempo;
      const paso = delta * (rapido ? PASO_RAPIDO : PASO);
      if (mando === 1) mira.x = Math.max(0, Math.min(malla.ancho - 1, mira.x + SENTIDO_X * paso));
      else mira.y = Math.max(0, Math.min(malla.alto - 1, mira.y + SENTIDO_Y * paso));
    }

    return {
      get terminado() { return fase === 'final' && tiempo - hasta >= FINAL_MS; },
      get puntos() { return puntos; },
      get estado() { return `${aciertos} objetivos · ${vidas} misiles de margen`; },

      tick(dt) {
        tiempo += dt;
        if (aviso && tiempo >= aviso.hasta) aviso = null;
        if (!objetivo) return;
        if (fase === 'cayendo') {
          progreso += (dt / caida) * PRISA[Math.min(pulsados.size, 2)];
          if (progreso >= 1) estallar();
        } else if (fase === 'explosion' && tiempo >= hasta) {
          if (vidas <= 0) { fase = 'final'; hasta = tiempo; } else nuevoObjetivo();
        }
      },

      mando(ev) { mover(ev.mando, ev.delta); },

      mandoBoton(ev) {
        if (ev.pulsado) pulsados.add(ev.mando);
        else pulsados.delete(ev.mando);
      },

      dibujar(lienzo) {
        malla ??= new Malla(lienzo, FILAS, HUECO_X, HUECO_Y);
        if (!mira) {
          mira = { x: Math.round(malla.ancho / 2), y: Math.round(malla.alto / 2) };
          nuevoObjetivo();
        }

        if (fase === 'final') {
          texto(lienzo[3], 'FIN');
          texto(lienzo[4], puntos);
          texto(lienzo[5], 'PTS');
          etiquetaValor(lienzo[9], 'PUNTOS', puntos);
          return;
        }

        const cx = Math.round(mira.x);
        const cy = Math.round(mira.y);

        if (fase === 'cayendo') {
          pintarSprite(malla, objetivo.tipo, objetivo.x, objetivo.y);
          cruz(malla, cx, cy);
          const r = Math.round(RADIO_INICIAL + (RADIO_FINAL - RADIO_INICIAL) * Math.min(1, progreso));
          anillo(malla, cx, cy, r);
        } else {
          // La explosión: un fogonazo macizo al principio y luego una onda que se abre.
          // El objetivo acertado ya no se pinta; el fallado sigue ahí, para que se vea por cuánto.
          const t = 1 - (hasta - tiempo) / EXPLOSION_MS;
          const acertado = aviso && aviso.texto !== 'FALLO';
          if (!acertado) pintarSprite(malla, objetivo.tipo, objetivo.x, objetivo.y);
          if (t < 0.2) malla.disco(impacto.x, impacto.y, RADIO_EXPLOSION);
          else {
            anillo(malla, impacto.x, impacto.y, Math.round(4 + t * 16));
            if (t < 0.6) anillo(malla, impacto.x, impacto.y, Math.round(2 + t * 8));
          }
        }

        // El marcador: el aviso del último misil un momento, los puntos el resto.
        if (aviso) texto(lienzo[9], aviso.texto);
        else etiquetaValor(lienzo[9], 'PUNTOS', puntos);
        // Los misiles de margen, como cuadraditos en la esquina: se cuentan sin leer.
        for (let i = 0; i < vidas; i++) rect(lienzo[9], 2 + i * 6, 2, 4, 4, 1);
        if (fase === 'explosion' && aviso && aviso.texto !== 'FALLO') destello(lienzo[9], W - 8, 6, 4);
      },
    };
  },
};
