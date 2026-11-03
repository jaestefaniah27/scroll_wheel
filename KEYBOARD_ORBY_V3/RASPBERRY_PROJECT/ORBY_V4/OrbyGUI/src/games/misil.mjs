// Misil: vista de satélite sobre las nueve teclas, un objetivo en el suelo y un misil que
// ya viene cayendo. La mira se mueve con los dos mandos (el izquierdo en horizontal, el
// derecho en vertical) y el círculo que se cierra sobre ella es el misil: cuando llega al
// centro, explota donde esté la mira. Si el objetivo estaba debajo, puntos; si no, fallo.
// Tres fallos y se acaba. Un objetivo fallado sigue ahí: el siguiente misil va a por él.
//
// Fases 1 y 2 de un juego más grande (la 3 es el VTOL, con el terreno pasando y defensas
// que disparan): lo que se decide aquí —mira, sprites, puntuación por dificultad— es lo
// que reutilizará la otra.
//
// Fase 2: el suelo es un territorio grande (misil-mapa.mjs) y las pantallas ven una zona de
// él: el puesto del desierto, el puente sobre el río o la base con la pista. Hay tiendas,
// soldados, depósitos de combustible, camiones y tanques aparcados, y otros vehículos que
// circulan por las carreteras, siempre de cara a donde avanzan (el dibujo se gira en cada
// curva), nunca de lado ni marcha atrás. Un misil se lleva por delante todo lo que quede
// dentro del círculo; lo destruido deja un cráter y vuelve a aparecer pasado un rato. Fallo
// es que no haya nada dentro.
//
// Fase 3: el VTOL. Las pantallas son lo que ve el artillero desde el aparato, que se queda un
// rato sobre cada zona (ESTANCIA_MS) y luego vuela a la siguiente en unos segundos, con
// arranque y frenada suaves (recorrido). La mira va con el aparato: el suelo se desliza por
// debajo. La pantalla 10 enseña, con una barra, cuánto queda para irse.
//
// Decisiones:
//   · las nueve pantallas son un solo suelo (Malla) con huecos cortos entre ellas. Con el
//     hueco «real» de los otros juegos la mira tardaba demasiadas vueltas de mando en
//     cruzarlo; aquí el hueco solo tiene que hacer que la mira desaparezca un instante
//   · el objetivo nunca cae en un hueco ni cerca del borde de su pantalla (MARGEN): pegado
//     al marco no se distinguía bien dónde estaba, y un objetivo que no se ve no es un reto
//   · el misil tarda siempre lo mismo en caer. Acortarlo con cada acierto dejaba tan poco
//     tiempo para apuntar que dejaba de ser divertido; la dificultad la suben los objetivos
//     que se mueven, no las prisas
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
//   · un objetivo que se mueve vale más, en proporción a su velocidad. La tienda nunca se
//     mueve: es el objetivo fácil que sigue saliendo cuando lo demás ya corre
//   · la pantalla 10 es el marcador: puntos y, encima, los misiles que quedan por fallar

import { Malla, W, H, px, rect, destello, etiquetaValor, texto } from './dibujo.mjs';
import { MAPA_W, NODOS, ADYACENTES, TRAMOS, ESTATICOS, ZONAS, construirMapa } from './misil-mapa.mjs';

export const FILAS = [[1, 2, 3], [4, 5, 6], [7, 8, 9]];
export const HUECO_X = 16;
export const HUECO_Y = 14;

const VIDAS = 3;
const CAIDA_MS = 7000;
const MARGEN = 8;                 // distancia mínima del objetivo al borde de su pantalla
// Cuánto corre el misil con los mandos pulsados: ninguno, uno, los dos.
const PRISA = [1, 1.8, 3];
export const PASO = 4;                   // píxeles de mira por paso del mando
export const PASO_RAPIDO = 9;            // si se gira deprisa: cruzar el suelo no puede costar 60 pasos
export const RAPIDO_MS = 45;             // dos pasos más juntos que esto son girar deprisa
// El positivo del mando, en cada eje. Al jugar se sentía mejor al revés del sentido de
// fábrica; se invierte solo aquí, sin tocar lo que cada perfil hace con los mandos.
export const SENTIDO_X = -1;
export const SENTIDO_Y = -1;

const RADIO_INICIAL = 34;         // el círculo del misil al empezar a caer
// Lo que alcanza la explosión. El círculo de la mira se cierra hasta este radio, así que lo
// que se ve dentro al final es justo lo que se lleva por delante.
export const RADIO_EXPLOSION = 7;
const RADIO_FINAL = RADIO_EXPLOSION;
const HUECO_CRUZ = 3;             // la cruz no se dibuja a menos de esto del centro
const MARCA_CADA = 8;             // las rayitas de la cruz
const EXPLOSION_MS = 650;
const AVISO_MS = 1200;
const FINAL_MS = 3500;

// El mapa. Los vehículos que circulan ganan velocidad con los aciertos, hasta un tope.
const VELOCIDAD_POR_ACIERTO = 0.06;   // +6 % de velocidad por acierto, sobre la del tipo
const VELOCIDAD_MAX = 2.5;
// Lo que vale de más moverse: a BONO_VELOCIDAD px/s, el doble.
const BONO_VELOCIDAD = 20;
const VALOR_MAX = 60;             // un soldado, tan pequeño, valdría una barbaridad
// El VTOL: cuánto se queda sobre cada zona y cuánto tarda en ir a la siguiente. La estancia da
// para tres o cuatro misiles sin prisas, más si se acelera la caída; el vuelo, para que se vea
// el terreno pasar sin que sea una espera.
export const ESTANCIA_MS = 24000;
export const TRANSICION_MS = 3500;
const CRATER_MS = 6000;
const REAPARECE_MS = 14000;       // lo destruido vuelve a su sitio (o a una carretera)
export const MUNDO_W = 3 * W + 2 * HUECO_X;
export const MUNDO_H = 3 * H + 2 * HUECO_Y;

// Los objetivos, vistos desde arriba. Lo que vale cada uno sale de su tamaño, la caja que
// ocupa: es lo que hay que meter en el círculo. Contar los píxeles pintados no vale: la
// tienda, hueca, salía más pequeña que el camión y se ve mucho más grande. Sin redondear,
// para que dos objetivos de tamaño parecido no empaten.
const PUNTOS_POR_AREA = 2000;
export function sprite(filas) {
  const ancho = Math.max(...filas.map((f) => f.length));
  const valor = Math.min(VALOR_MAX, Math.round(PUNTOS_POR_AREA / (ancho * filas.length)));
  return { w: ancho, h: filas.length, valor, filas: filas.map((f) => f.padEnd(ancho, '.')) };
}

// El mismo dibujo mirando hacia `dir` ('der', 'izq', 'arr', 'aba'). Los dibujos están de
// cara a la derecha; para ir en vertical se trasponen (el cañón del tanque o la cabina del
// camión acaban abajo) y se voltean para ir hacia arriba.
export function orientar(s, dir) {
  if (dir === 'der') return s;
  if (dir === 'izq') return { ...s, filas: s.filas.map((f) => [...f].reverse().join('')) };
  const filas = [];
  for (let i = 0; i < s.w; i++) {
    let f = '';
    for (let j = 0; j < s.h; j++) f += s.filas[j][i];
    filas.push(f);
  }
  if (dir === 'arr') filas.reverse();
  return { ...s, w: s.h, h: s.w, filas };
}

const DIRECCIONES = { der: [1, 0], izq: [-1, 0], aba: [0, 1], arr: [0, -1] };

export const OBJETIVOS = [
  {
    nombre: 'tienda', velocidad: 0,
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
    // Cabina delante (a la derecha, como todos los dibujos) y caja larga detrás: lo que lo distingue del tanque es que es fino.
    nombre: 'camion', velocidad: 22,
    ...sprite([
      '###########...',
      '##############',
      '##########.###',
      '##############',
      '###########...',
    ]),
  },
  {
    // Las orugas a rayas y el cañón que sobresale: sin cañón se confunde con el camión.
    nombre: 'tanque', velocidad: 12,
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
    // Un tanque de combustible redondo, visto desde arriba: el anillo de la tapa lo distingue
    // de la tienda, que es cuadrada.
    nombre: 'deposito', velocidad: 0,
    ...sprite([
      '..#####..',
      '.#######.',
      '###...###',
      '##.###.##',
      '##.#.#.##',
      '##.###.##',
      '###...###',
      '.#######.',
      '..#####..',
    ]),
  },
  {
    nombre: 'soldado', velocidad: 7,
    ...sprite([
      '.#.',
      '###',
      '.#.',
      '#.#',
    ]),
  },
];

export function pintarSprite(malla, s, x, y) {
  for (let j = 0; j < s.h; j++) {
    for (let i = 0; i < s.w; i++) if (s.filas[j][i] === '#') malla.px(x + i, y + j);
  }
}

export function anillo(malla, cx, cy, r) {
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
export function cruz(malla, cx, cy) {
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

// Dónde está la cámara a los `t` ms de vuelo: quieta en una zona, o de camino a la siguiente
// (en bucle). El paso de una a otra sigue medio coseno: arranca y frena sin tirones, que a un
// píxel por paso un arranque en seco se nota como un salto. Aparte para probarlo sin jugar.
export function recorrido(t) {
  const ciclo = ESTANCIA_MS + TRANSICION_MS;
  const n = Math.floor(t / ciclo);
  const r = t - n * ciclo;
  const zona = n % ZONAS.length;
  const a = ZONAS[zona];
  if (r < ESTANCIA_MS) return { x: a.x, y: a.y, zona, enVuelo: false, queda: (ESTANCIA_MS - r) / ESTANCIA_MS };
  const b = ZONAS[(zona + 1) % ZONAS.length];
  const k = (1 - Math.cos(Math.PI * ((r - ESTANCIA_MS) / TRANSICION_MS))) / 2;
  return { x: Math.round(a.x + (b.x - a.x) * k), y: Math.round(a.y + (b.y - a.y) * k), zona, enVuelo: true, queda: 0 };
}

const TIPO = Object.fromEntries(OBJETIVOS.map((o) => [o.nombre, o]));

// Hacia dónde va un tramo de (x0, y0) a (x1, y1): siempre uno de los cuatro, porque las
// carreteras son rectas y paralelas a los ejes.
function direccion([x0, y0], [x1, y1]) {
  if (x1 !== x0) return x1 > x0 ? 'der' : 'izq';
  return y1 > y0 ? 'aba' : 'arr';
}

// Un vehículo que circula: va de `desde` a `hasta` (nodos de la red) y `dist` es lo que
// lleva recorrido. En el cruce elige tramo al azar, salvo volver por donde vino (solo si
// no hay otro: marcha atrás nunca, el vehículo da la vuelta como una calle sin salida).
function posicionar(v) {
  const a = NODOS[v.desde];
  const b = NODOS[v.hasta];
  v.dir = direccion(a, b);
  v.forma = orientar(v.tipo, v.dir);
  const [dx, dy] = DIRECCIONES[v.dir];
  v.x = a[0] + dx * v.dist - v.forma.w / 2;
  v.y = a[1] + dy * v.dist - v.forma.h / 2;
}

function avanzarVehiculo(v, px, aleatorio) {
  v.dist += px;
  for (;;) {
    const [a, b] = [NODOS[v.desde], NODOS[v.hasta]];
    const largo = Math.abs(b[0] - a[0]) + Math.abs(b[1] - a[1]);
    if (v.dist < largo) break;
    v.dist -= largo;
    const opciones = ADYACENTES[v.hasta].filter((n) => n !== v.desde);
    const siguiente = opciones.length ? opciones[aleatorio(opciones.length)] : v.desde;
    v.desde = v.hasta;
    v.hasta = siguiente;
  }
  posicionar(v);
}

export default {
  id: 'misil',
  nombre: 'Misil',
  descripcion: 'Mueve la mira con los dos mandos (izquierdo: horizontal, derecho: vertical) y pon objetivos dentro del círculo antes de que se cierre y caiga el misil: tiendas, soldados, depósitos, camiones y tanques. Cuanto más pequeño el objetivo, más puntos, y los vehículos en marcha valen más. Pulsa un mando para que caiga antes, o los dos para que caiga aún más rápido: vale más puntos. Tres fallos y se acaba.',
  icono: '#MISIL',
  unidad: 'puntos',
  mejorSiMenor: false,

  crear({ aleatorio }) {
    let malla = null;
    let tiempo = 0;
    let mira = null;              // { x, y } en el suelo; se centra al empezar
    // Todo lo que hay en el mapa: { tipo, forma, x, y, dir, vel, plantilla } con (x, y) la
    // esquina de arriba a la izquierda en el suelo y `vel` en px/s. Los vehículos que
    // circulan llevan además { desde, hasta, dist } (ver posicionar).
    let entidades = [];
    let crateres = [];            // { x, y, hasta }
    let pendientes = [];          // lo destruido que volverá: { plantilla, en }
    let mapa = null;              // el dibujo fijo del territorio, ver construirMapa
    // La esquina de arriba a la izquierda de lo que se ve, en el territorio. La mira, el
    // impacto y la malla van en coordenadas de la vista; las entidades y los cráteres, del
    // territorio.
    const camara = { x: ZONAS[0].x, y: ZONAS[0].y };
    let vuelo = 0;                // ms de recorrido del VTOL (se para con la partida)
    let ruta = recorrido(0);
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

    const factorVelocidad = () => Math.min(VELOCIDAD_MAX, 1 + aciertos * VELOCIDAD_POR_ACIERTO);

    function crearEstatica(plantilla) {
      const [nombre, x, y, dir = 'der'] = plantilla.estatica;
      const tipo = TIPO[nombre];
      return { tipo, forma: orientar(tipo, dir), x, y, dir, vel: 0, plantilla };
    }

    function crearVehiculo(plantilla) {
      const tipo = TIPO[plantilla.vehiculo];
      const [p, q] = TRAMOS[aleatorio(TRAMOS.length)];
      const alReves = aleatorio(2) === 1;
      const v = { tipo, vel: 0, plantilla, desde: alReves ? q : p, hasta: alReves ? p : q, dist: 0 };
      const [a, b] = [NODOS[v.desde], NODOS[v.hasta]];
      v.dist = aleatorio(Math.abs(b[0] - a[0]) + Math.abs(b[1] - a[1]));
      posicionar(v);
      return v;
    }

    const crearEntidad = (pl) => (pl.estatica ? crearEstatica(pl) : crearVehiculo(pl));

    function poblar() {
      const plantillas = [
        ...ESTATICOS.map((estatica) => ({ estatica })),
        { vehiculo: 'camion' }, { vehiculo: 'camion' }, { vehiculo: 'camion' },
        { vehiculo: 'tanque' }, { vehiculo: 'tanque' },
      ];
      entidades = plantillas.map(crearEntidad);
      mapa = construirMapa(entidades.filter((e) => !e.desde).map((e) => ({ x: e.x, y: e.y, w: e.forma.w, h: e.forma.h })));
    }

    function soltarMisil() {
      progreso = 0;
      empezado = tiempo;
      fase = 'cayendo';
    }

    // ¿Toca la explosión el dibujo? El punto de la caja más cercano al impacto está dentro
    // del radio.
    function alcanza(e) {
      const x = Math.round(e.x) - camara.x;
      const y = Math.round(e.y) - camara.y;
      const cx = Math.max(x, Math.min(impacto.x, x + e.forma.w - 1));
      const cy = Math.max(y, Math.min(impacto.y, y + e.forma.h - 1));
      return Math.hypot(cx - impacto.x, cy - impacto.y) <= RADIO_EXPLOSION;
    }

    function estallar() {
      impacto = { x: Math.round(mira.x), y: Math.round(mira.y) };
      const tocados = entidades.filter(alcanza);
      if (tocados.length) {
        // Lo que se adelantó pulsando: un misil que cayó en un tercio del tiempo vale el triple.
        const prisa = Math.max(1, CAIDA_MS / Math.max(1, tiempo - empezado));
        let ganados = 0;
        for (const e of tocados) {
          ganados += Math.round(e.tipo.valor * prisa * (1 + e.vel / BONO_VELOCIDAD));
          crateres.push({ x: Math.round(e.x + e.forma.w / 2), y: Math.round(e.y + e.forma.h / 2), hasta: tiempo + CRATER_MS });
          pendientes.push({ plantilla: e.plantilla, en: tiempo + REAPARECE_MS });
        }
        entidades = entidades.filter((e) => !tocados.includes(e));
        puntos += ganados;
        aciertos += tocados.length;
        aviso = { texto: `+${ganados}`, hasta: tiempo + AVISO_MS };
      } else {
        vidas--;
        aviso = { texto: 'FALLO', hasta: tiempo + AVISO_MS };
      }
      fase = 'explosion';
      hasta = tiempo + EXPLOSION_MS;
    }

    // El mundo sigue en marcha mientras cae el misil y durante la explosión.
    function moverMundo(dt) {
      for (const e of entidades) {
        if (!e.desde) continue;
        e.vel = e.tipo.velocidad * factorVelocidad();
        avanzarVehiculo(e, (e.vel * dt) / 1000, aleatorio);
      }
      crateres = crateres.filter((c) => c.hasta > tiempo);
      const listos = pendientes.filter((p) => p.en <= tiempo);
      if (listos.length) {
        pendientes = pendientes.filter((p) => p.en > tiempo);
        for (const p of listos) entidades.push(crearEntidad(p.plantilla));
      }
    }

    function mover(mando, delta) {
      if (fase === 'final' || !mira || !delta) return;
      const rapido = tiempo - ultimoPaso[mando] < RAPIDO_MS;
      ultimoPaso[mando] = tiempo;
      const paso = delta * (rapido ? PASO_RAPIDO : PASO);
      if (mando === 1) mira.x = Math.max(0, Math.min(malla.ancho - 1, mira.x + SENTIDO_X * paso));
      else mira.y = Math.max(0, Math.min(malla.alto - 1, mira.y + SENTIDO_Y * paso));
    }

    // Copia a cada pantalla su trozo del territorio, saltándose los huecos entre pantallas.
    function pintarMapa(lienzo) {
      FILAS.forEach((fila, j) => fila.forEach((p, i) => {
        const fb = lienzo[p - 1];
        const ox = camara.x + i * (W + HUECO_X);
        const oy = camara.y + j * (H + HUECO_Y);
        for (let y = 0; y < H; y++) {
          const base = (oy + y) * MAPA_W + ox;
          for (let x = 0; x < W; x++) if (mapa[base + x]) px(fb, x, y);
        }
      }));
    }

    // Un vehículo o una tienda se pintan sobre una caja en blanco un píxel más grande:
    // sin ese halo se pierden en el terreno.
    function pintarEntidad(e) {
      const x = Math.round(e.x) - camara.x;
      const y = Math.round(e.y) - camara.y;
      malla.rect(x - 1, y - 1, e.forma.w + 2, e.forma.h + 2, 0);
      pintarSprite(malla, e.forma, x, y);
    }

    return {
      get terminado() { return fase === 'final' && tiempo - hasta >= FINAL_MS; },
      get puntos() { return puntos; },
      // Solo lectura, para las pruebas: lo que hay ahora en el mapa.
      get entidades() { return entidades; },
      get camara() { return { ...camara }; },
      get zona() { return ruta.enVuelo ? null : ZONAS[ruta.zona].nombre; },
      get estado() { return `${aciertos} objetivos · ${vidas} misiles de margen`; },

      tick(dt) {
        tiempo += dt;
        if (aviso && tiempo >= aviso.hasta) aviso = null;
        if (!mira) return;
        if (fase !== 'final') {
          vuelo += dt;
          ruta = recorrido(vuelo);
          camara.x = ruta.x;
          camara.y = ruta.y;
          moverMundo(dt);
        }
        if (fase === 'cayendo') {
          progreso += (dt / CAIDA_MS) * PRISA[Math.min(pulsados.size, 2)];
          if (progreso >= 1) estallar();
        } else if (fase === 'explosion' && tiempo >= hasta) {
          if (vidas <= 0) { fase = 'final'; hasta = tiempo; } else soltarMisil();
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
          poblar();
          soltarMisil();
        }

        if (fase === 'final') {
          texto(lienzo[3], 'FIN');
          texto(lienzo[4], puntos);
          texto(lienzo[5], 'PTS');
          etiquetaValor(lienzo[9], 'PUNTOS', puntos);
          return;
        }

        pintarMapa(lienzo);
        for (const c of crateres) {
          const x = c.x - camara.x, y = c.y - camara.y;
          malla.rect(x - 4, y - 4, 9, 9, 0);
          anillo(malla, x, y, 4); malla.px(x, y);
        }
        for (const e of entidades) pintarEntidad(e);

        const cx = Math.round(mira.x);
        const cy = Math.round(mira.y);

        if (fase === 'cayendo') {
          cruz(malla, cx, cy);
          const r = Math.round(RADIO_INICIAL + (RADIO_FINAL - RADIO_INICIAL) * Math.min(1, progreso));
          anillo(malla, cx, cy, r);
        } else {
          // La explosión: un fogonazo macizo al principio y luego una onda que se abre.
          const t = 1 - (hasta - tiempo) / EXPLOSION_MS;
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
        // Van en las filas 1-4: los puntos empiezan en la 6 y una fila en blanco los separa.
        for (let i = 0; i < vidas; i++) rect(lienzo[9], 2 + i * 6, 1, 4, 4, 1);
        // Lo que queda sobre esta zona: una barra que se vacía. En vuelo, el marco parpadea.
        const enMarcha = ruta.enVuelo && Math.floor(tiempo / 250) % 2 === 0;
        if (!ruta.enVuelo || enMarcha) {
          rect(lienzo[9], 24, 1, 46, 1, 1); rect(lienzo[9], 24, 4, 46, 1, 1);
          rect(lienzo[9], 24, 1, 1, 4, 1); rect(lienzo[9], 69, 1, 1, 4, 1);
        }
        rect(lienzo[9], 25, 2, Math.round(44 * ruta.queda), 2, 1);
        if (fase === 'explosion' && aviso && aviso.texto !== 'FALLO') destello(lienzo[9], W - 8, H - 7, 4);
      },
    };
  },
};
