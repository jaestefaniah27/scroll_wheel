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
// él: la base con la pista, el puesto del desierto o el puente sobre el río, y más al este el
// oasis, el aeródromo y el cuartel. Hay tiendas,
// soldados, depósitos de combustible, camiones y tanques aparcados, y otros vehículos que
// circulan por las carreteras, siempre de cara a donde avanzan (el dibujo se gira en cada
// curva), nunca de lado ni marcha atrás. Un misil se lleva por delante todo lo que quede
// dentro del círculo; lo destruido deja un cráter y no vuelve: el mapa se va vaciando. Fallo
// es que no haya nada dentro. Una zona sin nada que disparar se deja atrás sin gastar misiles.
// En el helipuerto de algunas zonas puede esperar un helicóptero que despega al poco de llegar:
// si se acierta, vale muchísimo.
//
// Fase 3: el VTOL. Las pantallas son lo que ve el artillero desde el aparato, que se queda
// ESTANCIA_MS sobre cada zona y luego vuela a la siguiente, con arranque y frenada suaves
// (trayecto) y más tiempo cuanto más lejos esté. Las zonas van en ruta lineal: al acabar la
// sexta la partida termina con victoria y las vidas que quedan dan puntos (BONO_VIDA). Nunca
// se mueve con un misil en el aire: se va cuando el último ha estallado. La mira va con el
// aparato: el suelo se desliza por debajo, y además el aparato vaivenea (deriva) un poco al azar.
// La pantalla 10 enseña, con una barra, cuánto queda para irse y, en vuelo, la zona a la que va.
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
//   · las carreteras son de sentido único: con 9 px de calzada no caben dos carriles, y así
//     nunca hay dos vehículos de frente
//   · cada vehículo decide su siguiente tramo al entrar en el actual (`luego`), no al salir:
//     para mirar qué hay delante pasado el cruce hay que saber por dónde va a seguir
//   · los vehículos no se pisan: avanzan lo que les deja el de delante, medido por la carretera.
//     En las incorporaciones pasa primero el que está más cerca del cruce, sin semáforos
//   · un convoy comparte camino y velocidad (la del más lento): si el tanque frena al camión y
//     no al revés, la fila se desordena
//   · la estancia es solo de tiempo: exigir un mínimo de misiles obligaba a malgastarlos en
//     zonas vacías, y la gracia es apuntar rápido para aprovechar los 40 s
//   · el helicóptero no cuenta para saber si la zona está vacía: se va solo y no debe retener al VTOL
//   · el tráfico no se acaba: entra un convoy nuevo cerca de la vista, fuera de ella (no aparece
//     de la nada delante del jugador), y lo que queda lejos se borra para no gastar

import { Malla, W, H, px, rect, destello, etiquetaValor, texto } from './dibujo.mjs';
import { MAPA_W, MAPA_H, NODOS, SALIDAS, TRAMOS, ESTATICOS, ZONAS, SENDAS, construirMapa } from './misil-mapa.mjs';

export const FILAS = [[1, 2, 3], [4, 5, 6], [7, 8, 9]];
export const HUECO_X = 16;
export const HUECO_Y = 14;

const VIDAS = 3;
const CAIDA_MS = 7000;
const MARGEN = 8;                 // distancia mínima del objetivo al borde de su pantalla
// Cuánto corre el misil con los mandos pulsados: ninguno, uno, los dos. Con 1,8 y 3 apenas se
// notaba la diferencia: a 2,5 y 4,5 el misil cae en 2,8 s y 1,6 s en lugar de 7.
const PRISA = [1, 2.5, 4.5];
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

// El tráfico. Provisional: la fase 4 lo sustituye por la tabla NIVELES, una fila por zona.
const TRAFICO = { llegadaMs: 6000, maxVehiculos: 6, convoyMax: 3, patrulla: 0.3 };
// Cada cuántos ms sale un grupo a pie, por zona (0: ninguno). Provisional: la fase 4 lo mete en NIVELES.
const PIE_MS = [0, 20000, 15000, 12000, 10000, 8000];
const SEPARACION_PIE = 6;         // px, a lo largo de la senda, entre dos soldados de un grupo
const HOLGURA = 4;                // aire entre dos vehículos que van en fila
const CERCA = 200;                // los vehículos a menos de esto de la vista son los que cuentan
const LEJOS = 400;                // y a más de esto se borran
const ENTRADA_MIN = 20;           // un convoy nuevo no entra a menos de esto de la vista...
const ENTRADA_MAX = 80;           // ...pero sí a menos de esto
const CRUCES_DE_ANTEMANO = 4;
const INTENTOS_URGENTES = 30;
const PRONTO = 40;               // y esto es lo que se considera «a punto de verse»
const ACERCAMIENTO = 220;         // si el convoy no llega a la vista en esto de camino, no entra
const CONVOYES_AL_EMPEZAR = 2;
// Lo que vale de más moverse: a BONO_VELOCIDAD px/s, el doble.
const BONO_VELOCIDAD = 20;
const VALOR_MAX = 60;             // un soldado, tan pequeño, valdría una barbaridad
// El VTOL: cuánto se queda sobre cada zona y cuánto tarda en ir a la siguiente. El vuelo, para que
// se vea el terreno pasar sin que sea una espera.
export const ESTANCIA_MS = 40000;
export const BONO_VIDA = 200;             // por cada vida que queda al acabar la última zona
export const TRANSICION_MS = 3500;        // el vuelo más corto; los largos crecen con la distancia
const MS_POR_PX = 12;
// El helicóptero y el vaivén. Provisionales: la fase 4 mete la probabilidad y la amplitud en NIVELES.
const HELI_PROB = 0.5;
const VAIVEN_PX = [2, 3, 4, 5, 6, 7];     // por zona
const VAIVEN_TAU = 450;
const HELI_POSADO_MS = 1000;
const HELI_ACEL = 45;
const HELI_VEL = 70;
const VALOR_HELI = 150;
export const MUNDO_W = 3 * W + 2 * HUECO_X;
export const MUNDO_H = 3 * H + 2 * HUECO_Y;

// Los objetivos, vistos desde arriba. Lo que vale cada uno sale de su tamaño, la caja que
// ocupa: es lo que hay que meter en el círculo. Contar los píxeles pintados no vale: la
// tienda, hueca, salía más pequeña que el camión y se ve mucho más grande. Sin redondear,
// para que dos objetivos de tamaño parecido no empaten.
const PUNTOS_POR_AREA = 2000;
// `filas2` es el segundo fotograma de un dibujo animado (las luces de la patrulla).
export function sprite(filas, filas2) {
  const ancho = Math.max(...filas.map((f) => f.length));
  const valor = Math.min(VALOR_MAX, Math.round(PUNTOS_POR_AREA / (ancho * filas.length)));
  const rellenar = (fs) => fs.map((f) => f.padEnd(ancho, '.'));
  const s = { w: ancho, h: filas.length, valor, filas: rellenar(filas) };
  if (filas2) s.filas2 = rellenar(filas2);
  return s;
}

// El mismo dibujo mirando hacia `dir` ('der', 'izq', 'arr', 'aba'). Los dibujos están de
// cara a la derecha; para ir en vertical se trasponen (el cañón del tanque o la cabina del
// camión acaban abajo) y se voltean para ir hacia arriba.
function girarFilas(filas, w, h, dir) {
  if (dir === 'izq') return filas.map((f) => [...f].reverse().join(''));
  const out = [];
  for (let i = 0; i < w; i++) {
    let f = '';
    for (let j = 0; j < h; j++) f += filas[j][i];
    out.push(f);
  }
  if (dir === 'arr') out.reverse();
  return out;
}

export function orientar(s, dir) {
  if (dir === 'der') return s;
  const g = { ...s, filas: girarFilas(s.filas, s.w, s.h, dir) };
  if (s.filas2) g.filas2 = girarFilas(s.filas2, s.w, s.h, dir);
  if (dir === 'arr' || dir === 'aba') { g.w = s.h; g.h = s.w; }
  return g;
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
  {
    // Ala delta con el morro a la derecha: lo grande que es el ala lo separa del tanque.
    nombre: 'caza', velocidad: 0,
    ...sprite([
      '..##.......',
      '..###......',
      '...###.....',
      '#..####....',
      '##########.',
      '###########',
      '##########.',
      '#..####....',
      '...###.....',
      '..###......',
      '..##.......',
    ]),
  },
  {
    // Más corto que el camión y con la barra de luces encima: lo que más corre del mapa. Las
    // luces parpadean: el segundo fotograma cambia la barra.
    nombre: 'patrulla', velocidad: 34,
    ...sprite([
      '.######..',
      '#########',
      '##.#.####',
      '#########',
      '.######..',
    ], [
      '.######..',
      '#########',
      '###.#.###',
      '#########',
      '.######..',
    ]),
  },
  {
    // Parapeto cuadrado con los dos cañones hacia arriba: ni redondo como el depósito ni hueco
    // como la tienda. No se gira: los cañones apuntan al cielo, no a la carretera.
    nombre: 'antiaereo', velocidad: 0,
    ...sprite([
      '..#.#..',
      '..#.#..',
      '..#.#..',
      '#######',
      '#.###.#',
      '#.###.#',
      '#######',
    ]),
  },
  {
    // El soldado con el tubo al hombro: una columna más que el soldado, hacia la derecha.
    nombre: 'rpg', velocidad: 7,
    ...sprite([
      '.#..',
      '####',
      '.#..',
      '#.#.',
    ]),
  },
  {
    // Visto desde arriba, con la cola a la izquierda (el morro mira a la derecha, como todos). El rotor
    // es una cruz que en el segundo fotograma pasa a aspa: así se ve girar. Cabe en el círculo de la H.
    // Su valor es fijo: por tamaño saldría barato y es lo que más tiene que valer.
    nombre: 'helicoptero', velocidad: 0, valorFijo: true,
    ...sprite([
      '......#......',
      '......#......',
      '......#......',
      '.....###.....',
      '#...#####....',
      '#...#####....',
      '###########..',
      '#...#####....',
      '#...#####....',
      '.....###.....',
      '......#......',
      '......#......',
      '......#......',
    ], [
      '.............',
      '.#.........#.',
      '..#.......#..',
      '...#.###.#...',
      '#...#####....',
      '#...#####....',
      '#########....',
      '#...#####....',
      '#...#####....',
      '...#.###.#...',
      '..#.......#..',
      '.#.........#.',
      '.............',
    ]),
    valor: VALOR_HELI,
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

// Lo que dura el vuelo de `zona` a la siguiente: a más distancia, más tiempo, para que la velocidad
// con la que pasa el terreno no se dispare en los vuelos largos.
export function duracionVuelo(zona) {
  const a = ZONAS[zona], b = ZONAS[zona + 1];
  return Math.max(TRANSICION_MS, Math.hypot(b.x - a.x, b.y - a.y) * MS_POR_PX);
}

// Dónde está la cámara a `k` del vuelo desde la zona `zona` (0 al salir, 1 al llegar a la
// siguiente). El paso sigue medio coseno: arranca y frena sin tirones, que a un píxel por paso un
// arranque en seco se nota como un salto. Aparte para probarlo sin jugar.
export function trayecto(zona, k) {
  const a = ZONAS[zona];
  const b = ZONAS[zona + 1];
  const s = (1 - Math.cos(Math.PI * k)) / 2;
  return { x: Math.round(a.x + (b.x - a.x) * s), y: Math.round(a.y + (b.y - a.y) * s) };
}

// ¿Hay algo de `entidades` dentro de lo que ve la cámara puesta en `zona`? Para saber si
// merece la pena quedarse: sin nada que disparar, los seis misiles obligatorios serían fallos.
export function hayObjetivosEn(entidades, zona) {
  // Los que caminan no retienen al VTOL: se van o se quedan quietos en un rato.
  return entidades.some((e) => !e.pie && e.x + e.forma.w > zona.x && e.x < zona.x + MUNDO_W
    && e.y + e.forma.h > zona.y && e.y < zona.y + MUNDO_H);
}

const TIPO = Object.fromEntries(OBJETIVOS.map((o) => [o.nombre, o]));

// Qué camina en un grupo. Hoy siempre soldados; la fase 4 (4.5) hace que en la zona 6 1 de cada 4
// sea un RPG, que no dispara hasta pararse.
const tipoAPie = () => TIPO.soldado;

// El punto de la senda a `s` px de su inicio. Antes del inicio sigue la prolongación del primer
// tramo hacia atrás: así el grupo sale en fila de la puerta.
function puntoEnRuta(ruta, s) {
  if (s <= 0) {
    const [a, b] = ruta;
    const l = Math.hypot(b[0] - a[0], b[1] - a[1]);
    return [a[0] + ((b[0] - a[0]) / l) * s, a[1] + ((b[1] - a[1]) / l) * s];
  }
  let resto = s;
  for (let i = 0; i + 1 < ruta.length; i++) {
    const [a, b] = [ruta[i], ruta[i + 1]];
    const l = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (resto <= l) return [a[0] + ((b[0] - a[0]) * resto) / l, a[1] + ((b[1] - a[1]) * resto) / l];
    resto -= l;
  }
  return ruta[ruta.length - 1];
}

// Hacia dónde va un tramo de (x0, y0) a (x1, y1): siempre uno de los cuatro, porque las
// carreteras son rectas y paralelas a los ejes.
function direccion([x0, y0], [x1, y1]) {
  if (x1 !== x0) return x1 > x0 ? 'der' : 'izq';
  return y1 > y0 ? 'aba' : 'arr';
}

const largoTramo = (desde, hasta) => {
  const [a, b] = [NODOS[desde], NODOS[hasta]];
  return Math.abs(b[0] - a[0]) + Math.abs(b[1] - a[1]);
};
const restante = (v) => largoTramo(v.desde, v.hasta) - v.dist;

// Un vehículo que circula: va de `desde` a `hasta` (nodos de la red) y `dist` es lo que lleva
// recorrido. `luego` es el nodo al que irá después de `hasta`, elegido al entrar en el tramo.
function posicionar(v) {
  const a = NODOS[v.desde];
  const b = NODOS[v.hasta];
  v.dir = direccion(a, b);
  v.forma = orientar(v.tipo, v.dir);
  const [dx, dy] = DIRECCIONES[v.dir];
  v.x = a[0] + dx * v.dist - v.forma.w / 2;
  v.y = a[1] + dy * v.dist - v.forma.h / 2;
}

// Un convoy sigue el `camino` que decide el primero que llega a cada cruce; los demás lo repiten.
// Sin callejones en la red (lo comprueba un test) siempre hay salida.
function elegirLuego(v, aleatorio) {
  const c = v.convoy;
  if (c && c.camino[v.paso + 2] !== undefined) { v.luego = c.camino[v.paso + 2]; return; }
  const opciones = SALIDAS[v.hasta];
  v.luego = opciones[aleatorio(opciones.length)];
  if (c) c.camino[v.paso + 2] = v.luego;
}

export function avanzarVehiculo(v, px, aleatorio) {
  if (v.luego === undefined) elegirLuego(v, aleatorio);
  v.dist += px;
  for (;;) {
    const largo = largoTramo(v.desde, v.hasta);
    if (v.dist < largo) break;
    v.dist -= largo;
    v.desde = v.hasta;
    v.hasta = v.luego;
    if (v.convoy) v.paso++;
    elegirLuego(v, aleatorio);
  }
  posicionar(v);
}

// Lo que sobra (o falta, si es negativo) de aire entre `v` y lo que tiene delante, medido por la
// carretera entre centros. Delante es: el de su tramo que va por delante, cualquiera que salga
// del cruce al que llega, y el que llega a ese mismo cruce desde otro tramo y está más cerca
// de él (empate: el creado antes), que es como se resuelven las incorporaciones.
function holgura(v, vehiculos) {
  const rv = restante(v);
  let min = Infinity;
  for (const o of vehiculos) {
    if (o === v) continue;
    // En línea recta basta con el largo de los dos. Si uno gira respecto al otro, las cajas
    // pueden rozarse en la esquina con los centros más lejos: hace falta sumar también los anchos.
    const recto = (v.tipo.w + o.tipo.w) / 2 + HOLGURA;
    const esquina = (v.tipo.w + o.tipo.w + v.tipo.h + o.tipo.h) / 2 + 1;
    let distancia, hueco;
    if (o.desde === v.desde && o.hasta === v.hasta) {
      if (o.dist < v.dist || (o.dist === v.dist && vehiculos.indexOf(o) > vehiculos.indexOf(v))) continue;
      distancia = o.dist - v.dist; hueco = recto;
    } else if (o.desde === v.hasta) {
      distancia = rv + o.dist; hueco = o.dir === v.dir ? recto : esquina;
    } else if (o.hasta === v.hasta) {
      const ro = restante(o);
      if (ro > rv || (ro === rv && vehiculos.indexOf(o) > vehiculos.indexOf(v))) continue;
      distancia = rv - ro; hueco = esquina;
    } else continue;
    min = Math.min(min, distancia - hueco);
  }
  return min;
}

// Los px que puede avanzar `v` sin acercarse a otro más de lo permitido.
export const espacioDelante = (v, vehiculos) => Math.max(0, holgura(v, vehiculos));

// Mueve a todos `ms`, en el orden en que vienen (siempre el mismo). Cada convoy va a la
// velocidad de su vehículo más lento.
export function circular(vehiculos, ms, aleatorio) {
  const lento = new Map();
  for (const v of vehiculos) if (v.convoy) lento.set(v.convoy, Math.min(lento.get(v.convoy) ?? Infinity, v.tipo.velocidad));
  for (const v of vehiculos) {
    v.vel = v.convoy ? lento.get(v.convoy) : v.tipo.velocidad;
    avanzarVehiculo(v, Math.min((v.vel * ms) / 1000, espacioDelante(v, vehiculos)), aleatorio);
  }
}

// Un convoy en fila por el tramo `tramo` ('pq'): la cabeza a la fracción `k` (0-1) de lo que cabe
// y los demás detrás. Es null si no cabe entero en el tramo.
export function armarConvoy(tipos, tramo, k, aleatorio) {
  const [p, q] = tramo;
  const sitio = [0];
  for (let i = 1; i < tipos.length; i++) sitio.push(sitio[i - 1] + (tipos[i - 1].w + tipos[i].w) / 2 + HOLGURA + 2);
  const cola = sitio[sitio.length - 1] + tipos[tipos.length - 1].w / 2;
  const cabeza = tipos[0].w / 2;
  const largo = largoTramo(p, q);
  if (largo - cabeza < cola) return null;
  const base = cola + k * (largo - cabeza - cola);
  // El camino se sortea de antemano unos cruces: así quien lo coloca puede saber si pasará por
  // la vista. Más allá, lo decide el primero que llegue (elegirLuego).
  const camino = [p, q];
  for (let i = 0; i < CRUCES_DE_ANTEMANO; i++) { const salidas = SALIDAS[camino[camino.length - 1]]; camino.push(salidas[aleatorio(salidas.length)]); }
  const convoy = { camino };
  return tipos.map((tipo, i) => {
    const v = { tipo, vel: 0, desde: p, hasta: q, luego: convoy.camino[2], dist: base - sitio[i], convoy, paso: 0 };
    posicionar(v);
    return v;
  });
}

// Lo que dista la caja de `e` del rectángulo que ve la cámara (0 si se solapan).
function distanciaAVista(e, cam) {
  const dx = Math.max(cam.x - (e.x + e.forma.w), e.x - (cam.x + MUNDO_W), 0);
  const dy = Math.max(cam.y - (e.y + e.forma.h), e.y - (cam.y + MUNDO_H), 0);
  return Math.hypot(dx, dy);
}

export default {
  id: 'misil',
  nombre: 'Misil',
  descripcion: 'Mueve la mira con los dos mandos (izquierdo: horizontal, derecho: vertical) y pon objetivos dentro del círculo antes de que se cierre y caiga el misil: tiendas, soldados, depósitos, camiones y tanques. Cuanto más pequeño el objetivo, más puntos, y los vehículos en marcha valen más. Pulsa un mando para que caiga antes, o los dos para que caiga aún más rápido: vale más puntos. Lo destruido no vuelve. Tres fallos y se acaba.',
  icono: '#MISIL',
  unidad: 'puntos',
  mejorSiMenor: false,
  // Lo que se guarda además de los puntos (ver scores.mjs): objetivos destruidos de cada tipo,
  // y el máximo con un solo misil. La vista lo enseña con estas etiquetas.
  estadisticas: {
    suma: [
      ['total', 'Objetivos destruidos'], ['tienda', 'Tiendas'], ['soldado', 'Soldados'],
      ['deposito', 'Depósitos'], ['camion', 'Camiones'], ['tanque', 'Tanques'], ['patrulla', 'Patrullas'],
      ['helicoptero', 'Helicópteros'],
    ],
    max: [['aLaVez', 'Más objetivos con un solo misil'], ['zona', 'Zona más lejana']],
  },

  // Solo para pruebas: `inmortal` (los fallos no gastan vidas, para medir un minuto de tráfico),
  // `quieto` (sin vaivén, para apuntar a un objetivo concreto), `zonaInicial` (empezar en otra zona)
  // y `pieMs` (el ritmo de los grupos a pie).
  crear({ aleatorio, inmortal = false, quieto = false, zonaInicial = 0, pieMs }) {
    let malla = null;
    let tiempo = 0;
    let mira = null;              // { x, y } en el suelo; se centra al empezar
    // Todo lo que hay en el mapa: { tipo, forma, x, y, dir, vel } con (x, y) la
    // esquina de arriba a la izquierda en el suelo y `vel` en px/s. Los vehículos que
    // circulan llevan además { desde, hasta, dist } (ver posicionar).
    let entidades = [];
    let crateres = [];            // { x, y }: lo destruido no vuelve, el cráter se queda
    let mapa = null;              // el dibujo fijo del territorio, ver construirMapa
    // La esquina de arriba a la izquierda de lo que se ve, en el territorio. La mira, el
    // impacto y la malla van en coordenadas de la vista; las entidades y los cráteres, del
    // territorio.
    const camara = { x: ZONAS[zonaInicial].x, y: ZONAS[zonaInicial].y };
    let zonaIdx = zonaInicial;    // la zona donde está el VTOL
    let estancia = 0;             // ms parado sobre esta zona (no cuenta mientras vuela)
    let ganada = false;
    // El vaivén: la cámara persigue a `objetivo` (relativo a la zona) con un suavizado, y cada tanto
    // se sortea un objetivo nuevo.
    const deriva = { x: 0, y: 0 };
    const objetivo = { x: 0, y: 0 };
    let proximoVaiven = 0;
    let vuelo = null;             // ms de vuelo hacia la siguiente zona; null si está parado
    let progreso = 0;             // 0 al soltarse, 1 al llegar
    let empezado = 0;             // cuándo se soltó este misil
    let fase = 'cayendo';         // 'cayendo' | 'explosion' | 'final'
    let hasta = 0;
    let impacto = null;           // { x, y }: dónde estalló el último
    let aviso = null;             // { texto, hasta }
    let vidas = VIDAS;
    let puntos = 0;
    let aciertos = 0;
    const muertos = {};           // objetivos destruidos por tipo
    let masALaVez = 0;            // el máximo alcanzado por un solo misil
    const pulsados = new Set();
    const ultimoPaso = { 1: -Infinity, 2: -Infinity };

    let llegada = TRAFICO.llegadaMs;   // ms que faltan para el próximo convoy

    function crearEstatica([nombre, x, y, dir = 'der']) {
      const tipo = TIPO[nombre];
      return { tipo, forma: orientar(tipo, dir), x, y, dir, vel: 0 };
    }

    const vehiculos = () => entidades.filter((e) => e.desde);

    // Grupos a pie (fase 5). `pie` en la entidad lleva { ruta, s, tope, alFinal }: `s` es lo andado por
    // la senda (negativo mientras está en fila detrás de la puerta) y `tope` dónde se para.
    const pieMsZona = () => pieMs ?? PIE_MS[zonaIdx];
    const sortearPie = () => { const ms = pieMsZona(); return ms ? ms / 2 + aleatorio(ms) : Infinity; };
    let proximoPie = sortearPie();

    // 5.1: sale un grupo por la puerta de un edificio (o del estático de una senda) de esta zona. Las
    // sendas desde aparcamientos y las de subir a un vehículo son de 5.2 y 5.3.
    function lanzarPie() {
      const posibles = SENDAS.filter((d) => d.zona === ZONAS[zonaIdx].nombre && d.de !== 'aparcamiento' && d.fin !== 'subir');
      if (!posibles.length) return;
      const { puntos: ruta, fin } = posibles[aleatorio(posibles.length)];
      let total = 0;
      for (let i = 1; i < ruta.length; i++) total += Math.hypot(ruta[i][0] - ruta[i - 1][0], ruta[i][1] - ruta[i - 1][1]);
      const tipo = tipoAPie();
      for (let k = 0, n = 2 + aleatorio(3); k < n; k++) {
        // Los que se quedan lo hacen en fila contra el final, no todos en el mismo píxel.
        const pie = { ruta, s: -k * SEPARACION_PIE, tope: fin === 'quedarse' ? total - k * SEPARACION_PIE : total, alFinal: fin };
        const e = { tipo, forma: tipo, x: 0, y: 0, dir: 'der', vel: tipo.velocidad, pie };
        colocarPie(e);
        entidades.push(e);
      }
    }

    function colocarPie(e) {
      const [x, y] = puntoEnRuta(e.pie.ruta, e.pie.s);
      e.x = x - e.forma.w / 2;
      e.y = y - e.forma.h / 2;
    }

    function andar(dt) {
      for (const e of entidades) {
        if (!e.pie) continue;
        e.pie.s = Math.min(e.pie.tope, e.pie.s + (e.vel * dt) / 1000);
        colocarPie(e);
        if (e.pie.s < e.pie.tope) continue;
        // Entrar es escaparse: desaparece sin dar puntos. Quedarse lo vuelve un estático más.
        if (e.pie.alFinal === 'entrar') e.pie = null;
        else { delete e.pie; e.vel = 0; }
      }
      entidades = entidades.filter((e) => e.pie !== null);
    }

    // Intenta meter un convoy en un tramo y un punto al azar; `vale(miembros)` dice si el sitio sirve.
    // Una sola tentativa: si no sirve o no cabe, se prueba otra vez en el siguiente tick. Las
    // patrullas van solas, nunca en convoy: son las rápidas.
    function meterConvoy(vale) {
      const sola = aleatorio(1000) / 1000 < TRAFICO.patrulla;
      const tipos = sola ? [TIPO.patrulla]
        : Array.from({ length: 1 + aleatorio(TRAFICO.convoyMax) }, () => TIPO[aleatorio(2) ? 'camion' : 'tanque']);
      const miembros = armarConvoy(tipos, TRAMOS[aleatorio(TRAMOS.length)], aleatorio(1000) / 1000, aleatorio);
      if (!miembros || !vale(miembros)) return false;
      const todos = [...vehiculos(), ...miembros];
      if (!todos.every((v) => holgura(v, todos) >= 0)) return false;
      entidades.push(...miembros);
      return true;
    }

    // Al llegar a una zona con helipuerto, a veces espera un helicóptero posado en la H.
    function ponerHelicoptero() {
      const hp = ZONAS[zonaIdx].helipuerto;
      if (!hp || aleatorio(1000) / 1000 >= HELI_PROB) return;
      const tipo = TIPO.helicoptero;
      entidades.push({ tipo, forma: tipo, x: hp[0] - (tipo.w >> 1), y: hp[1] - (tipo.h >> 1), dir: 'der', vel: 0, vx: 0, vy: 0, volador: true, despegaEn: tiempo + HELI_POSADO_MS });
    }

    // Despega hacia un rumbo al azar, acelerando, de morro; y se borra al salir de la vista.
    function moverHelicopteros(dt) {
      for (const e of entidades) {
        if (!e.volador || tiempo < e.despegaEn) continue;
        if (!e.rumbo) {
          const a = (aleatorio(360) * Math.PI) / 180;
          e.rumbo = [Math.cos(a), Math.sin(a)];
          e.dir = Math.abs(e.rumbo[0]) > Math.abs(e.rumbo[1]) ? (e.rumbo[0] > 0 ? 'der' : 'izq') : (e.rumbo[1] > 0 ? 'aba' : 'arr');
          e.forma = orientar(e.tipo, e.dir);
        }
        e.vel = Math.min(HELI_VEL, e.vel + (HELI_ACEL * dt) / 1000);
        e.vx = e.rumbo[0] * e.vel;
        e.vy = e.rumbo[1] * e.vel;
        e.x += (e.vx * dt) / 1000;
        e.y += (e.vy * dt) / 1000;
      }
      entidades = entidades.filter((e) => !(e.volador && e.vel > 0 && distanciaAVista(e, camara) > 0));
    }

    function poblar() {
      entidades = ESTATICOS.map(crearEstatica);
      mapa = construirMapa(entidades.map((e) => ({ x: e.x, y: e.y, w: e.forma.w, h: e.forma.h })));
      // Con el tráfico llegando de fuera, la primera zona empezaría vacía unos segundos.
      for (let i = 0; i < CONVOYES_AL_EMPEZAR; i++) {
        for (let intento = 0; intento < 200 && !meterConvoy((m) => m.some((e) => distanciaAVista(e, camara) === 0)); intento++);
      }
    }

    // ¿Está `v` en la vista o llegará a ella por su camino en `alcance` px? Se sondea con una copia.
    function llegaALaVista(v, alcance = ACERCAMIENTO) {
      const sonda = { ...v, convoy: { camino: [...v.convoy.camino] } };
      for (let d = 0; ; d += 10) {
        if (distanciaAVista(sonda, camara) === 0) return true;
        if (d >= alcance) return false;
        avanzarVehiculo(sonda, 10, () => 0);
      }
    }

    // Entra un convoy cada `llegadaMs` si hay sitio, y lo que queda lejos de la vista se borra. Si
    // no hay ni uno en la vista ni de camino, ni el temporizador ni el tope esperan: los circuitos
    // son cortos y los vehículos lentos dan vueltas cerca durante minutos, así que con el tope
    // lleno la vista se quedaba vacía. El que entra tiene que llegar a la vista por su camino:
    // uno que entra alejándose no se verá en un buen rato y ocupa un hueco de los que cuentan.
    function trafico(dt) {
      entidades = entidades.filter((e) => !(e.desde || e.volador) || distanciaAVista(e, camara) <= LEJOS);
      llegada -= dt;
      const hay = vehiculos();
      const vacia = !hay.some((e) => llegaALaVista(e, PRONTO));
      if (!vacia && (llegada > 0 || hay.filter((e) => distanciaAVista(e, camara) < CERCA).length >= TRAFICO.maxVehiculos)) return;
      const entra = (m) => m.every((e) => distanciaAVista(e, camara) > ENTRADA_MIN)
        && m.some((e) => distanciaAVista(e, camara) < ENTRADA_MAX) && llegaALaVista(m[0], vacia ? PRONTO : ACERCAMIENTO);
      // Con la vista vacía no hay un tick que perder: se prueban varios sitios de golpe.
      for (let i = 0; i < (vacia ? INTENTOS_URGENTES : 1); i++) {
        if (meterConvoy(entra)) { llegada = TRAFICO.llegadaMs; break; }
      }
    }

    function soltarMisil() {
      progreso = 0;
      empezado = tiempo;
      fase = 'cayendo';
    }

    // Se mira al acabar cada explosión, nunca a mitad de un misil: así el VTOL no se mueve
    // mientras se dispara, y el temporizador del disparo queda parado durante el vuelo.
    // El helicóptero no cuenta: se va solo y no debe retener al VTOL en una zona vacía.
    const hayObjetivos = () => hayObjetivosEn(entidades.filter((e) => !e.volador), ZONAS[zonaIdx]);
    function debeVolar() {
      return estancia >= ESTANCIA_MS || !hayObjetivos();
    }

    // Acabar la última zona es ganar: cada vida que queda suma.
    function victoria() {
      ganada = true;
      puntos += vidas * BONO_VIDA;
      fase = 'final';
      hasta = tiempo;
    }

    // Si la zona nueva está vacía el VTOL sigue de largo sin soltar nada: un misil sobre un
    // suelo sin objetivos sería un fallo regalado.
    function llegar() {
      zonaIdx++;
      estancia = 0;
      ponerHelicoptero();
      proximoPie = sortearPie();
      if (!hayObjetivos()) {
        if (zonaIdx === ZONAS.length - 1) victoria(); else vuelo = 0;
        return;
      }
      vuelo = null;
      soltarMisil();
    }

    // Lo que falta para irse, de 1 a 0: la barra de la pantalla 10. En vuelo no queda nada.
    function quedaParaVolar() {
      if (vuelo !== null) return 0;
      return 1 - Math.min(1, estancia / ESTANCIA_MS);
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
        masALaVez = Math.max(masALaVez, tocados.length);
        for (const e of tocados) {
          muertos[e.tipo.nombre] = (muertos[e.tipo.nombre] || 0) + 1;
          ganados += Math.round(e.tipo.valor * prisa * (1 + e.vel / BONO_VELOCIDAD));
          // Un helicóptero en el aire no deja cráter en el suelo.
          if (!(e.volador && e.vel > 0)) crateres.push({ x: Math.round(e.x + e.forma.w / 2), y: Math.round(e.y + e.forma.h / 2) });
        }
        entidades = entidades.filter((e) => !tocados.includes(e));
        puntos += ganados;
        aciertos += tocados.length;
        aviso = { texto: `+${ganados}`, hasta: tiempo + AVISO_MS };
      } else {
        if (!inmortal) vidas--;
        aviso = { texto: 'FALLO', hasta: tiempo + AVISO_MS };
      }
      fase = 'explosion';
      hasta = tiempo + EXPLOSION_MS;
    }

    // El mundo sigue en marcha mientras cae el misil y durante la explosión.
    function moverMundo(dt) {
      circular(vehiculos(), dt, aleatorio);
      moverHelicopteros(dt);
      trafico(dt);
      andar(dt);
      if (vuelo === null && (proximoPie -= dt) <= 0) { lanzarPie(); proximoPie = sortearPie(); }
    }

    function vaivenear(dt) {
      const amplitud = quieto ? 0 : VAIVEN_PX[zonaIdx];
      if (amplitud) {
        proximoVaiven -= dt;
        if (proximoVaiven <= 0) {
          proximoVaiven = 1500 + aleatorio(2500);
          objetivo.x = (aleatorio(2001) / 1000 - 1) * amplitud;
          objetivo.y = (aleatorio(2001) / 1000 - 1) * amplitud;
        }
      }
      const k = 1 - Math.exp(-dt / VAIVEN_TAU);
      deriva.x += (objetivo.x - deriva.x) * k;
      deriva.y += (objetivo.y - deriva.y) * k;
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
      const luces = e.forma.filas2 && Math.floor(tiempo / 200) % 2;
      pintarSprite(malla, luces ? { ...e.forma, filas: e.forma.filas2 } : e.forma, x, y);
    }

    return {
      // Al acabar se pasa directo a la pantalla de resultados y botones (final.mjs): sin un «FIN» propio.
      get terminado() { return fase === 'final'; },
      get puntos() { return puntos; },
      get detalle() { return { suma: { ...muertos, total: aciertos }, max: { aLaVez: masALaVez, zona: zonaIdx + 1 } }; },
      // Solo lectura, para las pruebas: lo que hay ahora en el mapa.
      get entidades() { return entidades; },
      get camara() { return { ...camara }; },
      get crateres() { return crateres.length; },
      get volando() { return vuelo !== null; },
      get victoria() { return ganada; },
      get estancia() { return estancia; },
      get zona() { return vuelo !== null ? null : ZONAS[zonaIdx].nombre; },
      get estado() { return `${aciertos} objetivos · ${vidas} misiles de margen`; },

      tick(dt) {
        tiempo += dt;
        if (aviso && tiempo >= aviso.hasta) aviso = null;
        if (!mira) return;
        if (fase !== 'final') {
          if (vuelo !== null) {
            vuelo += dt;
            if (vuelo >= duracionVuelo(zonaIdx)) llegar();
          } else {
            estancia += dt;
          }
          vaivenear(dt);
          const c = vuelo === null ? ZONAS[zonaIdx] : trayecto(zonaIdx, vuelo / duracionVuelo(zonaIdx));
          // `camara` indexa el mapa de bits: entera y sin salirse de él.
          camara.x = Math.max(0, Math.min(MAPA_W - MUNDO_W, Math.round(c.x + deriva.x)));
          camara.y = Math.max(0, Math.min(MAPA_H - MUNDO_H, Math.round(c.y + deriva.y)));
          moverMundo(dt);
        }
        if (fase === 'cayendo') {
          progreso += (dt / CAIDA_MS) * PRISA[Math.min(pulsados.size, 2)];
          if (progreso >= 1) estallar();
        } else if (fase === 'explosion' && tiempo >= hasta) {
          if (vidas <= 0) { fase = 'final'; hasta = tiempo; }
          else if (debeVolar()) {
            if (zonaIdx === ZONAS.length - 1) victoria(); else { vuelo = 0; fase = 'volando'; }
          }
          else soltarMisil();
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
          ponerHelicoptero();
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
        } else if (fase === 'explosion') {
          // La explosión: un fogonazo macizo al principio y luego una onda que se abre.
          const t = 1 - (hasta - tiempo) / EXPLOSION_MS;
          if (t < 0.2) malla.disco(impacto.x, impacto.y, RADIO_EXPLOSION);
          else {
            anillo(malla, impacto.x, impacto.y, Math.round(4 + t * 16));
            if (t < 0.6) anillo(malla, impacto.x, impacto.y, Math.round(2 + t * 8));
          }
        }

        // El marcador: el aviso del último misil un momento, los puntos el resto. En vuelo, la zona a la que va.
        const enVuelo = vuelo !== null;
        if (enVuelo) texto(lienzo[9], 'Z' + (zonaIdx + 2));
        else if (aviso) texto(lienzo[9], aviso.texto);
        else etiquetaValor(lienzo[9], 'PUNTOS', puntos);
        // Los misiles de margen, como cuadraditos en la esquina: se cuentan sin leer.
        // Van en las filas 1-4: los puntos empiezan en la 6 y una fila en blanco los separa.
        for (let i = 0; i < vidas; i++) rect(lienzo[9], 2 + i * 6, 1, 4, 4, 1);
        // Lo que queda sobre esta zona: una barra que se vacía.
        if (!enVuelo) {
          rect(lienzo[9], 24, 1, 46, 1, 1); rect(lienzo[9], 24, 4, 46, 1, 1);
          rect(lienzo[9], 24, 1, 1, 4, 1); rect(lienzo[9], 69, 1, 1, 4, 1);
          rect(lienzo[9], 25, 2, Math.round(44 * quedaParaVolar()), 2, 1);
        }
        if (fase === 'explosion' && aviso && aviso.texto !== 'FALLO') destello(lienzo[9], W - 8, H - 7, 4);
      },
    };
  },
};
