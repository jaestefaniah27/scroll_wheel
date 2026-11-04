// Air Strike (id interno `misil`): vista de satélite sobre las nueve teclas, un objetivo en el suelo y un misil que
// ya viene cayendo. La mira se mueve con los dos mandos (el izquierdo en horizontal, el
// derecho en vertical) y el círculo que se cierra sobre ella es el misil: cuando llega al
// centro, explota donde esté la mira. Si el objetivo estaba debajo, puntos; si no, fallo.
// Tres fallos o impactos y se acaba. Un objetivo fallado sigue ahí: el siguiente misil va a por él.
//
// Lo que decide la mira, los sprites y la puntuación por dificultad está aquí; el VTOL, el
// fuego enemigo y los soldados se montan encima, en este mismo fichero.
//
// Fase 2: el suelo es un territorio grande (misil-mapa.mjs) y las pantallas ven una zona de
// él, en ruta de ida: la base con la pista, el puesto del desierto, el puente sobre el río, el
// oasis, el aeródromo y el cuartel. Hay tiendas, soldados, depósitos de combustible, cazas,
// camiones y tanques aparcados, y el tráfico no para: convoyes de camiones y tanques y patrullas
// rápidas por carreteras de sentido único, siempre de cara a donde avanzan (el dibujo se gira en
// cada curva), nunca de lado ni marcha atrás. Un misil se lleva por delante todo lo que quede
// dentro del círculo; lo destruido deja un cráter y no vuelve: el mapa se va vaciando. Fallo es
// que no haya nada dentro. Una zona sin nada que disparar se deja atrás sin gastar misiles. En el
// helipuerto de la base, el aeródromo y el cuartel puede esperar un helicóptero que despega al
// segundo de llegar: si se acierta, vale muchísimo.
//
// Fase 3: el VTOL. Las pantallas son lo que ve el artillero desde el aparato. Se queda 40 s
// (ESTANCIA_MS, fijos) sobre cada zona y luego vuela a la siguiente, con arranque y frenada
// suaves (trayecto) y más tiempo cuanto más lejos esté. Si la zona se queda vacía, se va antes.
// Las zonas van en ruta lineal: al acabar la sexta la partida termina con victoria y las vidas
// que quedan dan puntos (BONO_VIDA). Nunca se mueve con un misil en el aire: se va cuando el
// último ha estallado. La mira va con el aparato: el suelo se desliza por debajo, y el aparato
// se mece (deriva) un poco al azar. Desde la zona 3 el enemigo dispara al VTOL parado:
// antiaéreos, camiones lanzadores en las zonas 5 y 6 y soldados con RPG en la 6. Hay una amenaza
// a la vez (dos en la 6) y el tirador avisado parpadea hasta el impacto; destruirlo antes lo
// quita. Los grupos a pie salen de los edificios, bajan de un camión o una patrulla que aparca
// en una explanada, o suben a un vehículo aparcado, que entonces arranca y se pone a circular.
// Air Strike: a quien te apunta se le ve (alterna su dibujo y su negativo, con un marco de blanco y una línea de
// puntos hasta el VTOL) y la mira lleva una señal de peligro; pasado el apunte sale su misil y, al llegar, «TE HAN
// DADO». Lo destruido estalla y deja un cráter punteado, distinto de cualquier vehículo. Al perder, el mapa no
// desaparece: «HAS PERDIDO» y, si te mató un tirador, un círculo parpadea sobre él; 2 s después, los resultados.
// La pantalla 10 enseña, con una barra, cuánto queda para irse y, en vuelo, la zona a la que va.
//
// Decisiones:
//   · las nueve pantallas son un solo suelo (Malla) con huecos cortos entre ellas. Con el
//     hueco «real» de los otros juegos la mira tardaba demasiadas vueltas de mando en
//     cruzarlo; aquí el hueco solo tiene que hacer que la mira desaparezca un instante
//   · el objetivo nunca cae en un hueco ni cerca del borde de su pantalla (MARGEN): pegado
//     al marco no se distinguía bien dónde estaba, y un objetivo que no se ve no es un reto
//   · sin pulsar los mandos, el misil tarda siempre lo mismo en caer. Acortarlo con cada acierto dejaba tan poco
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
//   · el helicóptero y los que caminan no cuentan para saber si la zona está vacía: se van solos
//     o se quedan quietos, y no deben retener al VTOL
//   · el tráfico no se acaba: entra un convoy nuevo cerca de la vista, fuera de ella (no aparece
//     de la nada delante del jugador), y lo que queda lejos se borra para no gastar
//   · todo lo que cambia de una zona a la siguiente sale de NIVELES, una fila por zona. Sube todo
//     a la vez: subir una sola cosa (la velocidad, por ejemplo) se notaba como un salto, no como
//     una escalada
//   · el enemigo dispara al VTOL parado con un aviso (la amenaza): el tirador alterna su dibujo hasta el
//     impacto. Con el misil tardando hasta 1,8 s en caer, hay que poder responder: una sola
//     amenaza a la vez (dos en la última zona) y 3 s entre un impacto y el siguiente
//   · el misil enemigo siempre va al centro de la vista, que es el VTOL, y no a donde apunta la mira: no hay
//     forma de esquivarlo, solo de destruir al tirador antes (decisión abierta; esquivar sería otra fase)
//   · el mundo se congela en la derrota en vez de saltar a los resultados: ver qué te ha matado es lo que
//     hace que perder se entienda
//   · fallos e impactos gastan la misma vida: un solo contador en la pantalla 10, que se lee de
//     un vistazo, y el juego no tiene que explicar dos
//   · un camión cualquiera puede salirse de la carretera, aparcar y desplegar el lanzador: hasta
//     que se sale parece uno más, y esa es la gracia

import { Malla, W, H, px, leer, rect, destello, etiquetaValor, texto, invertir } from './dibujo.mjs';
import { MAPA_W, MAPA_H, NODOS, SALIDAS, TRAMOS, ESTATICOS, ZONAS, SENDAS, APARCAMIENTOS, EXPLANADA, construirMapa } from './misil-mapa.mjs';

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
// Lo que se enseña «HAS PERDIDO» antes de pasar a los resultados. final.mjs ignora las pulsaciones
// 900 ms (ARMA_MS): con estos 2 s el jugador ve qué le ha matado sin pulsar nada por error.
export const DERROTA_MS = 2000;
const DESTELLO_MS = 150;
// Lo que tarda en llegar el misil enemigo desde que sale. Como `avisoMs` nunca baja de 3500, siempre hay al menos
// 1,7 s de apunte (el marco y la mira) antes de que salga nada: primero se ve que te apuntan y luego sale el misil.
export const VUELO_ENEMIGO_MS = 1800;

// Lo que cambia de una zona a la siguiente. Cada fila aprieta un poco más en todo a la vez:
// subir una sola cosa (la velocidad, por ejemplo) se notaba como un salto, no como una escalada.
// `vel` multiplica la velocidad de los vehículos; `puntos`, lo que vale cada acierto. 0 en
// `disparoMs`, `lanzadorMs` o `pieMs` es «en esta zona no». `avisoMs` nunca baja de 3500: son
// los 1,6-1,8 s de caída con los dos mandos más apuntar, y puede pillarte con otro misil en el aire.
// `heli`, `vaiven` y `pieMs` son de las fases 2 y 5. `simultaneas` (1 si falta) son las amenazas a la vez.
export const NIVELES = [
  //  vel   llegadaMs maxVeh convoyMax patrulla heli vaiven puntos disparoMs avisoMs lanzadorMs rpg pieMs
  { vel: 1.0, llegadaMs: 8000, maxVehiculos: 4, convoyMax: 1, patrulla: 0,   heli: 0.5, vaiven: 2, puntos: 1.0, disparoMs: 0,     avisoMs: 0,    lanzadorMs: 0,     rpg: false, pieMs: 0 },
  { vel: 1.15, llegadaMs: 7000, maxVehiculos: 5, convoyMax: 2, patrulla: 0.2, heli: 0,   vaiven: 3, puntos: 1.2, disparoMs: 0,     avisoMs: 0,    lanzadorMs: 0,     rpg: false, pieMs: 20000 },
  { vel: 1.3, llegadaMs: 6000, maxVehiculos: 6, convoyMax: 2, patrulla: 0.3, heli: 0,   vaiven: 4, puntos: 1.4, disparoMs: 12000, avisoMs: 5000, lanzadorMs: 0,     rpg: false, pieMs: 15000 },
  { vel: 1.5, llegadaMs: 5000, maxVehiculos: 7, convoyMax: 3, patrulla: 0.4, heli: 0,   vaiven: 5, puntos: 1.6, disparoMs: 10000, avisoMs: 4500, lanzadorMs: 0,     rpg: false, pieMs: 12000 },
  { vel: 1.7, llegadaMs: 4000, maxVehiculos: 8, convoyMax: 3, patrulla: 0.5, heli: 0.6, vaiven: 6, puntos: 1.8, disparoMs: 8000,  avisoMs: 4000, lanzadorMs: 15000, rpg: false, pieMs: 10000 },
  { vel: 1.9, llegadaMs: 3500, maxVehiculos: 9, convoyMax: 4, patrulla: 0.6, heli: 0.7, vaiven: 7, puntos: 2.0, disparoMs: 6000,  avisoMs: 3500, lanzadorMs: 12000, rpg: true,  pieMs: 8000, simultaneas: 2 },
];
// Entre un impacto y el siguiente, para dar tiempo a apuntar y a que caiga el misil.
export const ESPACIO_DISPAROS_MS = 3000;
export const LANZADOR_DESPLIEGUE_MS = 2000;
export const DESEMBARCO_MS = 3000;   // lo que espera aparcado un vehículo mientras bajan sus soldados
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
    // Un tanque de combustible visto desde arriba: dos tapas y la correa que lo ciñe por el medio. Esa barra
    // recta es lo que lo separa del cráter (mancha irregular y sin una sola línea) y de la tienda (cuadrada y hueca).
    nombre: 'deposito', velocidad: 0,
    ...sprite([
      '..#####..',
      '.#.....#.',
      '##.###.##',
      '#..#.#..#',
      '#########',
      '#..#.#..#',
      '##.###.##',
      '.#.....#.',
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
    // Lanzador antiaéreo de lado (10x10: más ancho pisa la senda del aeródromo): base ancha con ruedas, dos tubos de misil inclinados hacia arriba y un plato de
    // radar sobre un mástil, a la izquierda. El plato gira en el segundo fotograma: es lo que lo separa de la tienda
    // (hueca y cuadrada) y del depósito (redondo). No se gira: los tubos apuntan al cielo, no a la carretera.
    // Su valor es el de cuando medía 7x7: más grande saldría más barato y no es más fácil de acertar.
    nombre: 'antiaereo', velocidad: 0, dispara: true, valorFijo: true,
    ...sprite([
      '........#.',
      '###....#.#',
      '#.#...#.#.',
      '.#...#.#..',
      '.#..#.#...',
      '.#.#.#....',
      '##########',
      '##########',
      '.##.##.##.',
      '.##.##.##.',
    ], [
      '#.......#.',
      '.#.....#.#',
      '..#...#.#.',
      '.#...#.#..',
      '.#..#.#...',
      '.#.#.#....',
      '##########',
      '##########',
      '.##.##.##.',
      '.##.##.##.',
    ]),
    valor: 41,
  },
  {
    // El soldado con el tubo al hombro: una columna más que el soldado, hacia la derecha.
    nombre: 'rpg', velocidad: 7, dispara: true,
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
  {
    // El camión con la caja cambiada por la rampa: dos tubos y el hueco entre ellos. Es el único
    // aviso de que viene algo. Mismo tamaño que el camión, para que quepa donde él aparcó.
    nombre: 'lanzador', velocidad: 22,
    ...sprite([
      '##########....',
      '#.........####',
      '#.........#.##',
      '#.........####',
      '##########....',
    ]),
  },
];

// Texto grande sobre lo que ya hay dibujado: se le abre un halo de un píxel para que se lea sobre el mapa.
function textoSobreMapa(fb, s) {
  const t = new Uint8Array(fb.length);
  texto(t, s);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    if (leer(t, x, y)) for (let j = -1; j <= 1; j++) for (let i = -1; i <= 1; i++) px(fb, x + i, y + j, 0);
  }
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) if (leer(t, x, y)) px(fb, x, y, 1);
}

// Explosiones al destruir. El depósito (combustible) es una bola de fuego con chispas; un vehículo o un
// tirador revienta a golpes escalonados; una tienda o un soldado, un fogonazo corto. Todo el azar se
// sortea aquí, al crearla, y no al dibujar: así la misma semilla da las mismas explosiones.
const HUMO_MS = 1500;
const CLASE_EXPLOSION = { deposito: 'grande', tienda: 'pequena', soldado: 'pequena', rpg: 'pequena' };
export function crearExplosion(x, y, clase, t0, aleatorio) {
  let estallidos, chispas = 0;
  if (clase === 'grande') { estallidos = [{ dx: 0, dy: 0, t: 0, r: 12, dur: 900 }]; chispas = 8; }
  else if (clase === 'pequena') estallidos = [{ dx: 0, dy: 0, t: 0, r: 4, dur: 300 }];
  else {
    estallidos = [];
    let t = 0;
    for (let i = 0, n = 2 + aleatorio(2); i < n; i++, t += 60 + aleatorio(61)) {
      estallidos.push({ dx: aleatorio(7) - 3, dy: aleatorio(7) - 3, t, r: 4 + aleatorio(2), dur: 300 });
    }
    estallidos.push({ dx: 0, dy: 0, t, r: 7, dur: 450 });
  }
  const humo = Array.from({ length: 3 + aleatorio(2) }, () => aleatorio(5) - 2);
  const humoDesde = t0 + Math.max(...estallidos.map((b) => b.t + b.dur));
  return { x, y, tipo: clase, t0, estallidos, chispas, humo, humoDesde, fin: humoDesde + HUMO_MS };
}

// El cráter que deja lo destruido: una mancha irregular rellena a puntos (un píxel sí, otro no), con el
// contorno a trozos y unas motas de chamusquina alrededor. Regla: nada recto en un cráter y nada redondo
// y hueco en un vehículo, para no confundir un cráter con un tanque o un depósito. La semilla sale de la
// posición, sin azar nuevo: el mismo cráter se dibuja igual en cada fotograma.
// `filas` lleva '#' (tinta), ',' (se limpia, sin tinta) y '.' (no se toca).
const mezcla = (n) => { n = Math.imul(n ^ (n >>> 15), 2246822519); n = Math.imul(n ^ (n >>> 13), 3266489917); return (n ^ (n >>> 16)) >>> 0; };
const LARGO_MAX_RECTO = 4;
export function dibujoCrater({ x, y, w, h, tipo }) {
  const semilla = x * 31 + y;
  const tam = tipo === 'deposito' ? 15 : Math.max(7, Math.min(13, Math.max(w, h)));
  const R = tam / 2, S = tam + 6, c = S >> 1;
  // El radio varía con el ángulo, interpolando entre ocho valores sorteados por la semilla.
  const radioA = (ang) => {
    const u = ((ang + Math.PI) / (2 * Math.PI)) * 8, k = Math.floor(u) % 8, f = u - Math.floor(u);
    const v = (n) => 0.72 + 0.28 * ((mezcla(semilla * 8 + n) % 100) / 100);
    return R * (v(k) * (1 - f) + v((k + 1) % 8) * f);
  };
  const g = Array.from({ length: S }, () => Array(S).fill('.'));
  for (let j = 0; j < S; j++) {
    for (let i = 0; i < S; i++) {
      const d = Math.hypot(i - c, j - c), r = radioA(Math.atan2(j - c, i - c));
      if (d > r + 1) continue;
      g[j][i] = ',';
      if (d <= r - 1) { if ((i + j) % 2 === 0) g[j][i] = '#'; }
      else if (d <= r && mezcla(semilla + i * 7 + j * 13) % 3 !== 0) g[j][i] = '#';
    }
  }
  for (let k = 0, n = 3 + (semilla & 1); k < n; k++) {
    const a = ((mezcla(semilla + 101 * k) % 360) * Math.PI) / 180, d = R + 2 + (mezcla(semilla + 7 * k) % 2);
    const i = c + Math.round(Math.cos(a) * d), j = c + Math.round(Math.sin(a) * d);
    if (g[j]?.[i] === '.') g[j][i] = '#';
  }
  // Ninguna línea de más de LARGO_MAX_RECTO píxeles seguidos, en horizontal ni en vertical.
  const cortar = (get, set) => {
    for (let a = 0; a < S; a++) {
      let run = 0;
      for (let b = 0; b < S; b++) {
        if (get(a, b) === '#') { if (++run > LARGO_MAX_RECTO) { set(a, b, ','); run = 0; } } else run = 0;
      }
    }
  };
  cortar((a, b) => g[a][b], (a, b, v) => { g[a][b] = v; });
  cortar((a, b) => g[b][a], (a, b, v) => { g[b][a] = v; });
  return { w: S, h: S, filas: g.map((f) => f.join('')) };
}

export function pintarSprite(malla, s, x, y, valor = 1) {
  for (let j = 0; j < s.h; j++) {
    for (let i = 0; i < s.w; i++) if (s.filas[j][i] === '#') malla.px(x + i, y + j, valor);
  }
}

// El tirador avisado parpadea, cada vez más deprisa hasta el impacto (de 300 a 80 ms). Aparte, porque lo
// usan el tirador, la señal de la mira y el marcador, y los tres tienen que ir al mismo compás.
export function periodoAviso(a, tiempo) {
  return 300 - 220 * Math.min(1, (tiempo - a.desde) / (a.hasta - a.desde));
}

// Triángulo de peligro con «!» (el palo y el punto salen en negro): 7x6 sobre la mira, 8x7 en el marcador.
const PELIGRO_MIRA = ['...#...', '..###..', '..#.#..', '.##.##.', '.#####.', '###.###'];
const PELIGRO_MARCADOR = ['...##...', '..####..', '..#..#..', '.##..##.', '.######.', '########', '###..###'];
// Con un halo negro de un píxel, para que se lea sobre el terreno. `pon(x, y, valor)` pinta donde toque.
function pintarFilas(pon, filas, x0, y0) {
  const hay = (i, j) => filas[j]?.[i] === '#';
  for (let j = -1; j <= filas.length; j++) for (let i = -1; i <= filas[0].length; i++) {
    if ([-1, 0, 1].some((dj) => [-1, 0, 1].some((di) => hay(i + di, j + dj)))) pon(x0 + i, y0 + j, 0);
  }
  filas.forEach((f, j) => [...f].forEach((c, i) => { if (c === '#') pon(x0 + i, y0 + j, 1); }));
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
// merece la pena quedarse: sin nada dentro, el VTOL se va sin soltar ningún misil.
export function hayObjetivosEn(entidades, zona) {
  // Los que caminan no retienen al VTOL: se van o se quedan quietos en un rato.
  return entidades.some((e) => !e.pie && e.x + e.forma.w > zona.x && e.x < zona.x + MUNDO_W
    && e.y + e.forma.h > zona.y && e.y < zona.y + MUNDO_H);
}

const TIPO = Object.fromEntries(OBJETIVOS.map((o) => [o.nombre, o]));
// Desplegado vale el doble que el camión; a mano, porque su tamaño es el mismo.
TIPO.lanzador.valor = 2 * TIPO.camion.valor;

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
export function circular(vehiculos, ms, aleatorio, factor = 1) {
  const lento = new Map();
  for (const v of vehiculos) if (v.convoy) lento.set(v.convoy, Math.min(lento.get(v.convoy) ?? Infinity, v.tipo.velocidad));
  for (const v of vehiculos) {
    v.vel = (v.convoy ? lento.get(v.convoy) : v.tipo.velocidad) * factor;
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
  // El id no cambia con el nombre: es la clave de las puntuaciones guardadas (scores.mjs) y borraría los récords.
  id: 'misil',
  nombre: 'Air Strike',
  descripcion: 'Eres el artillero de un VTOL. Mueve la mira con los dos mandos (izquierdo: horizontal, derecho: vertical) y pon objetivos dentro del círculo antes de que se cierre y caiga el misil: tiendas, soldados, depósitos, camiones y tanques. Cuanto más pequeño el objetivo, más puntos, y los vehículos en marcha valen más. Pulsa un mando para que caiga antes, o los dos para que caiga aún más rápido: vale más puntos. Lo destruido no vuelve. Son seis zonas en ruta de ida: se acaba en la última, y las vidas que quedan dan puntos. Hay cazas, un helicóptero que despega en algunos helipuertos y patrullas que cruzan las carreteras. Desde la tercera zona te disparan: destruye a quien te apunta antes de que impacte. El VTOL se mece. Tres fallos o impactos y se acaba.',
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

  // Solo para pruebas: `inmortal` (los fallos y los impactos no gastan vidas, para medir un minuto
  // de tráfico), `quieto` (sin vaivén, para apuntar a un objetivo concreto), `zonaInicial` (empezar
  // en otra zona) y `pieMs` (el ritmo de los grupos a pie).
  crear({ aleatorio, inmortal = false, quieto = false, zonaInicial = 0, pieMs }) {
    let malla = null;
    let tiempo = 0;
    let mira = null;              // { x, y } en el suelo; se centra al empezar
    // Todo lo que hay en el mapa: { tipo, forma, x, y, dir, vel } con (x, y) la
    // esquina de arriba a la izquierda en el suelo y `vel` en px/s. Los vehículos que
    // circulan llevan además { desde, hasta, dist } (ver posicionar).
    let entidades = [];
    let explosiones = [];         // { x, y, t0, ... } en el territorio, como los cráteres: la cámara se mueve y siguen en su sitio
    let crateres = [];            // { x, y, w, h, tipo }: lo destruido no vuelve, el cráter se queda (w, h: la caja del destruido)
    let mapa = null;              // el dibujo fijo del territorio, ver construirMapa
    // La esquina de arriba a la izquierda de lo que se ve, en el territorio. La mira, el
    // impacto y la malla van en coordenadas de la vista; las entidades y los cráteres, del
    // territorio.
    let zonaIdx = zonaInicial;    // la zona donde está el VTOL
    const camara = { x: ZONAS[zonaIdx].x, y: ZONAS[zonaIdx].y };
    const nivel = () => NIVELES[zonaIdx];
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
    let fase = 'cayendo';         // 'cayendo' | 'explosion' | 'derrota' | 'final'
    let hasta = 0;
    let impacto = null;           // { x, y }: dónde estalló el último
    let aviso = null;             // { texto, hasta }
    let culpable = null;          // { x, y, w, h } (territorio) del tirador que dio el golpe final, o null
    let derrotaDesde = 0;
    let golpe = null;             // { desde, hasta, vida }: el último impacto que no mató, para el «TE HAN DADO» y la vida que parpadea
    let porImpacto = false;       // si la derrota fue por un impacto (destello) o por fallos
    let vidas = VIDAS;
    let puntos = 0;
    let aciertos = 0;
    const muertos = {};           // objetivos destruidos por tipo
    let masALaVez = 0;            // el máximo alcanzado por un solo misil
    const pulsados = new Set();
    const ultimoPaso = { 1: -Infinity, 2: -Infinity };

    let llegada = nivel().llegadaMs;   // ms que faltan para el próximo convoy

    // El fuego enemigo. `amenazas`: { e, desde, hasta } del tirador que ha apuntado al VTOL;
    // `ultimoImpacto` es cuándo cayó el último, para guardar el espacio entre disparos.
    let amenazas = [];
    let ultimoImpacto = -Infinity;
    let impactos = 0;
    let proximoDisparo = nivel().disparoMs;
    let proximoLanzador = 0;           // se sortea al llegar a una zona con lanzador
    let cola = [];                     // lanzadores desplegados que esperan su primer disparo

    function crearEstatica([nombre, x, y, dir = 'der', arranque]) {
      const tipo = TIPO[nombre];
      return { tipo, forma: orientar(tipo, dir), x, y, dir, vel: 0, tirador: !!tipo.dispara, ...(arranque && { arranque }) };
    }

    const vehiculos = () => entidades.filter((e) => e.desde);

    // Grupos a pie (fase 5). `pie` en la entidad lleva { ruta, s, tope, alFinal }: `s` es lo andado por
    // la senda (negativo mientras está en fila detrás de la puerta) y `tope` dónde se para.
    const pieMsZona = () => pieMs ?? nivel().pieMs;
    const sortearPie = () => { const ms = pieMsZona(); return ms ? ms / 2 + aleatorio(ms) : Infinity; };
    let proximoPie = sortearPie();

    // Al saltar el temporizador se elige al azar entre las situaciones posibles en esta zona: salir de un
    // edificio (5.1), bajar de un vehículo que va a pasar por una explanada libre (5.2) o subir al
    // vehículo aparcado del final de una senda (5.3), si sigue vivo y nadie va ya hacia él.
    function lanzarPie() {
      const zona = ZONAS[zonaIdx].nombre;
      const sendas = SENDAS.filter((d) => d.zona === zona);
      const situaciones = [];
      const edificios = sendas.filter((d) => d.de !== 'aparcamiento' && d.fin !== 'subir');
      if (edificios.length) situaciones.push(() => soltarGrupo(edificios[aleatorio(edificios.length)]));
      const bajan = candidatos([TIPO.camion, TIPO.patrulla]);
      if (bajan.length) situaciones.push(() => asignarMision(bajan, 'desembarcar'));
      const suben = sendas.filter((d) => d.fin === 'subir').map((d) => [d, vehiculoDelFinal(d)])
        .filter(([, v]) => v && !entidades.some((e) => e.pie?.objetivo === v));
      if (suben.length) situaciones.push(() => { const [d, v] = suben[aleatorio(suben.length)]; soltarGrupo(d, v); });
      if (situaciones.length) situaciones[situaciones.length > 1 ? aleatorio(situaciones.length) : 0]();
    }

    // El vehículo aparcado que espera al final de una senda 'subir': el que tiene `arranque` más cerca de su último punto.
    function vehiculoDelFinal({ puntos }) {
      const [fx, fy] = puntos[puntos.length - 1];
      let mejor = null, dmin = 25;
      for (const e of entidades) {
        if (!e.arranque || e.desde || e.mision) continue;
        const d = Math.hypot(e.x + e.forma.w / 2 - fx, e.y + e.forma.h / 2 - fy);
        if (d < dmin) { mejor = e; dmin = d; }
      }
      return mejor;
    }

    function soltarGrupo({ puntos: ruta, fin }, objetivo) {
      let total = 0;
      for (let i = 1; i < ruta.length; i++) total += Math.hypot(ruta[i][0] - ruta[i - 1][0], ruta[i][1] - ruta[i - 1][1]);
      const grupo = [];
      for (let k = 0, n = 2 + aleatorio(3); k < n; k++) {
        // Donde hay RPG, uno de cada cuatro lo lleva. No dispara hasta pararse: andando se le caza.
        const tipo = nivel().rpg && aleatorio(4) === 0 ? TIPO.rpg : TIPO.soldado;
        // Los que se quedan lo hacen en fila contra el final, no todos en el mismo píxel; los que suben, todos hasta el vehículo.
        const pie = { ruta, s: -k * SEPARACION_PIE, tope: fin === 'quedarse' ? total - k * SEPARACION_PIE : total, alFinal: fin, k, total, grupo, objetivo };
        const e = { tipo, forma: tipo, x: 0, y: 0, dir: 'der', vel: tipo.velocidad, pie };
        grupo.push(e);
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
        const p = e.pie;
        if (!p) continue;
        // Si destruyen el vehículo al que iban, acaban como un grupo cualquiera que se queda.
        if (p.alFinal === 'subir' && !entidades.includes(p.objetivo)) { p.alFinal = 'quedarse'; p.tope = p.total - p.k * SEPARACION_PIE; }
        p.s = Math.min(p.tope, p.s + (e.vel * dt) / 1000);
        colocarPie(e);
        if (p.s < p.tope) continue;
        // Entrar es escaparse: desaparece sin dar puntos. Quedarse lo vuelve un estático más.
        if (p.alFinal === 'entrar') e.pie = null;
        else if (p.alFinal === 'subir') {
          e.vel = 0;
          // Suben todos a la vez, cuando llega el último.
          const vivos = p.grupo.filter((m) => entidades.includes(m));
          if (vivos.every((m) => m.pie.s >= m.pie.tope)) { for (const m of vivos) m.pie = null; arrancar(p.objetivo); }
        } else { delete e.pie; e.vel = 0; e.tirador = !!e.tipo.dispara; }
      }
      entidades = entidades.filter((e) => e.pie !== null);
    }

    // Intenta meter un convoy en un tramo y un punto al azar; `vale(miembros)` dice si el sitio sirve.
    // Una sola tentativa: si no sirve o no cabe, se prueba otra vez en el siguiente tick. Las
    // patrullas van solas, nunca en convoy: son las rápidas.
    function meterConvoy(vale) {
      const sola = aleatorio(1000) / 1000 < nivel().patrulla;
      const tipos = sola ? [TIPO.patrulla]
        : Array.from({ length: 1 + aleatorio(nivel().convoyMax) }, () => TIPO[aleatorio(2) ? 'camion' : 'tanque']);
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
      if (!hp || aleatorio(1000) / 1000 >= nivel().heli) return;
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
      const sonda = { ...v, convoy: v.convoy && { camino: [...v.convoy.camino] } };
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
      if (!vacia && (llegada > 0 || hay.filter((e) => distanciaAVista(e, camara) < CERCA).length >= nivel().maxVehiculos)) return;
      const entra = (m) => m.every((e) => distanciaAVista(e, camara) > ENTRADA_MIN)
        && m.some((e) => distanciaAVista(e, camara) < ENTRADA_MAX) && llegaALaVista(m[0], vacia ? PRONTO : ACERCAMIENTO);
      // Con la vista vacía no hay un tick que perder: se prueban varios sitios de golpe.
      for (let i = 0; i < (vacia ? INTENTOS_URGENTES : 1); i++) {
        if (meterConvoy(entra)) { llegada = nivel().llegadaMs; break; }
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
      proximoDisparo = nivel().disparoMs;
      proximoLanzador = sortearLanzador();
      if (!hayObjetivos()) {
        if (zonaIdx === ZONAS.length - 1) victoria(); else vuelo = 0;
        return;
      }
      vuelo = null;
      soltarMisil();
    }

    // --- Fuego enemigo (fase 4) ---

    // El impacto sacude al VTOL para que se note: un golpe de deriva que el vaivén luego recoge.
    function sacudir() {
      if (quieto) return;
      deriva.x += aleatorio(2) ? 10 : -10;
      deriva.y += aleatorio(2) ? 6 : -6;
    }

    const tiradores = () => entidades.filter((e) => e.tirador && hayObjetivosEn([e], ZONAS[zonaIdx]));

    // El aviso dura `avisoMs`, o hasta 3 s después del último impacto previsto si eso es más:
    // así dos disparos nunca caen juntos aunque haya dos amenazas a la vez.
    function amenazar(e) {
      const ultimo = amenazas.length ? Math.max(...amenazas.map((a) => a.hasta)) : ultimoImpacto;
      const hasta = Math.max(tiempo + nivel().avisoMs, ultimo + ESPACIO_DISPAROS_MS);
      amenazas.push({ e, desde: tiempo, lanzaEn: hasta - VUELO_ENEMIGO_MS, hasta });
    }

    function impactar(a) {
      amenazas = amenazas.filter((o) => o !== a);
      ultimoImpacto = a.hasta;
      impactos++;
      if (!inmortal) vidas--;
      aviso = { texto: 'TOCADO', hasta: tiempo + AVISO_MS };
      sacudir();
      if (vidas <= 0) perder(a.e);
      else golpe = { desde: tiempo, hasta: tiempo + AVISO_MS, vida: inmortal ? -1 : vidas };
    }

    // Dónde está el misil enemigo de `a` (en la vista) cuando va por `k` del vuelo, de 0 a 1. Sale del centro del
    // tirador y acelera hasta el VTOL (el centro de la vista): k² en vez de k, como un misil de verdad. Se calcula
    // con la cámara de ahora, no con la de cuando salió: el vaivén mueve la vista y el misil no puede quedarse atrás.
    function posMisilEnemigo(a, k) {
      const x0 = a.e.x + a.e.forma.w / 2 - camara.x, y0 = a.e.y + a.e.forma.h / 2 - camara.y;
      const x1 = Math.round(malla.ancho / 2), y1 = Math.round(malla.alto / 2);
      const q = Math.max(0, Math.min(1, k)) ** 2;
      return { x: x0 + (x1 - x0) * q, y: y0 + (y1 - y0) * q };
    }
    const kMisilEnemigo = (a) => (tiempo - a.lanzaEn) / (a.hasta - a.lanzaEn);
    const enElAire = (a) => tiempo >= a.lanzaEn && tiempo < a.hasta;

    // Congelar el mundo en vez de saltar a los resultados: el jugador ve el mapa donde ha perdido y, si
    // fue un tirador, cuál. `culpable` se guarda como caja del territorio porque la cámara ya no se mueve.
    function perder(tirador) {
      fase = 'derrota';
      derrotaDesde = tiempo;
      hasta = tiempo + DERROTA_MS;
      porImpacto = !!tirador;
      golpe = null;
      culpable = tirador ? { x: Math.round(tirador.x), y: Math.round(tirador.y), w: tirador.forma.w, h: tirador.forma.h } : null;
      amenazas = [];
      cola = [];
    }

    function disparar(dt) {
      const libre = () => amenazas.length < (nivel().simultaneas ?? 1);
      // Un lanzador recién desplegado dispara sin esperar al turno; si no cabe, espera: no se descarta.
      cola = cola.filter((e) => entidades.includes(e));
      while (cola.length && libre()) {
        const e = cola.shift();
        if (!amenazas.some((a) => a.e === e) && hayObjetivosEn([e], ZONAS[zonaIdx])) amenazar(e);
      }
      if (nivel().disparoMs) {
        proximoDisparo -= dt;
        if (proximoDisparo <= 0 && libre()) {
          const quedan = tiradores().filter((e) => !amenazas.some((a) => a.e === e));
          if (quedan.length) amenazar(quedan[aleatorio(quedan.length)]);
          proximoDisparo = nivel().disparoMs;
        }
      }
      for (const a of [...amenazas]) if (tiempo >= a.hasta && fase !== 'final' && fase !== 'derrota') impactar(a);
    }

    // --- Camión lanzamisiles ---

    const sortearLanzador = () => (nivel().lanzadorMs ? Math.round(nivel().lanzadorMs * (0.5 + aleatorio(1001) / 1000)) : 0);

    // Los vehículos de `tipos` que circulan, sin misión, y todavía no han pasado por una explanada
    // libre de esta zona (libre: sin lanzador ni desembarco): [[v, ap], ...].
    function candidatos(tipos) {
      const libres = APARCAMIENTOS.filter((a) => a.zona === ZONAS[zonaIdx].nombre && !entidades.some((e) => e.mision?.ap === a));
      const cands = [];
      for (const v of vehiculos()) {
        if (!tipos.includes(v.tipo) || v.mision) continue;
        for (const ap of libres) {
          const aqui = v.desde + v.hasta === ap.tramo ? v.dist < ap.dist : v.hasta === ap.tramo[0] && v.luego === ap.tramo[1];
          if (aqui) cands.push([v, ap]);
        }
      }
      return cands;
    }

    // `luego` es lo que hace al aparcar: 'lanzar' despliega el lanzador; 'desembarcar' suelta soldados y vuelve.
    function asignarMision(cands, luego) {
      const [v, ap] = cands[aleatorio(cands.length)];
      v.mision = { ap, luego, fase: 'red', t: 0 };
    }

    function elegirLanzador() {
      const cands = candidatos([TIPO.camion]);
      if (cands.length) asignarMision(cands, 'lanzar');
    }

    // Deja la red: sale recto de la carretera por el acceso, de cara a la explanada.
    function salirDeLaRed(v) {
      const { ap } = v.mision;
      const [p, q] = ap.tramo.split('').map((n) => NODOS[n]);
      v.desde = v.hasta = v.convoy = undefined;
      v.dir = ap.x > p[0] ? 'der' : 'izq';
      v.forma = orientar(v.tipo, v.dir);
      v.cx = p[0];
      v.cy = p[1] + Math.sign(q[1] - p[1]) * ap.dist;
      v.mision.fase = 'entra';
    }

    // 5.3: ya han subido; el vehículo sale recto hasta su punto de la carretera, de cara a donde va. El
    // aparcamiento puede quedar de lado, así que se gira hacia el eje en que más le falta.
    function arrancar(v) {
      const { tramo, dist } = v.arranque;
      const [p, q] = tramo.split('').map((n) => NODOS[n]);
      const meta = [p[0] + Math.sign(q[0] - p[0]) * dist, p[1] + Math.sign(q[1] - p[1]) * dist];
      v.cx = v.x + v.forma.w / 2;
      v.cy = v.y + v.forma.h / 2;
      const [dx, dy] = [meta[0] - v.cx, meta[1] - v.cy];
      v.dir = Math.abs(dx) > Math.abs(dy) ? (dx > 0 ? 'der' : 'izq') : (dy > 0 ? 'aba' : 'arr');
      v.forma = orientar(v.tipo, v.dir);
      v.mision = { luego: 'arrancar', fase: 'sale', meta, tramo, dist };
    }

    // Vuelve a la red en (tramo, dist) si hay hueco con los de alrededor; si no, se queda como estaba.
    function incorporar(v, tramo, dist) {
      const antes = { x: v.x, y: v.y, dir: v.dir, forma: v.forma };
      Object.assign(v, { desde: tramo[0], hasta: tramo[1], luego: undefined, dist });
      posicionar(v);
      const todos = vehiculos();
      if (todos.every((o) => o === v || (holgura(o, [o, v]) >= 0 && holgura(v, [v, o]) >= 0))) return true;
      Object.assign(v, antes, { desde: undefined, hasta: undefined });
      return false;
    }

    // Recto hacia `meta` por el eje al que mira; al llegar intenta incorporarse y, sin hueco, espera.
    function salir(v, dt) {
      const m = v.mision;
      const horizontal = v.dir === 'der' || v.dir === 'izq';
      const eje = horizontal ? 'cx' : 'cy';
      const fin = m.meta[horizontal ? 0 : 1];
      const paso = (v.tipo.velocidad * nivel().vel * dt) / 1000;
      v[eje] += Math.max(-paso, Math.min(paso, fin - v[eje]));
      v.vel = v[eje] === fin ? 0 : v.tipo.velocidad * nivel().vel;
      if (v[eje] !== fin) return;
      [v.cx, v.cy] = m.meta;
      if (incorporar(v, m.tramo, m.dist)) { delete v.mision; delete v.cx; delete v.cy; }
    }

    const OPUESTA = { der: 'izq', izq: 'der', arr: 'aba', aba: 'arr' };

    function moverLanzadores(dt) {
      for (const v of entidades) {
        const m = v.mision;
        if (!m) continue;
        if (m.fase === 'red') {
          if (v.desde + v.hasta === m.ap.tramo) { if (v.dist >= m.ap.dist) salirDeLaRed(v); }
          else if (!(v.hasta === m.ap.tramo[0] && v.luego === m.ap.tramo[1])) delete v.mision;  // ya la pasó
        } else if (m.fase === 'entra') {
          const meta = m.ap.x + EXPLANADA / 2;
          const sentido = v.dir === 'der' ? 1 : -1;
          v.vel = v.tipo.velocidad * nivel().vel;
          v.cx = sentido > 0 ? Math.min(meta, v.cx + (v.vel * dt) / 1000) : Math.max(meta, v.cx - (v.vel * dt) / 1000);
          if (v.cx === meta) {
            v.vel = 0;
            m.t = 0;
            if (m.luego === 'lanzar') m.fase = 'despliega';
            else {
              // Se gira al aparcar para quedar de cara a la salida: así vuelve a la carretera marcha adelante.
              m.fase = 'baja';
              v.dir = OPUESTA[v.dir];
              v.forma = orientar(v.tipo, v.dir);
              soltarGrupo(SENDAS.find((d) => d.zona === m.ap.zona && d.de === 'aparcamiento'));
            }
          }
        } else if (m.fase === 'despliega') {
          m.t += dt;
          if (m.t >= LANZADOR_DESPLIEGUE_MS / 2 && v.tipo === TIPO.camion) { v.tipo = TIPO.lanzador; v.forma = orientar(v.tipo, v.dir); }
          if (m.t >= LANZADOR_DESPLIEGUE_MS) { m.fase = 'listo'; v.tirador = true; cola.push(v); }
        } else if (m.fase === 'baja') {
          m.t += dt;
          if (m.t >= DESEMBARCO_MS) Object.assign(m, { fase: 'sale', meta: [NODOS[m.ap.tramo[0]][0], v.cy], tramo: m.ap.tramo, dist: m.ap.dist });
        } else if (m.fase === 'sale') salir(v, dt);
        if (v.mision && m.fase !== 'red') { v.x = v.cx - v.forma.w / 2; v.y = v.cy - v.forma.h / 2; }
      }
      if (nivel().lanzadorMs && vuelo === null && (proximoLanzador -= dt) <= 0) {
        elegirLanzador();
        proximoLanzador = sortearLanzador();
      }
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
        for (const [orden, e] of tocados.entries()) {
          muertos[e.tipo.nombre] = (muertos[e.tipo.nombre] || 0) + 1;
          ganados += Math.round(e.tipo.valor * prisa * (1 + e.vel / BONO_VELOCIDAD) * nivel().puntos);
          // Un helicóptero en el aire no deja cráter en el suelo.
          const ex = Math.round(e.x + e.forma.w / 2), ey = Math.round(e.y + e.forma.h / 2);
          if (!(e.volador && e.vel > 0)) crateres.push({ x: ex, y: ey, w: e.forma.w, h: e.forma.h, tipo: e.tipo.nombre });
          // Desfasadas 40 ms por orden: los destruidos de un mismo misil no estallan todos a la vez.
          explosiones.push(crearExplosion(ex, ey, CLASE_EXPLOSION[e.tipo.nombre] ?? 'media', tiempo + 40 * orden, aleatorio));
        }
        entidades = entidades.filter((e) => !tocados.includes(e));
        // Destruir al tirador con su misil ya en el aire lo cancela (el premio por reaccionar) y el misil se
        // deshace con una explosión pequeña donde esté, en lugar de desaparecer sin más.
        for (const a of amenazas) {
          if (!tocados.includes(a.e) || !enElAire(a)) continue;
          const m = posMisilEnemigo(a, kMisilEnemigo(a));
          explosiones.push(crearExplosion(Math.round(m.x + camara.x), Math.round(m.y + camara.y), 'pequena', tiempo + 40 * tocados.length, aleatorio));
        }
        amenazas = amenazas.filter((a) => entidades.includes(a.e));
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
      circular(vehiculos(), dt, aleatorio, nivel().vel);
      moverHelicopteros(dt);
      moverLanzadores(dt);
      trafico(dt);
      andar(dt);
      if (vuelo === null && (proximoPie -= dt) <= 0) { lanzarPie(); proximoPie = sortearPie(); }
    }

    function vaivenear(dt) {
      const amplitud = quieto ? 0 : nivel().vaiven;
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
      if (fase === 'final' || fase === 'derrota' || !mira || !delta) return;
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
      const forma = luces ? { ...e.forma, filas: e.forma.filas2 } : e.forma;
      // El tirador avisado no desaparece (así no se pierde de vista): alterna su dibujo y su negativo.
      const a = amenazas.find((o) => o.e === e);
      if (a && Math.floor(tiempo / periodoAviso(a, tiempo)) % 2) {
        malla.rect(x, y, e.forma.w, e.forma.h, 1);
        pintarSprite(malla, forma, x, y, 0);
        return;
      }
      pintarSprite(malla, forma, x, y);
    }

    // Quién te apunta: una línea de puntos (uno de cada tres píxeles) del tirador al VTOL, que es el centro de la
    // vista, donde empieza la mira. Va debajo de las entidades: no tapa nada.
    function pintarLineaDeApunte(a) {
      const x0 = a.e.x + a.e.forma.w / 2 - camara.x, y0 = a.e.y + a.e.forma.h / 2 - camara.y;
      const x1 = Math.round(malla.ancho / 2), y1 = Math.round(malla.alto / 2);
      const n = Math.max(1, Math.round(Math.hypot(x1 - x0, y1 - y0)));
      for (let k = 0; k <= n; k += 3) malla.px(Math.round(x0 + ((x1 - x0) * k) / n), Math.round(y0 + ((y1 - y0) * k) / n));
    }

    // El marco de blanco fijado: cuatro esquinas de 3 px a 2 px de la caja, parpadeando a la vez.
    function pintarMarco(a) {
      if (Math.floor(tiempo / periodoAviso(a, tiempo)) % 2) return;
      const { x: ex, y: ey } = a.e, { w, h } = a.e.forma;
      const x = Math.round(ex) - camara.x - 3, y = Math.round(ey) - camara.y - 3, W2 = w + 6, H2 = h + 6;
      for (const [cx, cy, sx, sy] of [[x, y, 1, 1], [x + W2 - 1, y, -1, 1], [x, y + H2 - 1, 1, -1], [x + W2 - 1, y + H2 - 1, -1, -1]]) {
        for (let k = 0; k < 3; k++) { malla.px(cx + sx * k, cy); malla.px(cx, cy + sy * k); }
      }
    }

    // Cabeza de 2x2 con halo negro, una estela de cuatro píxeles separados por la trayectoria y una chispa en la
    // cola que parpadea. Pasa por los huecos entre pantallas sin dibujarse, como la mira: no es un fallo.
    function pintarMisilEnemigo(a) {
      const k = kMisilEnemigo(a);
      const en = (dk) => { const m = posMisilEnemigo(a, k - dk); return [Math.round(m.x), Math.round(m.y)]; };
      for (let j = 1; j <= 4; j++) { const [x, y] = en(0.035 * j); malla.px(x, y); }
      if (Math.floor(tiempo / 50) % 2) { const [x, y] = en(0.035 * 5); malla.px(x, y); }
      const [x, y] = en(0);
      malla.rect(x - 1, y - 1, 4, 4, 0);
      malla.rect(x, y, 2, 2, 1);
    }

    // La señal de peligro en la mira: cuatro esquinas de 4 px a 12 px del centro y el triángulo con «!» encima,
    // al compás de la amenaza más urgente. Así se nota sin apartar la vista de lo que se apunta.
    function pintarPeligroEnMira(cx, cy) {
      if (!amenazas.length) return;
      const periodo = Math.min(...amenazas.map((a) => periodoAviso(a, tiempo)));
      if (Math.floor(tiempo / periodo) % 2) return;
      const pon = (x, y, v) => malla.px(x, y, v);
      const esquinas = [];
      for (const [sx, sy] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) {
        for (let k = 0; k < 4; k++) esquinas.push([cx + sx * 12 - sx * k, cy + sy * 12], [cx + sx * 12, cy + sy * 12 - sy * k]);
      }
      esquinas.forEach(([x, y]) => { for (let j = -1; j <= 1; j++) for (let i = -1; i <= 1; i++) pon(x + i, y + j, 0); });
      esquinas.forEach(([x, y]) => pon(x, y, 1));
      pintarFilas(pon, PELIGRO_MIRA, cx - 3, cy - 21);
    }

    // Se limpia solo la mancha, no una caja entera: lo que hay junto al cráter se queda.
    function pintarCrater(c) {
      const d = dibujoCrater(c);
      const x0 = c.x - (d.w >> 1) - camara.x, y0 = c.y - (d.h >> 1) - camara.y;
      d.filas.forEach((f, j) => [...f].forEach((ch, i) => {
        if (ch !== '.') malla.px(x0 + i, y0 + j, ch === '#' ? 1 : 0);
      }));
    }

    // Un solo bit: el fogonazo alterna disco blanco y anillo con halo negro, que es lo que se ve sobre el terreno.
    function pintarExplosion(ex) {
      const x0 = ex.x - camara.x, y0 = ex.y - camara.y;
      for (const b of ex.estallidos) {
        const tau = tiempo - ex.t0 - b.t;
        if (tau < 0 || tau >= b.dur) continue;
        const k = tau / b.dur;
        const r = Math.max(1, Math.round(b.r * (k < 0.6 ? 0.5 + (0.5 * k) / 0.6 : 1 - ((k - 0.6) / 0.4) * 0.5)));
        malla.disco(x0 + b.dx, y0 + b.dy, r + 1, 0);
        if (Math.floor(tau / 80) % 2 === 0) malla.disco(x0 + b.dx, y0 + b.dy, r); else anillo(malla, x0 + b.dx, y0 + b.dy, r);
      }
      const tc = tiempo - ex.t0;
      if (ex.chispas && tc >= 0 && tc < 700) {
        for (let i = 0; i < ex.chispas; i++) {
          const a = (i * Math.PI * 2) / ex.chispas, d = 5 + (i % 2) * 3 + (tc / 700) * 12;
          const cx = Math.round(x0 + Math.cos(a) * d), cy = Math.round(y0 + Math.sin(a) * d);
          malla.rect(cx - 1, cy - 1, 3, 3, 0); malla.px(cx, cy);
        }
      }
      const th = tiempo - ex.humoDesde;
      if (th >= 0 && th < HUMO_MS) {
        ex.humo.forEach((dx, i) => malla.px(x0 + dx + ((Math.floor(th / 250) + i) % 2), y0 - 2 - Math.round((th / HUMO_MS) * 10) - i));
      }
    }

    // El mapa sigue ahí, atenuado con una trama de ajedrez (un bit por píxel: no hay grises), y el texto
    // encima. El culpable se vuelve a pintar en claro y lo rodean dos círculos que se alternan.
    function pintarDerrota(lienzo) {
      for (let p = 0; p < 9; p++) for (let y = 0; y < H; y++) for (let x = (y & 1); x < W; x += 2) px(lienzo[p], x, y, 0);
      textoSobreMapa(lienzo[3], 'HAS');
      textoSobreMapa(lienzo[4], 'PERDIDO');
      if (culpable) {
        const e = entidades.find((o) => Math.round(o.x) === culpable.x && Math.round(o.y) === culpable.y);
        if (e) pintarEntidad(e);
        const x = culpable.x + (culpable.w >> 1) - camara.x, y = culpable.y + (culpable.h >> 1) - camara.y;
        anillo(malla, x, y, Math.floor(tiempo / 250) % 2 ? 9 : 13);
      }
      etiquetaValor(lienzo[9], 'PUNTOS', puntos);
      if (porImpacto && tiempo - derrotaDesde < DESTELLO_MS) for (let p = 0; p < 9; p++) invertir(lienzo[p]);
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
      get explosiones() { return explosiones.map(({ x, y, t0, tipo, fin }) => ({ x, y, t0, tipo, fin })); },
      get volando() { return vuelo !== null; },
      get victoria() { return ganada; },
      get estancia() { return estancia; },
      get amenazas() { return amenazas.map((a) => ({ ...a })); },
      // Los misiles enemigos en el aire, en coordenadas de la vista.
      get misilesEnemigos() { return amenazas.filter(enElAire).map((a) => posMisilEnemigo(a, kMisilEnemigo(a))); },
      get golpe() { return golpe && { ...golpe }; },
      get impactos() { return impactos; },
      get vidas() { return vidas; },
      get fase() { return fase; },
      get culpable() { return culpable && { ...culpable }; },
      get mira() { return mira && { ...mira }; },
      get zona() { return vuelo !== null ? null : ZONAS[zonaIdx].nombre; },
      get estado() { return `${aciertos} objetivos · ${vidas} misiles de margen`; },

      tick(dt) {
        tiempo += dt;
        explosiones = explosiones.filter((e) => tiempo < e.fin);   // también en la derrota y mientras cae el misil
        if (aviso && tiempo >= aviso.hasta) aviso = null;
        if (golpe && tiempo >= golpe.hasta) golpe = null;
        if (!mira) return;
        if (fase === 'derrota') {
          if (tiempo >= hasta) fase = 'final';
          return;
        }
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
        // Se dispara solo al VTOL parado: en vuelo el aviso se pierde con la zona.
        if (vuelo !== null) { amenazas = []; cola = []; }
        else if (fase !== 'final') disparar(dt);
        if (fase === 'cayendo') {
          progreso += (dt / CAIDA_MS) * PRISA[Math.min(pulsados.size, 2)];
          if (progreso >= 1) estallar();
        } else if (fase === 'explosion' && tiempo >= hasta) {
          if (vidas <= 0) perder(null);
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
        for (const c of crateres) pintarCrater(c);
        for (const a of amenazas) pintarLineaDeApunte(a);
        for (const e of entidades) pintarEntidad(e);
        for (const a of amenazas) pintarMarco(a);
        for (const a of amenazas) if (enElAire(a)) pintarMisilEnemigo(a);
        for (const ex of explosiones) pintarExplosion(ex);

        if (fase === 'derrota') { pintarDerrota(lienzo); return; }

        const cx = Math.round(mira.x);
        const cy = Math.round(mira.y);

        if (fase === 'cayendo') {
          cruz(malla, cx, cy);
          pintarPeligroEnMira(cx, cy);
          const r = Math.round(RADIO_INICIAL + (RADIO_FINAL - RADIO_INICIAL) * Math.min(1, progreso));
          anillo(malla, cx, cy, r);
        } else if (fase === 'explosion') {
          pintarPeligroEnMira(cx, cy);
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
        // Con una amenaza en marcha, el triángulo de peligro junto a las vidas, al mismo compás que la mira.
        if (amenazas.length && Math.floor(tiempo / Math.min(...amenazas.map((a) => periodoAviso(a, tiempo)))) % 2 === 0) {
          PELIGRO_MARCADOR.forEach((f, j) => [...f].forEach((c, i) => { if (c === '#') px(lienzo[9], 21 + i, j, 1); }));
        }
        // Lo que queda sobre esta zona: una barra que se vacía. Empieza después del triángulo.
        if (!enVuelo) {
          rect(lienzo[9], 31, 1, 39, 1, 1); rect(lienzo[9], 31, 4, 39, 1, 1);
          rect(lienzo[9], 31, 1, 1, 4, 1); rect(lienzo[9], 69, 1, 1, 4, 1);
          rect(lienzo[9], 32, 2, Math.round(37 * quedaParaVolar()), 2, 1);
        }
        if (fase === 'explosion' && aviso && aviso.texto !== 'FALLO') destello(lienzo[9], W - 8, H - 7, 4);
        // La vida que acaba de costar un impacto parpadea tres veces antes de irse, en lugar de quitarse de golpe.
        if (golpe && golpe.vida >= 0) {
          const t = tiempo - golpe.desde;
          if (t < 900 && Math.floor(t / 150) % 2 === 0) rect(lienzo[9], 2 + golpe.vida * 6, 1, 4, 4, 1);
        }
        if (golpe) {
          textoSobreMapa(lienzo[3], 'TE HAN');
          textoSobreMapa(lienzo[4], 'DADO');
          if (tiempo - golpe.desde < DESTELLO_MS) for (let p = 0; p < 9; p++) invertir(lienzo[p]);
        }
      },
    };
  },
};
