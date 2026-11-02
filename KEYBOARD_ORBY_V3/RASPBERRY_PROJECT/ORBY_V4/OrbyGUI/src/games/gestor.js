// Quien lleva las sesiones de juego de la app, vengan de donde vengan.
//
// Una sesión puede empezar de dos maneras y las dos acaban en el mismo recorrido (flujo.mjs):
//   · en la app: el botón «Jugar» de la vista de Juegos arranca ese juego
//   · en el teclado: «Juegos» en su selector de perfiles. El teclado entra en modo juego
//     por su cuenta y avisa con GAME:OPEN; la app, que es quien tiene los juegos, abre el
//     menú en sus pantallas
//
// El teclado necesita siempre la app para jugar, y la app el teclado, así que los dos
// tienen que estar de acuerdo en cada momento de si se está jugando. Por eso este módulo
// escucha desde el arranque y no solo cuando la vista de Juegos está abierta, y por eso
// recoge también el caso de llegar tarde: si el teclado ya estaba en modo juego cuando la
// app se conecta (se reabrió la ventana, o se enchufó con el juego a medias) lo adopta en vez
// de dejarlo con las pantallas colgadas hasta que caduque.

import * as device from '../device.js';
import { jugar } from './engine.js';
import { JUEGOS, juegoPorId } from './catalogo.mjs';
import { crearMenu } from './menu.mjs';
import { crearFinal } from './final.mjs';
import { recorrer } from './flujo.mjs';
import { registrar, paraGuardar, formatearPuntos } from './scores.mjs';
import * as almacen from './almacen.js';

const oyentes = new Set();

// Lo que la vista necesita para pintarse. `ultimo` es el lienzo vivo del motor: se muta en
// cada fotograma, así que no se copia.
const estado = { activo: false, titulo: '', juego: null, partida: null, ultimo: null };

let sesionActual = null;

// Tras cerrar una sesión, una respuesta atrasada de GET_STATE todavía puede decir que el
// teclado estaba en juego. Durante este rato no se adopta nada por esa vía; GAME:OPEN, que
// es el teclado pidiéndolo ahora mismo, vale siempre.
const SILENCIO_TRAS_SESION_MS = 2500;
let adoptarDesde = 0;

export function suscribir(fn) {
  oyentes.add(fn);
  return () => oyentes.delete(fn);
}

function avisar(tipo, datos) {
  for (const fn of oyentes) fn(tipo, datos);
}

export function leerEstado() {
  return estado;
}

export function activo() {
  return estado.activo;
}

// Una sesión del motor con los ganchos que la vista necesita para enseñarse.
function correr(juego, titulo, juegoId = null) {
  estado.titulo = titulo;
  estado.juego = juegoId;
  estado.partida = null;
  avisar('estado');
  const s = jugar(juego, {
    cerrar: false,
    alEntrar: (partida) => { estado.partida = partida; avisar('estado'); },
    alFotograma: (lienzo) => { estado.ultimo = lienzo; avisar('fotograma'); },
  });
  sesionActual = s;
  return s.resultado;
}

async function registrarPartida(juego, puntos, detalle = null) {
  await almacen.cargar();
  const nuevo = registrar(almacen.leer(juego.id), puntos, {
    mejorSiMenor: juego.mejorSiMenor,
    detalle: Array.isArray(detalle?.niveles) ? detalle.niveles : null,
  });
  const guardado = paraGuardar(nuevo);
  try {
    await almacen.guardar(juego.id, guardado);
  } catch (err) {
    avisar('error', 'No se pudo guardar la puntuación');
  }
  // El primer resultado de un juego es récord de oficio: no tiene gracia celebrarlo.
  const record = nuevo.record && nuevo.partidas > 1;
  avisar('partida', { juego, puntos, record, partidas: nuevo.partidas });
  return { record, partidas: nuevo.partidas, mejor: nuevo.mejor };
}

const OPS = {
  juegoPorId,
  menu: () => correr(crearMenu(JUEGOS), 'Menú de juegos'),
  partida: (juego) => correr(juego, juego.nombre, juego.id),
  async final(juego, puntos, detalle) {
    const r = await registrarPartida(juego, puntos, detalle);
    return correr(crearFinal({
      juego,
      textoPuntos: formatearPuntos(juego.unidad, puntos, { corto: true }),
      textoMejor: formatearPuntos(juego.unidad, r.mejor, { corto: true }),
      partidas: r.partidas,
      record: r.record,
    }), 'Fin de partida', juego.id);
  },
  cerrar: () => device.gameStop().catch(() => {}),
};

// Arranca una sesión: por un juego concreto, o por el menú si no se dice cuál. Rechaza si el
// teclado no deja entrar (sin conexión, o con un menú abierto en él).
export async function iniciar({ juegoId = null } = {}) {
  if (estado.activo) return;
  estado.activo = true;
  estado.ultimo = null;
  try {
    await recorrer(OPS, juegoId ? juegoPorId(juegoId) : null);
  } finally {
    estado.activo = false;
    estado.titulo = '';
    estado.juego = null;
    estado.partida = null;
    sesionActual = null;
    adoptarDesde = performance.now() + SILENCIO_TRAS_SESION_MS;
    avisar('estado');
  }
}

export function parar() {
  sesionActual?.cancelar();
}

// --- Lo que viene del teclado ---------------------------------------------------------

function desdeElTeclado({ pedido }) {
  if (estado.activo) return;
  if (!pedido && performance.now() < adoptarDesde) return;
  // La app no se mueve de la vista en que esté: ver los juegos en la pantalla es cosa del
  // jugador, que abre la pestaña si le apetece. Se juega igual con ella cerrada.
  iniciar().catch((err) => avisar('error', String(err?.message || err)));
}

let escuchando = false;

// Se llama una vez al arrancar la app.
export function escuchar() {
  if (escuchando) return;
  escuchando = true;

  const visto = (linea) => {
    // GAME:OPEN: el teclado acaba de entrar. STATE:MODE:GAME: ya estaba dentro cuando
    // preguntamos (la ventana se reabrió, o nos conectamos tarde).
    if (linea === 'GAME:OPEN') desdeElTeclado({ pedido: true });
    else if (linea === 'STATE:MODE:GAME') desdeElTeclado({ pedido: false });
  };
  // Las líneas de STATE: las puede recoger una petición en curso (GET_STATE) y entonces
  // llegan como respuesta y no como telemetría.
  device.on('telemetry', visto);
  device.on('response', visto);

  // El saludo trae el modo que había al conectar, pero con la ventana recién abierta por
  // un GAME:OPEN el saludo llegó antes que ella y dice lo que el teclado era entonces: se
  // vuelve a preguntar.
  const preguntarModo = () => setTimeout(() => device.send('GET_STATE'), 1200);
  device.on('connected', (info) => {
    if (info?.mode === 'GAME') desdeElTeclado({ pedido: false });
    preguntarModo();
  });
  // Y si la conexión ya estaba declarada cuando esto empieza a escuchar, el evento de
  // arriba pasó sin nadie: se pregunta igualmente.
  if (device.isConnected()) preguntarModo();
}
