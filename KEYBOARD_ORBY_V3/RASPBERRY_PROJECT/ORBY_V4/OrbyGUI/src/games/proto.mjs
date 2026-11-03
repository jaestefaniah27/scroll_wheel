// El protocolo del modo juego (firmware 4.29, bandera GAMES), sin tocar el puerto.
//
// Va aparte de engine.js para poder probarlo sin teclado: aquí solo hay cuentas con
// bytes y con líneas de texto, nada de window.orby.
//
// Un fotograma son las diez pantallas de 72x40 (360 bytes cada una, 5 páginas de 72
// columnas, el mismo formato que los iconos). Lo que viaja es una cabecera de texto
// y detrás, sin separador, los bytes crudos de las pantallas que han cambiado:
//
//   GAME_BIN:<máscara>:<n>\n  +  360 bytes por cada bit de la máscara
//
// La máscara lleva un bit por pantalla (bit 0 = pantalla 1). Mandar solo lo que
// cambia es lo que más fotogramas por segundo da: un juego suele mover una o dos
// pantallas a la vez, y el coste es casi proporcional a los bytes.

export const PANTALLAS = 10;
export const FB = 360;

// Un lienzo es un fotograma entero: diez búferes, uno por pantalla.
export function lienzo() {
  return Array.from({ length: PANTALLAS }, () => new Uint8Array(FB));
}

export function copiarLienzo(destino, origen) {
  for (let i = 0; i < PANTALLAS; i++) destino[i].set(origen[i]);
}

export function limpiarLienzo(l) {
  for (const p of l) p.fill(0);
}

const CABECERA = new TextEncoder();

// Lo que hay que escribir en el puerto para pasar de `anterior` a `actual`, o null si
// no ha cambiado nada. Con `anterior` a null se mandan las diez, que es lo que hace
// falta al empezar (el teclado entra con las pantallas en negro, pero no se fía uno).
export function armarFotograma(anterior, actual, n) {
  let mascara = 0;
  let cambiadas = 0;
  for (let i = 0; i < PANTALLAS; i++) {
    if (!anterior || !iguales(anterior[i], actual[i])) {
      mascara |= 1 << i;
      cambiadas++;
    }
  }
  if (!mascara) return null;

  const cabecera = CABECERA.encode(`GAME_BIN:${mascara}:${n}\n`);
  const bytes = new Uint8Array(cabecera.length + cambiadas * FB);
  bytes.set(cabecera, 0);
  let pos = cabecera.length;
  for (let i = 0; i < PANTALLAS; i++) {
    if ((mascara >> i) & 1) {
      bytes.set(actual[i], pos);
      pos += FB;
    }
  }
  return { bytes, mascara, pantallas: cambiadas };
}

function iguales(a, b) {
  for (let i = 0; i < FB; i++) if (a[i] !== b[i]) return false;
  return true;
}

// Las teclas del teclado se numeran del 1 al 12 en KEY_EV y las pantallas del 1 al 10.
// Las nueve primeras teclas llevan su pantalla debajo, la tecla 11 lleva la 10, y la
// 10 (la de menú) y la 12 (SUPER) no tienen pantalla. Es el key_to_oled[] de main.cpp.
export function pantallaDeTecla(tecla) {
  if (tecla >= 1 && tecla <= 9) return tecla;
  return tecla === 11 ? 10 : null;
}

export function teclaDePantalla(pantalla) {
  if (pantalla >= 1 && pantalla <= 9) return pantalla;
  return pantalla === 10 ? 11 : null;
}

function microsegundos(campo) {
  if (campo === undefined || campo === '') return null;
  const v = Number(campo);
  return Number.isFinite(v) ? v : null;
}

// Milisegundos entre dos instantes del reloj del teclado. Es un contador de 32 bits en µs
// que da la vuelta cada 71 minutos: la resta sin signo sigue valiendo a través de la vuelta.
export function msEntre(desdeUs, hastaUs) {
  return ((hastaUs - desdeUs) >>> 0) / 1000;
}

// Una línea del teclado vista desde un juego. Lo que no le importa a un juego (perfiles,
// espejo, respuestas a peticiones) sale como null.
export function leerEvento(linea) {
  if (linea.startsWith('KEY_EV:')) {
    // En juego (firmware 4.29) lleva detrás el µs de la pulsación en el reloj del teclado.
    // Un firmware que no lo manda deja `us` en null y el juego se apaña con el reloj del PC.
    const [, tecla, estado, us] = linea.split(':');
    const t = Number(tecla);
    return { tipo: 'tecla', tecla: t, pantalla: pantallaDeTecla(t), pulsada: estado === '1', us: microsegundos(us) };
  }
  if (linea.startsWith('ENC_SW:')) {
    const [, mando, estado] = linea.split(':');
    return { tipo: 'mando-boton', mando: Number(mando), pulsado: estado === '1' };
  }
  if (linea.startsWith('ENC:')) {
    const [, mando, delta] = linea.split(':');
    return { tipo: 'mando', mando: Number(mando), delta: Number(delta) };
  }
  if (linea.startsWith('WHEEL:')) {
    // Si dos líneas se pegaron ("WHEEL:5:WHEEL:-3:2100") el ángulo no se puede leer;
    // wheel-dial.js se ha topado con lo mismo. Se descarta antes que inventarlo.
    const partes = linea.split(':');
    if (partes.length !== 3) return null;
    const delta = Number(partes[1]);
    const angulo = Number(partes[2]);
    if (!Number.isFinite(delta) || !Number.isFinite(angulo)) return null;
    return { tipo: 'rueda', delta, angulo };
  }
  if (linea.startsWith('GAME:SHOWN:')) {
    // GAME:SHOWN:<n>[:<µs>]: el µs es cuándo acabó de pintarse, en el mismo reloj que KEY_EV.
    const [n, us] = linea.slice('GAME:SHOWN:'.length).split(':');
    return { tipo: 'acuse', n: Number(n), us: microsegundos(us) };
  }
  // El teclado sale del juego solo: porque la app calló (GAME:TIMEOUT) o porque algo lo
  // sacó a un menú (MODE:MENU). El gesto de los dos mandos ya no lo hace en juego (los dos
  // pulsados son una entrada más), pero un firmware anterior sí. Para el juego es lo mismo.
  if (linea === 'GAME:TIMEOUT') return { tipo: 'fin', motivo: 'timeout' };
  if (linea.startsWith('MODE:')) return { tipo: 'fin', motivo: 'menu' };
  return null;
}

// Cuántos fotogramas se permiten sin acuse a la vez. Con uno solo, cada fotograma
// espera el viaje de ida y vuelta entero; con más, el puerto no se queda parado. Más
// de unos pocos no sube la tasa y sí la latencia con la que se ve lo que se ha hecho.
export const EN_VUELO = 3;

// Si se puede mandar otro fotograma: `enviado` es el número del último que se mandó y
// `acusado` el último que el teclado ha confirmado pintado (-1 si ninguno).
export function puedeEnviar(enviado, acusado, enVuelo = EN_VUELO) {
  return enviado - acusado < enVuelo;
}
