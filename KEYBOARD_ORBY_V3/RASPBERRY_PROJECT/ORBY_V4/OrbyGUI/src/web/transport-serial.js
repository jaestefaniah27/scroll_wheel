// Web Serial: el mismo CDC que abre electron/serial.js, pero desde el navegador.
//
// El teclado no necesita nada nuevo. Expone un CDC ACM y habla por líneas de texto,
// así que aquí basta con abrir el puerto, ponerle las señales y trocear el flujo.
// El protocolo, la cola de peticiones de device.js y las vistas se quedan igual.

import { splitLines } from './split-lines.mjs';

// El VID de los ejemplos de TinyUSB, en decimal: Web Serial pide números, no las
// cadenas hexadecimales que devuelve serialport. Debe seguir a KNOWN_IDS de
// electron/serial.js. No se filtra por PID a propósito: el firmware lo sube cada vez
// que cambia la estructura de un informe HID, y filtrarlo dejaría fuera a los
// teclados con firmware anterior.
export const ORBY_USB_VID = 0xCafe;

const BAUD_RATE = 115200;

const oyentes = new Map();
let puerto = null;
let escritor = null;
let lector = null;
let identidad = null;
let conectado = false;
let buscando = false;
let cerrandoAdrede = false;
let reintento = null;

export function disponible() {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

export function on(evento, cb) {
  if (!oyentes.has(evento)) oyentes.set(evento, new Set());
  oyentes.get(evento).add(cb);
}

function emitir(evento, dato) {
  const set = oyentes.get(evento);
  if (set) for (const cb of set) cb(dato);
}

export function estado() {
  if (conectado) return 'connected';
  return buscando ? 'searching' : 'disconnected';
}

export function info() {
  return identidad;
}

// --- Identidad -------------------------------------------------------------

// Misma forma que _parseDeviceInfo en electron/serial.js: compat.js y macros-store.js
// leen estas claves en minúscula, y una identidad con otra forma haría que la app
// tratara a un teclado 4.3 como si fuera anterior a las páginas y las huellas.
function parsearIdentidad(linea) {
  const out = { raw: linea, port: 'Web Serial' };
  for (const parte of linea.split(':')) {
    if (!parte.includes('=')) continue;
    const [clave, valor] = parte.split('=');
    out[clave.toLowerCase()] = valor;
  }
  return out;
}

// --- Apertura --------------------------------------------------------------

async function abrir(p) {
  await p.open({ baudRate: BAUD_RATE });
  // TinyUSB no acepta lo que le mandamos si el host no ha levantado DTR: sin esto
  // el teclado nunca ve el ACK y la presentación no llega jamás.
  try { await p.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch { /* algún backend no las soporta */ }

  puerto = p;
  escritor = p.writable.getWriter();
  cerrandoAdrede = false;
  bucleLectura();

  // El primer ACK se pierde a menudo si el CDC acaba de enumerarse, igual que en
  // la app de escritorio: se insiste unas cuantas veces hasta que se presenta.
  for (let i = 0; i < 5 && !conectado; i++) {
    await enviar('ACK');
    await new Promise((r) => setTimeout(r, 400 + i * 200));
  }
  return conectado;
}

async function bucleLectura() {
  const decodificador = new TextDecoder();
  let resto = '';

  try {
    lector = puerto.readable.getReader();
    for (;;) {
      const { value, done } = await lector.read();
      if (done) break;
      const trozo = decodificador.decode(value, { stream: true });
      const r = splitLines(resto, trozo);
      resto = r.resto;
      for (const linea of r.lineas) manejarLinea(linea);
    }
  } catch (err) {
    // Desenchufar el teclado hace que la lectura reviente con NetworkError. No es
    // un fallo que contarle al usuario: es la vía normal por la que nos enteramos
    // de que se ha ido, porque el evento 'disconnect' de navigator.serial no
    // siempre llega antes.
    if (!cerrandoAdrede) console.warn('[web] la lectura del puerto se ha cortado:', err.message);
  } finally {
    try { lector?.releaseLock(); } catch { /* ya liberado */ }
    lector = null;
    if (!cerrandoAdrede) await caida();
  }
}

function manejarLinea(linea) {
  if (linea.startsWith('ORBY_V4:')) {
    // El teclado repite su presentación cada vez que se le manda un ACK, así que
    // esta línea llega también con la conexión ya hecha. Anunciarla otra vez haría
    // a la app resincronizarse entera.
    const yaConectado = conectado && identidad?.raw === linea;
    identidad = parsearIdentidad(linea);
    conectado = true;
    buscando = false;
    if (!yaConectado) emitir('connected', identidad);
    return;
  }
  emitir('data', linea);
}

async function caida() {
  if (!conectado && !puerto) return;
  conectado = false;
  identidad = null;
  try { escritor?.releaseLock(); } catch { /* ya liberado */ }
  escritor = null;
  try { await puerto?.close(); } catch { /* ya cerrado, o se lo llevó el desenchufe */ }
  puerto = null;
  emitir('disconnected');
  reintentar();
}

// --- Reconexión ------------------------------------------------------------
// El permiso lo concede el usuario una vez y se queda concedido para ese puerto,
// así que volver a abrirlo tras un desenchufe NO necesita otro gesto: se puede
// insistir solo, igual que hace el escaneo de la app de escritorio.
const REINTENTO_MS = 3000;

function reintentar() {
  if (reintento) return;
  buscando = true;
  emitir('searching');
  reintento = setInterval(async () => {
    if (conectado) { clearInterval(reintento); reintento = null; return; }
    const p = await puertoAutorizado();
    if (!p) return;
    try { await abrir(p); } catch { /* sigue sin estar listo */ }
  }, REINTENTO_MS);
}

async function puertoAutorizado() {
  const puertos = await navigator.serial.getPorts();
  return puertos.find((p) => p.getInfo?.().usbVendorId === ORBY_USB_VID) || puertos[0] || null;
}

// --- API pública -----------------------------------------------------------

export async function enviar(comando) {
  if (!escritor) return;
  try {
    await escritor.write(new TextEncoder().encode(`${comando}\n`));
  } catch (err) {
    emitir('error', `No se pudo escribir en el puerto: ${err.message}`);
  }
}

// Pide un puerto nuevo. Solo se puede llamar desde el manejador de un clic: Chrome
// rechaza requestPort() fuera de un gesto del usuario.
export async function pedirPuerto() {
  try {
    const p = await navigator.serial.requestPort({ filters: [{ usbVendorId: ORBY_USB_VID }] });
    return await abrir(p);
  } catch (err) {
    // NotFoundError = el usuario cerró el diálogo sin elegir nada. No es un error
    // que merezca un aviso rojo.
    if (err.name === 'NotFoundError') return false;
    // En Windows solo un proceso puede tener el COM abierto: con OrbyGUI de
    // escritorio corriendo, esto falla siempre y el motivo no se adivina.
    emitir('error', err.name === 'NetworkError'
      ? 'No se pudo abrir el puerto. Si tienes OrbyGUI de escritorio abierta, ciérrala: solo un programa puede usar el teclado a la vez.'
      : `No se pudo abrir el puerto: ${err.message}`);
    return false;
  }
}

// Intenta abrir un puerto ya autorizado en una visita anterior. No necesita gesto,
// así que es lo que se llama al cargar la página.
let oyenteDesconexion = false;

export async function arrancar() {
  if (!disponible()) return false;

  // Una sola vez: reconectar() vuelve a pasar por aquí, y con un oyente por llamada
  // un desenchufe acabaría disparando caida() tantas veces como reconexiones hubiera.
  if (!oyenteDesconexion) {
    oyenteDesconexion = true;
    navigator.serial.addEventListener('disconnect', (ev) => {
      if (ev.target === puerto) caida();
    });
  }

  const p = await puertoAutorizado();
  if (!p) return false;
  try {
    return await abrir(p);
  } catch {
    return false;
  }
}

export async function reconectar() {
  cerrandoAdrede = true;
  try { lector?.cancel(); } catch { /* nada que cancelar */ }
  await caida();
  cerrandoAdrede = false;
  await arrancar();
}
