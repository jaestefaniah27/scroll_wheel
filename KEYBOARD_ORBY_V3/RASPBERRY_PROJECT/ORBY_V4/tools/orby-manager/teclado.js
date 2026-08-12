// Lee el firmware del teclado Orby conectado por USB CDC, sin depender de que
// OrbyGUI esté cerrada ni de tener el puerto para nosotros: es una consulta de
// usar y tirar (abre, pregunta, cierra), pensada para el panel de versiones.

'use strict';

const path = require('path');

const RAIZ = path.resolve(__dirname, '..', '..');

// `serialport` es la única dependencia del panel, y la declara en su propio
// package.json desde que OrbyGUI dejó de tenerla: la app abre el puerto desde
// Rust y no necesita el binding de Node, así que ya no hay copia prestada de la
// que tirar. Se sigue intentando la de OrbyGUI primero por si queda instalada de
// antes, y así un `npm install` olvidado en esta carpeta no rompe la tarjeta.
function cargarSerialPort() {
  for (const donde of [__dirname, path.join(RAIZ, 'OrbyGUI')]) {
    try {
      return require(path.join(donde, 'node_modules', 'serialport'));
    } catch { /* se prueba el siguiente */ }
  }
  return null;
}

function vacio(extra) {
  return {
    disponible: false,
    version: null,
    puerto: null,
    ocupado: false,
    info: null,
    raw: null,
    error: null,
    ...extra,
  };
}

// Trocea el handshake `ORBY_V4:FW=4.5:KEYS=12:...` en pares clave/valor, igual
// que `parsear_info` en src-tauri/src/serial.rs: el primer campo es el nombre
// del dispositivo, no una clave, así que se descarta.
function parsearHandshake(linea) {
  const partes = linea.split(':');
  const info = {};
  for (const parte of partes.slice(1)) {
    if (!parte.includes('=')) continue;
    const [clave, valor] = parte.split('=');
    info[clave.toLowerCase()] = valor;
  }
  return info;
}

async function leerTeclado() {
  const sp = cargarSerialPort();
  if (!sp) {
    return vacio({
      error: 'No se encuentra el módulo serialport (falta "npm install" en tools/orby-manager)',
    });
  }
  const { SerialPort } = sp;

  let puertos;
  try {
    puertos = await SerialPort.list();
  } catch (err) {
    return vacio({ error: `No se pudieron listar los puertos serie: ${err.message}` });
  }

  // Mismo criterio que el escaneo de OrbyGUI (src-tauri/src/serial.rs):
  // VID cafe con cualquier PID, porque el firmware lo sube cada vez que cambia
  // un informe HID, y como red el fabricante conteniendo "orby".
  const candidato = puertos.find((p) => {
    const vid = p.vendorId?.toLowerCase();
    if (vid === 'cafe') return true;
    return p.manufacturer && p.manufacturer.toLowerCase().includes('orby');
  });

  if (!candidato) {
    return vacio({ error: 'No se ve ningún teclado Orby en los puertos COM' });
  }

  let puerto = null;
  let handshakeLinea = null;
  let buffer = '';
  let temporizador = null;

  try {
    await new Promise((resolve) => {
      puerto = new SerialPort({ path: candidato.path, baudRate: 115200, autoOpen: false });

      puerto.open((err) => {
        if (err) {
          resolve({ errorApertura: err });
          return;
        }

        puerto.on('data', (data) => {
          buffer += data.toString();
          const lineas = buffer.split(/\r?\n/);
          buffer = lineas.pop() || '';
          for (const linea of lineas) {
            const recortada = linea.trim();
            if (recortada.startsWith('ORBY_V4:') && !handshakeLinea) {
              handshakeLinea = recortada;
              resolve({});
            }
          }
        });

        puerto.write('ACK\n');

        temporizador = setTimeout(() => resolve({}), 1500);
      });
    }).then((resultado) => {
      if (resultado && resultado.errorApertura) throw resultado.errorApertura;
    });
  } catch (err) {
    clearTimeout(temporizador);
    const mensaje = String(err.message || err);
    // "Access denied" (Windows) / "Permission denied" (otros): el caso normal
    // es que OrbyGUI de escritorio tiene el puerto abierto. No es un fallo de
    // este panel, así que se marca `ocupado` para que la UI lo pinte como aviso.
    const cogido = /access denied|permission denied/i.test(mensaje);
    return vacio({
      ocupado: cogido,
      error: cogido
        ? 'El puerto lo tiene cogido otra aplicación, cierra OrbyGUI'
        : `No se pudo abrir el puerto ${candidato.path}: ${mensaje}`,
    });
  } finally {
    clearTimeout(temporizador);
    // Se cierra siempre, con handshake o sin él: dejar el puerto cogido aquí le
    // impediría a OrbyGUI conectar después.
    if (puerto && puerto.isOpen) {
      await new Promise((resolve) => puerto.close(() => resolve()));
    }
  }

  if (!handshakeLinea) {
    return vacio({ error: `El teclado en ${candidato.path} no ha contestado a tiempo` });
  }

  const info = parsearHandshake(handshakeLinea);
  return {
    disponible: true,
    version: info.fw ?? null,
    puerto: candidato.path,
    ocupado: false,
    info,
    raw: handshakeLinea,
    error: null,
  };
}

module.exports = { leerTeclado };
