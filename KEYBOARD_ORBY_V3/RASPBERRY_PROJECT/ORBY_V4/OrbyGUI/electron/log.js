// Registro en disco de lo que le pasa a la conexión con el teclado.
//
// En desarrollo basta con el terminal, pero la app instalada no tiene consola
// que mirar: cuando alguien dice "no me detecta el teclado", no había ni una
// línea con la que saber si el puerto no se abrió, si el teclado no se presentó
// o si la conexión se murió sin avisar. Esto deja un rastro que se puede pedir.
//
// Solo el ciclo de vida de la conexión: ni la telemetría ni los comandos, que
// serían megas al día y aquí no aportan.

const fs = require('fs');
const path = require('path');

// Fuera de Electron (las pruebas sueltas de tools/ cargan serial.js en Node a
// pelo) no hay carpeta de datos de usuario: entonces esto es solo la consola.
let app = null;
try { ({ app } = require('electron')); } catch { /* Node pelado */ }

// Se parte en dos ficheros de este tamaño: el actual y el anterior. Con eso
// sobra para ver qué pasó en el último arranque sin dejar el disco lleno.
const MAX_BYTES = 256 * 1024;

let file = null;

function target() {
  if (!file && app) file = path.join(app.getPath('userData'), 'orby.log');
  return file;
}

function rotate() {
  try {
    if (fs.statSync(target()).size < MAX_BYTES) return;
    try { fs.unlinkSync(`${target()}.1`); } catch { /* no había anterior */ }
    fs.renameSync(target(), `${target()}.1`);
  } catch { /* no existe todavía, o no se puede rotar: se sigue escribiendo */ }
}

function write(line) {
  if (!target()) return;
  const stamp = new Date().toISOString();
  try {
    rotate();
    fs.appendFileSync(target(), `${stamp} ${line}\n`, 'utf8');
  } catch { /* si no se puede escribir el registro, no se rompe nada por ello */ }
}

// Sale también por la consola: en desarrollo es donde se mira.
function log(...parts) {
  const line = parts.join(' ');
  console.log(line);
  write(line);
}

module.exports = { log, file: target };
