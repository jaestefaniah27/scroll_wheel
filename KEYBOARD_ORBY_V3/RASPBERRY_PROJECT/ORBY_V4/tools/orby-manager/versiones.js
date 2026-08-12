'use strict';

// Leer y escribir las cuatro fuentes de versión del proyecto (firmware y app),
// más la versión instalada según el registro de Windows.
//
// Las escrituras son quirúrgicas (regex sobre la línea concreta) y no
// reserializaciones: reescribir package.json o tauri.conf.json con
// JSON.stringify reordena claves y destroza el diff que luego revisa el
// usuario en el panel. Por el mismo motivo se respetan los finales de línea
// que ya tenga cada fichero — este repo mezcla CRLF (los del firmware y los
// de OrbyGUI/package.json) con LF (los de src-tauri) y convertirlos de más
// ensuciaría un diff que debería tener solo el número de versión.
//
// Los bumps validan y calculan los tres ficheros ANTES de escribir ninguno:
// si uno de los tres patrones no encaja, no se toca nada. Un bump a medias
// dejaría el repo con dos versiones distintas y sin forma fácil de saber cuál
// manda.

const fs = require('fs');
const path = require('path');
const { execFile } = require('child_process');

const RAIZ = path.resolve(__dirname, '..', '..');

const RUTAS = {
  versionFw: path.join(RAIZ, 'include', 'orby_version.h'),
  packageJson: path.join(RAIZ, 'OrbyGUI', 'package.json'),
  compat: path.join(RAIZ, 'OrbyGUI', 'src', 'compat.js'),
  compatibilidad: path.join(RAIZ, 'docs', 'COMPATIBILIDAD.md'),
  tauriConf: path.join(RAIZ, 'OrbyGUI', 'src-tauri', 'tauri.conf.json'),
  cargoToml: path.join(RAIZ, 'OrbyGUI', 'src-tauri', 'Cargo.toml'),
};

// Rutas relativas con barras normales: es lo que enseña la UI, y en Windows
// path.relative() devuelve barras invertidas.
function relativa(rutaAbsoluta) {
  return path.relative(RAIZ, rutaAbsoluta).split(path.sep).join('/');
}

// El separador de línea real del fichero, para insertar texto nuevo sin
// convertir el resto del fichero al vuelo.
function detectarEol(contenido) {
  return contenido.includes('\r\n') ? '\r\n' : '\n';
}

function leerFirmware() {
  const contenido = fs.readFileSync(RUTAS.versionFw, 'utf8');

  const major = contenido.match(/#define ORBY_FW_MAJOR (\d+)/);
  if (!major) {
    throw new Error(
      `No se encontró "#define ORBY_FW_MAJOR <número>" en ${relativa(RUTAS.versionFw)}`
    );
  }
  const minor = contenido.match(/#define ORBY_FW_MINOR (\d+)/);
  if (!minor) {
    throw new Error(
      `No se encontró "#define ORBY_FW_MINOR <número>" en ${relativa(RUTAS.versionFw)}`
    );
  }

  const majorNum = Number(major[1]);
  const minorNum = Number(minor[1]);
  return { major: majorNum, minor: minorNum, version: `${majorNum}.${minorNum}` };
}

// La vía Tauri no está en todas las ramas: hasta que se fusione la migración,
// main no tiene src-tauri. Sin esta comprobación, el panel abierto en una rama
// sin esa carpeta enseñaría un error rojo en vez de decir simplemente que ahí
// no está.
function hayTauri() {
  return fs.existsSync(RUTAS.tauriConf) && fs.existsSync(RUTAS.cargoToml);
}

// La app lleva su número en TRES ficheros que tienen que decir lo mismo:
// tauri.conf.json (el que acaba en el instalador y en el latest.json que mira el
// actualizador), Cargo.toml (el que compila el binario) y package.json (el del
// frontend). Se devuelven los tres para poder avisar si se han separado, en vez
// de enseñar uno y callarse los otros: con tauri.conf.json y Cargo.toml en
// desacuerdo, el .exe dice una versión y el binario otra, y eso no se ve hasta
// tenerlo delante.
//
// La vía navegador no tiene número propio a propósito: sale del mismo
// `vite build` y se sirve como ficheros estáticos, sin nada que actualizar.
function leerTauri() {
  const conf = fs.readFileSync(RUTAS.tauriConf, 'utf8');
  const enConf = conf.match(/"version"\s*:\s*"([^"]+)"/);
  if (!enConf) {
    throw new Error(`No se encontró la clave "version" en ${relativa(RUTAS.tauriConf)}`);
  }
  const cargo = fs.readFileSync(RUTAS.cargoToml, 'utf8');
  const enCargo = bloquePackage(cargo).match(/^version\s*=\s*"([^"]+)"/m);
  if (!enCargo) {
    throw new Error(`No se encontró "version" dentro de [package] en ${relativa(RUTAS.cargoToml)}`);
  }
  const pkg = fs.readFileSync(RUTAS.packageJson, 'utf8');
  const enPkg = pkg.match(/"version"\s*:\s*"([^"]+)"/);
  if (!enPkg) {
    throw new Error(`No se encontró la clave "version" en ${relativa(RUTAS.packageJson)}`);
  }
  return { version: enConf[1], cargo: enCargo[1], pkg: enPkg[1] };
}

function leerFwRecomendado() {
  const contenido = fs.readFileSync(RUTAS.compat, 'utf8');
  const version = contenido.match(/FW_RECOMMENDED\s*=\s*'([^']+)'/);
  if (!version) {
    throw new Error(
      `No se encontró "FW_RECOMMENDED = '...'" en ${relativa(RUTAS.compat)}`
    );
  }
  return version[1];
}

// Busca en el registro la entrada de desinstalación de Orby. Nunca lanza:
// es una tarjeta más del panel, y un registro raro no puede tirar el resto.
async function leerInstalada() {
  if (process.platform !== 'win32') {
    return { version: null, error: 'solo en Windows' };
  }

  return new Promise((resolve) => {
    execFile(
      'reg.exe',
      ['query', 'HKCU\\Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall', '/s'],
      { encoding: 'utf8', maxBuffer: 16 * 1024 * 1024 },
      (error, stdout) => {
        if (error) {
          resolve({ version: null, error: error.message });
          return;
        }

        // Cada clave de desinstalación es un bloque separado por línea en
        // blanco. Solo nos interesan los pares DisplayName/DisplayVersion,
        // y esos son ASCII puro, así que sobreviven aunque el resto del
        // bloque venga con basura por culpa de la página de códigos de la
        // consola (reg.exe no respeta encoding:'utf8', solo Node lo intenta).
        const bloques = stdout.split(/\r?\n\s*\r?\n/);
        for (const bloque of bloques) {
          const nombre = bloque.match(/DisplayName\s+REG_SZ\s+(.+)/i);
          if (!nombre || !/orby/i.test(nombre[1])) continue;
          const version = bloque.match(/DisplayVersion\s+REG_SZ\s+(.+)/i);
          if (version) {
            resolve({ version: version[1].trim(), error: null });
            return;
          }
        }
        resolve({ version: null, error: 'no hay ninguna entrada de Orby en el registro' });
      }
    );
  });
}

// Inserta una fila nueva justo bajo la línea separadora de la tabla de
// docs/COMPATIBILIDAD.md, copiando la tercera columna de la fila que hoy
// encabeza la tabla (así no se inventa un texto de estilo distinto al del
// resto del documento).
function insertarFilaCompatibilidad(contenido, nueva, nota) {
  const eol = detectarEol(contenido);
  const lineas = contenido.split(/\r\n|\n/);

  const idxMarcador = lineas.findIndex((linea) => linea.trim() === '<!-- tabla:inicio -->');
  if (idxMarcador === -1) {
    throw new Error(
      `No se encontró el marcador <!-- tabla:inicio --> en ${relativa(RUTAS.compatibilidad)}`
    );
  }

  const idxSeparador = idxMarcador + 2;
  const separador = (lineas[idxSeparador] || '').trim();
  if (!/^\|[-:\s|]+\|$/.test(separador)) {
    throw new Error(
      `No se encontró la línea separadora de la tabla (|---|---|---|) tras <!-- tabla:inicio --> en ${relativa(RUTAS.compatibilidad)}`
    );
  }

  const idxPrimeraFila = idxMarcador + 3;
  const primeraFila = lineas[idxPrimeraFila] || '';
  // Columna = cualquier secuencia de caracteres que no sea "|" o "\", o un
  // par escapado "\X" (para no partir por la mitad un "\|" dentro del texto,
  // que es justo lo que hace la fila de la versión 4.5 de hoy).
  const columna = '((?:\\\\.|[^|\\\\])*)';
  const patronFila = new RegExp(`^\\|${columna}\\|${columna}\\|${columna}\\|\\s*$`);
  const coincidencia = primeraFila.match(patronFila);
  if (!coincidencia) {
    throw new Error(
      `No se pudo leer la primera fila de la tabla de compatibilidad en ${relativa(RUTAS.compatibilidad)}: "${primeraFila}"`
    );
  }

  const terceraColumna = coincidencia[3].trim();
  const notaLimpia = nota && nota.trim() ? nota.trim() : 'Pendiente de describir';
  const notaEscapada = notaLimpia.replace(/\|/g, '\\|');
  const filaNueva = `| **${nueva}** | ${notaEscapada} | ${terceraColumna} |`;

  lineas.splice(idxPrimeraFila, 0, filaNueva);
  return lineas.join(eol);
}

function bumpFirmware(tipo, nota) {
  if (tipo !== 'major' && tipo !== 'minor') {
    throw new Error(`Tipo de bump de firmware inválido: "${tipo}" (solo "major" o "minor")`);
  }

  const actual = leerFirmware();
  let major = actual.major;
  let minor = actual.minor;
  if (tipo === 'minor') {
    minor += 1;
  } else {
    major += 1;
    minor = 0;
  }
  const nueva = `${major}.${minor}`;

  // 1. include/orby_version.h — se calcula pero no se escribe todavía.
  const hContenido = fs.readFileSync(RUTAS.versionFw, 'utf8');
  const patronMajor = /#define ORBY_FW_MAJOR \d+/;
  const patronMinor = /#define ORBY_FW_MINOR \d+/;
  if (!patronMajor.test(hContenido) || !patronMinor.test(hContenido)) {
    throw new Error(
      `No se encontraron los "#define ORBY_FW_MAJOR/MINOR" en ${relativa(RUTAS.versionFw)}`
    );
  }
  const hNuevo = hContenido
    .replace(patronMajor, `#define ORBY_FW_MAJOR ${major}`)
    .replace(patronMinor, `#define ORBY_FW_MINOR ${minor}`);

  // 2. OrbyGUI/src/compat.js
  const compatContenido = fs.readFileSync(RUTAS.compat, 'utf8');
  const patronRecomendado = /FW_RECOMMENDED\s*=\s*'[^']+'/;
  if (!patronRecomendado.test(compatContenido)) {
    throw new Error(
      `No se encontró "FW_RECOMMENDED = '...'" en ${relativa(RUTAS.compat)}`
    );
  }
  const compatNuevo = compatContenido.replace(patronRecomendado, `FW_RECOMMENDED = '${nueva}'`);

  // 3. docs/COMPATIBILIDAD.md — lanza si la tabla no tiene la forma esperada.
  const mdContenido = fs.readFileSync(RUTAS.compatibilidad, 'utf8');
  const mdNuevo = insertarFilaCompatibilidad(mdContenido, nueva, nota);

  // Las tres sustituciones encajan: ahora sí se escribe, las tres o ninguna.
  fs.writeFileSync(RUTAS.versionFw, hNuevo, 'utf8');
  fs.writeFileSync(RUTAS.compat, compatNuevo, 'utf8');
  fs.writeFileSync(RUTAS.compatibilidad, mdNuevo, 'utf8');

  return {
    anterior: actual.version,
    nueva,
    ficheros: [
      relativa(RUTAS.versionFw),
      relativa(RUTAS.compat),
      relativa(RUTAS.compatibilidad),
    ],
  };
}

// Los límites del bloque [package] del Cargo.toml. Más abajo en el mismo
// fichero hay una clave "version" por cada dependencia, y tocar una de esas
// rompe el build: nada que lea o escriba la versión del paquete puede mirar
// fuera de aquí.
function limitesPackage(contenido) {
  const inicio = contenido.indexOf('[package]');
  if (inicio === -1) {
    throw new Error(`No se encontró la sección [package] en ${relativa(RUTAS.cargoToml)}`);
  }
  const desde = inicio + '[package]'.length;
  // Siguiente cabecera de sección ("[workspace]" aquí), que marca el final del
  // bloque. Si no hay ninguna, el bloque llega hasta el final del fichero.
  const siguiente = contenido.slice(desde).match(/\r?\n\[/);
  return { inicio, fin: siguiente ? desde + siguiente.index : contenido.length };
}

function bloquePackage(contenido) {
  const { inicio, fin } = limitesPackage(contenido);
  return contenido.slice(inicio, fin);
}

function sustituirVersionCargoPackage(contenido, nueva) {
  const { inicio: inicioPackage, fin: finPackage } = limitesPackage(contenido);
  const bloque = contenido.slice(inicioPackage, finPackage);
  const patronVersion = /^version\s*=\s*"[^"]+"/m;
  if (!patronVersion.test(bloque)) {
    throw new Error(
      `No se encontró "version = "..."" dentro de [package] en ${relativa(RUTAS.cargoToml)}`
    );
  }
  const bloqueNuevo = bloque.replace(patronVersion, `version = "${nueva}"`);

  return contenido.slice(0, inicioPackage) + bloqueNuevo + contenido.slice(finPackage);
}

function subirSemver(version, tipo, donde) {
  if (tipo !== 'major' && tipo !== 'minor' && tipo !== 'patch') {
    throw new Error(`Tipo de bump inválido: "${tipo}" (solo "major", "minor" o "patch")`);
  }
  const partes = version.split('.').map(Number);
  if (partes.length !== 3 || partes.some((n) => Number.isNaN(n))) {
    throw new Error(`La versión en ${donde} no tiene forma x.y.z: "${version}"`);
  }
  let [major, minor, patch] = partes;
  if (tipo === 'major') { major += 1; minor = 0; patch = 0; }
  else if (tipo === 'minor') { minor += 1; patch = 0; }
  else { patch += 1; }
  return `${major}.${minor}.${patch}`;
}

// Los tres ficheros a la vez, o ninguno. Si se sube uno solo, el instalador y el
// binario dicen versiones distintas y nadie se entera hasta tener el .exe delante.
function bumpTauri(tipo) {
  const actual = leerTauri();
  if (actual.version !== actual.cargo || actual.version !== actual.pkg) {
    throw new Error(
      `tauri.conf.json dice ${actual.version}, Cargo.toml dice ${actual.cargo} y ` +
      `package.json dice ${actual.pkg}: iguálalos a mano antes de subir la versión, ` +
      'o el bump elegiría por ti cuál era la buena.'
    );
  }
  const nueva = subirSemver(actual.version, tipo, relativa(RUTAS.tauriConf));

  const tauriContenido = fs.readFileSync(RUTAS.tauriConf, 'utf8');
  const patronVersionJson = /"version"\s*:\s*"[^"]+"/;
  if (!patronVersionJson.test(tauriContenido)) {
    throw new Error(`No se encontró la clave "version" en ${relativa(RUTAS.tauriConf)}`);
  }
  const tauriNuevo = tauriContenido.replace(patronVersionJson, `"version": "${nueva}"`);

  const cargoContenido = fs.readFileSync(RUTAS.cargoToml, 'utf8');
  const cargoNuevo = sustituirVersionCargoPackage(cargoContenido, nueva);

  const pkgContenido = fs.readFileSync(RUTAS.packageJson, 'utf8');
  if (!patronVersionJson.test(pkgContenido)) {
    throw new Error(`No se encontró la clave "version" en ${relativa(RUTAS.packageJson)}`);
  }
  const pkgNuevo = pkgContenido.replace(patronVersionJson, `"version": "${nueva}"`);

  // Las tres sustituciones encajan: ahora sí se escribe. Cargo.lock no se toca:
  // cargo lo pone al día solo en el siguiente build.
  fs.writeFileSync(RUTAS.tauriConf, tauriNuevo, 'utf8');
  fs.writeFileSync(RUTAS.cargoToml, cargoNuevo, 'utf8');
  fs.writeFileSync(RUTAS.packageJson, pkgNuevo, 'utf8');

  return {
    anterior: actual.version,
    nueva,
    ficheros: [relativa(RUTAS.tauriConf), relativa(RUTAS.cargoToml), relativa(RUTAS.packageJson)],
  };
}

module.exports = {
  RAIZ,
  RUTAS,
  leerFirmware,
  hayTauri,
  leerTauri,
  leerFwRecomendado,
  leerInstalada,
  bumpFirmware,
  bumpTauri,
};
