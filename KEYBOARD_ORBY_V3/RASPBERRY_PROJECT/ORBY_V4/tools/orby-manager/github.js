'use strict';

// API REST de GitHub para el pipeline de releases: crear la release, subir
// assets con progreso y leer el estado de lo ya publicado. Sin dependencias
// externas (regla del gestor): todo por `https` nativo, incluida la subida
// de ficheros grandes, que necesita un stream para no cargar el .exe o el
// .uf2 entero en memoria y para poder informar del progreso por bytes.

const https = require('https');
const fs = require('fs');
const path = require('path');

const REPO = 'jaestefaniah27/scroll_wheel';
const API_HOST = 'api.github.com';

// El token no se guarda en disco ni se registra: vive solo en el entorno del
// proceso que lanzó el gestor, y esta es la única función que lo lee.
function hayToken() {
  return !!(process.env.GH_TOKEN && process.env.GH_TOKEN.trim());
}

function cabeceras() {
  const h = {
    Accept: 'application/vnd.github+json',
    'X-GitHub-Api-Version': '2022-11-28',
    // GitHub rechaza las peticiones sin User-Agent con un 403 que no explica
    // por qué, así que va siempre, haya token o no.
    'User-Agent': 'orby-manager',
  };
  if (hayToken()) h.Authorization = `Bearer ${process.env.GH_TOKEN}`;
  return h;
}

function intentarJSON(texto) {
  try {
    return JSON.parse(texto);
  } catch {
    return null;
  }
}

// Construye el error de una respuesta que no es 2xx sin volcar cabeceras
// (donde iría el token): solo el mensaje que manda GitHub en el cuerpo, o el
// cuerpo crudo recortado si no es JSON.
function errorDeRespuesta(status, texto, cuerpoJSON) {
  const mensaje = (cuerpoJSON && cuerpoJSON.message) || (texto ? texto.slice(0, 300) : 'sin cuerpo');
  const err = new Error(`GitHub ${status}: ${mensaje}`);
  err.status = status;
  return err;
}

// `ruta` admite tanto un camino relativo a la API ('/repos/...') como una URL
// completa ya resuelta (el `url` de una release, por ejemplo).
function pedir(metodo, ruta, cuerpo) {
  return new Promise((resolve, reject) => {
    const url = ruta.startsWith('http') ? new URL(ruta) : new URL(ruta, `https://${API_HOST}`);
    const datos = cuerpo != null ? JSON.stringify(cuerpo) : null;
    const headers = cabeceras();
    if (datos) {
      headers['Content-Type'] = 'application/json';
      headers['Content-Length'] = Buffer.byteLength(datos);
    }

    const req = https.request(
      { hostname: url.hostname, path: url.pathname + url.search, method: metodo, headers },
      (res) => {
        const trozos = [];
        res.on('data', (d) => trozos.push(d));
        res.on('end', () => {
          const texto = Buffer.concat(trozos).toString('utf8');
          const cuerpoResp = texto ? intentarJSON(texto) : null;
          if (res.statusCode < 200 || res.statusCode >= 300) {
            reject(errorDeRespuesta(res.statusCode, texto, cuerpoResp));
            return;
          }
          resolve(cuerpoResp);
        });
      }
    );
    req.on('error', reject);
    if (datos) req.write(datos);
    req.end();
  });
}

async function listarReleases(porPagina = 20) {
  return pedir('GET', `/repos/${REPO}/releases?per_page=${porPagina}`);
}

// Se usa el endpoint por tag en vez de filtrar el listado porque es el único
// que da un 404 limpio cuando la etiqueta no existe todavía (paso de
// comprobaciones del pipeline, antes de publicar).
async function obtenerReleasePorTag(tag) {
  try {
    return await pedir('GET', `/repos/${REPO}/releases/tags/${encodeURIComponent(tag)}`);
  } catch (err) {
    if (err.status === 404) return null;
    throw err;
  }
}

// `prerelease` llega tal cual desde quien orquesta el pipeline: el paso de
// firmware siempre lo manda a `true` porque una release de firmware que no
// sea prerelease rompe el actualizador de la app (firmware.rs filtra por ese
// prefijo de tag, no por el flag, así que el daño real es que
// GET /repos/.../releases/latest —que sí mira el flag— empezaría a devolver
// firmware en vez de la última versión de la app). Aquí no se fuerza nada:
// decidir el valor es responsabilidad de quien llama.
async function crearRelease({ tag, nombre, notas, prerelease }) {
  if (!hayToken()) throw new Error('Falta GH_TOKEN en el entorno');
  return pedir('POST', `/repos/${REPO}/releases`, {
    tag_name: tag,
    name: nombre,
    body: notas,
    prerelease: !!prerelease,
    draft: false,
  });
}

// `uploadUrl` viene con plantilla RFC 6570 ('.../assets{?name,label}'): no es
// una URL usable tal cual, hay que cortarla en la '{' y montar la query.
function subirAsset(uploadUrl, ficheroPath, onProgreso) {
  if (!hayToken()) throw new Error('Falta GH_TOKEN en el entorno');
  return new Promise((resolve, reject) => {
    const base = uploadUrl.split('{')[0];
    const nombreAsset = encodeURIComponent(path.basename(ficheroPath));
    const url = new URL(`${base}?name=${nombreAsset}`);
    const total = fs.statSync(ficheroPath).size;

    const headers = cabeceras();
    headers['Content-Type'] = 'application/octet-stream';
    headers['Content-Length'] = total;

    const req = https.request(
      { hostname: url.hostname, path: url.pathname + url.search, method: 'POST', headers },
      (res) => {
        const trozos = [];
        res.on('data', (d) => trozos.push(d));
        res.on('end', () => {
          const texto = Buffer.concat(trozos).toString('utf8');
          const cuerpoResp = texto ? intentarJSON(texto) : null;
          if (res.statusCode < 200 || res.statusCode >= 300) {
            reject(errorDeRespuesta(res.statusCode, texto, cuerpoResp));
            return;
          }
          resolve(cuerpoResp);
        });
      }
    );
    req.on('error', reject);

    // El progreso se mide en los bytes que ya salieron del fichero hacia el
    // socket, no en los que GitHub confirma recibidos: es la única señal que
    // Node da sin esperar a que termine la subida entera.
    let subidos = 0;
    const lectura = fs.createReadStream(ficheroPath);
    lectura.on('data', (chunk) => {
      subidos += chunk.length;
      if (typeof onProgreso === 'function') onProgreso(subidos, total);
    });
    lectura.on('error', reject);
    lectura.pipe(req);
  });
}

// Decide si una release está "completa" según lo que exige su propio tipo de
// etiqueta. Ver docs/PLAN_MANAGER.md, sección 5: nació del fallo real de
// publicar firmware sin marcarlo como prerelease, que dejó copias instaladas
// sin poder actualizarse.
function veredicto(release) {
  const tag = release.tag_name || '';
  const assets = release.assets || [];
  const motivos = [];

  if (tag.startsWith('fw-v')) {
    if (release.draft) motivos.push('la release está en borrador, no publicada');
    if (!release.prerelease) {
      motivos.push(
        'no está marcada como prerelease: una release de firmware que no lo sea rompe el autoupdate de la app'
      );
    }
    const uf2 = assets.find((a) => a.name && a.name.endsWith('.uf2') && a.size > 0);
    if (!uf2) motivos.push('falta el asset .uf2 (o tiene tamaño 0)');
    return { tipo: 'fw', completada: motivos.length === 0, motivos };
  }

  if (tag.startsWith('v')) {
    if (release.draft) motivos.push('la release está en borrador, no publicada');
    if (release.prerelease) motivos.push('está marcada como prerelease y una release de app no debería estarlo');
    // Los tres van juntos o la release no sirve: el actualizador pide el
    // latest.json, ese apunta al .exe y comprueba su firma contra el .sig.
    // Publicar el .exe solo deja a todo el parque sin actualizar y en silencio,
    // que es el fallo más caro de esta lista porque nadie se entera.
    const json = assets.find((a) => a.name === 'latest.json');
    const exe = assets.find((a) => a.name && a.name.endsWith('.exe'));
    const sig = assets.find((a) => a.name && a.name.endsWith('.exe.sig'));
    if (!json) motivos.push('falta el asset latest.json (el actualizador no la vería)');
    if (!exe) motivos.push('falta el instalador .exe');
    if (!sig) motivos.push('falta la firma .exe.sig (el actualizador rechazaría la descarga)');
    return { tipo: 'app', completada: motivos.length === 0, motivos };
  }

  return { tipo: 'otra', completada: false, motivos: ['etiqueta que no sigue ningún patrón conocido'] };
}

module.exports = {
  REPO,
  hayToken,
  pedir,
  listarReleases,
  obtenerReleasePorTag,
  crearRelease,
  subirAsset,
  veredicto,
};
