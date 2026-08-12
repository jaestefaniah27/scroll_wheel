// Orby Manager — servidor del panel de control del proyecto.
//
// HTTP + SSE y nada más: el trabajo de verdad está en tareas.js (procesos y
// pipelines), versiones.js (los cuatro sitios donde vive un número de versión),
// github.js y teclado.js. Aquí solo se enruta y se difunde.
//
// El estado vive en memoria: procesos lanzados, tarea en curso y la última
// lectura del teclado. No hay base de datos ni fichero de estado a propósito —
// un panel que sobrevive a su propio proceso solo sirve para mentir sobre lo que
// hay corriendo ahora mismo.

const http = require('http');
const fs = require('fs');
const path = require('path');

const versiones = require('./versiones');
const teclado = require('./teclado');
const github = require('./github');
const tareas = require('./tareas');

const PUERTO = 7788;
const RAIZ = versiones.RAIZ;

// ---------------------------------------------------------------------------
// Difusión (SSE)
// ---------------------------------------------------------------------------

const clientes = new Set();
let tareaActual = null;

function difundir(tipo, datos) {
  const trama = `event: ${tipo}\ndata: ${JSON.stringify(datos)}\n\n`;
  for (const res of clientes) {
    try { res.write(trama); } catch { clientes.delete(res); }
  }
}

// El estado se recalcula leyendo ficheros, así que difundirlo es asíncrono. Los
// que lo disparan (un paso de pipeline, un proceso que muere) no tienen nada que
// esperar: si falla, se registra y ya.
function difundirEstado(forzarTeclado = false) {
  construirEstado(forzarTeclado)
    .then((estado) => difundir('estado', estado))
    .catch((err) => difundir('log', { texto: `No se pudo leer el estado: ${err.message}`, flujo: 'error' }));
}

const emisor = {
  log: (texto, flujo = 'salida') => difundir('log', { texto, flujo }),
  paso: (indice, titulo, estado) => {
    difundir('paso', { indice, titulo, estado });
    difundirEstado();
  },
  progreso: (valor, texto) => difundir('progreso', { valor, texto }),
};

// ---------------------------------------------------------------------------
// Estado
// ---------------------------------------------------------------------------

// Abrir el puerto COM tarda su segundo y medio y deja a OrbyGUI sin poder
// conectar mientras dura: en los refrescos automáticos (la tabla de releases se
// repinta cada 15 s durante un pipeline) se sirve la última lectura, y solo el
// botón «Recargar» pide una nueva.
let cacheTeclado = { valor: null, cuando: 0 };

async function leerTecladoQuizas(forzar) {
  if (!forzar && cacheTeclado.valor) {
    return { ...cacheTeclado.valor, cache: true, leidoHace: Date.now() - cacheTeclado.cuando };
  }
  const valor = await teclado.leerTeclado();
  cacheTeclado = { valor, cuando: Date.now() };
  return { ...valor, cache: false, leidoHace: 0 };
}

// "4.10" es posterior a "4.9": comparar como decimal los pone al revés. Misma
// regla que compareFw de OrbyGUI/src/compat.js.
function comparar(a, b) {
  const partes = (v) => String(v ?? '').split('.').map((n) => parseInt(n, 10) || 0);
  const x = partes(a);
  const y = partes(b);
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    const d = (x[i] || 0) - (y[i] || 0);
    if (d) return d < 0 ? -1 : 1;
  }
  return 0;
}

function calcularAvisos(v) {
  const avisos = [];

  // ¿Hay que publicar? Contra la versión de la app en el repositorio, que es la
  // que acaba en el instalador. La vía navegador no tiene número propio.
  if (!v.appTauri) {
    avisos.push({ nivel: 'error', texto: 'No se ha podido leer la versión de la app en el repositorio' });
  } else if (!v.appInstalada) {
    avisos.push({ nivel: 'aviso', texto: `No hay OrbyGUI instalada en este PC (el repositorio va por la ${v.appTauri})` });
  } else {
    const d = comparar(v.appTauri, v.appInstalada);
    if (d === 0) avisos.push({ nivel: 'ok', texto: `La app instalada (${v.appInstalada}) es la del repositorio` });
    else if (d > 0) avisos.push({ nivel: 'aviso', texto: `El repositorio va por la ${v.appTauri} y aquí está instalada la ${v.appInstalada}: falta publicar o instalar` });
    else avisos.push({ nivel: 'aviso', texto: `La app instalada (${v.appInstalada}) es más nueva que la del repositorio (${v.appTauri})` });
  }

  // Los tres ficheros de la versión tienen que decir lo mismo: tauri.conf.json
  // acaba en el instalador y en el latest.json que mira el actualizador,
  // Cargo.toml compila el binario y package.json es el del frontend.
  if (v.appTauri && v.appTauriCargo && v.appPkg
      && (v.appTauri !== v.appTauriCargo || v.appTauri !== v.appPkg)) {
    avisos.push({ nivel: 'error', texto: `La versión de la app no cuadra: ${v.appTauri} en tauri.conf.json, ${v.appTauriCargo} en Cargo.toml y ${v.appPkg} en package.json` });
  }

  // ¿Hay que flashear?
  if (v.teclado && v.teclado.version && v.fwRecomendado) {
    const d = comparar(v.fwRecomendado, v.teclado.version);
    if (d === 0) avisos.push({ nivel: 'ok', texto: `El teclado lleva la ${v.teclado.version}, que es la recomendada` });
    else if (d > 0) avisos.push({ nivel: 'aviso', texto: `El teclado lleva la ${v.teclado.version} y la recomendada es la ${v.fwRecomendado}: toca flashear` });
    else avisos.push({ nivel: 'aviso', texto: `El teclado lleva la ${v.teclado.version}, más nueva que la ${v.fwRecomendado} que conoce esta app` });
  } else if (v.teclado && v.teclado.ocupado) {
    avisos.push({ nivel: 'aviso', texto: 'El puerto del teclado está cogido por otra aplicación: cierra OrbyGUI para leer su versión' });
  } else {
    avisos.push({ nivel: 'aviso', texto: 'Sin teclado conectado: no se puede comparar el firmware' });
  }

  // ¿Se olvidó compat.js en el último bump? Es el desfase silencioso: el
  // firmware sube de versión, la app sigue ofreciendo el anterior y nadie se
  // entera hasta que el instalador de firmware no encuentra la release.
  if (v.firmwareRepo && v.fwRecomendado) {
    if (comparar(v.firmwareRepo, v.fwRecomendado) === 0) {
      avisos.push({ nivel: 'ok', texto: `orby_version.h y FW_RECOMMENDED dicen lo mismo (${v.firmwareRepo})` });
    } else {
      avisos.push({ nivel: 'error', texto: `orby_version.h dice ${v.firmwareRepo} y FW_RECOMMENDED de compat.js dice ${v.fwRecomendado}: se olvidó compat.js en el último bump` });
    }
  }

  // Sin token el panel entero funciona menos lo único que no se puede hacer de
  // otra forma. Vale más decirlo aquí, al abrir, que descubrirlo en el primer
  // paso de un pipeline de release.
  if (!github.hayToken()) {
    avisos.push({ nivel: 'error', texto: 'No hay GH_TOKEN en el entorno: se pueden leer las releases, pero no publicar ninguna' });
  }

  return avisos;
}

async function construirEstado(forzarTeclado = false) {
  const v = {
    firmwareRepo: null, appTauri: null, appTauriCargo: null, appPkg: null, hayTauri: false,
    fwRecomendado: null, appInstalada: null, instaladaError: null, teclado: null,
  };
  const fallos = [];

  try { v.firmwareRepo = versiones.leerFirmware().version; } catch (err) { fallos.push(err.message); }
  // En una rama sin el port de Tauri no hay nada que leer, y no es un fallo:
  // se deja en null y la tarjeta lo dice.
  v.hayTauri = versiones.hayTauri();
  if (v.hayTauri) {
    try {
      const tauri = versiones.leerTauri();
      v.appTauri = tauri.version;
      v.appTauriCargo = tauri.cargo;
      v.appPkg = tauri.pkg;
    } catch (err) { fallos.push(err.message); }
  }
  try { v.fwRecomendado = versiones.leerFwRecomendado(); } catch (err) { fallos.push(err.message); }

  const instalada = await versiones.leerInstalada();
  v.appInstalada = instalada.version;
  v.instaladaError = instalada.error;

  v.teclado = await leerTecladoQuizas(forzarTeclado);

  const avisos = calcularAvisos(v).concat(fallos.map((texto) => ({ nivel: 'error', texto })));

  return {
    versiones: v,
    avisos,
    proceso: tareas.estadoProceso(),
    tarea: tareaActual ? tareaActual.instantanea() : null,
    token: github.hayToken(),
    // La lista sale del servidor y no está escrita en el HTML: añadir una
    // herramienta es tocar un sitio, no dos.
    herramientas: Object.entries(tareas.HERRAMIENTAS).map(([clave, h]) => ({ clave, nombre: h.nombre })),
  };
}

const NOMBRE_COMPONENTE = { fw: 'firmware', tauri: 'la app' };

// Arrancar una tarea larga: se responde 202 al momento y el progreso viaja por
// SSE. El candado es uno solo para todas (releases, flasheo, herramientas):
// compilar el firmware mientras corre una release, o flashear a mitad de un
// build, es pedir un lío que luego no hay forma de leer en el log.
function arrancarTarea(res, pipeline, alTerminar = () => {}) {
  tareaActual = pipeline;
  emisor.log(pipeline.nombre + (pipeline.dry ? ' — ENSAYO EN SECO: no se compila, no se empuja y no se escribe en GitHub' : ''), 'info');
  difundirEstado();
  responder(res, 202, { ok: true });

  pipeline.correr().then(({ avisos, extra }) => {
    const mensaje = pipeline.dry
      ? `Ensayo terminado${avisos ? ` con ${avisos} paso(s) que habrían fallado` : ' sin nada que objetar'}`
      : `${pipeline.nombre}: hecho`;
    difundir('fin', { estado: avisos ? 'error' : 'ok', mensaje, extra });
    difundirEstado();
    alTerminar();
  }).catch((err) => {
    difundir('fin', { estado: 'error', mensaje: err.message, extra: pipeline.ctx.extra });
    difundirEstado();
  });
}

function exigirSinTarea() {
  if (tareaActual && tareaActual.estado === 'corriendo') {
    throw Object.assign(new Error(`Ya hay una tarea en marcha: ${tareaActual.nombre}`), { codigo: 409 });
  }
}

// ---------------------------------------------------------------------------
// Utilidades HTTP
// ---------------------------------------------------------------------------

function responder(res, codigo, objeto) {
  const cuerpo = JSON.stringify(objeto);
  res.writeHead(codigo, {
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': Buffer.byteLength(cuerpo),
    'Cache-Control': 'no-store',
  });
  res.end(cuerpo);
}

function leerCuerpo(req) {
  return new Promise((resolve, reject) => {
    let datos = '';
    req.on('data', (trozo) => {
      datos += trozo;
      // Nada de lo que manda esta UI pasa de unos kilobytes; el tope evita que un
      // POST perdido se coma la memoria del panel.
      if (datos.length > 64 * 1024) { reject(new Error('Petición demasiado grande')); req.destroy(); }
    });
    req.on('end', () => {
      if (!datos.trim()) { resolve({}); return; }
      try { resolve(JSON.parse(datos)); } catch { reject(new Error('El cuerpo de la petición no es JSON válido')); }
    });
    req.on('error', reject);
  });
}

// El panel lanza compilaciones y publica releases con el token del usuario: si
// una página cualquiera de internet pudiera hacerle un POST desde el navegador
// (basta con que resuelva su dominio a 127.0.0.1), publicaría por él. Se escucha
// solo en la interfaz local y, además, se exige que la petición diga venir de
// localhost.
function esLocal(req) {
  const host = (req.headers.host || '').split(':')[0];
  if (host !== 'localhost' && host !== '127.0.0.1' && host !== '[::1]') return false;
  const origen = req.headers.origin;
  if (origen && !/^https?:\/\/(localhost|127\.0\.0\.1|\[::1\]):7788$/.test(origen)) return false;
  return true;
}

// ---------------------------------------------------------------------------
// Rutas
// ---------------------------------------------------------------------------

async function enrutar(req, res, url) {
  const ruta = url.pathname;

  if (req.method === 'GET' && (ruta === '/' || ruta === '/index.html')) {
    const html = fs.readFileSync(path.join(__dirname, 'index.html'));
    res.writeHead(200, {
      'Content-Type': 'text/html; charset=utf-8',
      'Content-Length': html.length,
      // Sin esto, tocar el index.html y recargar no enseña nada nuevo.
      'Cache-Control': 'no-store',
    });
    res.end(html);
    return;
  }

  if (req.method === 'GET' && ruta === '/api/estado') {
    responder(res, 200, await construirEstado(url.searchParams.get('teclado') === '1'));
    return;
  }

  if (req.method === 'GET' && ruta === '/api/eventos') {
    res.writeHead(200, {
      'Content-Type': 'text/event-stream; charset=utf-8',
      'Cache-Control': 'no-store',
      Connection: 'keep-alive',
      // Node y los proxys intermedios trocean y bufferizan por su cuenta; esta
      // cabecera es la que hace que el log llegue línea a línea y no de golpe.
      'X-Accel-Buffering': 'no',
    });
    res.write('retry: 2000\n\n');
    clientes.add(res);

    // El panel puede abrirse con algo ya en marcha: lo primero que recibe es la
    // foto completa, no un lienzo en blanco.
    construirEstado().then((estado) => {
      try { res.write(`event: estado\ndata: ${JSON.stringify(estado)}\n\n`); } catch {}
    }).catch(() => {});

    // Un proxy o el propio Windows cortan una conexión que calla demasiado.
    const latido = setInterval(() => {
      try { res.write(': latido\n\n'); } catch { clearInterval(latido); }
    }, 20000);
    req.on('close', () => { clearInterval(latido); clientes.delete(res); });
    return;
  }

  if (req.method === 'POST' && ruta === '/api/lanzar') {
    const { modo } = await leerCuerpo(req);
    const proceso = tareas.lanzar(modo, emisor, () => difundirEstado());
    difundirEstado();
    responder(res, 200, { ok: true, ...proceso });
    return;
  }

  if (req.method === 'POST' && ruta === '/api/parar') {
    tareas.parar(emisor);
    responder(res, 200, { ok: true });
    return;
  }

  if (req.method === 'POST' && ruta === '/api/bump') {
    const { componente, tipo, nota } = await leerCuerpo(req);
    // Cambiar un número de versión a mitad de un build o de una release deja el
    // artefacto diciendo una cosa y el repositorio otra.
    exigirSinTarea();
    const resultado = componente === 'fw' ? versiones.bumpFirmware(tipo, nota)
      : componente === 'tauri' ? versiones.bumpTauri(tipo)
      : (() => { throw Object.assign(new Error(`Componente desconocido: ${componente}`), { codigo: 400 }); })();

    emisor.log(`Bump de ${NOMBRE_COMPONENTE[componente]}: ${resultado.anterior} → ${resultado.nueva}`, 'info');
    for (const f of resultado.ficheros) emisor.log(`  tocado ${f}`, 'info');

    // El diff se pide solo de los ficheros tocados: el repositorio de git es
    // scroll_wheel entero y un `git diff` a secas traería cambios de otros
    // subproyectos que no tienen nada que ver con este bump.
    let diff = '';
    try {
      diff = await tareas.leer('git', ['diff', '--', ...resultado.ficheros.map((f) => path.join(RAIZ, f))]);
    } catch (err) {
      diff = `(no se ha podido leer el diff: ${err.message})`;
    }
    difundirEstado();
    responder(res, 200, { ok: true, ...resultado, diff });
    return;
  }

  if (req.method === 'POST' && ruta === '/api/release') {
    const { componente, dry } = await leerCuerpo(req);
    exigirSinTarea();
    if (componente !== 'fw' && componente !== 'app') throw Object.assign(new Error(`Componente desconocido: ${componente}`), { codigo: 400 });

    arrancarTarea(res, componente === 'fw'
      ? tareas.pipelineFirmware({ dry: !!dry, emisor })
      : tareas.pipelineApp({ dry: !!dry, emisor }));
    return;
  }

  if (req.method === 'POST' && ruta === '/api/flashear') {
    await leerCuerpo(req);
    exigirSinTarea();
    // Al terminar se relee el teclado de verdad: la tarjeta tiene que enseñar la
    // versión recién instalada, no la que había en la caché de hace un minuto.
    arrancarTarea(res, tareas.pipelineFlashear({ emisor }), () => difundirEstado(true));
    return;
  }

  if (req.method === 'POST' && ruta === '/api/herramienta') {
    const { clave } = await leerCuerpo(req);
    exigirSinTarea();
    arrancarTarea(res, tareas.pipelineHerramienta(clave, { emisor }));
    return;
  }

  if (req.method === 'GET' && ruta === '/api/git') {
    responder(res, 200, await tareas.infoGit());
    return;
  }

  if (req.method === 'POST' && ruta === '/api/subir-cambios') {
    const { mensaje, bump } = await leerCuerpo(req);
    exigirSinTarea();
    arrancarTarea(res, tareas.pipelineSubirCambios({ mensaje, bump, emisor }));
    return;
  }

  if (req.method === 'GET' && ruta === '/api/releases') {
    const crudas = await github.listarReleases(20);
    const releases = crudas.map((r) => ({
      tag: r.tag_name,
      nombre: r.name,
      fecha: r.published_at || r.created_at,
      draft: !!r.draft,
      prerelease: !!r.prerelease,
      url: r.html_url,
      assets: (r.assets || []).map((a) => ({ nombre: a.name, tamano: a.size })),
      veredicto: github.veredicto(r),
    }));
    responder(res, 200, { releases });
    return;
  }

  responder(res, 404, { error: `No existe ${ruta}` });
}

// ---------------------------------------------------------------------------
// Servidor
// ---------------------------------------------------------------------------

const servidor = http.createServer((req, res) => {
  const url = new URL(req.url, `http://localhost:${PUERTO}`);
  if (!esLocal(req)) {
    responder(res, 403, { error: 'Este panel solo atiende peticiones locales' });
    return;
  }
  enrutar(req, res, url).catch((err) => {
    // Los mensajes de error son para leerlos en la UI: van tal cual, en
    // castellano, sin envolverlos en un "error interno" que no dice nada.
    responder(res, err.codigo || 500, { error: err.message });
  });
});

servidor.on('error', (err) => {
  if (err.code === 'EADDRINUSE') {
    console.error(`El puerto ${PUERTO} ya está ocupado: probablemente el panel ya esté abierto en http://localhost:${PUERTO}`);
    process.exit(1);
  }
  throw err;
});

// Solo en la interfaz local. Este proceso compila, empuja y publica con el token
// del usuario: no tiene por qué ser alcanzable desde la red.
servidor.listen(PUERTO, '127.0.0.1', () => {
  console.log(`Orby Manager en http://localhost:${PUERTO}`);
  console.log(`Repositorio: ${RAIZ}`);
  if (!github.hayToken()) console.log('Aviso: no hay GH_TOKEN en el entorno; las releases no podrán publicarse.');
});

// Cerrar el panel con el proceso lanzado vivo dejaría un vite y una cáscara
// corriendo sin ventana desde la que pararlos.
for (const senal of ['SIGINT', 'SIGTERM']) {
  process.on(senal, () => {
    const proceso = tareas.estadoProceso();
    if (proceso) {
      console.log(`Parando ${proceso.etiqueta} (PID ${proceso.pid})…`);
      tareas.matarArbol(proceso.pid);
    }
    process.exit(0);
  });
}
