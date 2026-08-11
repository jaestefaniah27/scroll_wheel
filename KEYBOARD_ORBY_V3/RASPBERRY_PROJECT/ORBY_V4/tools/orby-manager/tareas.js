// Procesos hijo con salida en vivo y pipelines de release.
//
// Todo lo que tarda o toca el repositorio vive aquí; server.js solo enruta HTTP.
// El acoplamiento con la UI se reduce a un `emisor` con tres métodos (log, paso,
// progreso): el pipeline no sabe si al otro lado hay un SSE, un fichero o nada.

const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const versiones = require('./versiones');
const github = require('./github');
const teclado = require('./teclado');

const RAIZ = versiones.RAIZ;
const ORBYGUI = path.join(RAIZ, 'OrbyGUI');

// El emisor nulo permite llamar a estas funciones desde un `node -e` suelto sin
// montar medio servidor para probarlas.
const EMISOR_NULO = { log() {}, paso() {}, progreso() {} };

// ---------------------------------------------------------------------------
// Ejecución de comandos
// ---------------------------------------------------------------------------

// Los scripts de npm son `npm.cmd` en Windows y spawn sin shell no sabe
// ejecutarlos; git y cmake sí son .exe, y con ellos el shell solo añadiría un
// nivel de comillas del que preocuparse (los mensajes de tag llevan espacios).
function ejecutar(cmd, args, opciones = {}) {
  const {
    cwd = RAIZ,
    env = process.env,
    shell = false,
    // 120 s por defecto: es el margen del plan para que un `git push` que se
    // queda esperando credenciales corte en vez de colgar el panel para siempre.
    timeout = 120000,
    emisor = EMISOR_NULO,
    silencioso = false,
  } = opciones;

  return new Promise((resolve, reject) => {
    emisor.log(`> ${cmd} ${args.join(' ')}`, 'info');

    let hijo;
    try {
      hijo = spawn(cmd, args, { cwd, env, shell, windowsHide: true });
    } catch (err) {
      reject(new Error(`No se pudo lanzar ${cmd}: ${err.message}`));
      return;
    }

    let salida = '';
    let vencido = false;

    const trocear = (flujo) => {
      let resto = '';
      return (trozo) => {
        salida += trozo;
        resto += trozo;
        const lineas = resto.split(/\r?\n/);
        resto = lineas.pop();
        for (const linea of lineas) {
          if (!silencioso) emisor.log(linea, flujo);
        }
      };
    };

    hijo.stdout.setEncoding('utf8');
    hijo.stderr.setEncoding('utf8');
    hijo.stdout.on('data', trocear('salida'));
    // git y cmake escriben su progreso normal por stderr: pintarlo de rojo sin
    // más mentiría sobre lo que ha pasado, así que la UI lo distingue pero no lo
    // trata como fallo.
    hijo.stderr.on('data', trocear('error'));

    const reloj = setTimeout(() => {
      vencido = true;
      matarArbol(hijo.pid);
    }, timeout);

    hijo.on('error', (err) => {
      clearTimeout(reloj);
      reject(new Error(`No se pudo lanzar ${cmd}: ${err.message}`));
    });

    hijo.on('close', (codigo) => {
      clearTimeout(reloj);
      if (vencido) {
        reject(new Error(
          `«${cmd} ${args.join(' ')}» ha pasado de ${Math.round(timeout / 1000)} s y se ha cortado. ` +
          'Si era un git push, lo más probable es que se quedara esperando credenciales: ' +
          'ejecútalo una vez a mano en una consola para que Windows las guarde.'
        ));
        return;
      }
      if (codigo !== 0) {
        reject(new Error(`«${cmd} ${args.join(' ')}» ha terminado con código ${codigo}`));
        return;
      }
      resolve({ codigo, salida });
    });
  });
}

// Matar solo al padre deja los hijos huérfanos: `npm run dev` arranca vite y
// electron con concurrently, y sin /T se quedan los dos corriendo con el puerto
// COM cogido y sin ventana que cerrar.
function matarArbol(pid) {
  if (!pid) return;
  if (process.platform !== 'win32') {
    try { process.kill(-pid, 'SIGKILL'); } catch { try { process.kill(pid, 'SIGKILL'); } catch {} }
    return;
  }
  try {
    spawn('taskkill', ['/pid', String(pid), '/T', '/F'], { windowsHide: true });
  } catch {}
}

// Salida de un comando de lectura, sin volcarla al log: `git status --porcelain`
// y compañía se consultan, no se enseñan.
async function leer(cmd, args, opciones = {}) {
  const { salida } = await ejecutar(cmd, args, { ...opciones, silencioso: true, timeout: 30000 });
  return salida.trim();
}

// ---------------------------------------------------------------------------
// Lanzar OrbyGUI
// ---------------------------------------------------------------------------

const MODOS = {
  'electron-dev': { etiqueta: 'Electron (dev)', args: ['run', 'dev'] },
  'electron':     { etiqueta: 'Electron', args: ['start'] },
  'tauri-dev':    { etiqueta: 'Tauri (dev)', args: ['run', 'tauri:dev'] },
  'tauri-build':  { etiqueta: 'Tauri (build)', args: ['run', 'tauri:build'] },
  'web':          { etiqueta: 'Web', args: ['run', 'preview:web'], url: 'http://localhost:5174' },
};

let procesoActual = null;

function estadoProceso() {
  if (!procesoActual) return null;
  return { modo: procesoActual.modo, etiqueta: procesoActual.etiqueta, pid: procesoActual.pid };
}

function lanzar(modo, emisor = EMISOR_NULO, alTerminar = () => {}) {
  const receta = MODOS[modo];
  if (!receta) throw new Error(`Modo desconocido: ${modo}`);
  // Uno solo a la vez: dos OrbyGUI compitiendo por el puerto COM no es un
  // escenario que nadie quiera depurar.
  if (procesoActual) throw new Error(`Ya hay algo corriendo: ${procesoActual.etiqueta} (PID ${procesoActual.pid})`);

  emisor.log(`Lanzando ${receta.etiqueta}: npm ${receta.args.join(' ')}`, 'info');
  const hijo = spawn('npm', receta.args, { cwd: ORBYGUI, shell: true, windowsHide: true });

  let resto = { salida: '', error: '' };
  for (const flujo of ['salida', 'error']) {
    const stream = flujo === 'salida' ? hijo.stdout : hijo.stderr;
    stream.setEncoding('utf8');
    stream.on('data', (trozo) => {
      resto[flujo] += trozo;
      const lineas = resto[flujo].split(/\r?\n/);
      resto[flujo] = lineas.pop();
      for (const linea of lineas) emisor.log(linea, flujo);
    });
  }

  hijo.on('close', (codigo) => {
    emisor.log(`${receta.etiqueta} ha terminado (código ${codigo})`, 'info');
    procesoActual = null;
    alTerminar();
  });

  procesoActual = { hijo, pid: hijo.pid, modo, etiqueta: receta.etiqueta };

  // La vía navegador no abre ventana por su cuenta. El margen es para que vite
  // haya construido y esté escuchando: abrir antes solo enseña un error de
  // conexión que hay que recargar a mano.
  if (receta.url) {
    setTimeout(() => {
      if (procesoActual && procesoActual.modo === modo) {
        emisor.log(`Abriendo ${receta.url}`, 'info');
        abrirNavegador(receta.url);
      }
    }, 6000);
  }

  return estadoProceso();
}

function parar(emisor = EMISOR_NULO) {
  if (!procesoActual) throw new Error('No hay nada corriendo');
  emisor.log(`Parando ${procesoActual.etiqueta} (PID ${procesoActual.pid}) y todo su árbol`, 'info');
  matarArbol(procesoActual.pid);
}

function abrirNavegador(url) {
  if (process.platform === 'win32') {
    // El primer argumento vacío es el título de la ventana: sin él, `start` se
    // come la URL entre comillas creyendo que es el título.
    spawn('cmd', ['/c', 'start', '', url], { windowsHide: true, detached: true }).unref();
  } else {
    spawn(process.platform === 'darwin' ? 'open' : 'xdg-open', [url], { detached: true }).unref();
  }
}

// ---------------------------------------------------------------------------
// Pipelines
// ---------------------------------------------------------------------------

class Pipeline {
  constructor({ nombre, dry, pasos, emisor = EMISOR_NULO }) {
    this.nombre = nombre;
    this.dry = !!dry;
    this.emisor = emisor;
    this.pasos = pasos.map((p) => ({ titulo: p.titulo, ejecutar: p.ejecutar, estado: 'pendiente' }));
    this.indice = 0;
    this.estado = 'corriendo';
    this.ctx = { dry: this.dry, emisor, extra: {} };
  }

  instantanea() {
    return {
      nombre: this.nombre,
      dry: this.dry,
      estado: this.estado,
      indice: this.indice,
      progreso: this.pasos.filter((p) => p.estado !== 'pendiente' && p.estado !== 'corriendo').length / this.pasos.length,
      pasos: this.pasos.map((p) => ({ titulo: p.titulo, estado: p.estado })),
    };
  }

  _marcar(i, estado) {
    this.pasos[i].estado = estado;
    this.emisor.paso(i, this.pasos[i].titulo, estado);
    this.emisor.progreso(this.instantanea().progreso, this.pasos[i].titulo);
  }

  async correr() {
    let avisos = 0;
    for (let i = 0; i < this.pasos.length; i++) {
      this.indice = i;
      this._marcar(i, 'corriendo');
      this.emisor.log(`── ${i + 1}/${this.pasos.length} · ${this.pasos[i].titulo}`, 'info');
      try {
        await this.pasos[i].ejecutar(this.ctx);
        this._marcar(i, 'ok');
      } catch (err) {
        // En ensayo un fallo no corta: la gracia del ensayo es ver la secuencia
        // entera y todo lo que habría que arreglar, no el primer tropiezo. En una
        // release de verdad sí para: seguir después de un paso roto solo sirve
        // para dejar el repositorio a medias.
        if (this.dry) {
          avisos++;
          this._marcar(i, 'aviso');
          this.emisor.log(`[ensayo] este paso habría fallado: ${err.message}`, 'error');
          continue;
        }
        this._marcar(i, 'error');
        this.estado = 'error';
        this.emisor.log(err.message, 'error');
        throw err;
      }
    }
    this.estado = 'ok';
    return { avisos, extra: this.ctx.extra };
  }
}

// En ensayo no se ejecuta nada que compile, escriba o empuje: se cuenta. Las
// lecturas (git status, listar releases, calcular una huella de un fichero que ya
// está) sí se hacen de verdad, que es lo único que puede fallar de forma
// informativa antes de publicar.
function ensayado(ctx, descripcion, accion) {
  if (ctx.dry) {
    ctx.emisor.log(`[ensayo] no se ejecuta: ${descripcion}`, 'info');
    return Promise.resolve(null);
  }
  return Promise.resolve(accion());
}

// Las comprobaciones previas se acumulan en vez de cortar en la primera: quien
// lanza un ensayo quiere la lista entera de lo que le falta, no descubrir un
// problema por intento. Al final, si hay algo, se lanza todo junto.
async function comprobar(problemas, fn) {
  try {
    await fn();
  } catch (err) {
    problemas.push(err.message);
  }
}

function rendirCuentas(problemas) {
  if (!problemas.length) return;
  throw new Error(problemas.length === 1 ? problemas[0] : problemas.map((p) => `• ${p}`).join('\n'));
}

async function exigirArbolLimpio(emisor) {
  // El repositorio de git es scroll_wheel entero, varios niveles por encima de
  // ORBY_V4: la etiqueta y el push son del repositorio, así que lo sucio de
  // cualquier otro subproyecto también entra en la foto que se va a publicar.
  // Se recorta solo por el final: los dos primeros caracteres de cada línea son
  // el estado del fichero y uno de ellos suele ser un espacio.
  const { salida } = await ejecutar('git', ['status', '--porcelain'], { emisor, silencioso: true, timeout: 30000 });
  const sucio = salida.replace(/\s+$/, '');
  if (sucio) {
    const lineas = sucio.split(/\r?\n/);
    const muestra = lineas.slice(0, 10).join('\n');
    throw new Error(
      `El árbol de trabajo tiene ${lineas.length} cambio(s) sin commitear. Commitea o guarda antes de publicar:\n${muestra}` +
      (lineas.length > 10 ? '\n…' : '')
    );
  }
}

async function exigirRamaMain(emisor) {
  const rama = await leer('git', ['rev-parse', '--abbrev-ref', 'HEAD'], { emisor });
  if (rama !== 'main') throw new Error(`Se publica desde main, y la rama actual es «${rama}»`);
}

// --- Git -------------------------------------------------------------------

// El repositorio es scroll_wheel entero y ORBY_V4 es una carpeta dentro. Se
// calcula una vez el prefijo para poder separar lo que toca a este proyecto de
// lo que hay revuelto en subproyectos hermanos.
let prefijoEnGit = null;
async function prefijoProyecto() {
  if (prefijoEnGit !== null) return prefijoEnGit;
  try {
    const raizGit = await leer('git', ['rev-parse', '--show-toplevel']);
    prefijoEnGit = path.relative(raizGit, RAIZ).split(path.sep).join('/');
  } catch {
    prefijoEnGit = '';
  }
  return prefijoEnGit;
}

async function infoGit() {
  const prefijo = await prefijoProyecto();
  const rama = await leer('git', ['rev-parse', '--abbrev-ref', 'HEAD']);

  const { salida } = await ejecutar('git', ['status', '--porcelain'], { silencioso: true, timeout: 30000 });
  const cambios = salida.replace(/\s+$/, '').split(/\r?\n/).filter(Boolean).map((linea) => {
    const ruta = linea.slice(3);
    return { estado: linea.slice(0, 2).trim(), ruta, delProyecto: !prefijo || ruta.startsWith(prefijo) };
  });

  // El byte 0x1f como separador y no un espacio: los mensajes de commit llevan de todo.
  const registro = await leer('git', ['log', '-8', '--date=short', '--pretty=format:%h%x1f%ad%x1f%s']);
  const commits = registro.split(/\r?\n/).filter(Boolean).map((linea) => {
    const [hash, fecha, mensaje] = linea.split('\x1f');
    return { hash, fecha, mensaje };
  });

  // Sin rama de seguimiento (una rama local recién creada) no hay nada que
  // comparar: se devuelve null y la UI dice que el push creará la remota.
  let remoto = null;
  let adelante = 0;
  let atras = 0;
  try {
    remoto = await leer('git', ['rev-parse', '--abbrev-ref', '--symbolic-full-name', '@{u}']);
    const cuenta = await leer('git', ['rev-list', '--left-right', '--count', 'HEAD...@{u}']);
    [adelante, atras] = cuenta.split(/\s+/).map((n) => parseInt(n, 10) || 0);
  } catch {
    remoto = null;
  }

  return { rama, remoto, adelante, atras, cambios, commits, prefijo };
}

function sha256DeFichero(ruta) {
  const hash = crypto.createHash('sha256');
  hash.update(fs.readFileSync(ruta));
  return hash.digest('hex');
}

// --- Firmware ---------------------------------------------------------------

function pipelineFirmware({ dry, emisor }) {
  const fw = versiones.leerFirmware();
  const tag = `fw-v${fw.version}`;
  const uf2Origen = path.join(RAIZ, 'build', 'ORBY_V4.uf2');
  const uf2Destino = path.join(RAIZ, 'build', `ORBY_V4-fw-${fw.version}.uf2`);
  const shaDestino = `${uf2Destino}.sha256`;

  return new Pipeline({
    nombre: `Release de firmware ${fw.version}`,
    dry,
    emisor,
    pasos: [
      {
        titulo: 'Comprobaciones',
        async ejecutar(ctx) {
          ctx.fw = fw;
          ctx.tag = tag;
          emisor.log(`include/orby_version.h dice ${fw.version} → etiqueta ${tag}`, 'info');

          const problemas = [];
          await comprobar(problemas, () => {
            if (!github.hayToken()) throw new Error('Falta GH_TOKEN en el entorno: sin él no se puede publicar');
          });
          await comprobar(problemas, () => exigirArbolLimpio(emisor));
          await comprobar(problemas, () => exigirRamaMain(emisor));
          await comprobar(problemas, async () => {
            const yaExiste = await leer('git', ['tag', '-l', tag], { emisor });
            if (yaExiste) throw new Error(`La etiqueta ${tag} ya existe en local. Sube la versión o borra la etiqueta.`);
          });

          // El workflow muerto comprobaba que la etiqueta y ORBY_FW_MINOR dijeran
          // lo mismo. Aquí la etiqueta se construye desde la cabecera, así que esa
          // comprobación es trivialmente cierta; lo que de verdad se desincroniza
          // —y nadie valida hoy— son los otros dos sitios donde vive el número.
          await comprobar(problemas, () => {
            const recomendado = versiones.leerFwRecomendado();
            if (recomendado !== fw.version) {
              throw new Error(
                `FW_RECOMMENDED de OrbyGUI/src/compat.js es ${recomendado} y el firmware es ${fw.version}. ` +
                'La app no ofrecería este firmware al actualizar: se olvidó el compat.js en el bump.'
              );
            }
          });
          await comprobar(problemas, () => {
            const doc = fs.readFileSync(versiones.RUTAS.compatibilidad, 'utf8');
            if (!doc.includes(`| **${fw.version}** |`)) {
              throw new Error(`docs/COMPATIBILIDAD.md no tiene fila para la ${fw.version}: falta decir qué trae esta versión.`);
            }
          });

          rendirCuentas(problemas);
          emisor.log('Todo en orden: árbol limpio, rama main, etiqueta libre y versión coherente en los tres sitios', 'info');
        },
      },
      {
        titulo: 'Compilar',
        async ejecutar(ctx) {
          await ensayado(ctx, 'cmake --build build', async () => {
            // Misma trampa que flash.ps1: una caché generada desde otra ruta deja
            // el build inservible y cmake no lo detecta solo.
            const cache = path.join(RAIZ, 'build', 'CMakeCache.txt');
            if (fs.existsSync(cache) && fs.readFileSync(cache, 'utf8').includes('scroll_wheel/scroll_wheel')) {
              emisor.log('Caché de cmake antigua e inválida: se borra build/', 'info');
              fs.rmSync(path.join(RAIZ, 'build'), { recursive: true, force: true });
            }
            // Se mira CMakeCache.txt y no la carpeta: un build interrumpido deja
            // build/ creada sin caché y `cmake --build` falla con "could not load cache".
            if (!fs.existsSync(cache)) {
              await ejecutar('cmake', ['-B', 'build', '-G', 'Ninja'], { emisor, timeout: 300000 });
            }
            await ejecutar('cmake', ['--build', 'build'], { emisor, timeout: 900000 });
          });
        },
      },
      {
        titulo: 'Empaquetar',
        async ejecutar(ctx) {
          await ensayado(ctx, `copiar ${path.basename(uf2Origen)} a ${path.basename(uf2Destino)} y calcular su SHA256`, () => {
            if (!fs.existsSync(uf2Origen)) throw new Error(`No está build/ORBY_V4.uf2: la compilación no ha dejado firmware`);
            fs.copyFileSync(uf2Origen, uf2Destino);
            const huella = sha256DeFichero(uf2Destino);
            // Formato de sha256sum (huella, dos espacios, nombre): es el que
            // entienden las herramientas de línea de órdenes de quien lo descargue.
            fs.writeFileSync(shaDestino, `${huella}  ${path.basename(uf2Destino)}\n`, 'utf8');
            ctx.huella = huella;
            ctx.extra.sha256 = huella;
            emisor.log(`${path.basename(uf2Destino)} · ${fs.statSync(uf2Destino).size} bytes`, 'info');
            emisor.log(`SHA256 ${huella}`, 'info');
          });
        },
      },
      {
        titulo: 'Etiquetar y empujar',
        async ejecutar(ctx) {
          await ensayado(ctx, `git tag -a ${tag} && git push origin main && git push origin ${tag}`, async () => {
            await ejecutar('git', ['tag', '-a', tag, '-m', `Firmware ${fw.version}`], { emisor });
            await ejecutar('git', ['push', 'origin', 'main'], { emisor });
            await ejecutar('git', ['push', 'origin', tag], { emisor });
          });
        },
      },
      {
        titulo: 'Publicar la release',
        async ejecutar(ctx) {
          const notas = notasFirmware(fw.version, ctx.huella, path.basename(uf2Destino));
          ctx.extra.notas = notas;
          await ensayado(ctx, `POST /repos/${github.REPO}/releases con tag ${tag} y prerelease:true`, async () => {
            // prerelease no es negociable y por eso va aquí escrito y no en un
            // parámetro: releases/latest de GitHub devuelve la más reciente sin
            // mirar la etiqueta, y una release de firmware que no sea prerelease
            // deja a electron-updater buscando latest.yml donde no lo hay.
            ctx.release = await github.crearRelease({
              tag,
              nombre: `Firmware ${fw.version}`,
              notas,
              prerelease: true,
            });
            emisor.log(`Release creada: ${ctx.release.html_url}`, 'info');
          });
        },
      },
      {
        titulo: 'Subir los ficheros',
        async ejecutar(ctx) {
          await ensayado(ctx, `subir ${path.basename(uf2Destino)} y ${path.basename(shaDestino)}`, async () => {
            for (const fichero of [uf2Destino, shaDestino]) {
              const nombre = path.basename(fichero);
              emisor.log(`Subiendo ${nombre}…`, 'info');
              await github.subirAsset(ctx.release.upload_url, fichero, (subidos, total) => {
                emisor.progreso(total ? subidos / total : 0, `${nombre} · ${Math.round(subidos / 1024)} KB de ${Math.round(total / 1024)} KB`);
              });
              emisor.log(`${nombre} subido`, 'info');
            }
          });
        },
      },
      {
        titulo: 'Verificar',
        async ejecutar(ctx) {
          // Lectura: en ensayo también se hace, y es lo que avisa de que la
          // etiqueta ya estaba publicada de un intento anterior.
          const release = await github.obtenerReleasePorTag(tag);
          if (!release) {
            if (ctx.dry) { emisor.log(`[ensayo] la release ${tag} todavía no existe, que es lo esperado`, 'info'); return; }
            throw new Error(`La release ${tag} no aparece en GitHub`);
          }
          if (ctx.dry) throw new Error(`La release ${tag} YA existe en GitHub (${release.html_url}): publicarla de verdad fallaría`);

          const uf2 = (release.assets || []).find((a) => a.name.endsWith('.uf2'));
          if (!uf2) throw new Error('La release no tiene ningún .uf2');
          const tamanoLocal = fs.statSync(uf2Destino).size;
          if (uf2.size !== tamanoLocal) {
            throw new Error(`El .uf2 publicado ocupa ${uf2.size} bytes y el local ${tamanoLocal}: la subida se ha cortado`);
          }
          if (!release.prerelease) {
            throw new Error('La release NO está marcada como prerelease: márcala a mano o romperás el autoupdate de la app');
          }
          emisor.log(`Verificada: ${release.html_url} · ${uf2.name} (${uf2.size} bytes)`, 'info');
        },
      },
    ],
  });
}

function notasFirmware(version, huella, nombreUf2) {
  return [
    `## Firmware ${version}`,
    '',
    'Se instala desde la propia app: **Ajustes → Firmware del teclado**. No hace falta',
    'tocar el botón BOOTSEL.',
    '',
    'Para instalarlo a mano: conecta el teclado con BOOTSEL pulsado y copia el `.uf2`',
    'a la unidad `RPI-RP2`.',
    '',
    'Comprueba el fichero antes de copiarlo:',
    '',
    '```',
    `Get-FileHash .\\${nombreUf2} -Algorithm SHA256`,
    '',
    `SHA256: ${huella || '(sin calcular: ensayo en seco)'}`,
    '```',
    '',
    'La configuración del teclado sobrevive al reflasheo; las secuencias se reenvían',
    'solas al conectar la app.',
  ].join('\n');
}

// --- App --------------------------------------------------------------------

// La release de la app es la de la vía Electron: es la que produce un
// instalador y la que mira electron-updater. La vía Tauri lleva su propia
// versión y todavía no se publica; la de navegador no tiene número.
function pipelineApp({ dry, emisor }) {
  const app = versiones.leerElectron();
  const tag = `v${app.version}`;
  const carpetaRelease = path.join(ORBYGUI, 'release');

  return new Pipeline({
    nombre: `Release de la app ${app.version}`,
    dry,
    emisor,
    pasos: [
      {
        titulo: 'Comprobaciones',
        async ejecutar(ctx) {
          ctx.tag = tag;
          emisor.log(`OrbyGUI/package.json dice ${app.version} → etiqueta ${tag}`, 'info');
          const problemas = [];
          await comprobar(problemas, () => {
            if (!github.hayToken()) throw new Error('Falta GH_TOKEN en el entorno: electron-builder no podría publicar');
          });
          await comprobar(problemas, () => exigirArbolLimpio(emisor));
          await comprobar(problemas, () => exigirRamaMain(emisor));

          // Aquí no se comprueba la etiqueta local: la crea electron-builder al
          // publicar, no nosotros. Lo que sí duele es encontrarse la release ya
          // hecha de un intento anterior a medias.
          await comprobar(problemas, async () => {
            const previa = await github.obtenerReleasePorTag(tag);
            if (previa) {
              throw new Error(
                `La release ${tag} ya existe en GitHub (${previa.html_url}). ` +
                'Súbele la versión a la app o borra esa release antes de repetir.'
              );
            }
          });

          rendirCuentas(problemas);
        },
      },
      {
        titulo: 'Compilar y publicar (electron-builder)',
        async ejecutar(ctx) {
          await ensayado(ctx, 'npm run release (vite build + electron-builder --publish always)', async () => {
            // publish.releaseType ya es `release` en package.json: electron-builder
            // crea la release, la publica y sube latest.yml y el .exe. GH_TOKEN va
            // por entorno porque es lo único que mira.
            await ejecutar('npm', ['run', 'release'], {
              cwd: ORBYGUI,
              shell: true,
              emisor,
              timeout: 1800000,
              env: { ...process.env, GH_TOKEN: process.env.GH_TOKEN },
            });
          });
        },
      },
      {
        titulo: 'Huella del instalador',
        async ejecutar(ctx) {
          // Lectura pura: en ensayo se calcula igual si hay un .exe de un build
          // anterior, y así se ve el bloque de notas que se publicaría.
          if (!fs.existsSync(carpetaRelease)) {
            if (ctx.dry) { emisor.log('[ensayo] no hay OrbyGUI/release/: nada que medir', 'info'); return; }
            throw new Error('No existe OrbyGUI/release/: electron-builder no ha dejado nada');
          }
          const exe = fs.readdirSync(carpetaRelease)
            .filter((n) => /^OrbyGUI-Setup-.*\.exe$/i.test(n))
            .map((n) => ({ n, m: fs.statSync(path.join(carpetaRelease, n)).mtimeMs }))
            .sort((a, b) => b.m - a.m)[0];
          if (!exe) {
            if (ctx.dry) { emisor.log('[ensayo] no hay ningún OrbyGUI-Setup-*.exe todavía', 'info'); return; }
            throw new Error('No hay ningún OrbyGUI-Setup-*.exe en OrbyGUI/release/');
          }
          const huella = sha256DeFichero(path.join(carpetaRelease, exe.n));
          ctx.extra.sha256 = huella;
          ctx.extra.notas = notasApp(app.version, huella);
          emisor.log(`${exe.n} · SHA256 ${huella}`, 'info');
          emisor.log('Las notas de la versión ya formateadas están en el botón «Copiar notas».', 'info');
        },
      },
      {
        titulo: 'Verificar',
        async ejecutar(ctx) {
          const release = await github.obtenerReleasePorTag(tag);
          if (!release) {
            if (ctx.dry) { emisor.log(`[ensayo] la release ${tag} todavía no existe, que es lo esperado`, 'info'); return; }
            throw new Error(`La release ${tag} no aparece en GitHub`);
          }
          const v = github.veredicto(release);
          if (!v.completada) throw new Error(`La release ${tag} está incompleta: ${v.motivos.join('; ')}`);
          emisor.log(`Verificada: ${release.html_url}`, 'info');
        },
      },
    ],
  });
}

function notasApp(version, huella) {
  return [
    `## OrbyGUI ${version}`,
    '',
    '## Instalación',
    '',
    'Windows mostrará el aviso de SmartScreen: el instalador todavía no va firmado',
    '(ver docs/PUBLICACION.md). Para continuar: **Más información → Ejecutar de',
    'todas formas**.',
    '',
    'Antes de ejecutarlo, comprueba que el fichero es el que publicamos:',
    '',
    '    Get-FileHash .\\OrbyGUI-Setup.exe -Algorithm SHA256',
    '',
    `    SHA256: ${huella}`,
  ].join('\n');
}

// ---------------------------------------------------------------------------
// Flashear el teclado
// ---------------------------------------------------------------------------

// No se reimplementa lo que hace flash.ps1 (pedir BOOTSEL por el CDC, buscar la
// unidad RPI-RP2, copiar el .uf2): se ejecuta el script que ya existe y está
// probado. Duplicarlo aquí sería tener dos versiones de la misma lógica que se
// desincronizarían a la primera.
function pipelineFlashear({ emisor }) {
  const fw = versiones.leerFirmware();

  return new Pipeline({
    nombre: `Flashear el teclado con la ${fw.version}`,
    dry: false,
    emisor,
    pasos: [
      {
        titulo: 'Comprobar el teclado',
        async ejecutar(ctx) {
          if (procesoActual) {
            // flash.ps1 necesita el puerto libre para mandar el BOOTSEL. Con
            // OrbyGUI abierta cae al método manual y se queda esperando 120 s a
            // que alguien enchufe el cable con el botón pulsado.
            throw new Error(
              `Hay ${procesoActual.etiqueta} corriendo y tiene cogido el puerto del teclado. ` +
              'Párala antes de flashear (botón «Parar» de la sección Lanzar).'
            );
          }
          const antes = await teclado.leerTeclado();
          ctx.antes = antes;
          if (antes.ocupado) {
            throw new Error('El puerto del teclado está cogido por otra aplicación: cierra OrbyGUI antes de flashear.');
          }
          if (antes.disponible) {
            emisor.log(`Teclado en ${antes.puerto} con la ${antes.version}; se va a instalar la ${fw.version}`, 'info');
          } else {
            // No es motivo para no seguir: flash.ps1 admite el camino manual
            // (enchufar con BOOTSEL pulsado) y espera él solo.
            emisor.log(`No se ve el teclado (${antes.error}). flash.ps1 esperará a que lo enchufes con BOOTSEL pulsado.`, 'info');
          }
        },
      },
      {
        titulo: 'Compilar y flashear (flash.ps1)',
        async ejecutar() {
          // -ExecutionPolicy Bypass porque el script no va firmado y la política
          // por defecto de Windows lo rechazaría; -NoProfile para no arrastrar el
          // perfil del usuario, que puede tardar segundos en cargar.
          //
          // Se invoca con -Command y no con -File para poder poner antes la
          // codificación de salida en UTF-8: por defecto PowerShell escribe en la
          // página de códigos de la consola (cp850) y aquí se lee como UTF-8, así
          // que los mensajes de flash.ps1 llegaban al log como «con Ǹxito».
          const guion = `[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; & '${path.join(RAIZ, 'flash.ps1')}'`;
          await ejecutar('powershell', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-Command', guion], {
            emisor,
            // El camino manual de flash.ps1 espera hasta 120 s a que aparezca la
            // unidad, y antes hay una compilación entera.
            timeout: 900000,
          });
        },
      },
      {
        titulo: 'Verificar',
        async ejecutar(ctx) {
          // El teclado se reinicia y tiene que volver a enumerarse: preguntarle
          // inmediatamente solo da "no se ve ningún teclado".
          for (let intento = 1; intento <= 6; intento++) {
            await new Promise((r) => setTimeout(r, 2500));
            const ahora = await teclado.leerTeclado();
            if (ahora.disponible) {
              ctx.extra.versionTeclado = ahora.version;
              if (ahora.version === fw.version) {
                emisor.log(`El teclado vuelve a estar en ${ahora.puerto} y anuncia la ${ahora.version}`, 'info');
                return;
              }
              throw new Error(
                `El teclado anuncia la ${ahora.version} y se acaba de instalar la ${fw.version}. ` +
                'Si el .uf2 se copió a medias, vuelve a flashear.'
              );
            }
            emisor.log(`Todavía no contesta (intento ${intento} de 6)…`, 'info');
          }
          throw new Error(
            'El teclado no ha vuelto a aparecer. Si la unidad RPI-RP2 sigue montada, la copia falló; ' +
            'si no, dale unos segundos y recarga la tarjeta del teclado.'
          );
        },
      },
    ],
  });
}

// ---------------------------------------------------------------------------
// Herramientas sueltas
// ---------------------------------------------------------------------------

// Un comando cada una, sin ceremonia. Van por el mismo Pipeline que las
// releases para que compartan barra, log y el candado de «una tarea a la vez»:
// compilar el firmware mientras corre una release sería pedir un lío.
const HERRAMIENTAS = {
  'compilar-fw': {
    nombre: 'Compilar el firmware',
    async ejecutar(emisor) {
      const cache = path.join(RAIZ, 'build', 'CMakeCache.txt');
      if (!fs.existsSync(cache)) {
        await ejecutar('cmake', ['-B', 'build', '-G', 'Ninja'], { emisor, timeout: 300000 });
      }
      await ejecutar('cmake', ['--build', 'build'], { emisor, timeout: 900000 });
      const uf2 = path.join(RAIZ, 'build', 'ORBY_V4.uf2');
      if (fs.existsSync(uf2)) emisor.log(`build/ORBY_V4.uf2 · ${fs.statSync(uf2).size} bytes`, 'info');
    },
  },
  'limpiar-build': {
    nombre: 'Limpiar build/',
    async ejecutar(emisor) {
      const build = path.join(RAIZ, 'build');
      if (!fs.existsSync(build)) { emisor.log('No hay carpeta build/ que limpiar', 'info'); return; }
      fs.rmSync(build, { recursive: true, force: true });
      emisor.log('build/ borrada. La próxima compilación vuelve a configurar cmake desde cero.', 'info');
    },
  },
  'dist': {
    nombre: 'Empaquetar el instalador (sin publicar)',
    async ejecutar(emisor) {
      // `npm run dist` es --publish never: construye el .exe y no sube nada.
      await ejecutar('npm', ['run', 'dist'], { cwd: ORBYGUI, shell: true, emisor, timeout: 1800000 });
      const carpeta = path.join(ORBYGUI, 'release');
      if (!fs.existsSync(carpeta)) return;
      const exe = fs.readdirSync(carpeta)
        .filter((n) => /^OrbyGUI-Setup-.*\.exe$/i.test(n))
        .map((n) => ({ n, m: fs.statSync(path.join(carpeta, n)).mtimeMs }))
        .sort((a, b) => b.m - a.m)[0];
      if (exe) {
        emisor.log(`OrbyGUI/release/${exe.n}`, 'info');
        emisor.log(`SHA256 ${sha256DeFichero(path.join(carpeta, exe.n))}`, 'info');
      }
    },
  },
  'pack-plugins': {
    nombre: 'Empaquetar los complementos',
    async ejecutar(emisor) {
      await ejecutar('npm', ['run', 'pack:plugins'], { cwd: ORBYGUI, shell: true, emisor, timeout: 300000 });
    },
  },
  'git-pull': {
    nombre: 'Traer cambios (git pull)',
    async ejecutar(emisor) {
      // --ff-only: si la rama ha divergido, mejor enterarse aquí que quedarse
      // con un merge automático hecho a espaldas de nadie.
      await ejecutar('git', ['pull', '--ff-only'], { emisor });
    },
  },
  'git-fetch': {
    nombre: 'Traer etiquetas (git fetch --tags)',
    async ejecutar(emisor) {
      await ejecutar('git', ['fetch', '--tags', '--prune'], { emisor });
    },
  },
};

// `codigo` viaja hasta el estado HTTP: lo que se rechaza por venir mal pedido es
// un 400, no un 500. La UI enseña el mensaje igual, pero un 500 en el registro
// del servidor haría buscar un fallo que no existe.
function malaPeticion(mensaje) {
  return Object.assign(new Error(mensaje), { codigo: 400 });
}

function pipelineHerramienta(clave, { emisor }) {
  const receta = HERRAMIENTAS[clave];
  if (!receta) throw malaPeticion(`Herramienta desconocida: ${clave}`);
  return new Pipeline({
    nombre: receta.nombre,
    dry: false,
    emisor,
    pasos: [{ titulo: receta.nombre, ejecutar: () => receta.ejecutar(emisor) }],
  });
}

// ---------------------------------------------------------------------------
// Commit y push
// ---------------------------------------------------------------------------

// Con bump opcional delante: subir la versión y commitearla en el mismo paso es
// justo lo que se olvida a mano, y deja el repositorio diciendo una versión que
// no está publicada en ningún sitio.
function pipelineSubirCambios({ mensaje, bump, emisor }) {
  if (!mensaje || !mensaje.trim()) throw malaPeticion('Hace falta un mensaje de commit');

  const pasos = [];

  if (bump && bump.componente) {
    pasos.push({
      titulo: `Bump de ${bump.componente}`,
      async ejecutar(ctx) {
        const resultado = bump.componente === 'fw' ? versiones.bumpFirmware(bump.tipo, bump.nota)
          : bump.componente === 'electron' ? versiones.bumpElectron(bump.tipo)
          : bump.componente === 'tauri' ? versiones.bumpTauri(bump.tipo)
          : (() => { throw new Error(`Componente desconocido: ${bump.componente}`); })();
        ctx.extra.bump = resultado;
        emisor.log(`${resultado.anterior} → ${resultado.nueva}`, 'info');
        for (const f of resultado.ficheros) emisor.log(`  tocado ${f}`, 'info');
      },
    });
  }

  pasos.push(
    {
      titulo: 'Preparar los cambios',
      async ejecutar(ctx) {
        // Se añade solo lo que cuelga de ORBY_V4: el repositorio es scroll_wheel
        // entero y un `git add -A` a secas arrastraría a un commit de este
        // proyecto lo que hubiera a medias en cualquier subproyecto hermano.
        await ejecutar('git', ['add', '-A', '--', RAIZ], { emisor });
        const preparado = await leer('git', ['diff', '--cached', '--name-only'], { emisor });
        if (!preparado) throw new Error('No hay ningún cambio de ORBY_V4 que commitear');
        ctx.ficheros = preparado.split(/\r?\n/).filter(Boolean);
        emisor.log(`${ctx.ficheros.length} fichero(s) preparados:`, 'info');
        for (const f of ctx.ficheros) emisor.log(`  ${f}`, 'info');
      },
    },
    {
      titulo: 'Commit',
      async ejecutar(ctx) {
        await ejecutar('git', ['commit', '-m', mensaje.trim()], { emisor });
        ctx.extra.commit = await leer('git', ['rev-parse', '--short', 'HEAD'], { emisor });
        emisor.log(`Commit ${ctx.extra.commit}`, 'info');
      },
    },
    {
      titulo: 'Push',
      async ejecutar(ctx) {
        const rama = await leer('git', ['rev-parse', '--abbrev-ref', 'HEAD'], { emisor });
        // -u para que una rama local recién creada quede con su remota puesta y
        // el siguiente push no exija repetir el nombre.
        await ejecutar('git', ['push', '-u', 'origin', rama], { emisor });
        ctx.extra.rama = rama;
        emisor.log(`Empujado a origin/${rama}`, 'info');
      },
    }
  );

  return new Pipeline({ nombre: `Commit y push: ${mensaje.trim()}`, dry: false, emisor, pasos });
}

module.exports = {
  RAIZ,
  ORBYGUI,
  MODOS,
  HERRAMIENTAS,
  infoGit,
  ejecutar,
  leer,
  matarArbol,
  abrirNavegador,
  lanzar,
  parar,
  estadoProceso,
  Pipeline,
  pipelineFirmware,
  pipelineApp,
  pipelineFlashear,
  pipelineHerramienta,
  pipelineSubirCambios,
};
