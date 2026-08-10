// Actualizar el firmware del teclado desde la app.
//
// El RP2040 no se actualiza por el puerto serie: se reinicia en el cargador de
// la ROM, aparece como una unidad USB llamada RPI-RP2, y flashear es copiar un
// .uf2 dentro. Todo el trabajo de este módulo es automatizar esos tres pasos y
// no dejar al usuario a medias si alguno falla.
//
// Los .uf2 salen de las releases del repositorio etiquetadas `fw-vX.Y` (ver
// .github/workflows/firmware.yml). Son releases distintas de las de la app: el
// teclado y la app se publican por separado, y cuál va con cuál lo decide
// OrbyGUI/src/compat.js, que es quien pasa aquí el `maxFw`.
//
// Van marcadas como **prerelease**, y no por estar a medias: comparten
// repositorio con las de la app, y `releases/latest` de GitHub devuelve la más
// reciente de todas sin mirar la etiqueta. Publicar un firmware después de una
// versión de la app dejaba a electron-updater buscando el `latest.yml` de la app
// dentro de una release de firmware, que no lo lleva ("Cannot find latest.yml in
// the latest release artifacts"): la app se quedaba sin actualizarse hasta la
// siguiente publicación. Una prerelease no cuenta para `releases/latest`, y aquí
// no molesta porque este listado las incluye a propósito.

const fs = require('fs');
const os = require('os');
const path = require('path');
const { EventEmitter } = require('events');

const TAG_PREFIX = 'fw-v';

// Cuánto se espera a que salga la unidad RPI-RP2. Con el comando BOOTSEL son
// dos o tres segundos; el margen largo es para el camino manual, en el que hay
// que desenchufar el cable, buscar el botón y volver a enchufar.
const DRIVE_TIMEOUT_MS = 90_000;
const DRIVE_POLL_MS = 400;

// --- Versiones ---------------------------------------------------------------

// "4.10" es posterior a "4.9". Mismo criterio que compareFw en src/compat.js:
// comparar por partes, no como número decimal.
function compareFw(a, b) {
  const partsOf = (v) => String(v ?? '').split('.').map((n) => parseInt(n, 10) || 0);
  const x = partsOf(a);
  const y = partsOf(b);
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    const d = (x[i] || 0) - (y[i] || 0);
    if (d) return d < 0 ? -1 : 1;
  }
  return 0;
}

// --- La unidad del cargador --------------------------------------------------

// La letra la asigna Windows y cambia de un día para otro, así que se busca por
// lo que la Pico deja en la raíz. La etiqueta del volumen (RPI-RP2) obligaría a
// llamar a PowerShell; INFO_UF2.TXT se ve con un fs.existsSync y además
// confirma que es una RP2 y no cualquier pendrive que se llame igual.
function findBootDrive() {
  if (process.platform === 'win32') {
    for (let i = 0; i < 26; i++) {
      const root = `${String.fromCharCode(67 + i)}:\\`; // de C: a Z:
      const info = path.join(root, 'INFO_UF2.TXT');
      try {
        if (fs.existsSync(info) && fs.readFileSync(info, 'utf8').includes('RP2')) return root;
      } catch {
        // Unidad ocupada o sin permisos: no es la nuestra, seguir buscando.
      }
    }
    return null;
  }

  // Linux y macOS montan por etiqueta bajo estos sitios.
  const bases = ['/media', `/media/${os.userInfo().username}`, '/run/media', '/Volumes'];
  for (const base of bases) {
    let entries = [];
    try { entries = fs.readdirSync(base); } catch { continue; }
    for (const name of entries) {
      const root = path.join(base, name);
      try {
        if (fs.existsSync(path.join(root, 'INFO_UF2.TXT'))) return root;
      } catch { /* igual que arriba */ }
    }
  }
  return null;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function waitForBootDrive(timeoutMs, isCancelled) {
  const limit = Date.now() + timeoutMs;
  while (Date.now() < limit) {
    if (isCancelled?.()) throw new Error('Actualización cancelada');
    const drive = findBootDrive();
    if (drive) return drive;
    await sleep(DRIVE_POLL_MS);
  }
  throw new Error('El teclado no ha aparecido en modo de actualización');
}

// --- El módulo ---------------------------------------------------------------

class FirmwareUpdater extends EventEmitter {
  // `repo` es "owner/nombre"; `serial` es la instancia de OrbySerial, que hace
  // falta para pedirle al teclado que se reinicie él solo.
  constructor({ repo, serial }) {
    super();
    this.repo = repo;
    this.serial = serial;
    this.cancelled = false;

    // status: 'idle' | 'checking' | 'downloading' | 'bootsel' | 'flashing' | 'done' | 'error'
    this.state = {
      status: 'idle',
      current: null,     // lo que dice el teclado conectado
      latest: null,      // la última release compatible con esta app
      available: false,  // hay una más nueva que la del teclado
      percent: 0,
      manual: false,     // true mientras se espera a que el usuario pulse BOOTSEL
      error: null,
    };
  }

  _set(patch) {
    this.state = { ...this.state, ...patch };
    this.emit('state', this.state);
    return this.state;
  }

  // Qué firmware hay publicado que esta app sepa usar. `maxFw` sale de
  // FW_RECOMMENDED en src/compat.js: publicar un firmware más nuevo que la app
  // no debe ofrecerse aquí, porque la app no sabría hablar con él.
  async check({ maxFw, currentFw } = {}) {
    this._set({ status: 'checking', error: null, current: currentFw ?? this.state.current });

    try {
      const res = await fetch(`https://api.github.com/repos/${this.repo}/releases?per_page=50`, {
        headers: { Accept: 'application/vnd.github+json', 'User-Agent': 'OrbyGUI' },
      });
      if (!res.ok) throw new Error(`GitHub respondió ${res.status}`);

      // Las prerelease SÍ entran: es como se publican las de firmware (ver la
      // cabecera). Solo se descartan los borradores, que nadie puede descargar.
      const candidatos = (await res.json())
        .filter((r) => !r.draft && r.tag_name?.startsWith(TAG_PREFIX))
        .map((r) => ({
          version: r.tag_name.slice(TAG_PREFIX.length),
          // El .uf2, no el .sha256 que lo acompaña.
          asset: (r.assets || []).find((a) => a.name.endsWith('.uf2')),
        }))
        .filter((r) => r.asset)
        .filter((r) => !maxFw || compareFw(r.version, maxFw) <= 0)
        .sort((a, b) => compareFw(b.version, a.version));

      const latest = candidatos[0] || null;
      const current = currentFw ?? this.state.current;

      return this._set({
        status: 'idle',
        latest,
        current,
        available: !!(latest && current && compareFw(latest.version, current) > 0),
      });
    } catch (err) {
      return this._set({ status: 'error', error: `No se pudo consultar las versiones: ${err.message}` });
    }
  }

  // Descarga el .uf2 a un temporal. Se comprueba el tamaño porque una descarga
  // cortada por la mitad es un .uf2 válido a ojos del cargador: copiaría medio
  // firmware y dejaría el teclado sin arrancar.
  async _download(asset) {
    this._set({ status: 'downloading', percent: 0, error: null });

    const res = await fetch(asset.browser_download_url, { headers: { 'User-Agent': 'OrbyGUI' } });
    if (!res.ok) throw new Error(`La descarga falló (${res.status})`);

    const total = asset.size || Number(res.headers.get('content-length')) || 0;
    const trozos = [];
    let leidos = 0;

    // Con getReader() y no con `for await`: el cuerpo de fetch es un
    // ReadableStream del estándar web, y que además sea iterable depende de la
    // implementación que traiga Electron. El lector sí está garantizado.
    const lector = res.body.getReader();
    for (;;) {
      const { done, value } = await lector.read();
      if (done) break;
      if (this.cancelled) { await lector.cancel(); throw new Error('Actualización cancelada'); }
      trozos.push(Buffer.from(value));
      leidos += value.length;
      if (total) this._set({ percent: Math.round((leidos / total) * 100) });
    }

    if (total && leidos !== total) {
      throw new Error(`La descarga llegó incompleta (${leidos} de ${total} bytes)`);
    }

    const destino = path.join(os.tmpdir(), asset.name);
    fs.writeFileSync(destino, Buffer.concat(trozos));
    return destino;
  }

  // Copiar el .uf2 a la unidad del cargador. El fallo esperado no es que no
  // copie: es que la Pico se reinicia en cuanto recibe el último bloque, así
  // que el cierre del fichero se encuentra la unidad ya desaparecida y Windows
  // devuelve EIO/EPERM/ENOENT. Eso *es* el éxito. Lo que de verdad delata un
  // fallo es que la unidad siga ahí después.
  _flash(uf2Path, drive) {
    this._set({ status: 'flashing', percent: 100 });
    const destino = path.join(drive, path.basename(uf2Path));
    try {
      fs.copyFileSync(uf2Path, destino);
    } catch (err) {
      if (!['EIO', 'EPERM', 'ENOENT', 'EBUSY', 'EINVAL'].includes(err.code)) throw err;
    }
  }

  // El proceso entero. `viaBootsel` dice si el teclado sabe reiniciarse solo
  // (bandera BOOTSEL=1 del handshake); si no, se le pide al usuario que lo haga
  // a mano y se espera igual.
  async update({ viaBootsel = false } = {}) {
    if (['downloading', 'bootsel', 'flashing'].includes(this.state.status)) return this.state;

    this.cancelled = false;
    const asset = this.state.latest?.asset;
    if (!asset) return this._set({ status: 'error', error: 'No hay ninguna versión que instalar' });

    try {
      const uf2 = await this._download(asset);

      // Primero se mira si ya está en modo cargador: puede que el usuario lo
      // haya dejado así, y entonces no hay ni teclado al que pedirle nada.
      let drive = findBootDrive();

      if (!drive) {
        this._set({ status: 'bootsel', manual: !viaBootsel });
        // El comando no contesta nada más: el teclado se reinicia acto seguido
        // y el puerto desaparece. La confirmación de que ha funcionado es que
        // aparezca la unidad, no una respuesta por el CDC.
        if (viaBootsel) this.serial?.sendCommand('BOOTSEL');
        drive = await waitForBootDrive(DRIVE_TIMEOUT_MS, () => this.cancelled);
      }

      this._flash(uf2, drive);

      // El teclado tarda un par de segundos en volver a enumerar. El puerto lo
      // reencuentra el autoescaneo de serial.js por su cuenta; aquí solo se
      // deja de decir "flasheando" cuando ya no está la unidad del cargador.
      for (let i = 0; i < 20 && findBootDrive(); i++) await sleep(500);

      return this._set({ status: 'done', percent: 100, available: false, manual: false });
    } catch (err) {
      return this._set({ status: 'error', error: err.message, manual: false });
    }
  }

  cancel() {
    this.cancelled = true;
    if (['downloading', 'bootsel'].includes(this.state.status)) {
      this._set({ status: 'idle', percent: 0, manual: false });
    }
    return this.state;
  }
}

module.exports = { FirmwareUpdater, compareFw, findBootDrive };
