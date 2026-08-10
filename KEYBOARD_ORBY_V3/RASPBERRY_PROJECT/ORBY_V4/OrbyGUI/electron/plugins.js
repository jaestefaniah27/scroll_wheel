// Anfitrión de complementos (plugins).
//
// La app base no sabe encender lámparas, ni hablar con un aire acondicionado, ni
// con un servidor de música: solo sabe que el teclado dispara acciones. Un
// complemento es lo que traduce "esta tecla" a "esto de fuera", y vive en la
// carpeta de datos del usuario, no dentro de la app: la instalación por defecto
// de OrbyGUI no trae ninguno.
//
// Un complemento es una carpeta con un `plugin.json` y un módulo de Node. El
// módulo corre en ESTE proceso, con todo lo que eso implica (red, disco, hijos):
// no hay caja de arena. Por eso instalar uno pide confirmación explícita, igual
// que instalar un programa. Ver docs/PLUGINS.md para el contrato completo.

const fs = require('fs');
const path = require('path');
const os = require('os');
const { execFile } = require('child_process');
const { app, dialog, shell } = require('electron');
const config = require('./config');

// Se sube cuando el contrato deje de ser compatible hacia atrás. Un complemento
// declara contra qué versión se escribió y se rechaza si no coincide, en vez de
// dejar que reviente a mitad de una pulsación con un error incomprensible.
const API_VERSION = 1;

// El id es también el nombre de la carpeta y la clave en la configuración, así
// que se limita a lo que es seguro meter en una ruta.
const ID_RE = /^[a-z0-9][a-z0-9-]{1,38}$/;

// id -> { id, dir, manifest, mod, error }
const loaded = new Map();

function pluginsDir() {
  return path.join(app.getPath('userData'), 'plugins');
}

function ensureDir() {
  const dir = pluginsDir();
  try {
    fs.mkdirSync(dir, { recursive: true });
  } catch (err) {
    console.error('No se pudo crear la carpeta de complementos:', err.message);
  }
  return dir;
}

// --- Estado guardado --------------------------------------------------------
// Lo que el usuario decidió sobre cada complemento (si está activo y sus
// ajustes) vive en la configuración local, no dentro de la carpeta del
// complemento: así desinstalarlo y volver a instalarlo no pierde la dirección
// que costó encontrar, y actualizar el complemento no pisa sus ajustes.

function entry(id) {
  return config.load().plugins?.[id] || {};
}

function saveEntry(id, patch) {
  config.save({ plugins: { [id]: patch } });
}

function settingsOf(id) {
  const rec = loaded.get(id);
  const defaults = {};
  for (const f of rec?.mod?.settings?.fields || []) {
    if (f.default !== undefined) defaults[f.key] = f.default;
  }
  return { ...defaults, ...(entry(id).settings || {}) };
}

function saveSettings(id, patch) {
  saveEntry(id, { settings: patch });
  return settingsOf(id);
}

function isEnabled(id) {
  // Sin nada guardado se considera activo: un complemento recién instalado no
  // debería necesitar un segundo clic para empezar a funcionar.
  return entry(id).enabled !== false;
}

// --- Carga ------------------------------------------------------------------

function readManifest(dir) {
  const raw = JSON.parse(fs.readFileSync(path.join(dir, 'plugin.json'), 'utf8'));
  if (!ID_RE.test(String(raw.id || ''))) {
    throw new Error('el campo "id" falta o tiene caracteres no permitidos (a-z, 0-9 y guiones)');
  }
  if (!raw.name) throw new Error('falta el campo "name"');
  if (Number(raw.apiVersion) !== API_VERSION) {
    throw new Error(`está hecho para la API de complementos v${raw.apiVersion || '?'}, `
                  + `y esta versión de OrbyGUI habla la v${API_VERSION}`);
  }
  const main = raw.main || 'main.js';
  if (!fs.existsSync(path.join(dir, main))) throw new Error(`no se encuentra su archivo principal (${main})`);
  return { ...raw, main };
}

// Lo que el complemento recibe. Deliberadamente pequeño: todo lo que necesita
// de la app pasa por aquí, así que ampliar el contrato es una decisión
// consciente y no algo que se cuele por accidente.
function makeApi(id, dir) {
  return {
    id,
    dir,
    settings: () => settingsOf(id),
    setSettings: (patch) => saveSettings(id, patch),
    log: (...args) => console.log(`[${id}]`, ...args),
    error: (...args) => console.error(`[${id}]`, ...args),
  };
}

function loadOne(dir) {
  let manifest = null;
  try {
    manifest = readManifest(dir);
  } catch (err) {
    console.error(`Complemento en ${dir}:`, err.message);
    return null;
  }

  const rec = { id: manifest.id, dir, manifest, mod: null, error: null };

  try {
    const entryPath = path.join(dir, manifest.main);
    // Un complemento reinstalado o actualizado tiene el mismo camino: sin
    // limpiar la caché de require, Node devolvería el módulo viejo.
    dropFromRequireCache(dir);
    const exported = require(entryPath);
    // Se admite tanto una fábrica (lo normal: así el complemento se queda con
    // su `api` en un cierre) como un objeto suelto para casos triviales.
    rec.mod = typeof exported === 'function' ? exported(makeApi(manifest.id, dir)) : exported;
    if (!rec.mod || typeof rec.mod !== 'object') throw new Error('no exporta nada utilizable');
  } catch (err) {
    rec.error = String(err.message || err);
    console.error(`Complemento "${manifest.id}":`, rec.error);
  }

  loaded.set(rec.id, rec);
  if (!rec.error) migrateLegacy(rec);
  return rec;
}

function dropFromRequireCache(dir) {
  const prefix = path.resolve(dir) + path.sep;
  for (const key of Object.keys(require.cache)) {
    if (key.startsWith(prefix)) delete require.cache[key];
  }
}

function loadAll() {
  const dir = ensureDir();
  let names = [];
  try {
    names = fs.readdirSync(dir, { withFileTypes: true }).filter((d) => d.isDirectory()).map((d) => d.name);
  } catch {
    return;
  }
  for (const name of names) loadOne(path.join(dir, name));
}

// --- Migración de lo que antes venía de serie -------------------------------
// La lámpara vivía dentro de la app antes de que existieran los complementos.
// Un complemento puede declarar en su manifiesto qué dejó atrás para que sus
// pasos guardados y sus ajustes sigan funcionando al instalarlo:
//   "legacy": { "stepType": "lamp", "configKey": "lamp" }
function migrateLegacy(rec) {
  const legacy = rec.manifest.legacy;
  if (!legacy) return;
  const cfg = config.load();
  const patch = {};

  if (legacy.stepType) {
    let tocado = false;
    const macros = (cfg.macros || []).map((m) => ({
      ...m,
      actions: (m.actions || []).map((a) => {
        if (a.type !== legacy.stepType) return a;
        tocado = true;
        const { type, ...resto } = a;
        return { type: 'plugin', plugin: rec.id, ...resto };
      }),
    }));
    if (tocado) patch.macros = macros;
  }

  // Los ajustes viejos se copian solo si el complemento no tiene ya los suyos:
  // reinstalarlo no debe resucitar una dirección que el usuario ya cambió.
  if (legacy.configKey && cfg[legacy.configKey] && !entry(rec.id).settings) {
    patch.plugins = { [rec.id]: { settings: { ...cfg[legacy.configKey] } } };
  }

  // `undefined` no llega al JSON, que es justo lo que se quiere: la clave vieja
  // desaparece del archivo en vez de quedarse ahí para siempre.
  if (legacy.configKey && cfg[legacy.configKey]) patch[legacy.configKey] = undefined;

  if (Object.keys(patch).length) {
    config.save(patch);
    console.log(`Complemento "${rec.id}": migrada la configuración anterior a los complementos`);
  }
}

// --- Descriptores para el renderer ------------------------------------------
// Solo datos: el renderer no puede recibir funciones por IPC, y tampoco debe
// poder llamar al código del complemento por su cuenta.

// Rango de un valor exacto (Fijar brillo, Fijar color...). Sin esto la acción
// se trata como las de un solo clic: sin `value` no hay nada que elegir antes
// de asignarla.
function valueDescriptor(v) {
  if (!v || !Number.isFinite(v.min) || !Number.isFinite(v.max)) return null;
  return {
    min: Number(v.min),
    max: Number(v.max),
    step: Number.isFinite(v.step) && v.step > 0 ? Number(v.step) : 1,
    default: Number.isFinite(v.default) ? Number(v.default) : Number(v.min),
  };
}

function actionDescriptors(rec) {
  const list = Array.isArray(rec.mod?.actions) ? rec.mod.actions : [];
  return list
    .filter((a) => a && a.op && a.label)
    .map((a) => ({
      op: String(a.op),
      label: String(a.label),
      // 'key' = tecla, 'click' = pulsación del encoder, 'turn' = giro.
      targets: Array.isArray(a.targets) && a.targets.length ? a.targets.map(String) : ['key'],
      step: Number.isFinite(a.step) ? Number(a.step) : 0,
      hint: a.hint ? String(a.hint) : '',
      value: valueDescriptor(a.value),
    }));
}

// Visores: lo que un complemento puede enseñar en la pantalla de una tecla sin
// que haga falta pulsarla (ver read() más abajo y src/live-oled.js).
function viewDescriptors(rec) {
  const list = Array.isArray(rec.mod?.views) ? rec.mod.views : [];
  return list
    .filter((v) => v && v.op && v.label)
    .map((v) => ({
      op: String(v.op),
      label: String(v.label),
      icon: v.icon ? String(v.icon) : 'oled',
    }));
}

function settingsDescriptor(rec) {
  const s = rec.mod?.settings;
  if (!s) return null;
  return {
    description: s.description ? String(s.description) : '',
    hasTest: typeof rec.mod.test === 'function',
    fields: (s.fields || []).filter((f) => f && f.key).map((f) => ({
      key: String(f.key),
      label: String(f.label || f.key),
      type: ['text', 'number', 'password', 'toggle', 'select'].includes(f.type) ? f.type : 'text',
      placeholder: f.placeholder ? String(f.placeholder) : '',
      hint: f.hint ? String(f.hint) : '',
      options: Array.isArray(f.options)
        ? f.options.map((o) => ({ value: String(o.value), label: String(o.label ?? o.value) }))
        : [],
    })),
  };
}

function descriptor(rec) {
  const m = rec.manifest;
  return {
    id: rec.id,
    name: String(m.name),
    version: String(m.version || '—'),
    description: String(m.description || ''),
    author: String(m.author || ''),
    // Nombre de un icono de src/icons.js; si no existe se pinta el genérico.
    icon: String(m.icon || 'plug'),
    enabled: isEnabled(rec.id),
    error: rec.error,
    actions: rec.error ? [] : actionDescriptors(rec),
    views: rec.error ? [] : viewDescriptors(rec),
    hasRead: !rec.error && typeof rec.mod.read === 'function',
    settings: rec.error ? null : settingsDescriptor(rec),
    values: rec.error ? {} : settingsOf(rec.id),
  };
}

function list() {
  return [...loaded.values()]
    .map(descriptor)
    .sort((a, b) => a.name.localeCompare(b.name, 'es'));
}

// --- Ejecución --------------------------------------------------------------
// Llega desde macros.js con el paso tal cual lo guardó el editor:
// { type: 'plugin', plugin: '<id>', op: '<accion>', value?: <número> }

async function runStep(step) {
  const rec = loaded.get(step?.plugin);
  if (!rec) {
    console.warn(`Paso de complemento "${step?.plugin}": no está instalado`);
    return;
  }
  if (rec.error) {
    console.warn(`Complemento "${rec.id}" no cargó: ${rec.error}`);
    return;
  }
  if (!isEnabled(rec.id)) return;   // desactivado a propósito: silencio, no error
  if (typeof rec.mod.run !== 'function') {
    console.warn(`Complemento "${rec.id}": no exporta run()`);
    return;
  }

  try {
    await rec.mod.run({ op: step.op, value: Number(step.value) || 0 }, settingsOf(rec.id));
  } catch (err) {
    console.error(`Complemento "${rec.id}", acción "${step.op}":`, err.message || err);
  }
}

// Para el visor de una tecla (ver src/live-oled.js): pide el valor actual sin
// tocar nada. A diferencia de test(), esto lo llama un temporizador cada par de
// segundos, así que sin conexión o con el complemento desactivado se calla en
// vez de acumular avisos en la consola.
async function read(id, op) {
  const rec = loaded.get(id);
  if (!rec || rec.error || !isEnabled(rec.id)) return { ok: false };
  if (typeof rec.mod.read !== 'function') return { ok: false };
  try {
    const res = await rec.mod.read(op, settingsOf(id));
    const value = Number(res?.value);
    return Number.isFinite(value) ? { ok: true, value } : { ok: false };
  } catch (err) {
    console.error(`Complemento "${rec.id}", visor "${op}":`, err.message || err);
    return { ok: false };
  }
}

async function test(id, values) {
  const rec = loaded.get(id);
  if (!rec) return { ok: false, error: 'no está instalado' };
  if (rec.error) return { ok: false, error: rec.error };
  if (typeof rec.mod.test !== 'function') return { ok: false, error: 'este complemento no tiene comprobación' };
  try {
    const res = await rec.mod.test({ ...settingsOf(id), ...(values || {}) });
    return res?.ok ? { ok: true, detail: String(res.detail || 'Responde') }
                   : { ok: false, error: String(res?.error || 'sin respuesta') };
  } catch (err) {
    return { ok: false, error: String(err.message || err) };
  }
}

// --- Instalar / desinstalar -------------------------------------------------

// Descomprimir sin añadir una dependencia: Windows ya trae Expand-Archive, y
// esta app solo corre en Windows (ver foreground.js). Si algún día deja de ser
// así, la carpeta suelta sigue funcionando en cualquier sistema.
function unzip(src, dest) {
  return new Promise((resolve, reject) => {
    // Expand-Archive mira la extensión y se niega si no es .zip, así que un
    // .orbyplugin (que por dentro es un zip) se le pasa con nombre de zip.
    let entrada = src;
    if (path.extname(src).toLowerCase() !== '.zip') {
      entrada = path.join(dest, '__paquete.zip');
      fs.copyFileSync(src, entrada);
    }
    const q = (s) => `'${String(s).replace(/'/g, "''")}'`;
    execFile('powershell.exe', [
      '-NoProfile', '-NonInteractive', '-Command',
      `Expand-Archive -LiteralPath ${q(entrada)} -DestinationPath ${q(dest)} -Force`,
    ], (err) => {
      if (entrada !== src) fs.rmSync(entrada, { force: true });
      if (err) reject(new Error('no se pudo descomprimir el archivo'));
      else resolve();
    });
  });
}

// Un .zip puede traer el plugin.json en la raíz o dentro de una única carpeta
// (que es lo que sale al comprimir la carpeta del complemento con el
// explorador). Se admiten los dos.
function findManifestDir(dir) {
  if (fs.existsSync(path.join(dir, 'plugin.json'))) return dir;
  const subs = fs.readdirSync(dir, { withFileTypes: true }).filter((d) => d.isDirectory());
  if (subs.length === 1) {
    const inner = path.join(dir, subs[0].name);
    if (fs.existsSync(path.join(inner, 'plugin.json'))) return inner;
  }
  return null;
}

async function install(win) {
  // Un complemento es una carpeta; el .zip es solo la forma cómoda de
  // repartirlo. En Windows un mismo diálogo no puede ofrecer archivos y
  // carpetas a la vez (Electron se queda con el selector de carpetas y el
  // filtro de .zip no llega a verse), así que se pregunta antes cuál de los dos
  // es en vez de dejar fuera media forma de instalar.
  const origen = await dialog.showMessageBox(win, {
    type: 'question',
    buttons: ['Cancelar', 'Archivo .zip', 'Carpeta'],
    defaultId: 1,
    cancelId: 0,
    title: 'Instalar complemento',
    message: '¿Cómo tienes el complemento?',
    detail: 'Lo normal es un archivo .zip descargado. Si lo estás desarrollando, elige la carpeta '
          + 'que contiene su plugin.json.',
    noLink: true,
  });
  if (origen.response === 0) return { ok: false, canceled: true };
  const comoArchivo = origen.response === 1;

  const { canceled, filePaths } = await dialog.showOpenDialog(win, {
    title: comoArchivo ? 'Elegir el archivo del complemento' : 'Elegir la carpeta del complemento',
    properties: [comoArchivo ? 'openFile' : 'openDirectory'],
    filters: comoArchivo ? [{ name: 'Complemento de Orby', extensions: ['zip', 'orbyplugin'] }] : [],
    buttonLabel: 'Instalar',
  });
  if (canceled || !filePaths?.length) return { ok: false, canceled: true };

  const picked = filePaths[0];
  let temp = null;
  let sourceDir = picked;

  try {
    if (fs.statSync(picked).isFile()) {
      temp = fs.mkdtempSync(path.join(os.tmpdir(), 'orby-plugin-'));
      await unzip(picked, temp);
      sourceDir = findManifestDir(temp) || temp;
    }

    const manifest = readManifest(sourceDir);
    const yaEsta = loaded.get(manifest.id);

    // Un complemento corre con los mismos permisos que la app: puede leer y
    // escribir archivos, salir a la red y lanzar programas. Quien lo instala
    // tiene que saberlo, y tiene que ser una decisión suya y no un efecto
    // secundario de haber pulsado "Instalar" sin leer.
    const { response } = await dialog.showMessageBox(win, {
      type: 'warning',
      buttons: ['Cancelar', yaEsta ? 'Actualizar' : 'Instalar'],
      defaultId: 0,
      cancelId: 0,
      title: 'Instalar complemento',
      message: `${manifest.name} ${manifest.version || ''}`.trim(),
      detail: `${manifest.description || 'Sin descripción.'}\n\n`
            + `Autor: ${manifest.author || 'desconocido'}\n\n`
            + 'Un complemento es un programa que se ejecuta dentro de OrbyGUI con tus mismos '
            + 'permisos: puede acceder a tus archivos, salir a internet y abrir otros programas. '
            + 'Instala solo complementos cuyo origen conozcas.'
            + (yaEsta ? '\n\nYa tienes este complemento instalado: se sustituirá por esta versión.' : ''),
      noLink: true,
    });
    if (response !== 1) return { ok: false, canceled: true };

    const dest = path.join(ensureDir(), manifest.id);
    if (fs.existsSync(dest)) {
      dropFromRequireCache(dest);
      fs.rmSync(dest, { recursive: true, force: true });
    }
    fs.cpSync(sourceDir, dest, { recursive: true });

    const rec = loadOne(dest);
    if (!rec) return { ok: false, error: 'el complemento no se pudo leer después de copiarlo' };
    // Instalarlo es querer usarlo: si estaba desactivado de una vez anterior,
    // vuelve a activarse.
    saveEntry(rec.id, { enabled: true });
    return { ok: true, plugin: descriptor(rec) };
  } catch (err) {
    return { ok: false, error: String(err.message || err) };
  } finally {
    if (temp) fs.rmSync(temp, { recursive: true, force: true });
  }
}

function uninstall(id) {
  const rec = loaded.get(id);
  if (!rec) return { ok: false, error: 'no está instalado' };
  try {
    // Los complementos que dejan algo corriendo (temporizadores, conexiones)
    // pueden soltarlo aquí. Node no sabe descargar un módulo ya cargado, así
    // que sin esto seguiría vivo hasta que se cierre la app.
    if (typeof rec.mod?.dispose === 'function') rec.mod.dispose();
  } catch (err) {
    console.error(`Complemento "${id}" al desinstalar:`, err.message);
  }
  try {
    dropFromRequireCache(rec.dir);
    fs.rmSync(rec.dir, { recursive: true, force: true });
  } catch (err) {
    return { ok: false, error: String(err.message || err) };
  }
  loaded.delete(id);
  // El estado guardado se queda: los pasos de las macros que lo usaban siguen
  // ahí, y reinstalarlo recupera sus ajustes en vez de empezar de cero.
  return { ok: true };
}

function setEnabled(id, enabled) {
  if (!loaded.has(id)) return { ok: false, error: 'no está instalado' };
  saveEntry(id, { enabled: Boolean(enabled) });
  return { ok: true, enabled: isEnabled(id) };
}

function openFolder() {
  return shell.openPath(ensureDir());
}

module.exports = {
  API_VERSION,
  loadAll,
  list,
  runStep,
  read,
  test,
  install,
  uninstall,
  setEnabled,
  saveSettings,
  settingsOf,
  openFolder,
};
