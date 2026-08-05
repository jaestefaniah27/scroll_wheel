const fs = require('fs');
const path = require('path');
const { app } = require('electron');

// Ajustes que solo viven en el PC (el firmware no sabe nada de aplicaciones de
// Windows). Se guardan en la carpeta de datos de usuario, no junto al código,
// para que sobrevivan a una reinstalación.

const DEFAULTS = {
  autoProfile: {
    enabled: false,
    rules: [],      // [{ id, match, profile, field }]
    fallback: null, // perfil al que volver cuando nada encaja (null = no tocar)
  },
  // Variaciones de perfil por aplicación: solo las diferencias respecto al
  // perfil base. Viven aquí porque el teclado no sabe qué app tienes delante.
  // [{ id, profile, name, match, field, keys, rotary, labels }]
  profileVariants: [],

  // Macros (secuencias) que ejecuta el PC al recibir MACRO:<id> por CDC. El
  // firmware solo guarda el id en el campo del modificador de la tecla o del
  // mando; la secuencia en sí vive aquí, no en el teclado.
  // [{ id, actions: [{ type: 'mouse_position', x, y } | { type: 'mouse_click', button: 'left'|'middle'|'right' }
  //                 | { type: 'delay', ms } | { type: 'key', code } | { type: 'center_mouse' /* legado */ }] }]
  macros: [],

  // Complementos instalados: si están activos y sus ajustes propios. El código
  // de cada uno vive en userData/plugins/<id>/, no aquí (ver electron/plugins.js).
  // Lo que decida el usuario sobre ellos sí vive aquí para que sobreviva a
  // desinstalar y volver a instalar el complemento.
  // { "<id>": { enabled: true, settings: { ... } } }
  plugins: {},

  // Calibración del dibujo de la rueda: depende de cómo estén montados el imán
  // y el marcador de la tapa, así que es del PC y no del firmware.
  // Medidos sobre el Orby montado: el giro va espejado y el marcador de la tapa
  // queda 62° adelantado respecto al cero del sensor.
  wheelDial: {
    invert: true,
    offsetDeg: 62,
    marker: 'dot',  // 'dot' | 'line'
  },

  // Espejo de lo que hay en el teclado (perfiles, páginas, etiquetas, atajos,
  // mandos, rueda e iconos OLED), en el mismo formato que una copia de
  // seguridad. Permite abrir la app sin el Orby delante (en modo lectura) y,
  // con él conectado, evita descargar los perfiles que no han cambiado:
  // GET_HASH dice cuáles son. Lo escribe src/mirror.js.
  // { savedAt, snapshot: {...}, icons: { "perfil:pagina:hueco": hex } }
  deviceMirror: null,
};

// Claves que se sustituyen enteras en vez de fusionarse. El espejo describe un
// estado completo: fusionándolo, un icono borrado o un perfil que ya no está
// seguirían apareciendo en el archivo para siempre.
const REPLACE_WHOLE = new Set(['deviceMirror']);

let cache = null;

function file() {
  return path.join(app.getPath('userData'), 'orby-config.json');
}

function merge(base, patch) {
  const out = { ...base };
  for (const [key, value] of Object.entries(patch || {})) {
    out[key] = (value && typeof value === 'object' && !Array.isArray(value) && !REPLACE_WHOLE.has(key))
      ? merge(base[key] || {}, value)
      : value;
  }
  return out;
}

function load() {
  if (cache) return cache;
  try {
    cache = merge(DEFAULTS, JSON.parse(fs.readFileSync(file(), 'utf8')));
  } catch {
    cache = structuredClone(DEFAULTS);
  }
  return cache;
}

function save(patch) {
  cache = merge(load(), patch);
  try {
    fs.writeFileSync(file(), JSON.stringify(cache, null, 2), 'utf8');
  } catch (err) {
    console.error('No se pudo guardar la configuración local:', err.message);
  }
  return cache;
}

module.exports = { load, save };
