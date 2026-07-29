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

  // Calibración del dibujo de la rueda: depende de cómo estén montados el imán
  // y el marcador de la tapa, así que es del PC y no del firmware.
  // Medidos sobre el Orby montado: el giro va espejado y el marcador de la tapa
  // queda 62° adelantado respecto al cero del sensor.
  wheelDial: {
    invert: true,
    offsetDeg: 62,
    marker: 'dot',  // 'dot' | 'line'
  },
};

let cache = null;

function file() {
  return path.join(app.getPath('userData'), 'orby-config.json');
}

function merge(base, patch) {
  const out = { ...base };
  for (const [key, value] of Object.entries(patch || {})) {
    out[key] = (value && typeof value === 'object' && !Array.isArray(value))
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
