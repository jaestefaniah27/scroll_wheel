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
