// Saca el descriptor que produce HOY electron/plugins.js para lampdesk.
//
// No se reescribe la lógica: se extraen las funciones del fichero real y se evalúan, para
// que la comparación con el port de Rust no dependa de que yo haya transcrito bien.

import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';

const BASE = '/home/user/scroll_wheel/KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI';
const fuente = readFileSync(`${BASE}/electron/plugins.js`, 'utf8');

// Extrae `function <nombre>(...) { ... }` contando llaves, que es más fiable que una
// expresión regular con anidamiento.
function extrae(nombre) {
  const inicio = fuente.indexOf(`function ${nombre}(`);
  if (inicio === -1) throw new Error(`no encuentro function ${nombre}`);
  let i = fuente.indexOf('{', inicio);
  let nivel = 0;
  for (let j = i; j < fuente.length; j++) {
    if (fuente[j] === '{') nivel++;
    else if (fuente[j] === '}') { nivel--; if (nivel === 0) return fuente.slice(inicio, j + 1); }
  }
  throw new Error(`no cierra function ${nombre}`);
}

const funciones = ['valueDescriptor', 'actionDescriptors', 'viewDescriptors',
                   'settingsDescriptor', 'descriptor'].map(extrae).join('\n\n');

// El complemento de verdad, con la API de anfitrión mínima que necesita su fábrica.
const require_ = createRequire(`${BASE}/plugins/lampdesk/`);
const fabrica = require_(`${BASE}/plugins/lampdesk/main.js`);
const mod = fabrica({ id: 'lampdesk', dir: '.', settings: () => ({}), setSettings() {}, log() {}, error() {} });

const manifest = JSON.parse(readFileSync(`${BASE}/plugins/lampdesk/plugin.json`, 'utf8'));

// Los dos ganchos que descriptor() usa del resto del módulo.
const contexto = {
  isEnabled: () => true,
  settingsOf: () => ({}),
};

const construir = new Function('rec', 'isEnabled', 'settingsOf', `
  ${funciones}
  return descriptor(rec);
`);

// El manifiesto viejo tenía apiVersion 1 y version 1.0.0; para comparar solo la FORMA del
// descriptor se usan los del manifiesto nuevo, que es lo que produce el port.
const rec = {
  id: 'lampdesk',
  manifest: { ...manifest, version: '2.0.0' },
  mod,
  error: null,
};

const d = construir(rec, contexto.isEnabled, contexto.settingsOf);
console.log(JSON.stringify(d, null, 2));
