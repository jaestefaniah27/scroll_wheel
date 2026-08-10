// Copia en el renderer de los complementos instalados.
//
// El código de los complementos corre en el proceso principal (ver
// electron/plugins.js): aquí solo llega su descripción —qué acciones ofrecen y
// qué ajustes piden— para poder pintar los botones del editor de perfiles y las
// tarjetas de Ajustes. Se guarda en memoria porque el editor la consulta en
// cada repintado y no puede esperar a una llamada asíncrona por botón.

let installed = [];
let cargado = false;
const listeners = new Set();

export async function refresh() {
  try {
    installed = await window.orby.plugins.list();
  } catch {
    installed = [];
  }
  cargado = true;
  for (const cb of listeners) cb();
  return installed;
}

export const init = refresh;

// Quien se apunta después de que la lista ya haya llegado se pierde el aviso y
// se queda enseñando "ningún complemento" para siempre. Se le da al momento en
// vez de depender de en qué orden se inicialicen las vistas.
export function onChange(cb) {
  listeners.add(cb);
  if (cargado) cb();
}

export function all() {
  return installed;
}

// Los que pueden ofrecer acciones ahora mismo. Uno desactivado o que no cargó
// sigue en la lista de Ajustes (para poder reactivarlo o ver el error), pero no
// debe aparecer como opción asignable a una tecla.
export function active() {
  return installed.filter((p) => p.enabled && !p.error);
}

export function byId(id) {
  return installed.find((p) => p.id === id) || null;
}

// `target`: 'key' (tecla), 'click' (pulsación de encoder) o 'turn' (giro).
// Un mismo complemento puede ofrecer cosas distintas según dónde se asigne: la
// lámpara se enciende con una tecla, pero el brillo necesita sentido de giro.
export function actionsFor(plugin, target) {
  return (plugin?.actions || []).filter((a) => a.targets.includes(target));
}

// Complementos que tienen algo que ofrecer en ese sitio, para no pintar una
// pestaña vacía.
export function withActionsFor(target) {
  return active().filter((p) => actionsFor(p, target).length > 0);
}

export function findAction(pluginId, op) {
  const plugin = byId(pluginId);
  const action = (plugin?.actions || []).find((a) => a.op === op) || null;
  return { plugin, action };
}

// Visores: solo tienen sentido en una tecla con pantalla propia, así que a
// diferencia de actionsFor no llevan `target` (ver docs/PLUGINS.md).
export function viewsFor(plugin) {
  return plugin?.views || [];
}

export function withViews() {
  return active().filter((p) => p.hasRead && viewsFor(p).length > 0);
}

export function findView(pluginId, op) {
  const plugin = byId(pluginId);
  const view = viewsFor(plugin).find((v) => v.op === op) || null;
  return { plugin, view };
}

// Valor en vivo de un visor (ver live-oled.js). `{ ok:false }` cubre tanto un
// fallo de la lectura como el complemento desactivado o sin conexión: al que
// sondea cada dos segundos no le interesa por qué, solo si hay valor o no.
export async function readView(pluginId, op) {
  try {
    return await window.orby.plugins.read(pluginId, op);
  } catch {
    return { ok: false };
  }
}

// Texto para un paso ya guardado: es lo que se lee en la réplica del teclado y
// en el resumen del inspector. El complemento puede haberse desinstalado y el
// paso seguir ahí, así que hay que saber decirlo también.
export function describeStep(step) {
  if (!step) return 'Sin asignar';
  const { plugin, action } = findAction(step.plugin, step.op);
  if (!plugin) return `Complemento «${step.plugin}» (no instalado)`;

  if (!action) {
    // No es una acción de un clic: puede ser un visor (pantalla en vivo).
    const { view } = findView(step.plugin, step.op);
    if (view) return `${plugin.name}: ${view.label.toLowerCase()} (pantalla)`;
    return `${plugin.name}: acción desconocida`;
  }

  // Un valor exacto (Fijar brillo...) lleva el número elegido al lado.
  if (action.value && Number.isFinite(step.value)) {
    return `${action.label} (${step.value})`;
  }

  // Solo los giros llevan cantidad, y su signo es lo que distingue subir de
  // bajar: enseñarlo evita tener que abrir el mando para saber cuál es cuál.
  if (action.targets.includes('turn') && Number(step.value)) {
    const signo = Number(step.value) < 0 ? '−' : '+';
    return `${action.label} ${signo}${Math.abs(Number(step.value))}`;
  }
  return `${plugin.name}: ${action.label.toLowerCase()}`;
}
