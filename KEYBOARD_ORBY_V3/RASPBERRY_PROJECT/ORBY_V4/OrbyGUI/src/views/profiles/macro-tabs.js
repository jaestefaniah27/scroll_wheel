// Las pestañas del inspector que editan una macro de forma acotada —"App",
// "Texto" y las secciones de energía y de complemento de "Multimedia"— más los
// ayudantes que la pestaña "Secuencia" usa para montar sus pasos.
//
// Todas comparten mecanismo con "Secuencia" (MACRO_MODIFIER + id, ver
// macros-store.js); lo que cambia es cuánto dejan editar.

import { state, markDirty, layerIndex, scrollFor } from '../../store.js';
import * as device from '../../device.js';
import { MACRO_MODIFIER, ROTARY_TYPES } from '../../hid-keys.js';
import { toast, requireDevice } from '../../ui.js';
import { icon } from '../../icons.js';
import * as variants from '../../variants.js';
import * as plugins from '../../plugins.js';
import { DEFAULT_STEP_DELAY_MS, ROTARY_DOWN_SLOTS, ROTARY_TWIN } from './constants.js';
import { view, currentProfile, selectedRotarySlot, editingVariant, currentAction,
         currentRotary, currentMacroId, isActiveView, rerender } from './view-state.js';
import { macroById, nextMacroId, ensureMacro, savePCMacros,
         isPowerMacro, isPluginMacro, pluginMacroOf } from './macros-store.js';
import { applyKeymap, applyRotary } from './keys.js';
import { escape, getCurrentWindowInfo } from './util.js';

// ¿Son el mismo gesto repetido? Solo tiene sentido para pasos de una sola
// pulsación/clic: repetir un movimiento de ratón o una posición no significa
// nada (cada uno lleva sus propias coordenadas).
export function sameStep(a, b) {
  if (a.type !== b.type) return false;
  if (a.type === 'hotkey') return a.modifier === b.modifier && a.keycode === b.keycode;
  if (a.type === 'mouse_click') return a.button === b.button;
  return false;
}

// Añade un paso y, si ya había alguno antes, la espera por defecto entre los
// dos: así cada dos pasos consecutivos tienen siempre un hueco configurable
// de por medio, sin tener que pedirlo aparte.
//
// Si el paso nuevo es exactamente el mismo gesto que el último de la lista
// (mismo atajo, mismo botón de clic...), no se añade una línea más: se suma
// a su contador de repeticiones. Así "flecha abajo ×15" o un doble clic no
// acumulan pasos de verdad (ni cuentan contra el límite de la macro en el
// propio teclado), y grabar la misma tecla varias veces seguidas no llena la
// lista de líneas idénticas.
export function seqAddAction(action) {
  const id = currentMacroId();
  if (id === null) return;
  const m = ensureMacro(id);
  const last = m.actions[m.actions.length - 1];
  if (last && last.type !== 'delay' && sameStep(last, action)) {
    last.count = (last.count || 1) + (action.count || 1);
    savePCMacros(id);
    rerender();
    return;
  }
  if (m.actions.length && m.actions[m.actions.length - 1].type !== 'delay') {
    m.actions.push({ type: 'delay', ms: DEFAULT_STEP_DELAY_MS });
  }
  m.actions.push({ ...action, count: action.count || 1 });
  savePCMacros(id);
  rerender();
}

// --- Pestaña "App": una tecla que abre directamente una app o un archivo ---
// Reutiliza el mismo mecanismo de macro que "Secuencia" (MACRO_MODIFIER + id,
// ver más arriba), pero limitado a un único paso "open_app": no hay lista de
// pasos que gestionar, solo el objetivo. Ver isAppMacro (macros-store.js) y setTab (profiles.js) para cómo se
// distingue de una secuencia de verdad.
export function currentAppStep() {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  if (!m) return null;
  if (!m.actions.length) m.actions.push({ type: 'open_app', target: '' });
  return m.actions[0];
}

export function setAppKind(kind) {
  const step = currentAppStep();
  if (!step) return;
  step.kind = kind;
  savePCMacros(currentMacroId());
  rerender();
}

// --- Pestaña "Texto": una tecla que escribe un texto tal cual --------------
// Mismo mecanismo que "App" (MACRO_MODIFIER + id, un único paso), aquí de tipo
// "text". Lo mete el PC por el camino unicode del sistema (ver typeText en
// electron/macros.js), no el teclado: el firmware solo sabe mandar usages HID,
// que dependen de la distribución que tenga puesta el usuario.
export function currentTextStep() {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  if (!m) return null;
  if (!m.actions.length) m.actions.push({ type: 'text', text: '' });
  return m.actions[0];
}

// --- Pestaña "Multimedia", sección de energía ------------------------------
// Mismo mecanismo que "App" (macro de un único paso, ver isPowerMacro), pero
// sin pestaña propia: se asigna al vuelo con un solo clic, igual que una
// acción multimedia normal. Si la tecla ya era una acción de energía se
// reutiliza la misma macro (solo cambia el modo); si no, se crea una nueva.
export function setPowerAction(mode) {
  const action = currentAction();
  const id = (action.modifier === MACRO_MODIFIER && isPowerMacro(action.keycode))
    ? action.keycode : nextMacroId();
  const m = ensureMacro(id);
  m.actions = [{ type: 'system_power', mode }];
  savePCMacros(id);
  applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
}

// --- Secciones de complemento de la pestaña Multimedia ---------------------
// Calcado de setPowerAction: macro de un solo paso, un clic y listo.
export function setPluginAction(pluginId, op) {
  const action = currentAction();
  const previo = pluginMacroOf(action.modifier === MACRO_MODIFIER ? action.keycode : -1);
  // Se reutiliza la macro solo si ya era de ESTE complemento: cambiar de uno a
  // otro es cambiar de acción, no editarla.
  const id = previo?.plugin === pluginId ? action.keycode : nextMacroId();
  const m = ensureMacro(id);
  m.actions = [{ type: 'plugin', plugin: pluginId, op }];
  savePCMacros(id);
  applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
}

// ¿Este hueco está girando al revés de lo que le tocaría por su sentido?
export function giroInvertido(base, step) {
  const porDefecto = ROTARY_DOWN_SLOTS.has(base) ? -1 : 1;
  return Math.sign(Number(step?.value) || 0) === -porDefecto;
}

// Le da la vuelta al par entero del mando, no solo al hueco que se está
// editando: si el encoder está montado del revés, lo que hay que invertir son
// los dos sentidos a la vez, y hacerlo hueco por hueco deja a medias un estado
// en el que los dos giros suben o los dos bajan.
//
// Solo toca el paso de la macro, que vive en el PC: el hueco sigue apuntando a
// la misma macro, así que no hay nada que escribir en el teclado.
export function invertirGiroPlugin() {
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return;

  const variant = editingVariant();
  const huecos = [base, ROTARY_TWIN[base]].filter((b) => b !== undefined);

  for (const b of huecos) {
    const accion = variants.effectiveRotary(prof, variant, b, view.layer);
    if (accion?.modifier !== MACRO_MODIFIER || !isPluginMacro(accion.keycode)) continue;
    const paso = macroById(accion.keycode).actions[0];
    // Encender/apagar no tiene sentido de giro: sin cantidad no hay signo que
    // dar la vuelta.
    if (!Number(paso.value)) continue;
    paso.value = -Number(paso.value);
    savePCMacros(accion.keycode);
  }

  rerender();
}

export function setRotaryPluginAction(pluginId, op) {
  const { action: spec } = plugins.findAction(pluginId, op);
  if (!spec) return;

  const action = currentRotary();
  const previo = (action.type === ROTARY_TYPES.KEY && action.modifier === MACRO_MODIFIER)
    ? pluginMacroOf(action.keycode) : null;
  const id = previo?.plugin === pluginId ? action.keycode : nextMacroId();

  // Solo los giros llevan cantidad; la pulsación de un encoder es un evento
  // suelto, sin dirección que aplicarle.
  const esGiro = spec.targets.includes('turn') && spec.step > 0;
  const signo = ROTARY_DOWN_SLOTS.has(selectedRotarySlot()) ? -1 : 1;

  const m = ensureMacro(id);
  m.actions = esGiro
    ? [{ type: 'plugin', plugin: pluginId, op, value: signo * spec.step }]
    : [{ type: 'plugin', plugin: pluginId, op }];
  savePCMacros(id);
  applyRotary({ type: ROTARY_TYPES.KEY, modifier: MACRO_MODIFIER, keycode: id });
}

export async function pickAppTarget(kind) {
  const step = currentAppStep();
  if (!step) return;
  let picked;
  try {
    picked = await window.orby.pickAppOrFile(kind);
  } catch {
    toast('El selector de archivos no está disponible', 'error');
    return;
  }
  if (!picked?.ok) return;
  step.target = picked.path;
  step.kind = kind;
  savePCMacros(currentMacroId());
  rerender();
}

// Toma directamente la app que está en primer plano ahora mismo: mismo
// detector que usa "Añadir app actual" en las variaciones (ver
// getCurrentWindowInfo más arriba), pero aquí hace falta el ejecutable
// entero, no solo el nombre del proceso.
export async function pickAppFocus() {
  const step = currentAppStep();
  if (!step) return;
  let info;
  try {
    info = await getCurrentWindowInfo();
  } catch {
    toast('El detector de aplicaciones no está disponible', 'error');
    return;
  }
  if (!info?.path) {
    toast('No se ha podido saber qué ejecutable es esa ventana', 'error');
    return;
  }
  step.target = info.path;
  step.kind = 'app';
  savePCMacros(currentMacroId());
  rerender();
  toast(`Añadida "${info.process || info.path}"`);
}

// Apps instaladas para el buscador de la pestaña "App": se piden una vez al
// proceso principal (recorre el menú Inicio, ver electron/apps.js) y se
// guardan en memoria; no cambian mientras la app sigue abierta.
export let installedApps = [];
export let installedAppsLoading = false;

export function ensureInstalledApps() {
  if (installedAppsLoading || installedApps.length) return;
  installedAppsLoading = true;
  window.orby.listInstalledApps()
    .then((list) => { installedApps = list || []; })
    .catch(() => { installedApps = []; })
    .finally(() => {
      installedAppsLoading = false;
      if (isActiveView() && (view.tab === 'app' || view.tab === 'sequence')) rerender();
    });
}

// Compartido entre la pestaña "App" y los pasos "Abrir" de una secuencia:
// mismo <datalist>, mismo id, así que el navegador solo necesita uno vivo en
// el DOM a la vez (las dos pestañas nunca se enseñan juntas).
export function installedAppsDatalist() {
  return `<datalist id="installed-apps-list">
    ${installedApps.map((a) => `<option value="${escape(a.target)}" label="${escape(a.name)}"></option>`).join('')}
  </datalist>`;
}

export function renderAppTab(macroId) {
  const step = macroId === null ? { target: '', kind: 'app' } : (macroById(macroId)?.actions?.[0] || { target: '', kind: 'app' });
  const kind = step.kind === 'file' ? 'file' : 'app';

  if (kind === 'app') ensureInstalledApps();

  return `
    <div class="field">
      <span class="field-label">Qué abrir</span>
      <div class="row-inline" style="gap:6px">
        <button class="type-chip ${kind === 'app' ? 'on' : ''}" data-act="app-kind" data-kind="app">Aplicación</button>
        <button class="type-chip ${kind === 'file' ? 'on' : ''}" data-act="app-kind" data-kind="file">Archivo</button>
      </div>
    </div>

    ${kind === 'app' ? `
      <div class="row-inline mt-4" style="gap:6px">
        <button class="secondary-btn" data-act="app-focus">${icon('fit', 16)} App en foco</button>
        <button class="secondary-btn" data-act="app-browse" data-kind="app">${icon('upload', 16)} Examinar…</button>
      </div>

      <label class="field mt-4">
        <span class="field-label">Aplicación</span>
        <input type="text" class="text-input" list="installed-apps-list" data-act="app-target"
               value="${escape(step.target || '')}"
               placeholder="Escribe para buscar entre las instaladas, o usa los botones de arriba">
        ${installedAppsDatalist()}
        ${installedAppsLoading && !installedApps.length
          ? '<span class="setting-desc">Buscando aplicaciones instaladas…</span>' : ''}
      </label>
    ` : `
      <label class="field mt-4">
        <span class="field-label">Archivo</span>
        <div class="row-inline" style="gap:6px">
          <input type="text" class="text-input" style="flex:1" data-act="app-target"
                 value="${escape(step.target || '')}" placeholder="Ruta del archivo">
          <button class="secondary-btn" data-act="app-browse" data-kind="file">
            ${icon('upload', 16)} Examinar…
          </button>
        </div>
      </label>
    `}

    <p class="setting-desc">
      Se ejecuta en el PC al pulsar la tecla (necesita esta app abierta), igual que un paso
      "Abrir" de una secuencia.
    </p>

    <button class="secondary-btn full" data-act="seq-clear">${icon('trash', 16)} Quitar</button>`;
}

// Pestaña "Texto": un solo campo, el texto que escribe la tecla. Para meterlo
// entre clics, esperas u otras teclas está la pestaña Secuencia, que tiene el
// mismo paso ("Escribir texto") junto a los demás.
export function renderTextTab(macroId) {
  const step = macroId === null ? { text: '' } : (macroById(macroId)?.actions?.[0] || { text: '' });

  return `
    <label class="field">
      <span class="field-label">Texto que escribe la tecla</span>
      <textarea class="text-input" rows="4" data-act="text-value"
                placeholder="Por ejemplo tu correo, una firma o un trozo de código">${escape(step.text || '')}</textarea>
    </label>

    <p class="setting-desc">
      Lo escribe el PC, así que necesita esta app abierta (vale con el icono de la bandeja).
      Al ir por unicode y no por códigos de tecla, sale igual con cualquier distribución: eñes,
      acentos y símbolos incluidos. Los textos de más de cuatro letras entran de golpe por el
      portapapeles y un Ctrl+V (se restaura lo que tuvieras copiado), así que da igual lo largos
      que sean; donde Ctrl+V no pegue —la consola clásica, algún juego— no aparecerá nada. Los
      saltos de línea se mandan como Intro y los tabuladores como Tab.
    </p>

    <button class="secondary-btn full" data-act="seq-clear">${icon('trash', 16)} Quitar</button>`;
}

// Abre el selector nativo de archivos para elegir qué abrir con este paso.
// Sin `editIndex`, añade un paso nuevo; con él, sustituye el objetivo del
// paso existente (botón "Examinar" junto a cada paso "Abrir…").
export async function pickOpenTarget(editIndex = null, kind = 'app') {
  let picked;
  try {
    picked = await window.orby.pickAppOrFile(kind);
  } catch {
    toast('El selector de archivos no está disponible', 'error');
    return;
  }
  if (!picked?.ok) return;

  if (editIndex !== null) {
    const step = seqActionAt(editIndex);
    if (step) { step.target = picked.path; step.kind = kind; savePCMacros(currentMacroId()); rerender(); }
  } else {
    seqAddAction({ type: 'open_app', target: picked.path, kind });
  }
}

// Cambia si un paso "Abrir" de la secuencia busca una app o un archivo:
// mismo distingo que en la pestaña App (ver setAppKind), pero por índice de
// paso en vez de sobre el único paso de esa pestaña.
export function setSeqOpenKind(index, kind) {
  const step = seqActionAt(index);
  if (!step) return;
  step.kind = kind;
  savePCMacros(currentMacroId());
  rerender();
}

// Botón "App en foco" de un paso de secuencia: mismo mecanismo que
// pickAppFocus (pestaña App), pero escribe en el paso `index` en vez de en
// el único paso de esa pestaña.
export async function pickSeqOpenFocus(index) {
  const step = seqActionAt(index);
  if (!step) return;
  let info;
  try {
    info = await getCurrentWindowInfo();
  } catch {
    toast('El detector de aplicaciones no está disponible', 'error');
    return;
  }
  if (!info?.path) {
    toast('No se ha podido saber qué ejecutable es esa ventana', 'error');
    return;
  }
  step.target = info.path;
  step.kind = 'app';
  savePCMacros(currentMacroId());
  rerender();
  toast(`Añadida "${info.process || info.path}"`);
}

export function seqRemoveAction(index) {
  const id = currentMacroId();
  if (id === null) return;
  const m = macroById(id);
  if (!m) return;
  m.actions.splice(index, 1);
  normalizeSequence(m);
  savePCMacros(id);
  rerender();
}

// Reordena un paso real (nunca una espera) intercambiándolo con el paso real
// más cercano en esa dirección. Las esperas se quedan donde están: siguen
// marcando el hueco entre "el paso de antes" y "el de después" de esa
// posición de la lista, sea cual sea la acción que acabe ocupándola.
export function seqMoveAction(index, dir) {
  const id = currentMacroId();
  if (id === null) return;
  const m = macroById(id);
  if (!m) return;
  const acts = m.actions;
  if (!acts[index] || acts[index].type === 'delay') return;
  let j = index + dir;
  while (acts[j] && acts[j].type === 'delay') j += dir;
  if (!acts[j]) return;
  [acts[index], acts[j]] = [acts[j], acts[index]];
  savePCMacros(id);
  rerender();
}

// Quita esperas huérfanas después de borrar un paso: ninguna al principio, ni
// al final, ni dos seguidas (podría pasar al borrar el paso que había entre
// ambas).
export function normalizeSequence(m) {
  const out = [];
  for (const a of m.actions) {
    if (a.type === 'delay' && (!out.length || out[out.length - 1].type === 'delay')) continue;
    out.push(a);
  }
  if (out.length && out[out.length - 1].type === 'delay') out.pop();
  m.actions = out;
}

// Paso concreto de la secuencia que se está editando ahora mismo, por índice
// (usado al cambiar el botón de un clic o la duración de una espera).
export function seqActionAt(index) {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  return m?.actions[index] || null;
}

// Constructor de un paso "tecla": el mismo selector de modificadores + tecla
// que la pestaña Atajo, pero como paso de secuencia en vez de la acción de la
// tecla física. Estado transitorio, se resetea la tecla (no los modificadores,
// para encadenar varias con el mismo modificador) tras cada "Añadir".
export let seqKeyBuilder = { modifier: 0, keycode: 0 };

// --- Captura de posición del ratón ------------------------------------------
// A diferencia de la grabación de teclas, aquí el ratón es la entrada: se
// enseña la posición del cursor en pantalla en vivo (viene del proceso
// principal porque puede estar fuera de la ventana) y Esc fija el paso.
export let lastMousePos = { x: 0, y: 0 };
let posPollTimer = null;
// null = capturando un paso nuevo; si no, índice del paso que se está
// recapturando (el botón de destino junto a cada "Posición de ratón").
export let capturePosEditIndex = null;

// El sondeo lo corta también el repintado general (ver render en profiles.js):
// si se sale del modo por otro sitio —elegir otra tecla, cambiar de pestaña—
// nadie llamaría a stopPositionCapture y se quedaría corriendo de fondo. Va por
// función y no exportando posPollTimer porque una exportación reasignada es de
// solo lectura para quien la importa.
export function cancelPositionPoll() {
  if (!posPollTimer) return;
  clearInterval(posPollTimer);
  posPollTimer = null;
}

export function startPositionCapture(editIndex = null) {
  view.capturing = 'position';
  capturePosEditIndex = editIndex;
  rerender();
  if (posPollTimer) clearInterval(posPollTimer);
  posPollTimer = setInterval(async () => {
    try {
      const p = await window.orby.getMousePosition();
      lastMousePos = p;
      const el = document.getElementById('seq-live-pos');
      if (el) el.textContent = `(${p.x}, ${p.y}) — mueve el ratón y pulsa Esc para fijarla`;
    } catch { /* sin Electron (dev en navegador suelto): no hay nada que sondear */ }
  }, 80);
}

export function stopPositionCapture(commit) {
  if (posPollTimer) { clearInterval(posPollTimer); posPollTimer = null; }
  view.capturing = false;
  const editIndex = capturePosEditIndex;
  capturePosEditIndex = null;

  if (!commit) { rerender(); return; }

  if (editIndex !== null) {
    const step = seqActionAt(editIndex);
    if (step) {
      step.type = 'mouse_position';
      step.x = lastMousePos.x;
      step.y = lastMousePos.y;
      savePCMacros();
    }
    rerender();
  } else {
    seqAddAction({ type: 'mouse_position', x: lastMousePos.x, y: lastMousePos.y });
  }
}

// Botón "Secuencia" del inspector de mando: mismo mecanismo que en las teclas,
// pero sin pestañas, porque el mando ya elige su acción con el type-grid.
export function applyRotaryMacro() {
  if (currentRotary().modifier === MACRO_MODIFIER) return;
  const id = nextMacroId();
  ensureMacro(id);
  applyRotary({ type: ROTARY_TYPES.KEY, modifier: MACRO_MODIFIER, keycode: id });
}
