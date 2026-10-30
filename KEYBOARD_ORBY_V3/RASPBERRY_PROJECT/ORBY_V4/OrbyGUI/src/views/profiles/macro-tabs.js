// Las pestañas del inspector que editan una macro de forma acotada —"App",
// "Texto" y las secciones de energía y de complemento de "Multimedia"— más los
// ayudantes que la pestaña "Secuencia" usa para montar sus pasos.
//
// Todas comparten mecanismo con "Secuencia" (MACRO_MODIFIER + id, ver
// macros-store.js); lo que cambia es cuánto dejan editar.

import { state, markDirty, layerIndex, scrollFor } from '../../store.js';
import * as device from '../../device.js';
import { MACRO_MODIFIER, ROTARY_TYPES, CONSUMER_TURN_PAIRS, consumerPairFor } from '../../hid-keys.js';
import { toast, requireDevice } from '../../ui.js';
import { icon } from '../../icons.js';
import * as variants from '../../variants.js';
import * as plugins from '../../plugins.js';
import { DEFAULT_STEP_DELAY_MS, ROTARY_DOWN_SLOTS, ROTARY_TWIN } from './constants.js';
import { view, currentProfile, selectedRotarySlot, editingVariant, currentAction,
         currentRotary, currentMacroId, isActiveView, rerender, pluginValuePick } from './view-state.js';
import { macroById, nextMacroId, ensureMacro, savePCMacros,
         isPowerMacro, pluginMacroOf, macroDeviceEligible, MACRO_MAX_STEPS_DEVICE } from './macros-store.js';
import { applyKeymap, applyRotary, applyRotaryPair } from './keys.js';
import { escape, getCurrentWindowInfo } from './util.js';

// --- Selector deslizante de una acción de valor exacto ----------------------
// "Fijar brillo"/"Fijar color" no se asignan con un solo clic: primero hay que
// elegir el número. Se abre este selector en el mismo sitio del botón (ver
// profiles.js) y solo al confirmar se escribe la macro.

// `current`: el valor ya guardado en el paso, si el hueco ya era esta misma
// acción (para no arrancar siempre del valor por defecto al reabrir).
export function openPluginValuePick(pluginId, op, mode, current) {
  const { action: spec } = plugins.findAction(pluginId, op);
  if (!spec?.value) return;
  pluginValuePick.open = true;
  pluginValuePick.plugin = pluginId;
  pluginValuePick.op = op;
  pluginValuePick.mode = mode;
  pluginValuePick.value = Number.isFinite(current) ? current : spec.value.default;
  rerender();
}

export function setPluginValuePickValue(value) {
  pluginValuePick.value = value;
}

export function cancelPluginValuePick() {
  pluginValuePick.open = false;
  rerender();
}

export function confirmPluginValuePick() {
  const { plugin, op, mode, value } = pluginValuePick;
  pluginValuePick.open = false;
  if (mode === 'rotary') setRotaryPluginAction(plugin, op, value);
  else setPluginAction(plugin, op, value);
}

// ¿Son el mismo gesto repetido? Solo tiene sentido para pasos de una sola
// pulsación/clic: repetir un movimiento de ratón o una posición no significa
// nada (cada uno lleva sus propias coordenadas).
export function sameStep(a, b) {
  if (a.type !== b.type) return false;
  if (a.type === 'hotkey') return a.modifier === b.modifier && a.keycode === b.keycode;
  if (a.type === 'mouse_click') return a.button === b.button;
  return false;
}

// --- Pasos de una secuencia -------------------------------------------------
// En los datos, la espera entre dos pasos es una acción más ({type:'delay'})
// que va entre ambos: así la ejecutan el PC y el teclado. En el editor, en
// cambio, cada paso lleva dentro la espera que le sigue, para no dedicarle
// una fila entera. stepUnits() pasa de una forma a la otra y buildActions()
// deshace el camino.
//
// Una unidad es { idx, action, delayIdx }: idx es la posición del paso en
// m.actions y delayIdx la de su espera posterior (null en el último paso, o
// si por lo que sea no la tiene). Las esperas sueltas al principio o
// repetidas no se pueden crear desde el editor; si vienen de una secuencia
// antigua se suman a la del paso de antes, que es lo que tardaban en total.
export function stepUnits(actions) {
  const units = [];
  actions.forEach((a, i) => {
    if (a.type !== 'delay') { units.push({ idx: i, action: a, delayIdx: null, extra: 0 }); return; }
    const u = units[units.length - 1];
    if (!u) return;
    if (u.delayIdx === null) u.delayIdx = i;
    else u.extra += a.ms || 0;
  });
  return units;
}

// Monta m.actions desde unidades ya ordenadas. Cada paso menos el último
// lleva su espera; un paso que pasa a estar en medio sin tener una (era el
// último) estrena la de por defecto.
function buildActions(actions, units) {
  const out = [];
  units.forEach((u, k) => {
    out.push(u.action);
    if (k === units.length - 1) return;
    const ms = u.delayIdx !== null ? (actions[u.delayIdx].ms || 0) + u.extra : DEFAULT_STEP_DELAY_MS;
    out.push({ type: 'delay', ms });
  });
  return out;
}

// Dónde se meten los pasos nuevos: antes del paso número `at` (0 = al
// principio). null es "al final", lo de siempre. Va atado a la macro: elegir
// otra tecla vuelve a añadir al final sin tener que acordarse de quitarlo.
let insertPoint = null; // { macroId, at }

export function seqInsertAt() {
  const id = currentMacroId();
  if (!insertPoint || insertPoint.macroId !== id) return null;
  const m = macroById(id);
  const total = m ? stepUnits(m.actions).length : 0;
  return insertPoint.at >= total ? null : insertPoint.at;
}

export function setSeqInsertAt(at) {
  const id = currentMacroId();
  insertPoint = (id === null || at === null) ? null : { macroId: id, at };
  rerender();
}

// Añade un paso donde marque el punto de inserción (por defecto, al final), y
// con él la espera por defecto que tiene que haber entre dos pasos seguidos.
// Tras añadir, el punto avanza detrás del paso nuevo: así varios seguidos
// quedan en el orden en que se añaden, igual que al escribir.
//
// Si el paso nuevo es exactamente el mismo gesto que el de justo antes (mismo
// atajo, mismo botón de clic...), no se añade una línea más: se suma a su
// contador de repeticiones. Así "flecha abajo ×15" o un doble clic no
// acumulan pasos de verdad (ni cuentan contra el límite de la macro en el
// propio teclado), y grabar la misma tecla varias veces seguidas no llena la
// lista de líneas idénticas.
export function seqAddAction(action) {
  const id = currentMacroId();
  if (id === null) return;
  const m = ensureMacro(id);
  const units = stepUnits(m.actions);
  const at = seqInsertAt() ?? units.length;
  const prev = units[at - 1];

  if (prev && sameStep(prev.action, action)) {
    prev.action.count = (prev.action.count || 1) + (action.count || 1);
    savePCMacros(id);
    rerender();
    return;
  }

  const step = { ...action, count: action.count || 1 };
  units.splice(at, 0, { idx: -1, action: step, delayIdx: null, extra: 0 });
  m.actions = buildActions(m.actions, units);
  if (insertPoint?.macroId === id && at < units.length - 1) insertPoint.at = at + 1;
  savePCMacros(id);
  rerender();
}

// Lleva el paso número `from` a la posición `to`, con su espera: la espera
// se ve dentro del paso, así que se mueve con él.
export function seqMoveStep(from, to) {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  if (!m || from === to) return;
  const units = stepUnits(m.actions);
  if (!units[from] || to < 0 || to >= units.length) return;
  const [u] = units.splice(from, 1);
  units.splice(to, 0, u);
  m.actions = buildActions(m.actions, units);
  savePCMacros(id);
  rerender();
}

// Espera que sigue al paso que está en m.actions[stepIdx]. Si no tiene una
// (una secuencia vieja con dos pasos pegados), se crea.
export function seqSetDelayAfter(stepIdx, ms) {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  if (!m) return;
  const u = stepUnits(m.actions).find((x) => x.idx === stepIdx);
  if (!u) return;
  if (u.delayIdx !== null) m.actions[u.delayIdx].ms = ms;
  else m.actions.splice(stepIdx + 1, 0, { type: 'delay', ms });
  savePCMacros(id);
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
//
// `value`, si se da, es el número elegido en el selector deslizante (ver
// pluginValuePick en view-state.js) para una acción de valor exacto ("Fijar
// brillo"...). Las demás acciones —y los visores— no lo llevan.
export function setPluginAction(pluginId, op, value) {
  const action = currentAction();
  const previo = pluginMacroOf(action.modifier === MACRO_MODIFIER ? action.keycode : -1);
  // Se reutiliza la macro solo si ya era de ESTE complemento: cambiar de uno a
  // otro es cambiar de acción, no editarla.
  const id = previo?.plugin === pluginId ? action.keycode : nextMacroId();
  const m = ensureMacro(id);
  const step = { type: 'plugin', plugin: pluginId, op };
  if (Number.isFinite(value)) step.value = value;
  m.actions = [step];
  savePCMacros(id);
  applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
}

// ¿Este hueco está girando al revés de lo que le tocaría por su sentido?
export function giroInvertido(base, step) {
  const porDefecto = ROTARY_DOWN_SLOTS.has(base) ? -1 : 1;
  return Math.sign(Number(step?.value) || 0) === -porDefecto;
}

// Lo mismo que giroInvertido, pero para una pareja multimedia (Volumen,
// Brillo, Zoom): no hay un valor con signo que mirar, así que se compara el
// keycode que hay en el hueco con el que le tocaría por su sentido.
export function consumerGiroInvertido(base, action) {
  const pair = consumerPairFor(action?.keycode);
  if (!pair) return false;
  const porDefecto = ROTARY_DOWN_SLOTS.has(base) ? pair.down : pair.up;
  return action.keycode !== porDefecto;
}

// Le da la vuelta al par entero del mando, no solo al hueco que se está
// editando: si el encoder está montado del revés, lo que hacía cada sentido
// pasa al otro. Sirve tanto para el giro de un complemento como para una
// pareja multimedia (Volumen, Brillo, Zoom): en los dos casos son dos huecos
// con acciones opuestas por construcción.
//
// Intercambia las dos acciones enteras (no el signo del valor guardado): así
// no depende de que el otro sentido tuviera ya algo asignado, que era el bug
// de antes — invertir con la pulsación configurada solo en un lado dejaba el
// otro sin tocar, y el próximo giro que se asignara ahí salía con el mismo
// signo en vez del contrario.
export function invertirGiroPar() {
  const prof = currentProfile();
  const base = selectedRotarySlot();
  const twin = base === null ? undefined : ROTARY_TWIN[base];
  if (!prof || base === null || twin === undefined) return;

  const variant = editingVariant();
  const accBase = variants.effectiveRotary(prof, variant, base, view.layer);
  const accTwin = variants.effectiveRotary(prof, variant, twin, view.layer);
  applyRotaryPair(base, { ...accTwin }, twin, { ...accBase });
}

// `value`: igual que en setPluginAction, el número del selector deslizante
// para una acción de valor exacto asignada al clic del mando (los giros de
// verdad, con `step`, nunca llevan esto: van por setGiroPluginAction).
export function setRotaryPluginAction(pluginId, op, value) {
  const { action: spec } = plugins.findAction(pluginId, op);
  if (!spec) return;

  const base = selectedRotarySlot();
  // Solo los giros llevan cantidad; la pulsación de un encoder es un evento
  // suelto, sin dirección que aplicarle.
  const esGiro = spec.targets.includes('turn') && spec.step > 0;

  if (esGiro && base !== null && ROTARY_TWIN[base] !== undefined) {
    setGiroPluginAction(base, pluginId, op, spec);
    return;
  }

  const action = currentRotary();
  const previo = (action.type === ROTARY_TYPES.KEY && action.modifier === MACRO_MODIFIER)
    ? pluginMacroOf(action.keycode) : null;
  const id = previo?.plugin === pluginId ? action.keycode : nextMacroId();

  const m = ensureMacro(id);
  const step = { type: 'plugin', plugin: pluginId, op };
  if (Number.isFinite(value)) step.value = value;
  m.actions = [step];
  savePCMacros(id);
  applyRotary({ type: ROTARY_TYPES.KEY, modifier: MACRO_MODIFIER, keycode: id });
}

// El giro de un encoder son dos huecos físicos (CW y CCW) para un mismo
// control ("Brillo de la lámpara", por ejemplo): se escriben siempre juntos,
// con signos opuestos por construcción, en vez de dejar que cada sentido se
// edite por separado. Así es imposible que los dos acaben subiendo (o
// bajando) a la vez, que era justo el bug: antes cada hueco calculaba su
// propio signo al vuelo, y una asignación hecha antes de que el otro sentido
// existiera se quedaba corta.
function setGiroPluginAction(base, pluginId, op, spec) {
  const twin = ROTARY_TWIN[base];
  const prof = currentProfile();
  const variant = editingVariant();

  const posSlot = ROTARY_DOWN_SLOTS.has(base) ? twin : base;
  const negSlot = ROTARY_DOWN_SLOTS.has(base) ? base : twin;

  // Reutiliza la macro del hueco si ya era de este mismo complemento (igual
  // que hacía la versión de un solo hueco), para no ir dejando macros sueltas
  // cada vez que se cambia de "Brillo" a "Color".
  const idFor = (slot) => {
    const previa = variants.effectiveRotary(prof, variant, slot, view.layer);
    const paso = (previa.type === ROTARY_TYPES.KEY && previa.modifier === MACRO_MODIFIER)
      ? pluginMacroOf(previa.keycode) : null;
    return paso?.plugin === pluginId ? previa.keycode : nextMacroId();
  };

  // ensureMacro() de cada id justo después de calcularlo, no los dos juntos al
  // final: nextMacroId() mira qué ids ya existen en la lista, y si ningún
  // sentido tenía todavía una macro de este complemento, pedir los dos ids
  // ANTES de reservar el primero le daba a los dos el mismo número. Las dos
  // escrituras cabían en la misma macro y la segunda (la negativa) se comía a
  // la primera: los dos sentidos del encoder acababan bajando.
  const idPos = idFor(posSlot);
  ensureMacro(idPos).actions = [{ type: 'plugin', plugin: pluginId, op, value: spec.step }];

  const idNeg = idFor(negSlot);
  ensureMacro(idNeg).actions = [{ type: 'plugin', plugin: pluginId, op, value: -spec.step }];

  savePCMacros(idPos);
  savePCMacros(idNeg);

  applyRotaryPair(
    posSlot, { type: ROTARY_TYPES.KEY, modifier: MACRO_MODIFIER, keycode: idPos },
    negSlot, { type: ROTARY_TYPES.KEY, modifier: MACRO_MODIFIER, keycode: idNeg },
  );
}

// Mismo mecanismo que setGiroPluginAction pero para un par multimedia (Volumen,
// Brillo, Zoom): sin macro de por medio, así que basta con escribir Subir/Bajar
// en el hueco que corresponda según el sentido de montaje del encoder.
export function setRotaryConsumerPair(pairId) {
  const pair = CONSUMER_TURN_PAIRS.find((p) => p.id === pairId);
  const base = selectedRotarySlot();
  const twin = base === null ? undefined : ROTARY_TWIN[base];
  if (!pair || base === null || twin === undefined) return;

  const posSlot = ROTARY_DOWN_SLOTS.has(base) ? twin : base;
  const negSlot = ROTARY_DOWN_SLOTS.has(base) ? base : twin;

  applyRotaryPair(
    posSlot, { type: ROTARY_TYPES.CONSUMER, modifier: 0, keycode: pair.up },
    negSlot, { type: ROTARY_TYPES.CONSUMER, modifier: 0, keycode: pair.down },
  );
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

// Se pide una sola vez aunque la lista llegue vacía o falle: repintar al
// terminar vuelve a pasar por aquí, y mirando solo installedApps.length una
// lista vacía se pedía y repintaba sin fin, colgando la ventana.
let installedAppsAsked = false;
export function ensureInstalledApps() {
  if (installedAppsLoading || installedAppsAsked) return;
  installedAppsAsked = true;
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

// Quién escribe el texto. El teclado lo teclea él solo (firmware 4.20) si cada carácter
// tiene tecla en la distribución española y cabe en una secuencia; si no, lo escribe
// el PC por unicode, que sale igual con cualquier distribución.
function textLocationNote(macroId) {
  const m = macroId === null ? null : macroById(macroId);
  if (m && macroDeviceEligible(m)) {
    return `<p class="setting-desc">
      Con esta app abierta lo escribe ella (de golpe, más rápido y con cualquier distribución);
      sin ella lo teclea el propio teclado, como si el PC tuviera la distribución
      <strong>española</strong>: con otra, los símbolos saldrían cambiados.
      Los saltos de línea se mandan como Intro y los tabuladores como Tab.
    </p>`;
  }
  return `<p class="setting-desc">
      Lo escribe el PC, así que necesita esta app abierta (vale con el icono de la bandeja).
      Al ir por unicode y no por códigos de tecla, sale igual con cualquier distribución: eñes,
      acentos y símbolos incluidos. Los textos de más de cuatro letras entran de golpe por el
      portapapeles y un Ctrl+V (se restaura lo que tuvieras copiado); donde Ctrl+V no pegue
      —la consola clásica, algún juego— no aparecerá nada. Para que lo teclee el teclado sin la
      app hace falta el firmware 4.20 y un texto sin emojis ni alfabetos ajenos al español, de
      unos ${MACRO_MAX_STEPS_DEVICE * 3} caracteres como mucho.
    </p>`;
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

    ${textLocationNote(macroId)}

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
  // El punto de inserción va por número de paso: si se borra uno de antes,
  // tiene que correrse para seguir señalando el mismo hueco.
  if (insertPoint?.macroId === id && m.actions[index]?.type !== 'delay') {
    const step = stepUnits(m.actions).findIndex((u) => u.idx === index);
    if (step >= 0 && step < insertPoint.at) insertPoint.at--;
  }
  m.actions.splice(index, 1);
  normalizeSequence(m);
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
