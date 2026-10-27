// Las macros del editor: dónde se guardan, cómo se crean y cómo se suben al
// teclado, más los predicados que distinguen unas de otras.
//
// Una "macro" es cualquier acción que no cabe en el firmware como atajo: una
// secuencia de pasos, abrir una app, escribir un texto, una acción de energía,
// una de complemento o una grabación. Todas se guardan igual (un id en la
// configuración del PC, MACRO_MODIFIER + ese id en el hueco del teclado) y se
// distinguen por lo que llevan dentro.

import * as device from '../../device.js';
import { state, markDirty } from '../../store.js';
import { MACRO_MODIFIER, ROTARY_TYPES, describeAction, describeRotary } from '../../hid-keys.js';
import * as plugins from '../../plugins.js';
import * as compat from '../../compat.js';
import { compileMacro, POOL_STEPS } from '../../native/compile.mjs';
import { layoutBlobs, allocate, sectorsUsedBy, crc16, crc32, toHex, SECTOR, ALIGN } from '../../native/pool.mjs';
import { LAYOUTS, defaultLayout } from '../../native/layouts.mjs';

// --- Macros (secuencias ejecutadas en el PC) --------------------------------
// Viven en la configuración local (window.orby.getConfig/setConfig), no en el
// teclado: el firmware solo guarda el id en el campo del modificador de la
// tecla o del mando (MACRO_MODIFIER) y avisa por CDC al pulsarla.
//
// La lista no se exporta: se reasigna al releer la configuración, y una
// exportación reasignada deja a los demás módulos mirando el array viejo. Se
// llega a ella por allMacros() y se modifica por ensureMacro/removeMacro.
let pcMacros = [];

export function allMacros() {
  return pcMacros;
}

// Quita una macro de la lista. Lo usa la pestaña "Grabar" al deshacer la pareja
// de borrado de una grabación.
export function removeMacro(id) {
  pcMacros = pcMacros.filter((m) => m.id !== id);
  savePCMacros(id);
}

export async function loadPCMacros() {
  try {
    const cfg = await window.orby.getConfig();
    pcMacros = cfg?.macros || [];
    if (LAYOUTS[cfg?.textLayout]) textLayout = cfg.textLayout;
  } catch {
    pcMacros = [];
  }
  compiled.clear();
}

// `id`, si se da, es la macro que acaba de cambiar: además de persistir la
// configuración del PC, se intenta subir (o limpiar) su copia en el teclado.
export function savePCMacros(id) {
  // Los pasos se editan en sitio (step.x = …) y a veces se guarda sin id: lo
  // compilado de cualquiera puede haber cambiado.
  compiled.clear();
  window.orby.setConfig({ macros: pcMacros });
  if (id !== undefined) syncMacroToDevice(id);
}

export function macroById(id) {
  return pcMacros.find((m) => m.id === id);
}

export function ensureMacro(id) {
  let m = macroById(id);
  if (!m) {
    m = { id, actions: [] };
    pcMacros.push(m);
    savePCMacros(id);
  }
  return m;
}

// El id más bajo que esté libre, no el siguiente al mayor. Con "el mayor + 1"
// los ids crecían para siempre: borrar y volver a crear macros bastaba para
// pasar de MACRO_MAX_COUNT_DEVICE, y a partir de ahí ninguna macro nueva podía
// vivir en el teclado aunque hubiera hueco de sobra (el firmware indexa un
// array por id, no lleva una lista).
export function nextMacroId() {
  const used = new Set(pcMacros.map((m) => m.id));
  let id = 1;
  while (used.has(id)) id++;
  return id;
}

// --- Reproducción en el propio teclado ---------------------------------
// Qué pasos lleva cada acción en el teclado lo decide src/native/compile.mjs:
// desde el firmware 4.7 eso incluye texto, abrir apps, energía, posición de
// ratón y grabaciones, no solo esperas, atajos y clics. Lo que de verdad no
// tiene traducción (un complemento, un emoji) sigue yendo por el PC.
//
// TODO(homing-absoluto) quedó resuelto con el puntero absoluto del 4.7
// (MSTEP_ABS_MOVE): ya no depende de la velocidad del puntero de Windows. El
// MSTEP_HOME_MOVE del firmware se queda sin usar.
export { MAX_STEPS as MACRO_MAX_STEPS_DEVICE } from '../../native/compile.mjs';

// Cuántas secuencias caben en el teclado. El firmware las guarda en un array
// indexado por id, así que un id fuera de rango no es "una macro más": es un
// comando que el teclado rechaza con ERR:BAD_ARGS, un error que ninguna
// petición espera y que por tanto se comía los 4 s de espera enteros, una vez
// por macro y en serie, al conectar.
// El firmware 4.2 lo anuncia en el handshake (MAXMACROS); con uno anterior se
// asume el 64 de toda la vida, que es lo que valía MACRO_MAX_COUNT.
export const MACRO_CAPACITY_FALLBACK = 64; // == MACRO_MAX_COUNT en main.cpp

export function deviceMacroCapacity() {
  const n = parseInt(state.deviceInfo?.maxmacros ?? '', 10);
  return Number.isFinite(n) && n > 0 ? n : MACRO_CAPACITY_FALLBACK;
}

// Sectores del pool de datos (textos y grabaciones). 0 sin firmware 4.7.
function poolSectors() {
  const n = parseInt(state.deviceInfo?.pool ?? '', 10);
  return Number.isFinite(n) && n > 0 ? n : 0;
}

// Acciones de energía del PC, elegibles desde la pestaña Multimedia (macro de
// un único paso "system_power"). Con el firmware 4.7 también las hace el
// teclado: ver POWER_RUN y compañía en src/native/compile.mjs.
export const POWER_MODE_LABELS = {
  sleep: 'Suspender', hibernate: 'Hibernar', restart: 'Reiniciar',
  shutdown: 'Apagar', lock: 'Bloquear pantalla', logoff: 'Cerrar sesión',
};
// Acciones de complemento. Mismo mecanismo que las acciones de energía: una
// macro de un único paso que ejecuta el PC, aquí llamando al complemento
// instalado que la ofrece (ver electron/plugins.js). La app base no conoce
// ninguna: la lista sale de lo que declare cada complemento.
//
// Cada complemento dice dónde encaja cada acción: las teclas y la pulsación de
// un encoder solo pueden con las órdenes que no tienen dirección, mientras que
// un giro necesita una acción con sentido (el signo lo pone el propio giro).
// Hueco por defecto entre cada repetición de un mismo paso (tecla o clic
// repetido, ver sameStep en macro-tabs.js): se usa mientras el paso no traiga
// uno propio (el campo `gap`, editable a partir de 2 repeticiones). Debe
// coincidir con el mismo nombre en electron/macros.js (ahí se usa para las
// secuencias que corren por el camino del PC, no del teclado).
export const DEFAULT_REPEAT_GAP_MS = 20;

// --- Distribución de teclado del PC --------------------------------------
// Para escribir un texto el teclado manda posiciones de tecla, y qué carácter
// sale lo decide la distribución de Windows. Se guarda en la configuración del
// PC; si nunca se ha elegido, la del idioma del sistema.
let textLayout = defaultLayout(globalThis.navigator?.language);

export function getTextLayout() {
  return textLayout;
}

export function setTextLayout(id) {
  if (!LAYOUTS[id] || id === textLayout) return;
  textLayout = id;
  compiled.clear();
  window.orby.setConfig({ textLayout: id });
  // Todo texto subido con la distribución vieja escribiría otra cosa.
  syncAllMacrosToDevice().catch(() => {});
}

// El escritorio virtual de este PC, para traducir posiciones de ratón al
// puntero absoluto. Solo lo sabe la app de escritorio (getMousePosition trae
// `desk`); en el navegador queda en null y esas posiciones se quedan en el PC.
let currentDesk = null;

async function refreshDesk() {
  try {
    const p = await window.orby.getMousePosition();
    if (p?.desk?.w > 0 && p?.desk?.h > 0) currentDesk = p.desk;
  } catch { /* sin backend */ }
}

// Lo compilado se guarda por macro hasta que algo la cambie: keyNeedsApp se
// pregunta en cada repintado de la rejilla, y una grabación larga son miles de
// eventos.
const compiled = new Map();

function compileFor(m) {
  const native = compat.supports(state.deviceInfo, 'native') && poolSectors() > 0;
  const key = `${m.id}|${native}|${textLayout}|${currentDesk ? 1 : 0}`;
  const hit = compiled.get(m.id);
  if (hit && hit.key === key && hit.macro === m) return hit.result;
  const result = compileMacro(m, { native, layout: textLayout, desk: currentDesk });
  compiled.set(m.id, { key, macro: m, result });
  return result;
}

// Por qué una macro se queda en el PC, o null si la toca el teclado.
export function macroDeviceBlocker(m) {
  if (!m) return 'no existe';
  if (m.id >= deviceMacroCapacity()) return 'no cabe en la memoria de secuencias del teclado';
  const r = compileFor(m);
  return r.ok ? null : r.reason;
}

export function macroDeviceEligible(m) {
  return macroDeviceBlocker(m) === null;
}

// Si la toca el teclado pero con la app abierta la hace el PC (ver
// MSTEP_PREFER_PC en main.cpp).
export function macroPrefersPc(m) {
  return macroDeviceEligible(m) && compileFor(m).preferPc === true;
}

// --- Lo que tiene el teclado ---------------------------------------------
// Los pasos de cada macro tal y como están en el teclado, leídos una vez por
// conexión. Hacen falta para no reescribir lo que ya está igual y para saber
// qué sectores del pool tiene ya cada una.
const onDevice = new Map();
let onDeviceComplete = false;

async function deviceSteps(id) {
  if (onDevice.has(id)) return onDevice.get(id);
  const steps = await device.getMacro(id);
  onDevice.set(id, steps || []);
  return onDevice.get(id);
}

// Quién usa cada sector del pool. Se miran todas las macros del teclado, no
// solo las de este PC: un texto subido desde otro equipo también ocupa sitio.
async function poolOwners() {
  if (!onDeviceComplete) {
    for (let id = 0; id < deviceMacroCapacity(); id++) await deviceSteps(id);
    onDeviceComplete = true;
  }
  const owners = new Map();
  for (const [id, steps] of onDevice) {
    for (const sec of sectorsUsedBy(steps, POOL_STEPS)) owners.set(sec, id);
  }
  return owners;
}

// Si el teclado ya tiene exactamente esos pasos. Con `steps` vacío pregunta por
// "no tiene nada", que es lo que hace falta antes de mandar un MACRO_CLEAR.
//
// Devuelve false si no se puede saber (el teclado no contesta): ante la duda se
// escribe, que es lo que se hacía siempre.
async function matchesDevice(id, steps) {
  let current;
  try {
    current = await deviceSteps(id);
  } catch {
    return false;
  }
  if (!current || current.length !== steps.length) return false;
  return steps.every((s, i) => {
    const c = current[i];
    return c && c.type === s.type && c.a === s.a && c.b === s.b
        && c.repeat === s.repeat && c.gap === s.gap;
  });
}

// Coloca los trozos de pool de una macro y escribe los sectores que no tengan
// ya lo que toca. Rellena `a` (dónde), `b` (cuánto) y `gap` (CRC) de esos
// pasos. false si no hay sitio.
async function placePoolData(id, steps) {
  const withBlob = steps.filter((s) => s.blob);
  if (!withBlob.length) return true;

  const { image, offsets, sectors } = layoutBlobs(withBlob.map((s) => s.blob));
  const first = allocate(id, sectors, await poolOwners(), poolSectors());
  if (first === null) return false;

  withBlob.forEach((s, i) => {
    s.a = (first * SECTOR + offsets[i]) / ALIGN;
    s.b = s.blob.length;
    s.gap = crc16(s.blob) & 0x7FFF;
  });

  for (let k = 0; k < sectors; k += 16) {
    const count = Math.min(16, sectors - k);
    const have = await device.poolHash(first + k, count);
    for (let j = 0; j < count; j++) {
      const sector = first + k + j;
      const want = image.subarray((k + j) * SECTOR, (k + j + 1) * SECTOR);
      if (have[sector] === crc32(want)) continue;
      const got = await device.poolWriteSector(sector, want, toHex);
      if (got !== crc32(want)) throw new Error(`el sector ${sector} del pool no se grabó bien`);
    }
  }
  return true;
}

// Las sincronizaciones van de una en una: se lanzan al editar y al conectar, y
// dos a la vez podrían repartirse el mismo hueco del pool.
let syncQueue = Promise.resolve();

// Sube (o, si ya no es jugable en el teclado, limpia) la copia de una macro en
// el dispositivo. No bloquea al que llama: si falla, la macro se queda
// funcionando por el camino del PC hasta el próximo intento.
//
// Antes de escribir se compara con lo que el teclado ya tiene. Sin eso, la
// resincronización de la conexión reescribía todas las secuencias, marcaba la
// configuración como pendiente y el guardado automático gastaba una escritura
// de Flash del teclado en cada enchufada, sin que hubiera cambiado nada.
export function syncMacroToDevice(id) {
  const run = syncQueue.then(() => syncOne(id));
  syncQueue = run.catch(() => {});
  return run;
}

async function syncOne(id) {
  // Sin esto, un firmware anterior a esta función respondería "ERR:UNKNOWN_CMD"
  // a SET_MACRO_STEP, que ninguna petición espera: cada intento agotaría su
  // tiempo de espera (varios segundos) en vez de fallar al momento.
  if (!state.connected || !compat.supports(state.deviceInfo, 'macros')) return;
  // Un id que no cabe en el teclado nunca llegó a subirse, así que tampoco hay
  // nada que limpiar: mandar MACRO_CLEAR con él solo saca un ERR:BAD_ARGS.
  if (id >= deviceMacroCapacity()) return;
  const m = macroById(id);

  const clear = async () => {
    if (await matchesDevice(id, [])) return;   // ya estaba vacía
    try { await device.macroClear(id); onDevice.set(id, []); } catch { /* sin conexión o firmware viejo: no pasa nada */ }
    markDirty();
  };

  if (!m || !macroDeviceEligible(m)) return clear();

  // Copia: los pasos de pool se rellenan con su sitio, y lo compilado se
  // reutiliza entre llamadas.
  const steps = compileFor(m).steps.map((s) => ({ ...s }));

  try {
    if (!(await placePoolData(id, steps))) {
      console.warn(`[secuencias] la ${id} no cabe en el pool del teclado: se queda en el PC`);
      return clear();
    }
    if (await matchesDevice(id, steps)) return;

    for (let i = 0; i < steps.length; i++) {
      const s = steps[i];
      await device.setMacroStep(id, i, s.type, s.a, s.b, s.repeat, s.gap);
    }
    await device.macroTrunc(id, steps.length);
    onDevice.set(id, steps.map(({ type, a, b, repeat, gap }) => ({ type, a, b, repeat, gap })));
    // Vive en RAM del teclado hasta que se guarde en Flash, igual que un
    // cambio de atajo: sin esto, el aviso de "Guardar en Flash" no se
    // encendería y un apagado se llevaría la secuencia por delante.
    markDirty();
  } catch (e) {
    // El teclado puede no estar conectado o llevar un firmware sin este
    // comando: la macro sigue funcionando por el PC mientras tanto. Lo leído
    // del teclado ya no es de fiar: se vuelve a preguntar la próxima vez.
    onDevice.delete(id);
    onDeviceComplete = false;
    console.warn(`[secuencias] no se pudo subir la ${id}:`, e?.message || e);
  }
}

// Repasa todas las secuencias al conectar y sube solo las que el teclado no
// tenga ya iguales: cubre tanto un teclado recién reflasheado (su Flash de
// secuencias está vacía) como una macro editada mientras estaba desconectado.
//
// Las macros que no puede tocar el teclado (un complemento…) no tienen nada
// que subir, solo habría que limpiar la copia del teclado si alguna vez se
// subió. Cuáles se subieron se apunta en la configuración del PC, así que
// preguntar por las demás es gastar una ida y vuelta por el CDC para oír "no
// tengo nada". Con cien macros eso eran cien preguntas en cada conexión.
//
// La lista apuntada puede no existir (primera vez con esta versión): entonces
// se repasan todas una vez y se apunta el resultado.
export async function syncAllMacrosToDevice() {
  if (!compat.supports(state.deviceInfo, 'macros')) return;
  // Otra conexión, quizá otro teclado: lo que se leyó del anterior no vale.
  onDevice.clear();
  onDeviceComplete = false;
  compiled.clear();
  await loadPCMacros();
  await refreshDesk();

  const known = await uploadedIds();
  for (const m of pcMacros) {
    // Sin nada que subir y sin constancia de que el teclado tenga una copia
    // suya, no hay nada que comprobar.
    if (!macroDeviceEligible(m) && known && !known.includes(m.id)) continue;
    await syncMacroToDevice(m.id);
  }

  const now = pcMacros.filter((m) => macroDeviceEligible(m)).map((m) => m.id);
  window.orby.setConfig({ macrosOnDevice: now });
}

// Los ids que este PC ha subido al teclado, o null si nunca se ha apuntado.
async function uploadedIds() {
  try {
    const cfg = await window.orby.getConfig();
    return Array.isArray(cfg?.macrosOnDevice) ? cfg.macrosOnDevice : null;
  } catch {
    return null;
  }
}

// Una macro "vale" para la pestaña App si es justo lo que esa pestaña sabe
// editar: un único paso de abrir algo. Cualquier otra cosa (varios pasos, u
// otro tipo) es una Secuencia de verdad.
export function isAppMacro(id) {
  const acts = macroById(id)?.actions || [];
  return acts.length === 1 && acts[0].type === 'open_app';
}

// Igual que isAppMacro, pero para la pestaña Texto: un único paso que escribe
// un texto tal cual. Con más pasos (o con el texto mezclado con clics, teclas…)
// ya es una Secuencia, y se edita como tal.
export function isTextMacro(id) {
  const acts = macroById(id)?.actions || [];
  return acts.length === 1 && acts[0].type === 'text';
}

// Igual que isAppMacro, pero para una acción de energía del PC (Suspender,
// Apagar…). Vive en la pestaña Multimedia por ser también una sola pulsación
// sin más ajustes que elegir cuál, aunque por dentro sea una macro de un paso
// (el firmware no puede tocar esto: ver electron/macros.js).
export function isPowerMacro(id) {
  const acts = macroById(id)?.actions || [];
  return acts.length === 1 && acts[0].type === 'system_power';
}

// Igual, para una acción de un complemento.
export function isPluginMacro(id) {
  const acts = macroById(id)?.actions || [];
  return acts.length === 1 && acts[0].type === 'plugin';
}

// El paso de un complemento concreto: sirve para saber si el mando ya está
// dentro de "su" pestaña o si hay que cambiarlo de complemento.
export function pluginMacroOf(id) {
  return isPluginMacro(id) ? macroById(id).actions[0] : null;
}

// describeRotary (hid-keys.js) solo ve el modificador y el código, y ahí todas
// las macros son iguales. Aquí sí se puede mirar qué guarda la macro.
export function describeRotaryFull(action) {
  if (action?.type === ROTARY_TYPES.KEY && action.modifier === MACRO_MODIFIER && isPluginMacro(action.keycode)) {
    return plugins.describeStep(macroById(action.keycode).actions[0]);
  }
  return describeRotary(action);
}

// Una grabación no es una lista de pasos montada a mano, sino la captura de lo
// que hizo el usuario con el ratón y el teclado. Se marca con `kind` para
// distinguirla de una secuencia (ver la pestaña "Grabar").
export function isRecordingMacro(id) {
  return macroById(id)?.kind === 'recording';
}

// La pareja de borrado de una grabación: la tecla que en la capa SUPER tira lo
// grabado. Apunta a la suya con `target` (ver recorder.js).
export function isResetMacro(id) {
  return macroById(id)?.kind === 'recording-reset';
}

// Grabación o su tecla de borrado: las dos pertenecen a la pestaña "Grabar".
export function isRecordOrResetMacro(id) {
  return isRecordingMacro(id) || isResetMacro(id);
}

// describeAction solo ve lo que guarda el firmware (modificador + código), y
// ahí todas las macros son iguales. Las tres formas que sabe editar la app
// —secuencia, abrir algo y grabación— se distinguen mirando la macro.
export function describeKey(action) {
  if (action.modifier !== MACRO_MODIFIER) return describeAction(action.modifier, action.keycode);
  if (isResetMacro(action.keycode)) return 'Borrar grabación';
  if (isRecordingMacro(action.keycode)) {
    return macroById(action.keycode).events?.length ? 'Reproducir grabación' : 'Grabar operación';
  }
  if (isAppMacro(action.keycode)) {
    const target = macroById(action.keycode).actions[0].target || '';
    return target ? `Abrir ${target.split(/[\\/]/).pop()}` : 'Abrir…';
  }
  if (isTextMacro(action.keycode)) {
    const text = (macroById(action.keycode).actions[0].text || '').replace(/\s+/g, ' ').trim();
    if (!text) return 'Escribir texto';
    return `Escribir "${text.length > 14 ? `${text.slice(0, 14)}…` : text}"`;
  }
  if (isPowerMacro(action.keycode)) {
    return POWER_MODE_LABELS[macroById(action.keycode).actions[0].mode] || 'Energía';
  }
  if (isPluginMacro(action.keycode)) {
    return plugins.describeStep(macroById(action.keycode).actions[0]);
  }
  return describeAction(action.modifier, action.keycode);
}

// Si esta acción la tiene que ejecutar el PC. Es lo contrario de "el teclado sabe
// tocarla solo": una secuencia con pasos que el firmware no conoce (abrir una app,
// escribir un texto, un complemento, una grabación), o una que no cabe en su
// memoria de secuencias.
//
// Sin la app abierta esa tecla no hace nada, y hasta ahora no había forma de saber
// cuál era cuál mirando la rejilla. En la WebGUI la diferencia es permanente: ahí
// no hay app de escritorio detrás, nunca.
export function keyNeedsApp(action) {
  if (!action || action.modifier !== MACRO_MODIFIER) return false;
  const m = macroById(action.keycode);
  if (!m) return false;
  return !macroDeviceEligible(m);
}

// Lo mismo para un mando giratorio. Solo las acciones de tipo "atajo" pueden llevar
// una macro dentro; el resto (multimedia, desplazar, zoom) las toca el firmware.
export function rotaryNeedsApp(action) {
  if (!action || action.type !== ROTARY_TYPES.KEY) return false;
  return keyNeedsApp(action);
}
