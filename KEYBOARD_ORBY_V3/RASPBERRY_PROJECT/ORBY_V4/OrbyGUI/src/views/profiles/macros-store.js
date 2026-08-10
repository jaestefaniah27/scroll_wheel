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
  } catch {
    pcMacros = [];
  }
}

// `id`, si se da, es la macro que acaba de cambiar: además de persistir la
// configuración del PC, se intenta subir (o limpiar) su copia en el teclado.
export function savePCMacros(id) {
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
// El firmware sabe tocar espera, tecla, clic y movimiento relativo sin ayuda
// de nadie. Debe coincidir con MacroStepType en main.cpp.
//
// TODO(homing-absoluto): la posición absoluta (MSTEP_HOME_MOVE en main.cpp,
// tipo 5) se implementó y funciona byte a byte tal y como se pidió —lo
// confirma la telemetría de MACRO_TEST: home a +4000,+4000 y tramo final a
// (x - esquina, y - esquina) salen exactos, sin redondeos ni signos
// cambiados—, pero el resultado en pantalla depende del multiplicador de
// velocidad del puntero de Windows (el de "velocidad del puntero", no el de
// "mejorar precisión"; ese ya se descartó), que no hay forma de leer ni
// anular desde el teclado. Sin una calibración (mandar un desplazamiento
// conocido, medir cuánto se movió de verdad y guardar el factor), la
// posición absoluta sin la app abierta no es fiable, así que de momento NO
// se sube al teclado (se queda excluida de DEVICE_STEP_TYPE, cae siempre al
// camino del PC) y la esquina de home queda aparcada. Retomar con una
// pantalla de calibración en el editor antes de reactivarlo. Aparcada a
// petición expresa del usuario — no es la prioridad ahora.
export const DEVICE_STEP_TYPE = { delay: 1, hotkey: 2, mouse_move: 3, mouse_click: 4 };
export const MACRO_MAX_STEPS_DEVICE = 48; // == MACRO_MAX_STEPS en main.cpp
export const CLICK_BUTTON_CODE = { left: 0, middle: 1, right: 2 };

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
// Acciones de energía del PC, elegibles desde la pestaña Multimedia (macro de
// un único paso "system_power", ver electron/macros.js).
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

export function macroDeviceEligible(m) {
  const acts = m?.actions || [];
  return acts.length > 0
      && acts.length <= MACRO_MAX_STEPS_DEVICE
      && m.id < deviceMacroCapacity()
      && acts.every((a) => a.type in DEVICE_STEP_TYPE);
}

// Los pasos de una macro tal y como los quiere el teclado. Se saca aparte
// porque sirve para dos cosas: subirlos y compararlos con los que ya tiene.
function deviceStepsOf(m) {
  return m.actions.map((a) => {
    const type = DEVICE_STEP_TYPE[a.type];
    let a1 = 0, b1 = 0;
    if (a.type === 'delay') a1 = Math.max(0, Math.min(32767, Math.round(a.ms) || 0));
    else if (a.type === 'hotkey') { a1 = a.modifier || 0; b1 = a.keycode || 0; }
    else if (a.type === 'mouse_move') { a1 = a.dx || 0; b1 = a.dy || 0; }
    else if (a.type === 'mouse_click') { a1 = CLICK_BUTTON_CODE[a.button] ?? 0; }
    // Solo tecla y clic saben repetirse en el firmware (ver sameStep en macro-tabs.js): el
    // resto de tipos siempre suben con 1 repetición y sin hueco.
    const repeatable = a.type === 'hotkey' || a.type === 'mouse_click';
    const repeat = repeatable ? Math.max(1, Math.min(255, a.count || 1)) : 1;
    const gap = repeat > 1 ? Math.max(0, Math.min(32767, Math.round(a.gap ?? DEFAULT_REPEAT_GAP_MS))) : 0;
    return { type, a: a1, b: b1, repeat, gap };
  });
}

// Si el teclado ya tiene exactamente esos pasos. Con `steps` vacío pregunta por
// "no tiene nada", que es lo que hace falta antes de mandar un MACRO_CLEAR.
//
// Devuelve false si no se puede saber (el teclado no contesta): ante la duda se
// escribe, que es lo que se hacía siempre.
async function matchesDevice(id, steps) {
  let current;
  try {
    current = await device.getMacro(id);
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

// Sube (o, si ya no es jugable en el teclado, limpia) la copia de una macro en
// el dispositivo. No bloquea al que llama: si falla, la macro se queda
// funcionando por el camino del PC hasta el próximo intento.
//
// Antes de escribir se compara con lo que el teclado ya tiene. Sin eso, la
// resincronización de la conexión reescribía todas las secuencias, marcaba la
// configuración como pendiente y el guardado automático gastaba una escritura
// de Flash del teclado en cada enchufada, sin que hubiera cambiado nada.
export async function syncMacroToDevice(id) {
  // Sin esto, un firmware anterior a esta función respondería "ERR:UNKNOWN_CMD"
  // a SET_MACRO_STEP, que ninguna petición espera: cada intento agotaría su
  // tiempo de espera (varios segundos) en vez de fallar al momento.
  if (!state.connected || !compat.supports(state.deviceInfo, 'macros')) return;
  // Un id que no cabe en el teclado nunca llegó a subirse, así que tampoco hay
  // nada que limpiar: mandar MACRO_CLEAR con él solo saca un ERR:BAD_ARGS.
  if (id >= deviceMacroCapacity()) return;
  const m = macroById(id);

  if (!m || !macroDeviceEligible(m)) {
    if (await matchesDevice(id, [])) return;   // ya estaba vacía
    try { await device.macroClear(id); } catch { /* sin conexión o firmware viejo: no pasa nada */ }
    markDirty();
    return;
  }

  const steps = deviceStepsOf(m);
  if (await matchesDevice(id, steps)) return;

  try {
    for (let i = 0; i < steps.length; i++) {
      const s = steps[i];
      await device.setMacroStep(id, i, s.type, s.a, s.b, s.repeat, s.gap);
    }
    await device.macroTrunc(id, steps.length);
    // Vive en RAM del teclado hasta que se guarde en Flash, igual que un
    // cambio de atajo: sin esto, el aviso de "Guardar en Flash" no se
    // encendería y un apagado se llevaría la secuencia por delante.
    markDirty();
  } catch {
    // El teclado puede no estar conectado o llevar un firmware sin este
    // comando: la macro sigue funcionando por el PC mientras tanto.
  }
}

// Repasa todas las secuencias al conectar y sube solo las que el teclado no
// tenga ya iguales: cubre tanto un teclado recién reflasheado (su Flash de
// secuencias está vacía) como una macro editada mientras estaba desconectado.
//
// La mayoría de las macros no las puede tocar el teclado (abrir una app,
// escribir un texto, un complemento…): de esas no hay nada que subir, solo
// habría que limpiar la copia del teclado si alguna vez se subió. Cuáles se
// subieron se apunta en la configuración del PC, así que preguntar por las
// demás es gastar una ida y vuelta por el CDC para oír "no tengo nada". Con
// cien macros eso eran cien preguntas en cada conexión.
//
// La lista apuntada puede no existir (primera vez con esta versión): entonces
// se repasan todas una vez y se apunta el resultado.
export async function syncAllMacrosToDevice() {
  if (!compat.supports(state.deviceInfo, 'macros')) return;
  await loadPCMacros();

  const known = await uploadedIds();
  for (const m of pcMacros) {
    // Sin nada que subir y sin constancia de que el teclado tenga una copia
    // suya, no hay nada que comprobar.
    if (!macroDeviceEligible(m) && known && !known.includes(m.id)) continue;
    await syncMacroToDevice(m.id);
  }

  const capacity = deviceMacroCapacity();
  const now = pcMacros.filter((m) => macroDeviceEligible(m) && m.id < capacity).map((m) => m.id);
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
