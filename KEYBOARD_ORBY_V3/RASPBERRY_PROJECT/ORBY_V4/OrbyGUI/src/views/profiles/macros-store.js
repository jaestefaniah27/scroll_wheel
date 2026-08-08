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

export function nextMacroId() {
  return pcMacros.reduce((max, m) => Math.max(max, m.id), 0) + 1;
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
      && acts.every((a) => a.type in DEVICE_STEP_TYPE);
}

// Sube (o, si ya no es jugable en el teclado, limpia) la copia de una macro en
// el dispositivo. No bloquea al que llama: si falla, la macro se queda
// funcionando por el camino del PC hasta el próximo intento.
export async function syncMacroToDevice(id) {
  // Sin esto, un firmware anterior a esta función respondería "ERR:UNKNOWN_CMD"
  // a SET_MACRO_STEP, que ninguna petición espera: cada intento agotaría su
  // tiempo de espera (varios segundos) en vez de fallar al momento.
  if (!state.connected || state.deviceInfo?.macros !== '1') return;
  const m = macroById(id);

  if (!m || !macroDeviceEligible(m)) {
    try { await device.macroClear(id); } catch { /* sin conexión o firmware viejo: no pasa nada */ }
    return;
  }

  try {
    for (let i = 0; i < m.actions.length; i++) {
      const a = m.actions[i];
      const type = DEVICE_STEP_TYPE[a.type];
      let p1 = 0, p2 = 0;
      if (a.type === 'delay') p1 = Math.max(0, Math.min(32767, Math.round(a.ms) || 0));
      else if (a.type === 'hotkey') { p1 = a.modifier || 0; p2 = a.keycode || 0; }
      else if (a.type === 'mouse_move') { p1 = a.dx || 0; p2 = a.dy || 0; }
      else if (a.type === 'mouse_click') { p1 = CLICK_BUTTON_CODE[a.button] ?? 0; }
      // Solo tecla y clic saben repetirse en el firmware (ver sameStep en macro-tabs.js): el
      // resto de tipos siempre suben con 1 repetición y sin hueco.
      const repeatable = a.type === 'hotkey' || a.type === 'mouse_click';
      const count = repeatable ? Math.max(1, Math.min(255, a.count || 1)) : 1;
      const gap = count > 1 ? Math.max(0, Math.min(32767, Math.round(a.gap ?? DEFAULT_REPEAT_GAP_MS))) : 0;
      await device.setMacroStep(id, i, type, p1, p2, count, gap);
    }
    await device.macroTrunc(id, m.actions.length);
    // Vive en RAM del teclado hasta que se guarde en Flash, igual que un
    // cambio de atajo: sin esto, el aviso de "Guardar en Flash" no se
    // encendería y un apagado se llevaría la secuencia por delante.
    markDirty();
  } catch {
    // El teclado puede no estar conectado o llevar un firmware sin este
    // comando: la macro sigue funcionando por el PC mientras tanto.
  }
}

// Reenvía todas las secuencias al conectar: cubre tanto un teclado recién
// reflasheado (su Flash de secuencias está vacía) como una macro editada
// mientras estaba desconectado.
export async function syncAllMacrosToDevice() {
  if (state.deviceInfo?.macros !== '1') return;
  await loadPCMacros();
  for (const m of pcMacros) await syncMacroToDevice(m.id);
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
