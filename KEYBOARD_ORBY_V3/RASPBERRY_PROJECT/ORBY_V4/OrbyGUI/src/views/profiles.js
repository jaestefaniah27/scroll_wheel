// Editor de perfiles.
//
// Los datos vienen del firmware (GET_PROFILE) y cada cambio se escribe con
// SET_LABEL / SET_KEYMAP / SET_ROTARY / SET_PSCROLL / SET_NAME. Los cambios
// viven en RAM del microcontrolador hasta que se pulsa "Guardar en Flash", igual
// que en el menú físico del teclado.
//
// La réplica del teclado es la misma que la del editor de iconos —cada tecla
// enseña su icono real— para no tener dos representaciones distintas de lo
// mismo. El botón "Editar icono" salta al editor con esa tecla ya elegida.
//
// El interruptor NORMAL/SUPER afecta a todo el perfil: teclas, mandos giratorios
// y rueda de scroll. Cada capa guarda sus propias acciones, así que un mismo
// perfil puede hacer dos cosas distintas según se mantenga SUPER o no.

import * as device from '../device.js';
import { state, notify, markDirty, subscribe, syncFromDevice, profileMeta, labelSlot, keymapSlot,
         rotarySlot, layerIndex, scrollFor, KEY_TO_SCREEN, KEY_SUPER, KEY_MENU,
         hasPages, maxPages, pageCountOf, selectPage, addPage, removePage } from '../store.js';
import { MODIFIERS, CONSUMER_MODIFIER, CONSUMER_ACTIONS, GOTO_PAGE_MODIFIER, PAGE_STATE_MODIFIER,
         MACRO_MODIFIER, KEY_GROUPS, describeAction, eventToAction,
         ROTARY_TYPES, ROTARY_SLOTS, isScrollType, describeRotary } from '../hid-keys.js';
import { icon } from '../icons.js';
import { toast, requireDevice } from '../ui.js';
import { goTo } from '../nav.js';
import * as cache from '../oled-cache.js';
import * as fb from '../oled-fb.js';
import * as variants from '../variants.js';

// Mandos giratorios agrupados como se ven en el teclado.
const ROTARY_GROUPS = [
  {
    name: 'Izquierdo', icon: 'reset',
    parts: [
      { slot: ROTARY_SLOTS.ENC1_CW,    label: 'Giro horario',     short: '↻' },
      { slot: ROTARY_SLOTS.ENC1_CCW,   label: 'Giro antihorario', short: '↺' },
      { slot: ROTARY_SLOTS.ENC1_CLICK, label: 'Pulsación',        short: '⏺', discrete: true },
    ],
  },
  {
    name: 'Derecho', icon: 'reset',
    parts: [
      { slot: ROTARY_SLOTS.ENC2_CW,    label: 'Giro horario',     short: '↻' },
      { slot: ROTARY_SLOTS.ENC2_CCW,   label: 'Giro antihorario', short: '↺' },
      { slot: ROTARY_SLOTS.ENC2_CLICK, label: 'Pulsación',        short: '⏺', discrete: true },
    ],
  },
];

// La rueda ya no comparte rejilla con los encoders (tiene su propia tarjeta,
// más abajo, con botones directos para "hacia abajo" y "hacia arriba"), pero
// el inspector necesita su nombre y su etiqueta cuando se abre desde ahí.
const WHEEL_GROUP = {
  name: 'Rueda de scroll', icon: 'wheel',
  parts: [
    { slot: ROTARY_SLOTS.WHEEL_CW,  label: 'Hacia abajo',  short: '↓' },
    { slot: ROTARY_SLOTS.WHEEL_CCW, label: 'Hacia arriba', short: '↑' },
  ],
};

// Tipos ofrecidos en el inspector. Los de desplazamiento no tienen sentido en
// una pulsación, que es un evento suelto sin dirección.
const ROTARY_TYPE_OPTIONS = [
  { type: ROTARY_TYPES.NONE,     label: 'Nada' },
  { type: ROTARY_TYPES.CONSUMER, label: 'Multimedia / sistema' },
  { type: ROTARY_TYPES.KEY,      label: 'Atajo de teclado' },
  { type: ROTARY_TYPES.SCROLL_V, label: 'Desplazar vertical', turnOnly: true },
  { type: ROTARY_TYPES.SCROLL_H, label: 'Desplazar horizontal', turnOnly: true },
  { type: ROTARY_TYPES.ZOOM,     label: 'Zoom (Ctrl + rueda)', turnOnly: true },
];

const SCROLL_PRESETS = [
  { value: 12,  name: 'Preciso',  desc: 'Timeline, PCB, edición fina' },
  { value: 30,  name: 'Suave',    desc: 'Lectura y navegación lenta' },
  { value: 60,  name: 'Estándar', desc: 'Equivalente a rueda de ratón' },
  { value: 120, name: 'Rápido',   desc: 'Documentos y logs largos' },
];

const view = {
  editingProfile: 0,
  layer: 'normal',
  variantId: null,  // null = perfil base; si no, la variación que se edita
  selected: null,   // { kind: 'key', index } | { kind: 'rotary', slot }
  tab: 'shortcut',  // pestaña del inspector de tecla: shortcut | sequence | media | pages
  capturing: false, // false | true (atajo) | 'sequence' (grabando una macro) | 'position' (capturando dónde poner el ratón)
  busy: false,      // hay un alta/baja de perfil en marcha
};

// Tiempo de espera que se inserta automáticamente entre dos pasos de una
// secuencia. Bastante bajo para no notarse, pero suficiente para que el
// sistema/la app de destino registre el paso anterior (mover el ratón, un
// clic) antes del siguiente.
const DEFAULT_STEP_DELAY_MS = 50;

// Página del perfil que se está editando en esta vista.
function currentPageIdx() {
  const prof = state.profiles[view.editingProfile];
  return prof ? (prof.pageIdx || 0) : 0;
}

// Variación que se está editando. Editar el perfil base es lo normal; al elegir
// una variación, cada cambio se guarda como diferencia respecto a ese base.
//
// Una variación es de una página concreta: si se está mirando otra, no pinta
// nada aquí (sus huecos son de la suya).
function editingVariant() {
  const v = view.variantId ? variants.get(view.variantId) : null;
  if (!v) return null;
  return (v.profile === view.editingProfile && v.page === currentPageIdx()) ? v : null;
}

// --- Macros (secuencias ejecutadas en el PC) --------------------------------
// Viven en la configuración local (window.orby.getConfig/setConfig), no en el
// teclado: el firmware solo guarda el id en el campo del modificador de la
// tecla o del mando (MACRO_MODIFIER) y avisa por CDC al pulsarla.
let pcMacros = [];

async function loadPCMacros() {
  try {
    const cfg = await window.orby.getConfig();
    pcMacros = cfg?.macros || [];
  } catch {
    pcMacros = [];
  }
}

// `id`, si se da, es la macro que acaba de cambiar: además de persistir la
// configuración del PC, se intenta subir (o limpiar) su copia en el teclado.
function savePCMacros(id) {
  window.orby.setConfig({ macros: pcMacros });
  if (id !== undefined) syncMacroToDevice(id);
}

function macroById(id) {
  return pcMacros.find((m) => m.id === id);
}

function ensureMacro(id) {
  let m = macroById(id);
  if (!m) {
    m = { id, actions: [] };
    pcMacros.push(m);
    savePCMacros(id);
  }
  return m;
}

function nextMacroId() {
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
const DEVICE_STEP_TYPE = { delay: 1, hotkey: 2, mouse_move: 3, mouse_click: 4 };
const MACRO_MAX_STEPS_DEVICE = 48; // == MACRO_MAX_STEPS en main.cpp
const CLICK_BUTTON_CODE = { left: 0, middle: 1, right: 2 };
// Acciones de energía del PC, elegibles desde la pestaña Multimedia (macro de
// un único paso "system_power", ver electron/macros.js).
const POWER_MODE_LABELS = {
  sleep: 'Suspender', hibernate: 'Hibernar', restart: 'Reiniciar',
  shutdown: 'Apagar', lock: 'Bloquear pantalla', logoff: 'Cerrar sesión',
};
// Hueco por defecto entre cada repetición de un mismo paso (tecla o clic
// repetido, ver sameStep): se usa mientras el paso no traiga uno propio (el
// campo `gap`, editable en el inspector a partir de 2 repeticiones). Debe
// coincidir con el mismo nombre en electron/macros.js (ahí se usa para las
// secuencias que corren por el camino del PC, no del teclado).
const DEFAULT_REPEAT_GAP_MS = 20;

function macroDeviceEligible(m) {
  const acts = m?.actions || [];
  return acts.length > 0
      && acts.length <= MACRO_MAX_STEPS_DEVICE
      && acts.every((a) => a.type in DEVICE_STEP_TYPE);
}

// Sube (o, si ya no es jugable en el teclado, limpia) la copia de una macro en
// el dispositivo. No bloquea al que llama: si falla, la macro se queda
// funcionando por el camino del PC hasta el próximo intento.
async function syncMacroToDevice(id) {
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
      // Solo tecla y clic saben repetirse en el firmware (ver sameStep): el
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

function selectedKeyIndex() {
  return view.selected?.kind === 'key' ? view.selected.index : null;
}

function selectedRotarySlot() {
  return view.selected?.kind === 'rotary' ? view.selected.slot : null;
}

function rotaryPart(baseSlot) {
  for (const g of [...ROTARY_GROUPS, WHEEL_GROUP]) {
    const p = g.parts.find((x) => x.slot === baseSlot);
    if (p) return { group: g, part: p };
  }
  return null;
}

function currentProfile() {
  return state.profiles[view.editingProfile] || null;
}

function isActiveView() {
  return document.getElementById('view-profiles')?.classList.contains('active');
}

export function init() {
  const body = document.getElementById('profiles-body');
  body.addEventListener('click', onClick);
  body.addEventListener('change', onChange);
  body.addEventListener('input', onInput);
  window.addEventListener('keydown', onCapture, true);

  // Enter en el campo de aplicaciones añade, para poder encadenar varias.
  body.addEventListener('keydown', (e) => {
    if (e.target.id === 'variant-new-match' && e.key === 'Enter') {
      e.preventDefault();
      addMatchFromField();
    }
  });

  // Las miniaturas llegan por el puerto serie con retraso: cuando la caché se
  // completa hay que repintar la rejilla, no la vista entera.
  cache.onChange(() => { if (isActiveView()) renderKeyGrid(); });

  // Las macros (secuencias) viven en la configuración local del PC, no en el
  // teclado: se leen una vez al arrancar y se repintan si ya se estaba
  // enseñando el inspector de una tecla en modo "Secuencia".
  loadPCMacros().then(() => { if (isActiveView() && view.selected) render(); });

  // Cuando el detector de aplicaciones aplica o quita una variación hay que
  // reflejarlo aquí (el distintivo de "aplicada ahora").
  variants.onChange((reason) => {
    if (reason === 'applied' && isActiveView()) render();
  });

  // Grabar operación: el proceso principal es quien engancha el ratón y el
  // teclado, así que avisa desde allí de en qué punto está.
  window.orby.recorder.onState((info) => {
    onRecorderState(info).catch((err) => console.error('Grabación:', err));
  });

  // Repintado solo cuando cambia algo que se ve aquí: si repintásemos con cada
  // notificación, escribir un nombre o una etiqueta perdería el foco al vuelo,
  // porque cada escritura marca la configuración como pendiente de guardar.
  let lastCount = -1;
  let lastActive = -1;
  let lastPage = -1;
  subscribe(() => {
    // La página entra en la cuenta porque el teclado la cambia por su cuenta y
    // aquí se está editando justamente esa: la rejilla tiene que seguirla.
    if (state.profiles.length === lastCount && state.activeProfileIdx === lastActive
        && state.pageIdx === lastPage) return;
    lastCount = state.profiles.length;
    lastActive = state.activeProfileIdx;
    lastPage = state.pageIdx;
    if (isActiveView()) render();
  });
}

// --- Interacción -----------------------------------------------------------

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;
  const act = el.dataset.act;

  if (act === 'pick-profile') {
    view.editingProfile = Number(el.dataset.idx);
    view.variantId = null;
    view.selected = null;
    render();
  } else if (act === 'pick-variant') {
    view.variantId = el.dataset.id || null;
    view.selected = null;
    render();
  } else if (act === 'new-variant') {
    createVariant();
  } else if (act === 'del-variant') {
    removeVariant();
  } else if (act === 'add-match') {
    addMatchFromField();
  } else if (act === 'add-match-current') {
    addCurrentApp();
  } else if (act === 'del-match') {
    variants.removeMatch(view.variantId, el.dataset.match);
    render();
  } else if (act === 'clear-override') {
    clearOverrideOfSelection();
  } else if (act === 'layer') {
    view.layer = el.dataset.layer;
    view.selected = null;
    render();
  } else if (act === 'page') {
    choosePage(Number(el.dataset.page));
  } else if (act === 'page-add') {
    createPage();
  } else if (act === 'page-del') {
    deletePage(Number(el.dataset.page));
  } else if (act === 'pick-key') {
    view.selected = { kind: 'key', index: Number(el.dataset.key) };
    view.capturing = false;
    view.tab = tabForAction(currentAction());
    render();
  } else if (act === 'pick-rotary') {
    view.selected = { kind: 'rotary', slot: Number(el.dataset.slot) };
    view.capturing = false;
    render();
  } else if (act === 'edit-icon') {
    // Salta al editor de iconos con este perfil, esta tecla y esta capa.
    goTo('view-oled', { profile: view.editingProfile, key: Number(el.dataset.key), layer: view.layer });
  } else if (act === 'copy-key') {
    copyKey();
  } else if (act === 'paste-key') {
    pasteKey();
  } else if (act === 'rotary-type') {
    applyRotary({ type: Number(el.dataset.type), modifier: 0, keycode: 0 });
  } else if (act === 'rotary-consumer') {
    applyRotary({ type: ROTARY_TYPES.CONSUMER, modifier: 0, keycode: Number(el.dataset.index) });
  } else if (act === 'rotary-mod') {
    const cur = currentRotary();
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: cur.modifier ^ Number(el.dataset.bit), keycode: cur.keycode });
  } else if (act === 'activate') {
    device.setProfile(view.editingProfile)
      .then(() => {
        state.activeProfileIdx = view.editingProfile;
        notify();
        render();
      })
      .catch(() => toast('El teclado no confirmó el cambio de perfil', 'error'));
  } else if (act === 'capture') {
    view.capturing = !view.capturing;
    render();
  } else if (act === 'clear-action') {
    clearKeyAction();
  } else if (act === 'set-goto-page') {
    applyKeymap(GOTO_PAGE_MODIFIER, Number(el.dataset.page));
  } else if (act === 'set-page-state') {
    applyKeymap(PAGE_STATE_MODIFIER, 0);
  } else if (act === 'set-consumer') {
    applyKeymap(CONSUMER_MODIFIER, Number(el.dataset.index));
  } else if (act === 'set-power') {
    setPowerAction(el.dataset.mode);
  } else if (act === 'toggle-mod') {
    const bit = Number(el.dataset.bit);
    const current = currentAction();
    // Multimedia y páginas ocupan el campo del modificador, así que tocar un
    // modificador sale de ese modo en vez de mezclarse con él.
    if (current.modifier >= CONSUMER_MODIFIER || current.modifier === GOTO_PAGE_MODIFIER
        || current.modifier === PAGE_STATE_MODIFIER) applyKeymap(bit, 0);
    else applyKeymap(current.modifier ^ bit, current.keycode);
  } else if (act === 'profile-new') {
    createProfile(null);
  } else if (act === 'profile-dup') {
    createProfile(view.editingProfile);
  } else if (act === 'profile-del') {
    removeProfile();
  } else if (act === 'scroll-preset') {
    applyScroll({ detentsPerRev: Number(el.dataset.value) });
  } else if (act === 'scroll-invert') {
    applyScroll({ invert: !currentScroll().invert });
  } else if (act === 'set-tab') {
    setTab(el.dataset.tab);
  } else if (act === 'seq-add-position') {
    if (view.capturing === 'position') stopPositionCapture(false);
    else startPositionCapture();
  } else if (act === 'seq-recapture') {
    if (view.capturing === 'position') stopPositionCapture(false);
    else startPositionCapture(Number(el.dataset.index));
  } else if (act === 'seq-add-click') {
    seqAddAction({ type: 'mouse_click', button: 'left' });
  } else if (act === 'seq-add-open') {
    // En blanco, como el primer estado de la pestaña App: se rellena con los
    // mismos controles (App en foco / Examinar / buscador) que enseña cada
    // paso ya en la lista.
    seqAddAction({ type: 'open_app', target: '', kind: 'app' });
  } else if (act === 'seq-open-browse') {
    pickOpenTarget(Number(el.dataset.index), el.dataset.kind);
  } else if (act === 'seq-open-kind') {
    setSeqOpenKind(Number(el.dataset.index), el.dataset.kind);
  } else if (act === 'seq-open-focus') {
    pickSeqOpenFocus(Number(el.dataset.index));
  } else if (act === 'app-kind') {
    setAppKind(el.dataset.kind);
  } else if (act === 'app-browse') {
    pickAppTarget(el.dataset.kind);
  } else if (act === 'app-focus') {
    pickAppFocus();
  } else if (act === 'seq-add-move') {
    seqAddAction({ type: 'mouse_move', dx: 10, dy: 0 });
  } else if (act === 'seq-key-mod') {
    seqKeyBuilder.modifier ^= Number(el.dataset.bit);
    render();
  } else if (act === 'seq-add-hotkey') {
    if (!seqKeyBuilder.modifier && !seqKeyBuilder.keycode) return;
    seqAddAction({ type: 'hotkey', modifier: seqKeyBuilder.modifier, keycode: seqKeyBuilder.keycode });
    seqKeyBuilder = { ...seqKeyBuilder, keycode: 0 };
  } else if (act === 'seq-del') {
    seqRemoveAction(Number(el.dataset.index));
  } else if (act === 'seq-move-up') {
    seqMoveAction(Number(el.dataset.index), -1);
  } else if (act === 'seq-move-down') {
    seqMoveAction(Number(el.dataset.index), 1);
  } else if (act === 'seq-record') {
    view.capturing = view.capturing === 'sequence' ? false : 'sequence';
    render();
  } else if (act === 'seq-clear') {
    view.tab = 'shortcut';
    applyKeymap(0, 0);
  } else if (act === 'rotary-macro') {
    applyRotaryMacro();
  } else if (act === 'rec-toggle') {
    toggleRecording();
  } else if (act === 'rec-mode') {
    setRecordMode(el.dataset.mode);
  } else if (act === 'rec-speed') {
    setRecordSpeed(el.dataset.speed);
  } else if (act === 'rec-clear') {
    clearRecording();
  } else if (act === 'rec-stop') {
    window.orby.recorder.stop();
  }
}

function onChange(e) {
  const act = e.target.dataset.act;
  if (act === 'pick-keycode') {
    // Multimedia, páginas y secuencias se apropian del campo del modificador;
    // elegir una tecla desde la pestaña Atajo sale de esos modos igual que
    // tocar un modificador (ver toggle-mod).
    const current = currentAction();
    const isSpecial = current.modifier === CONSUMER_MODIFIER || current.modifier === GOTO_PAGE_MODIFIER
                    || current.modifier === PAGE_STATE_MODIFIER || current.modifier === MACRO_MODIFIER;
    const mod = isSpecial ? 0 : current.modifier;
    applyKeymap(mod, Number(e.target.value));
  } else if (act === 'rotary-keycode') {
    const cur = currentRotary();
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: cur.modifier, keycode: Number(e.target.value) });
  } else if (act === 'scroll-slider') {
    applyScroll({ detentsPerRev: Number(e.target.value) });
  } else if (act === 'variant-field') {
    variants.update(view.variantId, { field: e.target.value });
  } else if (act === 'variant-name') {
    // `change` salta al salir del campo: repintar aquí no corta la escritura.
    variants.update(view.variantId, { name: e.target.value.trim() });
    render();
  } else if (act === 'seq-click-button') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.button = e.target.value; savePCMacros(currentMacroId()); }
  } else if (act === 'seq-count') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) {
      step.count = Math.max(1, Math.min(99, Math.round(Number(e.target.value)) || 1));
      savePCMacros(currentMacroId());
      render();
    }
  } else if (act === 'seq-delay-ms') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.ms = Math.max(0, Number(e.target.value) || 0); savePCMacros(currentMacroId()); }
  } else if (act === 'seq-gap-ms') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) {
      step.gap = Math.max(0, Math.min(32767, Math.round(Number(e.target.value)) || 0));
      savePCMacros(currentMacroId());
    }
  } else if (act === 'seq-pos-x') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.x = Math.round(Number(e.target.value) || 0); savePCMacros(currentMacroId()); }
  } else if (act === 'seq-pos-y') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.y = Math.round(Number(e.target.value) || 0); savePCMacros(currentMacroId()); }
  } else if (act === 'seq-open-target') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.target = e.target.value; savePCMacros(currentMacroId()); }
  } else if (act === 'app-target') {
    const step = currentAppStep();
    if (step) { step.target = e.target.value; savePCMacros(currentMacroId()); }
  } else if (act === 'seq-move-dx') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.dx = clampMoveDelta(e.target.value); savePCMacros(currentMacroId()); }
  } else if (act === 'seq-move-dy') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.dy = clampMoveDelta(e.target.value); savePCMacros(currentMacroId()); }
  } else if (act === 'seq-key-pick') {
    seqKeyBuilder.keycode = Number(e.target.value);
  }
}

// El paso "mover ratón" viaja al teclado como delta relativo de 8 bits con
// signo (lo que cabe en un informe HID de ratón): más que esto no tiene
// sentido pedirlo desde el editor.
function clampMoveDelta(v) {
  const n = Math.round(Number(v) || 0);
  return Math.max(-127, Math.min(127, n));
}

let labelDebounce = null;
function onInput(e) {
  const act = e.target.dataset.act;

  if (act === 'edit-name') {
    const text = e.target.value.slice(0, 7);
    clearTimeout(labelDebounce);
    labelDebounce = setTimeout(async () => {
      const prof = currentProfile();
      if (!prof || !requireDevice()) return;
      prof.name = text;
      try {
        await device.setName(view.editingProfile, text);
        markDirty();
        notify();
        renderTabs();
      } catch {
        toast('El teclado no confirmó el nombre', 'error');
      }
    }, 250);

  } else if (act === 'scroll-slider') {
    // Deslizar solo actualiza el número: el comando se manda al soltar, para no
    // inundar el CDC con un SET_PSCROLL por cada píxel.
    const val = Number(e.target.value);
    const readout = document.getElementById('scroll-value');
    if (readout) readout.textContent = val;
    updateDerived(val);
  }
}

// Captura un atajo pulsándolo físicamente en el teclado del PC.
function onCapture(e) {
  if (!view.capturing || !view.selected) return;

  // Capturando una posición de ratón: el ratón es la entrada, el teclado solo
  // hace falta para fijarla con Esc. El resto de teclas no se toca.
  if (view.capturing === 'position') {
    if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); stopPositionCapture(true); }
    return;
  }

  e.preventDefault();
  e.stopPropagation();

  if (e.key === 'Escape') { view.capturing = false; render(); return; }

  // Grabando una secuencia: cada tecla capturable se añade como un paso más,
  // sin salir del modo (se graban varias seguidas). Se guarda en HID (modifier
  // + keycode), como un paso "tecla" cualquiera: así el teclado también sabe
  // reproducirla, no solo el PC. Cubre cualquier tecla que EVENT_CODE_TO_HID
  // (hid-keys.js) sepa resolver a un keycode HID: letras, dígitos, F1-F24,
  // flechas, numérico, edición..., con los modificadores que se tuvieran
  // pulsados en ese instante.
  if (view.capturing === 'sequence') {
    const captured = eventToAction(e);
    if (!captured.keycode) return;
    seqAddAction({ type: 'hotkey', modifier: captured.modifier, keycode: captured.keycode });
    return;
  }

  const action = eventToAction(e);
  if (!action.keycode) return; // solo modificadores: seguimos esperando

  view.capturing = false;
  if (view.selected.kind === 'rotary') {
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: action.modifier, keycode: action.keycode });
  } else {
    applyKeymap(action.modifier, action.keycode);
  }
}

// --- Variaciones ------------------------------------------------------------

// `foreground.current()` solo devuelve algo si el proceso PowerShell que vigila
// la ventana en primer plano ya está arrancado, y eso solo pasa si la función
// "Auto" (cambio de perfil por app) se ha activado alguna vez. Si nadie la ha
// tocado, current() siempre da null aunque haya algo enfocado: aquí se arranca
// bajo demanda y se reintenta unas cuantas veces mientras llega el primer dato.
async function getCurrentWindowInfo() {
  let info = await window.orby.foreground.current();
  if (info) return info;

  await window.orby.foreground.start();
  for (let i = 0; i < 8 && !info; i++) {
    await new Promise((r) => setTimeout(r, 300));
    info = await window.orby.foreground.current();
  }
  return info;
}

// Se propone la app que tengas delante: es casi siempre para la que quieres la
// variación, y ahorra escribir el nombre del ejecutable a mano.
async function createVariant() {
  let match = '';
  try {
    const info = await getCurrentWindowInfo();
    match = (info?.process || '').toLowerCase();
  } catch { /* sin detector: se rellena a mano */ }

  // Nace pegada a la página que se está editando: sus huecos son los de esta.
  const variant = variants.create(view.editingProfile, {
    page: currentPageIdx(),
    name: match ? match.replace(/\.exe$/, '') : 'Variación',
    matches: match ? [match] : [],
  });
  view.variantId = variant.id;
  view.selected = null;
  render();
  toast(match ? `Variación creada para "${match}"` : 'Variación creada: indícale a qué app se aplica');
}

// Apps que disparan la variación: se pueden añadir varias, porque el mismo
// retoque suele valer para más de un programa.
function addMatchFromField() {
  const input = document.getElementById('variant-new-match');
  if (!input) return;
  const text = input.value.trim();
  if (!text) return;

  if (!variants.addMatch(view.variantId, text)) {
    toast('Esa aplicación ya está en la lista', 'info');
    return;
  }
  input.value = '';
  render();
  // Volver el foco al campo para poder encadenar varias sin usar el ratón.
  document.getElementById('variant-new-match')?.focus();
}

async function addCurrentApp() {
  try {
    const info = await getCurrentWindowInfo();
    const proc = (info?.process || '').toLowerCase();
    if (!proc) { toast('Aún no se ha detectado ninguna ventana', 'error'); return; }
    if (!variants.addMatch(view.variantId, proc)) {
      toast('Esa aplicación ya está en la lista', 'info');
      return;
    }
    render();
  } catch {
    toast('El detector de aplicaciones no está disponible', 'error');
  }
}

async function removeVariant() {
  const variant = editingVariant();
  if (!variant) return;
  if (!confirm(`Se borrará la variación "${variant.name}" y sus ${variants.countOverrides(variant)} cambios.\n\n`
             + 'El perfil base no se toca.\n\n¿Continuar?')) return;

  await variants.remove(variant.id);
  view.variantId = null;
  view.selected = null;
  render();
  toast('Variación eliminada');
}

// Devuelve la tecla o el mando seleccionados al valor del perfil base.
async function clearOverrideOfSelection() {
  const variant = editingVariant();
  if (!variant || !view.selected) return;

  const kind = view.selected.kind === 'rotary' ? 'rotary' : 'keys';
  const slot = view.selected.kind === 'rotary'
    ? rotarySlot(view.selected.slot, view.layer)
    : keymapSlot(view.selected.index, view.layer);

  variants.clearOverride(variant.id, kind, slot);
  if (view.selected.kind === 'key') {
    variants.clearOverride(variant.id, 'labels', labelSlot(view.selected.index, view.layer));
  }

  // Si la variación está puesta, el teclado tiene que volver al valor base. Lo
  // hace variants: sabe en qué página escribió, que no tiene por qué ser la que
  // se está mirando aquí.
  if (variants.isApplied(variant.id)) {
    try {
      if (kind === 'keys') {
        await variants.revertSlot(variant.id, 'keys', slot);
        const ls = labelSlot(view.selected.index, view.layer);
        if (ls >= 0) await variants.revertSlot(variant.id, 'labels', ls);
      } else {
        await variants.revertSlot(variant.id, 'rotary', slot);
      }
    } catch {
      toast('El teclado no confirmó la vuelta al valor base', 'error');
    }
  }
  if (view.selected.kind === 'key') view.tab = tabForAction(currentAction());
  render();
}

// --- Alta y baja de perfiles ------------------------------------------------

async function createProfile(copyFrom) {
  if (view.busy) return;
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (state.profiles.length >= state.maxProfiles) {
    toast(`El teclado admite como máximo ${state.maxProfiles} perfiles`, 'error');
    return;
  }

  view.busy = true;
  render();
  try {
    const idx = copyFrom === null ? await device.addProfile() : await device.dupProfile(copyFrom);
    // Los índices del banco de iconos se desplazan con la lista, así que lo
    // leído deja de valer.
    cache.invalidate();
    await syncFromDevice();
    view.editingProfile = Number.isInteger(idx) ? idx : state.profiles.length - 1;
    view.selected = null;
    markDirty();
    toast(copyFrom === null ? 'Perfil creado' : 'Perfil duplicado');
  } catch (err) {
    toast(`No se pudo crear el perfil: ${err.message}`, 'error');
  } finally {
    view.busy = false;
    render();
  }
}

async function removeProfile() {
  if (view.busy) return;
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (state.profiles.length <= 1) { toast('Tiene que quedar al menos un perfil', 'error'); return; }

  const prof = currentProfile();
  if (!confirm(`Se borrará el perfil "${prof?.name || ''}" con sus atajos, mandos e iconos.\n\n`
             + 'El cambio no será permanente hasta que pulses "Guardar en Flash".\n\n¿Continuar?')) return;

  view.busy = true;
  render();
  try {
    await device.delProfile(view.editingProfile);
    // Los índices de los perfiles siguientes bajan uno: las variaciones tienen
    // que seguirlos, y las del perfil borrado se van con él.
    variants.shiftProfiles(view.editingProfile);
    view.variantId = null;
    cache.invalidate();
    await syncFromDevice();
    view.editingProfile = Math.min(view.editingProfile, state.profiles.length - 1);
    view.selected = null;
    markDirty();
    toast('Perfil eliminado');
  } catch (err) {
    toast(`No se pudo eliminar: ${err.message}`, 'error');
  } finally {
    view.busy = false;
    render();
  }
}

// --- Escritura de acciones --------------------------------------------------

// Lo que hace la tecla o el mando con lo que hay seleccionado: el perfil base,
// o el base con la variación encima si se está editando una.
function currentAction() {
  const prof = currentProfile();
  const index = selectedKeyIndex();
  if (!prof || index === null) return { modifier: 0, keycode: 0 };
  return variants.effectiveKey(prof, editingVariant(), index, view.layer);
}

function currentRotary() {
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return { type: 0, modifier: 0, keycode: 0 };
  return variants.effectiveRotary(prof, editingVariant(), base, view.layer);
}

function currentScroll() {
  return scrollFor(currentProfile(), view.layer);
}

// Escribe la etiqueta de un hueco, respetando variación (igual que applyKeymap):
// se usa tanto al escribir en el campo de texto como al pegar una tecla copiada.
// Los cambios se comprueban contra el teclado ANTES de tocar el modelo de la
// app: sin él conectado no se edita nada (ver requireDevice), porque la copia
// del PC y la del Orby tienen que seguir siendo la misma.
async function writeLabel(slot, text) {
  if (!requireDevice()) return;
  const prof = currentProfile();
  if (!prof) return;
  const variant = editingVariant();

  if (variant) {
    variants.setOverride(variant.id, 'labels', slot, text);
    renderKeyGrid();
    if (variants.isApplied(variant.id)) {
      try { await variants.writeAppliedSlot(variant.id, 'labels', slot, text); }
      catch { toast('El teclado no confirmó la etiqueta', 'error'); }
    }
    return;
  }

  prof.labels[slot] = text;
  try {
    await device.setLabel(view.editingProfile, slot, text);
    markDirty();
    await variants.reassertSlot('labels', slot);
    renderKeyGrid();
  } catch {
    toast('El teclado no confirmó la etiqueta', 'error');
  }
}

async function applyKeymap(modifier, keycode) {
  if (!requireDevice()) return;
  const prof = currentProfile();
  const index = selectedKeyIndex();
  if (!prof || index === null) return;

  const slot = keymapSlot(index, view.layer);
  const variant = editingVariant();

  // Editando una variación no se toca el perfil ni la Flash: se guarda la
  // diferencia en el PC, y solo se escribe en el teclado si esa variación es la
  // que está puesta ahora mismo.
  if (variant) {
    variants.setOverride(variant.id, 'keys', slot, { modifier, keycode });
    render();
    if (variants.isApplied(variant.id)) {
      try {
        await variants.writeAppliedSlot(variant.id, 'keys', slot, { modifier, keycode });
      } catch {
        toast('No se pudo escribir la asignación', 'error');
      }
    }
    return;
  }

  prof.keys[slot] = { modifier, keycode };
  render();

  try {
    await device.setKeymap(view.editingProfile, slot, modifier, keycode);
    markDirty();
    // Si la variación puesta redefine este hueco, manda ella.
    await variants.reassertSlot('keys', slot);
  } catch {
    toast('No se pudo escribir la asignación', 'error');
  }
}

// Quitar una tecla: se va el atajo (o la macro) y su pantalla se queda en negro.
// Si era una grabación, se lleva también su tecla de borrado en SUPER: sin ella
// la capa SUPER se quedaría con un RESET que ya no borra nada.
async function clearKeyAction() {
  const before = currentAction();
  if (before.modifier === MACRO_MODIFIER && isRecordOrResetMacro(before.keycode)) {
    const rec = recordMacro(before.keycode);
    if (rec) await detachResetKey(rec.id);
  }
  await applyKeymap(0, 0);
  await blankKeyScreen();
}

// Bytes de un framebuffer de pantalla (72x40 monocromo).
const OLED_FRAME_BYTES = 360;

// Deja en negro la pantalla de la tecla seleccionada (capa actual).
//
// OLED_CLEAR no sirve para esto: solo suelta el icono propio del hueco, y
// entonces el firmware vuelve a pintar la etiqueta de texto con su marco —y en
// el perfil 0, página 1 y capa normal, directamente el mapa de bits horneado de
// fábrica (ver refresh_single_screen en main.cpp)—, así que la tecla nunca se
// apagaba. Subir un icono todo a ceros sí lo respeta en todos los casos.
//
// El icono es del hueco (perfil, página, capa, tecla), no de la variación en
// edición: esa distinción no existe para el OLED. La etiqueta sí, así que se
// borra con writeLabel, que ya sabe si toca guardarla como diferencia.
async function blankKeyScreen() {
  if (!requireDevice()) return;
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null) return;
  const lslot = labelSlot(i, view.layer);
  if (lslot < 0) return;

  // Sin esto, quitar el icono más adelante desde el editor descubriría el texto
  // de la tecla que había antes.
  await writeLabel(lslot, '');

  try {
    const blank = new Uint8Array(OLED_FRAME_BYTES);
    await device.uploadOled(view.editingProfile, lslot, blank);
    cache.set(view.editingProfile, lslot, blank);
    prof.oledMask |= (1 << lslot);
    markDirty();
    render();
  } catch {
    toast('No se pudo apagar la pantalla de la tecla', 'error');
  }
}

// --- Copiar y pegar teclas ---------------------------------------------------
// Copia el icono (si tiene uno propio), la etiqueta y la acción de la tecla
// seleccionada, para poder pegarlos en cualquier otra tecla: de otra capa, de
// otra página o de otro perfil. Vive solo en memoria (no sobrevive a cerrar la
// app), como el resto de estado transitorio de este editor.
let keyClipboard = null;

async function copyKey() {
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null) return;

  const action = currentAction();
  const lslot = labelSlot(i, view.layer);
  const label = lslot >= 0 ? variants.effectiveLabel(prof, editingVariant(), i, view.layer) : '';
  const iconBytes = lslot >= 0 ? cache.get(view.editingProfile, lslot) : null;

  // Una secuencia no viaja por su id (lo reutilizaría, enlazando las dos
  // teclas): se lleva una copia de sus pasos para duplicarla al pegar.
  let macroActions = null;
  if (action.modifier === MACRO_MODIFIER) {
    const m = macroById(action.keycode);
    if (m) macroActions = JSON.parse(JSON.stringify(m.actions || []));
  }

  keyClipboard = {
    action: { modifier: action.modifier, keycode: action.keycode },
    label,
    icon: iconBytes ? Uint8Array.from(iconBytes) : null,
    macroActions,
  };
  toast('Tecla copiada');
  render();
}

async function pasteKey() {
  if (!keyClipboard || !requireDevice()) return;
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null) return;

  let { modifier, keycode } = keyClipboard.action;
  if (modifier === MACRO_MODIFIER && keyClipboard.macroActions) {
    const id = nextMacroId();
    const m = ensureMacro(id);
    m.actions = JSON.parse(JSON.stringify(keyClipboard.macroActions));
    savePCMacros(id);
    keycode = id;
  }

  await applyKeymap(modifier, keycode);

  const lslot = labelSlot(i, view.layer);
  if (lslot < 0) { toast('Tecla pegada'); return; }

  await writeLabel(lslot, keyClipboard.label || '');

  try {
    if (keyClipboard.icon) {
      await device.uploadOled(view.editingProfile, lslot, keyClipboard.icon);
      cache.set(view.editingProfile, lslot, Uint8Array.from(keyClipboard.icon));
      prof.oledMask |= (1 << lslot);
      markDirty();
    } else if (cache.get(view.editingProfile, lslot)) {
      // La copiada no tenía icono propio: si el destino sí, se limpia para
      // que enseñe la etiqueta de texto, igual que la tecla de origen.
      await device.clearOled(view.editingProfile, lslot);
      cache.set(view.editingProfile, lslot, null);
      prof.oledMask &= ~(1 << lslot);
      markDirty();
    }
    render();
  } catch {
    toast('No se pudo pegar el icono', 'error');
  }

  toast('Tecla pegada');
}

async function applyRotary(action) {
  if (!requireDevice()) return;
  const prof = currentProfile();
  const base = selectedRotarySlot();
  if (!prof || base === null) return;

  const slot = rotarySlot(base, view.layer);
  const variant = editingVariant();

  // Los desplazamientos son bidireccionales, así que el firmware ignora la
  // acción del sentido contrario. Espejamos el par para que la interfaz no
  // enseñe una configuración que no se va a aplicar.
  const twinBase = { [ROTARY_SLOTS.ENC1_CW]: ROTARY_SLOTS.ENC1_CCW,
                     [ROTARY_SLOTS.ENC1_CCW]: ROTARY_SLOTS.ENC1_CW,
                     [ROTARY_SLOTS.ENC2_CW]: ROTARY_SLOTS.ENC2_CCW,
                     [ROTARY_SLOTS.ENC2_CCW]: ROTARY_SLOTS.ENC2_CW,
                     [ROTARY_SLOTS.WHEEL_CW]: ROTARY_SLOTS.WHEEL_CCW,
                     [ROTARY_SLOTS.WHEEL_CCW]: ROTARY_SLOTS.WHEEL_CW }[base];
  const mirror = twinBase !== undefined && isScrollType(action.type);
  const writes = [[slot, action]];
  if (mirror) writes.push([rotarySlot(twinBase, view.layer), { ...action }]);

  // Editando una variación se guardan diferencias en el PC; el teclado solo se
  // toca si esa variación es la que está puesta ahora mismo.
  if (variant) {
    for (const [s, a] of writes) variants.setOverride(variant.id, 'rotary', s, a);
    render();
    if (variants.isApplied(variant.id)) {
      try {
        for (const [s, a] of writes) {
          await variants.writeAppliedSlot(variant.id, 'rotary', s, a);
        }
      } catch {
        toast('No se pudo escribir la acción del mando', 'error');
      }
    }
    return;
  }

  for (const [s, a] of writes) prof.rotary[s] = a;

  render();
  try {
    for (const [s, a] of writes) {
      await device.setRotary(view.editingProfile, s, a.type, a.modifier, a.keycode);
      await variants.reassertSlot('rotary', s);
    }
    markDirty();
  } catch {
    toast('No se pudo escribir la acción del mando', 'error');
  }
}

// Una macro "vale" para la pestaña App si es justo lo que esa pestaña sabe
// editar: un único paso de abrir algo. Cualquier otra cosa (varios pasos, u
// otro tipo) es una Secuencia de verdad.
function isAppMacro(id) {
  const acts = macroById(id)?.actions || [];
  return acts.length === 1 && acts[0].type === 'open_app';
}

// Igual que isAppMacro, pero para una acción de energía del PC (Suspender,
// Apagar…). Vive en la pestaña Multimedia por ser también una sola pulsación
// sin más ajustes que elegir cuál, aunque por dentro sea una macro de un paso
// (el firmware no puede tocar esto: ver electron/macros.js).
function isPowerMacro(id) {
  const acts = macroById(id)?.actions || [];
  return acts.length === 1 && acts[0].type === 'system_power';
}

// Una grabación no es una lista de pasos montada a mano, sino la captura de lo
// que hizo el usuario con el ratón y el teclado. Se marca con `kind` para
// distinguirla de una secuencia (ver la pestaña "Grabar").
function isRecordingMacro(id) {
  return macroById(id)?.kind === 'recording';
}

// describeAction solo ve lo que guarda el firmware (modificador + código), y
// ahí todas las macros son iguales. Las tres formas que sabe editar la app
// —secuencia, abrir algo y grabación— se distinguen mirando la macro.
function describeKey(action) {
  if (action.modifier !== MACRO_MODIFIER) return describeAction(action.modifier, action.keycode);
  if (isResetMacro(action.keycode)) return 'Borrar grabación';
  if (isRecordingMacro(action.keycode)) {
    return macroById(action.keycode).events?.length ? 'Reproducir grabación' : 'Grabar operación';
  }
  if (isAppMacro(action.keycode)) {
    const target = macroById(action.keycode).actions[0].target || '';
    return target ? `Abrir ${target.split(/[\\/]/).pop()}` : 'Abrir…';
  }
  if (isPowerMacro(action.keycode)) {
    return POWER_MODE_LABELS[macroById(action.keycode).actions[0].mode] || 'Energía';
  }
  return describeAction(action.modifier, action.keycode);
}

// Pestaña que abre el inspector de tecla por defecto, según lo que ya tenga
// asignado el hueco elegido.
function tabForAction(action) {
  if (action.modifier === CONSUMER_MODIFIER) return 'media';
  if (action.modifier === GOTO_PAGE_MODIFIER || action.modifier === PAGE_STATE_MODIFIER) return 'pages';
  if (action.modifier === MACRO_MODIFIER) {
    if (isRecordOrResetMacro(action.keycode)) return 'record';
    if (isPowerMacro(action.keycode)) return 'media';
    return isAppMacro(action.keycode) ? 'app' : 'sequence';
  }
  return 'shortcut';
}

// Cambia de pestaña en el inspector de tecla. La primera vez que se entra en
// "Secuencia", "App" o "Grabar" sin que el hueco ya tenga una macro de esa
// forma, se crea una macro nueva (vacía, con un paso "abrir" en blanco, o de
// tipo grabación) y se asigna al vuelo. Si ya había una macro de la forma
// correcta (se venía de la misma pestaña, o se vuelve a ella), se reutiliza tal
// cual, sin perder su contenido.
function setTab(tab) {
  const prevTab = view.tab;
  view.tab = tab;

  // Se abandona una grabación: su tecla de borrado en SUPER deja de tener
  // sentido, así que se va con ella.
  if (prevTab === 'record' && tab !== 'record') {
    const leaving = currentAction();
    if (leaving.modifier === MACRO_MODIFIER && isRecordOrResetMacro(leaving.keycode)) {
      const rec = recordMacro(leaving.keycode);
      if (rec) detachResetKey(rec.id);
    }
  }

  if (tab === 'sequence' && prevTab !== 'sequence') {
    const action = currentAction();
    if (action.modifier !== MACRO_MODIFIER || isAppMacro(action.keycode)
        || isPowerMacro(action.keycode) || isRecordOrResetMacro(action.keycode)) {
      const id = nextMacroId();
      ensureMacro(id);
      applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
      return;
    }
  }

  if (tab === 'app' && prevTab !== 'app') {
    const action = currentAction();
    if (action.modifier !== MACRO_MODIFIER || !isAppMacro(action.keycode)) {
      const id = nextMacroId();
      const m = ensureMacro(id);
      m.actions = [{ type: 'open_app', target: '' }];
      savePCMacros(id);
      applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
      return;
    }
  }

  if (tab === 'record' && prevTab !== 'record') {
    const action = currentAction();
    if (action.modifier !== MACRO_MODIFIER || !isRecordOrResetMacro(action.keycode)) {
      startRecordingKey();
      return;
    }
  }

  render();
}

// Convierte la tecla seleccionada en una tecla de grabación: la macro, la
// tecla de borrado en la capa SUPER y los iconos que explican qué hace cada
// una (RECORD / RESET, y PLAY en cuanto haya algo grabado).
async function startRecordingKey() {
  const id = nextMacroId();
  const m = ensureMacro(id);
  m.kind = 'recording';
  m.mode = 'once';
  m.events = [];
  // Sin pasos: así macroDeviceEligible la descarta y el teclado avisa siempre
  // por CDC en vez de intentar tocarla él, que no puede.
  m.actions = [];
  savePCMacros(id);

  await applyKeymap(MACRO_MODIFIER, id);
  await attachResetKey(id);
  await refreshRecordIcons(id);
  render();
}

// Macro asignada al hueco seleccionado (tecla o mando), o null si no lo es.
function currentMacroId() {
  const a = view.selected?.kind === 'rotary' ? currentRotary() : currentAction();
  return a.modifier === MACRO_MODIFIER ? a.keycode : null;
}

// ¿Son el mismo gesto repetido? Solo tiene sentido para pasos de una sola
// pulsación/clic: repetir un movimiento de ratón o una posición no significa
// nada (cada uno lleva sus propias coordenadas).
function sameStep(a, b) {
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
function seqAddAction(action) {
  const id = currentMacroId();
  if (id === null) return;
  const m = ensureMacro(id);
  const last = m.actions[m.actions.length - 1];
  if (last && last.type !== 'delay' && sameStep(last, action)) {
    last.count = (last.count || 1) + (action.count || 1);
    savePCMacros(id);
    render();
    return;
  }
  if (m.actions.length && m.actions[m.actions.length - 1].type !== 'delay') {
    m.actions.push({ type: 'delay', ms: DEFAULT_STEP_DELAY_MS });
  }
  m.actions.push({ ...action, count: action.count || 1 });
  savePCMacros(id);
  render();
}

// --- Pestaña "App": una tecla que abre directamente una app o un archivo ---
// Reutiliza el mismo mecanismo de macro que "Secuencia" (MACRO_MODIFIER + id,
// ver más arriba), pero limitado a un único paso "open_app": no hay lista de
// pasos que gestionar, solo el objetivo. Ver isAppMacro/setTab para cómo se
// distingue de una secuencia de verdad.
function currentAppStep() {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  if (!m) return null;
  if (!m.actions.length) m.actions.push({ type: 'open_app', target: '' });
  return m.actions[0];
}

function setAppKind(kind) {
  const step = currentAppStep();
  if (!step) return;
  step.kind = kind;
  savePCMacros(currentMacroId());
  render();
}

// --- Pestaña "Multimedia", sección de energía ------------------------------
// Mismo mecanismo que "App" (macro de un único paso, ver isPowerMacro), pero
// sin pestaña propia: se asigna al vuelo con un solo clic, igual que una
// acción multimedia normal. Si la tecla ya era una acción de energía se
// reutiliza la misma macro (solo cambia el modo); si no, se crea una nueva.
function setPowerAction(mode) {
  const action = currentAction();
  const id = (action.modifier === MACRO_MODIFIER && isPowerMacro(action.keycode))
    ? action.keycode : nextMacroId();
  const m = ensureMacro(id);
  m.actions = [{ type: 'system_power', mode }];
  savePCMacros(id);
  applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
}

async function pickAppTarget(kind) {
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
  render();
}

// Toma directamente la app que está en primer plano ahora mismo: mismo
// detector que usa "Añadir app actual" en las variaciones (ver
// getCurrentWindowInfo más arriba), pero aquí hace falta el ejecutable
// entero, no solo el nombre del proceso.
async function pickAppFocus() {
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
  render();
  toast(`Añadida "${info.process || info.path}"`);
}

// Apps instaladas para el buscador de la pestaña "App": se piden una vez al
// proceso principal (recorre el menú Inicio, ver electron/apps.js) y se
// guardan en memoria; no cambian mientras la app sigue abierta.
let installedApps = [];
let installedAppsLoading = false;

function ensureInstalledApps() {
  if (installedAppsLoading || installedApps.length) return;
  installedAppsLoading = true;
  window.orby.listInstalledApps()
    .then((list) => { installedApps = list || []; })
    .catch(() => { installedApps = []; })
    .finally(() => {
      installedAppsLoading = false;
      if (isActiveView() && (view.tab === 'app' || view.tab === 'sequence')) render();
    });
}

// Compartido entre la pestaña "App" y los pasos "Abrir" de una secuencia:
// mismo <datalist>, mismo id, así que el navegador solo necesita uno vivo en
// el DOM a la vez (las dos pestañas nunca se enseñan juntas).
function installedAppsDatalist() {
  return `<datalist id="installed-apps-list">
    ${installedApps.map((a) => `<option value="${escape(a.target)}" label="${escape(a.name)}"></option>`).join('')}
  </datalist>`;
}

function renderAppTab(macroId) {
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

// Abre el selector nativo de archivos para elegir qué abrir con este paso.
// Sin `editIndex`, añade un paso nuevo; con él, sustituye el objetivo del
// paso existente (botón "Examinar" junto a cada paso "Abrir…").
async function pickOpenTarget(editIndex = null, kind = 'app') {
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
    if (step) { step.target = picked.path; step.kind = kind; savePCMacros(currentMacroId()); render(); }
  } else {
    seqAddAction({ type: 'open_app', target: picked.path, kind });
  }
}

// Cambia si un paso "Abrir" de la secuencia busca una app o un archivo:
// mismo distingo que en la pestaña App (ver setAppKind), pero por índice de
// paso en vez de sobre el único paso de esa pestaña.
function setSeqOpenKind(index, kind) {
  const step = seqActionAt(index);
  if (!step) return;
  step.kind = kind;
  savePCMacros(currentMacroId());
  render();
}

// Botón "App en foco" de un paso de secuencia: mismo mecanismo que
// pickAppFocus (pestaña App), pero escribe en el paso `index` en vez de en
// el único paso de esa pestaña.
async function pickSeqOpenFocus(index) {
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
  render();
  toast(`Añadida "${info.process || info.path}"`);
}

function seqRemoveAction(index) {
  const id = currentMacroId();
  if (id === null) return;
  const m = macroById(id);
  if (!m) return;
  m.actions.splice(index, 1);
  normalizeSequence(m);
  savePCMacros(id);
  render();
}

// Reordena un paso real (nunca una espera) intercambiándolo con el paso real
// más cercano en esa dirección. Las esperas se quedan donde están: siguen
// marcando el hueco entre "el paso de antes" y "el de después" de esa
// posición de la lista, sea cual sea la acción que acabe ocupándola.
function seqMoveAction(index, dir) {
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
  render();
}

// Quita esperas huérfanas después de borrar un paso: ninguna al principio, ni
// al final, ni dos seguidas (podría pasar al borrar el paso que había entre
// ambas).
function normalizeSequence(m) {
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
function seqActionAt(index) {
  const id = currentMacroId();
  const m = id === null ? null : macroById(id);
  return m?.actions[index] || null;
}

// Constructor de un paso "tecla": el mismo selector de modificadores + tecla
// que la pestaña Atajo, pero como paso de secuencia en vez de la acción de la
// tecla física. Estado transitorio, se resetea la tecla (no los modificadores,
// para encadenar varias con el mismo modificador) tras cada "Añadir".
let seqKeyBuilder = { modifier: 0, keycode: 0 };

// --- Captura de posición del ratón ------------------------------------------
// A diferencia de la grabación de teclas, aquí el ratón es la entrada: se
// enseña la posición del cursor en pantalla en vivo (viene del proceso
// principal porque puede estar fuera de la ventana) y Esc fija el paso.
let lastMousePos = { x: 0, y: 0 };
let posPollTimer = null;
// null = capturando un paso nuevo; si no, índice del paso que se está
// recapturando (el botón de destino junto a cada "Posición de ratón").
let capturePosEditIndex = null;

function startPositionCapture(editIndex = null) {
  view.capturing = 'position';
  capturePosEditIndex = editIndex;
  render();
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

function stopPositionCapture(commit) {
  if (posPollTimer) { clearInterval(posPollTimer); posPollTimer = null; }
  view.capturing = false;
  const editIndex = capturePosEditIndex;
  capturePosEditIndex = null;

  if (!commit) { render(); return; }

  if (editIndex !== null) {
    const step = seqActionAt(editIndex);
    if (step) {
      step.type = 'mouse_position';
      step.x = lastMousePos.x;
      step.y = lastMousePos.y;
      savePCMacros();
    }
    render();
  } else {
    seqAddAction({ type: 'mouse_position', x: lastMousePos.x, y: lastMousePos.y });
  }
}

// Botón "Secuencia" del inspector de mando: mismo mecanismo que en las teclas,
// pero sin pestañas, porque el mando ya elige su acción con el type-grid.
function applyRotaryMacro() {
  if (currentRotary().modifier === MACRO_MODIFIER) return;
  const id = nextMacroId();
  ensureMacro(id);
  applyRotary({ type: ROTARY_TYPES.KEY, modifier: MACRO_MODIFIER, keycode: id });
}

// Sensibilidad e inversión de la rueda: un ajuste más del perfil y de la capa.
async function applyScroll(patch) {
  const prof = currentProfile();
  if (!prof || !requireDevice()) return;

  const li = layerIndex(view.layer);
  const next = { ...scrollFor(prof, view.layer), ...patch };
  prof.scroll[li] = next;

  // Si es lo que el teclado está usando ahora mismo, el resto de la app tiene
  // que enterarse (el dashboard enseña la sensibilidad en vivo).
  if (view.editingProfile === state.activeProfileIdx && li === (state.superActive ? 1 : 0)) {
    state.scroll = { ...state.scroll, ...next };
  }

  render();
  try {
    await device.setProfileScroll(view.editingProfile, li, next.detentsPerRev, next.invert);
    markDirty();
  } catch {
    toast('El teclado no confirmó la calibración de la rueda', 'error');
  }
}

// --- Render ----------------------------------------------------------------

export function render() {
  const body = document.getElementById('profiles-body');
  if (!body) return;

  // El repintado sustituye TODO el innerHTML (rejilla e inspector incluidos),
  // así que sin esto cada tecleo en un campo o cada clic perdía el scroll de
  // las dos columnas y las devolvía arriba del todo.
  const scrollMain = body.querySelector('.editor-main')?.scrollTop;
  const scrollInspector = body.querySelector('.editor-inspector')?.scrollTop;

  // Cualquier repintado que no sea "seguimos capturando una posición" corta
  // el sondeo: evita dejarlo corriendo de fondo si se sale del modo por otro
  // sitio (elegir otra tecla, cambiar de pestaña...).
  if (view.capturing !== 'position' && posPollTimer) {
    clearInterval(posPollTimer);
    posPollTimer = null;
  }

  if (!state.profiles.length) {
    body.innerHTML = `<div class="empty-panel glass-panel">
      ${icon('plug', 40)}
      <h3>Sin perfiles cargados</h3>
      <p>Conecta el Orby una vez: a partir de ahí queda una copia en el PC y podrás
         editarlos aunque no lo tengas enchufado.</p>
    </div>`;
    return;
  }

  if (view.editingProfile >= state.profiles.length) view.editingProfile = 0;
  const prof = currentProfile();

  body.innerHTML = `
    <div class="profile-bar" id="profile-bar">${renderTabsInner()}</div>
    ${renderVariantBar()}

    <div class="editor-layout">
      <div class="editor-main">
        ${renderVariantSettings()}
        <div class="editor-board glass-panel">
          <div class="editor-board-head">
            <div class="head-row">
              <label class="field-inline">
                <span>Nombre</span>
                <input type="text" maxlength="7" value="${escape(prof.name)}"
                       data-act="edit-name" class="text-input compact">
              </label>
              ${view.editingProfile === state.activeProfileIdx
                ? '<span class="pill pill-live">Perfil activo</span>'
                : '<button class="secondary-btn" data-act="activate">Activar en el teclado</button>'}
            </div>
            <div class="head-row">
              <div class="layer-toggle">
                <button class="toggle-btn ${view.layer === 'normal' ? 'active' : ''}" data-act="layer" data-layer="normal">NORMAL</button>
                <button class="toggle-btn ${view.layer === 'super' ? 'active' : ''}" data-act="layer" data-layer="super">SUPER</button>
              </div>
              ${renderPageBar()}
            </div>
          </div>
          ${renderPageHint()}

          <div class="okey-grid" id="profile-key-grid">${renderKeyGridInner()}</div>
          <p class="grid-status" id="profile-grid-status"></p>
        </div>

        <div class="glass-panel oled-card">
          <div class="card-header">${icon('reset', 20)}<h2>Encoders</h2></div>
          <div class="rotary-groups">${renderRotaryGroups()}</div>
          <p class="setting-desc mt-4">
            Cada capa guarda sus propias acciones: con <strong>SUPER</strong> mantenida los encoders
            hacen lo que configures aquí en la capa SUPER. Dentro del menú del teclado siguen
            sirviendo para navegar.
          </p>
        </div>

        ${renderWheelCard()}
      </div>

      ${renderInspector()}
    </div>`;

  // Los <canvas> de las miniaturas no se pueden serializar en la plantilla: se
  // pintan después de insertar el HTML, tanto en la rejilla como en el inspector.
  cache.paintThumbs(body);
  paintKeyGrid();
  updateDerived(currentScroll().detentsPerRev);
  cache.loadProfile(view.editingProfile);

  if (scrollMain !== undefined) {
    const el = body.querySelector('.editor-main');
    if (el) el.scrollTop = scrollMain;
  }
  if (scrollInspector !== undefined) {
    const el = body.querySelector('.editor-inspector');
    if (el) el.scrollTop = scrollInspector;
  }
}

function renderTabsInner() {
  const full = state.profiles.length >= state.maxProfiles;
  return `
    <div class="profile-tabs">
      ${state.profiles.map((p, i) => `
        <button class="profile-tab ${i === view.editingProfile ? 'active' : ''} ${i === state.activeProfileIdx ? 'is-live' : ''}"
                data-act="pick-profile" data-idx="${i}">
          <span class="tab-icon" style="--accent:${profileMeta(i).accent}">${icon(profileMeta(i).icon, 18)}</span>
          <span class="tab-name">${escape(p.name)}</span>
          ${i === state.activeProfileIdx ? '<span class="tab-live">EN USO</span>' : ''}
        </button>`).join('')}
    </div>
    <div class="profile-bar-actions">
      <button class="secondary-btn" data-act="profile-new" ${full || view.busy ? 'disabled' : ''}
              title="${full ? `Máximo ${state.maxProfiles} perfiles` : 'Crear un perfil vacío'}">
        ${icon('plus', 16)} Nuevo
      </button>
      <button class="secondary-btn" data-act="profile-dup" ${full || view.busy ? 'disabled' : ''}
              title="Copiar el perfil actual con sus iconos">
        ${icon('profiles', 16)} Duplicar
      </button>
      <button class="secondary-btn danger" data-act="profile-del"
              ${state.profiles.length <= 1 || view.busy ? 'disabled' : ''}>
        ${icon('trash', 16)} Eliminar
      </button>
      <span class="profile-count">${state.profiles.length} / ${state.maxProfiles}</span>
    </div>`;
}

function renderTabs() {
  const bar = document.getElementById('profile-bar');
  if (bar) bar.innerHTML = renderTabsInner();
}

// Variaciones del perfil: el mismo juego de atajos con un par de teclas
// distintas para una aplicación concreta, sin duplicar el perfil entero.
function renderVariantBar() {
  // Solo las de esta página: la misma tecla en otra página es otro atajo, así
  // que una variación de la 2 no tiene nada que decir mientras se edita la 1.
  const list = variants.forProfile(view.editingProfile, currentPageIdx());

  const prof = state.profiles[view.editingProfile];
  const pageTxt = (hasPages() && prof && pageCountOf(prof) > 1)
    ? ` · página ${currentPageIdx() + 1}` : '';

  return `
    <div class="variant-bar">
      <span class="variant-bar-label">${icon('bolt', 14)} Según la app${pageTxt}</span>
      <div class="chip-row">
        <button class="chip ${!view.variantId ? 'on' : ''}" data-act="pick-variant" data-id="">
          Perfil base
        </button>
        ${list.map((v) => `
          <button class="chip ${view.variantId === v.id ? 'on' : ''} ${variants.isApplied(v.id) ? 'is-live' : ''}"
                  data-act="pick-variant" data-id="${v.id}"
                  title="${escape(v.matches.length ? `Se aplica con: ${v.matches.join(', ')}` : 'Sin aplicaciones asignadas')}">
            ${escape(v.name)}
            <em class="chip-count">${variants.countOverrides(v)}</em>
          </button>`).join('')}
        <button class="chip ghost" data-act="new-variant">${icon('plus', 14)} Nueva variación</button>
      </div>
    </div>`;
}

// ---------- Páginas ----------
// Editar la página N exige que el teclado tenga puesta la página N, porque los
// comandos de edición del firmware actúan sobre la activa. Por eso elegir una
// pestaña la cambia también en el teclado (y solo tiene sentido en el perfil
// activo; en los demás se edita su primera página).
async function choosePage(idx) {
  const prof = state.profiles[view.editingProfile];
  if (!prof || idx === (prof.pageIdx || 0)) return;

  // La restricción es del firmware: de un perfil que no tiene puesto solo sabe
  // llegar a su primera página. Sin teclado delante no pinta nada, porque
  // entonces esto es solo mirar la copia local, que no se puede editar.
  if (state.connected && view.editingProfile !== state.activeProfileIdx) {
    toast('Para editar otra página, activa antes este perfil en el teclado');
    return;
  }

  view.selected = null;
  view.variantId = null;   // las variaciones son de cada página

  if (!state.connected) {
    prof.pageIdx = idx;
    render();
    return;
  }

  try {
    await selectPage(idx);
    cache.loadProfile(view.editingProfile);   // los iconos son de cada página
    render();
  } catch (e) {
    toast(`No he podido cambiar de página: ${e.message}`);
    render();
  }
}

async function createPage() {
  const prof = state.profiles[view.editingProfile];
  if (!prof || !requireDevice()) return;
  if (view.editingProfile !== state.activeProfileIdx) {
    toast('Activa este perfil en el teclado para añadirle páginas');
    return;
  }
  try {
    if (await addPage()) {
      const p = state.profiles[view.editingProfile];
      await selectPage(pageCountOf(p) - 1);   // abrir la recién creada
      cache.loadProfile(view.editingProfile);
      toast(`Página ${pageCountOf(p)} añadida, copiando la anterior`);
    } else {
      toast(`El tope es de ${maxPages()} páginas por perfil`);
    }
    render();
  } catch (e) {
    toast(`No he podido añadir la página: ${e.message}`);
  }
}

async function deletePage(idx) {
  const prof = state.profiles[view.editingProfile];
  if (!prof || pageCountOf(prof) <= 1 || !requireDevice()) return;
  if (view.editingProfile !== state.activeProfileIdx) {
    toast('Activa este perfil en el teclado para borrarle páginas');
    return;
  }
  if (!confirm(`¿Eliminar la página ${idx + 1} de ${escape(prof.name)}?\n\n`
             + 'Se van sus teclas, etiquetas, mandos e iconos. Las páginas siguientes '
             + 'se recolocan.')) return;
  try {
    await removePage(idx);
    variants.shiftPages(view.editingProfile, idx);
    cache.loadProfile(view.editingProfile);
    view.selected = null;
    view.variantId = null;
    toast(`Página ${idx + 1} eliminada`);
    render();
  } catch (e) {
    toast(`No he podido eliminar la página: ${e.message}`);
  }
}

// Pestañas de página. Solo aparecen si el firmware las soporta: con uno anterior
// al 4.0, maxPages es 1 y la barra no se dibuja, así que la vista queda como antes.
function renderPageBar() {
  if (!hasPages()) return '';
  const prof = state.profiles[view.editingProfile];
  if (!prof) return '';
  const count = pageCountOf(prof);
  const cur = prof.pageIdx || 0;

  let tabs = '';
  for (let i = 0; i < count; i++) {
    tabs += `<button class="toggle-btn ${i === cur ? 'active' : ''}"
                     data-act="page" data-page="${i}" title="Página ${i + 1}">${i + 1}</button>`;
  }
  const canAdd = count < maxPages();
  return `
    <div class="layer-toggle page-toggle">
      ${tabs}
      ${canAdd ? `<button class="toggle-btn page-add" data-act="page-add"
                          title="Añadir una página copiando la actual">+</button>` : ''}
      ${count > 1 ? `<button class="toggle-btn page-del" data-act="page-del" data-page="${cur}"
                             title="Eliminar la página ${cur + 1}">−</button>` : ''}
    </div>`;
}

function renderPageHint() {
  if (!hasPages()) return '';
  const prof = state.profiles[view.editingProfile];
  if (!prof) return '';
  const count = pageCountOf(prof);
  if (count <= 1) {
    return `<p class="grid-status">Este perfil tiene una sola página. Con
      <strong>+</strong> añades otra copiando esta, y el teclado alterna entre ellas
      con una pulsación corta del botón de menú.</p>`;
  }
  const live = view.editingProfile === state.activeProfileIdx;
  return `<p class="grid-status">Editando la <strong>página ${(prof.pageIdx || 0) + 1}</strong>
    de ${count}. Cada página tiene sus teclas, etiquetas, mandos, rueda e iconos.
    ${live ? 'La página que elijas aquí es la que se pone en el teclado, así que las pantallas enseñan lo que estás tocando.' : ''}</p>`;
}

function renderVariantSettings() {
  const variant = editingVariant();
  if (!variant) return '';

  const applied = variants.isApplied(variant.id);
  const count = variants.countOverrides(variant);

  return `
    <div class="glass-panel oled-card variant-card">
      <div class="card-header">
        ${icon('bolt', 20)}<h2>Variación de ${escape(currentProfile().name)}</h2>
        <span class="pill ${applied ? 'pill-live' : ''}">${applied ? 'Aplicada ahora' : `${count} cambios`}</span>
      </div>

      <div class="row-inline">
        <label class="field">
          <span class="field-label">Nombre</span>
          <input type="text" class="text-input" value="${escape(variant.name)}" data-act="variant-name">
        </label>
        <label class="field">
          <span class="field-label">Comparar contra</span>
          <select class="select-input" data-act="variant-field">
            <option value="any"     ${variant.field === 'any' ? 'selected' : ''}>Programa o título</option>
            <option value="process" ${variant.field === 'process' ? 'selected' : ''}>Solo programa</option>
            <option value="title"   ${variant.field === 'title' ? 'selected' : ''}>Solo título</option>
          </select>
        </label>
      </div>

      <div class="field">
        <span class="field-label">Se aplica con estas aplicaciones</span>
        ${variant.matches.length ? `
          <div class="chip-row match-list">
            ${variant.matches.map((m) => `
              <span class="match-chip">
                ${escape(m)}
                <button class="match-x" data-act="del-match" data-match="${escape(m)}"
                        title="Quitar">${icon('close', 12)}</button>
              </span>`).join('')}
          </div>` : `
          <p class="setting-desc">Todavía ninguna: la variación no se aplicará sola.</p>`}

        <div class="row-inline">
          <input type="text" class="text-input compact match-input" id="variant-new-match"
                 placeholder="p. ej. notepad" spellcheck="false">
          <button class="secondary-btn" data-act="add-match">${icon('plus', 16)} Añadir</button>
          <button class="secondary-btn" data-act="add-match-current">${icon('plug', 16)} La app de delante</button>
        </div>
      </div>

      <p class="setting-desc">
        Estás editando <strong>solo las diferencias</strong>: lo que cambies aquí se
        guarda como excepción de este perfil para esas apps, y todo lo demás lo sigue
        mandando el perfil base. Las teclas y mandos redefinidos salen marcados abajo.
        ${count === 0 ? '<br>Todavía no hay ninguna diferencia: cambia una tecla para empezar.' : ''}
      </p>

      <div class="row-inline">
        <button class="secondary-btn danger" data-act="del-variant">${icon('trash', 16)} Eliminar variación</button>
      </div>
    </div>`;
}

// Réplica del teclado con el icono real de cada tecla, igual que en el editor
// de iconos: es la representación que ya conoce el usuario.
function renderKeyGridInner() {
  const prof = currentProfile();
  const variant = editingVariant();
  const selected = selectedKeyIndex();
  let html = '';

  for (let i = 0; i < 12; i++) {
    const screen = KEY_TO_SCREEN[i];
    const lslot = labelSlot(i, view.layer);
    const action = variants.effectiveKey(prof, variant, i, view.layer);
    const label = lslot >= 0 ? variants.effectiveLabel(prof, variant, i, view.layer) : '';
    const bmp = lslot >= 0 ? cache.get(view.editingProfile, lslot) : null;
    const assigned = action.modifier || action.keycode;
    // Marca las teclas que la variación redefine respecto al perfil base.
    const changed = variant && (variants.override(variant, 'keys', keymapSlot(i, view.layer))
                             || variants.override(variant, 'labels', lslot));

    html += `
      <button class="okey ${selected === i ? 'selected' : ''} ${bmp ? 'has-icon' : ''} ${screen ? '' : 'roleless'} ${changed ? 'is-override' : ''}"
              data-act="pick-key" data-key="${i}">
        <span class="okey-num">T${i + 1}${screen ? `<em>P${screen}</em>` : ''}${changed ? '<i class="okey-dot" title="Cambiado en esta variación"></i>' : ''}</span>
        <span class="okey-screen">
          ${screen
            ? (bmp ? `<canvas class="okey-canvas" data-bmp="${cache.cacheKey(view.editingProfile, lslot)}"></canvas>`
                   : `<span class="okey-text">${escape(label || '—')}</span>`)
            : `<span class="okey-role">${i + 1 === KEY_SUPER ? 'SUPER' : 'MENÚ'}</span>`}
        </span>
        <span class="okey-action ${assigned ? 'assigned' : ''}">${escape(describeKey(action))}</span>
      </button>`;
  }
  return html;
}

function paintKeyGrid() {
  const grid = document.getElementById('profile-key-grid');
  if (grid) cache.paintThumbs(grid);

  const status = document.getElementById('profile-grid-status');
  if (status) {
    status.textContent = cache.isLoading()
      ? 'Leyendo iconos del teclado…'
      : `Capa ${view.layer === 'super' ? 'SUPER' : 'NORMAL'} · las teclas ${KEY_MENU} y ${KEY_SUPER} `
        + `no tienen pantalla: la ${KEY_SUPER} es el modificador SUPER y la ${KEY_MENU} abre el menú `
        + 'al mantenerla.';
  }
}

function renderKeyGrid() {
  const grid = document.getElementById('profile-key-grid');
  if (!grid) return;
  grid.innerHTML = renderKeyGridInner();
  paintKeyGrid();
}

function renderRotaryGroups() {
  const prof = currentProfile();
  const variant = editingVariant();
  const selected = selectedRotarySlot();

  return ROTARY_GROUPS.map((group) => `
    <div class="rotary-group">
      <span class="rotary-group-name">${icon(group.icon, 14)} ${group.name}</span>
      ${group.parts.map((part) => {
        const action = variants.effectiveRotary(prof, variant, part.slot, view.layer);
        const changed = variant && variants.override(variant, 'rotary', rotarySlot(part.slot, view.layer));
        return `
          <button class="rotary-part ${selected === part.slot ? 'selected' : ''} ${action.type ? 'assigned' : ''} ${changed ? 'is-override' : ''}"
                  data-act="pick-rotary" data-slot="${part.slot}">
            <span class="rp-dir">${part.short}</span>
            <span class="rp-body">
              <em>${part.label}</em>
              <strong>${escape(describeRotary(action))}</strong>
            </span>
          </button>`;
      }).join('')}
    </div>`).join('');
}

function renderWheelCard() {
  const s = currentScroll();
  const layerName = view.layer === 'super' ? 'SUPER' : 'normal';
  const prof = currentProfile();
  const variant = editingVariant();
  const cw  = variants.effectiveRotary(prof, variant, ROTARY_SLOTS.WHEEL_CW,  view.layer);
  const ccw = variants.effectiveRotary(prof, variant, ROTARY_SLOTS.WHEEL_CCW, view.layer);

  return `
    <div class="glass-panel oled-card">
      <div class="card-header">
        ${icon('wheel', 20)}<h2>Rueda de scroll</h2>
        <span class="pill">Capa ${layerName}</span>
      </div>

      ${editingVariant() ? `
        <p class="setting-desc override-note">
          La sensibilidad de la rueda es siempre la del <strong>perfil base</strong>:
          las variaciones cambian atajos y etiquetas, no la calibración.
        </p>` : ''}

      <!-- El slider ocupa el ancho completo de la tarjeta, fuera de la rejilla de
           dos columnas: son 234 pasos y cuanto más largo es el recorrido, más
           fino se puede afinar el punto exacto. -->
      <div class="wheel-tune-head">
        <div class="wheel-readout">
          <span id="scroll-value">${s.detentsPerRev}</span>
          <small>clics por vuelta completa</small>
        </div>

        <input type="range" id="scroll-slider" class="premium-slider wheel-slider" data-act="scroll-slider"
               min="3" max="120" step="1" value="${s.detentsPerRev}">

        <div class="slider-row">
          <span>Más lento · 3</span>
          <span>120 · Más rápido</span>
        </div>
      </div>

      <div class="wheel-tune">
        <div class="wheel-tune-main">
          <div class="preset-row">
            ${SCROLL_PRESETS.map((p) => `
              <button class="preset-btn ${s.detentsPerRev === p.value ? 'on' : ''}" data-act="scroll-preset" data-value="${p.value}">
                <strong>${p.name}</strong>
                <span>${p.value} clics</span>
                <em>${p.desc}</em>
              </button>`).join('')}
          </div>

          <ul class="info-list" id="scroll-derived"></ul>

          <div class="row-inline mt-4">
            <button class="secondary-btn ${s.invert ? 'is-on' : ''}" data-act="scroll-invert">
              ${icon('reset', 16)} ${s.invert ? 'Dirección invertida' : 'Invertir dirección'}
            </button>
          </div>
        </div>

        <div class="wheel-tune-side">
          <div class="hires-state ${state.scroll.hires ? 'ok' : 'off'}">
            <span class="hires-dot"></span>
            <strong>${state.scroll.hires ? 'Alta resolución activa' : 'Alta resolución no negociada'}</strong>
          </div>
          <div class="hires-state ${state.scroll.hiresPan ? 'ok' : 'off'}">
            <span class="hires-dot"></span>
            <strong>${state.scroll.hiresPan
              ? 'Paneo horizontal en alta resolución'
              : 'Paneo horizontal sin negociar'}</strong>
          </div>

          <div class="field">
            <span class="field-label">Acción del giro</span>
            <button class="rotary-part ${selectedRotarySlot() === ROTARY_SLOTS.WHEEL_CW ? 'selected' : ''}"
                    data-act="pick-rotary" data-slot="${ROTARY_SLOTS.WHEEL_CW}">
              <span class="rp-dir">↓</span>
              <span class="rp-body"><em>Hacia abajo</em><strong>${escape(describeRotary(cw))}</strong></span>
            </button>
            <button class="rotary-part ${selectedRotarySlot() === ROTARY_SLOTS.WHEEL_CCW ? 'selected' : ''}"
                    data-act="pick-rotary" data-slot="${ROTARY_SLOTS.WHEEL_CCW}">
              <span class="rp-dir">↑</span>
              <span class="rp-body"><em>Hacia arriba</em><strong>${escape(describeRotary(ccw))}</strong></span>
            </button>
          </div>

          <p class="setting-desc">
            La sensibilidad pertenece a <strong>${escape(currentProfile().name)}</strong> en la capa
            ${layerName}: cada perfil —y cada capa— puede tener la suya. El dibujo de la rueda en
            pantalla (forma del marcador, sentido, desfase) se calibra en <strong>Ajustes</strong>.
            ${state.scroll.hires
              ? 'Con el multiplicador negociado el desplazamiento viaja en unidades de 1/120 de clic.'
              : 'Hasta que Windows pida el multiplicador, las aplicaciones antiguas saltarán de tres en tres líneas.'}
          </p>
        </div>
      </div>
    </div>`;
}

function updateDerived(detents) {
  const el = document.getElementById('scroll-derived');
  if (!el) return;
  const degPerDetent = 360 / detents;
  const unitsPerCount = (detents * 120) / 4096;
  el.innerHTML = `
    <li><span class="lbl">Giro por clic</span><span class="val">${degPerDetent.toFixed(1)}°</span></li>
    <li><span class="lbl">Unidades HID por cuenta</span><span class="val">${unitsPerCount.toFixed(3)}</span></li>
    <li><span class="lbl">Líneas por vuelta (Windows)</span><span class="val">${(detents * 3).toFixed(0)}</span></li>`;
}

// Lista de teclas HID reutilizada por los dos editores.
function keycodeOptions(selectedCode) {
  return KEY_GROUPS.map((g) => `
    <optgroup label="${g.name}">
      ${g.keys.map((k) => `<option value="${k.code}" ${selectedCode === k.code ? 'selected' : ''}>${escape(k.label)}</option>`).join('')}
    </optgroup>`).join('');
}

function renderInspector() {
  if (!view.selected) {
    return `<div class="editor-inspector glass-panel empty-panel">
      ${icon('key', 36)}
      <h3>Elige una tecla o un mando</h3>
      <p>Selecciona cualquier tecla para cambiar su icono, su etiqueta y su atajo, o un giro
         de los encoders y la rueda para reasignarlo.</p>
    </div>`;
  }

  return view.selected.kind === 'rotary' ? renderRotaryInspector() : renderKeyInspector();
}

function renderRotaryInspector() {
  const base = selectedRotarySlot();
  const found = rotaryPart(base);
  const action = currentRotary();
  const isClick = Boolean(found?.part.discrete);
  const isMacro = action.type === ROTARY_TYPES.KEY && action.modifier === MACRO_MODIFIER;
  const types = ROTARY_TYPE_OPTIONS.filter((t) => !(isClick && t.turnOnly));

  const variant = editingVariant();
  const changed = Boolean(variant && variants.override(variant, 'rotary', rotarySlot(base, view.layer)));
  const baseAction = currentProfile().rotary?.[rotarySlot(base, view.layer)] || { type: 0, modifier: 0, keycode: 0 };

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>${escape(found?.group.name || 'Mando')}</h3>
        <span class="pill">${view.layer === 'super' ? 'Capa SUPER' : 'Capa normal'}</span>
      </div>
      <p class="setting-desc inspector-sub">${escape(found?.part.label || '')}</p>

      ${variant ? `
        <div class="override-note ${changed ? 'is-override' : ''}">
          <span>${changed
            ? `Cambiado en <strong>${escape(variant.name)}</strong> · el perfil base hace
               <code>${escape(describeRotary(baseAction))}</code>`
            : `Editando <strong>${escape(variant.name)}</strong>: solo valdrá con esa app`}</span>
          ${changed ? `<button class="secondary-btn" data-act="clear-override">
                         ${icon('reset', 14)} Volver al valor base
                       </button>` : ''}
        </div>` : ''}

      <div class="field">
        <span class="field-label">Tipo de acción</span>
        <div class="type-grid">
          ${types.map((t) => `
            <button class="type-chip ${action.type === t.type && !(t.type === ROTARY_TYPES.KEY && isMacro) ? 'on' : ''}"
                    data-act="rotary-type" data-type="${t.type}">${t.label}</button>`).join('')}
          <button class="type-chip ${isMacro ? 'on' : ''}" data-act="rotary-macro">Secuencia</button>
        </div>
      </div>

      ${action.type === ROTARY_TYPES.CONSUMER ? `
        <div class="field">
          <span class="field-label">Acción</span>
          <div class="consumer-grid">
            ${CONSUMER_ACTIONS.map((c) => `
              <button class="consumer-chip ${action.keycode === c.index ? 'on' : ''}"
                      data-act="rotary-consumer" data-index="${c.index}">${c.label}</button>`).join('')}
          </div>
        </div>` : ''}

      ${isMacro ? renderSequenceEditor(action.keycode) : ''}

      ${action.type === ROTARY_TYPES.KEY && !isMacro ? `
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${MODIFIERS.map((m) => `
              <button class="mod-chip ${action.modifier & m.bit ? 'on' : ''}"
                      data-act="rotary-mod" data-bit="${m.bit}">${m.label}</button>`).join('')}
          </div>
        </div>
        <label class="field">
          <span class="field-label">Tecla</span>
          <select class="select-input" data-act="rotary-keycode">
            <option value="0" ${!action.keycode ? 'selected' : ''}>— ninguna —</option>
            ${keycodeOptions(action.keycode)}
          </select>
        </label>
        <button class="primary-btn full ${view.capturing ? 'is-capturing' : ''}" data-act="capture">
          ${icon('key', 16)} ${view.capturing ? 'Pulsa el atajo… (Esc cancela)' : 'Capturar atajo del teclado'}
        </button>` : ''}

      ${isScrollType(action.type) ? `
        <p class="setting-desc">
          El sentido va implícito en el giro, así que esta acción cubre las dos direcciones.
          ${action.type === ROTARY_TYPES.SCROLL_V
            ? 'En la rueda magnética es la única opción que aprovecha la alta resolución; el resto trabajan por clics completos.'
            : ''}
        </p>` : ''}

      <div class="inspector-summary">
        <span class="field-label">Resultado</span>
        <code>${escape(describeRotary(action))}</code>
      </div>
    </div>`;
}

function renderKeyInspector() {
  const i = selectedKeyIndex();
  const prof = currentProfile();
  const variant = editingVariant();
  const lslot = labelSlot(i, view.layer);
  const action = currentAction();
  // Los cuatro modos que se apropian del campo del modificador: mientras uno
  // esté puesto, ni los modificadores ni la tecla pintan nada.
  const isConsumer = action.modifier === CONSUMER_MODIFIER;
  const isMacro     = action.modifier === MACRO_MODIFIER;
  const isSpecial  = isConsumer || isMacro || action.modifier === GOTO_PAGE_MODIFIER
                                            || action.modifier === PAGE_STATE_MODIFIER;
  const keyOptions = keycodeOptions(isSpecial ? 0 : action.keycode);
  const hasIcon = lslot >= 0 && Boolean(cache.get(view.editingProfile, lslot));
  const baseAction = prof.keys[keymapSlot(i, view.layer)] || { modifier: 0, keycode: 0 };
  const changed = Boolean(variant && (variants.override(variant, 'keys', keymapSlot(i, view.layer))
                                   || variants.override(variant, 'labels', lslot)));
  const tab = view.tab;

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>Tecla ${i + 1}</h3>
        <div class="row-inline" style="gap:6px">
          <button class="tool-btn small" data-act="copy-key" title="Copiar icono, etiqueta y acción de esta tecla">
            ${icon('copy', 15)}
          </button>
          <button class="tool-btn small" data-act="paste-key" title="${keyClipboard ? 'Pegar en esta tecla' : 'Copia una tecla primero'}"
                  ${keyClipboard ? '' : 'disabled'}>
            ${icon('paste', 15)}
          </button>
          <button class="tool-btn small" data-act="clear-action" title="Quita el atajo o la macro y deja la pantalla de la tecla en negro">
            ${icon('trash', 15)}
          </button>
          <span class="pill">${view.layer === 'super' ? 'Capa SUPER' : 'Capa normal'}</span>
        </div>
      </div>

      ${variant ? `
        <div class="override-note ${changed ? 'is-override' : ''}">
          <span>${changed
            ? `Cambiada en <strong>${escape(variant.name)}</strong> · el perfil base hace
               <code>${escape(describeAction(baseAction.modifier, baseAction.keycode))}</code>`
            : `Editando <strong>${escape(variant.name)}</strong>: lo que cambies aquí solo
               valdrá con esa app`}</span>
          ${changed ? `<button class="secondary-btn" data-act="clear-override">
                         ${icon('reset', 14)} Volver al valor base
                       </button>` : ''}
        </div>` : ''}

      ${lslot >= 0 ? `
        <div class="field">
          <span class="field-label">Pantalla OLED</span>
          <div class="inspector-screen">
            <span class="okey-screen">
              ${hasIcon ? `<canvas class="okey-canvas" data-bmp="${cache.cacheKey(view.editingProfile, lslot)}"></canvas>`
                        : `<span class="okey-text">${escape(prof.labels[lslot] || '—')}</span>`}
            </span>
            <button class="secondary-btn" data-act="edit-icon" data-key="${i}">
              ${icon('pencil', 16)} ${hasIcon ? 'Editar icono' : 'Dibujar icono'}
            </button>
          </div>
        </div>` : `
        <p class="setting-desc">
          Esta tecla no tiene pantalla: la ${KEY_SUPER} es el modificador SUPER y la ${KEY_MENU}
          abre el menú del teclado al mantenerla, así que el firmware no ejecuta su atajo.
        </p>`}

      <div class="inspector-tabs">
        <button class="inspector-tab ${tab === 'shortcut' ? 'active' : ''}" data-act="set-tab" data-tab="shortcut">Atajo</button>
        <button class="inspector-tab ${tab === 'sequence' ? 'active' : ''}" data-act="set-tab" data-tab="sequence">Secuencia</button>
        <button class="inspector-tab ${tab === 'record' ? 'active' : ''}" data-act="set-tab" data-tab="record">Grabar</button>
        <button class="inspector-tab ${tab === 'app' ? 'active' : ''}" data-act="set-tab" data-tab="app">App</button>
        <button class="inspector-tab ${tab === 'media' ? 'active' : ''}" data-act="set-tab" data-tab="media">Multimedia</button>
        ${hasPages() ? `<button class="inspector-tab ${tab === 'pages' ? 'active' : ''}" data-act="set-tab" data-tab="pages">Páginas</button>` : ''}
      </div>

      ${tab === 'shortcut' ? `
        <div class="field">
          <span class="field-label">Modificadores</span>
          <div class="mod-grid">
            ${MODIFIERS.map((m) => `
              <button class="mod-chip ${!isSpecial && (action.modifier & m.bit) ? 'on' : ''}"
                      data-act="toggle-mod" data-bit="${m.bit}">${m.label}</button>`).join('')}
          </div>
        </div>

        <label class="field">
          <span class="field-label">Tecla</span>
          <select class="select-input" data-act="pick-keycode">
            <option value="0" ${!isSpecial && !action.keycode ? 'selected' : ''}>— ninguna —</option>
            ${keyOptions}
          </select>
        </label>

        <div class="inspector-actions">
          <button class="primary-btn full ${view.capturing === true ? 'is-capturing' : ''}" data-act="capture">
            ${icon('key', 16)} ${view.capturing === true ? 'Pulsa el atajo… (Esc cancela)' : 'Capturar atajo del teclado'}
          </button>
        </div>` : ''}

      ${tab === 'sequence' ? renderSequenceEditor(isMacro ? action.keycode : null) : ''}

      ${tab === 'record' ? renderRecordTab(isMacro ? action.keycode : null) : ''}

      ${tab === 'app' ? renderAppTab(isMacro ? action.keycode : null) : ''}

      ${tab === 'media' ? `
        <div class="field">
          <span class="field-label">Acción multimedia</span>
          <div class="consumer-grid consumer-grid-3col">
            ${CONSUMER_ACTIONS.map((c) => `
              <button class="consumer-chip ${isConsumer && action.keycode === c.index ? 'on' : ''}"
                      data-act="set-consumer" data-index="${c.index}">${c.label}</button>`).join('')}
          </div>
        </div>
        <div class="field">
          <span class="field-label">Energía del PC</span>
          <div class="consumer-grid">
            ${Object.entries(POWER_MODE_LABELS).map(([mode, label]) => `
              <button class="consumer-chip ${isMacro && isPowerMacro(action.keycode) && macroById(action.keycode).actions[0].mode === mode ? 'on' : ''}"
                      data-act="set-power" data-mode="${mode}">${label}</button>`).join('')}
          </div>
          <p class="setting-desc">
            Esta acción la ejecuta el PC, no el teclado, así que OrbyGUI tiene que estar abierto
            —vale con el icono de la bandeja— para que funcione.
          </p>
        </div>` : ''}

      ${tab === 'pages' ? renderPageActions(action) : ''}
    </div>`;
}

// --- Pestaña "Grabar": capturar una operación y repetirla -------------------
// A diferencia de una secuencia, aquí no se monta nada paso a paso: el proceso
// principal engancha el ratón y el teclado del PC (electron/recorder.js) y
// guarda lo que el usuario haga, con sus tiempos. La tecla hace de interruptor:
// pulsar empieza a grabar, pulsar otra vez para y guarda, y a partir de ahí
// cada pulsación reproduce lo grabado.

const RECORD_MODES = [
  { id: 'once', label: 'Una vez',           desc: 'Reproduce la operación de principio a fin y para.' },
  { id: 'loop', label: 'En bucle',          desc: 'Repite sin parar hasta que vuelvas a pulsar la tecla.' },
  { id: 'hold', label: 'Mientras se pulsa', desc: 'Repite mientras mantengas la tecla, y para al soltarla.' },
];

// Multiplicador de velocidad al reproducir. x3 por defecto: para repetir algo
// varias veces, reproducirlo a la velocidad original de grabación sobra.
const RECORD_SPEEDS = [1, 2, 3, 5];
const RECORD_SPEED_DEFAULT = 3;

// En qué punto está la grabación, según lo que avise el proceso principal.
let recordPhase = { id: null, phase: 'idle' };

// Lo que la pantalla de la tecla enseñaba antes de ponerle el aviso, para poder
// devolvérselo. `bytes` a null = ese hueco no tenía icono propio.
let screenBackup = null;
let restoreTimer = null;

// Texto que enseña la pantalla de la tecla en cada situación. La tecla se
// explica sola: qué hace ahora mismo y qué hace con SUPER.
const RECORD_ICON_EMPTY = 'RECORD';
const RECORD_ICON_READY = 'PLAY';
const RECORD_ICON_RESET = 'RESET';

// La macro de grabación de un hueco. La tecla de borrado (capa SUPER) apunta a
// la suya con `target`, así que desde ella también se llega a la grabación.
function recordMacro(id) {
  const m = id === null ? null : macroById(id);
  if (m?.kind === 'recording') return m;
  if (m?.kind === 'recording-reset') return macroById(m.target) || null;
  return null;
}

function isResetMacro(id) {
  return macroById(id)?.kind === 'recording-reset';
}

// ¿Tiene esta grabación su tecla de borrado montada en la otra capa?
function hasResetKey(recId) {
  const pos = keyPosForMacro(recId);
  if (!pos) return false;
  const other = pos.layer === 'normal' ? 'super' : 'normal';
  const pair = state.profiles[pos.profile]?.keys[keymapSlot(pos.key, other)];
  return pair?.modifier === MACRO_MODIFIER && macroById(pair.keycode)?.target === recId;
}

// Grabación o su tecla de borrado: las dos pertenecen a la pestaña "Grabar".
function isRecordOrResetMacro(id) {
  return isRecordingMacro(id) || isResetMacro(id);
}

function recordDuration(m) {
  const events = m?.events || [];
  return events.length ? events[events.length - 1].t : 0;
}

// Dónde está la tecla que dispara una macro: perfil, número de tecla y capa.
// Hace falta para pintar su pantalla y la de su pareja en la otra capa.
function keyPosForMacro(id) {
  for (let p = 0; p < state.profiles.length; p++) {
    const prof = state.profiles[p];
    for (const layer of ['normal', 'super']) {
      for (let i = 0; i < 12; i++) {
        const a = prof.keys[keymapSlot(i, layer)];
        if (a?.modifier === MACRO_MODIFIER && a.keycode === id && labelSlot(i, layer) >= 0) {
          return { profile: p, key: i, layer };
        }
      }
    }
  }
  return null;
}

function keyScreenForMacro(id) {
  const pos = keyPosForMacro(id);
  return pos ? { profile: pos.profile, slot: labelSlot(pos.key, pos.layer) } : null;
}

// Escribe un icono definitivo (a diferencia de los avisos pasajeros de
// showKeyStatus): va a la caché y al mapa de iconos del perfil, y cuenta como
// cambio pendiente para que el guardado automático lo lleve a la Flash.
async function writeKeyIcon(profileIdx, lslot, text) {
  if (lslot < 0) return;
  const bytes = statusFrame(text);
  await queueIconWrite(async () => {
    try {
      await device.uploadOled(profileIdx, lslot, bytes);
      cache.set(profileIdx, lslot, Uint8Array.from(bytes));
      const prof = state.profiles[profileIdx];
      if (prof) prof.oledMask |= (1 << lslot);
      markDirty();
    } catch {
      toast('No se pudo escribir el icono de la tecla', 'error');
    }
  });
}

// Deja las dos pantallas de la pareja como toca: RECORD o PLAY en la capa
// normal según haya algo grabado, y RESET en la de SUPER si esa tecla lleva el
// borrado. Se llama cada vez que la grabación cambia de estado.
async function refreshRecordIcons(recId) {
  const m = macroById(recId);
  const pos = keyPosForMacro(recId);
  if (!m || !pos) return;

  await writeKeyIcon(pos.profile, labelSlot(pos.key, pos.layer),
                     m.events?.length ? RECORD_ICON_READY : RECORD_ICON_EMPTY);

  // La pareja de borrado solo existe si se creó (ver attachResetKey): con la
  // grabación puesta en la capa SUPER no hay otra capa donde ponerla.
  const other = pos.layer === 'normal' ? 'super' : 'normal';
  const prof = state.profiles[pos.profile];
  const pair = prof?.keys[keymapSlot(pos.key, other)];
  if (pair?.modifier === MACRO_MODIFIER && macroById(pair.keycode)?.target === recId) {
    await writeKeyIcon(pos.profile, labelSlot(pos.key, other), RECORD_ICON_RESET);
  }
}

// Monta la tecla de borrado en la capa SUPER de la misma tecla: pulsar
// SUPER + esa tecla tira lo grabado y la deja lista para grabar otra vez.
// Solo tiene sentido creando la grabación en la capa normal; si se crea ya en
// SUPER no hay otra capa libre donde ponerla y se avisa en el editor.
async function attachResetKey(recId) {
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null || view.layer !== 'normal') return;

  const slot = keymapSlot(i, 'super');
  const resetId = nextMacroId();
  const reset = ensureMacro(resetId);
  reset.kind = 'recording-reset';
  reset.target = recId;
  reset.actions = [];   // sin pasos: el teclado avisa por CDC en vez de tocarla

  const m = macroById(recId);
  if (m) m.resetId = resetId;
  savePCMacros(resetId);

  prof.keys[slot] = { modifier: MACRO_MODIFIER, keycode: resetId };
  try {
    await device.setKeymap(view.editingProfile, slot, MACRO_MODIFIER, resetId);
    markDirty();
    await variants.reassertSlot('keys', slot);
  } catch {
    toast('No se pudo asignar la tecla de borrado en SUPER', 'error');
  }
}

// Al dejar de ser una grabación (se cambia de pestaña, o se quita la tecla) hay
// que llevarse por delante también su tecla de borrado: si no, la capa SUPER
// se queda con un RESET que ya no borra nada.
async function detachResetKey(recId) {
  const m = macroById(recId);
  const pos = keyPosForMacro(recId);
  if (!m || !pos) return;

  const other = pos.layer === 'normal' ? 'super' : 'normal';
  const slot = keymapSlot(pos.key, other);
  const prof = state.profiles[pos.profile];
  const pair = prof?.keys[slot];
  if (pair?.modifier !== MACRO_MODIFIER || macroById(pair.keycode)?.target !== recId) return;

  pcMacros = pcMacros.filter((x) => x.id !== pair.keycode);
  savePCMacros(pair.keycode);

  prof.keys[slot] = { modifier: 0, keycode: 0 };
  try {
    await device.setKeymap(pos.profile, slot, 0, 0);
    await device.clearOled(pos.profile, labelSlot(pos.key, other));
    cache.set(pos.profile, labelSlot(pos.key, other), null);
    prof.oledMask &= ~(1 << labelSlot(pos.key, other));
    markDirty();
  } catch {
    toast('No se pudo quitar la tecla de borrado de SUPER', 'error');
  }
}

// Un cartel a pantalla completa con el texto centrado y el marco de siempre.
function statusFrame(text) {
  const source = fb.makeTextSource(text, { fontSize: 20, bold: true, font: 'Segoe UI Black' });
  const scale = fb.fitScale(source);
  const size = fb.measureSource(source);
  const raster = fb.rasterizeLayer(source, {
    x: Math.round((fb.OLED_W - size.width * scale) / 2),
    y: Math.round((fb.OLED_H - size.height * scale) / 2),
    scale, threshold: 128, blur: 0, dither: false, invert: 'none',
  });
  fb.drawFrame(raster);
  return raster;
}

// Subir un icono son cuatro OLED_CHUNK seguidos contra el mismo hueco. Si dos
// escrituras se solapan, sus trozos se entrelazan y el teclado acaba pintando
// una mezcla de los dos dibujos: es lo que pasaba al reiniciar una grabación
// mientras se estaba reproduciendo (llega "reset" y, en mitad de su escritura,
// el "idle" de la reproducción cortada empieza otra). Se encolan para que cada
// una termine antes de que empiece la siguiente.
let iconQueue = Promise.resolve();

function queueIconWrite(run) {
  const next = iconQueue.then(run, run);
  iconQueue = next.catch(() => {});
  return next;
}

// El aviso se manda a la RAM del teclado y NO se marca como cambio pendiente:
// así el guardado automático no se lleva un "REC" a la Flash y el icono de
// verdad sigue siendo el que estaba.
async function showKeyStatus(id, text) {
  clearTimeout(restoreTimer);
  if (!state.connected) return;
  const target = keyScreenForMacro(id);
  if (!target) return;

  if (!screenBackup) {
    const existing = cache.get(target.profile, target.slot);
    screenBackup = { ...target, bytes: existing ? Uint8Array.from(existing) : null };
  }
  const bytes = statusFrame(text);
  await queueIconWrite(async () => {
    try {
      await device.uploadOled(target.profile, target.slot, bytes);
    } catch { /* si no llega, el aviso se queda solo en la app */ }
  });
}

async function restoreKeyStatus() {
  clearTimeout(restoreTimer);
  const saved = screenBackup;
  screenBackup = null;
  if (!saved || !state.connected) return;
  await queueIconWrite(async () => {
    try {
      if (saved.bytes) await device.uploadOled(saved.profile, saved.slot, saved.bytes);
      else await device.clearOled(saved.profile, saved.slot);
    } catch { /* al reconectar se relee todo de todas formas */ }
  });
}

// Avisos del proceso principal: grabando / guardada / reproduciendo / parada.
async function onRecorderState({ id, phase }) {
  recordPhase = { id, phase };

  if (phase === 'recording') {
    await showKeyStatus(id, 'REC');
  } else if (phase === 'playing') {
    await showKeyStatus(id, 'RUN');
  } else if (phase === 'saved' || phase === 'empty') {
    // El proceso principal ha escrito los eventos en la configuración: hay que
    // releerla antes de volver a guardarla desde aquí, o se perdería.
    await loadPCMacros();
    await showKeyStatus(id, phase === 'saved' ? 'OK' : 'VACIO');
    restoreTimer = setTimeout(async () => {
      recordPhase = { id: null, phase: 'idle' };
      // No se devuelve el icono anterior: ahora hay algo grabado, así que la
      // tecla pasa a poner PLAY. De eso se encarga refreshRecordIcons.
      screenBackup = null;
      await refreshRecordIcons(id);
      if (isActiveView()) render();
    }, 1200);
  } else if (phase === 'reset') {
    // Se ha pulsado SUPER + la tecla: lo grabado se ha ido y vuelve a RECORD.
    await loadPCMacros();
    recordPhase = { id: null, phase: 'idle' };
    screenBackup = null;
    await refreshRecordIcons(id);
    toast('Grabación borrada: la tecla vuelve a estar lista para grabar');
  } else {
    await restoreKeyStatus();
  }

  if (isActiveView()) render();
}

function setRecordMode(mode) {
  const m = recordMacro(currentMacroId());
  if (!m) return;
  m.mode = mode;
  savePCMacros(m.id);
  render();
}

function setRecordSpeed(speed) {
  const m = recordMacro(currentMacroId());
  if (!m) return;
  m.speed = Number(speed) || RECORD_SPEED_DEFAULT;
  savePCMacros(m.id);
  render();
}

async function clearRecording() {
  const m = recordMacro(currentMacroId());
  if (!m || !m.events?.length) return;
  if (!confirm('Se borrará la operación grabada en esta tecla.\n\n¿Continuar?')) return;
  m.events = [];
  savePCMacros(m.id);
  await refreshRecordIcons(m.id);  // vuelve a RECORD
  render();
}

async function toggleRecording() {
  const m = recordMacro(currentMacroId());
  if (!m) return;

  // Con algo grabado, la tecla reproduce en vez de grabar (es lo que se espera
  // de ella el 99% de las veces), así que "grabar de nuevo" tiene que vaciarla
  // antes: es lo mismo que hace SUPER + la tecla. Los eventos los lee el
  // proceso principal de la configuración del PC, así que hay que esperar a que
  // quede escrita o leería todavía los viejos y se pondría a reproducir.
  if (recordPhase.phase !== 'recording' && m.events?.length) {
    if (!confirm('Se sustituirá la operación grabada en esta tecla.\n\n¿Grabar de nuevo?')) return;
    m.events = [];
    await window.orby.setConfig({ macros: pcMacros });
    await refreshRecordIcons(m.id);
    render();
  }

  window.orby.recorder.toggle(m.id);
}

function renderRecordTab(macroId) {
  const m = recordMacro(macroId);
  if (!m) {
    return '<p class="setting-desc">Preparando la grabación de esta tecla…</p>';
  }

  // Este hueco es la pareja de borrado, no la grabación en sí.
  if (isResetMacro(macroId)) {
    const pos = keyPosForMacro(m.id);
    const events = m.events || [];
    return `
      <div class="field">
        <span class="field-label">Borrar la grabación</span>
        <div class="rec-state ${events.length ? 'ok' : 'off'}">
          ${pos ? `Esta tecla borra lo grabado en la tecla ${pos.key + 1} de la capa normal.` : 'Tecla de borrado.'}
          ${events.length ? ` Ahora mismo hay ${events.length} eventos guardados.` : ' Ahora mismo no hay nada que borrar.'}
        </div>
      </div>
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-clear" ${events.length ? '' : 'disabled'}>
          ${icon('trash', 16)} Borrar ahora
        </button>
      </div>
      <p class="setting-desc">
        Se creó sola al convertir la otra tecla en grabación. Para quitarla, cambia esa tecla a
        cualquier otra pestaña que no sea "Grabar".
      </p>`;
  }

  const events = m.events || [];
  const phase = recordPhase.id === m.id ? recordPhase.phase : 'idle';
  const recording = phase === 'recording';
  const playing = phase === 'playing';
  const mode = m.mode || 'once';
  const speed = m.speed || RECORD_SPEED_DEFAULT;
  const seconds = (recordDuration(m) / 1000).toFixed(1);

  return `
    <div class="field">
      <span class="field-label">Operación grabada</span>
      <div class="rec-state ${recording ? 'is-rec' : (events.length ? 'ok' : 'off')}">
        ${recording
          ? 'Grabando… haz la operación y vuelve a pulsar la tecla (o el botón de abajo) para terminar.'
          : events.length
            ? `${events.length} eventos · ${seconds} s de ratón y teclado`
            : 'Todavía no hay nada grabado en esta tecla.'}
      </div>
    </div>

    <div class="inspector-actions">
      <button class="primary-btn ${recording ? 'is-capturing' : ''}" data-act="rec-toggle">
        ${icon(recording ? 'square' : 'oled', 16)}
        ${recording ? 'Terminar grabación' : (events.length ? 'Grabar de nuevo' : 'Empezar a grabar')}
      </button>
      <button class="secondary-btn danger" data-act="rec-clear" ${events.length ? '' : 'disabled'}>
        ${icon('trash', 16)} Borrar grabación
      </button>
    </div>

    <p class="setting-desc">
      Se captura lo que hagas con el ratón y el teclado del PC, con sus tiempos. Mientras grabas, la
      pantalla de la tecla pone <strong>REC</strong>; al terminar, <strong>OK</strong>. Para parar de
      grabar vale tanto la propia tecla como el botón de aquí arriba.
      ${events.length ? 'Con algo grabado, pulsar la tecla lo reproduce; para cambiarlo, usa "Grabar de nuevo".' : ''}
    </p>

    <div class="rec-state off">
      La pantalla de esta tecla pone <strong>${events.length ? RECORD_ICON_READY : RECORD_ICON_EMPTY}</strong>
      ${hasResetKey(m.id)
        ? `, y con SUPER pone <strong>${RECORD_ICON_RESET}</strong>: <strong>SUPER + esta tecla</strong>
           borra lo grabado y la deja lista para grabar otra vez.`
        : `. La tecla de borrado automática solo se monta al crear la grabación en la capa NORMAL:
           en SUPER no queda otra capa donde ponerla.`}
    </div>

    <div class="field mt-4">
      <span class="field-label">Al reproducir</span>
      <div class="chip-row">
        ${RECORD_MODES.map((r) => `
          <button class="chip ${mode === r.id ? 'on' : ''}" data-act="rec-mode" data-mode="${r.id}">${r.label}</button>`).join('')}
      </div>
      <p class="setting-desc">${RECORD_MODES.find((r) => r.id === mode)?.desc || ''}</p>
    </div>

    <div class="field mt-4">
      <span class="field-label">Velocidad de reproducción</span>
      <div class="chip-row">
        ${RECORD_SPEEDS.map((s) => `
          <button class="chip ${speed === s ? 'on' : ''}" data-act="rec-speed" data-speed="${s}">${s}x</button>`).join('')}
      </div>
      <p class="setting-desc">Por defecto x3: reproduce tres veces más rápido que como se grabó.</p>
    </div>

    ${playing ? `
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-stop">${icon('square', 16)} Parar reproducción</button>
      </div>` : ''}

    <p class="setting-desc">
      Esta acción la ejecuta siempre el PC (el teclado no puede saber dónde está el ratón), así que
      OrbyGUI tiene que estar abierto —vale con el icono de la bandeja— para que funcione.
    </p>`;
}

// Editor de una macro (secuencia ejecutada en el PC): lista de acciones con
// botón de borrar cada una, botones para añadir las de un solo paso, y un modo
// de grabación que captura pulsaciones reales del teclado del PC.
function renderSequenceEditor(macroId) {
  const macro = macroId === null ? null : macroById(macroId);
  const actions = macro?.actions || [];
  const recording = view.capturing === 'sequence';
  const capturingPos = view.capturing === 'position';

  // El buscador de apps instaladas (datalist) de cada paso "Abrir" necesita
  // la lista cargada, igual que en la pestaña App (ver ensureInstalledApps).
  if (actions.some((a) => a.type === 'open_app' && a.kind !== 'file')) ensureInstalledApps();

  const CLICK_LABELS = { left: 'Izquierdo', middle: 'Central', right: 'Derecho' };
  const lastRealIdx = actions.reduce((acc, a, i) => (a.type === 'delay' ? acc : i), -1);

  // Botones para subir/bajar un paso, deshabilitados en los extremos.
  const moveButtons = (idx, isFirst, isLast) => `
    <button class="tool-btn small" data-act="seq-move-up" data-index="${idx}" title="Subir paso" ${isFirst ? 'disabled' : ''}>
      ${icon('up', 14)}
    </button>
    <button class="tool-btn small" data-act="seq-move-down" data-index="${idx}" title="Bajar paso" ${isLast ? 'disabled' : ''}>
      ${icon('down', 14)}
    </button>`;

  // Cuántas veces se repite el gesto (mismo atajo o mismo clic seguido, ver
  // seqAddAction): no añade pasos de verdad, solo repite este al ejecutarlo.
  const countField = (a, idx) => `
    <input type="number" class="text-input compact seq-count-input" min="1" max="99"
           data-act="seq-count" data-index="${idx}" value="${a.count || 1}" title="Veces">`;

  // Hueco entre cada repetición: solo tiene sentido a partir de 2 veces (con
  // 1 no hay nada que espaciar). Si la acción no trae uno propio (secuencias
  // guardadas antes de que esto existiera), se enseña el valor por defecto.
  const gapField = (a, idx) => a.count > 1 ? `
    <input type="number" class="text-input compact seq-gap-input" min="0" max="32767" step="5"
           data-act="seq-gap-ms" data-index="${idx}" value="${a.gap ?? DEFAULT_REPEAT_GAP_MS}" title="Espera entre repeticiones (ms)">` : '';

  let stepNum = 0;
  const items = actions.map((a, idx) => {
    // La espera entre dos pasos no es un paso más: es el hueco configurable
    // que pide siempre haber entre dos pasos consecutivos.
    if (a.type === 'delay') {
      return `
        <li class="seq-item seq-gap">
          <span>${icon('reset', 13)} Espera</span>
          <span class="row-inline" style="gap:4px">
            <input type="number" class="text-input compact seq-gap-input" min="0" step="10"
                   value="${a.ms}" data-act="seq-delay-ms" data-index="${idx}">
            <span class="setting-desc">ms</span>
          </span>
        </li>`;
    }

    stepNum++;

    if (a.type === 'mouse_position') {
      return `
        <li class="seq-item">
          <span>${stepNum}. Posición de ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-x" data-index="${idx}" value="${a.x}" title="x">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-y" data-index="${idx}" value="${a.y}" title="y">
            <button class="tool-btn small" data-act="seq-recapture" data-index="${idx}" title="Recapturar con el ratón">
              ${icon('fit', 14)}
            </button>
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    // Formato antiguo, sin coordenadas propias: solo se puede borrar, no
    // editar ni recapturar (no hay paso nuevo de este tipo desde el editor).
    if (a.type === 'center_mouse') {
      return `
        <li class="seq-item">
          <span>${stepNum}. Centrar ratón</span>
          <span class="row-inline" style="gap:4px">
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    if (a.type === 'mouse_move') {
      return `
        <li class="seq-item">
          <span>${stepNum}. Mover ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dx" data-index="${idx}" value="${a.dx}" title="dx">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dy" data-index="${idx}" value="${a.dy}" title="dy">
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    if (a.type === 'mouse_click') {
      const btn = a.button || 'left';
      return `
        <li class="seq-item">
          <span>${stepNum}. Clic ${a.count > 1 ? `×${a.count}` : ''}</span>
          <span class="row-inline" style="gap:6px">
            <select class="select-input compact" data-act="seq-click-button" data-index="${idx}">
              ${Object.entries(CLICK_LABELS).map(([value, label]) =>
                `<option value="${value}" ${btn === value ? 'selected' : ''}>${label}</option>`).join('')}
            </select>
            ${countField(a, idx)}
            ${gapField(a, idx)}
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    if (a.type === 'open_app') {
      const openKind = a.kind === 'file' ? 'file' : 'app';
      return `
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${stepNum}. Abrir</span>
            <span class="row-inline" style="gap:4px">
              ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
                ${icon('trash', 14)}
              </button>
            </span>
          </div>
          <div class="row-inline mt-4" style="gap:6px">
            <button class="type-chip small ${openKind === 'app' ? 'on' : ''}" data-act="seq-open-kind" data-index="${idx}" data-kind="app">Aplicación</button>
            <button class="type-chip small ${openKind === 'file' ? 'on' : ''}" data-act="seq-open-kind" data-index="${idx}" data-kind="file">Archivo</button>
          </div>
          ${openKind === 'app' ? `
            <div class="row-inline mt-4" style="gap:6px">
              <button class="tool-btn small" data-act="seq-open-focus" data-index="${idx}" title="Usar la app en primer plano ahora mismo">
                ${icon('fit', 14)} App en foco
              </button>
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${idx}" data-kind="app" title="Examinar…">
                ${icon('upload', 14)}
              </button>
            </div>
            <input type="text" class="text-input compact mt-4" style="width:100%" list="installed-apps-list"
                   data-act="seq-open-target" data-index="${idx}" value="${escape(a.target || '')}"
                   placeholder="Escribe para buscar entre las instaladas" title="${escape(a.target || '')}">
          ` : `
            <div class="row-inline mt-4" style="gap:6px">
              <input type="text" class="text-input compact" style="flex:1;min-width:120px"
                     data-act="seq-open-target" data-index="${idx}" value="${escape(a.target || '')}"
                     placeholder="Ruta del archivo" title="${escape(a.target || '')}">
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${idx}" data-kind="file" title="Examinar…">
                ${icon('upload', 14)}
              </button>
            </div>
          `}
        </li>`;
    }

    if (a.type === 'hotkey') {
      return `
        <li class="seq-item">
          <span>${stepNum}. ${escape(describeAction(a.modifier, a.keycode))} ${a.count > 1 ? `×${a.count}` : ''}</span>
          <span class="row-inline" style="gap:4px">
            ${countField(a, idx)}
            ${gapField(a, idx)}
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    // a.type === 'key' (grabado pulsando físicamente, sin modificadores)
    return `
      <li class="seq-item">
        <span>${stepNum}. Tecla ${escape(a.code)}</span>
        <span class="row-inline" style="gap:4px">
          ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
          <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
            ${icon('trash', 14)}
          </button>
        </span>
      </li>`;
  }).join('');

  return `
    <div class="field">
      <span class="field-label">Pasos de la secuencia</span>
      ${actions.length ? `<ul class="seq-list">${items}</ul>`
                        : `<p class="setting-desc">Todavía no tiene ningún paso.</p>`}
      ${installedAppsDatalist()}

      <div class="row-inline mt-4">
        <button class="secondary-btn ${capturingPos ? 'is-capturing' : ''}" data-act="seq-add-position">
          ${icon('fit', 16)} ${capturingPos ? (capturePosEditIndex !== null ? 'Recapturando… (Esc fija)' : 'Capturando… (Esc fija)') : 'Posición de ratón'}
        </button>
        <button class="secondary-btn" data-act="seq-add-click">${icon('bolt', 16)} Clic</button>
        <button class="secondary-btn" data-act="seq-add-move">${icon('reset', 16)} Mover ratón</button>
        <button class="secondary-btn" data-act="seq-add-open">${icon('upload', 16)} Abrir app/archivo</button>
      </div>
      ${capturingPos ? `<p class="setting-desc" id="seq-live-pos">(${lastMousePos.x}, ${lastMousePos.y}) — mueve el ratón y pulsa Esc para fijarla</p>` : ''}

      <span class="field-label mt-4">Tecla (con modificadores)</span>
      <div class="mod-grid">
        ${MODIFIERS.map((m) => `
          <button class="mod-chip ${seqKeyBuilder.modifier & m.bit ? 'on' : ''}"
                  data-act="seq-key-mod" data-bit="${m.bit}">${m.label}</button>`).join('')}
      </div>
      <div class="row-inline mt-4">
        <select class="select-input" data-act="seq-key-pick" style="flex:1">
          <option value="0" ${!seqKeyBuilder.keycode ? 'selected' : ''}>— ninguna —</option>
          ${keycodeOptions(seqKeyBuilder.keycode)}
        </select>
        <button class="secondary-btn" data-act="seq-add-hotkey">${icon('plus', 16)} Añadir</button>
      </div>

      <button class="primary-btn full mt-4 ${recording ? 'is-capturing' : ''}" data-act="seq-record">
        ${icon('key', 16)} ${recording ? 'Grabando… pulsa teclas (Esc termina)' : 'Grabar secuencia de teclas'}
      </button>

      ${macro ? renderSequenceLocation(macro) : ''}

      <p class="setting-desc">
        "Grabar secuencia" solo reconoce letras, dígitos, Enter y Espacio; para el resto de
        teclas o combinaciones con modificadores usa "Tecla" arriba. Entre cada dos pasos se
        espera automáticamente lo que pongas en "Espera" (${DEFAULT_STEP_DELAY_MS} ms por
        defecto suele bastar).
      </p>

      <button class="secondary-btn full" data-act="seq-clear">${icon('trash', 16)} Quitar secuencia</button>
    </div>`;
}

// Dice si esta secuencia la toca el propio teclado (sigue funcionando aunque
// cierres la app) o si necesita el PC, y por qué: espera, tecla, clic (con
// repeticiones incluidas, ver macro_repeat_or_advance en main.cpp) y
// movimiento relativo sí se suben; posición de ratón absoluta, de momento no
// (ver TODO(homing-absoluto) junto a DEVICE_STEP_TYPE) — igual que el formato
// antiguo "centrar ratón" o tener más pasos de los que caben en el teclado.
function renderSequenceLocation(macro) {
  if (macroDeviceEligible(macro)) {
    return `<p class="setting-desc">${icon('check', 13)} Se ejecuta en el propio teclado: sigue
              funcionando aunque cierres esta app.</p>`;
  }
  const needsOpen = (macro.actions || []).some((a) => a.type === 'open_app');
  const needsPos = (macro.actions || []).some((a) => a.type === 'mouse_position' || a.type === 'center_mouse');
  const reason = needsOpen
    ? 'abre una app o un archivo, algo que solo sabe hacer el PC'
    : needsPos
    ? 'usa una posición de ratón absoluta, que de momento solo sabe reproducir el PC'
    : `tiene más de ${MACRO_MAX_STEPS_DEVICE} pasos, o alguno de un formato antiguo`;
  return `<p class="setting-desc">Se ejecuta en el PC, no en el teclado (${reason}): solo
            funciona con esta app abierta.</p>`;
}

// Las dos acciones de página. Como las multimedia, no llevan modificadores ni
// tecla: son un modo aparte.
function renderPageActions(action) {
  if (!hasPages()) return '';
  const prof = state.profiles[view.editingProfile];
  const count = prof ? pageCountOf(prof) : 1;
  const isGoto  = action.modifier === GOTO_PAGE_MODIFIER;
  const isState = action.modifier === PAGE_STATE_MODIFIER;

  let chips = '';
  for (let n = 1; n <= maxPages(); n++) {
    const beyond = n > count;
    chips += `<button class="consumer-chip ${isGoto && action.keycode === n ? 'on' : ''}"
                      data-act="set-goto-page" data-page="${n}"
                      title="${beyond ? 'Este perfil todavía no tiene esa página' : ''}">
                Página ${n}${beyond ? ' ·' : ''}
              </button>`;
  }

  return `
    <div class="field">
      <span class="field-label">Páginas</span>
      <div class="consumer-grid">${chips}</div>
      <button class="consumer-chip full ${isState ? 'on' : ''}" data-act="set-page-state">
        Estado de páginas
      </button>
      <p class="setting-desc">
        <strong>Página N</strong> salta directamente a esa página.
        <strong>Estado de páginas</strong> hace que la tecla enseñe en qué página estás
        (por ejemplo <code>P2/${count}</code>) y que al pulsarla se abra el gestor: cada
        pantalla con su número, la actual en negativo, y pulsas para saltar.
      </p>
    </div>`;
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
