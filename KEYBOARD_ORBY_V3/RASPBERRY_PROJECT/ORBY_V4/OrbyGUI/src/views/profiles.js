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
import { MODIFIERS, CONSUMER_MODIFIER, CONSUMER_ACTIONS, CONSUMER_TURN_OPTIONS, consumerPairFor,
         GOTO_PAGE_MODIFIER, PAGE_STATE_MODIFIER,
         MACRO_MODIFIER, KEY_GROUPS, describeAction, eventToAction,
         ROTARY_TYPES, ROTARY_SLOTS, isScrollType, describeRotary } from '../hid-keys.js';
import { icon } from '../icons.js';
import { toast, requireDevice } from '../ui.js';
import { goTo } from '../nav.js';
import * as cache from '../oled-cache.js';
import * as fb from '../oled-fb.js';
import * as variants from '../variants.js';
import * as plugins from '../plugins.js';
import * as platform from '../platform.js';

import { ROTARY_GROUPS, ROTARY_TYPE_OPTIONS, SCROLL_PRESETS } from './profiles/constants.js';
import { view, setRenderers, currentPageIdx, editingVariant, selectedKeyIndex, selectedRotarySlot,
         rotaryPart, currentProfile, isActiveView, currentAction, currentRotary, currentScroll,
         currentMacroId, pluginValuePick } from './profiles/view-state.js';
import { loadPCMacros, savePCMacros, macroById, ensureMacro, nextMacroId, POWER_MODE_LABELS,
         isAppMacro, isTextMacro, isPowerMacro, isPluginMacro, pluginMacroOf, describeRotaryFull,
         isRecordOrResetMacro, describeKey, keyNeedsApp, rotaryNeedsApp } from './profiles/macros-store.js';
import { applyKeymap, blankKeyScreen, keyClipboard, copyKey, pasteKey,
         applyRotary } from './profiles/keys.js';
import { recordMacro, detachResetKey, onRecorderState, setRecordMode, setRecordSpeed,
         clearRecording, toggleRecording, renderRecordTab,
         startRecordingKey } from './profiles/recorder.js';
import { seqAddAction, currentAppStep, setAppKind, currentTextStep, setPowerAction, setPluginAction,
         giroInvertido, invertirGiroPlugin, setRotaryPluginAction, setRotaryConsumerPair,
         pickAppTarget, pickAppFocus,
         renderAppTab, renderTextTab, pickOpenTarget, setSeqOpenKind, pickSeqOpenFocus,
         seqRemoveAction, seqMoveAction, seqActionAt, seqKeyBuilder, startPositionCapture,
         stopPositionCapture, applyRotaryMacro, cancelPositionPoll,
         openPluginValuePick, setPluginValuePickValue, cancelPluginValuePick,
         confirmPluginValuePick } from './profiles/macro-tabs.js';
import { renderSequenceEditor } from './profiles/sequence-editor.js';
import { getCurrentWindowInfo, keycodeOptions, escape } from './profiles/util.js';

// Subir las secuencias al teclado lo hace ahora macros-store.js, pero main.js
// se lo sigue pidiendo a esta vista: era parte de su interfaz antes del corte.
export { syncAllMacrosToDevice } from './profiles/macros-store.js';


export function init() {
  // Lo primero: los módulos del editor repintan a través de este registro (ver
  // view-state.js), y init() ya dispara cosas que pueden querer hacerlo.
  setRenderers({ render, renderKeyGrid });

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

  // Instalar, activar o quitar un complemento cambia qué acciones se pueden
  // asignar: el inspector tiene que enterarse aunque el cambio se haya hecho
  // desde Ajustes.
  plugins.onChange(() => { if (isActiveView()) render(); });

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
  } else if (act === 'rotary-consumer-pair') {
    setRotaryConsumerPair(el.dataset.pair);
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
  } else if (act === 'seq-add-text') {
    // En blanco, como el paso "Abrir": el texto se escribe en el propio paso.
    seqAddAction({ type: 'text', text: '' });
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
    // Se toca el campo, no el objeto entero: seqKeyBuilder lo declara
    // macro-tabs.js y una exportación reasignada es de solo lectura desde aquí.
    seqKeyBuilder.keycode = 0;
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
  } else if (act === 'set-plugin') {
    setPluginAction(el.dataset.plugin, el.dataset.op);
  } else if (act === 'rotary-plugin') {
    setRotaryPluginAction(el.dataset.plugin, el.dataset.op);
  } else if (act === 'plugin-value-open') {
    const mode = el.dataset.mode;
    const rot = currentRotary();
    const macroId = mode === 'rotary'
      ? (rot.type === ROTARY_TYPES.KEY && rot.modifier === MACRO_MODIFIER ? rot.keycode : null)
      : currentMacroId();
    const paso = macroId != null ? pluginMacroOf(macroId) : null;
    const current = (paso?.plugin === el.dataset.plugin && paso.op === el.dataset.op) ? paso.value : undefined;
    openPluginValuePick(el.dataset.plugin, el.dataset.op, mode, current);
  } else if (act === 'plugin-value-confirm') {
    confirmPluginValuePick();
  } else if (act === 'plugin-value-cancel') {
    cancelPluginValuePick();
  } else if (act === 'rotary-plugin-invert') {
    invertirGiroPlugin();
  } else if (act === 'rotary-plugin-tab') {
    // Entrar en un complemento sin haber elegido todavía qué controla: se
    // asigna la primera acción que encaje en este hueco, para que el mando
    // quede con algo que hacer, igual que "Secuencia" crea una macro vacía.
    const pluginId = el.dataset.plugin;
    const yaEsta = currentRotary();
    const previo = yaEsta.modifier === MACRO_MODIFIER ? pluginMacroOf(yaEsta.keycode) : null;
    if (previo?.plugin !== pluginId) {
      const clic = Boolean(rotaryPart(selectedRotarySlot())?.part.discrete);
      const primera = plugins.actionsFor(plugins.byId(pluginId), clic ? 'click' : 'turn')[0];
      if (primera) setRotaryPluginAction(pluginId, primera.op);
    }
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
  } else if (act === 'seq-text') {
    // `change` salta al salir del campo: guarda ya lo último escrito sin
    // esperar a que venza el retardo de onInput.
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.text = e.target.value; saveMacroNow(currentMacroId()); }
  } else if (act === 'text-value') {
    const step = currentTextStep();
    if (step) { step.text = e.target.value; saveMacroNow(currentMacroId()); }
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

// Escribir en un campo de texto dispara un evento por tecla, y cada guardado
// reescribe la copia de la macro en el teclado (ver savePCMacros): se espera a
// que pare de escribir en vez de mandar una ráfaga por el puerto serie. No
// repinta, que se llevaría por delante el foco y el cursor del campo.
let macroTextDebounce = null;
function saveMacroLater(id) {
  clearTimeout(macroTextDebounce);
  macroTextDebounce = setTimeout(() => savePCMacros(id), 400);
}

// Lo mismo pero ya: al salir del campo (`change`) no hay nada que esperar, y
// así la etiqueta de la tecla en la rejilla se pone al día al momento.
function saveMacroNow(id) {
  clearTimeout(macroTextDebounce);
  savePCMacros(id);
  render();
}

let labelDebounce = null;
function onInput(e) {
  const act = e.target.dataset.act;

  if (act === 'seq-text') {
    const step = seqActionAt(Number(e.target.dataset.index));
    if (step) { step.text = e.target.value; saveMacroLater(currentMacroId()); }

  } else if (act === 'text-value') {
    const step = currentTextStep();
    if (step) { step.text = e.target.value; saveMacroLater(currentMacroId()); }

  } else if (act === 'edit-name') {
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

  } else if (act === 'plugin-value-slider') {
    // Igual que el de la rueda: deslizar solo actualiza el número en pantalla,
    // la macro se escribe al pulsar "Aplicar" (ver plugin-value-confirm).
    const val = Number(e.target.value);
    setPluginValuePickValue(val);
    const readout = document.getElementById('plugin-value-readout');
    if (readout) readout.textContent = val;

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


// Pestaña que abre el inspector de tecla por defecto, según lo que ya tenga
// asignado el hueco elegido.
function tabForAction(action) {
  if (action.modifier === CONSUMER_MODIFIER) return 'media';
  if (action.modifier === GOTO_PAGE_MODIFIER || action.modifier === PAGE_STATE_MODIFIER) return 'pages';
  if (action.modifier === MACRO_MODIFIER) {
    if (isRecordOrResetMacro(action.keycode)) return 'record';
    if (isPowerMacro(action.keycode) || isPluginMacro(action.keycode)) return 'media';
    if (isTextMacro(action.keycode)) return 'text';
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
        || isPowerMacro(action.keycode) || isPluginMacro(action.keycode)
        || isTextMacro(action.keycode) || isRecordOrResetMacro(action.keycode)) {
      const id = nextMacroId();
      ensureMacro(id);
      applyKeymap(MACRO_MODIFIER, id); // ya repinta al terminar
      return;
    }
  }

  if (tab === 'text' && prevTab !== 'text') {
    const action = currentAction();
    if (action.modifier !== MACRO_MODIFIER || !isTextMacro(action.keycode)) {
      const id = nextMacroId();
      const m = ensureMacro(id);
      m.actions = [{ type: 'text', text: '' }];
      savePCMacros(id);
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
  if (view.capturing !== 'position') cancelPositionPoll();

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
      toast(`Página ${pageCountOf(p)} añadida, vacía`);
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
      <strong>+</strong> añades otra en blanco, y el teclado alterna entre ellas
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
    // Esta tecla la ejecuta el PC, no el teclado: sin OrbyGUI abierta no hace nada.
    const necesitaApp = keyNeedsApp(action);
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
        <span class="okey-action ${assigned ? 'assigned' : ''}">${escape(describeKey(action))}${
          necesitaApp
            ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esta tecla no hace nada">${icon('plug', 11)}</i>`
            : ''}</span>
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

// Un giro de complemento (brillo, color…) o de una pareja multimedia con
// arriba/abajo (Volumen, Brillo, Zoom) reparte su valor entre CW y CCW por
// construcción: no tiene sentido editarlos por separado, así que la lista los
// enseña como un único control "Giro". Lo mismo vale para desplazamiento y el
// zoom por Ctrl+rueda, que ya llegan con la misma acción en los dos huecos
// (ver isScrollType en applyRotary, keys.js). Atajo de teclado y las acciones
// multimedia sueltas (Silenciar, Reproducir/Pausa…) sí necesitan una tecla
// distinta por sentido, así que esas se quedan con sus dos filas de siempre.
function isPluginTurnAction(action) {
  return action?.type === ROTARY_TYPES.KEY && action.modifier === MACRO_MODIFIER && isPluginMacro(action.keycode);
}

function isUnifiedTurnAction(action) {
  if (!action?.type) return true;
  if (isScrollType(action.type)) return true;
  if (isPluginTurnAction(action)) return true;
  return action.type === ROTARY_TYPES.CONSUMER && Boolean(consumerPairFor(action.keycode));
}

function renderRotaryGroups() {
  const prof = currentProfile();
  const variant = editingVariant();
  const selected = selectedRotarySlot();

  const renderPart = (part, action, changed) => `
    <button class="rotary-part ${selected === part.slot ? 'selected' : ''} ${action.type ? 'assigned' : ''} ${changed ? 'is-override' : ''}"
            data-act="pick-rotary" data-slot="${part.slot}">
      <span class="rp-dir">${part.short}</span>
      <span class="rp-body">
        <em>${part.label}</em>
        <strong>${escape(describeRotaryFull(action))}${
          rotaryNeedsApp(action)
            ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${icon('plug', 11)}</i>`
            : ''}</strong>
      </span>
    </button>`;

  return ROTARY_GROUPS.map((group) => {
    const [cwPart, ccwPart, clickPart] = group.parts;
    const cwAction = variants.effectiveRotary(prof, variant, cwPart.slot, view.layer);
    const ccwAction = variants.effectiveRotary(prof, variant, ccwPart.slot, view.layer);
    const merged = isUnifiedTurnAction(cwAction) && isUnifiedTurnAction(ccwAction);

    const cwChanged = variant && variants.override(variant, 'rotary', rotarySlot(cwPart.slot, view.layer));
    const ccwChanged = variant && variants.override(variant, 'rotary', rotarySlot(ccwPart.slot, view.layer));
    const clickAction = variants.effectiveRotary(prof, variant, clickPart.slot, view.layer);
    const clickChanged = variant && variants.override(variant, 'rotary', rotarySlot(clickPart.slot, view.layer));

    const turnRows = merged
      ? renderPart({ slot: cwPart.slot, label: 'Giro', short: '⟳' }, cwAction, cwChanged)
      : renderPart(cwPart, cwAction, cwChanged) + renderPart(ccwPart, ccwAction, ccwChanged);

    return `
    <div class="rotary-group">
      <span class="rotary-group-name">${icon(group.icon, 14)} ${group.name}</span>
      ${turnRows}
      ${renderPart(clickPart, clickAction, clickChanged)}
    </div>`;
  }).join('');
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
              <span class="rp-body"><em>Hacia abajo</em><strong>${escape(describeRotaryFull(cw))}${
                rotaryNeedsApp(cw)
                  ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${icon('plug', 11)}</i>`
                  : ''}</strong></span>
            </button>
            <button class="rotary-part ${selectedRotarySlot() === ROTARY_SLOTS.WHEEL_CCW ? 'selected' : ''}"
                    data-act="pick-rotary" data-slot="${ROTARY_SLOTS.WHEEL_CCW}">
              <span class="rp-dir">↑</span>
              <span class="rp-body"><em>Hacia arriba</em><strong>${escape(describeRotaryFull(ccw))}${
                rotaryNeedsApp(ccw)
                  ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${icon('plug', 11)}</i>`
                  : ''}</strong></span>
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
  // Una pulsación no tiene sentido de giro, así que ahí un complemento solo
  // puede ofrecer órdenes sueltas; lo que se mueve por pasos es cosa del giro.
  const target = isClick ? 'click' : 'turn';
  const pluginStep = isMacro ? pluginMacroOf(action.keycode) : null;
  const isPlugin = Boolean(pluginStep);
  const activePlugin = isPlugin ? plugins.byId(pluginStep.plugin) : null;
  const pluginOps = plugins.actionsFor(activePlugin, target);
  const puedeInvertir = isPlugin && !isClick && Boolean(Number(pluginStep.value));
  const invertido = puedeInvertir && giroInvertido(base, pluginStep);
  const types = ROTARY_TYPE_OPTIONS.filter((t) => !(isClick && t.turnOnly));

  const variant = editingVariant();
  const changed = Boolean(variant && variants.override(variant, 'rotary', rotarySlot(base, view.layer)));
  const baseAction = currentProfile().rotary?.[rotarySlot(base, view.layer)] || { type: 0, modifier: 0, keycode: 0 };

  // Sin asignar todavía, ya con un complemento, con desplazamiento/zoom o con
  // una pareja multimedia (Volumen, Brillo, Zoom), el giro se edita como
  // control único (ver renderRotaryGroups); solo Atajo de teclado y las
  // acciones multimedia sueltas, que necesitan tecla distinta por sentido,
  // siguen enseñando "Giro horario"/"antihorario".
  const subLabel = !isClick && isUnifiedTurnAction(action) ? 'Giro' : found?.part.label;

  return `
    <div class="editor-inspector glass-panel">
      <div class="inspector-head">
        <h3>${escape(found?.group.name || 'Mando')}</h3>
        <span class="pill">${view.layer === 'super' ? 'Capa SUPER' : 'Capa normal'}</span>
      </div>
      <p class="setting-desc inspector-sub">${escape(subLabel || '')}</p>

      ${variant ? `
        <div class="override-note ${changed ? 'is-override' : ''}">
          <span>${changed
            ? `Cambiado en <strong>${escape(variant.name)}</strong> · el perfil base hace
               <code>${escape(describeRotaryFull(baseAction))}</code>`
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
          <button class="type-chip ${isMacro && !isPlugin ? 'on' : ''}" data-act="rotary-macro">Secuencia</button>
          ${plugins.withActionsFor(target).map((p) => `
            <button class="type-chip ${activePlugin?.id === p.id ? 'on' : ''}"
                    data-act="rotary-plugin-tab" data-plugin="${p.id}">${escape(p.name)}</button>`).join('')}
        </div>
      </div>

      ${isPlugin ? `
        <div class="field">
          <span class="field-label">Qué controla</span>
          ${pluginOps.length ? `
            <div class="consumer-grid">
              ${pluginOps.map((a) => `
                <button class="consumer-chip ${pluginStep.op === a.op ? 'on' : ''}"
                        data-act="${a.value ? 'plugin-value-open' : 'rotary-plugin'}" data-mode="rotary"
                        data-plugin="${pluginStep.plugin}"
                        data-op="${escape(a.op)}">${escape(a.label)}</button>`).join('')}
            </div>
            ${pluginValuePick.open && pluginValuePick.mode === 'rotary' && pluginValuePick.plugin === pluginStep.plugin
              ? renderPluginValuePicker(activePlugin, pluginValuePick.op) : ''}` : `
            <p class="setting-desc">
              ${activePlugin
                ? 'Este complemento no ofrece nada para este mando.'
                : `El complemento «${escape(pluginStep.plugin)}» no está instalado o está
                   desactivado: el mando no hará nada hasta que vuelva.`}
            </p>`}
          ${puedeInvertir ? `
            <button class="consumer-chip ${invertido ? 'on' : ''}" data-act="rotary-plugin-invert"
                    style="margin-top:8px">
              ${icon('reset', 14)} Invertir giro
            </button>` : ''}
          <p class="setting-desc">
            ${isClick
              ? 'Esto lo maneja el PC, así que OrbyGUI tiene que estar abierto.'
              : `Cada muesca mueve el valor un paso, y el sentido lo pone el propio giro: pon lo
                 mismo en los dos sentidos del encoder y ya sube y baja. Si el mando está montado
                 al revés, <em>Invertir giro</em> le da la vuelta a los dos sentidos a la vez.
                 Esto lo maneja el PC, así que OrbyGUI tiene que estar abierto.`}
          </p>
        </div>` : ''}

      ${action.type === ROTARY_TYPES.CONSUMER ? `
        <div class="field">
          <span class="field-label">Acción</span>
          <div class="consumer-grid">
            ${isClick
              ? CONSUMER_ACTIONS.map((c) => `
                  <button class="consumer-chip ${action.keycode === c.index ? 'on' : ''}"
                          data-act="rotary-consumer" data-index="${c.index}">${c.label}</button>`).join('')
              : CONSUMER_TURN_OPTIONS.map((c) => c.pairId
                  ? `<button class="consumer-chip ${action.keycode === c.up || action.keycode === c.down ? 'on' : ''}"
                             data-act="rotary-consumer-pair" data-pair="${c.pairId}">${c.label}</button>`
                  : `<button class="consumer-chip ${action.keycode === c.index ? 'on' : ''}"
                             data-act="rotary-consumer" data-index="${c.index}">${c.label}</button>`).join('')}
          </div>
          ${!isClick ? `
            <p class="setting-desc">
              Cada muesca sube o baja un paso, y el sentido lo pone el propio giro: no hace
              falta elegir qué va en cada lado.
            </p>` : ''}
        </div>` : ''}

      ${isMacro && !isPlugin ? renderSequenceEditor(action.keycode) : ''}

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
        <code>${escape(describeRotaryFull(action))}</code>
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
        ${[
          { id: 'shortcut', label: 'Atajo',      cap: null },
          { id: 'sequence', label: 'Secuencia',  cap: null },
          { id: 'text',     label: 'Texto',      cap: 'text' },
          { id: 'record',   label: 'Grabar',     cap: 'recorder' },
          { id: 'app',      label: 'App',        cap: 'openApp' },
          { id: 'media',    label: 'Multimedia', cap: null },
        ].map(({ id, label, cap }) => {
          // Una pestaña que aquí no puede funcionar se deja a la vista pero apagada:
          // esconderla dejaría al usuario preguntándose dónde está la opción que sí
          // tenía en el PC, y "abrir una app" no es algo que un navegador vaya a poder
          // hacer nunca.
          const apagada = cap && !platform.can(cap);
          return `<button class="inspector-tab ${tab === id ? 'active' : ''} ${apagada ? 'unsupported' : ''}"
                          ${apagada ? 'disabled title="Necesita OrbyGUI de escritorio"' : ''}
                          data-act="set-tab" data-tab="${id}">${label}</button>`;
        }).join('')}
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

      ${tab === 'text' ? renderTextTab(isMacro ? action.keycode : null) : ''}

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
        </div>
        ${renderPluginKeyActions(isMacro ? action.keycode : null)}` : ''}

      ${tab === 'pages' ? renderPageActions(action) : ''}
    </div>`;
}

// Un bloque por complemento instalado dentro de la pestaña Multimedia, con sus
// acciones asignables a una tecla. Sin complementos no pinta nada: la
// instalación por defecto de OrbyGUI no trae ninguno, y una sección vacía solo
// haría preguntarse qué falta.
function renderPluginKeyActions(macroId) {
  const paso = macroId === null ? null : pluginMacroOf(macroId);
  const pick = pluginValuePick;

  return plugins.withActionsFor('key').map((p) => `
    <div class="field">
      <span class="field-label">${escape(p.name)}</span>
      <div class="consumer-grid">
        ${plugins.actionsFor(p, 'key').map((a) => `
          <button class="consumer-chip ${paso?.plugin === p.id && paso.op === a.op ? 'on' : ''}"
                  data-act="${a.value ? 'plugin-value-open' : 'set-plugin'}" data-mode="key"
                  data-plugin="${p.id}" data-op="${escape(a.op)}">
            ${escape(a.label)}
          </button>`).join('')}
      </div>
      ${pick.open && pick.mode === 'key' && pick.plugin === p.id
        ? renderPluginValuePicker(p, pick.op) : ''}
      <p class="setting-desc">
        ${escape(p.description || 'Lo ejecuta el PC, así que OrbyGUI tiene que estar abierto.')}
        Lo que necesite sentido de giro (subir, bajar) se asigna a un mando, no a una tecla.
      </p>
    </div>`).join('') + renderPluginViewActions(macroId);
}

// Selector deslizante para una acción de valor exacto ("Fijar brillo"...): se
// pinta en el sitio del botón que lo abrió (ver openPluginValuePick) y solo al
// confirmar se escribe la macro.
function renderPluginValuePicker(plugin, op) {
  const { action: spec } = plugins.findAction(plugin.id, op);
  if (!spec?.value) return '';
  const { min, max, step } = spec.value;
  const v = pluginValuePick.value;

  return `
    <div class="plugin-value-pick">
      <div class="field">
        <span class="field-label">${escape(spec.label)} <b id="plugin-value-readout">${v}</b></span>
        <input type="range" class="premium-slider" data-act="plugin-value-slider"
               min="${min}" max="${max}" step="${step}" value="${v}">
      </div>
      <div class="row-inline" style="gap:6px">
        <button class="primary-btn" data-act="plugin-value-confirm">${icon('check', 16)} Aplicar</button>
        <button class="secondary-btn" data-act="plugin-value-cancel">${icon('close', 16)} Cancelar</button>
      </div>
    </div>`;
}

// Bloque de "visores": teclas que no hacen nada al pulsarlas, solo enseñan un
// valor en su propia pantalla (ver src/live-oled.js). Solo tiene sentido en
// una tecla con pantalla propia (KEY_TO_SCREEN), no en el clic de un mando.
function renderPluginViewActions(macroId) {
  const paso = macroId === null ? null : pluginMacroOf(macroId);
  const conPantalla = KEY_TO_SCREEN[selectedKeyIndex()] > 0;
  if (!conPantalla) return '';

  const bloques = plugins.withViews().map((p) => `
    <div class="field">
      <span class="field-label">${escape(p.name)} · pantalla</span>
      <div class="consumer-grid">
        ${plugins.viewsFor(p).map((v) => `
          <button class="consumer-chip ${paso?.plugin === p.id && paso.op === v.op ? 'on' : ''}"
                  data-act="set-plugin" data-plugin="${p.id}" data-op="${escape(v.op)}">
            ${escape(v.label)}
          </button>`).join('')}
      </div>
    </div>`).join('');
  if (!bloques) return '';

  return `${bloques}
    <p class="setting-desc">
      La tecla deja de hacer nada al pulsarla: solo enseña el valor en su pantalla, y se
      actualiza solo cada par de segundos mientras esta página esté puesta en el teclado.
    </p>`;
}


// Los saltos de página no se describen como un atajo normal ni ocupan una
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

