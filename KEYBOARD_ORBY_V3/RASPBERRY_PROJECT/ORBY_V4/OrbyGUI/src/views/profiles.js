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
         rotarySlot, layerIndex, scrollFor, KEY_TO_SCREEN, KEY_SUPER, KEY_MENU, keyRole, isMenuEncoderSlot,
         hasPages, maxPages, pageCountOf, selectPage, addPage, removePage } from '../store.js';
import { MODIFIERS, CONSUMER_MODIFIER, CONSUMER_ACTIONS, CONSUMER_TURN_OPTIONS, consumerPairFor,
         GOTO_PAGE_MODIFIER, PAGE_STATE_MODIFIER,
         MACRO_MODIFIER, KEY_GROUPS, describeAction, eventToAction,
         ROTARY_TYPES, ROTARY_SLOTS, ROTARY_MENUS, isScrollType, isTwoWayType, describeRotary } from '../hid-keys.js';
import { icon } from '../icons.js';
import { toast, requireDevice, confirmar } from '../ui.js';
import { goTo } from '../nav.js';
import * as cache from '../oled-cache.js';
import * as fb from '../oled-fb.js';
import * as variants from '../variants.js';
import * as plugins from '../plugins.js';
import * as platform from '../platform.js';
import * as compat from '../compat.js';
import { LIBRARY, installProfile } from '../profile-library.js';
import { addRulesFor } from './auto.js';

import { ROTARY_GROUPS, WHEEL_GROUP, ROTARY_TYPE_OPTIONS, SCROLL_PRESETS } from './profiles/constants.js';
import { view, setRenderers, currentPageIdx, editingVariant, selectedKeyIndex, selectedRotarySlot,
         rotaryPart, currentProfile, isActiveView, inPanel, currentAction, currentRotary, currentScroll,
         currentMacroId, pluginValuePick } from './profiles/view-state.js';
import { loadPCMacros, savePCMacros, macroById, ensureMacro, nextMacroId, POWER_MODE_LABELS,
         isAppMacro, isTextMacro, isPowerMacro, isPluginMacro, pluginMacroOf, describeRotaryFull,
         isRecordOrResetMacro, describeKey, keyNeedsApp, rotaryNeedsApp } from './profiles/macros-store.js';
import { applyKeymap, blankKeyScreen, keyClipboard, copyKey, pasteKey,
         applyRotary } from './profiles/keys.js';
import { armDrag, startSortDrag } from './profiles/drag-sort.js';
import * as undo from './profiles/undo.js';
import { recordMacro, detachResetKey, onRecorderState, setRecordMode, setRecordSpeed,
         clearRecording, toggleRecording, renderRecordTab,
         startRecordingKey } from './profiles/recorder.js';
import { seqAddAction, currentAppStep, setAppKind, currentTextStep, setPowerAction, setPluginAction,
         giroInvertido, consumerGiroInvertido, invertirGiroPar, setRotaryPluginAction, setRotaryConsumerPair,
         pickAppTarget, pickAppFocus,
         renderAppTab, renderTextTab, pickOpenTarget, setSeqOpenKind, pickSeqOpenFocus,
         seqRemoveAction, seqMoveStep, seqSetDelayAfter, setSeqInsertAt, seqActionAt, seqKeyBuilder, startPositionCapture,
         stopPositionCapture, applyRotaryMacro, cancelPositionPoll,
         openPluginValuePick, setPluginValuePickValue, cancelPluginValuePick,
         confirmPluginValuePick } from './profiles/macro-tabs.js';
import { renderSequenceEditor } from './profiles/sequence-editor.js';
import { getCurrentWindowInfo, keycodeOptions, escape } from './profiles/util.js';
import { keyScreen as screenOfKey, profileScreen, paintProfileScreens } from '../key-screen.js';

// Subir las secuencias al teclado lo hace ahora macros-store.js, pero main.js
// se lo sigue pidiendo a esta vista: era parte de su interfaz antes del corte.
export { syncAllMacrosToDevice, syncMacroStore, pruneUnusedMacros } from './profiles/macros-store.js';


export function init() {
  // Lo primero: los módulos del editor repintan a través de este registro (ver
  // view-state.js), y init() ya dispara cosas que pueden querer hacerlo.
  setRenderers({ render, renderKeyGrid });

  wire(document.getElementById('profiles-body'));

  // Pulsar fuera de la lista y de la ficha (el fondo de la vista o el hueco
  // bajo la ficha) vuelve al Inicio, como tocar fuera de una hoja. Se mira el
  // primer elemento de la ruta y no closest(): la vista se repinta con muchos
  // clics y el elemento pulsado puede no estar ya en el documento.
  document.getElementById('view-profiles').addEventListener('click', (e) => {
    const hit = e.composedPath()[0];
    if (hit?.id === 'view-profiles' || hit?.id === 'profiles-body'
        || hit?.classList?.contains('pm') || hit?.classList?.contains('pm-detail')) {
      goTo('view-dashboard');
    }
  });
  window.addEventListener('keydown', onCapture, true);

  // Ctrl+Z deshace, como en cualquier editor. Dentro de un campo de texto no:
  // ahí es el deshacer del propio campo. Capturando un atajo tampoco: Ctrl+Z
  // es justo lo que se quiere asignar.
  window.addEventListener('keydown', (e) => {
    if (!(e.ctrlKey || e.metaKey) || e.shiftKey || e.altKey || e.key.toLowerCase() !== 'z') return;
    if (!isActiveView() || view.capturing) return;
    if (e.target.closest?.('input, textarea, select, [contenteditable="true"]')) return;
    e.preventDefault();
    undo.undo();
  });
  // El botón se pone al día solo, sin repintar el panel: un repintado a media
  // escritura de una etiqueta le quitaría el foco al campo.
  undo.onChange(paintUndoButtons);

  // Las miniaturas llegan por el puerto serie con retraso. En el panel las
  // repinta la réplica; en la vista de perfiles basta con pintar los <canvas>
  // que ya hay, salvo que un perfil estrene icono: entonces se dibujó con la
  // baldosa decorativa y no hay <canvas> donde pintarlo, así que se rehace.
  // Solo cuando cambia esa cuenta: repintar en cada bitmap que entra sería
  // rehacerla veinte veces seguidas al conectar.
  cache.onChange(() => {
    if (!isActiveView()) return;
    if (inPanel()) { renderKeyGrid(); return; }
    const body = document.getElementById('profiles-body');
    const pintados = body.querySelectorAll('.pm-row-icon.is-oled').length;
    const esperados = state.profiles.filter((p, i) => profileScreen(p, i)).length;
    if (pintados !== esperados) render();
    else paintProfileScreens(body, state.profiles);
  });

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
    // El panel del Inicio edita siempre el perfil que tiene puesto el teclado,
    // que es el que enseña la réplica de al lado: si cambia, lo sigue.
    if (view.panel && view.editingProfile !== state.activeProfileIdx) {
      view.editingProfile = state.activeProfileIdx;
      view.variantId = null;
    }
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

// Los mismos manejadores para la vista y para el panel del Inicio: los dos
// pintan los mismos controles con los mismos data-act.
function wire(host) {
  host.addEventListener('click', onClick);
  host.addEventListener('change', onChange);
  host.addEventListener('input', onInput);
  host.addEventListener('pointerdown', onPointerDown);
  // Enter en el campo de aplicaciones añade, para poder encadenar varias.
  host.addEventListener('keydown', (e) => {
    if (e.target.id === 'variant-new-match' && e.key === 'Enter') {
      e.preventDefault();
      addMatchFromField();
    }
  });
}

function selectTarget({ key = null, rotary = null }) {
  view.capturing = false;
  // Otra pieza, otra acción: se vuelve a su pantalla principal.
  view.panelNav = null;
  view.navDir = null;
  if (key !== null) {
    view.selected = { kind: 'key', index: key };
    view.tab = tabForAction(currentAction());
  } else if (rotary !== null) {
    view.selected = { kind: 'rotary', slot: rotary };
  }
}

// Entrada desde el botón de perfiles de la barra: se abre por el perfil que
// tiene puesto el teclado.
export function openTarget({ profile = state.activeProfileIdx, section = null } = {}) {
  view.editingProfile = profile;
  view.variantId = null;
  view.libraryOpen = false;
  render();
  // `section` es a dónde llevar la vista: el botón "Páginas" del panel cae
  // en esa parte de la ficha, y se resalta un instante para que se vea dónde.
  const target = section === 'pages' && document.getElementById('pm-pages');
  if (target) {
    target.scrollIntoView({ block: 'start', behavior: 'smooth' });
    const list = document.getElementById('pm-pages-list');
    list?.classList.remove('pm-flash');
    void list?.offsetWidth;
    list?.classList.add('pm-flash');
  }
}

// "Configurar teclas, rueda y mandos": se editan en el Inicio, sobre la
// réplica, que enseña siempre el perfil del teclado. Si no es este, se pone
// antes.
async function editKeysOf(idx) {
  if (idx !== state.activeProfileIdx) {
    if (!requireDevice()) return;
    try {
      await device.setProfile(idx);
      state.activeProfileIdx = idx;
      notify();
    } catch {
      toast('El teclado no confirmó el cambio de perfil', 'error');
      return;
    }
  }
  goTo('view-dashboard', { zoom: 'keys' });
}

// ---- Panel lateral del Inicio ----------------------------------------------
// El Inicio amplía una zona de la réplica y pide aquí el inspector de lo que se
// elija en ella. Se edita siempre el perfil activo y su página: es lo que
// enseñan las pantallas de la réplica de al lado.
const wired = new WeakSet();
let onPanelRender = null;

export function openPanel(host, zone, target = {}, { onRender = null } = {}) {
  if (!wired.has(host)) { wire(host); wired.add(host); }
  if (view.panel !== host || view.zone !== zone) host.innerHTML = '';
  view.panel = host;
  view.zone = zone;
  onPanelRender = onRender;
  view.editingProfile = state.activeProfileIdx;
  view.variantId = null;
  selectTarget(target);
  render();
}

export function selectInPanel(target) {
  if (!view.panel) return;
  selectTarget(target);
  render();
}

export function closePanel() {
  // Sin repintar: el panel se va, y una captura a medias se deja sin más.
  cancelPositionPoll();
  view.capturing = false;
  // Se vuelve a abrir en la capa normal, que es la que enseña el teclado.
  view.layer = 'normal';
  view.panel = null;
  view.zone = null;
  onPanelRender = null;
}

export function panelSelection() {
  return view.selected;
}

export function panelLayer() {
  return view.layer;
}

// La variación que se edita en el panel, o null con el perfil base.
export function panelVariant() {
  return view.panel ? editingVariant() : null;
}

// Esc dentro de un tipo de acción vuelve a la tecla, como el gesto de atrás
// de iOS; solo en la pantalla de la tecla cierra el zoom. Devuelve si lo ha
// gastado aquí.
export function panelBack() {
  if (!inPanel() || !view.panelNav) return false;
  navBack();
  return true;
}

function navBack() {
  view.panelNav = null;
  view.navDir = 'back';
  view.capturing = false;
  render();
}

// Los tipos de acción de una tecla, en el orden en que se ofrecen. Cada uno
// abre la pestaña del inspector del mismo nombre (`tab`), que es quien sabe
// crear la macro que haga falta al entrar (ver setTab). `cap` es la función de
// la plataforma que necesita: sin ella el tipo se ve, apagado, para que no
// parezca que ha desaparecido.
const KEY_KINDS = [
  { id: 'shortcut', label: 'Atajo de teclado', icon: 'keyboard', tint: 'blue' },
  { id: 'media',    label: 'Multimedia y PC',  icon: 'music',    tint: 'pink' },
  { id: 'app',      label: 'Abrir app',        icon: 'app',      tint: 'green',  cap: 'openApp' },
  { id: 'text',     label: 'Escribir texto',   icon: 'text',     tint: 'orange', cap: 'text' },
  { id: 'sequence', label: 'Secuencia',        icon: 'steps',    tint: 'indigo' },
  { id: 'record',   label: 'Grabar',           icon: 'record',   tint: 'red',    cap: 'recorder' },
  { id: 'pages',    label: 'Ir a página',      icon: 'pages',    tint: 'cyan',   pages: true },
];

function kindOf(action) {
  return action.modifier || action.keycode ? tabForAction(action) : null;
}

// Panel de una tecla. Dos niveles, como Ajustes de iOS: la tecla (cómo se ve
// en grande, qué hace y los tipos de acción) y, al entrar en un tipo, su
// editor con "‹ Tecla N" para volver.
function renderKeyPanel() {
  const i = selectedKeyIndex();
  if (i === null) return '';
  return view.panelNav ? renderKeyKind(i, view.panelNav) : renderKeyRoot(i);
}

// Estado del botón de deshacer: apagado sin nada que deshacer, y con el nombre
// de lo que se va a deshacer en el título (el botón es solo un icono).
function undoAttrs() {
  return undo.canUndo()
    ? `title="Deshacer: ${escape(undo.lastLabel())} (Ctrl+Z)"`
    : 'disabled title="Nada que deshacer"';
}

function paintUndoButtons() {
  const can = undo.canUndo();
  document.querySelectorAll('[data-act="undo"]').forEach((b) => {
    b.disabled = !can;
    b.title = can ? `Deshacer: ${undo.lastLabel()} (Ctrl+Z)` : 'Nada que deshacer';
  });
}

// La pantalla de la tecla tal como la dibuja el teclado (key-screen.js): el
// icono subido, el del catálogo con su nombre o la etiqueta en su fuente. El
// <canvas> se pinta después de insertar el HTML (paintKeyScreens).
function keyScreen(i, lslot, cls) {
  if (lslot < 0) {
    const role = keyRole(i);
    return `<span class="${cls} is-role">${role === 'super' ? 'SUPER' : role === 'menu' ? 'MENÚ' : 'SIN PANTALLA'}</span>`;
  }
  return `<span class="${cls}"><canvas data-key-screen="${i}"></canvas></span>`;
}

function paintKeyScreens(root) {
  const prof = currentProfile();
  const variant = editingVariant();
  root.querySelectorAll('canvas[data-key-screen]').forEach((cv) => {
    const i = Number(cv.dataset.keyScreen);
    const scr = screenOfKey(prof, view.editingProfile, i, view.layer, currentPageIdx(), {
      action: variants.effectiveKey(prof, variant, i, view.layer),
      label: variants.effectiveLabel(prof, variant, i, view.layer),
    });
    if (scr) fb.renderToCanvas(scr, cv, { zoom: 1, grid: false });
  });
}

function renderKeyRoot(i) {
  const prof = currentProfile();
  const variant = editingVariant();
  const lslot = labelSlot(i, view.layer);
  const action = currentAction();
  const kind = kindOf(action);
  const hasIcon = lslot >= 0 && Boolean(cache.get(view.editingProfile, lslot));
  const baseAction = prof.keys[keymapSlot(i, view.layer)] || { modifier: 0, keycode: 0 };
  const changed = Boolean(variant && (variants.override(variant, 'keys', keymapSlot(i, view.layer))
                                   || variants.override(variant, 'labels', lslot)));
  const now = kind ? describeKey(action) : 'Sin asignar';
  // Papel fijo (SUPER, o la de menú mientras no se libere): no hay nada que
  // asignar. Sin pantalla a secas (la 10 liberada, en este prototipo) sí hay
  // acción, solo falta lo de cómo se ve.
  const role = keyRole(i);
  const canFreeMenu = role === 'menu' && compat.supports(state.deviceInfo, 'menuKey');

  return `
    <div class="kp-hero">
      ${keyScreen(i, lslot, 'kp-screen')}
      <div class="kp-hero-text">
        <h3>Tecla ${i + 1}</h3>
        ${role ? '' : `<span class="kp-now ${kind ? '' : 'is-empty'}">${escape(now)}</span>`}
        ${keyNeedsApp(action) ? `<span class="kp-badge">${icon('plug', 12)} La ejecuta el PC con Orvy abierta</span>` : ''}
      </div>
    </div>

    ${role ? '' : `<div class="kp-tools">
      <button class="kp-tool" data-act="copy-key">${icon('copy', 17)}<span>Copiar</span></button>
      <button class="kp-tool" data-act="paste-key" ${keyClipboard ? '' : 'disabled'}
              title="${keyClipboard ? 'Pegar en esta tecla' : 'Copia una tecla primero'}">${icon('paste', 17)}<span>Pegar</span></button>
      <button class="kp-tool danger" data-act="clear-action"
              title="Quita la acción y deja la pantalla de la tecla en negro">${icon('trash', 17)}<span>Borrar</span></button>
    </div>`}

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

    ${role ? `
      <p class="kp-note">
        Esta tecla no tiene pantalla ni acción propia:
        ${role === 'super'
          ? 'mantenida, cambia el resto del teclado a la capa SUPER.'
          : 'una pulsación corta pasa de página y mantenida abre el menú del teclado.'}
        ${canFreeMenu ? `Si llevas las páginas y el menú con un mando, puedes dejarla
          libre en <strong>Ajustes → Tecla 10</strong> y darle la acción que quieras.` : ''}
      </p>` : `
      <h4 class="kp-section">Qué hace</h4>
      <div class="kp-kinds">
        ${KEY_KINDS.filter((k) => !k.pages || hasPages()).map((k) => {
          const off = k.cap && !platform.can(k.cap);
          const on = kind === k.id;
          return `
          <button class="kp-kind ${on ? 'on' : ''}" data-act="kp-open" data-kind="${k.id}" data-tint="${k.tint}"
                  ${off ? 'disabled title="Necesita Orvy de escritorio"' : ''}>
            <span class="nav-icon">${icon(k.icon, 15)}</span>
            <span class="kp-kind-label">${k.label}</span>
            ${on ? `<span class="kp-kind-value">${escape(now)}</span>` : ''}
          </button>`;
        }).join('')}
      </div>

      ${lslot < 0 ? `
      <p class="kp-note">
        Esta tecla no tiene pantalla en este prototipo: hace lo que le pongas, pero no
        hay icono que dibujar.
      </p>` : `
      <h4 class="kp-section">Cómo se ve</h4>
      <div class="kp-list">
        <button class="nav-row" data-act="edit-icon" data-key="${i}" data-tint="violet">
          <span class="nav-icon">${icon('oled', 15)}</span>
          <span class="nav-row-text">
            <span class="nav-row-title">${hasIcon ? 'Editar el icono' : 'Dibujar un icono'}</span>
            <span class="nav-row-sub">Lo que enseña la pantalla de la tecla, píxel a píxel</span>
          </span>
          <span class="nav-row-chev">${icon('chevron', 14)}</span>
        </button>
      </div>`}`}`;
}

function renderKeyKind(i, kindId) {
  const k = KEY_KINDS.find((x) => x.id === kindId) || KEY_KINDS[0];
  const action = currentAction();
  return `
    <button class="kp-back" data-act="kp-back">${icon('back', 15)}<span>Tecla ${i + 1}</span></button>
    <div class="kp-kind-head" data-tint="${k.tint}">
      <span class="nav-icon">${icon(k.icon, 15)}</span>
      <h3>${k.label}</h3>
    </div>
    <div class="kp-kind-body">${keyTabBody(k.id, action)}</div>`;
}

// --- Interacción -----------------------------------------------------------

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;
  const act = el.dataset.act;
  if (act === 'undo') { undo.undo(); return; }
  undo.beginGesture();

  if (act === 'pick-profile') {
    view.editingProfile = Number(el.dataset.idx);
    view.variantId = null;
    view.libraryOpen = false;
    render();
  } else if (act === 'pick-variant') {
    // Pulsar la que ya está abierta la cierra, como una fila desplegable.
    view.variantId = el.dataset.id && el.dataset.id !== view.variantId ? el.dataset.id : null;
    render();
  } else if (act === 'go-pages') {
    // Desde el panel del Inicio: las páginas se gestionan en la ficha del
    // perfil, y hasta ahora había que saber que estaban allí.
    goTo('view-profiles', { profile: state.activeProfileIdx, section: 'pages' });
  } else if (act === 'pm-edit-keys') {
    editKeysOf(view.editingProfile);
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
    // En el panel la pieza es la que se ha tocado en la réplica: cambiar de
    // capa es mirar la misma tecla en la otra, no volver a elegir.
    if (!view.panel) view.selected = null;
    render();
  } else if (act === 'page') {
    choosePage(Number(el.dataset.page));
  } else if (act === 'page-add') {
    createPage();
  } else if (act === 'page-del') {
    deletePage(Number(el.dataset.page));
  } else if (act === 'kp-open') {
    // Entrar en Secuencia, Texto, App o Grabar crea y asigna su macro al vuelo
    // (setTab): sin teclado no se puede, y es mejor decirlo antes de cambiar
    // de pantalla que dejar un editor que no guarda nada. Mirar lo que ya hay
    // asignado sí se puede siempre.
    const kind = el.dataset.kind;
    if (['sequence', 'text', 'app', 'record'].includes(kind)
        && kind !== kindOf(currentAction()) && !requireDevice()) return;
    view.panelNav = kind;
    view.navDir = 'fwd';
    setTab(kind);
  } else if (act === 'kp-back') {
    navBack();
  } else if (act === 'zp-piece') {
    selectTarget({ rotary: Number(el.dataset.slot) });
    render();
  } else if (act === 'rp-open') {
    view.selected = { kind: 'rotary', slot: Number(el.dataset.slot) };
    view.capturing = false;
    view.panelNav = 'gesture';
    view.navDir = 'fwd';
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
  } else if (act === 'rotary-menu') {
    applyRotary({ type: ROTARY_TYPES.MENU, modifier: 0, keycode: Number(el.dataset.menu) });
  } else if (act === 'rotary-consumer') {
    applyRotary({ type: ROTARY_TYPES.CONSUMER, modifier: 0, keycode: Number(el.dataset.index) });
  } else if (act === 'rotary-consumer-pair') {
    setRotaryConsumerPair(el.dataset.pair);
  } else if (act === 'rotary-mod') {
    const cur = currentRotary();
    applyRotary({ type: ROTARY_TYPES.KEY, modifier: cur.modifier ^ Number(el.dataset.bit), keycode: cur.keycode });
  } else if (act === 'activate') {
    if (!requireDevice()) return;
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
  } else if (act === 'library-toggle') {
    view.libraryOpen = !view.libraryOpen;
    render();
  } else if (act === 'library-add') {
    addFromLibrary(el.dataset.id);
  } else if (act === 'profile-new') {
    createProfile(null);
  } else if (act === 'profile-dup') {
    createProfile(view.editingProfile);
  } else if (act === 'profile-del') {
    removeProfile();
  } else if (act === 'profile-icon') {
    goTo('view-oled', { profile: view.editingProfile, kind: 'profile' });
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
  } else if (act === 'seq-insert-at') {
    setSeqInsertAt(el.dataset.at === 'end' ? null : Number(el.dataset.at));
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
  } else if (act === 'rotary-invert') {
    invertirGiroPar();
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

// Arrastrar un paso de la secuencia por su asa (lo mueve drag-sort.js). Las
// teclas se arrastran en la réplica del Inicio: ver dashboard.js.
function onPointerDown(e) {
  const handle = e.target.closest('[data-drag="seq-step"]');
  if (!handle) return;
  // Sin esto el navegador empieza a seleccionar texto de las filas.
  e.preventDefault();
  const list = handle.closest('.seq-list');
  armDrag(e, {
    start: (ev) => {
      const items = [...list.querySelectorAll(':scope > .seq-item')];
      startSortDrag({
        items, fromIndex: Number(handle.dataset.step), event: ev, layout: 'list', container: list,
        onDrop: (from, to) => seqMoveStep(from, to),
      });
    },
  });
}

function onChange(e) {
  const act = e.target.dataset.act;
  undo.beginGesture();
  if (act === 'panel-variant') {
    view.variantId = e.target.value || null;
    view.panelNav = null;
    render();
  } else if (act === 'pick-keycode') {
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
  } else if (act === 'seq-wait') {
    const ms = Math.max(0, Math.round(Number(e.target.value)) || 0);
    e.target.value = ms;
    seqSetDelayAfter(Number(e.target.dataset.index), ms);
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
  undo.beginGesture();

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
        // Sin repintar la vista, que le quitaría el foco al campo: basta con
        // la fila de la lista.
        const name = document.querySelector('.pm-row.on .pm-row-name');
        if (name) name.textContent = text;
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
  if (!await confirmar({
    titulo: `¿Borrar la variación «${variant.name}»?`,
    mensaje: `Se van sus ${variants.countOverrides(variant)} cambios. El perfil base no se toca.`,
    aceptar: 'Borrar',
    peligro: true,
  })) return;

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

// Añade un perfil de la biblioteca. Son unas doscientas órdenes por el CDC y
// tarda unos segundos: el panel enseña por qué página va.
async function addFromLibrary(id) {
  if (view.busy) return;
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  const def = LIBRARY.find((p) => p.id === id);
  if (!def) return;

  view.busy = true;
  view.libraryMsg = `Añadiendo ${def.app}…`;
  render();
  try {
    const idx = await installProfile(def, (msg) => {
      view.libraryMsg = `${def.app}: ${msg}`;
      const el = document.getElementById('library-status');
      if (el) el.textContent = view.libraryMsg;
    });
    view.editingProfile = idx;
    view.selected = null;
    const rules = addRulesFor(def.processes, idx);
    toast(rules ? `${def.app} añadido; se pondrá solo al abrir el programa`
                : `${def.app} añadido`);
  } catch (err) {
    toast(`No se pudo añadir ${def.app}: ${err.message}`, 'error');
    // A medias, lo que haya en el teclado manda: que la app no enseñe otra cosa.
    try { await syncFromDevice(); } catch { /* sin teclado no hay nada que releer */ }
  } finally {
    view.busy = false;
    view.libraryMsg = null;
    render();
  }
}

function renderLibrary() {
  const full = state.profiles.length >= state.maxProfiles;
  const groups = [...new Set(LIBRARY.map((p) => p.group))];
  const installed = new Set(state.profiles.map((p) => (p.name || '').toLowerCase()));
  return `
    <div class="library-panel glass-panel">
      <div class="library-head">
        <div>
          <h3>Biblioteca de perfiles</h3>
          <p class="setting-desc">Perfiles con los atajos más usados de cada programa, una página
            por tarea y más órdenes con SUPER. Cada tecla trae su icono en las pantallas.
            Se añaden al final de la lista; luego se editan como cualquier otro.</p>
        </div>
        <button class="secondary-btn" data-act="library-toggle">${icon('close', 16)} Cerrar</button>
      </div>
      ${view.libraryMsg ? `<div class="library-status" id="library-status">${escape(view.libraryMsg)}</div>` : ''}
      ${full ? `<div class="plugin-empty">El teclado ya tiene ${state.maxProfiles} perfiles: borra alguno para
        añadir otro.</div>` : ''}
      ${groups.map((g) => `
        <div class="library-group">
          <h4>${escape(g)}</h4>
          ${LIBRARY.filter((p) => p.group === g).map((p) => `
            <div class="plugin-row">
              <span class="plugin-row-icon">${icon('profiles', 18)}</span>
              <div class="plugin-row-main">
                <span class="plugin-row-name">${escape(p.app)} <b>${escape(p.name)}</b></span>
                <span class="plugin-row-desc">${escape(p.description)}</span>
                <span class="library-pages">${p.pages.map((pg) => `<span class="pill">${escape(pg.title)}</span>`).join('')}</span>
              </div>
              <div class="plugin-row-actions">
                ${installed.has(p.name.toLowerCase()) ? '<span class="pill">Ya está</span>' : ''}
                <button class="secondary-btn" data-act="library-add" data-id="${escape(p.id)}"
                        ${full || view.busy || !state.connected ? 'disabled' : ''}>
                  ${icon('plus', 16)} Añadir
                </button>
              </div>
            </div>`).join('')}
        </div>`).join('')}
    </div>`;
}

async function removeProfile() {
  if (view.busy) return;
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (state.profiles.length <= 1) { toast('Tiene que quedar al menos un perfil', 'error'); return; }

  const prof = currentProfile();
  if (!await confirmar({
    titulo: `¿Eliminar el perfil «${prof?.name || ''}»?`,
    mensaje: 'Se van con él sus atajos, mandos e iconos.\n\n'
           + 'El cambio no será permanente hasta que pulses "Guardar en Flash".',
    aceptar: 'Eliminar',
    peligro: true,
  })) return;

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
  if (inPanel()) {
    if (view.capturing !== 'position') cancelPositionPoll();
    renderPanel();
    return;
  }
  const body = document.getElementById('profiles-body');
  if (!body) return;

  // El repintado sustituye todo el innerHTML: sin esto cada clic devolvía las
  // dos columnas arriba del todo.
  const scrollList = body.querySelector('.pm-list-scroll')?.scrollTop;
  const scrollDetail = body.querySelector('.pm-detail')?.scrollTop;

  if (!state.profiles.length) {
    body.innerHTML = `<div class="empty-panel glass-panel">
      ${icon('plug', 40)}
      <h3>Sin perfiles cargados</h3>
      <p>Conecta el Orvy una vez: a partir de ahí queda una copia en el PC y podrás
         verlos aunque no lo tengas enchufado.</p>
    </div>`;
    return;
  }

  if (view.editingProfile >= state.profiles.length) view.editingProfile = 0;
  const full = state.profiles.length >= state.maxProfiles;

  body.innerHTML = `
    <div class="pm">
      <aside class="pm-list">
        <div class="pm-list-scroll">
          ${state.profiles.map((p, i) => `
            <button class="pm-row ${i === view.editingProfile && !view.libraryOpen ? 'on' : ''}"
                    data-act="pick-profile" data-idx="${i}">
              ${profileTile(i, 'pm-row-icon')}
              <span class="pm-row-text">
                <span class="pm-row-name">${escape(p.name)}</span>
                <span class="pm-row-sub">${i === state.activeProfileIdx ? 'En uso' : ''}${
                  hasPages() && pageCountOf(p) > 1 ? `${i === state.activeProfileIdx ? ' · ' : ''}${pageCountOf(p)} páginas` : ''}</span>
              </span>
            </button>`).join('')}
        </div>
        <div class="pm-list-foot">
          <button class="pm-foot-btn" data-act="profile-new" ${full || view.busy ? 'disabled' : ''}
                  title="${full ? `Máximo ${state.maxProfiles} perfiles` : 'Crear un perfil vacío'}">
            ${icon('plus', 15)} Nuevo
          </button>
          <button class="pm-foot-btn ${view.libraryOpen ? 'on' : ''}" data-act="library-toggle"
                  title="Perfiles listos para Altium, KiCad, VS Code, Office, Photoshop...">
            ${icon('download', 15)} Biblioteca
          </button>
          <span class="pm-count">${state.profiles.length} / ${state.maxProfiles}</span>
        </div>
      </aside>

      <section class="pm-detail">
        ${view.libraryOpen ? renderLibrary() : renderProfileDetail()}
      </section>
    </div>`;

  // Los <canvas> de los iconos no se pueden serializar en la plantilla.
  cache.paintThumbs(body);
  paintProfileScreens(body, state.profiles);

  if (scrollList) body.querySelector('.pm-list-scroll').scrollTop = scrollList;
  if (scrollDetail) body.querySelector('.pm-detail').scrollTop = scrollDetail;
}

// El icono de un perfil: el suyo de verdad (el que enseña su menú en el
// teclado) en una pantallita negra, o la baldosa de color si no tiene.
function profileTile(i, cls) {
  return profileScreen(state.profiles[i], i)
    ? `<span class="${cls} is-oled"><canvas data-profile-screen="${i}"></canvas></span>`
    : `<span class="${cls}" style="--accent:${profileMeta(i).accent}">${icon(profileMeta(i).icon, 16)}</span>`;
}

// Ficha de un perfil, en grupos como Ajustes de iOS. Las teclas no se editan
// aquí: se editan en el Inicio, sobre la réplica, y desde aquí se va allí.
function renderProfileDetail() {
  const i = view.editingProfile;
  const prof = currentProfile();
  const live = i === state.activeProfileIdx;
  const full = state.profiles.length >= state.maxProfiles;
  const hasProfileIcon = compat.supports(state.deviceInfo, 'profileIcon');
  const count = pageCountOf(prof);
  const cur = currentPageIdx();
  const list = variants.forProfile(i, cur);

  const row = ({ act, ic, tint, title, sub = '', attrs = '', danger = false, chev = true }) => `
    <button class="nav-row ${danger ? 'is-danger' : ''}" data-act="${act}" data-tint="${tint}" ${attrs}>
      <span class="nav-icon">${icon(ic, 15)}</span>
      <span class="nav-row-text">
        <span class="nav-row-title">${title}</span>
        ${sub ? `<span class="nav-row-sub">${sub}</span>` : ''}
      </span>
      ${chev ? `<span class="nav-row-chev">${icon('chevron', 14)}</span>` : ''}
    </button>`;

  return `
    <div class="pm-hero">
      ${profileTile(i, 'pm-hero-icon')}
      <div class="pm-hero-text">
        <input type="text" maxlength="7" value="${escape(prof.name)}" data-act="edit-name"
               class="pm-name" spellcheck="false" aria-label="Nombre del perfil">
        <span class="pm-hero-sub">Hasta 7 letras: es lo que cabe en la pantalla del teclado</span>
      </div>
      ${live
        ? '<span class="pill pill-live">En uso</span>'
        : '<button class="primary-btn" data-act="activate">Usar en el teclado</button>'}
    </div>

    <div class="kp-list">
      ${row({ act: 'pm-edit-keys', ic: 'keyboard', tint: 'blue', title: 'Configurar teclas, rueda y mandos',
              sub: live ? 'En el Inicio, sobre la réplica' : 'Lo pone en el teclado y te lleva al Inicio' })}
      ${row({ act: 'profile-icon', ic: 'oled', tint: 'violet', title: 'Icono del perfil',
              sub: hasProfileIcon ? 'El que enseña el menú de perfiles del teclado'
                : `Necesita firmware ${compat.FEATURES.profileIcon.since} o posterior`,
              attrs: hasProfileIcon ? '' : 'disabled' })}
      ${row({ act: 'profile-dup', ic: 'copy', tint: 'gray', title: 'Duplicar', sub: 'Una copia con sus iconos, al final de la lista',
              attrs: full || view.busy ? 'disabled' : '', chev: false })}
    </div>

    ${hasPages() ? `
      <h4 class="kp-section" id="pm-pages">Páginas</h4>
      <div class="kp-list" id="pm-pages-list">
        ${Array.from({ length: count }, (_, n) => `
          <div class="nav-row pm-page ${n === cur ? 'on' : ''}">
            <button class="pm-page-pick" data-act="page" data-page="${n}" ${live ? '' : 'disabled'}>
              <span class="pm-page-num">${n + 1}</span>
              <span class="nav-row-title">Página ${n + 1}</span>
              ${n === cur ? `<span class="pm-check">${icon('check', 15)}</span>` : ''}
            </button>
            ${count > 1 ? `<button class="pm-page-del" data-act="page-del" data-page="${n}" title="Eliminar la página ${n + 1}"
                                    ${live ? '' : 'disabled'}>${icon('minus', 14)}</button>` : ''}
          </div>`).join('')}
        <button class="nav-row pm-add" data-act="page-add" ${live && count < maxPages() ? '' : 'disabled'}>
          <span class="pm-add-icon">${icon('plus', 14)}</span>
          <span class="nav-row-text">
            <span class="nav-row-title">Añadir página</span>
            ${count >= maxPages() ? `<span class="nav-row-sub">Ya tiene el máximo de ${maxPages()} páginas</span>` : ''}
          </span>
        </button>
      </div>
      <p class="kp-note">${live
        ? 'Cada página tiene sus teclas, mandos, rueda e iconos. En el teclado se pasa de una a otra con una pulsación corta del botón de menú.'
        : 'Las páginas se cambian sobre el perfil que tiene puesto el teclado: úsalo primero.'}</p>` : ''}

    <h4 class="kp-section">Según la app${hasPages() && count > 1 ? ` · página ${cur + 1}` : ''}</h4>
    <div class="kp-list">
      ${list.map((v) => row({ act: 'pick-variant', ic: 'bolt', tint: 'orange', title: escape(v.name),
        sub: `${variants.countOverrides(v)} cambios${v.matches.length ? ` · ${escape(v.matches.join(', '))}` : ' · sin apps'}${
          variants.isApplied(v.id) ? ' · aplicada ahora' : ''}`,
        attrs: `data-id="${v.id}"` })).join('')}
      <button class="nav-row pm-add" data-act="new-variant" ${platform.can('autoProfile') ? '' : 'disabled'}>
        <span class="pm-add-icon">${icon('plus', 14)}</span>
        <span class="nav-row-title">Nueva variación</span>
      </button>
    </div>
    <p class="kp-note">Una variación cambia un par de teclas de este perfil cuando tienes delante
      una app concreta. Lo que cambia se edita en el Inicio, eligiéndola arriba del panel.</p>
    ${renderVariantSettings()}

    <div class="kp-list pm-danger">
      ${row({ act: 'profile-del', ic: 'trash', tint: 'red', title: 'Eliminar perfil', danger: true, chev: false,
              attrs: state.profiles.length <= 1 || view.busy ? 'disabled' : '' })}
    </div>`;
}

// Piezas de la zona de la rueda, en el orden en que están en el aparato. Cada
// una se abre por su giro horario, que es lo primero que se quiere cambiar.
const WHEEL_PIECES = [
  { id: 'enc1',  label: 'Mando izq.', name: 'Mando izquierdo', slot: ROTARY_SLOTS.ENC1_CW,  group: ROTARY_GROUPS[0] },
  { id: 'wheel', label: 'Rueda',      name: 'Rueda',           slot: ROTARY_SLOTS.WHEEL_CW, group: WHEEL_GROUP },
  { id: 'enc2',  label: 'Mando der.', name: 'Mando derecho',   slot: ROTARY_SLOTS.ENC2_CW,  group: ROTARY_GROUPS[1] },
];

function pieceOf(slot) {
  if (slot <= ROTARY_SLOTS.ENC1_CLICK) return WHEEL_PIECES[0];
  if (slot <= ROTARY_SLOTS.ENC2_CLICK) return WHEEL_PIECES[2];
  return WHEEL_PIECES[1];
}

// El panel del Inicio: cabecera con "Listo", la capa, y debajo el inspector de
// siempre. En la zona de la rueda va además el selector de pieza y, de la pieza
// elegida, sus giros y su pulsación (o la sensibilidad, si es la rueda).
function renderPanel() {
  const host = view.panel;
  // Con la misma pieza se conserva el scroll (cada clic en un control repinta
  // el panel entero); con otra, se empieza arriba, que es donde está su nombre.
  const target = JSON.stringify([view.zone, view.selected, view.panelNav]);
  const scrollTop = host.dataset.target === target ? host.querySelector('.zp-scroll')?.scrollTop : 0;
  host.dataset.target = target;

  if (!state.profiles.length) {
    host.innerHTML = `
      <header class="zp-head"><div class="zp-titles"><h2>Sin perfiles</h2></div>
        <button class="zp-done" data-act="panel-done">Listo</button></header>
      <div class="zp-scroll"><p class="setting-desc">Conecta el Orvy una vez y quedará una copia en el PC.</p></div>`;
    return;
  }
  // Siempre el perfil del teclado, que es el que enseña la réplica. La vista
  // de perfiles puede haber elegido otro mientras el panel estaba abierto.
  if (view.editingProfile !== state.activeProfileIdx) {
    view.editingProfile = state.activeProfileIdx;
    view.variantId = null;
  }

  const prof = currentProfile();
  const count = pageCountOf(prof);
  // Variaciones de esta página: con alguna, el panel deja elegir si se edita
  // el perfil base o lo que cambia una de ellas.
  const vlist = variants.forProfile(view.editingProfile, currentPageIdx());
  const where = hasPages() && count > 1 ? `${prof.name} · página ${currentPageIdx() + 1} de ${count}` : prof.name;
  const piece = view.zone === 'wheel' && view.selected?.kind === 'rotary' ? pieceOf(view.selected.slot) : null;

  host.innerHTML = `
    <header class="zp-head">
      <div class="zp-titles">
        <h2>${view.zone === 'keys' ? 'Teclas' : 'Rueda y mandos'}</h2>
        <span>${escape(where)}</span>
      </div>
      <div class="zp-head-actions">
        <button class="zp-pages zp-undo" data-act="undo" ${undoAttrs()}>${icon('undo', 15)}</button>
        ${hasPages() ? `
          <button class="zp-pages" data-act="go-pages" title="Añadir, quitar o cambiar de página">
            ${icon('pages', 14)}<span>Páginas</span>
          </button>` : ''}
        <button class="zp-done" data-act="panel-done">Listo</button>
      </div>
    </header>
    <div class="zp-bar">
      <div class="layer-toggle">
        <button class="toggle-btn ${view.layer === 'normal' ? 'active' : ''}" data-act="layer" data-layer="normal">NORMAL</button>
        <button class="toggle-btn ${view.layer === 'super' ? 'active' : ''}" data-act="layer" data-layer="super">SUPER</button>
      </div>
      ${vlist.length ? `
        <select class="select-input zp-variant" data-act="panel-variant" title="Qué se edita">
          <option value="" ${!view.variantId ? 'selected' : ''}>Perfil base</option>
          ${vlist.map((v) => `<option value="${v.id}" ${view.variantId === v.id ? 'selected' : ''}>
            Variación: ${escape(v.name)}</option>`).join('')}
        </select>` : ''}
      ${view.zone === 'wheel' ? `
        <div class="layer-toggle zp-pieces">
          ${WHEEL_PIECES.map((p) => `
            <button class="toggle-btn ${piece?.id === p.id ? 'active' : ''}" data-act="zp-piece"
                    data-slot="${p.slot}">${p.label}</button>`).join('')}
        </div>` : ''}
    </div>
    <div class="zp-scroll">
      <div class="zp-page ${view.navDir ? `enter-${view.navDir}` : ''}">
        ${view.zone === 'keys' ? renderKeyPanel() : renderPiecePanel(piece)}
      </div>
    </div>`;
  view.navDir = null;

  cache.paintThumbs(host);
  paintKeyScreens(host);
  if (piece?.id === 'wheel' && !view.panelNav) updateDerived(currentScroll().detentsPerRev);
  const scroller = host.querySelector('.zp-scroll');
  if (scroller && scrollTop) scroller.scrollTop = scrollTop;
  onPanelRender?.();
}

// Panel de una pieza de la zona de la rueda, con los mismos dos niveles que el
// de las teclas: la pieza (sus gestos y, la rueda, su sensibilidad) y, al
// entrar en un gesto, el editor de su acción.
function renderPiecePanel(piece) {
  if (!piece) return '';
  if (piece.id !== 'wheel' && isMenuEncoderSlot(piece.group.parts[0].slot)) return renderMenuEncoderPanel(piece);
  if (view.panelNav === 'gesture') {
    const found = rotaryPart(selectedRotarySlot());
    const action = currentRotary();
    const isClick = Boolean(found?.part.discrete);
    const name = !isClick && isUnifiedTurnAction(action) ? 'Girar' : gestureName(found?.part, piece);
    return `
      <button class="kp-back" data-act="kp-back">${icon('back', 15)}<span>${piece.name}</span></button>
      <div class="kp-kind-head" data-tint="${isClick ? 'orange' : 'blue'}">
        <span class="nav-icon">${icon(isClick ? 'record' : 'reset', 15)}</span>
        <h3>${escape(name)}</h3>
      </div>
      ${renderRotaryInspector()}`;
  }

  return `
    <div class="kp-hero">
      <span class="rp-dial ${piece.id === 'wheel' ? 'is-wheel' : ''}"><i></i></span>
      <div class="kp-hero-text">
        <h3>${piece.name}</h3>
        <span class="kp-now">${piece.id === 'wheel'
          ? `${currentScroll().detentsPerRev} clics por vuelta${currentScroll().invert ? ' · invertida' : ''}`
          : 'Gira y se pulsa'}</span>
      </div>
    </div>

    <h4 class="kp-section">Gestos</h4>
    <div class="kp-list">${gestureRows(piece)}</div>

    ${piece.id === 'wheel' ? renderWheelTune() : `
      <p class="kp-note">Dentro del menú del teclado los mandos siguen sirviendo para navegar,
        hagan lo que hagan aquí.</p>`}`;
}

// El mando de menú no se configura: lo lleva el firmware igual para todos los
// perfiles (es lo que hace la tecla de menú, y por eso no se deja sin ella).
function renderMenuEncoderPanel(piece) {
  return `
    <div class="kp-hero">
      <span class="rp-dial"><i></i></span>
      <div class="kp-hero-text">
        <h3>${piece.name}</h3>
        <span class="kp-now">Mando de menú</span>
      </div>
    </div>

    <div class="kp-list">
      <div class="nav-row" data-tint="blue">
        <span class="nav-icon">${icon('reset', 15)}</span>
        <span class="nav-row-text">
          <span class="nav-row-title">Girar</span>
          <span class="nav-row-sub is-set">Cambia de página</span>
        </span>
      </div>
      <div class="nav-row" data-tint="orange">
        <span class="nav-icon">${icon('record', 15)}</span>
        <span class="nav-row-text">
          <span class="nav-row-title">Mantener pulsado</span>
          <span class="nav-row-sub is-set">Selector de perfiles</span>
        </span>
      </div>
      <div class="nav-row" data-tint="indigo">
        <span class="nav-icon">${icon('record', 15)}</span>
        <span class="nav-row-text">
          <span class="nav-row-title">Mantener con SUPER</span>
          <span class="nav-row-sub is-set">Gestor de páginas</span>
        </span>
      </div>
    </div>

    <p class="kp-note">Se mantiene pulsado y se gira, sin soltar, hasta el elemento que se quiere;
      al soltar se elige. Un toque corto lo deja abierto hasta confirmar con otro clic. Mientras está
      abierto, las teclas siguen haciendo lo que les hayas asignado. Este mando no se configura por
      perfil; para usarlo de otra forma, quítalo en Ajustes → Tecla 10.</p>`;
}

// Nombre de cada gesto tal como se hace con la mano.
function gestureName(part, piece) {
  if (!part) return '';
  if (part.discrete) return 'Pulsar';
  const cw = part.slot === piece.group.parts[0].slot;
  if (piece.id === 'wheel') return cw ? 'Girar hacia abajo' : 'Girar hacia arriba';
  return cw ? 'Girar a la derecha' : 'Girar a la izquierda';
}

// Una fila por gesto. Un giro que se edita como uno solo (desplazar, zoom,
// pareja multimedia, complemento: ver isUnifiedTurnAction) sale como una sola
// fila "Girar"; si no, una por sentido.
function gestureRows(piece) {
  const prof = currentProfile();
  const variant = editingVariant();
  const [cwPart, ccwPart, clickPart] = piece.group.parts;
  const of = (slot) => variants.effectiveRotary(prof, variant, slot, view.layer);
  const changedOf = (slot) => variant && variants.override(variant, 'rotary', rotarySlot(slot, view.layer));
  const cw = of(cwPart.slot);
  const ccw = of(ccwPart.slot);
  const merged = isUnifiedTurnAction(cw) && isUnifiedTurnAction(ccw);

  const row = (slot, title, ic, tint, action) => `
    <button class="nav-row ${changedOf(slot) ? 'is-override' : ''}" data-act="rp-open" data-slot="${slot}" data-tint="${tint}">
      <span class="nav-icon">${icon(ic, 15)}</span>
      <span class="nav-row-text">
        <span class="nav-row-title">${title}</span>
        <span class="nav-row-sub ${action.type ? 'is-set' : ''}">${escape(describeRotaryFull(action))}${
          rotaryNeedsApp(action) ? ' · la ejecuta el PC' : ''}</span>
      </span>
      <span class="nav-row-chev">${icon('chevron', 14)}</span>
    </button>`;

  return (merged
    ? row(cwPart.slot, 'Girar', 'reset', 'blue', cw)
    : row(cwPart.slot, gestureName(cwPart, piece), 'reset', 'blue', cw)
      + row(ccwPart.slot, gestureName(ccwPart, piece), 'reset', 'indigo', ccw))
    + (clickPart ? row(clickPart.slot, 'Pulsar', 'record', 'orange', of(clickPart.slot)) : '');
}

// Sensibilidad de la rueda, con los mismos controles (y los mismos id, que
// leen onInput y updateDerived) que la tarjeta de la vista de Perfiles.
function renderWheelTune() {
  const s = currentScroll();
  return `
    <h4 class="kp-section">Sensibilidad</h4>
    ${editingVariant() ? `
      <p class="setting-desc override-note">
        La sensibilidad es siempre la del <strong>perfil base</strong>: las variaciones
        cambian atajos y etiquetas, no la calibración.
      </p>` : ''}
    <div class="wt-card">
      <div class="wheel-readout">
        <span id="scroll-value">${s.detentsPerRev}</span>
        <small>clics por vuelta completa</small>
      </div>
      <input type="range" id="scroll-slider" class="premium-slider wheel-slider" data-act="scroll-slider"
             min="3" max="120" step="1" value="${s.detentsPerRev}">
      <div class="slider-row"><span>Más lento</span><span>Más rápido</span></div>
      <div class="wt-presets">
        ${SCROLL_PRESETS.map((p) => `
          <button class="preset-btn ${s.detentsPerRev === p.value ? 'on' : ''}" data-act="scroll-preset" data-value="${p.value}">
            <strong>${p.name}</strong>
            <span>${p.value} clics</span>
          </button>`).join('')}
      </div>
    </div>

    <div class="kp-list wt-list">
      <div class="nav-row is-static" data-tint="gray">
        <span class="nav-icon">${icon('reset', 15)}</span>
        <span class="nav-row-text">
          <span class="nav-row-title">Invertir dirección</span>
          <span class="nav-row-sub">Para que la rueda baje al girarla hacia el otro lado</span>
        </span>
        <button class="switch ${s.invert ? 'on' : ''}" data-act="scroll-invert" title="Invertir dirección">
          <span class="switch-knob"></span>
        </button>
      </div>
    </div>

    <ul class="info-list wt-derived" id="scroll-derived"></ul>
    <p class="kp-note">
      ${state.scroll.hires
        ? 'Windows ha pedido la alta resolución: el desplazamiento vertical va en fracciones de clic.'
        : 'Windows todavía no ha pedido la alta resolución: las aplicaciones antiguas saltarán de tres en tres líneas.'}
      Cada perfil, y cada capa, guarda su propia sensibilidad.
    </p>`;
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
  // Sin escape(): confirmar() pone el texto con textContent, y escapado se
  // vería un &amp; literal en el título.
  if (!await confirmar({
    titulo: `¿Eliminar la página ${idx + 1} de ${prof.name}?`,
    mensaje: 'Se van sus teclas, etiquetas, mandos e iconos. Las páginas siguientes se recolocan.',
    aceptar: 'Eliminar',
    peligro: true,
  })) return;
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

// Lo llaman los módulos del editor cuando cambia lo que hay en una tecla. La
// réplica del Inicio se repinta sola con la caché; lo que hay que avisarle es
// qué pieza está elegida y en qué capa (ver openPanel).
function renderKeyGrid() {
  if (inPanel()) onPanelRender?.();
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
  if (isTwoWayType(action.type)) return true;
  if (isPluginTurnAction(action)) return true;
  return action.type === ROTARY_TYPES.CONSUMER && Boolean(consumerPairFor(action.keycode));
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

// El editor de la acción de un gesto, para el panel del Inicio: sin caja ni
// cabecera, que allí pone él con su "‹ Atrás".
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
  const puedeInvertirPlugin = isPlugin && !isClick && Boolean(Number(pluginStep.value));
  // Igual que el giro de un complemento, una pareja multimedia (Volumen,
  // Brillo, Zoom) reparte Subir/Bajar entre los dos huecos por construcción:
  // también se puede montar al revés, así que también necesita su botón.
  const consumerPair = !isClick && action.type === ROTARY_TYPES.CONSUMER
    ? consumerPairFor(action.keycode) : null;
  const puedeInvertir = puedeInvertirPlugin || Boolean(consumerPair);
  const invertido = puedeInvertirPlugin ? giroInvertido(base, pluginStep)
    : consumerPair ? consumerGiroInvertido(base, action) : false;
  const types = ROTARY_TYPE_OPTIONS.filter((t) => !(isClick && t.turnOnly) && !(!isClick && t.clickOnly)
    && (!t.feature || compat.supports(state.deviceInfo, t.feature)));

  const variant = editingVariant();
  const changed = Boolean(variant && variants.override(variant, 'rotary', rotarySlot(base, view.layer)));
  const baseAction = currentProfile().rotary?.[rotarySlot(base, view.layer)] || { type: 0, modifier: 0, keycode: 0 };


  return `
    <div class="rp-body-bare">

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
            <button class="consumer-chip ${invertido ? 'on' : ''}" data-act="rotary-invert"
                    style="margin-top:8px">
              ${icon('reset', 14)} Invertir giro
            </button>` : ''}
          <p class="setting-desc">
            ${isClick
              ? 'Esto lo maneja el PC, así que Orvy tiene que estar abierto.'
              : `Cada muesca mueve el valor un paso, y el sentido lo pone el propio giro: pon lo
                 mismo en los dos sentidos del encoder y ya sube y baja. Si el mando está montado
                 al revés, <em>Invertir giro</em> le da la vuelta a los dos sentidos a la vez.
                 Esto lo maneja el PC, así que Orvy tiene que estar abierto.`}
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
          ${consumerPair ? `
            <button class="consumer-chip ${invertido ? 'on' : ''}" data-act="rotary-invert"
                    style="margin-top:8px">
              ${icon('reset', 14)} Invertir giro
            </button>` : ''}
          ${!isClick ? `
            <p class="setting-desc">
              Cada muesca sube o baja un paso, y el sentido lo pone el propio giro: no hace
              falta elegir qué va en cada lado. Si el mando está montado al revés,
              <em>Invertir giro</em> le da la vuelta a los dos sentidos a la vez.
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

      ${action.type === ROTARY_TYPES.PAGE ? `
        <p class="setting-desc">
          Una página por muesca: en sentido horario, la siguiente; al revés, la anterior.
          Da la vuelta al llegar al final. Con un perfil de una sola página no hace nada.
        </p>` : ''}

      ${action.type === ROTARY_TYPES.MENU ? `
        <div class="field">
          <span class="field-label">Qué menú</span>
          <div class="consumer-grid">
            ${ROTARY_MENUS.map((m) => `
              <button class="consumer-chip ${action.keycode === m.value ? 'on' : ''}"
                      data-act="rotary-menu" data-menu="${m.value}">${m.label}</button>`).join('')}
          </div>
          <p class="setting-desc">
            Mantenlo pulsado y gira sin soltar hasta lo que quieras: al soltar se elige. Un
            toque corto deja el menú abierto, y entonces cualquiera de los dos mandos mueve la
            selección al girar y la confirma al pulsar. Con el menú abierto, las teclas siguen
            haciendo lo que tengan asignado.
          </p>
        </div>` : ''}

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

// Lo que se edita de cada tipo de acción de una tecla. Lo usan el inspector de
// la vista de Perfiles (debajo de sus pestañas) y el panel del Inicio (en la
// pantalla a la que se entra desde cada tipo).
function keyTabBody(tab, action) {
  // Los cuatro modos que se apropian del campo del modificador: mientras uno
  // esté puesto, ni los modificadores ni la tecla pintan nada.
  const isConsumer = action.modifier === CONSUMER_MODIFIER;
  const isMacro    = action.modifier === MACRO_MODIFIER;
  const isSpecial  = isConsumer || isMacro || action.modifier === GOTO_PAGE_MODIFIER
                                           || action.modifier === PAGE_STATE_MODIFIER;
  const keyOptions = tab === 'shortcut' ? keycodeOptions(isSpecial ? 0 : action.keycode) : '';
  return `
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
            Esta acción la ejecuta el PC, no el teclado, así que Orvy tiene que estar abierto
            —vale con el icono de la bandeja— para que funcione.
          </p>
        </div>
        ${renderPluginKeyActions(isMacro ? action.keycode : null)}` : ''}

      ${tab === 'pages' ? renderPageActions(action) : ''}`;
}

// Un bloque por complemento instalado dentro de la pestaña Multimedia, con sus
// acciones asignables a una tecla. Sin complementos no pinta nada: la
// instalación por defecto de Orvy no trae ninguno, y una sección vacía solo
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
        ${escape(p.description || 'Lo ejecuta el PC, así que Orvy tiene que estar abierto.')}
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

