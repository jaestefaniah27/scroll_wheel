// Editor de iconos OLED (72x40, 1 bit).
//
// Dibuja sobre un framebuffer con el mismo direccionamiento que usa el firmware
// y lo sube troceado con OLED_CHUNK. El hueco es (pantalla-1) + 10 si la capa
// es SUPER, exactamente el índice que consulta refresh_single_screen().
//
// No es una vista a la que se pueda entrar por la barra superior: se abre
// siempre desde la tecla que se esté configurando (o desde el icono del perfil)
// y edita ESE icono y ninguno más. Por eso no hay selector de perfil, de capa
// ni réplica del teclado; el destino ya viene decidido en openTarget().
//
// La forma es la de una hoja de edición de Apple (Fotos, Marcación): el lienzo
// es el escenario, con la pantalla del teclado como protagonista; las
// herramientas flotan debajo, y el inspector de la derecha cambia según lo que
// se esté haciendo —añadir algo o ajustar lo que se está colocando—. Se sale
// por "Cancelar" o "Guardar", arriba, como en cualquier hoja.

import * as device from '../device.js';
import { state, markDirty, subscribe, KEY_TO_SCREEN, hasPages, pageCountOf,
         ensureEditTarget } from '../store.js';
import { icon } from '../icons.js';
import * as lib from '../icon-library.js';
import { toast, requireDevice, confirmar, pintarDeslizador } from '../ui.js';
import { goBack } from '../nav.js';
import * as fb from '../oled-fb.js';
import * as cache from '../oled-cache.js';

const ZOOM_MIN = 3;
const ZOOM_MAX = 28;
const ZOOM_DEFAULT = 9;

// El texto se rasteriza a este cuerpo y luego se escala a lo que pida la capa.
// No se enseña: el tamaño que entiende el usuario es el de la capa en píxeles
// de la pantalla, y un segundo "tamaño" de fuente solo cambiaba la calidad del
// trazo, que con un cuerpo grande sale siempre mejor al encogerlo.
const TEXT_RASTER_PX = 32;

// Fuentes presentes en cualquier instalación de Windows. Nada de webfonts: la
// app tiene que funcionar sin conexión.
const FONTS = [
  { css: 'Segoe UI',        label: 'Segoe UI' },
  { css: 'Segoe UI Black',  label: 'Segoe Black' },
  { css: 'Arial',           label: 'Arial' },
  { css: 'Arial Black',     label: 'Arial Black' },
  { css: 'Impact',          label: 'Impact' },
  { css: 'Tahoma',          label: 'Tahoma' },
  { css: 'Verdana',         label: 'Verdana' },
  { css: 'Consolas',        label: 'Consolas' },
  { css: 'Courier New',     label: 'Courier New' },
  { css: 'Georgia',         label: 'Georgia' },
  { css: 'Times New Roman', label: 'Times' },
];

// `key` es el atajo de teclado: las letras de siempre en los editores de
// imagen (la G de relleno y la M de selección vienen de Photoshop).
const TOOLS = [
  { id: 'pencil', icon: 'pencil', label: 'Lápiz',     key: 'P' },
  { id: 'eraser', icon: 'eraser', label: 'Borrador',  key: 'E' },
  { id: 'fill',   icon: 'fill',   label: 'Relleno',   key: 'G' },
  { id: 'select', icon: 'select', label: 'Selección', key: 'M' },
];

const TABS = [
  { id: 'library', label: 'Iconos' },
  { id: 'text',    label: 'Texto' },
  { id: 'image',   label: 'Imagen' },
];

const view = {
  profile: 0,
  // 'key' (por defecto) edita el icono de una tecla; 'profile' edita el icono
  // del perfil que pinta el menú físico de perfiles (hueco fijo, página 0).
  kind: 'key',
  // Página del perfil que se estaba editando al entrar. Se guarda porque el
  // teclado puede cambiar de página mientras se dibuja (su botón de menú, una
  // tecla de salto), y el icono tiene que acabar en la que se abrió, no en la
  // que el teclado tenga puesta al pulsar "Guardar".
  page: 0,
  key: 0,          // índice de tecla 0..11
  layer: 'normal',
  tool: 'pencil',
  tab: 'library',
  zoom: ZOOM_DEFAULT,
  buffer: fb.createFramebuffer(),
  drawing: false,
  drawValue: 1,
  lastPos: null,
  undoStack: [],
  redoStack: [],
  dirty: false,    // se ha tocado el lienzo desde que se abrió esta tecla
  saving: false,

  // Capa flotante en colocación (icono de la biblioteca, texto, imagen
  // importada o una selección recortada del propio lienzo)
  source: null,
  layerXf: null,   // { x, y, scale, threshold, blur, dither, invert, mode }
  // Mientras el usuario no mueva ni escale la capa, se mantiene encajada: así
  // el texto crece letra a letra sin salirse de la pantalla. En cuanto la toca,
  // manda él.
  layerMoved: false,
  dragging: null,  // { kind:'move'|'resize', ... }
  selecting: null, // { x0, y0, x1, y1 } mientras se arrastra un recuadro de selección
  outline: null,   // último recuadro pintado sobre el lienzo (ver repaint)

  textDraft: '',
  textFont: 'Segoe UI',
  textBold: true,

  // Biblioteca de iconos: filtros del buscador. Se guardan en la vista para que
  // volver de colocar un icono no obligue a rebuscar lo mismo.
  libQuery: '',
  libCat: 'all',
};

function root() {
  return document.getElementById('view-oled');
}

// El atajo de teclado y el pegado escuchan en `window`: sin esto, Esc o Ctrl+Z
// en cualquier otra vista acabarían tocando un icono que no se está viendo.
function isActive() {
  return !!root()?.classList.contains('active');
}

export function init() {
  render();
  const el = root();
  el.addEventListener('click', onClick);
  el.addEventListener('change', onChange);
  el.addEventListener('input', onInput);
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('paste', onPaste);
  bindDrop(el);

  // Al conectar NO se tira la caché de iconos.
  //
  // Se hacía cuando la única forma de saber qué tenía el teclado era leerlo
  // entero: lo guardado podía ser de otro teclado o de antes de un cambio, así
  // que se empezaba de cero. Ahora hay algo mejor: la copia del PC (mirror.js)
  // llega con sus bitmaps y las huellas de GET_HASH dicen cuáles siguen
  // valiendo; main.js tira los de los perfiles que SÍ han cambiado.
  //
  // Y era caro: este manejador es síncrono y el de main.js no (espera a la copia
  // del PC y a GET_HASH), así que el borrado siempre ganaba la carrera. La
  // comparación se encontraba la caché vacía, no podía calcular ninguna huella
  // —hace falta el bitmap de cada hueco ocupado— y concluía que TODOS los
  // perfiles habían cambiado: en cada conexión se releían los perfiles enteros
  // y se volvían a bajar los veinte iconos de cada página.
  device.on('connected', () => { loadPreviews(); });

  let lastProfileCount = 0;
  subscribe(() => {
    if (state.profiles.length !== lastProfileCount) {
      lastProfileCount = state.profiles.length;
      if (view.profile >= state.profiles.length) view.profile = 0;
      loadPreviews();
      render();
    }
  });
}

// Abre el editor sobre una tecla concreta, o sobre el icono del perfil con
// `kind: 'profile'`. Es la ÚNICA forma de entrar aquí: lo llama el botón
// "Editar icono" del editor de perfiles (o el de "Icono del perfil" en la
// barra de pestañas) con el destino ya decidido.
export function openTarget({ profile, key, layer, kind } = {}) {
  if (Number.isInteger(profile) && profile < state.profiles.length) view.profile = profile;
  if (layer === 'normal' || layer === 'super') view.layer = layer;
  view.kind = kind === 'profile' ? 'profile' : 'key';
  cancelLayer(false);

  if (Number.isInteger(key) && screenOf(key)) view.key = key;
  // El icono del perfil vive fijo en la página 0, sea cual sea la que el
  // teclado tenga puesta.
  view.page = view.kind === 'profile' ? 0 : (state.profiles[view.profile]?.pageIdx || 0);

  const cached = cache.get(view.profile, slot(), view.page);
  view.buffer = cached ? Uint8Array.from(cached) : fb.createFramebuffer();
  view.undoStack = [];
  view.redoStack = [];
  view.dirty = false;
  view.saving = false;
  render();
  loadPreviews();
  // Se abre encajado en el hueco que haya, como una foto en Fotos. Hace falta
  // esperar un fotograma: hasta que la vista no está maquetada, el contenedor
  // del lienzo no tiene medidas.
  requestAnimationFrame(() => setZoom(fitZoom()));

  // La barra superior pone un título fijo por vista; aquí depende del destino.
  const title = document.getElementById('view-title');
  if (title) title.textContent = view.kind === 'profile' ? 'Icono del perfil' : 'Icono de la tecla';
}

// Antes de irse por cualquier camino (Cancelar, Esc o la flecha de la barra
// superior, que entra por main.js): si hay algo que se perdería, se pregunta.
// Devuelve si se puede salir.
export async function confirmLeave() {
  if (!view.dirty && !view.layerXf) return true;
  const ok = await confirmar({
    titulo: '¿Descartar los cambios?',
    mensaje: 'Lo que has dibujado en este icono no se guardará en el teclado.',
    aceptar: 'Descartar',
    peligro: true,
  });
  if (ok) {
    cancelLayer(false);
    view.dirty = false;
  }
  return ok;
}

// Vuelve a donde se vino: el panel del Inicio o la vista de Perfiles. La tecla
// sigue seleccionada allí, así que se cae justo en el mismo inspector.
// `dirty` se baja antes porque goBack() pasa otra vez por confirmLeave().
function backToProfiles() {
  cancelLayer(false);
  view.dirty = false;
  goBack();
}

async function exitWithoutSaving() {
  if (await confirmLeave()) backToProfiles();
}

// --- Destino ----------------------------------------------------------------

function screenOf(keyIndex) {
  return KEY_TO_SCREEN[keyIndex]; // 0 = la tecla no tiene pantalla
}

function slotOf(keyIndex, layer = view.layer) {
  const screen = screenOf(keyIndex);
  return screen ? (screen - 1) + (layer === 'super' ? 10 : 0) : -1;
}

function slot() {
  return view.kind === 'profile' ? device.PROFILE_ICON_SLOT : slotOf(view.key);
}

// Los bitmaps guardados se comparten con el editor de perfiles a través de
// oled-cache.js: leerlos por el CDC es lento y no tiene sentido hacerlo dos veces.
function loadPreviews() {
  cache.loadProfile(view.profile);
}

// --- Eventos ----------------------------------------------------------------

// Los controles segmentados se marcan en el sitio en vez de repintar el panel:
// repintarlo movería el scroll del inspector bajo el ratón.
function markSegment(el) {
  el.parentElement.querySelectorAll(`[data-act="${el.dataset.act}"]`)
    .forEach((b) => b.classList.toggle('active', b === el));
}

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el || el.disabled) return;
  const act = el.dataset.act;

  if (act === 'tool')          { setTool(el.dataset.tool); }
  else if (act === 'tab')      { view.tab = el.dataset.tab; renderInspector(); }
  else if (act === 'zoom-in')  { setZoom(view.zoom + zoomStep(view.zoom)); }
  else if (act === 'zoom-out') { setZoom(view.zoom - zoomStep(view.zoom - 1)); }
  else if (act === 'zoom-fit') { setZoom(fitZoom()); }
  else if (act === 'exit')     { exitWithoutSaving(); }
  else if (act === 'clear')    { pushUndo(); fb.clear(view.buffer); repaint(); }
  else if (act === 'invert')   { pushUndo(); fb.invert(view.buffer); repaint(); }
  else if (act === 'frame')    { pushUndo(); fb.drawFrame(view.buffer); repaint(); }
  else if (act === 'undo')     { undo(); }
  else if (act === 'redo')     { redo(); }
  else if (act === 'import')   { document.getElementById('oled-file').click(); }
  else if (act === 'apply')    { applyLayer(); }
  else if (act === 'cancel')   { cancelLayer(true); }
  else if (act === 'center')   { view.layerMoved = true; centerLayer(); }
  else if (act === 'fit-layer'){ view.layerMoved = false; fitLayer(); }
  else if (act === 'nudge')    { nudgeLayer(Number(el.dataset.dx), Number(el.dataset.dy)); }
  else if (act === 'size-step'){ view.layerMoved = true; setLayerHeight(layerHeightPx() + Number(el.dataset.d)); }
  else if (act === 'xf-invert') { view.layerXf.invert = el.dataset.mode; markSegment(el); repaint(); }
  else if (act === 'xf-blur')   { view.layerXf.blur = Number(el.dataset.v); markSegment(el); repaint(); }
  else if (act === 'layer-mode'){ view.layerXf.mode = el.dataset.mode; markSegment(el); repaint(); }
  else if (act === 'xf-dither') {
    view.layerXf.dither = !view.layerXf.dither;
    el.classList.toggle('on', view.layerXf.dither);
    el.setAttribute('aria-checked', String(view.layerXf.dither));
    repaint();
  }
  else if (act === 'text-font') {
    view.textFont = el.dataset.font;
    el.parentElement.querySelectorAll('[data-act="text-font"]')
      .forEach((b) => b.classList.toggle('on', b === el));
    if (view.source?.kind === 'text') refreshTextSource();
  }
  else if (act === 'text-bold') {
    view.textBold = el.dataset.bold === '1';
    markSegment(el);
    if (view.source?.kind === 'text') refreshTextSource();
  }
  else if (act === 'lib-cat')  { setLibCat(el.dataset.cat); }
  else if (act === 'lib-pick') { pickLibraryIcon(el.dataset.id); }
  else if (act === 'upload')   { upload(); }
  else if (act === 'reset-slot') { resetSlot(); }
  else if (act === 'load-current') { loadFromDevice(); }
}

function onChange(e) {
  if (e.target.id === 'oled-file' && e.target.files?.[0]) {
    importImage(e.target.files[0]);
    e.target.value = '';
  }
}

function onInput(e) {
  const act = e.target.dataset.act;
  if (!act) return;

  if (act === 'lib-search')      { view.libQuery = e.target.value; refreshLibraryGrid(); }
  else if (act === 'text-draft') { onTextDraft(e.target.value); }
  else if (!view.layerXf) return;
  else if (act === 'xf-size')      { view.layerMoved = true; setLayerHeight(Number(e.target.value), false); }
  else if (act === 'xf-threshold') { view.layerXf.threshold = Number(e.target.value); repaint(); }
}

function onKeyDown(e) {
  if (!isActive() || document.querySelector('.alert-backdrop')) return;
  const inField = ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName);
  const mod = e.ctrlKey || e.metaKey;
  const key = e.key.toLowerCase();

  // Lo que funciona también escribiendo: salir de la capa, fijarla o guardar.
  if (e.key === 'Escape') {
    e.preventDefault();
    if (view.layerXf) cancelLayer(true);
    else if (inField) e.target.blur();
    else exitWithoutSaving();
    return;
  }
  if (mod && e.key === 'Enter') { e.preventDefault(); view.layerXf ? applyLayer() : upload(); return; }
  if (mod && key === 's')       { e.preventDefault(); upload(); return; }
  if (inField) return;

  if (mod && (key === '+' || key === '=')) { e.preventDefault(); setZoom(view.zoom + zoomStep(view.zoom)); return; }
  if (mod && key === '-')                  { e.preventDefault(); setZoom(view.zoom - zoomStep(view.zoom - 1)); return; }
  if (mod && key === '0')                  { e.preventDefault(); setZoom(fitZoom()); return; }
  if (mod && (key === 'y' || (key === 'z' && e.shiftKey))) { e.preventDefault(); redo(); return; }
  if (mod && key === 'z') {
    e.preventDefault();
    // Con una capa a medio colocar, deshacer es quitarla: es lo último que se hizo.
    if (view.layerXf) cancelLayer(true); else undo();
    return;
  }

  if (view.layerXf) {
    if (e.key === 'Enter') { e.preventDefault(); applyLayer(); return; }
    if (e.key === 'Delete' || e.key === 'Backspace') { e.preventDefault(); deleteLayer(); return; }
    const step = e.shiftKey ? 5 : 1;
    const nudge = { ArrowLeft: [-step, 0], ArrowRight: [step, 0], ArrowUp: [0, -step], ArrowDown: [0, step] }[e.key];
    if (!nudge) return;
    e.preventDefault();
    nudgeLayer(nudge[0], nudge[1]);
    return;
  }

  if (mod || e.altKey) return;
  const tool = TOOLS.find((t) => t.key.toLowerCase() === key);
  if (tool) setTool(tool.id);
}

// Pegar una imagen copiada de cualquier sitio (una captura, el navegador) la
// coloca directamente, sin pasar por un fichero.
function onPaste(e) {
  if (!isActive() || ['INPUT', 'TEXTAREA'].includes(e.target.tagName)) return;
  const file = [...(e.clipboardData?.items || [])]
    .find((it) => it.kind === 'file' && it.type.startsWith('image/'))?.getAsFile();
  if (!file) return;
  e.preventDefault();
  importImage(file);
}

// Arrastrar una imagen desde el Explorador a cualquier punto del editor.
function bindDrop(el) {
  const hasFile = (ev) => [...(ev.dataTransfer?.types || [])].includes('Files');
  const show = (on) => document.getElementById('ie-stage')?.classList.toggle('is-drop', on);
  let depth = 0; // dragenter/dragleave saltan en cada hijo: se cuentan

  el.addEventListener('dragenter', (ev) => { if (hasFile(ev)) { depth++; show(true); } });
  el.addEventListener('dragleave', () => { if (depth && --depth === 0) show(false); });
  el.addEventListener('dragover', (ev) => { if (hasFile(ev)) ev.preventDefault(); });
  el.addEventListener('drop', (ev) => {
    if (!hasFile(ev)) return;
    ev.preventDefault();
    depth = 0;
    show(false);
    const file = [...ev.dataTransfer.files].find((f) => f.type.startsWith('image/'));
    if (file) importImage(file);
    else toast('Eso no es una imagen', 'error');
  });
}

function setTool(id) {
  if (view.layerXf) return;
  view.tool = id;
  renderPalette();
  renderCaption();
}

// --- Lienzo -----------------------------------------------------------------

function zoomStep(level) {
  return level < 8 ? 1 : (level < 16 ? 2 : 4);
}

function fitZoom() {
  const wrap = document.getElementById('oled-canvas-wrap');
  if (!wrap) return ZOOM_DEFAULT;
  const styles = getComputedStyle(wrap);
  // El marco de la pantalla también ocupa: se descuenta para que encajar deje
  // la pantalla entera a la vista, bisel incluido.
  const bezel = 2 * 14;
  const usableW = wrap.clientWidth - parseFloat(styles.paddingLeft) - parseFloat(styles.paddingRight) - bezel;
  const usableH = wrap.clientHeight - parseFloat(styles.paddingTop) - parseFloat(styles.paddingBottom) - bezel;
  return Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, Math.floor(Math.min(usableW / fb.OLED_W, usableH / fb.OLED_H))));
}

function clampZoom(z) {
  return Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, Math.round(z)));
}

function setZoom(next, anchor = null) {
  const target = clampZoom(next);
  if (target === view.zoom) return;

  const canvas = document.getElementById('oled-canvas');
  const wrap = document.getElementById('oled-canvas-wrap');

  let imageX = null, imageY = null;
  if (anchor && canvas) {
    const rect = canvas.getBoundingClientRect();
    imageX = (anchor.clientX - rect.left) / view.zoom;
    imageY = (anchor.clientY - rect.top) / view.zoom;
  }

  view.zoom = target;
  repaint();
  updateZoomLabel();

  if (imageX !== null && wrap && canvas) {
    const rect = canvas.getBoundingClientRect();
    wrap.scrollLeft += (rect.left + imageX * target) - anchor.clientX;
    wrap.scrollTop  += (rect.top  + imageY * target) - anchor.clientY;
  }
}

function updateZoomLabel() {
  const el = document.getElementById('oled-zoom-level');
  if (el) el.textContent = `${view.zoom}×`;
  const out = document.querySelector('#view-oled [data-act="zoom-out"]');
  const inn = document.querySelector('#view-oled [data-act="zoom-in"]');
  if (out) out.disabled = view.zoom <= ZOOM_MIN;
  if (inn) inn.disabled = view.zoom >= ZOOM_MAX;
}

const HANDLE_TOLERANCE = 4; // en píxeles de la pantalla

function bindCanvas() {
  const canvas = document.getElementById('oled-canvas');
  const wrap = document.getElementById('oled-canvas-wrap');
  if (!canvas) return;

  const posOf = (ev) => {
    const r = canvas.getBoundingClientRect();
    return {
      x: ((ev.clientX - r.left) / r.width) * fb.OLED_W,
      y: ((ev.clientY - r.top) / r.height) * fb.OLED_H,
    };
  };

  // Dónde caería el agarre: en el tirador de la esquina, escala; en el resto,
  // mueve. Se busca donde el usuario lo ve (el recuadro pintado, que se ciñe a
  // los píxeles).
  const onHandle = (p) => {
    const shown = view.outline || fb.layerBounds(view.source, view.layerXf);
    return Math.abs(p.x - (shown.x + shown.width)) <= HANDLE_TOLERANCE
        && Math.abs(p.y - (shown.y + shown.height)) <= HANDLE_TOLERANCE;
  };

  wrap?.addEventListener('wheel', (ev) => {
    if (!ev.ctrlKey) return;
    ev.preventDefault();
    const dir = ev.deltaY < 0 ? 1 : -1;
    setZoom(view.zoom + dir * zoomStep(dir > 0 ? view.zoom : view.zoom - 1), ev);
  }, { passive: false });

  canvas.addEventListener('pointerdown', (ev) => {
    ev.preventDefault();
    canvas.setPointerCapture(ev.pointerId);
    const p = posOf(ev);

    // Con una capa en colocación el lienzo sirve para moverla/escalarla, no
    // para pintar: pintar encima antes de fijarla no tendría sentido.
    if (view.layerXf) {
      // El arrastre se calcula sobre la caja de la capa: es la única que escala
      // de forma lineal y no se recorta al salirse de la pantalla, así que la
      // capa no pega saltos al agarrarla.
      const box = fb.layerBounds(view.source, view.layerXf);
      view.layerMoved = true;
      view.dragging = onHandle(p)
        ? { kind: 'resize', originX: box.x, originY: box.y, startH: box.height, startW: box.width }
        : { kind: 'move', grabX: p.x - box.x, grabY: p.y - box.y };
      return;
    }

    // Herramienta de selección: el propio arrastre dibuja el recuadro, y al
    // soltar (ver pointerup) se recorta a una capa flotante que se puede
    // mover o redimensionar con el mismo mecanismo de arriba.
    if (view.tool === 'select') {
      view.selecting = { x0: p.x, y0: p.y, x1: p.x, y1: p.y };
      return;
    }

    pushUndo();
    const px = Math.floor(p.x), py = Math.floor(p.y);
    if (view.tool === 'fill') {
      fb.floodFill(view.buffer, px, py, ev.button === 2 ? 0 : 1);
      repaint();
      return;
    }
    view.drawing = true;
    view.drawValue = (view.tool === 'eraser' || ev.button === 2) ? 0 : 1;
    view.lastPos = { x: px, y: py };
    fb.setPixel(view.buffer, px, py, view.drawValue);
    repaint();
  });

  canvas.addEventListener('pointermove', (ev) => {
    const p = posOf(ev);
    updateCoords(p);

    if (view.dragging) {
      if (view.dragging.kind === 'move') {
        view.layerXf.x = p.x - view.dragging.grabX;
        view.layerXf.y = p.y - view.dragging.grabY;
        repaint();
        updateXfLabels();
      } else {
        const { originX, originY, startW, startH } = view.dragging;
        const ratio = Math.max((p.x - originX) / Math.max(startW, 0.001),
                               (p.y - originY) / Math.max(startH, 0.001));
        setLayerHeight(startH * Math.max(0.05, ratio));
      }
      return;
    }

    if (view.layerXf) {
      canvas.style.cursor = onHandle(p) ? 'nwse-resize' : 'move';
      return;
    }

    if (view.selecting) {
      view.selecting.x1 = p.x;
      view.selecting.y1 = p.y;
      repaint();
      return;
    }

    if (!view.drawing) return;
    const px = Math.floor(p.x), py = Math.floor(p.y);
    if (view.lastPos) fb.drawLine(view.buffer, view.lastPos.x, view.lastPos.y, px, py, view.drawValue);
    view.lastPos = { x: px, y: py };
    repaint();
  });

  const stop = () => {
    view.drawing = false; view.lastPos = null; view.dragging = null;
    if (view.selecting) finishSelection();
  };
  canvas.addEventListener('pointerup', stop);
  canvas.addEventListener('pointercancel', () => {
    view.drawing = false; view.lastPos = null; view.dragging = null; view.selecting = null;
  });
  canvas.addEventListener('pointerleave', () => updateCoords(null));
  canvas.addEventListener('contextmenu', (ev) => ev.preventDefault());
}

// Abajo a la izquierda: el píxel bajo el cursor, o las medidas de la pantalla
// si el cursor no está encima. Es lo que hace falta para dibujar al píxel.
function updateCoords(p) {
  const el = document.getElementById('ie-coords');
  if (!el) return;
  const inside = p && p.x >= 0 && p.y >= 0 && p.x < fb.OLED_W && p.y < fb.OLED_H;
  el.textContent = inside
    ? `x ${Math.floor(p.x)}   y ${Math.floor(p.y)}`
    : `${fb.OLED_W} × ${fb.OLED_H} px`;
}

// --- Capa flotante ----------------------------------------------------------

function defaultTransform(source) {
  const scale = fb.fitScale(source);
  const ink = fb.sourceInkBox(source);
  return {
    // El origen de la capa no coincide con el de la tinta (el texto reserva
    // hueco para ascendentes y descendentes), así que centrar es centrar la
    // caja de tinta y descontar después ese desfase.
    x: (fb.OLED_W - ink.width * scale) / 2 - ink.x * scale,
    y: (fb.OLED_H - ink.height * scale) / 2 - ink.y * scale,
    scale,
    threshold: 128,
    // Los iconos de la biblioteca llegan marcados como `crisp`: son trazos
    // vectoriales limpios y el desenfoque solo les comería las puntas.
    blur: (source.kind === 'image' && !source.crisp) ? 1 : 0,
    // El difuminado solo tiene sentido en fotos; en iconos de línea es lo que
    // produce esos bordes mordidos, así que arranca desactivado.
    dither: false,
    invert: 'none',
    // El texto y los iconos se añaden por encima de lo que ya hubiera dibujado
    // (se puede ir componiendo un icono por partes); una imagen importada
    // sustituye el lienzo entero, que es lo esperable al pegar una foto.
    mode: (source.kind === 'text' || source.crisp) ? 'merge' : 'replace',
  };
}

// El tamaño se controla en píxeles de alto de la pantalla, no en porcentaje del
// original. Un porcentaje del tamaño natural tiene un rango útil distinto según
// la fuente —un PNG de 800 px encaja al 5 % y un texto de 16 px necesita 170 %—
// así que la barra quedaba casi toda inservible. En píxeles significa lo mismo
// siempre, y el tope de SIZE_MAX_PX limita la capa a tres veces la pantalla.
const SIZE_MIN_PX = 2;
const SIZE_MAX_PX = fb.OLED_H * 3;

// El tamaño se mide sobre la tinta, no sobre la caja de maquetación: "20 px"
// tiene que significar que lo dibujado mide 20 px de alto, que es lo que se ve
// en el recuadro y lo que dice la etiqueta.
function naturalHeight() {
  return Math.max(1, fb.sourceInkBox(view.source).height);
}

function layerHeightPx() {
  return view.layerXf.scale * naturalHeight();
}

function setLayerHeight(px, syncSlider = true) {
  if (!view.layerXf) return;
  const clamped = Math.max(SIZE_MIN_PX, Math.min(SIZE_MAX_PX, px));
  const before = fb.layerBounds(view.source, view.layerXf);
  const ink = fb.sourceInkBox(view.source);

  view.layerXf.scale = clamped / naturalHeight();
  // El desfase entre el origen de la capa y el de la tinta también escala, así
  // que hay que recolocar la capa o el contenido se iría de sitio al cambiar el
  // tamaño. Se ancla la esquina superior izquierda de lo dibujado.
  view.layerXf.x = before.x - ink.x * view.layerXf.scale;
  view.layerXf.y = before.y - ink.y * view.layerXf.scale;

  if (syncSlider) syncSizeSlider();
  repaint();
  updateXfLabels();
}

function syncSizeSlider() {
  const slider = document.getElementById('xf-size');
  if (!slider) return;
  slider.value = Math.round(layerHeightPx());
  pintarDeslizador(slider);
}

function nudgeLayer(dx, dy) {
  if (!view.layerXf) return;
  view.layerMoved = true;
  view.layerXf.x += dx;
  view.layerXf.y += dy;
  repaint();
  updateXfLabels();
}

// Recuadro de lo que se va a dibujar: los píxeles que de verdad se encienden.
// Si la capa se ha sacado entera de la pantalla no hay nada que medir y se cae
// a la caja de la capa, que sigue estando donde toca.
function inkBox(raster = null) {
  return fb.framebufferInk(raster || fb.rasterizeLayer(view.source, view.layerXf))
      || fb.layerBounds(view.source, view.layerXf);
}

// Centra la TINTA, no la caja de la capa. Van dos pasadas: la primera con las
// métricas de la fuente, que funcionan aunque la capa esté fuera de la pantalla
// pero se quedan a un par de píxeles; la segunda mide los píxeles ya
// rasterizados y corrige. Desplazar un número entero de píxeles mueve el
// resultado exactamente esa cantidad, así que con una corrección basta.
function placeCentered() {
  const ink = fb.sourceInkBox(view.source);
  const s = view.layerXf.scale;
  view.layerXf.x = Math.round((fb.OLED_W - ink.width * s) / 2) - ink.x * s;
  view.layerXf.y = Math.round((fb.OLED_H - ink.height * s) / 2) - ink.y * s;

  const box = inkBox();
  view.layerXf.x += Math.round((fb.OLED_W - box.width) / 2 - box.x);
  view.layerXf.y += Math.round((fb.OLED_H - box.height) / 2 - box.y);
}

function centerLayer() {
  placeCentered();
  repaint();
  updateXfLabels();
}

const FIT_MARGIN = 1; // píxeles que se dejan libres a cada lado

// Encajar = el contenido dibujado toca los bordes dejando FIT_MARGIN. La escala
// que sale de las métricas se queda cerca, así que se afina midiendo la tinta
// rasterizada; el bucle converge en dos o tres vueltas y si la capa se sale, lo
// que se mide es la pantalla entera, que también hace encoger.
function fitLayer() {
  view.layerXf.scale = fb.fitScale(view.source);

  for (let i = 0; i < 6; i++) {
    placeCentered();
    const box = inkBox();
    const ratio = Math.min((fb.OLED_W - 2 * FIT_MARGIN) / box.width,
                           (fb.OLED_H - 2 * FIT_MARGIN) / box.height);
    if (Math.abs(ratio - 1) < 0.02) break;
    view.layerXf.scale *= Math.max(0.5, Math.min(2, ratio));
  }

  syncSizeSlider();
  centerLayer();
}

// Toda capa nueva entra centrada por aquí, con la misma corrección sobre los
// píxeles rasterizados que aplica el botón de centrar: si no, lo que se coloca
// aparece un par de píxeles descuadrado y hay que pulsar "Centrar" para algo
// que ya debería estar centrado.
//
// Si ya había otra capa, la nueva la sustituye (se está probando cuál queda
// mejor), salvo una selección recortada: esa son píxeles del propio dibujo y
// se dejan donde estén antes de empezar con lo nuevo.
function beginLayer(source) {
  if (view.source?.cut) applyLayer();
  else if (view.layerXf) endLayer();
  view.source = source;
  view.layerXf = defaultTransform(source);
  view.layerMoved = false;
  placeCentered();
  refreshChrome();
}

async function importImage(file) {
  try {
    const source = await fb.loadImageSource(file);
    source.label = file.name || 'Imagen pegada';
    beginLayer(source);
  } catch (err) {
    toast(`No se pudo leer la imagen: ${err.message}`, 'error');
  }
}

// --- Biblioteca de iconos ---------------------------------------------------
//
// Un icono de la biblioteca entra por el mismo camino que una imagen importada:
// se convierte en capa flotante y se coloca con los mismos controles. Así no hay
// dos mecanismos distintos que mantener, y lo que se elige del catálogo se puede
// mover, escalar o invertir antes de fijarlo.

function setLibCat(cat) {
  view.libCat = cat;
  document.querySelectorAll('[data-act="lib-cat"]')
    .forEach((b) => b.classList.toggle('on', b.dataset.cat === cat));
  refreshLibraryGrid();
}

// Solo se repinta la rejilla, no el panel entero: rehacer el panel al teclear
// le quitaría el foco al buscador en cada letra.
function refreshLibraryGrid() {
  const grid = document.getElementById('oled-lib-grid');
  if (grid) grid.outerHTML = renderLibraryGrid();
}

async function pickLibraryIcon(id) {
  const item = lib.byId(id);
  if (!item) return;
  try {
    // Blanco explícito: el rasterizador mide luminancia, y el color del tema
    // caería según el umbral.
    const source = await fb.makeSvgSource(lib.svgMarkup(item, { size: 256, color: '#fff' }));
    source.label = item.name;
    beginLayer(source);
  } catch (err) {
    toast(`No se pudo cargar el icono: ${err.message}`, 'error');
  }
}

// --- Texto ------------------------------------------------------------------
//
// El texto aparece en el lienzo mientras se escribe: no hay botón de "colocar".
// La primera letra crea la capa y las siguientes la rehacen.

function textSource() {
  const s = fb.makeTextSource(view.textDraft, {
    fontSize: TEXT_RASTER_PX, bold: view.textBold, font: view.textFont,
  });
  s.label = 'Texto';
  return s;
}

function onTextDraft(value) {
  view.textDraft = value;
  const isText = view.source?.kind === 'text';
  if (!value.trim()) {
    if (isText) cancelLayer(true);
    return;
  }
  if (isText) refreshTextSource();
  else if (!view.layerXf) beginLayer(textSource());
}

// Rehace la fuente del texto. Si el usuario no ha tocado la capa se vuelve a
// encajar (el texto crece sin salirse); si la ha colocado, se respeta dónde.
function refreshTextSource() {
  const keep = { ...view.layerXf };
  view.source = textSource();
  view.layerXf = keep;
  if (!view.layerMoved) { fitLayer(); return; }
  repaint();
  updateXfLabels();
}

// Recorta el recuadro que se acaba de arrastrar con la herramienta de
// selección: los píxeles que había se quitan del lienzo (se cortan, no se
// copian) y pasan a una capa flotante normal, en el mismo sitio, lista para
// moverse o redimensionarse con el mecanismo ya existente de capas.
function finishSelection() {
  const { x0, y0, x1, y1 } = view.selecting;
  view.selecting = null;

  const left = Math.max(0, Math.floor(Math.min(x0, x1)));
  const top = Math.max(0, Math.floor(Math.min(y0, y1)));
  const right = Math.min(fb.OLED_W, Math.ceil(Math.max(x0, x1)));
  const bottom = Math.min(fb.OLED_H, Math.ceil(Math.max(y0, y1)));
  const width = right - left, height = bottom - top;
  if (width < 1 || height < 1) { repaint(); return; }

  let any = false;
  for (let y = top; y < bottom && !any; y++) {
    for (let x = left; x < right; x++) {
      if (fb.getPixel(view.buffer, x, y)) { any = true; break; }
    }
  }
  if (!any) { repaint(); return; } // nada dibujado ahí: no crea una capa vacía

  pushUndo();

  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#fff';
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      if (fb.getPixel(view.buffer, left + x, top + y)) {
        ctx.fillRect(x, y, 1, 1);
        fb.setPixel(view.buffer, left + x, top + y, 0);
      }
    }
  }

  view.source = {
    kind: 'image', bitmap: canvas, naturalWidth: width, naturalHeight: height, cut: true, label: 'Selección',
  };
  view.layerXf = {
    x: left, y: top, scale: 1, threshold: 128, blur: 0, dither: false, invert: 'none', mode: 'merge',
  };
  view.layerMoved = true;
  refreshChrome();
}

function endLayer() {
  if (view.source?.kind === 'text') view.textDraft = '';
  view.source = null;
  view.layerXf = null;
  view.dragging = null;
}

function applyLayer() {
  if (!view.layerXf) return;
  pushUndo();
  const raster = fb.rasterizeLayer(view.source, view.layerXf);
  view.buffer = fb.compose(view.buffer, raster, view.layerXf.mode);
  endLayer();
  refreshChrome();
}

function cancelLayer(rerender = true) {
  view.selecting = null;
  if (!view.layerXf) return;
  // Una selección cortada ya había quitado sus píxeles del lienzo (ver
  // finishSelection): al cancelar hay que devolvérselos, no solo descartar
  // la capa flotante.
  if (view.source?.cut) view.buffer = view.undoStack.pop() || view.buffer;
  endLayer();
  if (rerender) refreshChrome();
}

// Suprimir con una selección cortada la borra de verdad: sus píxeles ya salieron
// del lienzo, y el paso queda en el historial para deshacerlo. Con cualquier
// otra capa es lo mismo que cancelar.
function deleteLayer() {
  if (!view.source?.cut) { cancelLayer(true); return; }
  endLayer();
  refreshChrome();
}

// --- Historial y pintado ----------------------------------------------------

// Todo lo que modifica el lienzo pasa por aquí antes de tocarlo, así que es el
// sitio natural para marcar que hay algo que se perdería al salir sin guardar.
// Un cambio nuevo invalida lo que se pudiera rehacer, como en cualquier editor.
function pushUndo() {
  view.undoStack.push(view.buffer.slice());
  if (view.undoStack.length > 50) view.undoStack.shift();
  view.redoStack = [];
  view.dirty = true;
}

function undo() {
  const prev = view.undoStack.pop();
  if (!prev) return;
  view.redoStack.push(view.buffer);
  view.buffer = prev;
  view.dirty = true;
  repaint();
}

function redo() {
  if (view.layerXf) return;
  const next = view.redoStack.pop();
  if (!next) return;
  view.undoStack.push(view.buffer);
  view.buffer = next;
  view.dirty = true;
  repaint();
}

function selectingBounds() {
  const { x0, y0, x1, y1 } = view.selecting;
  return {
    x: Math.min(x0, x1), y: Math.min(y0, y1),
    width: Math.abs(x1 - x0), height: Math.abs(y1 - y0),
  };
}

function repaint() {
  let shown = view.buffer;

  if (view.layerXf) {
    const raster = fb.rasterizeLayer(view.source, view.layerXf);
    shown = fb.compose(view.buffer, raster, view.layerXf.mode);
    view.outline = inkBox(raster);
  } else {
    view.outline = view.selecting ? selectingBounds() : null;
  }
  const outline = view.outline;

  const canvas = document.getElementById('oled-canvas');
  if (canvas) fb.renderToCanvas(shown, canvas, { zoom: view.zoom, outline });

  const preview = document.getElementById('oled-preview');
  if (preview) fb.renderToCanvas(shown, preview, { zoom: 2, grid: false });

  updateHistoryButtons();
}

// Deshacer y rehacer se apagan cuando no hay nada que deshacer o rehacer: un
// botón que no hace nada al pulsarlo parece roto.
function updateHistoryButtons() {
  const placing = !!view.layerXf;
  const set = (act, enabled) => {
    const b = document.querySelector(`#ie-palette [data-act="${act}"]`);
    if (b) b.disabled = placing || !enabled;
  };
  set('undo', view.undoStack.length > 0);
  set('redo', view.redoStack.length > 0);
}

function updateXfLabels() {
  if (!view.layerXf) return;
  const set = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; };
  // Se enseña la medida real de lo dibujado, que es lo que marca el recuadro.
  // La barra sigue moviéndose sobre la caja de la capa (ver setLayerHeight):
  // necesita una escala lineal, y la tinta rasterizada da saltos de un píxel.
  const box = view.outline || inkBox();

  set('xf-size-val', `${Math.round(box.width)} × ${Math.round(box.height)} px`);
  // La posición que interesa es la de lo dibujado, igual que el tamaño.
  set('xf-pos-val', `x ${Math.round(box.x)}   y ${Math.round(box.y)}`);
}

// --- Envío al teclado -------------------------------------------------------

// Guardar = mandar el icono a la tecla y volver a la configuración de esa
// tecla, que es de donde se entró. Si falla el envío no se sale: si no, el
// dibujo se perdería sin que la tecla se hubiera enterado. Una capa a medio
// colocar se fija antes: es lo que el usuario está viendo en el lienzo.
async function upload() {
  if (view.saving || !requireDevice()) return;
  if (slot() < 0) { toast('Esa tecla no tiene pantalla', 'error'); return; }
  if (view.layerXf) applyLayer();

  view.saving = true;
  renderHead();
  try {
    // El teclado ha podido irse a otra página mientras se dibujaba: se vuelve a
    // la que se abrió antes de mandar nada, o el icono caería en la otra. El
    // icono del perfil no depende de la página puesta, así que no hace falta.
    if (view.kind !== 'profile') await ensureEditTarget(view.profile, view.page);
    await device.uploadOled(view.profile, slot(), view.buffer);
    cache.set(view.profile, slot(), Uint8Array.from(view.buffer), view.page);
    const prof = state.profiles[view.profile];
    // El alias `oledMask` apunta a la página que se esté editando, que no tiene
    // por qué ser la 0: el icono del perfil marca su máscara directamente.
    if (prof) {
      if (view.kind === 'profile') prof.pages[0].oledMask |= (1 << slot());
      else prof.oledMask |= (1 << slot());
    }
    markDirty();
    view.saving = false;
    toast(view.kind === 'profile' ? 'Icono guardado en el perfil' : `Icono guardado en la tecla ${view.key + 1}`);
    backToProfiles();
  } catch (err) {
    view.saving = false;
    renderHead();
    toast(`No se pudo guardar: ${err.message}`, 'error');
  }
}

// Quitar el icono no se puede deshacer (se borra en el teclado, no en el
// lienzo), así que se pregunta.
async function resetSlot() {
  if (!requireDevice()) return;
  if (!await confirmar({
    titulo: '¿Quitar el icono?',
    mensaje: view.kind === 'profile'
      ? 'El menú de perfiles volverá a mostrar el nombre del perfil.'
      : 'La tecla volverá a mostrar su nombre en la pantalla.',
    aceptar: 'Quitar',
    peligro: true,
  })) return;
  try {
    if (view.kind !== 'profile') await ensureEditTarget(view.profile, view.page);
    await device.clearOled(view.profile, slot());
    cache.set(view.profile, slot(), null, view.page);
    const prof = state.profiles[view.profile];
    if (prof) {
      if (view.kind === 'profile') prof.pages[0].oledMask &= ~(1 << slot());
      else prof.oledMask &= ~(1 << slot());
    }
    cancelLayer(false);
    fb.clear(view.buffer);
    view.undoStack = [];
    view.redoStack = [];
    markDirty();
    view.dirty = false;
    refreshChrome();
    toast(view.kind === 'profile' ? 'Icono del perfil quitado' : 'Icono quitado; vuelve el nombre de la tecla');
  } catch (err) {
    toast(`Error: ${err.message}`, 'error');
  }
}

async function loadFromDevice() {
  if (!requireDevice()) return;
  if (view.kind !== 'profile') await ensureEditTarget(view.profile, view.page);
  const bytes = await device.getOled(view.profile, slot());
  if (!bytes) { toast(view.kind === 'profile' ? 'Este perfil no tiene icono guardado' : 'Esa tecla no tiene icono guardado', 'info'); return; }
  cache.set(view.profile, slot(), bytes, view.page);
  cancelLayer(false);
  pushUndo();
  view.buffer = Uint8Array.from(bytes);
  refreshChrome();
  toast('Icono recuperado del teclado');
}

// --- Render -----------------------------------------------------------------
//
// render() monta el esqueleto entero y solo se llama al abrir. Lo demás se
// repinta por partes (cabecera, paleta, rótulo, inspector): rehacer el
// esqueleto devolvería el lienzo ampliado a su scroll inicial en cada cambio.

export function render() {
  const el = root();
  if (!el) return;

  el.innerHTML = `
    <div class="ie">
      <section class="ie-stage" id="ie-stage">
        <header class="ie-head" id="ie-head">${renderHeadInner()}</header>
        <p class="ie-caption" id="ie-caption">${captionText()}</p>
        <div class="ie-canvas-wrap" id="oled-canvas-wrap">
          <div class="ie-screen"><canvas id="oled-canvas"></canvas></div>
        </div>
        <div class="ie-bottombar">
          <span class="ie-coords" id="ie-coords">${fb.OLED_W} × ${fb.OLED_H} px</span>
          <div class="ie-palette" id="ie-palette">${renderPaletteInner()}</div>
          <div class="ie-zoom">
            <button class="ie-btn" data-act="zoom-out" title="Alejar (Ctrl −)">${icon('minus', 16)}</button>
            <button class="ie-zoom-val" id="oled-zoom-level" data-act="zoom-fit" title="Encajar en la ventana (Ctrl 0)">${view.zoom}×</button>
            <button class="ie-btn" data-act="zoom-in" title="Acercar (Ctrl +)">${icon('plus', 16)}</button>
          </div>
        </div>
        <div class="ie-drop">${icon('photo', 30)}<span>Suelta la imagen para colocarla</span></div>
      </section>

      <aside class="ie-inspector" id="ie-inspector">${renderInspectorInner()}</aside>
    </div>
    <input type="file" id="oled-file" accept="image/*" hidden>`;

  bindCanvas();
  repaint();
  updateXfLabels();
  updateZoomLabel();
}

// Todo lo que cambia al empezar o terminar de colocar una capa.
function refreshChrome() {
  renderPalette();
  renderCaption();
  renderInspector();
  const canvas = document.getElementById('oled-canvas');
  if (canvas) canvas.style.cursor = view.layerXf ? 'move' : 'crosshair';
  repaint();
  updateXfLabels();
}

function renderHead() {
  const el = document.getElementById('ie-head');
  if (el) el.innerHTML = renderHeadInner();
}

function renderPalette() {
  const el = document.getElementById('ie-palette');
  if (el) el.innerHTML = renderPaletteInner();
}

function renderCaption() {
  const el = document.getElementById('ie-caption');
  if (el) el.innerHTML = captionText();
}

// Repintar el inspector no puede quitarle el foco a lo que se esté escribiendo:
// la primera letra del texto crea la capa y cambia el panel entero, y el
// usuario tiene que poder seguir escribiendo sin volver a pinchar el campo.
function renderInspector() {
  const el = document.getElementById('ie-inspector');
  if (!el) return;
  const focused = document.activeElement;
  const act = el.contains(focused) ? focused.dataset?.act : null;
  const caret = act && 'selectionStart' in focused ? [focused.selectionStart, focused.selectionEnd] : null;

  el.innerHTML = renderInspectorInner();

  if (!act) return;
  const again = el.querySelector(`[data-act="${act}"]`);
  if (!again) return;
  again.focus({ preventScroll: true });
  if (caret) again.setSelectionRange(...caret);
}

// Cabecera de hoja: Cancelar a la izquierda, qué se edita en el centro y
// Guardar a la derecha, como en Fotos o Calendario.
function renderHeadInner() {
  const prof = state.profiles[view.profile];
  const profName = escape(prof?.name || `Perfil ${view.profile + 1}`);
  const canSave = slot() >= 0 && !view.saving;

  let title, meta;
  if (view.kind === 'profile') {
    // Sin tecla, sin pantalla, sin capa y sin página: el icono del perfil es
    // uno solo y no depende de nada de eso.
    title = profName;
    meta = ['Icono del menú de perfiles'];
  } else {
    title = `Tecla ${view.key + 1}`;
    meta = [profName];
    // La página es la que se abrió (view.page), no la que el teclado tenga
    // puesta ahora: es a la que va el icono al guardar.
    if (hasPages() && prof && pageCountOf(prof) > 1) meta.push(`Página ${view.page + 1} de ${pageCountOf(prof)}`);
    if (view.layer === 'super') meta.push('<span class="ie-pill">SUPER</span>');
  }
  const dest = view.kind === 'profile' ? 'al perfil' : `a la tecla ${view.key + 1}`;

  return `
    <button class="ie-cancel" data-act="exit" title="Salir sin guardar (Esc)">Cancelar</button>
    <div class="ie-title">
      <h1>${title}</h1>
      <p>${meta.join('<i aria-hidden="true">·</i>')}</p>
    </div>
    <button class="primary-btn ie-save" id="btn-oled-upload" data-act="upload"
            title="Mandar el icono ${dest} (Ctrl S)" ${canSave ? '' : 'disabled'}>
      ${view.saving ? 'Guardando…' : 'Guardar'}
    </button>`;
}

// La paleta flotante: herramientas, historial y acciones sobre el lienzo.
// Mientras se coloca una capa se apaga entera: el lienzo está ocupado moviendo
// la capa y pintar debajo no tendría sentido.
function renderPaletteInner() {
  const off = view.layerXf ? 'disabled' : '';
  const btn = (act, ic, label, extra = '') =>
    `<button class="ie-btn" data-act="${act}" title="${label}" aria-label="${label}" ${off} ${extra}>${icon(ic, 18)}</button>`;

  return `
    <div class="ie-tools" role="radiogroup" aria-label="Herramienta">
      ${TOOLS.map((t) => `
        <button class="ie-btn ie-tool ${view.tool === t.id && !view.layerXf ? 'on' : ''}" data-act="tool" data-tool="${t.id}"
                role="radio" aria-checked="${view.tool === t.id}" title="${t.label} (${t.key})" aria-label="${t.label}" ${off}>
          ${icon(t.icon, 18)}
        </button>`).join('')}
    </div>
    <span class="ie-sep"></span>
    ${btn('undo', 'undo', 'Deshacer (Ctrl Z)')}
    ${btn('redo', 'redo', 'Rehacer (Ctrl Y)')}
    <span class="ie-sep"></span>
    ${btn('frame', 'square', 'Dibujar un marco')}
    ${btn('invert', 'invert', 'Invertir el lienzo')}
    ${btn('clear', 'trash', 'Vaciar el lienzo', 'data-danger')}`;
}

// Una línea sobre el lienzo que dice qué hace el ratón con la herramienta
// puesta. Sustituye al párrafo de ayuda de antes: nadie lee un párrafo, pero
// una frase justo encima de donde se pincha sí.
function captionText() {
  if (view.layerXf) {
    return 'Arrastra para mover · tira de la esquina para cambiar el tamaño · '
         + '<kbd>↩</kbd> fija · <kbd>Esc</kbd> cancela';
  }
  return {
    pencil: 'Clic para encender píxeles · clic derecho para apagarlos',
    eraser: 'Arrastra para apagar píxeles',
    fill:   'Clic para rellenar una zona cerrada · clic derecho para vaciarla',
    select: 'Arrastra un recuadro alrededor de algo para moverlo o cambiarle el tamaño',
  }[view.tool];
}

function renderInspectorInner() {
  return `
    <div class="ie-preview">
      <div class="ie-preview-screen"><canvas id="oled-preview" width="${fb.OLED_W * 2}" height="${fb.OLED_H * 2}"></canvas></div>
      <span>Vista previa</span>
    </div>
    ${view.layerXf ? renderPlacingPanel() : renderAddPanel()}`;
}

// --- Inspector: añadir ------------------------------------------------------

function renderAddPanel() {
  const body = { library: renderLibraryTab, text: renderTextTab, image: renderImageTab }[view.tab]();
  return `
    <div class="ie-tabs layer-toggle" role="tablist">
      ${TABS.map((t) => `
        <button class="toggle-btn ${view.tab === t.id ? 'active' : ''}" data-act="tab" data-tab="${t.id}"
                role="tab" aria-selected="${view.tab === t.id}">${t.label}</button>`).join('')}
    </div>
    <div class="ie-scroll">${body}</div>
    ${renderDeviceFooter()}`;
}

function renderLibraryTab() {
  return `
    <label class="ie-search">
      ${icon('search', 15)}
      <input type="search" data-act="lib-search" placeholder="Buscar ${lib.ICONS.length} iconos"
             value="${escape(view.libQuery)}" spellcheck="false">
    </label>
    <div class="ie-chips" role="tablist">
      <button class="ie-chip ${view.libCat === 'all' ? 'on' : ''}" data-act="lib-cat" data-cat="all">Todos</button>
      ${lib.CATEGORIES.map((c) => `
        <button class="ie-chip ${view.libCat === c.id ? 'on' : ''}" data-act="lib-cat" data-cat="${c.id}">${c.label}</button>`).join('')}
    </div>
    ${renderLibraryGrid()}`;
}

// Sin buscar y con "Todos", el catálogo va por secciones, como el de SF Symbols:
// una rejilla de doscientos iconos sin orden no se recorre con la vista.
function renderLibraryGrid() {
  const found = lib.search(view.libQuery, view.libCat);
  if (!found.length) {
    return `
      <div class="ie-empty" id="oled-lib-grid">
        ${icon('search', 26)}
        <strong>Sin resultados</strong>
        <span>Prueba con otra palabra, como «flecha», «carpeta» o «volumen».</span>
      </div>`;
  }
  const cell = (it) => `
    <button class="ie-icon" data-act="lib-pick" data-id="${it.id}" title="${escape(it.name)}" aria-label="${escape(it.name)}">
      ${lib.svgMarkup(it, { size: 24 })}
    </button>`;

  if (view.libCat !== 'all' || view.libQuery.trim()) {
    return `<div id="oled-lib-grid"><div class="ie-icon-grid">${found.map(cell).join('')}</div></div>`;
  }
  return `
    <div id="oled-lib-grid">
      ${lib.CATEGORIES.map((c) => {
        const items = found.filter((it) => it.cat === c.id);
        return items.length ? `
          <h3 class="ie-section">${c.label}</h3>
          <div class="ie-icon-grid">${items.map(cell).join('')}</div>` : '';
      }).join('')}
    </div>`;
}

function renderTextFields() {
  return `
    <textarea class="ie-textarea" rows="2" data-act="text-draft" spellcheck="false"
              placeholder="Escribe aquí">${escape(view.textDraft)}</textarea>
    <div class="ie-fonts">
      ${FONTS.map((f) => `
        <button class="ie-font ${view.textFont === f.css ? 'on' : ''}" data-act="text-font" data-font="${f.css}"
                style="font-family:'${f.css}'">${f.label}</button>`).join('')}
    </div>
    <div class="layer-toggle ie-seg">
      <button class="toggle-btn ${view.textBold ? '' : 'active'}" data-act="text-bold" data-bold="0">Normal</button>
      <button class="toggle-btn ${view.textBold ? 'active' : ''}" data-act="text-bold" data-bold="1"><b>Negrita</b></button>
    </div>`;
}

function renderTextTab() {
  return `
    ${renderTextFields()}
    <p class="ie-note">El texto aparece en la pantalla mientras escribes. Intro empieza otra línea, hasta cuatro.</p>`;
}

function renderImageTab() {
  return `
    <button class="ie-dropzone" data-act="import">
      ${icon('photo', 34)}
      <strong>Elegir una imagen…</strong>
      <span>PNG, JPG, SVG o WebP</span>
    </button>
    <p class="ie-note">También puedes arrastrarla hasta el lienzo o pegarla con <kbd>Ctrl</kbd> <kbd>V</kbd>.
      Antes de fijarla podrás moverla, cambiarle el tamaño y ajustar cómo pasa a blanco y negro.</p>`;
}

// Lo que toca el hueco del teclado sin pasar por el lienzo, al pie del
// inspector y separado de lo demás: son las dos únicas acciones que no se
// deshacen con Ctrl+Z.
function renderDeviceFooter() {
  return `
    <div class="ie-footer">
      <h3 class="ie-section">En el teclado</h3>
      <div class="ie-list">
        <button class="ie-row" data-act="load-current">
          ${icon('download', 17)}<span>Recuperar el icono guardado</span>
        </button>
        <button class="ie-row is-danger" data-act="reset-slot">
          ${icon('trash', 17)}<span>Quitar el icono</span>
        </button>
      </div>
    </div>`;
}

// --- Inspector: colocar -----------------------------------------------------

function renderPlacingPanel() {
  const xf = view.layerXf;
  const src = view.source;
  const isText = src.kind === 'text';
  const isPhoto = src.kind === 'image' && !src.crisp && !src.cut;
  // Una selección recortada ya es de 1 bit: el umbral no le hace nada.
  const hasThreshold = !src.cut;
  const seg = (act, attr, value, current, label, title = '') => `
    <button class="toggle-btn ${current === value ? 'active' : ''}" data-act="${act}" data-${attr}="${value}"
            ${title ? `title="${title}"` : ''}>${label}</button>`;

  return `
    <div class="ie-placing-head">
      <h2>${isText ? 'Texto' : escape(src.label || 'Imagen')}</h2>
      <span>Ajusta cómo queda antes de fijarlo en el lienzo</span>
    </div>

    <div class="ie-scroll">
      ${isText ? `<div class="ie-group">${renderTextFields()}</div>` : ''}

      <div class="ie-group">
        <div class="ie-row-head"><span>Tamaño</span><b id="xf-size-val"></b></div>
        <div class="ie-stepper">
          <button class="ie-btn small" data-act="size-step" data-d="-1" title="1 px menos" aria-label="1 px menos">${icon('minus', 14)}</button>
          <input type="range" class="premium-slider" id="xf-size" data-act="xf-size" aria-label="Tamaño"
                 min="${SIZE_MIN_PX}" max="${SIZE_MAX_PX}" step="1" value="${Math.round(layerHeightPx())}">
          <button class="ie-btn small" data-act="size-step" data-d="1" title="1 px más" aria-label="1 px más">${icon('plus', 14)}</button>
        </div>

        <div class="ie-row-head"><span>Posición</span><b id="xf-pos-val"></b></div>
        <div class="ie-position">
          <div class="ie-dpad" aria-label="Mover 1 px">
            <button data-act="nudge" data-dx="0"  data-dy="-1" title="Arriba (↑)" aria-label="Arriba">${icon('up', 15)}</button>
            <button data-act="nudge" data-dx="-1" data-dy="0"  title="Izquierda (←)" aria-label="Izquierda">${icon('back', 15)}</button>
            <button data-act="center" class="ie-dpad-center" title="Centrar" aria-label="Centrar"></button>
            <button data-act="nudge" data-dx="1"  data-dy="0"  title="Derecha (→)" aria-label="Derecha">${icon('chevron', 15)}</button>
            <button data-act="nudge" data-dx="0"  data-dy="1"  title="Abajo (↓)" aria-label="Abajo">${icon('down', 15)}</button>
          </div>
          <div class="ie-position-btns">
            <button class="secondary-btn" data-act="center">Centrar</button>
            <button class="secondary-btn" data-act="fit-layer" title="Lo más grande que cabe, centrado">Encajar</button>
            <span class="ie-hint"><kbd>⇧</kbd> + flechas: 5 px</span>
          </div>
        </div>
      </div>

      ${hasThreshold ? `
        <div class="ie-group">
          <div class="ie-row-head"><span>Grosor del trazo</span></div>
          <input type="range" class="premium-slider ie-threshold" data-act="xf-threshold" aria-label="Umbral de blanco y negro"
                 min="8" max="248" step="1" value="${xf.threshold}">
          <div class="slider-row"><span>Más grueso</span><span>Más fino</span></div>

          ${isPhoto ? `
            <div class="ie-row-head"><span>Suavizado</span></div>
            <div class="layer-toggle ie-seg">
              ${[0, 1, 2, 3].map((v) => seg('xf-blur', 'v', v, xf.blur, v ? String(v) : 'No')).join('')}
            </div>
            <div class="ie-switch-row">
              <span>Difuminado<small>Simula grises con puntos. Para fotos.</small></span>
              <button class="switch ${xf.dither ? 'on' : ''}" data-act="xf-dither" role="switch"
                      aria-checked="${xf.dither}" aria-label="Difuminado"><span class="switch-knob"></span></button>
            </div>` : ''}
        </div>` : ''}

      <div class="ie-group">
        <div class="ie-row-head"><span>Invertir</span></div>
        <div class="layer-toggle ie-seg">
          ${seg('xf-invert', 'mode', 'none', xf.invert, 'No')}
          ${seg('xf-invert', 'mode', 'colors', xf.invert, 'Trazo', 'Voltea el color de lo dibujado y respeta lo transparente')}
          ${seg('xf-invert', 'mode', 'box', xf.invert, 'Recuadro', 'Fondo encendido y contenido apagado, en el recuadro de la capa')}
        </div>
        <div class="ie-row-head"><span>Al fijar</span></div>
        <div class="layer-toggle ie-seg">
          ${seg('layer-mode', 'mode', 'merge', xf.mode, 'Sobre lo dibujado')}
          ${seg('layer-mode', 'mode', 'replace', xf.mode, 'Sustituir todo')}
        </div>
      </div>
    </div>

    <div class="ie-placing-actions">
      <button class="secondary-btn" data-act="cancel" title="Esc">Cancelar</button>
      <button class="primary-btn" data-act="apply" title="Intro">Fijar</button>
    </div>`;
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
