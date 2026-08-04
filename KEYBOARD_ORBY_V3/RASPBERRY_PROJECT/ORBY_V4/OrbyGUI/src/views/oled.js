// Editor de iconos OLED (72x40, 1 bit).
//
// Dibuja sobre un framebuffer con el mismo direccionamiento que usa el firmware
// y lo sube troceado con OLED_CHUNK. El hueco es (pantalla-1) + 10 si la capa
// es SUPER, exactamente el índice que consulta refresh_single_screen().
//
// No es una vista a la que se pueda entrar por la barra lateral: se abre
// siempre desde la tecla que se esté configurando en Perfiles (o en una macro)
// y edita ESE icono y ninguno más. Por eso no hay selector de perfil, de capa
// ni réplica del teclado; el destino ya viene decidido en openTarget(). Se sale
// por los dos botones fijos de arriba: guardar (manda el icono a la tecla y
// vuelve) o salir sin guardar.

import * as device from '../device.js';
import { state, markDirty, subscribe, KEY_TO_SCREEN, hasPages, pageCountOf } from '../store.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';
import { goTo } from '../nav.js';
import * as fb from '../oled-fb.js';
import * as cache from '../oled-cache.js';

const ZOOM_MIN = 3;
const ZOOM_MAX = 28;
const ZOOM_DEFAULT = 9;

// Fuentes presentes en cualquier instalación de Windows. Nada de webfonts: la
// app tiene que funcionar sin conexión.
const FONTS = [
  { css: 'Segoe UI',        label: 'Segoe UI' },
  { css: 'Segoe UI Black',  label: 'Segoe UI Black' },
  { css: 'Arial',           label: 'Arial' },
  { css: 'Arial Black',     label: 'Arial Black' },
  { css: 'Impact',          label: 'Impact (estrecha)' },
  { css: 'Tahoma',          label: 'Tahoma' },
  { css: 'Verdana',         label: 'Verdana' },
  { css: 'Consolas',        label: 'Consolas (mono)' },
  { css: 'Courier New',     label: 'Courier New (mono)' },
  { css: 'Georgia',         label: 'Georgia' },
  { css: 'Times New Roman', label: 'Times New Roman' },
];

const TOOLS = [
  { id: 'pencil', icon: 'pencil', label: 'Lápiz' },
  { id: 'eraser', icon: 'eraser', label: 'Borrador' },
  { id: 'fill',   icon: 'fill',   label: 'Relleno' },
  { id: 'select', icon: 'select', label: 'Selección (mover/redimensionar)' },
];

const view = {
  profile: 0,
  key: 0,          // índice de tecla 0..11
  layer: 'normal',
  tool: 'pencil',
  zoom: ZOOM_DEFAULT,
  buffer: fb.createFramebuffer(),
  drawing: false,
  drawValue: 1,
  lastPos: null,
  undoStack: [],
  dirty: false,    // se ha tocado el lienzo desde que se abrió esta tecla

  // Capa flotante en colocación (imagen importada, texto generado, o una
  // selección recortada del propio lienzo con la herramienta "select")
  source: null,
  layerXf: null,   // { x, y, scale, threshold, blur, dither, invert, mode }
  dragging: null,  // { kind:'move'|'resize', ... }
  selecting: null, // { x0, y0, x1, y1 } mientras se arrastra un recuadro de selección
  outline: null,   // último recuadro pintado sobre el lienzo (ver repaint)

  textDraft: '',
  textFont: 'Segoe UI',
  textSize: 16,
  textBold: true,
};

export function init() {
  render();
  const root = document.getElementById('view-oled');
  root.addEventListener('click', onClick);
  root.addEventListener('change', onChange);
  root.addEventListener('input', onInput);
  window.addEventListener('keydown', onKeyDown);

  device.on('connected', () => { cache.clearAll(); loadPreviews(); });

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

// Abre el editor sobre una tecla concreta. Es la ÚNICA forma de entrar aquí:
// lo llama el botón "Editar icono" del editor de perfiles con el perfil, la
// tecla y la capa que se estaban configurando.
export function openTarget({ profile, key, layer } = {}) {
  if (Number.isInteger(profile) && profile < state.profiles.length) view.profile = profile;
  if (layer === 'normal' || layer === 'super') view.layer = layer;
  cancelLayer(false);

  if (Number.isInteger(key) && screenOf(key)) view.key = key;

  const cached = cache.get(view.profile, slot());
  view.buffer = cached ? Uint8Array.from(cached) : fb.createFramebuffer();
  view.undoStack = [];
  view.dirty = false;
  render();
  loadPreviews();
}

// Vuelve al editor de perfiles, que es de donde siempre se viene. La tecla
// sigue seleccionada allí, así que se cae justo en el mismo inspector.
function backToProfiles() {
  cancelLayer(false);
  goTo('view-profiles');
}

// Salir sin guardar: lo dibujado se descarta y la tecla se queda con el icono
// que ya tuviera el teclado. Se pregunta solo si de verdad hay algo que perder.
function exitWithoutSaving() {
  if (view.dirty && !confirm('Se perderá lo que hayas dibujado en este icono.\n\n¿Salir sin guardar?')) return;
  backToProfiles();
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
  return slotOf(view.key);
}

// Los bitmaps guardados se comparten con el editor de perfiles a través de
// oled-cache.js: leerlos por el CDC es lento y no tiene sentido hacerlo dos veces.
function loadPreviews() {
  cache.loadProfile(view.profile);
}

// --- Eventos ----------------------------------------------------------------

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;
  const act = el.dataset.act;

  if (act === 'tool')          { view.tool = el.dataset.tool; renderToolbar(); }
  else if (act === 'zoom-in')  { setZoom(view.zoom + zoomStep(view.zoom)); }
  else if (act === 'zoom-out') { setZoom(view.zoom - zoomStep(view.zoom - 1)); }
  else if (act === 'zoom-fit') { setZoom(fitZoom()); }
  else if (act === 'exit')     { exitWithoutSaving(); }
  else if (act === 'clear')    { pushUndo(); fb.clear(view.buffer); repaint(); }
  else if (act === 'invert')   { pushUndo(); fb.invert(view.buffer); repaint(); }
  else if (act === 'frame')    { pushUndo(); fb.drawFrame(view.buffer); repaint(); }
  else if (act === 'undo')     { undo(); }
  else if (act === 'make-text'){ makeTextLayer(); }
  else if (act === 'import')   { document.getElementById('oled-file').click(); }
  else if (act === 'apply')    { applyLayer(); }
  else if (act === 'cancel')   { cancelLayer(true); }
  else if (act === 'center')   { centerLayer(); }
  else if (act === 'fit-layer'){ fitLayer(); }
  else if (act === 'nudge')    { nudgeLayer(Number(el.dataset.dx), Number(el.dataset.dy)); }
  else if (act === 'size-step'){ setLayerHeight(layerHeightPx() + Number(el.dataset.d)); }
  else if (act === 'xf-invert') {
    view.layerXf.invert = el.dataset.mode;
    el.parentElement.querySelectorAll('[data-act="xf-invert"]')
      .forEach((b) => b.classList.toggle('on', b.dataset.mode === el.dataset.mode));
    repaint();
  }
  else if (act === 'upload')   { upload(); }
  else if (act === 'reset-slot') { resetSlot(); }
  else if (act === 'load-current') { loadFromDevice(); }
}

function onChange(e) {
  if (e.target.id === 'oled-file' && e.target.files?.[0]) {
    importImage(e.target.files[0]);
    e.target.value = '';
  } else if (e.target.dataset.act === 'text-font') {
    view.textFont = e.target.value;
    if (view.source?.kind === 'text') refreshTextSource();
  } else if (e.target.dataset.act === 'layer-mode') {
    view.layerXf.mode = e.target.value;
    repaint();
  }
}

function onInput(e) {
  const act = e.target.dataset.act;
  if (!act) return;

  if (act === 'text-draft')      { view.textDraft = e.target.value; }
  else if (act === 'text-size')  { view.textSize = Number(e.target.value); if (view.source?.kind === 'text') refreshTextSource(); }
  else if (act === 'text-bold')  { view.textBold = e.target.checked; if (view.source?.kind === 'text') refreshTextSource(); }
  else if (!view.layerXf) return;
  else if (act === 'xf-size')      { setLayerHeight(Number(e.target.value), false); }
  else if (act === 'xf-threshold') { view.layerXf.threshold = Number(e.target.value); repaint(); updateXfLabels(); }
  else if (act === 'xf-blur')      { view.layerXf.blur = Number(e.target.value); repaint(); updateXfLabels(); }
  else if (act === 'xf-dither')    { view.layerXf.dither = e.target.checked; repaint(); }
}

function onKeyDown(e) {
  if (!view.layerXf) return;
  const inField = ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName);

  if (e.key === 'Escape') { cancelLayer(true); return; }
  if (e.key === 'Enter' && !inField) { e.preventDefault(); applyLayer(); return; }
  if (inField) return;

  const step = e.shiftKey ? 5 : 1;
  const nudge = { ArrowLeft: [-step, 0], ArrowRight: [step, 0], ArrowUp: [0, -step], ArrowDown: [0, step] }[e.key];
  if (!nudge) return;
  e.preventDefault();
  view.layerXf.x += nudge[0];
  view.layerXf.y += nudge[1];
  repaint();
  updateXfLabels();
}

// --- Lienzo -----------------------------------------------------------------

function zoomStep(level) {
  return level < 8 ? 1 : (level < 16 ? 2 : 4);
}

function fitZoom() {
  const wrap = document.getElementById('oled-canvas-wrap');
  if (!wrap) return ZOOM_DEFAULT;
  const styles = getComputedStyle(wrap);
  const usableW = wrap.clientWidth - parseFloat(styles.paddingLeft) - parseFloat(styles.paddingRight);
  const usableH = wrap.clientHeight - parseFloat(styles.paddingTop) - parseFloat(styles.paddingBottom);
  return clampZoom(Math.min(usableW / fb.OLED_W, usableH / fb.OLED_H));
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
      // El tirador se busca donde el usuario lo ve (el recuadro pintado, que se
      // ciñe a los píxeles), pero el arrastre se calcula sobre la caja de la
      // capa: es la única que escala de forma lineal y no se recorta al salirse
      // de la pantalla, así que la capa no pega saltos al agarrarla.
      const box = fb.layerBounds(view.source, view.layerXf);
      const shown = view.outline || box;
      const nearRight  = Math.abs(p.x - (shown.x + shown.width)) <= HANDLE_TOLERANCE;
      const nearBottom = Math.abs(p.y - (shown.y + shown.height)) <= HANDLE_TOLERANCE;

      view.dragging = (nearRight && nearBottom)
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
  canvas.addEventListener('contextmenu', (ev) => ev.preventDefault());

  canvas.style.cursor = view.layerXf ? 'move' : 'crosshair';
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
    blur: source.kind === 'image' ? 1 : 0,
    // El difuminado solo tiene sentido en fotos; en iconos de línea es lo que
    // produce esos bordes mordidos, así que arranca desactivado.
    dither: false,
    invert: 'none',
    // El texto se añade por encima de lo que ya hubiera dibujado (se puede ir
    // metiendo texto varias veces en el mismo icono); una imagen importada
    // sustituye el lienzo entero, que es lo esperable al pegar una foto.
    mode: source.kind === 'text' ? 'merge' : 'replace',
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
  if (slider) slider.value = Math.round(layerHeightPx());
}

function nudgeLayer(dx, dy) {
  if (!view.layerXf) return;
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
function beginLayer(source) {
  view.source = source;
  view.layerXf = defaultTransform(source);
  placeCentered();
  render();
}

async function importImage(file) {
  try {
    beginLayer(await fb.loadImageSource(file));
  } catch (err) {
    toast(`No se pudo leer la imagen: ${err.message}`, 'error');
  }
}

function makeTextLayer() {
  if (!view.textDraft.trim()) { toast('Escribe algo primero', 'error'); return; }
  beginLayer(fb.makeTextSource(view.textDraft, {
    fontSize: view.textSize, bold: view.textBold, font: view.textFont,
  }));
}

// Rehace la fuente del texto conservando la posición ya elegida.
function refreshTextSource() {
  const keep = { ...view.layerXf };
  view.source = fb.makeTextSource(view.textDraft, {
    fontSize: view.textSize, bold: view.textBold, font: view.textFont,
  });
  view.layerXf = keep;
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

  view.source = { kind: 'image', bitmap: canvas, naturalWidth: width, naturalHeight: height, cut: true };
  view.layerXf = {
    x: left, y: top, scale: 1, threshold: 128, blur: 0, dither: false, invert: 'none', mode: 'merge',
  };
  render();
}

function applyLayer() {
  if (!view.layerXf) return;
  pushUndo();
  const raster = fb.rasterizeLayer(view.source, view.layerXf);
  view.buffer = fb.compose(view.buffer, raster, view.layerXf.mode);
  view.source = null;
  view.layerXf = null;
  render();
}

function cancelLayer(rerender = true) {
  view.selecting = null;
  if (!view.layerXf) return;
  // Una selección cortada ya había quitado sus píxeles del lienzo (ver
  // finishSelection): al cancelar hay que devolvérselos, no solo descartar
  // la capa flotante.
  if (view.source?.cut) view.buffer = view.undoStack.pop() || view.buffer;
  view.source = null;
  view.layerXf = null;
  if (rerender) render();
}

// --- Historial y pintado ----------------------------------------------------

// Todo lo que modifica el lienzo pasa por aquí antes de tocarlo, así que es el
// sitio natural para marcar que hay algo que se perdería al salir sin guardar.
function pushUndo() {
  view.undoStack.push(view.buffer.slice());
  if (view.undoStack.length > 30) view.undoStack.shift();
  view.dirty = true;
}

function undo() {
  const prev = view.undoStack.pop();
  if (!prev) return;
  view.buffer = prev;
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
}

function updateXfLabels() {
  if (!view.layerXf) return;
  const set = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; };
  // Se enseña la medida real de lo dibujado, que es lo que marca el recuadro.
  // La barra sigue moviéndose sobre la caja de la capa (ver setLayerHeight):
  // necesita una escala lineal, y la tinta rasterizada da saltos de un píxel.
  const box = view.outline || inkBox();

  set('xf-size-val', `${Math.round(box.width)} × ${Math.round(box.height)} px`);
  set('xf-threshold-val', view.layerXf.threshold);
  set('xf-blur-val', view.layerXf.blur);
  // La posición que interesa es la de lo dibujado, igual que el tamaño.
  set('xf-pos-val', `x ${Math.round(box.x)} · y ${Math.round(box.y)}`);
}

// --- Envío al teclado -------------------------------------------------------

// Guardar = mandar el icono a la tecla y volver a la configuración de esa
// tecla, que es de donde se entró. Si falla el envío no se sale: si no, el
// dibujo se perdería sin que la tecla se hubiera enterado.
async function upload() {
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (slot() < 0) { toast('Esa tecla no tiene pantalla', 'error'); return; }

  const btn = document.getElementById('btn-oled-upload');
  if (btn) btn.disabled = true;
  try {
    await device.uploadOled(view.profile, slot(), view.buffer);
    cache.set(view.profile, slot(), Uint8Array.from(view.buffer));
    const prof = state.profiles[view.profile];
    if (prof) prof.oledMask |= (1 << slot());
    markDirty();
    view.dirty = false;
    toast(`Icono enviado a la tecla ${view.key + 1}`);
    backToProfiles();
  } catch (err) {
    toast(`Error al enviar: ${err.message}`, 'error');
    if (btn) btn.disabled = false;
  }
}

async function resetSlot() {
  try {
    await device.clearOled(view.profile, slot());
    cache.set(view.profile, slot(), null);
    const prof = state.profiles[view.profile];
    if (prof) prof.oledMask &= ~(1 << slot());
    fb.clear(view.buffer);
    markDirty();
    view.dirty = false;
    render();
    toast('Icono eliminado; vuelve la etiqueta de texto');
  } catch (err) {
    toast(`Error: ${err.message}`, 'error');
  }
}

async function loadFromDevice() {
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  const bytes = await device.getOled(view.profile, slot());
  if (!bytes) { toast('Esa tecla no tiene icono guardado', 'info'); return; }
  cache.set(view.profile, slot(), bytes);
  pushUndo();
  view.buffer = Uint8Array.from(bytes);
  repaint();
  toast('Icono cargado desde el teclado');
}

// --- Render -----------------------------------------------------------------

export function render() {
  const root = document.getElementById('view-oled');
  if (!root) return;

  root.innerHTML = `
    ${renderActionBar()}

    <div class="oled-grid">
      <div class="glass-panel oled-canvas-card">
        <div class="oled-toolbar" id="oled-toolbar">${renderToolbarInner()}</div>
        <div class="oled-canvas-wrap" id="oled-canvas-wrap">
          <canvas id="oled-canvas"></canvas>
        </div>
        <p class="oled-hint">${renderHint()}</p>
      </div>

      <div class="oled-side">
        ${view.layerXf ? renderLayerPanel() : renderSourcePanel()}
        ${renderSendPanel()}
      </div>
    </div>`;

  bindCanvas();
  repaint();
  updateXfLabels();
}

// Barra fija de arriba: qué se está editando y las dos únicas salidas del
// editor. Va fuera del contenedor que hace scroll (`.oled-side`), así que no
// puede quedar tapada por mucho que se baje en el panel lateral.
function renderActionBar() {
  const prof = state.profiles[view.profile];
  const pageTxt = (hasPages() && prof && pageCountOf(prof) > 1)
    ? ` · página ${(prof.pageIdx || 0) + 1} de ${pageCountOf(prof)}` : '';
  const where = `${escape(prof?.name || `Perfil ${view.profile + 1}`)} · tecla ${view.key + 1}`
    + ` · pantalla ${screenOf(view.key) || '—'}`
    + ` · capa ${view.layer === 'super' ? 'SUPER' : 'NORMAL'}${pageTxt}`;

  return `
    <header class="oled-actionbar">
      <div class="oled-actionbar-info">
        <h1>Icono de la tecla ${view.key + 1}</h1>
        <p>${where}</p>
      </div>
      <div class="oled-actionbar-btns">
        <button class="secondary-btn" data-act="exit">${icon('close', 16)} Salir sin guardar</button>
        <button class="primary-btn" id="btn-oled-upload" data-act="upload" ${slot() >= 0 ? '' : 'disabled'}>
          ${icon('check', 16)} Enviar a la tecla
        </button>
      </div>
    </header>`;
}

function renderToolbarInner() {
  const disabled = view.layerXf ? 'disabled' : '';
  return `
    ${TOOLS.map((t) => `
      <button class="tool-btn ${view.tool === t.id ? 'on' : ''}" data-act="tool" data-tool="${t.id}" title="${t.label}" ${disabled}>
        ${icon(t.icon, 18)}
      </button>`).join('')}
    <span class="tool-sep"></span>
    <button class="tool-btn" data-act="frame"  title="Marco" ${disabled}>${icon('square', 18)}</button>
    <button class="tool-btn" data-act="invert" title="Invertir" ${disabled}>${icon('invert', 18)}</button>
    <button class="tool-btn" data-act="undo"   title="Deshacer" ${disabled}>${icon('reset', 18)}</button>
    <button class="tool-btn danger" data-act="clear" title="Vaciar" ${disabled}>${icon('trash', 18)}</button>

    <span class="zoom-group">
      <button class="tool-btn" data-act="zoom-out" title="Alejar">${icon('minus', 18)}</button>
      <span class="zoom-level" id="oled-zoom-level">${view.zoom}×</span>
      <button class="tool-btn" data-act="zoom-in" title="Acercar">${icon('plus', 18)}</button>
      <button class="tool-btn" data-act="zoom-fit" title="Ajustar a la ventana">${icon('fit', 18)}</button>
    </span>`;
}

function renderToolbar() {
  const bar = document.getElementById('oled-toolbar');
  if (bar) bar.innerHTML = renderToolbarInner();
}

function renderHint() {
  if (view.layerXf) {
    return `Arrastra para mover la capa, y tira de la esquina inferior derecha para escalarla.
            La cruceta y las <strong>flechas</strong> del teclado mueven 1 px (con Shift, 5).
            <strong>Enter</strong> fija, <strong>Esc</strong> cancela.`;
  }
  return `Clic izquierdo pinta, clic derecho borra. <strong>Ctrl + rueda</strong> hace zoom sobre el punto
          del cursor. Las líneas violetas marcan las páginas de 8 px en las que el SSD1306 direcciona la memoria.`;
}

function renderSourcePanel() {
  return `
    <div class="glass-panel oled-card">
      <div class="card-header">${icon('text', 20)}<h2>Generar contenido</h2></div>

      <label class="field">
        <span class="field-label">Texto</span>
        <textarea class="text-input" rows="2" data-act="text-draft"
                  placeholder="Enter = salto de línea">${escape(view.textDraft)}</textarea>
      </label>

      <label class="field">
        <span class="field-label">Fuente</span>
        <select class="select-input" data-act="text-font">
          ${FONTS.map((f) => `
            <option value="${f.css}" ${view.textFont === f.css ? 'selected' : ''}
                    style="font-family:'${f.css}'">${f.label}</option>`).join('')}
        </select>
      </label>

      <div class="row-inline">
        <label class="field-inline"><span>Tamaño</span>
          <input type="number" class="text-input compact" data-act="text-size"
                 value="${view.textSize}" min="6" max="40">
        </label>
        <label class="check"><input type="checkbox" data-act="text-bold" ${view.textBold ? 'checked' : ''}> Negrita</label>
        <button class="secondary-btn" data-act="make-text">${icon('text', 16)} Colocar texto</button>
      </div>

      <div class="divider"></div>

      <button class="secondary-btn full" data-act="import">${icon('upload', 16)} Importar imagen (PNG / JPG / SVG)</button>
      <input type="file" id="oled-file" accept="image/*" hidden>
      <p class="setting-desc">Podrás moverla y escalarla dentro del recuadro antes de fijarla.</p>
    </div>`;
}

function renderLayerPanel() {
  const xf = view.layerXf;
  const isText = view.source.kind === 'text';

  return `
    <div class="glass-panel oled-card is-placing">
      <div class="card-header">${icon('fit', 20)}<h2>Colocando ${isText ? 'texto' : 'imagen'}</h2></div>

      ${isText ? `
        <label class="field">
          <span class="field-label">Fuente</span>
          <select class="select-input" data-act="text-font">
            ${FONTS.map((f) => `
              <option value="${f.css}" ${view.textFont === f.css ? 'selected' : ''}
                      style="font-family:'${f.css}'">${f.label}</option>`).join('')}
          </select>
        </label>
        <div class="row-inline">
          <label class="field-inline"><span>Tamaño</span>
            <input type="number" class="text-input compact" data-act="text-size" value="${view.textSize}" min="6" max="40">
          </label>
          <label class="check"><input type="checkbox" data-act="text-bold" ${view.textBold ? 'checked' : ''}> Negrita</label>
        </div>
        <div class="divider"></div>` : ''}

      <div class="field">
        <span class="field-label">Tamaño <b id="xf-size-val"></b></span>
        <div class="size-row">
          <button class="tool-btn small" data-act="size-step" data-d="-1" title="1 px menos">${icon('minus', 14)}</button>
          <input type="range" class="premium-slider" id="xf-size" data-act="xf-size"
                 min="${SIZE_MIN_PX}" max="${SIZE_MAX_PX}" step="1" value="${Math.round(layerHeightPx())}">
          <button class="tool-btn small" data-act="size-step" data-d="1" title="1 px más">${icon('plus', 14)}</button>
        </div>
      </div>

      <div class="field">
        <span class="field-label">Posición <b id="xf-pos-val"></b></span>
        <div class="nudge-row">
          <div class="nudge-pad">
            <button class="tool-btn small" data-act="nudge" data-dx="0"  data-dy="-1" title="Arriba 1 px">↑</button>
            <button class="tool-btn small" data-act="nudge" data-dx="-1" data-dy="0"  title="Izquierda 1 px">←</button>
            <button class="tool-btn small" data-act="center" title="Centrar">${icon('fit', 14)}</button>
            <button class="tool-btn small" data-act="nudge" data-dx="1"  data-dy="0"  title="Derecha 1 px">→</button>
            <button class="tool-btn small" data-act="nudge" data-dx="0"  data-dy="1"  title="Abajo 1 px">↓</button>
          </div>
          <button class="secondary-btn" data-act="fit-layer">Encajar</button>
        </div>
      </div>

      <label class="field">
        <span class="field-label">Umbral de blanco/negro <b id="xf-threshold-val">${xf.threshold}</b></span>
        <input type="range" class="premium-slider" data-act="xf-threshold"
               min="8" max="248" step="1" value="${xf.threshold}">
      </label>

      <label class="field">
        <span class="field-label">Suavizado de bordes <b id="xf-blur-val">${xf.blur}</b></span>
        <input type="range" class="premium-slider" data-act="xf-blur"
               min="0" max="3" step="1" value="${xf.blur}">
      </label>

      <label class="check"><input type="checkbox" data-act="xf-dither" ${xf.dither ? 'checked' : ''}> Difuminado (solo para fotos)</label>

      <div class="field mt-4">
        <span class="field-label">Invertir</span>
        <div class="chip-row">
          <button class="chip ${xf.invert === 'none' ? 'on' : ''}"   data-act="xf-invert" data-mode="none">No</button>
          <button class="chip ${xf.invert === 'colors' ? 'on' : ''}" data-act="xf-invert" data-mode="colors"
                  title="Voltea el color del contenido y respeta la transparencia">Colores</button>
          <button class="chip ${xf.invert === 'box' ? 'on' : ''}"    data-act="xf-invert" data-mode="box"
                  title="Voltea el rectángulo de la capa: fondo encendido, contenido apagado">Recuadro</button>
        </div>
        <p class="setting-desc">Solo afecta a la capa; el resto del lienzo no se toca.</p>
      </div>

      <label class="field mt-4">
        <span class="field-label">Al fijar</span>
        <select class="select-input" data-act="layer-mode">
          <option value="replace" ${xf.mode === 'replace' ? 'selected' : ''}>Sustituir el lienzo</option>
          <option value="merge"   ${xf.mode === 'merge' ? 'selected' : ''}>Combinar con lo dibujado</option>
        </select>
      </label>

      <div class="layer-actions">
        <button class="primary-btn full" data-act="apply">${icon('check', 16)} Fijar en el lienzo</button>
        <button class="secondary-btn full" data-act="cancel">${icon('close', 16)} Cancelar</button>
      </div>
    </div>`;
}

// El botón de enviar vive en la barra fija de arriba; aquí queda la
// previsualización a tamaño real y las dos acciones que tocan el hueco del
// teclado sin salir del editor.
function renderSendPanel() {
  return `
    <div class="glass-panel oled-card">
      <div class="card-header">${icon('oled', 20)}<h2>Pantalla</h2></div>
      <div class="oled-preview-box">
        <span class="field-label">Tamaño real</span>
        <canvas id="oled-preview" width="${fb.OLED_W * 2}" height="${fb.OLED_H * 2}"></canvas>
      </div>
      <div class="row-inline">
        <button class="secondary-btn" data-act="load-current">${icon('refresh', 16)} Releer del teclado</button>
        <button class="secondary-btn danger" data-act="reset-slot">${icon('trash', 16)} Quitar icono</button>
      </div>
      <p class="setting-desc">Recuerda pulsar <strong>Guardar en Flash</strong> para que sobreviva a la desconexión.</p>
    </div>`;
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
