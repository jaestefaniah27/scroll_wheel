// Editor de iconos OLED (72x40, 1 bit).
//
// Dibuja sobre un framebuffer con el mismo direccionamiento que usa el firmware
// y lo sube troceado con OLED_CHUNK. El hueco es (pantalla-1) + 10 si la capa
// es SUPER, exactamente el índice que consulta refresh_single_screen().
//
// La selección de destino se hace sobre una réplica del teclado en lugar de una
// lista de números: así se ve de un vistazo qué icono y qué atajo tiene cada
// tecla antes de tocarla.

import * as device from '../device.js';
import { state, markDirty, subscribe, labelSlot, keymapSlot, KEY_TO_SCREEN } from '../store.js';
import { describeAction } from '../hid-keys.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';
import * as fb from '../oled-fb.js';

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

  // Capa flotante en colocación (imagen importada o texto generado)
  source: null,
  layerXf: null,   // { x, y, scale, threshold, blur, dither, invert, mode }
  dragging: null,  // { kind:'move'|'resize', ... }

  // Vistas previas de las teclas leídas del firmware: clave "perfil:hueco"
  previews: new Map(),
  loadingPreviews: false,
  previewsStale: false,

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

  device.on('connected', () => { view.previews.clear(); loadPreviews(); });

  let lastProfileCount = 0;
  subscribe(() => {
    if (state.profiles.length !== lastProfileCount) {
      lastProfileCount = state.profiles.length;
      loadPreviews();
      render();
    }
  });
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

function previewKey(profile, s) {
  return `${profile}:${s}`;
}

// Descarga los bitmaps que el teclado tiene guardados para el perfil actual.
// Solo se piden los huecos marcados en la máscara, que llega con GET_PROFILE.
async function loadPreviews() {
  if (!state.connected) return;

  // Si se cambia de perfil mientras hay una descarga en curso, se reinicia al
  // terminar: si no, el perfil nuevo se quedaría sin previsualizaciones.
  if (view.loadingPreviews) { view.previewsStale = true; return; }

  view.loadingPreviews = true;
  view.previewsStale = false;
  renderKeyGrid();

  try {
    const profileAtStart = view.profile;
    const prof = state.profiles[profileAtStart];
    if (!prof) return;

    for (let s = 0; s < 20; s++) {
      if (view.profile !== profileAtStart) { view.previewsStale = true; break; }
      const cacheKey = previewKey(profileAtStart, s);
      if (view.previews.has(cacheKey)) continue;
      if (!(prof.oledMask & (1 << s))) { view.previews.set(cacheKey, null); continue; }
      view.previews.set(cacheKey, await device.getOled(profileAtStart, s));
    }
  } finally {
    view.loadingPreviews = false;
    renderKeyGrid();
    if (view.previewsStale) loadPreviews();
  }
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
  else if (act === 'key')      { pickKey(Number(el.dataset.key)); }
  else if (act === 'layer')    { view.layer = el.dataset.layer; cancelLayer(false); render(); }
  else if (act === 'profile')  { view.profile = Number(el.dataset.idx); cancelLayer(false); loadPreviews(); render(); }
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

function pickKey(index) {
  if (!screenOf(index)) return; // teclas 10 y 12 no tienen pantalla
  cancelLayer(false);
  view.key = index;

  // Cargamos lo que el teclado tenga en ese hueco para poder editarlo encima.
  const cached = view.previews.get(previewKey(view.profile, slot()));
  view.buffer = cached ? Uint8Array.from(cached) : fb.createFramebuffer();
  view.undoStack = [];
  render();
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
      const box = fb.layerBounds(view.source, view.layerXf);
      const nearRight  = Math.abs(p.x - (box.x + box.width)) <= HANDLE_TOLERANCE;
      const nearBottom = Math.abs(p.y - (box.y + box.height)) <= HANDLE_TOLERANCE;

      view.dragging = (nearRight && nearBottom)
        ? { kind: 'resize', originX: box.x, originY: box.y, startH: box.height, startW: box.width }
        : { kind: 'move', grabX: p.x - box.x, grabY: p.y - box.y };
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

    if (!view.drawing) return;
    const px = Math.floor(p.x), py = Math.floor(p.y);
    if (view.lastPos) fb.drawLine(view.buffer, view.lastPos.x, view.lastPos.y, px, py, view.drawValue);
    view.lastPos = { x: px, y: py };
    repaint();
  });

  const stop = () => { view.drawing = false; view.lastPos = null; view.dragging = null; };
  canvas.addEventListener('pointerup', stop);
  canvas.addEventListener('pointercancel', stop);
  canvas.addEventListener('contextmenu', (ev) => ev.preventDefault());

  canvas.style.cursor = view.layerXf ? 'move' : 'crosshair';
}

// --- Capa flotante ----------------------------------------------------------

function defaultTransform(source) {
  const scale = fb.fitScale(source);
  const { width, height } = fb.measureSource(source);
  return {
    x: (fb.OLED_W - width * scale) / 2,
    y: (fb.OLED_H - height * scale) / 2,
    scale,
    threshold: 128,
    blur: source.kind === 'image' ? 1 : 0,
    // El difuminado solo tiene sentido en fotos; en iconos de línea es lo que
    // produce esos bordes mordidos, así que arranca desactivado.
    dither: false,
    invert: 'none',
    mode: 'replace',
  };
}

// El tamaño se controla en píxeles de alto de la pantalla, no en porcentaje del
// original. Un porcentaje del tamaño natural tiene un rango útil distinto según
// la fuente —un PNG de 800 px encaja al 5 % y un texto de 16 px necesita 170 %—
// así que la barra quedaba casi toda inservible. En píxeles significa lo mismo
// siempre, y el tope de SIZE_MAX_PX limita la capa a tres veces la pantalla.
const SIZE_MIN_PX = 2;
const SIZE_MAX_PX = fb.OLED_H * 3;

function naturalHeight() {
  return Math.max(1, fb.measureSource(view.source).height);
}

function layerHeightPx() {
  return view.layerXf.scale * naturalHeight();
}

function setLayerHeight(px, syncSlider = true) {
  if (!view.layerXf) return;
  const clamped = Math.max(SIZE_MIN_PX, Math.min(SIZE_MAX_PX, px));
  view.layerXf.scale = clamped / naturalHeight();
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

function centerLayer() {
  const box = fb.layerBounds(view.source, view.layerXf);
  view.layerXf.x = Math.round((fb.OLED_W - box.width) / 2);
  view.layerXf.y = Math.round((fb.OLED_H - box.height) / 2);
  repaint();
  updateXfLabels();
}

function fitLayer() {
  view.layerXf.scale = fb.fitScale(view.source);
  syncSizeSlider();
  centerLayer();
}

async function importImage(file) {
  try {
    view.source = await fb.loadImageSource(file);
    view.layerXf = defaultTransform(view.source);
    render();
  } catch (err) {
    toast(`No se pudo leer la imagen: ${err.message}`, 'error');
  }
}

function makeTextLayer() {
  if (!view.textDraft.trim()) { toast('Escribe algo primero', 'error'); return; }
  view.source = fb.makeTextSource(view.textDraft, {
    fontSize: view.textSize, bold: view.textBold, font: view.textFont,
  });
  view.layerXf = defaultTransform(view.source);
  render();
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
  if (!view.layerXf) return;
  view.source = null;
  view.layerXf = null;
  if (rerender) render();
}

// --- Historial y pintado ----------------------------------------------------

function pushUndo() {
  view.undoStack.push(view.buffer.slice());
  if (view.undoStack.length > 30) view.undoStack.shift();
}

function undo() {
  const prev = view.undoStack.pop();
  if (!prev) return;
  view.buffer = prev;
  repaint();
}

// Framebuffer que se muestra: base, o base + capa flotante si hay una.
function displayBuffer() {
  if (!view.layerXf) return view.buffer;
  const raster = fb.rasterizeLayer(view.source, view.layerXf);
  return fb.compose(view.buffer, raster, view.layerXf.mode);
}

function repaint() {
  const shown = displayBuffer();
  const outline = view.layerXf ? fb.layerBounds(view.source, view.layerXf) : null;

  const canvas = document.getElementById('oled-canvas');
  if (canvas) fb.renderToCanvas(shown, canvas, { zoom: view.zoom, outline });

  const preview = document.getElementById('oled-preview');
  if (preview) fb.renderToCanvas(shown, preview, { zoom: 2, grid: false });
}

function updateXfLabels() {
  if (!view.layerXf) return;
  const set = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; };
  const box = fb.layerBounds(view.source, view.layerXf);

  set('xf-size-val', `${Math.round(box.width)} × ${Math.round(box.height)} px`);
  set('xf-threshold-val', view.layerXf.threshold);
  set('xf-blur-val', view.layerXf.blur);
  set('xf-pos-val', `x ${Math.round(view.layerXf.x)} · y ${Math.round(view.layerXf.y)}`);
}

// --- Envío al teclado -------------------------------------------------------

async function upload() {
  if (!state.connected) { toast('Teclado no conectado', 'error'); return; }
  if (slot() < 0) { toast('Esa tecla no tiene pantalla', 'error'); return; }

  const btn = document.getElementById('btn-oled-upload');
  btn.disabled = true;
  try {
    await device.uploadOled(view.profile, slot(), view.buffer);
    view.previews.set(previewKey(view.profile, slot()), Uint8Array.from(view.buffer));
    const prof = state.profiles[view.profile];
    if (prof) prof.oledMask |= (1 << slot());
    markDirty();
    renderKeyGrid();
    toast(`Icono enviado a la tecla ${view.key + 1}`);
  } catch (err) {
    toast(`Error al enviar: ${err.message}`, 'error');
  } finally {
    btn.disabled = false;
  }
}

async function resetSlot() {
  try {
    await device.clearOled(view.profile, slot());
    view.previews.set(previewKey(view.profile, slot()), null);
    const prof = state.profiles[view.profile];
    if (prof) prof.oledMask &= ~(1 << slot());
    fb.clear(view.buffer);
    markDirty();
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
  view.previews.set(previewKey(view.profile, slot()), bytes);
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
    <header class="view-header compact">
      <h1>Iconos OLED</h1>
      <p>Elige una tecla y dibuja, importa una imagen o genera texto para su pantalla de 72×40</p>
    </header>

    <div class="oled-grid">
      <div class="glass-panel oled-canvas-card">
        <div class="oled-toolbar" id="oled-toolbar">${renderToolbarInner()}</div>
        <div class="oled-canvas-wrap" id="oled-canvas-wrap">
          <canvas id="oled-canvas"></canvas>
        </div>
        <p class="oled-hint">${renderHint()}</p>
      </div>

      <div class="oled-side">
        ${renderTargetPanel()}
        ${view.layerXf ? renderLayerPanel() : renderSourcePanel()}
        ${renderSendPanel()}
      </div>
    </div>`;

  bindCanvas();
  repaint();
  updateXfLabels();
  // Las miniaturas de las teclas son <canvas>: hay que pintarlas después de
  // insertar el HTML, no se pueden serializar dentro de la plantilla.
  paintKeyThumbs();
  updateGridStatus();
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

// Réplica del teclado: misma disposición que el dashboard, pero cada tecla
// muestra su icono real y el atajo que ejecuta.
function renderKeyGridInner() {
  const prof = state.profiles[view.profile];
  let html = '';

  for (let i = 0; i < 12; i++) {
    const screen = screenOf(i);
    const s = slotOf(i);
    const action = prof ? (prof.keys[keymapSlot(i, view.layer)] || { modifier: 0, keycode: 0 }) : null;
    const label = prof && s >= 0 ? (prof.labels[labelSlot(i, view.layer)] || '') : '';
    const bmp = s >= 0 ? view.previews.get(previewKey(view.profile, s)) : null;

    if (!screen) {
      html += `
        <div class="okey no-screen">
          <span class="okey-num">T${i + 1}</span>
          <span class="okey-role">${i === 9 ? 'SUPER' : 'MENÚ'}</span>
        </div>`;
      continue;
    }

    html += `
      <button class="okey ${view.key === i ? 'selected' : ''} ${bmp ? 'has-icon' : ''}" data-act="key" data-key="${i}">
        <span class="okey-num">T${i + 1}<em>P${screen}</em></span>
        <span class="okey-screen">
          ${bmp ? `<canvas class="okey-canvas" data-bmp="${previewKey(view.profile, s)}"></canvas>`
                : `<span class="okey-text">${escape(label || '—')}</span>`}
        </span>
        <span class="okey-action">${escape(action ? describeAction(action.modifier, action.keycode) : '')}</span>
      </button>`;
  }
  return html;
}

// Los canvas se pintan tras insertar el HTML: no se pueden serializar.
function paintKeyThumbs() {
  document.querySelectorAll('.okey-canvas').forEach((canvas) => {
    const bmp = view.previews.get(canvas.dataset.bmp);
    if (bmp) fb.renderToCanvas(bmp, canvas, { zoom: 1, grid: false });
  });
}

function updateGridStatus() {
  const status = document.getElementById('oled-grid-status');
  if (!status) return;
  status.textContent = view.loadingPreviews
    ? 'Leyendo iconos del teclado…'
    : `Editando la tecla ${view.key + 1} · pantalla ${screenOf(view.key)} · capa ${view.layer === 'super' ? 'SUPER' : 'NORMAL'}`;
}

function renderKeyGrid() {
  const host = document.getElementById('oled-key-grid');
  if (!host) return;
  host.innerHTML = renderKeyGridInner();
  paintKeyThumbs();
  updateGridStatus();
}

function renderTargetPanel() {
  const names = state.profiles.length ? state.profiles.map((p) => p.name) : ['P1', 'P2', 'P3', 'P4'];

  return `
    <div class="glass-panel oled-card">
      <div class="card-header">${icon('profiles', 20)}<h2>Destino</h2></div>

      <div class="target-row">
        <div class="chip-row">
          ${names.map((n, i) => `
            <button class="chip ${view.profile === i ? 'on' : ''}" data-act="profile" data-idx="${i}">${escape(n)}</button>`).join('')}
        </div>
        <div class="layer-toggle">
          <button class="toggle-btn ${view.layer === 'normal' ? 'active' : ''}" data-act="layer" data-layer="normal">NORMAL</button>
          <button class="toggle-btn ${view.layer === 'super' ? 'active' : ''}" data-act="layer" data-layer="super">SUPER</button>
        </div>
      </div>

      <div class="okey-grid" id="oled-key-grid">${renderKeyGridInner()}</div>
      <p class="grid-status" id="oled-grid-status"></p>
    </div>`;
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

function renderSendPanel() {
  const has = slot() >= 0;
  return `
    <div class="glass-panel oled-card">
      <div class="card-header">${icon('oled', 20)}<h2>Pantalla</h2></div>
      <div class="oled-preview-box">
        <span class="field-label">Tamaño real</span>
        <canvas id="oled-preview" width="${fb.OLED_W * 2}" height="${fb.OLED_H * 2}"></canvas>
      </div>
      <button class="primary-btn full" id="btn-oled-upload" data-act="upload" ${has ? '' : 'disabled'}>
        ${icon('download', 16)} Enviar a la tecla ${view.key + 1}
      </button>
      <div class="row-inline mt-4">
        <button class="secondary-btn" data-act="load-current">${icon('refresh', 16)} Releer del teclado</button>
        <button class="secondary-btn danger" data-act="reset-slot">${icon('trash', 16)} Quitar icono</button>
      </div>
      <p class="setting-desc">Recuerda pulsar <strong>Guardar en Flash</strong> para que sobreviva a la desconexión.</p>
    </div>`;
}

function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
