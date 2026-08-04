// Framebuffer de las pantallas SSD1306 del Orby: 72x40 px, 1 bit por píxel.
//
// Formato exacto que espera el firmware (ver draw_premium_frame en main.cpp):
//   índice = página * 72 + x      con página = y >> 3
//   bit    = 1 << (y & 7)         bit 0 = fila superior de la página

export const OLED_W = 72;
export const OLED_H = 40;
export const OLED_PAGES = OLED_H / 8;      // 5
export const OLED_BYTES = OLED_W * OLED_PAGES; // 360

export function createFramebuffer() {
  return new Uint8Array(OLED_BYTES);
}

export function getPixel(fb, x, y) {
  if (x < 0 || y < 0 || x >= OLED_W || y >= OLED_H) return 0;
  return (fb[(y >> 3) * OLED_W + x] >> (y & 7)) & 1;
}

export function setPixel(fb, x, y, value) {
  if (x < 0 || y < 0 || x >= OLED_W || y >= OLED_H) return;
  const idx = (y >> 3) * OLED_W + x;
  const mask = 1 << (y & 7);
  if (value) fb[idx] |= mask;
  else fb[idx] &= ~mask;
}

export function invert(fb) {
  for (let i = 0; i < fb.length; i++) fb[i] = ~fb[i] & 0xff;
}

export function clear(fb) {
  fb.fill(0);
}

// Relleno por difusión desde (x, y).
export function floodFill(fb, x, y, value) {
  const target = getPixel(fb, x, y);
  if (target === value) return;
  const stack = [[x, y]];
  while (stack.length) {
    const [cx, cy] = stack.pop();
    if (cx < 0 || cy < 0 || cx >= OLED_W || cy >= OLED_H) continue;
    if (getPixel(fb, cx, cy) !== target) continue;
    setPixel(fb, cx, cy, value);
    stack.push([cx + 1, cy], [cx - 1, cy], [cx, cy + 1], [cx, cy - 1]);
  }
}

export function drawLine(fb, x0, y0, x1, y1, value) {
  const dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  let err = dx + dy;
  for (;;) {
    setPixel(fb, x0, y0, value);
    if (x0 === x1 && y0 === y1) break;
    const e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

export function drawFrame(fb) {
  for (let x = 0; x < OLED_W; x++) {
    setPixel(fb, x, 0, 1);
    setPixel(fb, x, OLED_H - 1, 1);
  }
  for (let y = 0; y < OLED_H; y++) {
    setPixel(fb, 0, y, 1);
    setPixel(fb, OLED_W - 1, y, 1);
  }
}

// --- Conversión desde imagen / texto ---------------------------------------

// Desenfoque de caja separable. Aplicado antes de umbralizar redondea las
// escaleras de 1 px que deja el reescalado, que es lo que hace que un icono
// vectorial se vea "roto" al bajarlo a 72x40.
function boxBlur(gray, radius) {
  if (radius <= 0) return gray;
  const tmp = new Float32Array(gray.length);
  const out = new Float32Array(gray.length);

  for (let y = 0; y < OLED_H; y++) {
    for (let x = 0; x < OLED_W; x++) {
      let sum = 0, count = 0;
      for (let k = -radius; k <= radius; k++) {
        const xx = x + k;
        if (xx < 0 || xx >= OLED_W) continue;
        sum += gray[y * OLED_W + xx];
        count++;
      }
      tmp[y * OLED_W + x] = sum / count;
    }
  }
  for (let x = 0; x < OLED_W; x++) {
    for (let y = 0; y < OLED_H; y++) {
      let sum = 0, count = 0;
      for (let k = -radius; k <= radius; k++) {
        const yy = y + k;
        if (yy < 0 || yy >= OLED_H) continue;
        sum += tmp[yy * OLED_W + x];
        count++;
      }
      out[y * OLED_W + x] = sum / count;
    }
  }
  return out;
}

// Modos de inversión:
//   'none'   sin tocar
//   'colors' voltea el color del contenido dibujado y deja la transparencia
//            como fondo. Es lo que hace visible un icono negro sobre alfa.
//   'box'    voltea el rectángulo que ocupa la capa: fondo encendido y
//            contenido apagado.
// Ninguno toca píxeles fuera de la capa; invertir el lienzo entero es la
// operación del botón "Invertir" de la barra de herramientas, no de esta.
function canvasToFramebuffer(ctx, { threshold = 128, dither = false, blur = 0, invert = 'none', bounds = null } = {}) {
  const { data } = ctx.getImageData(0, 0, OLED_W, OLED_H);
  const fb = createFramebuffer();

  // Luminancia en escala de grises, con alfa compuesto sobre negro.
  const flipColors = invert === 'colors';
  let gray = new Float32Array(OLED_W * OLED_H);
  for (let i = 0; i < OLED_W * OLED_H; i++) {
    const o = i * 4;
    const a = data[o + 3] / 255;
    const lum = 0.299 * data[o] + 0.587 * data[o + 1] + 0.114 * data[o + 2];
    // El volteo va antes de componer con el alfa: así lo transparente sigue
    // siendo fondo en vez de convertirse en blanco.
    gray[i] = (flipColors ? 255 - lum : lum) * a;
  }

  gray = boxBlur(gray, blur);

  // El recuadro se invierte después del desenfoque para que sus bordes queden
  // limpios en lugar de difuminarse.
  if (invert === 'box' && bounds) {
    const x0 = Math.max(0, Math.floor(bounds.x));
    const y0 = Math.max(0, Math.floor(bounds.y));
    const x1 = Math.min(OLED_W, Math.ceil(bounds.x + bounds.width));
    const y1 = Math.min(OLED_H, Math.ceil(bounds.y + bounds.height));
    for (let y = y0; y < y1; y++) {
      for (let x = x0; x < x1; x++) {
        const i = y * OLED_W + x;
        gray[i] = 255 - gray[i];
      }
    }
  }

  if (dither) {
    // Floyd-Steinberg: imprescindible para que una foto se lea en 1 bit.
    for (let y = 0; y < OLED_H; y++) {
      for (let x = 0; x < OLED_W; x++) {
        const i = y * OLED_W + x;
        const old = gray[i];
        const next = old < threshold ? 0 : 255;
        gray[i] = next;
        const err = old - next;
        if (x + 1 < OLED_W)                 gray[i + 1]            += err * 7 / 16;
        if (x > 0 && y + 1 < OLED_H)        gray[i + OLED_W - 1]   += err * 3 / 16;
        if (y + 1 < OLED_H)                 gray[i + OLED_W]       += err * 5 / 16;
        if (x + 1 < OLED_W && y + 1 < OLED_H) gray[i + OLED_W + 1] += err * 1 / 16;
      }
    }
  }

  for (let y = 0; y < OLED_H; y++) {
    for (let x = 0; x < OLED_W; x++) {
      setPixel(fb, x, y, gray[y * OLED_W + x] >= threshold ? 1 : 0);
    }
  }
  return fb;
}

// El lienzo auxiliar se deja TRANSPARENTE a propósito. Rellenarlo de negro
// opaco dejaba alfa = 1 en toda la superficie, y entonces cualquier volteo de
// color alcanzaba también a la zona vacía que rodea a la capa. El fondo negro
// se obtiene igualmente al componer la luminancia con el alfa.
function offscreen() {
  const c = document.createElement('canvas');
  c.width = OLED_W;
  c.height = OLED_H;
  return c.getContext('2d', { willReadFrequently: true });
}

// --- Capa flotante ----------------------------------------------------------
//
// Una imagen importada o un texto generado no se vuelcan directamente en el
// framebuffer: se convierten en una "capa" con posición y escala propias que se
// puede recolocar antes de fijarla. Antes se pegaban centradas y sin ajuste
// posible, y con 72x40 píxeles eso casi nunca cae donde interesa.

export async function loadImageSource(file) {
  const bitmap = await createImageBitmap(file);
  return { kind: 'image', bitmap, naturalWidth: bitmap.width, naturalHeight: bitmap.height };
}

// Capa a partir de un SVG en texto (la biblioteca de iconos). No pasa por
// createImageBitmap: con SVG depende de que el marcado traiga tamaño propio y
// de la versión de Chromium, mientras que <img> + data URI lo dibuja siempre.
// El SVG se rasteriza a RASTER_PX y no a 72x40 porque drawSource lo reescala
// después: partir de una copia grande deja los bordes limpios al encogerla.
const SVG_RASTER_PX = 256;

export function makeSvgSource(markup, { size = SVG_RASTER_PX } = {}) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.width = size;
    img.height = size;
    img.onload = () => resolve({
      kind: 'image', bitmap: img, naturalWidth: size, naturalHeight: size,
      // Trazo vectorial ya limpio: no necesita el desenfoque previo que se le
      // aplica a las fotos importadas (ver defaultTransform en views/oled.js).
      crisp: true,
    });
    img.onerror = () => reject(new Error('SVG no válido'));
    img.src = `data:image/svg+xml;charset=utf-8,${encodeURIComponent(markup)}`;
  });
}

export function makeTextSource(text, { fontSize = 16, bold = true, font = 'Segoe UI' } = {}) {
  return { kind: 'text', text: String(text), fontSize, bold, font };
}

function fontSpec(source) {
  return `${source.bold ? '700 ' : ''}${source.fontSize}px "${source.font}", sans-serif`;
}

export function textLines(source) {
  return source.text.split('\n').slice(0, 4);
}

// Tamaño natural de la capa en píxeles de la pantalla, sin aplicar la escala.
export function measureSource(source) {
  if (source.kind === 'image') {
    return { width: source.naturalWidth, height: source.naturalHeight };
  }
  const ctx = offscreen();
  ctx.font = fontSpec(source);
  const lines = textLines(source);
  const width = Math.max(1, ...lines.map((l) => ctx.measureText(l).width));
  return { width, height: lines.length * (source.fontSize + 2) };
}

// --- Caja de tinta ----------------------------------------------------------
//
// measureSource() devuelve la caja de MAQUETACIÓN: el ancho de avance del texto
// (con sus espacios laterales) y el alto de línea completo (con el hueco de
// ascendentes y descendentes aunque no haya ninguna). Eso es bastante más
// grande que los trazos que acaban encendiéndose, así que el recuadro de la
// capa sobraba por todos lados y centrar o encajar centraba el hueco en vez de
// lo dibujado.
//
// sourceInkBox() devuelve el rectángulo que ocupan de verdad los píxeles, en
// coordenadas naturales de la fuente (sin escala ni posición): (x, y) es el
// desplazamiento desde el origen de la capa. Se calcula una vez por fuente y se
// guarda en ella, porque no depende ni de la escala ni de la posición.
export function sourceInkBox(source) {
  if (!source._ink) {
    source._ink = source.kind === 'text' ? textInkBox(source) : imageInkBox(source);
  }
  return source._ink;
}

function textInkBox(source) {
  const ctx = offscreen();
  ctx.font = fontSpec(source);
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';

  const lineHeight = source.fontSize + 2;
  let left = Infinity, top = Infinity, right = -Infinity, bottom = -Infinity;

  textLines(source).forEach((line, i) => {
    if (!line.trim()) return; // una línea en blanco no enciende nada
    const m = ctx.measureText(line);
    const originY = i * lineHeight;
    // Las métricas actualBoundingBox* van referidas al origen del texto, que
    // aquí es la esquina superior izquierda por textAlign/textBaseline.
    left   = Math.min(left,   -(m.actualBoundingBoxLeft || 0));
    right  = Math.max(right,  m.actualBoundingBoxRight ?? m.width);
    top    = Math.min(top,    originY - (m.actualBoundingBoxAscent || 0));
    bottom = Math.max(bottom, originY + (m.actualBoundingBoxDescent || 0));
  });

  if (!(right > left) || !(bottom > top)) {
    const { width, height } = measureSource(source);
    return { x: 0, y: 0, width, height };
  }
  return { x: left, y: top, width: right - left, height: bottom - top };
}

// La imagen se escanea a resolución reducida: el error que introduce queda muy
// por debajo de un píxel de la pantalla una vez reescalada a 72x40, y evita
// recorrer los millones de píxeles de una foto cada vez que se importa.
const INK_SCAN_MAX = 512;

function imageInkBox(source) {
  const nw = source.naturalWidth, nh = source.naturalHeight;
  const full = { x: 0, y: 0, width: nw, height: nh };

  const s = Math.min(1, INK_SCAN_MAX / Math.max(nw, nh));
  const w = Math.max(1, Math.round(nw * s)), h = Math.max(1, Math.round(nh * s));
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext('2d', { willReadFrequently: true });
  ctx.drawImage(source.bitmap, 0, 0, w, h);

  let data;
  try { data = ctx.getImageData(0, 0, w, h).data; } catch { return full; }

  let left = w, top = h, right = -1, bottom = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (data[(y * w + x) * 4 + 3] < 8) continue; // transparente: no es tinta
      if (x < left) left = x;
      if (x > right) right = x;
      if (y < top) top = y;
      if (y > bottom) bottom = y;
    }
  }
  if (right < 0) return full; // imagen entera transparente

  const sx = nw / w, sy = nh / h;
  return {
    x: left * sx,
    y: top * sy,
    width: (right - left + 1) * sx,
    height: (bottom - top + 1) * sy,
  };
}

// Escala inicial: encajar dentro de la pantalla dejando un margen mínimo.
export function fitScale(source) {
  const ink = sourceInkBox(source);
  if (!ink.width || !ink.height) return 1;
  return Math.min((OLED_W - 2) / ink.width, (OLED_H - 2) / ink.height);
}

function drawSource(ctx, source, layer) {
  const { width, height } = measureSource(source);
  const w = width * layer.scale;
  const h = height * layer.scale;

  if (source.kind === 'image') {
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = 'high';
    ctx.drawImage(source.bitmap, layer.x, layer.y, w, h);
    return { x: layer.x, y: layer.y, width: w, height: h };
  }

  ctx.save();
  ctx.fillStyle = '#fff';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.translate(layer.x, layer.y);
  ctx.scale(layer.scale, layer.scale);
  ctx.font = fontSpec(source);

  const lineHeight = source.fontSize + 2;
  textLines(source).forEach((line, i) => ctx.fillText(line, 0, i * lineHeight));
  ctx.restore();

  return { x: layer.x, y: layer.y, width: w, height: h };
}

// Recuadro de la capa: se ciñe a la tinta, no a la caja de maquetación, para
// que lo que marca el recuadro naranja sea exactamente lo que se va a dibujar.
export function layerBounds(source, layer) {
  const ink = sourceInkBox(source);
  return {
    x: layer.x + ink.x * layer.scale,
    y: layer.y + ink.y * layer.scale,
    width: ink.width * layer.scale,
    height: ink.height * layer.scale,
  };
}

// Rasteriza la capa a 1 bit. `layer` = { x, y, scale, threshold, blur, dither, invert }.
export function rasterizeLayer(source, layer) {
  const ctx = offscreen();
  drawSource(ctx, source, layer);
  return canvasToFramebuffer(ctx, {
    threshold: layer.threshold,
    dither: layer.dither,
    blur: layer.blur,
    invert: layer.invert,
    bounds: layerBounds(source, layer),
  });
}

// Rectángulo que ocupan los píxeles encendidos de un framebuffer, o null si no
// hay ninguno. Es la medida exacta de lo que se va a ver: sourceInkBox() se
// queda cerca pero no clava, porque el antialiasing y el ajuste de la fuente
// hacen que el trazo rasterizado no sea el escalado lineal de las métricas.
// Solo puede medir lo que cabe en la pantalla; lo que se salga queda recortado.
export function framebufferInk(fb) {
  let left = OLED_W, top = OLED_H, right = -1, bottom = -1;
  for (let y = 0; y < OLED_H; y++) {
    for (let x = 0; x < OLED_W; x++) {
      if (!getPixel(fb, x, y)) continue;
      if (x < left) left = x;
      if (x > right) right = x;
      if (y < top) top = y;
      if (y > bottom) bottom = y;
    }
  }
  if (right < 0) return null;
  return { x: left, y: top, width: right - left + 1, height: bottom - top + 1 };
}

// Une dos framebuffers. En modo "replace" la capa sustituye todo el lienzo; en
// "merge" se suma a lo que ya hubiera dibujado.
export function compose(base, overlay, mode = 'replace') {
  const out = createFramebuffer();
  for (let i = 0; i < out.length; i++) {
    out[i] = mode === 'merge' ? (base[i] | overlay[i]) : overlay[i];
  }
  return out;
}

// Pinta el framebuffer en un <canvas> ampliado, con rejilla opcional.
// `outline` dibuja el recuadro de la capa flotante que se está colocando.
export function renderToCanvas(fb, canvas, { zoom = 8, grid = true, color = '#e9e2ff', outline = null } = {}) {
  canvas.width = OLED_W * zoom;
  canvas.height = OLED_H * zoom;
  const ctx = canvas.getContext('2d');

  ctx.fillStyle = '#05050a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = color;
  for (let y = 0; y < OLED_H; y++) {
    for (let x = 0; x < OLED_W; x++) {
      if (getPixel(fb, x, y)) ctx.fillRect(x * zoom, y * zoom, zoom, zoom);
    }
  }

  if (grid && zoom >= 5) {
    ctx.strokeStyle = 'rgba(255,255,255,0.07)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 0; x <= OLED_W; x++) {
      ctx.moveTo(x * zoom + 0.5, 0);
      ctx.lineTo(x * zoom + 0.5, canvas.height);
    }
    for (let y = 0; y <= OLED_H; y++) {
      ctx.moveTo(0, y * zoom + 0.5);
      ctx.lineTo(canvas.width, y * zoom + 0.5);
    }
    ctx.stroke();

    // Marcas de página: el firmware direcciona la pantalla en bandas de 8 px.
    ctx.strokeStyle = 'rgba(139,92,246,0.25)';
    ctx.beginPath();
    for (let p = 1; p < OLED_PAGES; p++) {
      ctx.moveTo(0, p * 8 * zoom + 0.5);
      ctx.lineTo(canvas.width, p * 8 * zoom + 0.5);
    }
    ctx.stroke();
  }

  if (outline) {
    ctx.save();
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(outline.x * zoom, outline.y * zoom, outline.width * zoom, outline.height * zoom);

    // Tiradores en las esquinas para que se vea que se puede redimensionar.
    ctx.setLineDash([]);
    ctx.fillStyle = '#f59e0b';
    const handle = Math.max(5, Math.min(10, zoom));
    const corners = [
      [outline.x, outline.y],
      [outline.x + outline.width, outline.y],
      [outline.x, outline.y + outline.height],
      [outline.x + outline.width, outline.y + outline.height],
    ];
    for (const [cx, cy] of corners) {
      ctx.fillRect(cx * zoom - handle / 2, cy * zoom - handle / 2, handle, handle);
    }
    ctx.restore();
  }
}
