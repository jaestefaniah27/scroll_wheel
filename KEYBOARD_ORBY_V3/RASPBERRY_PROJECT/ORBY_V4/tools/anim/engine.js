// ============================================================================
// Motor de la intro de ORBY
// ============================================================================
// Dibuja la animación sobre un lienzo virtual que coloca las diez pantallas
// donde están en el teclado, y la trocea en los diez framebuffers de 360 bytes
// que entiende el SSD1306. Ese troceado es EXACTAMENTE lo que se hornea y lo
// que reproduce el firmware: la vista previa no es una aproximación, es el
// resultado final.
//
// Se carga como script clásico (funciona con doble clic, sin servidor) y deja
// todo colgando de window.Anim.
(function (root) {
'use strict';

// --- Geometría --------------------------------------------------------------
var SCR_W = 72, SCR_H = 40;

// Rejilla de 3x4 teclas. Las teclas 10 y 12 no llevan pantalla, así que en la
// última fila solo está la del centro (OLED 10, montada en la tecla 11).
var SCREEN_COL = [0, 1, 2, 0, 1, 2, 0, 1, 2, 1];
var SCREEN_ROW = [0, 0, 0, 1, 1, 1, 2, 2, 2, 3];

function geom(p) {
  var pitchX = SCR_W + p.gapX, pitchY = SCR_H + p.gapY;
  var g = {
    pitchX: pitchX, pitchY: pitchY,
    W: 3 * pitchX, H: 4 * pitchY,
    ox: [], oy: []
  };
  for (var i = 0; i < 10; i++) {
    g.ox.push(SCREEN_COL[i] * pitchX + (p.gapX >> 1));
    g.oy.push(SCREEN_ROW[i] * pitchY + (p.gapY >> 1));
  }
  // Centro de la pantalla 5 (índice 4): el origen de toda la animación.
  g.cx = g.ox[4] + SCR_W / 2;
  g.cy = g.oy[4] + SCR_H / 2;
  return g;
}

// --- Parámetros -------------------------------------------------------------
// Cada uno lleva su rango: la interfaz se genera sola a partir de esta tabla.
var SCHEMA = [
  { group: 'Lienzo' },
  { key: 'gapX', label: 'Hueco horizontal', min: 0, max: 120, step: 2, def: 40,
    help: 'Separación entre pantallas. El valor físico real es 86, pero comprimido se lee mejor.' },
  { key: 'gapY', label: 'Hueco vertical', min: 0, max: 140, step: 2, def: 34, help: 'El valor físico real es 118.' },
  { key: 'fps', label: 'Fotogramas por segundo', min: 10, max: 60, step: 1, def: 30 },
  { key: 'dither', label: 'Tramado', type: 'select', def: 'bayer',
    options: [['bayer', 'Bayer 4x4 (degradados)'], ['hard', 'Duro (sin grises)']] },

  { group: 'Texto' },
  { key: 'text', label: 'Palabra', type: 'text', def: 'ORBY' },
  { key: 'font', label: 'Tipografía', type: 'text', def: 'Arial Black, Impact, sans-serif' },
  { key: 'weight', label: 'Grosor', min: 100, max: 900, step: 100, def: 900 },
  { key: 'tracking', label: 'Espaciado entre letras', min: -10, max: 30, step: 1, def: 0, unit: 'px' },

  { group: 'Aparición y crecimiento' },
  { key: 'sizeStart', label: 'Tamaño inicial', min: 2, max: 60, step: 1, def: 6, unit: 'px' },
  { key: 'sizeBase', label: 'Tamaño en la pantalla 5', min: 8, max: 60, step: 1, def: 30, unit: 'px' },
  // Por encima de ~180 px cada pantalla cae dentro de un trazo y solo se ve
  // blanco liso; deja de leerse como letras.
  { key: 'sizeMax', label: 'Tamaño máximo', min: 40, max: 600, step: 5, def: 150, unit: 'px' },
  { key: 'msAppear', label: 'Duración: aparece', min: 0, max: 2000, step: 50, def: 350, unit: 'ms' },
  { key: 'msHold', label: 'Duración: se queda quieto', min: 0, max: 2000, step: 50, def: 250, unit: 'ms' },
  { key: 'msGrow', label: 'Duración: crece', min: 50, max: 3000, step: 50, def: 550, unit: 'ms' },
  { key: 'msShrink', label: 'Duración: se encoge', min: 50, max: 3000, step: 50, def: 400, unit: 'ms' },
  { key: 'growEase', label: 'Curva al crecer', type: 'select', def: 'in',
    options: [['in', 'Acelerando'], ['out', 'Frenando'], ['inout', 'Suave'], ['lin', 'Lineal']] },
  { key: 'shrinkEase', label: 'Curva al encoger', type: 'select', def: 'in',
    options: [['in', 'Acelerando'], ['out', 'Frenando'], ['inout', 'Suave'], ['lin', 'Lineal']] },
  { key: 'clipS5', label: 'Recortar a la pantalla 5', type: 'bool', def: false,
    help: 'Si se marca, el texto no se derrama sobre las pantallas vecinas al crecer.' },

  { group: 'Explosión' },
  { key: 'flash', label: 'Fogonazo blanco', type: 'bool', def: true },
  { key: 'flashFrames', label: 'Fotogramas de fogonazo', min: 0, max: 6, step: 1, def: 2 },
  { key: 'msBurst', label: 'Duración: explosión', min: 100, max: 3000, step: 50, def: 500, unit: 'ms' },
  { key: 'msFall', label: 'Duración: caída de la arena', min: 100, max: 4000, step: 50, def: 1100, unit: 'ms' },

  { group: 'Ondas expansivas' },
  { key: 'ringCount', label: 'Número de ondas', min: 0, max: 6, step: 1, def: 3 },
  { key: 'ringDelay', label: 'Separación entre ondas', min: 0, max: 400, step: 10, def: 70, unit: 'ms' },
  { key: 'ringSpeed', label: 'Velocidad', min: 100, max: 2000, step: 20, def: 620, unit: 'px/s' },
  { key: 'ringThick', label: 'Grosor', min: 1, max: 40, step: 1, def: 12, unit: 'px' },
  { key: 'ringFade', label: 'Se apaga a los', min: 40, max: 500, step: 10, def: 260, unit: 'px' },

  { group: 'Arena' },
  { key: 'pCount', label: 'Número de partículas', min: 0, max: 1500, step: 10, def: 750 },
  // Las pantallas ocupan menos de un tercio del lienzo: con pocas partículas o
  // muy rápidas casi todas viajan por el hueco entre teclas y no se ven.
  { key: 'pSpeedMin', label: 'Velocidad mínima', min: 0, max: 800, step: 10, def: 30, unit: 'px/s' },
  { key: 'pSpeedMax', label: 'Velocidad máxima', min: 20, max: 2000, step: 10, def: 300, unit: 'px/s' },
  { key: 'pSize', label: 'Tamaño', min: 1, max: 4, step: 1, def: 1, unit: 'px' },
  { key: 'pBig', label: 'Proporción de granos gordos', min: 0, max: 100, step: 5, def: 25, unit: '%' },
  { key: 'gravity', label: 'Gravedad', min: 0, max: 3000, step: 25, def: 650, unit: 'px/s²' },
  { key: 'drag', label: 'Rozamiento del aire', min: 0, max: 100, step: 1, def: 35, unit: '%/s' },
  { key: 'jitter', label: 'Temblor (arena)', min: 0, max: 200, step: 5, def: 45 },
  { key: 'pFade', label: 'Se apagan al caer', type: 'bool', def: false,
    help: 'Si no, los granos siguen encendidos hasta que se salen del lienzo por abajo.' }
];

function defaults() {
  var p = {};
  SCHEMA.forEach(function (s) { if (s.key) p[s.key] = s.def; });
  return p;
}

// --- Curvas -----------------------------------------------------------------
function ease(kind, t) {
  if (t < 0) t = 0; else if (t > 1) t = 1;
  switch (kind) {
    case 'in':    return t * t;
    case 'out':   return 1 - (1 - t) * (1 - t);
    case 'inout': return t < 0.5 ? 2 * t * t : 1 - 2 * (1 - t) * (1 - t);
    default:      return t;
  }
}

// Generador pseudoaleatorio con semilla: la arena tiene que caer igual en la
// vista previa y en el horneado, si no lo que ves no es lo que se graba.
function rng(seed) {
  var s = seed >>> 0;
  return function () {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

// --- Fases ------------------------------------------------------------------
function phases(p) {
  var a = p.msAppear, h = a + p.msHold, g = h + p.msGrow, s = g + p.msShrink;
  return {
    appearEnd: a, holdEnd: h, growEnd: g,
    boom: s,                       // instante exacto en que el texto desaparece
    end: s + p.msBurst + p.msFall
  };
}

function totalFrames(p) {
  return Math.max(1, Math.round(phases(p).end * p.fps / 1000));
}

// Tamaño del texto en un instante dado (ms).
function textSize(p, t) {
  var ph = phases(p);
  if (t < ph.appearEnd) {
    return p.sizeStart + (p.sizeBase - p.sizeStart) * ease('out', t / Math.max(1, p.msAppear));
  }
  if (t < ph.holdEnd) return p.sizeBase;
  if (t < ph.growEnd) {
    return p.sizeBase + (p.sizeMax - p.sizeBase) * ease(p.growEase, (t - ph.holdEnd) / Math.max(1, p.msGrow));
  }
  if (t < ph.boom) {
    return p.sizeMax * (1 - ease(p.shrinkEase, (t - ph.growEnd) / Math.max(1, p.msShrink)));
  }
  return 0;
}

// --- Simulación de la arena -------------------------------------------------
// Se calcula entera de una vez al cambiar los parámetros, para que mover la
// barra de tiempo hacia atrás y hacia delante dé siempre lo mismo.
function simulate(p, g) {
  var n = p.pCount, frames = totalFrames(p), ph = phases(p);
  var boomFrame = Math.round(ph.boom * p.fps / 1000);
  var dt = 1 / p.fps;
  var rand = rng(20260729);

  var px = new Float32Array(n), py = new Float32Array(n);
  var vx = new Float32Array(n), vy = new Float32Array(n);
  var size = new Uint8Array(n), alive = new Uint8Array(n);

  for (var i = 0; i < n; i++) {
    var ang = rand() * Math.PI * 2;
    var sp = p.pSpeedMin + rand() * Math.max(0, p.pSpeedMax - p.pSpeedMin);
    px[i] = g.cx; py[i] = g.cy;
    vx[i] = Math.cos(ang) * sp;
    vy[i] = Math.sin(ang) * sp;
    size[i] = rand() * 100 < p.pBig ? p.pSize + 1 : p.pSize;
    alive[i] = 1;
  }

  // trail[f] = posiciones en el fotograma f (x, y, tamaño) empaquetadas
  var trail = new Array(frames);
  for (var f = 0; f < frames; f++) {
    if (f < boomFrame) { trail[f] = null; continue; }

    if (f > boomFrame) {
      var k = Math.pow(1 - Math.min(0.99, p.drag / 100), dt);
      for (var j = 0; j < n; j++) {
        if (!alive[j]) continue;
        vx[j] *= k;
        vy[j] = vy[j] * k + p.gravity * dt;
        // El temblor es lo que convierte los puntos en arena: sin él las
        // trayectorias son parábolas limpias y se nota que es matemática.
        px[j] += vx[j] * dt + (rand() - 0.5) * p.jitter * dt;
        py[j] += vy[j] * dt;
        if (py[j] > g.H + 8) alive[j] = 0;
      }
    }

    var out = [];
    for (var m = 0; m < n; m++) {
      if (!alive[m]) continue;
      out.push(px[m], py[m], size[m]);
    }
    trail[f] = out;
  }
  return { trail: trail, boomFrame: boomFrame };
}

// --- Dibujo de un fotograma -------------------------------------------------
// Se pinta en escala de grises sobre un canvas 2D (así el texto usa una
// tipografía de verdad y las ondas pueden tener bordes suaves) y después se
// binariza con tramado.
var BAYER = [
  [0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]
];

function drawFrame(ctx, p, g, sim, frame) {
  var t = frame * 1000 / p.fps;
  var ph = phases(p);

  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.globalAlpha = 1;
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, g.W, g.H);

  // Fogonazo: tapa todo justo al estallar.
  if (p.flash && p.flashFrames > 0) {
    var bf = frame - sim.boomFrame;
    if (bf >= 0 && bf < p.flashFrames) {
      ctx.fillStyle = '#fff';
      ctx.fillRect(0, 0, g.W, g.H);
      return;
    }
  }

  // --- Texto ---
  var size = textSize(p, t);
  if (size >= 1) {
    ctx.save();
    if (p.clipS5) {
      ctx.beginPath();
      ctx.rect(g.ox[4], g.oy[4], SCR_W, SCR_H);
      ctx.clip();
    }
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.font = p.weight + ' ' + size.toFixed(1) + 'px ' + p.font;
    if ('letterSpacing' in ctx) ctx.letterSpacing = p.tracking + 'px';
    ctx.fillText(p.text, g.cx, g.cy);
    ctx.restore();
  }

  // --- Ondas expansivas ---
  if (frame >= sim.boomFrame) {
    var since = (frame - sim.boomFrame) * 1000 / p.fps;
    for (var r = 0; r < p.ringCount; r++) {
      var age = since - r * p.ringDelay;
      if (age < 0) continue;
      var rad = age * p.ringSpeed / 1000;
      if (rad > p.ringFade) continue;
      var k = 1 - rad / p.ringFade;          // se adelgaza y se apaga al alejarse
      var th = Math.max(1, p.ringThick * k);
      ctx.globalAlpha = Math.min(1, 0.35 + 0.65 * k);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = th;
      ctx.beginPath();
      ctx.arc(g.cx, g.cy, rad, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  // --- Arena ---
  var pts = sim.trail[frame];
  if (pts) {
    ctx.fillStyle = '#fff';
    for (var i = 0; i < pts.length; i += 3) {
      var x = pts[i], y = pts[i + 1], s = pts[i + 2];
      if (p.pFade) {
        var life = 1 - (y - g.cy) / Math.max(1, g.H - g.cy);
        ctx.globalAlpha = Math.max(0.15, Math.min(1, life));
      }
      ctx.fillRect(x - s / 2, y - s / 2, s, s);
    }
    ctx.globalAlpha = 1;
  }
  // El instante del estallido y el final de las fases quedan a mano por si
  // hiciera falta depurar tiempos.
  void ph;
}

// --- Binarizado y troceado en las diez pantallas ----------------------------
// Devuelve un array de diez Uint8Array(360) en el formato de página del
// SSD1306: cada byte son ocho píxeles en vertical, bit 0 arriba.
function packScreens(imgData, p, g) {
  var data = imgData.data, W = g.W;
  var out = [];
  for (var s = 0; s < 10; s++) {
    var fb = new Uint8Array(360);
    var ox = g.ox[s], oy = g.oy[s];
    for (var y = 0; y < SCR_H; y++) {
      var vy = oy + y;
      for (var x = 0; x < SCR_W; x++) {
        var lum = data[((vy * W) + (ox + x)) * 4]; // dibujamos en gris: basta el rojo
        var on;
        if (p.dither === 'hard') {
          on = lum >= 128;
        } else {
          on = lum > (BAYER[y & 3][x & 3] + 0.5) * 16;
        }
        if (on) fb[(y >> 3) * SCR_W + x] |= (1 << (y & 7));
      }
    }
    out.push(fb);
  }
  return out;
}

// --- Compresión y exportación ----------------------------------------------
// Por fotograma: máscara de las pantallas que cambian y, de cada una, su
// framebuffer comprimido por longitud de carrera. Las pantallas que no cambian
// no ocupan nada y encima el firmware se ahorra el SPI.
function rleEncode(fb) {
  var out = [];
  var i = 0;
  while (i < fb.length) {
    var v = fb[i], n = 1;
    while (i + n < fb.length && fb[i + n] === v && n < 255) n++;
    out.push(n, v);
    i += n;
  }
  return out;
}

function bake(p, onProgress) {
  var g = geom(p);
  var sim = simulate(p, g);
  var frames = totalFrames(p);
  var cv = makeCanvas(g.W, g.H);
  var ctx = cv.getContext('2d', { willReadFrequently: true });

  var prev = [];
  for (var i = 0; i < 10; i++) prev.push(new Uint8Array(360));
  var first = true;

  var stream = [];
  for (var f = 0; f < frames; f++) {
    drawFrame(ctx, p, g, sim, f);
    var scr = packScreens(ctx.getImageData(0, 0, g.W, g.H), p, g);

    var mask = 0, payload = [];
    for (var s = 0; s < 10; s++) {
      var same = !first;
      if (same) {
        for (var b = 0; b < 360; b++) { if (scr[s][b] !== prev[s][b]) { same = false; break; } }
      }
      if (same) continue;
      mask |= (1 << s);
      var enc = rleEncode(scr[s]);
      payload.push(enc.length & 255, (enc.length >> 8) & 255);
      payload = payload.concat(enc);
      prev[s] = scr[s];
    }
    first = false;
    stream.push(mask & 255, (mask >> 8) & 255);
    for (var q = 0; q < payload.length; q++) stream.push(payload[q]);
    if (onProgress) onProgress(f + 1, frames);
  }
  return { bytes: stream, frames: frames, params: p };
}

function toHeader(baked) {
  var p = baked.params, b = baked.bytes;
  var lines = [];
  lines.push('// ============================================================');
  lines.push('// Intro de arranque de ORBY — GENERADO, no editar a mano');
  lines.push('// Creado con tools/anim/index.html el ' + new Date().toISOString().slice(0, 19).replace('T', ' '));
  lines.push('// ' + baked.frames + ' fotogramas a ' + p.fps + ' fps (' +
             (baked.frames * 1000 / p.fps / 1000).toFixed(2) + ' s), ' + b.length + ' bytes');
  lines.push('//');
  lines.push('// Ajustes con los que se generó (pegar en «Cargar ajustes» para retocarla):');
  lines.push('// PRESET ' + JSON.stringify(p));
  lines.push('// ============================================================');
  lines.push('#pragma once');
  lines.push('#include <stdint.h>');
  lines.push('');
  lines.push('#define INTRO_FPS    ' + p.fps);
  lines.push('#define INTRO_FRAMES ' + baked.frames);
  lines.push('');
  lines.push('// Por fotograma: uint16 con las pantallas que cambian (bit 0 = OLED 1) y,');
  lines.push('// por cada una, uint16 de longitud y su framebuffer comprimido en pares');
  lines.push('// (repeticiones, valor).');
  lines.push('static const uint8_t intro_stream[] = {');
  for (var i = 0; i < b.length; i += 16) {
    var row = b.slice(i, i + 16).map(function (v) {
      return '0x' + (v < 16 ? '0' : '') + v.toString(16);
    }).join(',');
    lines.push('    ' + row + ',');
  }
  lines.push('};');
  lines.push('');
  return lines.join('\n');
}

// --- Utilidades multiplataforma ---------------------------------------------
function makeCanvas(w, h) {
  if (typeof document !== 'undefined') {
    var c = document.createElement('canvas');
    c.width = w; c.height = h;
    return c;
  }
  throw new Error('Sin canvas disponible');
}

root.Anim = {
  SCR_W: SCR_W, SCR_H: SCR_H,
  SCREEN_COL: SCREEN_COL, SCREEN_ROW: SCREEN_ROW,
  SCHEMA: SCHEMA, defaults: defaults,
  geom: geom, phases: phases, totalFrames: totalFrames, textSize: textSize,
  simulate: simulate, drawFrame: drawFrame, packScreens: packScreens,
  bake: bake, toHeader: toHeader, makeCanvas: makeCanvas
};

})(typeof window !== 'undefined' ? window : globalThis);
