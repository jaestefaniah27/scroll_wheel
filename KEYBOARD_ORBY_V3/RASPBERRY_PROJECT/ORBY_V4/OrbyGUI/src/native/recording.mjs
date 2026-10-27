// Grabación (eventos de la app) → registros que reproduce el teclado.
//
// Grabar sigue siendo cosa del PC: el teclado no ve el ratón ni el otro
// teclado. Pero una vez grabado, reproducirlo es mandar teclas y mover un
// puntero, y eso el teclado lo sabe hacer solo (MSTEP_EVENTS en main.cpp).
//
// Los eventos llegan en el formato que guarda la grabadora
// (orby_core::grabacion::Evento): códigos de tecla de libuiohook y píxeles
// físicos del escritorio virtual. Aquí se pasan a usos HID y al rango del
// puntero absoluto. Módulo puro, sin DOM.

import { varint, MAX_BLOB } from './pool.mjs';

// Tipos de registro (EVT_* en main.cpp).
export const EVT = { KEY_DOWN: 1, KEY_UP: 2, ABS: 3, BTN_DOWN: 4, BTN_UP: 5, WHEEL: 6, HWHEEL: 7 };

export const ABS_MAX = 32767;

// La velocidad con la que reproduce el PC si la grabación no dice otra
// (VELOCIDAD_POR_DEFECTO en recorder.rs).
export const DEFAULT_SPEED = 3.0;

// Código de libuiohook (scancode del juego 1, con 0x0E00 o 0xE000 delante en
// las extendidas) → uso HID. Cubre lo mismo que src-tauri/crates/orby-core/src/uiohook.rs.
const UIOHOOK_TO_HID = new Map([
  [1, 0x29],
  [2, 0x1E], [3, 0x1F], [4, 0x20], [5, 0x21], [6, 0x22], [7, 0x23], [8, 0x24], [9, 0x25], [10, 0x26], [11, 0x27],
  [12, 0x2D], [13, 0x2E], [14, 0x2A], [15, 0x2B],
  [16, 0x14], [17, 0x1A], [18, 0x08], [19, 0x15], [20, 0x17], [21, 0x1C], [22, 0x18], [23, 0x0C], [24, 0x12], [25, 0x13],
  [26, 0x2F], [27, 0x30], [28, 0x28], [29, 0xE0],
  [30, 0x04], [31, 0x16], [32, 0x07], [33, 0x09], [34, 0x0A], [35, 0x0B], [36, 0x0D], [37, 0x0E], [38, 0x0F],
  [39, 0x33], [40, 0x34], [41, 0x35], [42, 0xE1], [43, 0x31],
  [44, 0x1D], [45, 0x1B], [46, 0x06], [47, 0x19], [48, 0x05], [49, 0x11], [50, 0x10],
  [51, 0x36], [52, 0x37], [53, 0x38], [54, 0xE5], [55, 0x55], [56, 0xE2], [57, 0x2C], [58, 0x39],
  [59, 0x3A], [60, 0x3B], [61, 0x3C], [62, 0x3D], [63, 0x3E], [64, 0x3F], [65, 0x40], [66, 0x41], [67, 0x42], [68, 0x43],
  [69, 0x53], [70, 0x47],
  [71, 0x5F], [72, 0x60], [73, 0x61], [74, 0x56], [75, 0x5C], [76, 0x5D], [77, 0x5E], [78, 0x57],
  [79, 0x59], [80, 0x5A], [81, 0x5B], [82, 0x62], [83, 0x63], [86, 0x64],
  [87, 0x44], [88, 0x45],
  [91, 0x68], [92, 0x69], [93, 0x6A],
  [99, 0x6B], [100, 0x6C], [101, 0x6D], [102, 0x6E], [103, 0x6F], [104, 0x70], [105, 0x71], [106, 0x72], [107, 0x73],
  [3612, 0x58], [3613, 0xE4], [3637, 0x54], [3639, 0x46], [3640, 0xE6],
  [3655, 0x4A], [3657, 0x4B], [3663, 0x4D], [3665, 0x4E], [3666, 0x49], [3667, 0x4C],
  [3675, 0xE3], [3676, 0xE7], [3677, 0x65],
  [57416, 0x52], [57419, 0x50], [57421, 0x4F], [57424, 0x51],
  // El numérico sin Bloq Num: libuiohook las da con códigos propios.
  [60999, 0x4A], [61000, 0x52], [61001, 0x4B], [61003, 0x50], [61005, 0x4F],
  [61007, 0x4D], [61008, 0x51], [61009, 0x4E], [61010, 0x49], [61011, 0x4C],
]);

export function uiohookToHid(code) {
  return UIOHOOK_TO_HID.get(code) ?? null;
}

// libuiohook numera 1 izquierdo, 2 derecho, 3 central; el informe, por bits.
const BUTTON_BIT = { 1: 0x01, 2: 0x02, 3: 0x04 };

// Píxel físico del escritorio virtual → rango del puntero absoluto.
export function toAbs(x, y, desk) {
  const nx = Math.round(((x - desk.x) * ABS_MAX) / Math.max(1, desk.w - 1));
  const ny = Math.round(((y - desk.y) * ABS_MAX) / Math.max(1, desk.h - 1));
  const clamp = (v) => Math.min(ABS_MAX, Math.max(0, v));
  return [clamp(nx), clamp(ny)];
}

const int8 = (v) => Math.max(-127, Math.min(127, Math.round(v)));

// Eventos → trozos de registros, cada uno de MAX_BLOB bytes como mucho (lo
// que cabe en el `b` de un paso) y cortados siempre entre registros.
// Devuelve { ok: true, chunks: Uint8Array[] } o { ok: false, reason }.
export function compileRecording(events, { desk, speed = DEFAULT_SPEED } = {}) {
  if (!Array.isArray(events) || !events.length) return { ok: false, reason: 'vacía' };
  const needsDesk = events.some((e) => e.k === 'move' || e.k === 'mdown' || e.k === 'mup');
  if (needsDesk && !(desk && desk.w > 0 && desk.h > 0)) {
    return { ok: false, reason: 'no se sabe el tamaño de la pantalla en la que se grabó' };
  }
  const factor = speed > 0 ? speed : DEFAULT_SPEED;

  const records = [];
  let lastT = 0;
  let pos = null;
  const push = (t, type, payload) => {
    const at = Math.round(t / factor);
    records.push([...varint(Math.max(0, at - lastT)), type, ...payload]);
    lastT = Math.max(lastT, at);
  };
  const moveTo = (t, x, y) => {
    const [nx, ny] = toAbs(x, y, desk);
    if (pos && pos[0] === nx && pos[1] === ny) return;
    pos = [nx, ny];
    push(t, EVT.ABS, [nx & 0xFF, nx >> 8, ny & 0xFF, ny >> 8]);
  };

  for (const e of events) {
    const t = Number(e.t) || 0;
    switch (e.k) {
      case 'kdown':
      case 'kup': {
        const usage = uiohookToHid(e.c);
        // Una tecla sin traducción se salta, como hace el PC (ver aplicar en recorder.rs).
        if (usage !== null) push(t, e.k === 'kdown' ? EVT.KEY_DOWN : EVT.KEY_UP, [usage]);
        break;
      }
      case 'move':
        moveTo(t, e.x, e.y);
        break;
      case 'mdown':
      case 'mup': {
        const bit = BUTTON_BIT[e.b];
        if (!bit) break;
        // El puntero se recoloca antes del clic, igual que en el PC: el
        // movimiento que llevaba hasta aquí pudo quedarse en el filtro.
        moveTo(t, e.x, e.y);
        push(t, e.k === 'mdown' ? EVT.BTN_DOWN : EVT.BTN_UP, [bit]);
        break;
      }
      case 'wheel': {
        // En la grabación, negativo es hacia arriba (como lo dejaba
        // libuiohook); en el HID, positivo. Un 0 es una muesca hacia abajo,
        // que es lo que hace el PC con él.
        const r = Number(e.r) || 1;
        push(t, e.d === 4 ? EVT.HWHEEL : EVT.WHEEL, [int8(-r) & 0xFF]);
        break;
      }
      default:
        break;
    }
  }

  if (!records.length) return { ok: false, reason: 'no queda nada que el teclado sepa reproducir' };

  const chunks = [];
  let cur = [];
  for (const r of records) {
    if (cur.length + r.length > MAX_BLOB) { chunks.push(Uint8Array.from(cur)); cur = []; }
    cur.push(...r);
  }
  if (cur.length) chunks.push(Uint8Array.from(cur));
  return { ok: true, chunks };
}
