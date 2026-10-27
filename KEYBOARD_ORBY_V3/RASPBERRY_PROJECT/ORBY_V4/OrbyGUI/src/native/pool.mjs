// El pool de datos de las secuencias, del lado de la app.
//
// Textos y grabaciones no caben en un paso (cinco bytes): viven en una región
// de Flash del teclado (FLASH_POOL_OFFSET en main.cpp) y los pasos apuntan a su
// trozo. El firmware no reparte nada: la app decide dónde va cada cosa. Este
// módulo es esa decisión, sin tocar el puerto, para poder probarla.
//
// El reparto es por sectores enteros: cada macro es dueña de uno o varios
// sectores seguidos y nadie más escribe en ellos. Así cambiar el texto de una
// macro reescribe solo sus sectores, y el resto de macros no ven moverse nada.

export const SECTOR = 4096;
// Unidad del desplazamiento que lleva el paso en `a` (POOL_ALIGN en main.cpp).
export const ALIGN = 16;
// El tamaño de un trozo va en `b`, un int16 del firmware.
export const MAX_BLOB = 32767;

// CRC-16/CCITT-FALSE: la de crc16_ccitt en main.cpp. El paso guarda sus 15
// bits bajos en `gap`, y el firmware no reproduce un trozo que no cuadre.
export function crc16(bytes) {
  let crc = 0xFFFF;
  for (const b of bytes) {
    crc ^= b << 8;
    for (let i = 0; i < 8; i++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
  }
  return crc;
}

// CRC-32 estándar (la de zip), la que devuelven POOL_HASH y POOL_COMMIT.
const CRC32_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();

export function crc32(bytes) {
  let c = 0xFFFFFFFF;
  for (const b of bytes) c = CRC32_TABLE[(c ^ b) & 0xFF] ^ (c >>> 8);
  return (c ^ 0xFFFFFFFF) >>> 0;
}

// Pone los trozos de una macro uno detrás de otro, cada uno alineado a ALIGN.
// Devuelve la imagen de sus sectores (relleno a 0xFF, como la Flash borrada)
// y dónde quedó cada trozo, relativo al primer sector de la macro.
export function layoutBlobs(blobs) {
  const offsets = [];
  let size = 0;
  for (const b of blobs) {
    size = Math.ceil(size / ALIGN) * ALIGN;
    offsets.push(size);
    size += b.length;
  }
  const sectors = Math.max(1, Math.ceil(size / SECTOR));
  const image = new Uint8Array(sectors * SECTOR).fill(0xFF);
  blobs.forEach((b, i) => image.set(b, offsets[i]));
  return { image, offsets, sectors: blobs.length ? sectors : 0 };
}

// Qué sectores usa una macro según los pasos que tiene ya en el teclado.
// `steps` son los de device.getMacro: { type, a, b }.
export function sectorsUsedBy(steps, poolTypes) {
  const used = new Set();
  for (const s of steps || []) {
    if (!poolTypes.includes(s.type) || s.b <= 0) continue;
    const start = s.a * ALIGN;
    const end = start + s.b - 1;
    for (let sec = Math.floor(start / SECTOR); sec <= Math.floor(end / SECTOR); sec++) used.add(sec);
  }
  return used;
}

// Dónde poner `count` sectores de la macro `id`.
//
//   - `owners`: Map sector → id de quien lo usa ahora en el teclado.
//   - `total`: sectores del pool (el POOL= del handshake).
//
// Si la macro ya tenía sitio y le sigue cabiendo, se queda donde estaba: así
// una edición no mueve nada y solo se reescriben los sectores que cambian.
// Si no, el primer hueco libre (contando como libres los suyos de antes).
// null si no hay hueco.
export function allocate(id, count, owners, total) {
  if (count <= 0) return null;
  const mine = [...owners.entries()].filter(([, o]) => o === id).map(([s]) => s).sort((a, b) => a - b);
  const free = (s) => s >= 0 && s < total && (!owners.has(s) || owners.get(s) === id);

  if (mine.length) {
    const first = mine[0];
    let fits = true;
    for (let s = first; s < first + count; s++) if (!free(s)) { fits = false; break; }
    if (fits) return first;
  }

  for (let first = 0; first + count <= total; first++) {
    let fits = true;
    for (let s = first; s < first + count; s++) if (!free(s)) { fits = false; first = s; break; }
    if (fits) return first;
  }
  return null;
}

// LEB128 sin signo: la espera de cada registro de una grabación.
export function varint(n) {
  const out = [];
  let v = Math.max(0, Math.floor(n));
  do {
    let b = v & 0x7F;
    v = Math.floor(v / 128);
    if (v) b |= 0x80;
    out.push(b);
  } while (v);
  return out;
}

export function toHex(bytes) {
  let s = '';
  for (const b of bytes) s += b.toString(16).padStart(2, '0');
  return s;
}
