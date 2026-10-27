// El bloque con las macros del PC que se guarda en la Flash del teclado (ver
// MSTORE_* en main.cpp). El firmware no lo interpreta: solo lo guarda, lo
// devuelve y comprueba su CRC. Todo lo de dentro lo decide este fichero.
//
// Formato:
//   bytes 0-2  "OMB"
//   byte  3    versión del formato (1)
//   byte  4    banderas: bit 0 = cuerpo comprimido con deflate sin cabecera
//   resto      JSON en UTF-8 {"macros":[...]}, comprimido si lo dice la bandera
//
// Va comprimido porque una grabación de ratón son miles de eventos de JSON muy
// repetitivo: comprimido ocupa cinco o diez veces menos, y el teclado tiene
// 224 KB para esto.
//
// Es un módulo puro (sin window ni teclado) para poder probarlo con node --test.

const MAGIC = [0x4f, 0x4d, 0x42]; // "OMB"
const VERSION = 1;
const FLAG_DEFLATE = 0x01;

const ENCODER = new TextEncoder();
const DECODER = new TextDecoder();

// El mismo CRC32 que crc32_feed en include/config_hash.h (y que hash.js).
const NIBBLE = new Uint32Array([
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
  0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
  0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
]);

export function crc32(bytes) {
  let crc = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) {
    crc = (crc ^ bytes[i]) >>> 0;
    crc = ((crc >>> 4) ^ NIBBLE[crc & 0x0f]) >>> 0;
    crc = ((crc >>> 4) ^ NIBBLE[crc & 0x0f]) >>> 0;
  }
  return (crc ^ 0xffffffff) >>> 0;
}

// Ordenadas por id y con las claves en orden fijo: dos PCs con las mismas
// macros producen el mismo texto, y así se puede decidir si hay algo nuevo
// comparando textos en vez de mirar macro a macro.
function sortKeys(v) {
  if (Array.isArray(v)) return v.map(sortKeys);
  if (v && typeof v === 'object') {
    const out = {};
    for (const k of Object.keys(v).sort()) out[k] = sortKeys(v[k]);
    return out;
  }
  return v;
}

export function canonical(macros) {
  const list = [...(macros || [])].sort((a, b) => a.id - b.id);
  return JSON.stringify({ macros: list.map(sortKeys) });
}

async function pipe(bytes, stream) {
  const out = new Blob([bytes]).stream().pipeThrough(stream);
  return new Uint8Array(await new Response(out).arrayBuffer());
}

const canCompress = () => typeof CompressionStream === 'function'
                       && typeof DecompressionStream === 'function';

export async function encode(macros) {
  const json = ENCODER.encode(canonical(macros));
  // Sin CompressionStream (no debería pasar en ningún Chromium actual) se
  // guarda sin comprimir: ocupa más, pero vale igual.
  const compress = canCompress();
  const body = compress ? await pipe(json, new CompressionStream('deflate-raw')) : json;
  const out = new Uint8Array(5 + body.length);
  out.set(MAGIC, 0);
  out[3] = VERSION;
  out[4] = compress ? FLAG_DEFLATE : 0;
  out.set(body, 5);
  return out;
}

// Devuelve la lista de macros. Lanza si el bloque no es de este formato: mejor
// no tocar nada que mezclar basura con las macros del PC.
export async function decode(bytes) {
  if (!bytes || bytes.length < 5 || bytes[0] !== MAGIC[0] || bytes[1] !== MAGIC[1] || bytes[2] !== MAGIC[2]) {
    throw new Error('El teclado no tiene un almacén de macros reconocible');
  }
  if (bytes[3] > VERSION) {
    throw new Error('Las macros del teclado las guardó una versión más nueva de la app');
  }
  let body = bytes.subarray(5);
  if (bytes[4] & FLAG_DEFLATE) {
    if (!canCompress()) throw new Error('Este entorno no sabe descomprimir el almacén de macros');
    body = await pipe(body, new DecompressionStream('deflate-raw'));
  }
  const parsed = JSON.parse(DECODER.decode(body));
  if (!Array.isArray(parsed?.macros)) throw new Error('Almacén de macros sin lista');
  return parsed.macros.filter((m) => Number.isInteger(m?.id));
}

// Junta lo que trae el teclado con lo que tiene el PC. El teclado manda: es lo
// que viaja de un ordenador a otro y lo que sus teclas tienen asignado. Las que
// solo existen en el PC se conservan (pueden ser de otro teclado). Devuelve la
// lista nueva y cuántas macros han cambiado o llegado.
export function merge(local, fromKeyboard) {
  const byId = new Map((local || []).map((m) => [m.id, m]));
  let changed = 0;
  for (const m of fromKeyboard || []) {
    const mine = byId.get(m.id);
    if (!mine || JSON.stringify(sortKeys(mine)) !== JSON.stringify(sortKeys(m))) changed++;
    byId.set(m.id, m);
  }
  return { macros: [...byId.values()].sort((a, b) => a.id - b.id), changed };
}
