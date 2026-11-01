// El archivo de perfiles desactivados que se guarda en la Flash del teclado (ver
// PSTORE_* en main.cpp). Igual que el de las macros, el firmware no lo
// interpreta: lo guarda, lo devuelve y comprueba su CRC. Todo lo de dentro lo
// decide este fichero.
//
// Formato:
//   bytes 0-2  "OPB"
//   byte  3    versión del formato (1)
//   byte  4    banderas: bit 0 = cuerpo comprimido con deflate sin cabecera
//   resto      JSON en UTF-8 {"profiles":[...]}, comprimido si lo dice la bandera
//
// Cada entrada es la foto exacta de un perfil tal y como estaba al desactivarlo
// (páginas, teclas, mandos, rueda, iconos de cada página e icono del perfil), con
// un `uid` que no cambia nunca y un `rev` que sube con cada edición. Con eso dos
// PCs distinguen «el mismo perfil, editado» de «otro perfil que se llama igual».
//
// Va comprimido porque los iconos son mapas de bits en hexadecimal, casi todo
// ceros: un perfil de cuatro páginas con 19 iconos pasa de 21 KB a unos 2.
//
// Módulo puro (sin window ni teclado) para poder probarlo con node --test.

import { crc32, pipe, canCompress } from './macro-blob.mjs';

export { crc32 };

const MAGIC = [0x4f, 0x50, 0x42]; // "OPB"
const VERSION = 1;
const FLAG_DEFLATE = 0x01;

const ENCODER = new TextEncoder();
const DECODER = new TextDecoder();

// Con las claves ordenadas, dos PCs con lo mismo producen el mismo texto y se
// puede decidir si hay algo nuevo comparando textos.
function sortKeys(v) {
  if (Array.isArray(v)) return v.map(sortKeys);
  if (v && typeof v === 'object') {
    const out = {};
    for (const k of Object.keys(v).sort()) out[k] = sortKeys(v[k]);
    return out;
  }
  return v;
}

// El orden de la lista es el que ve el usuario: no se reordena.
export function canonical(profiles) {
  return JSON.stringify({ profiles: (profiles || []).map(sortKeys) });
}

export async function encode(profiles) {
  const json = ENCODER.encode(canonical(profiles));
  const compress = canCompress();
  const body = compress ? await pipe(json, new CompressionStream('deflate-raw')) : json;
  const out = new Uint8Array(5 + body.length);
  out.set(MAGIC, 0);
  out[3] = VERSION;
  out[4] = compress ? FLAG_DEFLATE : 0;
  out.set(body, 5);
  return out;
}

// Devuelve la lista de perfiles archivados. Lanza si el bloque no es de este
// formato: mejor no tocar nada que mezclar basura con los perfiles del usuario.
export async function decode(bytes) {
  if (!bytes || bytes.length < 5 || bytes[0] !== MAGIC[0] || bytes[1] !== MAGIC[1] || bytes[2] !== MAGIC[2]) {
    throw new Error('El teclado no tiene un archivo de perfiles reconocible');
  }
  if (bytes[3] > VERSION) {
    throw new Error('Los perfiles archivados los guardó una versión más nueva de la app');
  }
  let body = bytes.subarray(5);
  if (bytes[4] & FLAG_DEFLATE) {
    if (!canCompress()) throw new Error('Este entorno no sabe descomprimir el archivo de perfiles');
    body = await pipe(body, new DecompressionStream('deflate-raw'));
  }
  const parsed = JSON.parse(DECODER.decode(body));
  if (!Array.isArray(parsed?.profiles)) throw new Error('Archivo de perfiles sin lista');
  return parsed.profiles.filter((p) => typeof p?.uid === 'string' && p.data);
}

// Un identificador corto y estable para un perfil nuevo en el archivo.
export function newUid(rand = Math.random) {
  let s = '';
  for (let i = 0; i < 12; i++) s += Math.floor(rand() * 16).toString(16);
  return s;
}

// La entrada que se guarda al desactivar un perfil.
export function entryFrom(data, { uid = newUid(), rev = 1, now = new Date() } = {}) {
  return { uid, rev, savedAt: now.toISOString(), name: data.name || '', data };
}

// El perfil editado en el archivo: mismo uid, revisión siguiente.
export function edited(entry, data, now = new Date()) {
  return { ...entry, rev: (entry.rev || 1) + 1, savedAt: now.toISOString(), name: data.name || entry.name, data };
}

// Las ids de macro que usan los perfiles del archivo (teclas, mandos, y las
// variaciones no, que viven en el PC). Sin esto, la limpieza de macros sin uso
// las borraría y al activar el perfil en otro PC quedarían teclas muertas.
export function macroIdsUsed(profiles, MACRO_MODIFIER, ROTARY_KEY) {
  const used = new Set();
  const add = (id) => { if (Number.isInteger(id)) used.add(id); };
  for (const e of profiles || []) {
    for (const pg of e.data?.pages || []) {
      for (const a of pg.keys || []) if (a && a.modifier === MACRO_MODIFIER) add(a.keycode);
      for (const a of pg.rotary || []) if (a && a.type === ROTARY_KEY && a.modifier === MACRO_MODIFIER) add(a.keycode);
    }
  }
  return used;
}

// Cuánto ocupa el archivo, para enseñar la barra de memoria sin subirlo.
export async function sizeOf(profiles) {
  return (await encode(profiles)).length;
}
