// Perfiles desactivados: el teclado lleva como mucho nueve activos y el resto
// se guardan comprimidos en su propia Flash (PSTORE_*, firmware 4.27), de modo
// que otro PC los ve y los puede activar tal y como se dejaron.
//
// Para el firmware el archivo es un bloque opaco. Aquí se decide qué lleva
// dentro (web/pstore-blob.mjs), cómo se desactiva un perfil y cómo se vuelve a
// activar. La regla que sostiene todo: el perfil nunca se quita de un sitio
// hasta que está verificado en el otro, así que un fallo a medias deja un
// duplicado, nunca una pérdida.

import * as device from '../../device.js';
import { state, markDirty, syncFromDevice, withPinnedPage } from '../../store.js';
import { writeAllPages } from '../../profile-library.js';
import * as cache from '../../oled-cache.js';
import * as variants from '../../variants.js';
import * as compat from '../../compat.js';
import { encode, decode, crc32, entryFrom } from '../../web/pstore-blob.mjs';
import { archived, setArchived } from './archive-state.js';
import { rescueRetiredMacros } from './macros-store.js';

export { archived };

const CHUNK = 192;             // == MStore::CHUNK en macro_store.h
const PROFILE_ICON_SLOT = device.PROFILE_ICON_SLOT;

export function supported() {
  return state.connected && compat.supports(state.deviceInfo, 'profileArchive');
}

const toHex = (bytes) => Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('');
const fromHex = (hex) => Uint8Array.from(hex.match(/../g) || [], (h) => parseInt(h, 16));

async function readBytes(len) {
  const out = new Uint8Array(len);
  for (let off = 0; off < len; off += CHUNK) {
    out.set(await device.pstoreGet(off, Math.min(CHUNK, len - off)), off);
  }
  return out;
}

// Lee el archivo del teclado y lo deja en memoria. Es la copia buena: lo que se
// activa o se borra siempre parte de una lectura reciente, no de lo que hubiera
// en memoria, para no pisar lo que haya hecho otro PC.
export async function load() {
  if (!supported()) { setArchived(null); return []; }
  const info = await device.pstoreInfo();
  let list = [];
  if (info.len > 0) {
    const bytes = await readBytes(info.len);
    if (crc32(bytes) !== info.crc) throw new Error('el archivo de perfiles llegó dañado del teclado');
    list = await decode(bytes);
  }
  setArchived(list);
  return list;
}

// Cuánto ocupa lo archivado y cuánto cabe, para la barra de memoria.
export async function usage() {
  const info = await device.pstoreInfo();
  return { used: info.len, max: info.max };
}

// Escribe la lista entera. El firmware solo da por buena la copia nueva si su
// CRC cuadra, y la anterior sigue intacta hasta entonces.
async function save(list) {
  const bytes = await encode(list);
  const crc = crc32(bytes);
  const info = await device.pstoreInfo();
  if (bytes.length > info.max) {
    throw new Error(`los perfiles archivados ocuparían ${Math.ceil(bytes.length / 1024)} KB y `
                    + `en el teclado caben ${Math.floor(info.max / 1024)} KB`);
  }
  if (!(info.len === bytes.length && info.crc === crc)) {
    await device.pstoreBegin(bytes.length, crc);
    for (let off = 0; off < bytes.length; off += CHUNK) {
      await device.pstoreData(off, toHex(bytes.subarray(off, off + CHUNK)));
    }
    await device.pstoreEnd();
  }
  // Releído, no dado por supuesto: es lo que decide si se puede quitar el
  // perfil del sitio de donde viene.
  const after = await device.pstoreInfo();
  if (after.len !== bytes.length || after.crc !== crc) {
    throw new Error('el teclado no ha guardado el archivo de perfiles');
  }
  setArchived(list);
}

// La foto de un perfil activo, con los iconos de TODAS sus páginas (la copia de
// seguridad solo guarda los de una).
export async function snapshot(idx, onProgress = () => {}) {
  const prof = await device.getProfile(idx);
  const pages = [];
  for (let pg = 0; pg < prof.pages.length; pg++) {
    const src = prof.pages[pg];
    const icons = {};
    for (let slot = 0; slot < 20; slot++) {
      if (!(src.oledMask & (1 << slot))) continue;
      onProgress(`Página ${pg + 1}: icono ${slot + 1} de 20…`);
      const bytes = await device.getOledPage(idx, pg, slot);
      if (bytes) icons[slot] = toHex(bytes);
    }
    pages.push({ labels: src.labels, keys: src.keys, rotary: src.rotary, scroll: src.scroll,
                 oledMask: src.oledMask, icons });
  }
  let profileIcon = null;
  if ((prof.pages[0]?.oledMask || 0) & (1 << PROFILE_ICON_SLOT)) {
    const bytes = await device.getOledPage(idx, 0, PROFILE_ICON_SLOT);
    if (bytes) profileIcon = toHex(bytes);
  }
  return { name: prof.name, pageCount: prof.pageCount || 1, pages, profileIcon };
}

// Saca un perfil del teclado y lo guarda comprimido. Devuelve la entrada.
export async function deactivate(idx, onProgress = () => {}) {
  if (!supported()) throw new Error('El teclado necesita el firmware 4.27 o posterior');
  if (!state.profilesComplete) throw new Error('Espera a que se lean todos los perfiles');
  if (state.profiles.length < 2) throw new Error('El teclado tiene que quedarse con un perfil como mínimo');

  const list = await load();
  onProgress('Leyendo el perfil…');
  const data = await snapshot(idx, onProgress);
  const entry = entryFrom(data);

  onProgress('Guardándolo comprimido en el teclado…');
  await save([...list, entry]);

  // Ya está a salvo: ahora sí se quita de los activos.
  onProgress('Quitándolo de los perfiles activos…');
  await device.delProfile(idx);
  variants.shiftProfiles(idx);
  cache.invalidate();
  await syncFromDevice();
  markDirty();
  return entry;
}

// Vuelve a poner en el teclado un perfil archivado, exactamente como estaba.
export async function activate(uid, onProgress = () => {}) {
  if (!supported()) throw new Error('El teclado necesita el firmware 4.27 o posterior');
  if (state.profiles.length >= state.maxProfiles) {
    throw new Error(`El teclado ya tiene ${state.maxProfiles} perfiles activos: desactiva alguno antes`);
  }
  const list = await load();
  const entry = list.find((e) => e.uid === uid);
  if (!entry) throw new Error('Ese perfil ya no está en el archivo del teclado');
  const data = entry.data;

  onProgress('Creando el perfil…');
  const ret = await device.addProfile();
  cache.invalidate();
  await syncFromDevice();
  const idx = Number.isInteger(ret) && ret < state.profiles.length ? ret : state.profiles.length - 1;

  if (data.name) await device.setName(idx, data.name);
  await writeAllPages(idx, data.pages, onProgress);

  const wanted = Math.min(data.pages.length, state.maxPages || 1);
  for (let pg = 0; pg < wanted; pg++) {
    const slots = Object.keys(data.pages[pg].icons || {});
    await withPinnedPage(idx, pg, async () => {
      for (let i = 0; i < slots.length; i++) {
        onProgress(`Página ${pg + 1}: icono ${i + 1} de ${slots.length}…`);
        await device.uploadOled(idx, Number(slots[i]), fromHex(data.pages[pg].icons[slots[i]]));
      }
    });
  }
  if (data.profileIcon) await device.uploadOled(idx, PROFILE_ICON_SLOT, fromHex(data.profileIcon));

  await device.setPage(0);
  cache.invalidate();
  markDirty();
  await syncFromDevice();

  // Solo ahora, con el perfil ya en el teclado, sale del archivo.
  await save(list.filter((e) => e.uid !== uid));
  await rescueRetiredMacros().catch(() => {});
  return idx;
}

// Borra un perfil archivado para siempre.
export async function discard(uid) {
  const list = await load();
  await save(list.filter((e) => e.uid !== uid));
}
