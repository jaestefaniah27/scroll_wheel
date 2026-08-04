// Copia de seguridad completa del teclado en un archivo del PC.
//
// La memoria del teclado es el único sitio donde viven los perfiles y los
// iconos, y cualquier cambio de formato en el firmware puede dejarlos atrás.
// Esto permite recuperarlos sin volver a dibujarlos.

import * as device from './device.js';
import { state, markDirty, syncFromDevice, pageCountOf } from './store.js';
import { toast } from './ui.js';
import * as cache from './oled-cache.js';

const FORMAT = 'orby-backup';
// v2: número de perfiles variable, mandos con capa SUPER (16 huecos) y rueda de
// scroll por perfil y capa.
const VERSION = 3;   // v3: perfiles con páginas

function bytesToHex(bytes) {
  let hex = '';
  for (const b of bytes) hex += b.toString(16).padStart(2, '0');
  return hex;
}

function hexToBytes(hex) {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.substr(i * 2, 2), 16);
  return out;
}

// Lee todo del dispositivo: perfiles con sus mandos y los bitmaps de cada hueco.
export async function exportAll(onProgress = () => {}) {
  if (!state.connected) throw new Error('Teclado no conectado');

  const total = state.profiles.length || 1;
  const payload = {
    format: FORMAT,
    version: VERSION,
    savedAt: new Date().toISOString(),
    firmware: state.deviceInfo?.fw || null,
    timeout: state.timeout,
    activeProfile: state.activeProfileIdx,
    profiles: [],
  };

  for (let p = 0; p < total; p++) {
    onProgress(`Leyendo perfil ${p + 1} de ${total}…`);
    const prof = await device.getProfile(p);
    const icons = {};

    for (let slot = 0; slot < 20; slot++) {
      if (!(prof.oledMask & (1 << slot))) continue;
      onProgress(`Perfil ${p + 1}: icono ${slot + 1} de 20…`);
      const bytes = await device.getOled(p, slot);
      if (bytes) icons[slot] = bytesToHex(bytes);
    }

    payload.profiles.push({
      name: prof.name,
      // labels/keys/rotary/scroll son accesos a la página que se esté editando,
      // así que se guardan aparte y además se guarda `pages` con TODAS: si no,
      // una copia de seguridad de un perfil de cuatro páginas se llevaría solo
      // una y el usuario no se enteraría hasta querer restaurarla.
      labels: prof.labels,
      keys: prof.keys,
      rotary: prof.rotary,
      scroll: prof.scroll,
      pageCount: prof.pageCount || 1,
      pages: (prof.pages || []).map((pg) => ({
        labels: pg.labels, keys: pg.keys, rotary: pg.rotary, scroll: pg.scroll,
        oledMask: pg.oledMask,
      })),
      // Los iconos son los de la página legible de este perfil (la activa si el
      // perfil está puesto en el teclado, la primera si no): GET_OLED actúa sobre
      // la página activa y no se puede pedir otra sin cambiarla.
      iconsPage: prof.pageIdx || 0,
      icons,
    });
  }

  return payload;
}

// Igual que exportAll pero sin hablar con el teclado: monta la copia con lo que
// la app ya tiene en memoria. Es lo que usa el espejo local (mirror.js) para
// guardarse en disco después de cada cambio, incluso con el Orby desenchufado,
// donde exportAll no puede leer nada.
export function snapshotFromState() {
  return {
    format: FORMAT,
    version: VERSION,
    savedAt: new Date().toISOString(),
    firmware: state.deviceInfo?.fw || null,
    timeout: state.timeout,
    activeProfile: state.activeProfileIdx,
    maxProfiles: state.maxProfiles,
    profiles: state.profiles.map((prof) => ({
      name: prof.name,
      labels: prof.labels,
      keys: prof.keys,
      rotary: prof.rotary,
      scroll: prof.scroll,
      pageCount: prof.pageCount || 1,
      maxPages: prof.maxPages || 1,
      pages: (prof.pages || []).map((pg) => ({
        labels: pg.labels, keys: pg.keys, rotary: pg.rotary, scroll: pg.scroll,
        oledMask: pg.oledMask,
      })),
      iconsPage: prof.pageIdx || 0,
      icons: iconsForPage(prof.idx, prof.pageIdx || 0),
    })),
  };
}

// Los bitmaps de un perfil y una página, tal y como los tiene la caché.
function iconsForPage(profileIdx, page) {
  const icons = {};
  for (let slot = 0; slot < 20; slot++) {
    const bytes = cache.get(profileIdx, slot, page);
    if (bytes) icons[slot] = bytesToHex(bytes);
  }
  return icons;
}

// Vuelca una copia al teclado. Como cualquier otro cambio, queda marcada como
// pendiente y el guardado automático la escribe en Flash sola.
export async function importAll(data, onProgress = () => {}) {
  if (data?.format !== FORMAT) throw new Error('El archivo no es una copia de Orby');
  if (!state.connected) throw new Error('Teclado no conectado');

  const total = Math.min(state.maxProfiles, data.profiles.length);

  // La copia puede traer más perfiles de los que hay ahora en el teclado: se
  // crean antes de escribir nada, porque el firmware rechaza índices que no
  // existen todavía.
  while (state.profiles.length < total) {
    onProgress(`Creando perfil ${state.profiles.length + 1}…`);
    await device.addProfile();
    await syncFromDevice();
  }

  for (let p = 0; p < total; p++) {
    const prof = data.profiles[p];
    onProgress(`Escribiendo perfil ${p + 1} de ${total}…`);

    if (prof.name) await device.setName(p, prof.name);

    // Las páginas que traiga la copia y no existan en el teclado hay que crearlas
    // antes de escribir: el firmware rechaza una página que no está.
    const wanted = Math.min(prof.pageCount || 1, state.maxPages || 1);
    while (pageCountOf(state.profiles[p]) < wanted) {
      onProgress(`Perfil ${p + 1}: creando página ${pageCountOf(state.profiles[p]) + 1}…`);
      await device.addPage(p, false);
      await syncFromDevice();
    }

    for (let s = 0; s < 20; s++) {
      await device.setLabel(p, s, prof.labels?.[s] ?? '');
    }
    for (let s = 0; s < 24; s++) {
      const k = prof.keys?.[s] || { modifier: 0, keycode: 0 };
      await device.setKeymap(p, s, k.modifier, k.keycode);
    }
    // Las copias antiguas traen 8 mandos (solo capa normal) y ninguna rueda por
    // perfil; se escribe lo que haya y el resto conserva lo del teclado.
    for (let s = 0; s < 16; s++) {
      const r = prof.rotary?.[s];
      if (r) await device.setRotary(p, s, r.type, r.modifier, r.keycode);
    }
    for (let layer = 0; layer < 2; layer++) {
      const sc = prof.scroll?.[layer] ?? data.scroll; // v1 guardaba una rueda global
      if (sc?.detentsPerRev) await device.setProfileScroll(p, layer, sc.detentsPerRev, sc.invert);
    }

    const icons = prof.icons || {};
    const slots = Object.keys(icons);
    for (let i = 0; i < slots.length; i++) {
      const slot = Number(slots[i]);
      onProgress(`Perfil ${p + 1}: icono ${i + 1} de ${slots.length}…`);
      await device.uploadOled(p, slot, hexToBytes(icons[slot]));
    }
  }

  // Los iconos del teclado ya no son los que hay en la caché de la app.
  cache.invalidate();
  markDirty();
  await syncFromDevice();
}

// --- Acciones enganchadas a los botones ------------------------------------

export async function runExport(setBusy) {
  try {
    setBusy('Leyendo el teclado…');
    const payload = await exportAll(setBusy);
    setBusy('Guardando archivo…');
    const res = await window.orby.saveBackup(payload);
    if (res.canceled) return;
    if (!res.ok) throw new Error(res.error || 'error desconocido');
    toast('Copia guardada correctamente');
  } catch (err) {
    toast(`No se pudo hacer la copia: ${err.message}`, 'error');
  } finally {
    setBusy(null);
  }
}

export async function runImport(setBusy) {
  try {
    const res = await window.orby.loadBackup();
    if (res.canceled) return;
    if (!res.ok) throw new Error(res.error || 'error desconocido');

    const count = res.data?.profiles?.length ?? 0;
    if (!confirm(`Se sobrescribirán los ${count} perfiles del teclado con la copia del `
               + `${new Date(res.data.savedAt).toLocaleString('es-ES')}.\n\n`
               + 'El cambio no será permanente hasta que pulses "Guardar en Flash".\n\n¿Continuar?')) return;

    setBusy('Restaurando…');
    await importAll(res.data, setBusy);
    toast('Copia restaurada; pulsa Guardar en Flash para fijarla');
  } catch (err) {
    toast(`No se pudo restaurar: ${err.message}`, 'error');
  } finally {
    setBusy(null);
  }
}
