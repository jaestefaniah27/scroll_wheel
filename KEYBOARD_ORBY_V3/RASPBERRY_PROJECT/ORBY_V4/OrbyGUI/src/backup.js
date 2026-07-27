// Copia de seguridad completa del teclado en un archivo del PC.
//
// La memoria del teclado es el único sitio donde viven los perfiles y los
// iconos, y cualquier cambio de formato en el firmware puede dejarlos atrás.
// Esto permite recuperarlos sin volver a dibujarlos.

import * as device from './device.js';
import { state, markDirty, syncFromDevice } from './store.js';
import { toast } from './ui.js';

const FORMAT = 'orby-backup';
const VERSION = 1;

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

  const payload = {
    format: FORMAT,
    version: VERSION,
    savedAt: new Date().toISOString(),
    firmware: state.deviceInfo?.fw || null,
    scroll: { ...state.scroll },
    brightness: state.brightness,
    timeout: state.timeout,
    activeProfile: state.activeProfileIdx,
    profiles: [],
  };

  for (let p = 0; p < 4; p++) {
    onProgress(`Leyendo perfil ${p + 1} de 4…`);
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
      labels: prof.labels,
      keys: prof.keys,
      rotary: prof.rotary,
      icons,
    });
  }

  return payload;
}

// Vuelca una copia al teclado. No escribe en Flash: eso lo decide el usuario.
export async function importAll(data, onProgress = () => {}) {
  if (data?.format !== FORMAT) throw new Error('El archivo no es una copia de Orby');
  if (!state.connected) throw new Error('Teclado no conectado');

  for (let p = 0; p < Math.min(4, data.profiles.length); p++) {
    const prof = data.profiles[p];
    onProgress(`Escribiendo perfil ${p + 1} de 4…`);

    if (prof.name) await device.setName(p, prof.name);

    for (let s = 0; s < 20; s++) {
      await device.setLabel(p, s, prof.labels?.[s] ?? '');
    }
    for (let s = 0; s < 24; s++) {
      const k = prof.keys?.[s] || { modifier: 0, keycode: 0 };
      await device.setKeymap(p, s, k.modifier, k.keycode);
    }
    // Los mandos giratorios no existían en las copias de la versión anterior.
    for (let s = 0; s < 8; s++) {
      const r = prof.rotary?.[s];
      if (r) await device.setRotary(p, s, r.type, r.modifier, r.keycode);
    }

    const icons = prof.icons || {};
    const slots = Object.keys(icons);
    for (let i = 0; i < slots.length; i++) {
      const slot = Number(slots[i]);
      onProgress(`Perfil ${p + 1}: icono ${i + 1} de ${slots.length}…`);
      await device.uploadOled(p, slot, hexToBytes(icons[slot]));
    }
  }

  if (data.scroll?.detentsPerRev) await device.setScroll(data.scroll.detentsPerRev);
  if (Number.isInteger(data.brightness)) await device.setBrightness(data.brightness);

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
