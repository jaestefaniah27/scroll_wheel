// Pestaña "Grabar" del inspector de tecla.

import * as device from '../../device.js';
import { state, markDirty, keymapSlot, labelSlot } from '../../store.js';
import { MACRO_MODIFIER } from '../../hid-keys.js';
import { toast } from '../../ui.js';
import { icon } from '../../icons.js';
import * as cache from '../../oled-cache.js';
import * as fb from '../../oled-fb.js';
import * as variants from '../../variants.js';
import { view, currentProfile, selectedKeyIndex, isActiveView, currentMacroId,
         rerender } from './view-state.js';
import { macroById, nextMacroId, ensureMacro, savePCMacros, removeMacro, allMacros,
         loadPCMacros, isRecordingMacro, isResetMacro, isRecordOrResetMacro } from './macros-store.js';
import { applyKeymap } from './keys.js';

// --- Pestaña "Grabar": capturar una operación y repetirla -------------------
// A diferencia de una secuencia, aquí no se monta nada paso a paso: el proceso
// principal engancha el ratón y el teclado del PC (src-tauri/src/recorder.rs) y
// guarda lo que el usuario haga, con sus tiempos. La tecla hace de interruptor:
// pulsar empieza a grabar, pulsar otra vez para y guarda, y a partir de ahí
// cada pulsación reproduce lo grabado.

export const RECORD_MODES = [
  { id: 'once', label: 'Una vez',           desc: 'Reproduce la operación de principio a fin y para.' },
  { id: 'loop', label: 'En bucle',          desc: 'Repite sin parar hasta que vuelvas a pulsar la tecla.' },
  { id: 'hold', label: 'Mientras se pulsa', desc: 'Repite mientras mantengas la tecla, y para al soltarla.' },
];

// Multiplicador de velocidad al reproducir. x3 por defecto: para repetir algo
// varias veces, reproducirlo a la velocidad original de grabación sobra.
export const RECORD_SPEEDS = [1, 2, 3, 5];
export const RECORD_SPEED_DEFAULT = 3;

// En qué punto está la grabación, según lo que avise el proceso principal.
export let recordPhase = { id: null, phase: 'idle' };

// Lo que la pantalla de la tecla enseñaba antes de ponerle el aviso, para poder
// devolvérselo. `bytes` a null = ese hueco no tenía icono propio.
export let screenBackup = null;
export let restoreTimer = null;

// Texto que enseña la pantalla de la tecla en cada situación. La tecla se
// explica sola: qué hace ahora mismo y qué hace con SUPER.
export const RECORD_ICON_EMPTY = 'RECORD';
export const RECORD_ICON_READY = 'PLAY';
export const RECORD_ICON_RESET = 'RESET';

// La macro de grabación de un hueco. La tecla de borrado (capa SUPER) apunta a
// la suya con `target`, así que desde ella también se llega a la grabación.
export function recordMacro(id) {
  const m = id === null ? null : macroById(id);
  if (m?.kind === 'recording') return m;
  if (m?.kind === 'recording-reset') return macroById(m.target) || null;
  return null;
}

// ¿Tiene esta grabación su tecla de borrado montada en la otra capa?
export function hasResetKey(recId) {
  const pos = keyPosForMacro(recId);
  if (!pos) return false;
  const other = pos.layer === 'normal' ? 'super' : 'normal';
  const pair = state.profiles[pos.profile]?.keys[keymapSlot(pos.key, other)];
  return pair?.modifier === MACRO_MODIFIER && macroById(pair.keycode)?.target === recId;
}

export function recordDuration(m) {
  const events = m?.events || [];
  return events.length ? events[events.length - 1].t : 0;
}

// Dónde está la tecla que dispara una macro: perfil, número de tecla y capa.
// Hace falta para pintar su pantalla y la de su pareja en la otra capa.
export function keyPosForMacro(id) {
  for (let p = 0; p < state.profiles.length; p++) {
    const prof = state.profiles[p];
    for (const layer of ['normal', 'super']) {
      for (let i = 0; i < 12; i++) {
        const a = prof.keys[keymapSlot(i, layer)];
        if (a?.modifier === MACRO_MODIFIER && a.keycode === id && labelSlot(i, layer) >= 0) {
          return { profile: p, key: i, layer };
        }
      }
    }
  }
  return null;
}

export function keyScreenForMacro(id) {
  const pos = keyPosForMacro(id);
  return pos ? { profile: pos.profile, slot: labelSlot(pos.key, pos.layer) } : null;
}

// Escribe un icono definitivo (a diferencia de los avisos pasajeros de
// showKeyStatus): va a la caché y al mapa de iconos del perfil, y cuenta como
// cambio pendiente para que el guardado automático lo lleve a la Flash.
export async function writeKeyIcon(profileIdx, lslot, text) {
  if (lslot < 0) return;
  const bytes = statusFrame(text);
  await queueIconWrite(async () => {
    try {
      await device.uploadOled(profileIdx, lslot, bytes);
      cache.set(profileIdx, lslot, Uint8Array.from(bytes));
      const prof = state.profiles[profileIdx];
      if (prof) prof.oledMask |= (1 << lslot);
      markDirty();
    } catch {
      toast('No se pudo escribir el icono de la tecla', 'error');
    }
  });
}

// Deja las dos pantallas de la pareja como toca: RECORD o PLAY en la capa
// normal según haya algo grabado, y RESET en la de SUPER si esa tecla lleva el
// borrado. Se llama cada vez que la grabación cambia de estado.
export async function refreshRecordIcons(recId) {
  const m = macroById(recId);
  const pos = keyPosForMacro(recId);
  if (!m || !pos) return;

  await writeKeyIcon(pos.profile, labelSlot(pos.key, pos.layer),
                     m.events?.length ? RECORD_ICON_READY : RECORD_ICON_EMPTY);

  // La pareja de borrado solo existe si se creó (ver attachResetKey): con la
  // grabación puesta en la capa SUPER no hay otra capa donde ponerla.
  const other = pos.layer === 'normal' ? 'super' : 'normal';
  const prof = state.profiles[pos.profile];
  const pair = prof?.keys[keymapSlot(pos.key, other)];
  if (pair?.modifier === MACRO_MODIFIER && macroById(pair.keycode)?.target === recId) {
    await writeKeyIcon(pos.profile, labelSlot(pos.key, other), RECORD_ICON_RESET);
  }
}

// Monta la tecla de borrado en la capa SUPER de la misma tecla: pulsar
// SUPER + esa tecla tira lo grabado y la deja lista para grabar otra vez.
// Solo tiene sentido creando la grabación en la capa normal; si se crea ya en
// SUPER no hay otra capa libre donde ponerla y se avisa en el editor.
export async function attachResetKey(recId) {
  const prof = currentProfile();
  const i = selectedKeyIndex();
  if (!prof || i === null || view.layer !== 'normal') return;

  const slot = keymapSlot(i, 'super');
  const resetId = nextMacroId();
  const reset = ensureMacro(resetId);
  reset.kind = 'recording-reset';
  reset.target = recId;
  reset.actions = [];   // sin pasos: el teclado avisa por CDC en vez de tocarla

  const m = macroById(recId);
  if (m) m.resetId = resetId;
  savePCMacros(resetId);

  prof.keys[slot] = { modifier: MACRO_MODIFIER, keycode: resetId };
  try {
    await device.setKeymap(view.editingProfile, slot, MACRO_MODIFIER, resetId);
    markDirty();
    await variants.reassertSlot('keys', slot);
  } catch {
    toast('No se pudo asignar la tecla de borrado en SUPER', 'error');
  }
}

// Al dejar de ser una grabación (se cambia de pestaña, o se quita la tecla) hay
// que llevarse por delante también su tecla de borrado: si no, la capa SUPER
// se queda con un RESET que ya no borra nada.
export async function detachResetKey(recId) {
  const m = macroById(recId);
  const pos = keyPosForMacro(recId);
  if (!m || !pos) return;

  const other = pos.layer === 'normal' ? 'super' : 'normal';
  const slot = keymapSlot(pos.key, other);
  const prof = state.profiles[pos.profile];
  const pair = prof?.keys[slot];
  if (pair?.modifier !== MACRO_MODIFIER || macroById(pair.keycode)?.target !== recId) return;

  removeMacro(pair.keycode);

  prof.keys[slot] = { modifier: 0, keycode: 0 };
  try {
    await device.setKeymap(pos.profile, slot, 0, 0);
    await device.clearOled(pos.profile, labelSlot(pos.key, other));
    cache.set(pos.profile, labelSlot(pos.key, other), null);
    prof.oledMask &= ~(1 << labelSlot(pos.key, other));
    markDirty();
  } catch {
    toast('No se pudo quitar la tecla de borrado de SUPER', 'error');
  }
}

// Un cartel a pantalla completa con el texto centrado y el marco de siempre.
export function statusFrame(text) {
  const source = fb.makeTextSource(text, { fontSize: 20, bold: true, font: 'Segoe UI Black' });
  const scale = fb.fitScale(source);
  const size = fb.measureSource(source);
  const raster = fb.rasterizeLayer(source, {
    x: Math.round((fb.OLED_W - size.width * scale) / 2),
    y: Math.round((fb.OLED_H - size.height * scale) / 2),
    scale, threshold: 128, blur: 0, dither: false, invert: 'none',
  });
  fb.drawFrame(raster);
  return raster;
}

// Subir un icono son cuatro OLED_CHUNK seguidos contra el mismo hueco. Si dos
// escrituras se solapan, sus trozos se entrelazan y el teclado acaba pintando
// una mezcla de los dos dibujos: es lo que pasaba al reiniciar una grabación
// mientras se estaba reproduciendo (llega "reset" y, en mitad de su escritura,
// el "idle" de la reproducción cortada empieza otra). Se encolan para que cada
// una termine antes de que empiece la siguiente.
export let iconQueue = Promise.resolve();

export function queueIconWrite(run) {
  const next = iconQueue.then(run, run);
  iconQueue = next.catch(() => {});
  return next;
}

// El aviso se manda a la RAM del teclado y NO se marca como cambio pendiente:
// así el guardado automático no se lleva un "REC" a la Flash y el icono de
// verdad sigue siendo el que estaba.
export async function showKeyStatus(id, text) {
  clearTimeout(restoreTimer);
  if (!state.connected) return;
  const target = keyScreenForMacro(id);
  if (!target) return;

  if (!screenBackup) {
    const existing = cache.get(target.profile, target.slot);
    screenBackup = { ...target, bytes: existing ? Uint8Array.from(existing) : null };
  }
  const bytes = statusFrame(text);
  await queueIconWrite(async () => {
    try {
      await device.uploadOled(target.profile, target.slot, bytes);
    } catch { /* si no llega, el aviso se queda solo en la app */ }
  });
}

export async function restoreKeyStatus() {
  clearTimeout(restoreTimer);
  const saved = screenBackup;
  screenBackup = null;
  if (!saved || !state.connected) return;
  await queueIconWrite(async () => {
    try {
      if (saved.bytes) await device.uploadOled(saved.profile, saved.slot, saved.bytes);
      else await device.clearOled(saved.profile, saved.slot);
    } catch { /* al reconectar se relee todo de todas formas */ }
  });
}

// Avisos del proceso principal: grabando / guardada / reproduciendo / parada.
export async function onRecorderState({ id, phase }) {
  recordPhase = { id, phase };

  if (phase === 'recording') {
    await showKeyStatus(id, 'REC');
  } else if (phase === 'playing') {
    await showKeyStatus(id, 'RUN');
  } else if (phase === 'saved' || phase === 'empty') {
    // El proceso principal ha escrito los eventos en la configuración: hay que
    // releerla antes de volver a guardarla desde aquí, o se perdería.
    await loadPCMacros();
    await showKeyStatus(id, phase === 'saved' ? 'OK' : 'VACIO');
    restoreTimer = setTimeout(async () => {
      recordPhase = { id: null, phase: 'idle' };
      // No se devuelve el icono anterior: ahora hay algo grabado, así que la
      // tecla pasa a poner PLAY. De eso se encarga refreshRecordIcons.
      screenBackup = null;
      await refreshRecordIcons(id);
      if (isActiveView()) rerender();
    }, 1200);
  } else if (phase === 'reset') {
    // Se ha pulsado SUPER + la tecla: lo grabado se ha ido y vuelve a RECORD.
    await loadPCMacros();
    recordPhase = { id: null, phase: 'idle' };
    screenBackup = null;
    await refreshRecordIcons(id);
    toast('Grabación borrada: la tecla vuelve a estar lista para grabar');
  } else {
    await restoreKeyStatus();
  }

  if (isActiveView()) rerender();
}

export function setRecordMode(mode) {
  const m = recordMacro(currentMacroId());
  if (!m) return;
  m.mode = mode;
  savePCMacros(m.id);
  rerender();
}

export function setRecordSpeed(speed) {
  const m = recordMacro(currentMacroId());
  if (!m) return;
  m.speed = Number(speed) || RECORD_SPEED_DEFAULT;
  savePCMacros(m.id);
  rerender();
}

export async function clearRecording() {
  const m = recordMacro(currentMacroId());
  if (!m || !m.events?.length) return;
  if (!confirm('Se borrará la operación grabada en esta tecla.\n\n¿Continuar?')) return;
  m.events = [];
  savePCMacros(m.id);
  await refreshRecordIcons(m.id);  // vuelve a RECORD
  rerender();
}

export async function toggleRecording() {
  const m = recordMacro(currentMacroId());
  if (!m) return;

  // Con algo grabado, la tecla reproduce en vez de grabar (es lo que se espera
  // de ella el 99% de las veces), así que "grabar de nuevo" tiene que vaciarla
  // antes: es lo mismo que hace SUPER + la tecla. Los eventos los lee el
  // proceso principal de la configuración del PC, así que hay que esperar a que
  // quede escrita o leería todavía los viejos y se pondría a reproducir.
  if (recordPhase.phase !== 'recording' && m.events?.length) {
    if (!confirm('Se sustituirá la operación grabada en esta tecla.\n\n¿Grabar de nuevo?')) return;
    m.events = [];
    await window.orby.setConfig({ macros: allMacros() });
    await refreshRecordIcons(m.id);
    rerender();
  }

  window.orby.recorder.toggle(m.id);
}

export function renderRecordTab(macroId) {
  const m = recordMacro(macroId);
  if (!m) {
    return '<p class="setting-desc">Preparando la grabación de esta tecla…</p>';
  }

  // Este hueco es la pareja de borrado, no la grabación en sí.
  if (isResetMacro(macroId)) {
    const pos = keyPosForMacro(m.id);
    const events = m.events || [];
    return `
      <div class="field">
        <span class="field-label">Borrar la grabación</span>
        <div class="rec-state ${events.length ? 'ok' : 'off'}">
          ${pos ? `Esta tecla borra lo grabado en la tecla ${pos.key + 1} de la capa normal.` : 'Tecla de borrado.'}
          ${events.length ? ` Ahora mismo hay ${events.length} eventos guardados.` : ' Ahora mismo no hay nada que borrar.'}
        </div>
      </div>
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-clear" ${events.length ? '' : 'disabled'}>
          ${icon('trash', 16)} Borrar ahora
        </button>
      </div>
      <p class="setting-desc">
        Se creó sola al convertir la otra tecla en grabación. Para quitarla, cambia esa tecla a
        cualquier otra pestaña que no sea "Grabar".
      </p>`;
  }

  const events = m.events || [];
  const phase = recordPhase.id === m.id ? recordPhase.phase : 'idle';
  const recording = phase === 'recording';
  const playing = phase === 'playing';
  const mode = m.mode || 'once';
  const speed = m.speed || RECORD_SPEED_DEFAULT;
  const seconds = (recordDuration(m) / 1000).toFixed(1);

  return `
    <div class="field">
      <span class="field-label">Operación grabada</span>
      <div class="rec-state ${recording ? 'is-rec' : (events.length ? 'ok' : 'off')}">
        ${recording
          ? 'Grabando… haz la operación y vuelve a pulsar la tecla (o el botón de abajo) para terminar.'
          : events.length
            ? `${events.length} eventos · ${seconds} s de ratón y teclado`
            : 'Todavía no hay nada grabado en esta tecla.'}
      </div>
    </div>

    <div class="inspector-actions">
      <button class="primary-btn ${recording ? 'is-capturing' : ''}" data-act="rec-toggle">
        ${icon(recording ? 'square' : 'oled', 16)}
        ${recording ? 'Terminar grabación' : (events.length ? 'Grabar de nuevo' : 'Empezar a grabar')}
      </button>
      <button class="secondary-btn danger" data-act="rec-clear" ${events.length ? '' : 'disabled'}>
        ${icon('trash', 16)} Borrar grabación
      </button>
    </div>

    <p class="setting-desc">
      Se captura lo que hagas con el ratón y el teclado del PC, con sus tiempos. Mientras grabas, la
      pantalla de la tecla pone <strong>REC</strong>; al terminar, <strong>OK</strong>. Para parar de
      grabar vale tanto la propia tecla como el botón de aquí arriba.
      ${events.length ? 'Con algo grabado, pulsar la tecla lo reproduce; para cambiarlo, usa "Grabar de nuevo".' : ''}
    </p>

    <div class="rec-state off">
      La pantalla de esta tecla pone <strong>${events.length ? RECORD_ICON_READY : RECORD_ICON_EMPTY}</strong>
      ${hasResetKey(m.id)
        ? `, y con SUPER pone <strong>${RECORD_ICON_RESET}</strong>: <strong>SUPER + esta tecla</strong>
           borra lo grabado y la deja lista para grabar otra vez.`
        : `. La tecla de borrado automática solo se monta al crear la grabación en la capa NORMAL:
           en SUPER no queda otra capa donde ponerla.`}
    </div>

    <div class="field mt-4">
      <span class="field-label">Al reproducir</span>
      <div class="chip-row">
        ${RECORD_MODES.map((r) => `
          <button class="chip ${mode === r.id ? 'on' : ''}" data-act="rec-mode" data-mode="${r.id}">${r.label}</button>`).join('')}
      </div>
      <p class="setting-desc">${RECORD_MODES.find((r) => r.id === mode)?.desc || ''}</p>
    </div>

    <div class="field mt-4">
      <span class="field-label">Velocidad de reproducción</span>
      <div class="chip-row">
        ${RECORD_SPEEDS.map((s) => `
          <button class="chip ${speed === s ? 'on' : ''}" data-act="rec-speed" data-speed="${s}">${s}x</button>`).join('')}
      </div>
      <p class="setting-desc">Por defecto x3: reproduce tres veces más rápido que como se grabó.</p>
    </div>

    ${playing ? `
      <div class="inspector-actions">
        <button class="secondary-btn danger" data-act="rec-stop">${icon('square', 16)} Parar reproducción</button>
      </div>` : ''}

    <p class="setting-desc">
      Esta acción la ejecuta siempre el PC (el teclado no puede saber dónde está el ratón), así que
      OrbyGUI tiene que estar abierto —vale con el icono de la bandeja— para que funcione.
    </p>`;
}

// Convierte la tecla seleccionada en una tecla de grabación: la macro, la
// tecla de borrado en la capa SUPER y los iconos que explican qué hace cada
// una (RECORD / RESET, y PLAY en cuanto haya algo grabado).
export async function startRecordingKey() {
  const id = nextMacroId();
  const m = ensureMacro(id);
  m.kind = 'recording';
  m.mode = 'once';
  m.events = [];
  // Sin pasos: así macroDeviceEligible (macros-store.js) la descarta y el teclado avisa siempre
  // por CDC en vez de intentar tocarla él, que no puede.
  m.actions = [];
  savePCMacros(id);

  await applyKeymap(MACRO_MODIFIER, id);
  await attachResetKey(id);
  await refreshRecordIcons(id);
  rerender();
}
