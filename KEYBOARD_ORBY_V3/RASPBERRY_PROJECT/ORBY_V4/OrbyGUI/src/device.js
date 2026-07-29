// Capa sobre window.orby: convierte el flujo de líneas del CDC en eventos con
// nombre y añade peticiones con respuesta (promesas) sobre un protocolo que por
// sí mismo no correlaciona petición y respuesta.

const listeners = new Map();

export function on(event, handler) {
  if (!listeners.has(event)) listeners.set(event, new Set());
  listeners.get(event).add(handler);
  return () => listeners.get(event).delete(handler);
}

function emit(event, payload) {
  const set = listeners.get(event);
  if (set) for (const fn of set) fn(payload);
}

// --- Peticiones pendientes -------------------------------------------------
// Cada entrada declara qué línea la resuelve y, opcionalmente, un acumulador
// para respuestas multilínea (GET_PROFILE, GET_STATE).
const pending = [];

function settlePending(line) {
  for (let i = 0; i < pending.length; i++) {
    const req = pending[i];
    if (req.collect && req.collect(line)) return true; // consumida, sigue abierta
    if (req.match(line)) {
      pending.splice(i, 1);
      clearTimeout(req.timer);
      req.resolve(req.result !== undefined ? req.result : line);
      return true;
    }
  }
  return false;
}

function request(command, { match, collect, result, timeout = 4000 } = {}) {
  window.orby.sendCommand(command);
  emit('tx', command);

  if (!match) return Promise.resolve(null);

  return new Promise((resolve, reject) => {
    const req = { match, collect, resolve, result };
    req.timer = setTimeout(() => {
      const i = pending.indexOf(req);
      if (i >= 0) pending.splice(i, 1);
      reject(new Error(`Sin respuesta a "${command}"`));
    }, timeout);
    pending.push(req);
  });
}

// --- API pública -----------------------------------------------------------

export function send(command) {
  window.orby.sendCommand(command);
  emit('tx', command);
}

export const setProfile   = (idx) => request(`SET_PROFILE:${idx}`, { match: (l) => l.startsWith('PROFILE:OK:') });
export const setBrightness = (v)  => request(`SET_BRIGHTNESS:${v}`, { match: (l) => l.startsWith('BRIGHTNESS:OK:') });
export const setTimeout_   = (v)  => request(`SET_TIMEOUT:${v}`,    { match: (l) => l.startsWith('TIMEOUT:OK:') });
export const saveToFlash   = ()   => request('SAVE_STATE',          { match: (l) => l.startsWith('SAVE:OK'), timeout: 8000 });
export const resetDefaults = ()   => request('RESET_DEFAULTS',      { match: (l) => l.startsWith('RESET:OK') });

// Sensibilidad e inversión de la rueda para un perfil y una capa concretos
// (capa 0 = normal, 1 = SUPER). Desde el firmware 3.0 la rueda es un ajuste más
// del perfil, no un valor global del teclado.
export const setProfileScroll = (profile, layer, detents, invert) =>
  request(`SET_PSCROLL:${profile}:${layer}:${detents}:${invert ? 1 : 0}`,
          { match: (l) => l.startsWith(`PSCROLL:OK:${profile}:${layer}:`) });

export const setLabel = (profile, slot, text) =>
  request(`SET_LABEL:${profile}:${slot}:${text}`, { match: (l) => l === `LABEL:OK:${profile}:${slot}` });

export const setKeymap = (profile, slot, modifier, keycode) =>
  request(`SET_KEYMAP:${profile}:${slot}:${modifier}:${keycode}`, { match: (l) => l === `KEYMAP:OK:${profile}:${slot}` });

export const setRotary = (profile, slot, type, modifier, keycode) =>
  request(`SET_ROTARY:${profile}:${slot}:${type}:${modifier}:${keycode}`,
          { match: (l) => l === `ROTARY:OK:${profile}:${slot}` });

export const setName = (profile, text) =>
  request(`SET_NAME:${profile}:${text}`, { match: (l) => l === `NAME:OK:${profile}` });

export const clearOled = (profile, slot) =>
  request(`OLED_CLEAR:${profile}:${slot}`, { match: (l) => l.startsWith(`OLED:CLEARED:${profile}:`) });

// --- Alta y baja de perfiles ------------------------------------------------
// El teclado responde con PROFILE:ADDED/DELETED y, acto seguido, con el recuento
// nuevo. Basta con esperar la primera línea: quien llama vuelve a sincronizar.
function profileMutation(command) {
  return request(command, {
    timeout: 6000,
    match: (l) => l.startsWith('PROFILE:ADDED:') || l.startsWith('PROFILE:DELETED:') || l.startsWith('ERR:'),
  }).then((line) => {
    if (line.startsWith('ERR:')) throw new Error(line.slice(4));
    return parseInt(line.slice(line.lastIndexOf(':') + 1), 10);
  });
}

export const addProfile = ()    => profileMutation('ADD_PROFILE');
export const dupProfile = (idx) => profileMutation(`DUP_PROFILE:${idx}`);
export const delProfile = (idx) => profileMutation(`DEL_PROFILE:${idx}`);

export function getState() {
  const state = {};
  return request('GET_STATE', {
    collect: (line) => {
      if (line.startsWith('STATE:PROFILE:'))    { state.profile   = parseInt(line.slice(14), 10); return true; }
      if (line.startsWith('STATE:PROFILES:')) {
        const [count, max] = line.slice(15).split(':').map(Number);
        state.profileCount = count;
        state.maxProfiles  = max;
        return true;
      }
      if (line.startsWith('STATE:BRIGHTNESS:')) { state.brightness = parseInt(line.slice(17), 10); return true; }
      if (line.startsWith('STATE:TIMEOUT:'))    { state.timeout   = parseInt(line.slice(14), 10); return true; }
      if (line.startsWith('STATE:MODE:'))       { state.mode      = line.slice(11); return true; }
      if (line.startsWith('STATE:SUPER:'))      { state.superActive = line.slice(12) === '1'; return true; }
      if (line.startsWith('SCROLL:OK:')) {
        const [det, inv, hires] = line.slice(10).split(':').map(Number);
        state.scroll = { detentsPerRev: det, invert: !!inv, hires: !!hires };
        return true;
      }
      return false;
    },
    match: (line) => line === 'STATE:END',
    result: state,
  });
}

// Lee un perfil completo del firmware: nombre, 20 etiquetas, 24 acciones de
// tecla, 16 de mando (8 por capa), la rueda de cada capa y la máscara de iconos.
export function getProfile(idx) {
  const prof = {
    idx, name: '', labels: new Array(20).fill(''), keys: [], rotary: [],
    scroll: [{ detentsPerRev: 60, invert: false }, { detentsPerRev: 60, invert: false }],
    oledMask: 0,
  };
  for (let i = 0; i < 24; i++) prof.keys.push({ modifier: 0, keycode: 0 });
  for (let i = 0; i < 16; i++) prof.rotary.push({ type: 0, modifier: 0, keycode: 0 });

  const prefix = `PROF:${idx}:`;
  return request(`GET_PROFILE:${idx}`, {
    timeout: 8000,
    collect: (line) => {
      if (!line.startsWith(prefix)) return false;
      const rest = line.slice(prefix.length);

      if (rest.startsWith('NAME:')) { prof.name = rest.slice(5); return true; }
      if (rest.startsWith('LBL:')) {
        const body = rest.slice(4);
        const sep = body.indexOf(':');
        prof.labels[parseInt(body.slice(0, sep), 10)] = body.slice(sep + 1);
        return true;
      }
      if (rest.startsWith('KEY:')) {
        const [slot, modifier, keycode] = rest.slice(4).split(':').map(Number);
        prof.keys[slot] = { modifier, keycode };
        return true;
      }
      if (rest.startsWith('ROT:')) {
        const [slot, type, modifier, keycode] = rest.slice(4).split(':').map(Number);
        prof.rotary[slot] = { type, modifier, keycode };
        return true;
      }
      if (rest.startsWith('SCR:')) {
        const [layer, detents, invert] = rest.slice(4).split(':').map(Number);
        prof.scroll[layer] = { detentsPerRev: detents, invert: !!invert };
        return true;
      }
      if (rest.startsWith('OLEDMASK:')) { prof.oledMask = parseInt(rest.slice(9), 10); return true; }
      return false;
    },
    match: (line) => line === `${prefix}END`,
    result: prof,
  });
}

// Lee el bitmap guardado en el teclado. Devuelve null si el hueco usa la
// etiqueta de texto en vez de un icono.
export function getOled(profile, slot) {
  const bytes = new Uint8Array(360);
  let found = false;
  const prefix = `OLEDDATA:${profile}:${slot}:`;

  return request(`GET_OLED:${profile}:${slot}`, {
    timeout: 6000,
    collect: (line) => {
      if (!line.startsWith(prefix)) return false;
      const rest = line.slice(prefix.length);
      if (rest === 'NONE' || rest === 'END') return false; // los resuelve `match`

      const sep = rest.indexOf(':');
      if (sep < 0) return false;
      const offset = parseInt(rest.slice(0, sep), 10);
      const hex = rest.slice(sep + 1);
      for (let i = 0; i + 1 < hex.length; i += 2) {
        bytes[offset + i / 2] = parseInt(hex.substr(i, 2), 16);
      }
      found = true;
      return true;
    },
    match: (line) => line === `${prefix}END` || line === `${prefix}NONE`,
    result: null, // se sustituye abajo
  }).then((line) => (found ? bytes : null), () => null);
}

// Envía un framebuffer de 360 bytes troceado en chunks que caben en el buffer
// de comandos del firmware (512 bytes).
const OLED_CHUNK_BYTES = 90;

export async function uploadOled(profile, slot, bytes) {
  for (let offset = 0; offset < bytes.length; offset += OLED_CHUNK_BYTES) {
    const slice = bytes.subarray(offset, Math.min(offset + OLED_CHUNK_BYTES, bytes.length));
    let hex = '';
    for (const b of slice) hex += b.toString(16).padStart(2, '0');
    await request(`OLED_CHUNK:${profile}:${slot}:${offset}:${hex}`, {
      match: (l) => l.startsWith(`OLED:OK:${profile}:${slot}:${offset}:`),
    });
  }
}

// --- Enganche con el proceso principal -------------------------------------

export function init() {
  window.orby.onConnected((info) => emit('connected', info));
  window.orby.onDisconnected(() => {
    // Rechazamos lo pendiente para que ninguna vista se quede colgada.
    while (pending.length) {
      const req = pending.pop();
      clearTimeout(req.timer);
      req.resolve(null);
    }
    emit('disconnected');
  });
  window.orby.onSearching(() => emit('searching'));
  window.orby.onError((err) => emit('error', err));

  window.orby.onData((line) => {
    emit('rx', line);
    const consumed = settlePending(line);
    if (!consumed) emit('telemetry', line);
    else emit('response', line);
  });
}
