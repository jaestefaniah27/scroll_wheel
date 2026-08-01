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

// Secuencias que puede tocar el propio teclado (espera, tecla, clic, mover el
// ratón un delta): se suben paso a paso, con el mismo id que guarda la tecla
// en su modificador MACRO_MODIFIER. La posición absoluta del ratón no tiene
// equivalente aquí: de esa se sigue encargando el PC por CDC (ver MACRO:<id>).
export const setMacroStep = (id, step, type, a, b) =>
  request(`SET_MACRO_STEP:${id}:${step}:${type}:${a}:${b}`, { match: (l) => l === `MACRO:OK:${id}:${step}` });

export const macroTrunc = (id, count) =>
  request(`MACRO_TRUNC:${id}:${count}`, { match: (l) => l === `MACRO:TRUNC:OK:${id}:${count}` });

export const macroClear = (id) =>
  request(`MACRO_CLEAR:${id}`, { match: (l) => l === `MACRO:CLEARED:${id}` });

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
      if (line.startsWith('STATE:TIMEOUT:'))    { state.timeout   = parseInt(line.slice(14), 10); return true; }
      if (line.startsWith('STATE:MODE:'))       { state.mode      = line.slice(11); return true; }
      if (line.startsWith('STATE:SUPER:'))      { state.superActive = line.slice(12) === '1'; return true; }
      if (line.startsWith('STATE:PAGE:')) {
        // Lo añadió el firmware 4.0; con uno anterior no llega y se queda en una
        // sola página, que es lo que había.
        const [page, count, max] = line.slice(11).split(':').map(Number);
        state.pageIdx   = page || 0;
        state.pageCount = count || 1;
        state.maxPages  = max || 1;
        return true;
      }
      if (line.startsWith('SCROLL:OK:')) {
        // El cuarto campo (alta resolución del paneo horizontal) lo añadió el
        // firmware después; con uno anterior llega undefined y queda en false.
        const [det, inv, hires, hiresPan] = line.slice(10).split(':').map(Number);
        state.scroll = { detentsPerRev: det, invert: !!inv, hires: !!hires, hiresPan: !!hiresPan };
        return true;
      }
      return false;
    },
    match: (line) => line === 'STATE:END',
    result: state,
  });
}

// Una página en blanco: 20 etiquetas, 24 acciones de tecla, 16 de mando (8 por
// capa), la rueda de cada capa y la máscara de iconos.
export function blankPage() {
  const pg = {
    labels: new Array(20).fill(''), keys: [], rotary: [],
    scroll: [{ detentsPerRev: 60, invert: false }, { detentsPerRev: 60, invert: false }],
    oledMask: 0,
  };
  for (let i = 0; i < 24; i++) pg.keys.push({ modifier: 0, keycode: 0 });
  for (let i = 0; i < 16; i++) pg.rotary.push({ type: 0, modifier: 0, keycode: 0 });
  return pg;
}

// Las vistas llevan escritas 2000 líneas contra `prof.labels`, `prof.keys`...
// En vez de reescribirlas todas, esos nombres pasan a ser accesos a la página
// seleccionada: leer y escribir por ellos va siempre a `prof.pages[prof.pageIdx]`,
// así que no puede haber dos copias que se desincronicen.
//
// No son enumerables a propósito: así JSON.stringify (la copia de seguridad) ve
// la estructura de páginas de verdad y no una mezcla de las dos cosas.
const PAGE_FIELDS = ['labels', 'keys', 'rotary', 'scroll', 'oledMask'];
export function bindPageAliases(prof) {
  for (const field of PAGE_FIELDS) {
    Object.defineProperty(prof, field, {
      get() { return (prof.pages[prof.pageIdx] || prof.pages[0])[field]; },
      set(v) { (prof.pages[prof.pageIdx] || prof.pages[0])[field] = v; },
      enumerable: false,
      configurable: true,
    });
  }
  return prof;
}

// Lee un perfil completo, con todas sus páginas.
//
// El firmware vuelca primero la página activa sin numerar (para las apps que no
// saben de páginas) y después una tanda por página con `P<n>:` delante. Aquí se
// leen las numeradas; la primera tanda sirve de reserva si el firmware es
// anterior a las páginas y no manda ninguna.
export function getProfile(idx) {
  const prof = { idx, name: '', pageCount: 1, maxPages: 1, pageIdx: 0, pages: [] };
  const legacy = blankPage();

  const ensure = (n) => {
    while (prof.pages.length <= n) prof.pages.push(blankPage());
    return prof.pages[n];
  };

  const applyField = (pg, rest) => {
    if (rest.startsWith('LBL:')) {
      const body = rest.slice(4);
      const sep = body.indexOf(':');
      pg.labels[parseInt(body.slice(0, sep), 10)] = body.slice(sep + 1);
      return true;
    }
    if (rest.startsWith('KEY:')) {
      const [slot, modifier, keycode] = rest.slice(4).split(':').map(Number);
      pg.keys[slot] = { modifier, keycode };
      return true;
    }
    if (rest.startsWith('ROT:')) {
      const [slot, type, modifier, keycode] = rest.slice(4).split(':').map(Number);
      pg.rotary[slot] = { type, modifier, keycode };
      return true;
    }
    if (rest.startsWith('SCR:')) {
      const [layer, detents, invert] = rest.slice(4).split(':').map(Number);
      pg.scroll[layer] = { detentsPerRev: detents, invert: !!invert };
      return true;
    }
    if (rest.startsWith('OLEDMASK:')) { pg.oledMask = parseInt(rest.slice(9), 10); return true; }
    return false;
  };

  const prefix = `PROF:${idx}:`;
  return request(`GET_PROFILE:${idx}`, {
    timeout: 12000,
    collect: (line) => {
      if (!line.startsWith(prefix)) return false;
      const rest = line.slice(prefix.length);

      if (rest.startsWith('NAME:')) { prof.name = rest.slice(5); return true; }
      if (rest.startsWith('PAGES:')) {
        const [count, max] = rest.slice(6).split(':').map(Number);
        prof.pageCount = count || 1;
        prof.maxPages  = max || 1;
        ensure(prof.pageCount - 1);
        return true;
      }
      // Tanda numerada: PROF:<n>:P<pg>:CAMPO:...
      const m = rest.match(/^P(\d+):(.*)$/);
      if (m) return applyField(ensure(parseInt(m[1], 10)), m[2]);
      // Tanda sin numerar, para firmwares anteriores a las páginas.
      return applyField(legacy, rest);
    },
    match: (line) => line === `${prefix}END`,
    result: prof,
  }).then((p) => {
    // Firmware antiguo: no mandó ninguna página numerada, así que lo que llegó
    // sin numerar es la única que hay.
    if (!p.pages.length) { p.pages.push(legacy); p.pageCount = 1; p.maxPages = 1; }
    return bindPageAliases(p);
  });
}

// ---------- Páginas ----------
export function setPage(pageIdx) {
  return request(`SET_PAGE:${pageIdx}`, { match: (l) => l.startsWith('PAGE:OK:') || l.startsWith('ERR:') });
}

export function addPage(profileIdx, copyCurrent = true) {
  return request(`ADD_PAGE:${profileIdx}:${copyCurrent ? 1 : 0}`,
                 { match: (l) => l.startsWith('PAGE:ADDED:') || l.startsWith('ERR:') });
}

export function delPage(profileIdx, pageIdx) {
  return request(`DEL_PAGE:${profileIdx}:${pageIdx}`,
                 { match: (l) => l.startsWith('PAGE:DELETED:') || l.startsWith('ERR:') });
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
