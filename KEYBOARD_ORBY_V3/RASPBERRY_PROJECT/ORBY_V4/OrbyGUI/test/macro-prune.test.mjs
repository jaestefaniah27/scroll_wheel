// La limpieza de las macros que ya no usa ninguna tecla (pruneUnusedMacros en
// views/profiles/macros-store.js), contra un teclado de mentira que habla los
// comandos de secuencias y del almacén como el firmware.
//
// Lo que importa: que se vaya lo que no usa nada, que NO se vaya nada que use
// una tecla, un mando o una variación de cualquier página o capa, y que no se
// haga nada si la lista de perfiles no es seguro la entera.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { crc32 } from '../src/web/macro-blob.mjs';

// --- El teclado ---------------------------------------------------------------
const keyboard = {
  macros: new Map(),      // id -> [{type,a,b,repeat,gap}]
  store: null,
  writing: null,
  handle(cmd) {
    if (cmd.startsWith('GET_MACRO:')) {
      const id = Number(cmd.slice(10));
      const steps = this.macros.get(id) || [];
      return [...steps.map((s, i) => `MACRO:${id}:STEP:${i}:${s.type}:${s.a}:${s.b}:${s.repeat}:${s.gap}`),
              `MACRO:${id}:END`];
    }
    if (cmd.startsWith('SET_MACRO_STEP:')) {
      const [id, step, type, a, b, repeat, gap] = cmd.slice(15).split(':').map(Number);
      const steps = this.macros.get(id) || [];
      steps[step] = { type, a, b, repeat, gap };
      this.macros.set(id, steps);
      return `MACRO:OK:${id}:${step}`;
    }
    if (cmd.startsWith('MACRO_TRUNC:')) {
      const [id, n] = cmd.slice(12).split(':').map(Number);
      this.macros.set(id, (this.macros.get(id) || []).slice(0, n));
      return `MACRO:TRUNC:OK:${id}:${n}`;
    }
    if (cmd.startsWith('MACRO_CLEAR:')) {
      const id = Number(cmd.slice(12));
      this.macros.delete(id);
      return `MACRO:CLEARED:${id}`;
    }
    if (cmd === 'MSTORE_INFO') {
      const s = this.store;
      return `MSTORE:INFO:${s ? s.bytes.length : 0}:${(s ? s.crc : 0).toString(16).padStart(8, '0')}:229120`;
    }
    if (cmd.startsWith('MSTORE_BEGIN:')) {
      const [len, crc] = cmd.slice(13).split(':');
      this.writing = { len: Number(len), crc: parseInt(crc, 16) >>> 0, bytes: new Uint8Array(Number(len)), written: 0 };
      return 'MSTORE:READY';
    }
    if (cmd.startsWith('MSTORE_DATA:')) {
      const rest = cmd.slice(12);
      const data = Buffer.from(rest.slice(rest.indexOf(':') + 1), 'hex');
      this.writing.bytes.set(data, this.writing.written);
      this.writing.written += data.length;
      return `MSTORE:ACK:${this.writing.written}`;
    }
    if (cmd === 'MSTORE_END') {
      this.store = { bytes: this.writing.bytes, crc: crc32(this.writing.bytes) };
      return `MSTORE:OK:${this.store.crc.toString(16)}`;
    }
    return 'ERR:UNKNOWN_CMD';
  },
};

// --- El PC (window.orby) --------------------------------------------------------
let onData = () => {};
let config = {};
globalThis.window = {
  orby: {
    sendCommand(cmd) {
      const out = keyboard.handle(cmd);
      queueMicrotask(() => [].concat(out).forEach((l) => onData(l)));
    },
    onConnected() {}, onDisconnected() {}, onSearching() {}, onError() {},
    onData(cb) { onData = cb; },
    getStatus: async () => 'connected',
    getDeviceInfo: async () => INFO,
    getConfig: async () => structuredClone(config),
    setConfig: async (patch) => { Object.assign(config, structuredClone(patch)); },
    plugins: { list: async () => [] },
  },
};
const INFO = { fw: '4.14', macros: '1', mstore: '1', maxmacros: '255' };

const device = await import('../src/device.js');
const { state } = await import('../src/store.js');
const variants = await import('../src/variants.js');
const store = await import('../src/views/profiles/macros-store.js');

device.init();
await new Promise((r) => setTimeout(r, 20));
state.connected = true;
state.deviceInfo = INFO;

const MACRO = 0xFB;
const ROT_KEY = 2;
const seq = (id) => ({ id, actions: [{ type: 'hotkey', modifier: 1, keycode: 4, count: 1 }] });

function page() {
  return {
    labels: new Array(20).fill(''),
    keys: Array.from({ length: 24 }, () => ({ modifier: 0, keycode: 0 })),
    rotary: Array.from({ length: 16 }, () => ({ type: 0, modifier: 0, keycode: 0 })),
  };
}

// Dos perfiles: el segundo con dos páginas. Usadas: 3 (tecla, capa normal),
// 7 (tecla en SUPER, página 2 del perfil 2), 9 (mando), 20 (solo en una
// variación) y 30/31 (grabación y su tecla de borrado, que apunta a la 30 por
// `target`; la 30 no la usa ninguna tecla directamente).
function profiles() {
  const p0 = { idx: 0, pageCount: 1, pages: [page()] };
  p0.pages[0].keys[2] = { modifier: MACRO, keycode: 3 };
  const p1 = { idx: 1, pageCount: 2, pages: [page(), page()] };
  p1.pages[1].keys[12 + 5] = { modifier: MACRO, keycode: 7 };
  p1.pages[0].rotary[3] = { type: ROT_KEY, modifier: MACRO, keycode: 9 };
  p1.pages[0].keys[12 + 0] = { modifier: MACRO, keycode: 31 };
  // Un mando de tipo multimedia con el mismo número NO es una macro.
  p1.pages[0].rotary[4] = { type: 1, modifier: MACRO, keycode: 11 };
  return [p0, p1];
}

const MACROS = [seq(3), seq(4), seq(7), seq(9), seq(11), seq(20), seq(93),
  { id: 30, kind: 'recording', events: [] },
  { id: 31, kind: 'recording-reset', target: 30 }];

async function setup() {
  config = {
    macros: structuredClone(MACROS),
    macrosOnDevice: [3, 4, 93],
    profileVariants: [{ id: 'v1', profile: 0, page: 0, name: 'x', matches: ['notepad'],
                        keys: { 5: { modifier: MACRO, keycode: 20 } }, rotary: {}, labels: {} }],
  };
  keyboard.macros = new Map([[4, [{ type: 2, a: 1, b: 4, repeat: 1, gap: 0 }]],
                             [93, [{ type: 2, a: 1, b: 4, repeat: 1, gap: 0 }]]]);
  state.profiles = profiles();
  state.profilesComplete = true;
}

await setup();
await variants.init();

test('se van las que no usa nada, y solo esas', async () => {
  const removed = await store.pruneUnusedMacros();
  assert.equal(removed, 3);
  assert.deepEqual(config.macros.map((m) => m.id), [3, 7, 9, 20, 30, 31]);
  assert.deepEqual(config.macrosRetiradas.map((m) => m.id), [4, 11, 93]);
});

test('la copia del teclado de las que se van queda vacía', () => {
  assert.equal(keyboard.macros.has(4), false);
  assert.equal(keyboard.macros.has(93), false);
});

test('sin la lista de perfiles entera no se toca nada', async () => {
  await setup();
  state.profilesComplete = false;
  assert.equal(await store.pruneUnusedMacros(), 0);
  assert.equal(config.macros.length, MACROS.length);
});

test('un perfil con páginas sin leer tampoco deja limpiar', async () => {
  await setup();
  state.profiles[1].pages.pop();      // dice pageCount 2 y trae 1
  assert.equal(await store.pruneUnusedMacros(), 0);
  assert.equal(config.macros.length, MACROS.length);
});

test('si una tecla vuelve a usar una retirada, se recupera', async () => {
  await setup();
  await store.pruneUnusedMacros();
  // Como al restaurar una copia de seguridad hecha antes de la limpieza.
  state.profiles[0].pages[0].keys[8] = { modifier: MACRO, keycode: 93 };
  assert.equal(await store.rescueRetiredMacros(), 1);
  assert.ok(config.macros.some((m) => m.id === 93));
  assert.ok(!config.macrosRetiradas.some((m) => m.id === 93));
  assert.ok(!('retiradaEl' in config.macros.find((m) => m.id === 93)));
  // Y vuelve a estar en el teclado, que sabe tocarla solo.
  assert.equal(keyboard.macros.get(93)?.length, 1);
});

test('el id libre más bajo vuelve a estar disponible', async () => {
  await setup();
  await store.pruneUnusedMacros();
  await store.loadPCMacros();
  assert.equal(store.nextMacroId(), 1);
  store.allMacros().push({ id: 1, actions: [] }, { id: 2, actions: [] });
  assert.equal(store.nextMacroId(), 4);
  // La subida del almacén programada por la limpieza no debe quedar viva.
  await new Promise((r) => setTimeout(r, 2200));
});
