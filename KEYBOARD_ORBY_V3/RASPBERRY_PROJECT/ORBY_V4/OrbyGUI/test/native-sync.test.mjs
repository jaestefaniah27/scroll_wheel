// La subida de secuencias de punta a punta, contra un teclado de mentira que
// contesta el protocolo serie como el firmware 4.7 (SET_MACRO_STEP, GET_MACRO,
// MACRO_TRUNC, MACRO_CLEAR y los POOL_*). Lo que se comprueba es lo que vería el
// reproductor: que cada paso de pool apunta a un trozo con su CRC, que ese trozo
// es el texto bien traducido y que no se reescribe nada que ya estaba igual.
import test from 'node:test';
import assert from 'node:assert/strict';

import { crc16, crc32 } from '../src/native/pool.mjs';
import { compileText } from '../src/native/layouts.mjs';
import { STEP } from '../src/native/compile.mjs';

const SECTOR = 4096;
const POOL_SECTORS = 112;

// --- El teclado de mentira --------------------------------------------------
function fakeKeyboard() {
  const kb = {
    macros: new Map(),                       // id → [{type,a,b,repeat,gap}]
    pool: new Uint8Array(POOL_SECTORS * SECTOR).fill(0xFF),
    stage: null,
    stageSector: -1,
    commits: [],
    log: [],
  };
  kb.handle = (cmd) => {
    kb.log.push(cmd);
    const f = cmd.split(':');
    switch (f[0]) {
      case 'GET_MACRO': {
        const id = Number(f[1]);
        const out = (kb.macros.get(id) || []).map((s, i) =>
          `MACRO:${id}:STEP:${i}:${s.type}:${s.a}:${s.b}:${s.repeat}:${s.gap}`);
        return [...out, `MACRO:${id}:END`];
      }
      case 'SET_MACRO_STEP': {
        const [id, step, type, a, b, repeat, gap] = f.slice(1).map(Number);
        const list = kb.macros.get(id) || [];
        list[step] = { type, a, b, repeat, gap };
        kb.macros.set(id, list);
        return [`MACRO:OK:${id}:${step}`];
      }
      case 'MACRO_TRUNC': {
        const [id, n] = f.slice(1).map(Number);
        kb.macros.set(id, (kb.macros.get(id) || []).slice(0, n));
        return [`MACRO:TRUNC:OK:${id}:${n}`];
      }
      case 'MACRO_CLEAR':
        kb.macros.delete(Number(f[1]));
        return [`MACRO:CLEARED:${f[1]}`];
      case 'POOL_HASH': {
        const [first, count] = f.slice(1).map(Number);
        const out = [];
        for (let s = first; s < first + count; s++) {
          const crc = crc32(kb.pool.subarray(s * SECTOR, (s + 1) * SECTOR));
          out.push(`POOL:HASH:${s}:${crc.toString(16).padStart(8, '0')}`);
        }
        return [...out, 'POOL:HASH:END'];
      }
      case 'POOL_BEGIN':
        kb.stage = new Uint8Array(SECTOR).fill(0xFF);
        kb.stageSector = Number(f[1]);
        return [`POOL:BEGIN:OK:${f[1]}`];
      case 'POOL_DATA': {
        const off = Number(f[1]);
        const hex = f[2];
        for (let i = 0; i < hex.length / 2; i++) kb.stage[off + i] = parseInt(hex.slice(i * 2, i * 2 + 2), 16);
        return [`POOL:DATA:OK:${off}`];
      }
      case 'POOL_COMMIT': {
        const s = Number(f[1]);
        if (s !== kb.stageSector) return ['ERR:BAD_ARGS'];
        kb.pool.set(kb.stage, s * SECTOR);
        kb.commits.push(s);
        const crc = crc32(kb.pool.subarray(s * SECTOR, (s + 1) * SECTOR));
        return [`POOL:COMMIT:OK:${s}:${crc.toString(16).padStart(8, '0')}`];
      }
      default:
        return [];
    }
  };
  // Lo que haría pool_blob en main.cpp: el trozo de un paso, si su CRC cuadra.
  kb.blobOf = (s) => {
    const off = s.a * 16;
    const bytes = kb.pool.subarray(off, off + s.b);
    return (crc16(bytes) & 0x7FFF) === s.gap ? bytes : null;
  };
  return kb;
}

// --- window.orby de mentira --------------------------------------------------
const kb = fakeKeyboard();
let onData = null;
let config = {};

globalThis.window = {
  orby: {
    sendCommand: (cmd) => setImmediate(() => kb.handle(cmd).forEach((l) => onData(l))),
    onData: (fn) => { onData = fn; },
    onConnected: (fn) => { setImmediate(() => fn({ port: 'X', raw: 'ORBY' })); },
    onDisconnected: () => {}, onSearching: () => {}, onError: () => {},
    getStatus: async () => 'connected',
    getDeviceInfo: async () => ({ port: 'X', raw: 'ORBY' }),
    getConfig: async () => structuredClone(config),
    setConfig: async (patch) => { config = { ...config, ...structuredClone(patch) }; },
    getMousePosition: async () => ({ x: 0, y: 0, desk: { x: 0, y: 0, w: 1920, h: 1080 } }),
  },
};

const device = await import('../src/device.js');
const { state } = await import('../src/store.js');
const store = await import('../src/views/profiles/macros-store.js');

device.init();
await new Promise((r) => setTimeout(r, 20));
state.connected = true;
state.deviceInfo = { fw: '4.7', macros: '1', maxmacros: '64', native: '1', pool: String(POOL_SECTORS) };

const textOf = (m) => m.actions.find((a) => a.type === 'text').text;

test('un texto sube al pool y el paso apunta a él con su CRC', async () => {
  config = { textLayout: 'es', macros: [{ id: 3, actions: [{ type: 'text', text: 'Hola, ¿qué tal? ñ@' }] }] };
  await store.syncAllMacrosToDevice();

  const steps = kb.macros.get(3);
  assert.equal(steps[0].type, STEP.PREFER_PC);
  assert.equal(steps[1].type, STEP.TEXT);
  const blob = kb.blobOf(steps[1]);
  assert.ok(blob, 'la CRC del paso cuadra con lo que hay en el pool');
  assert.deepEqual([...blob], [...compileText(textOf(config.macros[0]), 'es').bytes]);
});

test('volver a sincronizar sin cambios no escribe nada', async () => {
  const before = kb.commits.length;
  const setsBefore = kb.log.filter((c) => c.startsWith('SET_MACRO_STEP')).length;
  await store.syncAllMacrosToDevice();
  assert.equal(kb.commits.length, before, 'ningún sector de más');
  assert.equal(kb.log.filter((c) => c.startsWith('SET_MACRO_STEP')).length, setsBefore, 'ningún paso de más');
});

test('dos macros con texto no comparten sector, y editar una no toca la otra', async () => {
  config.macros.push({ id: 4, actions: [{ type: 'text', text: 'otra' }] });
  await store.syncAllMacrosToDevice();
  const sec3 = Math.floor(kb.macros.get(3)[1].a * 16 / SECTOR);
  const sec4 = Math.floor(kb.macros.get(4)[1].a * 16 / SECTOR);
  assert.notEqual(sec3, sec4);

  const before = kb.commits.length;
  store.macroById(4).actions[0].text = 'otra distinta';
  store.savePCMacros(4);
  await store.syncMacroToDevice(4);
  assert.deepEqual(kb.commits.slice(before), [sec4], 'solo se reescribe el sector de la 4');
  assert.ok(kb.blobOf(kb.macros.get(3)[1]), 'la 3 sigue intacta');
  assert.ok(kb.blobOf(kb.macros.get(4)[1]), 'y la 4 apunta a lo nuevo');
});

test('un texto largo ocupa varios sectores seguidos', async () => {
  config.macros.push({ id: 5, actions: [{ type: 'text', text: 'x'.repeat(5000) }] });
  await store.syncAllMacrosToDevice();
  const s = kb.macros.get(5)[1];
  assert.equal(s.b, 10000);
  assert.ok(kb.blobOf(s));
});

test('cambiar la distribución vuelve a subir los textos', async () => {
  store.setTextLayout('us');
  await store.syncMacroToDevice(3); // espera a que acabe la cola
  const blob = kb.blobOf(kb.macros.get(3)[1]);
  assert.deepEqual([...blob], [...compileText(textOf(config.macros[0]), 'us').bytes]);
  store.setTextLayout('es');
  await store.syncMacroToDevice(3);
});

test('una macro con un complemento se limpia del teclado', async () => {
  config.macros.find((m) => m.id === 4).actions = [{ type: 'plugin', plugin: 'lampdesk', op: 'toggle' }];
  await store.syncAllMacrosToDevice();
  assert.equal(kb.macros.has(4), false);
});

test('una grabación se sube con sus registros y su bucle', async () => {
  config.macros.push({
    id: 6, kind: 'recording', mode: 'loop', speed: 1,
    desk: { x: 0, y: 0, w: 1920, h: 1080 },
    events: [{ k: 'kdown', c: 30, t: 0 }, { k: 'kup', c: 30, t: 20 }],
  });
  await store.syncAllMacrosToDevice();
  const steps = kb.macros.get(6);
  assert.deepEqual(steps.map((s) => s.type), [STEP.PREFER_PC, STEP.EVENTS, STEP.LOOP]);
  assert.ok(kb.blobOf(steps[1]));
});

test('sin NATIVE=1 un texto ni se intenta subir', async () => {
  const saved = state.deviceInfo;
  state.deviceInfo = { fw: '4.6', macros: '1', maxmacros: '64' };
  try {
    assert.equal(store.macroDeviceEligible(store.macroById(3)), false);
    assert.match(store.macroDeviceBlocker(store.macroById(3)), /firmware/);
  } finally {
    state.deviceInfo = saved;
  }
});
