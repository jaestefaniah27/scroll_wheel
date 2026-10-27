// La copia de las macros en el teclado, de punta a punta: la app de verdad
// (device.js + macros-store.js) contra un teclado de mentira que habla el
// protocolo MSTORE_* como el firmware (la parte de Flash del firmware se prueba
// aparte en tools/test/test_macro_store.cpp).
//
// El caso que importa: unas macros hechas en un PC tienen que aparecer en otro
// PC que nunca las ha visto, solo por enchufar el teclado.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { crc32 } from '../src/web/macro-blob.mjs';

// --- El teclado ---------------------------------------------------------------
const keyboard = {
  store: null,            // { bytes, crc } vigente
  writing: null,          // { len, crc, bytes, written }
  writes: 0,
  handle(cmd) {
    if (cmd === 'MSTORE_INFO') {
      const s = this.store;
      return `MSTORE:INFO:${s ? s.bytes.length : 0}:${(s ? s.crc : 0).toString(16).padStart(8, '0')}:229120`;
    }
    if (cmd.startsWith('MSTORE_GET:')) {
      const [off, n] = cmd.slice(11).split(':').map(Number);
      const part = this.store.bytes.subarray(off, off + n);
      return `MSTORE:DATA:${off}:${Buffer.from(part).toString('hex')}`;
    }
    if (cmd.startsWith('MSTORE_BEGIN:')) {
      const [len, crc] = cmd.slice(13).split(':');
      this.writing = { len: Number(len), crc: parseInt(crc, 16) >>> 0, bytes: new Uint8Array(Number(len)), written: 0 };
      return 'MSTORE:READY';
    }
    if (cmd.startsWith('MSTORE_DATA:')) {
      const rest = cmd.slice(12);
      const off = Number(rest.slice(0, rest.indexOf(':')));
      const data = Buffer.from(rest.slice(rest.indexOf(':') + 1), 'hex');
      if (off !== this.writing.written) return `ERR:MSTORE_ORDER:${this.writing.written}`;
      this.writing.bytes.set(data, off);
      this.writing.written += data.length;
      return `MSTORE:ACK:${this.writing.written}`;
    }
    if (cmd === 'MSTORE_END') {
      const w = this.writing;
      const crc = crc32(w.bytes);
      if (w.written !== w.len) return `ERR:MSTORE_SHORT:${w.written}`;
      if (crc !== w.crc) return `ERR:MSTORE_CRC:${crc.toString(16)}`;
      this.store = { bytes: w.bytes, crc };
      this.writes++;
      return `MSTORE:OK:${crc.toString(16)}`;
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
      // La respuesta llega después, como por el puerto serie: si llegara dentro
      // de sendCommand, la petición todavía no estaría esperándola.
      queueMicrotask(() => onData(keyboard.handle(cmd)));
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
const INFO = { fw: '4.7', macros: '1', mstore: '1', maxmacros: '64' };

const device = await import('../src/device.js');
const { state } = await import('../src/store.js');
const store = await import('../src/views/profiles/macros-store.js');

device.init();
await new Promise((r) => setTimeout(r, 20));   // el enlace se da por bueno al preguntar el estado
state.connected = true;
state.deviceInfo = INFO;

const MACROS_A = [
  { id: 1, actions: [{ type: 'open_app', path: 'C:\\Altium\\X2.EXE' }] },
  { id: 2, name: 'Firma', actions: [{ type: 'text', text: 'Un saludo,\nJavier' }] },
  { id: 5, kind: 'recording', events: Array.from({ length: 800 }, (_, i) => ({ type: 'move', x: i, y: 2 * i, t: 16 * i })) },
];

test('PC A con macros y teclado vacío: las sube', async () => {
  config = { macros: structuredClone(MACROS_A) };
  const recovered = await store.syncMacroStore();
  assert.equal(recovered, 0);
  assert.ok(keyboard.store, 'el teclado no ha recibido nada');
  assert.equal(config.macroStore.crc, keyboard.store.crc);
  assert.equal(config.macroStore.pending, false);
});

test('PC B, que nunca las ha visto, las recupera al conectar', async () => {
  config = {};
  const recovered = await store.syncMacroStore();
  assert.equal(recovered, 3);
  assert.deepEqual(config.macros, MACROS_A);
  assert.deepEqual(store.allMacros(), MACROS_A);
});

test('reconectar sin cambios no baja ni escribe nada', async () => {
  const before = keyboard.writes;
  assert.equal(await store.syncMacroStore(), 0);
  assert.equal(keyboard.writes, before, 'ha vuelto a escribir la Flash sin motivo');
});

test('lo que se edita en B llega a A, y lo que solo tenía A no se pierde', async () => {
  // En B se cambia la firma.
  // (como savePCMacros: se guarda en la configuración de B y se sube)
  store.allMacros().find((m) => m.id === 2).actions[0].text = 'Saludos';
  config.macros = structuredClone(store.allMacros());
  await store.uploadStore();
  const configB = config;

  // A conecta otra vez: tiene la firma vieja y, además, una macro propia.
  config = { macros: [...structuredClone(MACROS_A), { id: 9, actions: [{ type: 'system_power', mode: 'sleep' }] }],
             macroStore: { crc: 12345, pending: false } };
  const recovered = await store.syncMacroStore();
  assert.equal(recovered, 1);
  assert.equal(config.macros.find((m) => m.id === 2).actions[0].text, 'Saludos');
  assert.ok(config.macros.some((m) => m.id === 9), 'se ha perdido la macro que solo tenía A');

  // Y el teclado se queda con la unión, para que B también la vea.
  config = configB;
  assert.equal(await store.syncMacroStore(), 1);
  assert.ok(config.macros.some((m) => m.id === 9));
});

test('un cambio que no llegó a subirse se sube al volver a conectar', async () => {
  store.allMacros().push({ id: 12, actions: [{ type: 'text', text: 'pendiente' }] });
  config.macros = store.allMacros();
  config.macroStore = { ...config.macroStore, pending: true };   // se desenchufó antes de subir
  const before = keyboard.writes;
  await store.syncMacroStore();
  assert.equal(keyboard.writes, before + 1);
  assert.equal(config.macroStore.pending, false);
});
