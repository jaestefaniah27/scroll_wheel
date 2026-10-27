import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { compileText, defaultLayout, cp1252Code, MOD, TEXT_HOLD_MODS, TEXT_RELEASE_MODS } from '../src/native/layouts.mjs';
import { crc16, crc32, layoutBlobs, allocate, sectorsUsedBy, varint, SECTOR, ALIGN } from '../src/native/pool.mjs';
import { compileRecording, toAbs, uiohookToHid, EVT } from '../src/native/recording.mjs';
import { compileMacro, runCommandFor, STEP, CONSUMER_USAGE, POOL_STEPS, MAX_STEPS } from '../src/native/compile.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const firmware = readFileSync(join(here, '..', '..', 'main.cpp'), 'utf8');

const pairs = (bytes) => {
  const out = [];
  for (let i = 0; i < bytes.length; i += 2) out.push([bytes[i], bytes[i + 1]]);
  return out;
};

// --- Texto ------------------------------------------------------------------

test('letras, mayúsculas y dígitos salen igual en las dos distribuciones', () => {
  for (const layout of ['es', 'us']) {
    const r = compileText('aZ1 0', layout);
    assert.ok(r.ok);
    assert.deepEqual(pairs(r.bytes), [[0, 0x04], [MOD.SHIFT, 0x1D], [0, 0x1E], [0, 0x2C], [0, 0x27]]);
  }
});

test('la @ depende de la distribución', () => {
  assert.deepEqual(pairs(compileText('@', 'es').bytes), [[MOD.ALTGR, 0x1F]]);
  assert.deepEqual(pairs(compileText('@', 'us').bytes), [[MOD.SHIFT, 0x1F]]);
});

test('en español la ñ es una tecla y la á, tecla muerta + letra', () => {
  assert.deepEqual(pairs(compileText('ñ', 'es').bytes), [[0, 0x33]]);
  assert.deepEqual(pairs(compileText('á', 'es').bytes), [[0, 0x34], [0, 0x04]]);
  assert.deepEqual(pairs(compileText('Ü', 'es').bytes), [[MOD.SHIFT, 0x34], [MOD.SHIFT, 0x18]]);
});

test('un acento suelto es su tecla muerta seguida de espacio', () => {
  assert.deepEqual(pairs(compileText('´', 'es').bytes), [[0, 0x34], [0, 0x2C]]);
  assert.deepEqual(pairs(compileText('~', 'es').bytes), [[MOD.ALTGR, 0x21], [0, 0x2C]]);
});

test('lo que no está en la distribución sale por Alt+0nnn', () => {
  // ñ en EE. UU. = Alt+0241
  assert.deepEqual(pairs(compileText('ñ', 'us').bytes), [
    [MOD.ALT, TEXT_HOLD_MODS], [0, 0x62], [0, 0x5A], [0, 0x5C], [0, 0x59], [0, TEXT_RELEASE_MODS],
  ]);
  assert.equal(cp1252Code('€'), 0x80);
  assert.equal(cp1252Code('—'), 0x97);
});

test('un carácter fuera de Windows-1252 no se puede escribir', () => {
  const r = compileText('hola 🙂', 'es');
  assert.equal(r.ok, false);
  assert.equal(r.char, '🙂');
});

test('saltos de línea y tabuladores son teclas, y \\r\\n es un solo salto', () => {
  assert.deepEqual(pairs(compileText('a\r\nb\tc\n', 'us').bytes), [
    [0, 0x04], [0, 0x28], [0, 0x05], [0, 0x2B], [0, 0x06], [0, 0x28],
  ]);
});

test('la distribución por defecto sale del idioma', () => {
  assert.equal(defaultLayout('es-ES'), 'es');
  assert.equal(defaultLayout('en-US'), 'us');
  assert.equal(defaultLayout(undefined), 'us');
});

// --- Pool -------------------------------------------------------------------

test('las CRC son las estándar', () => {
  const ascii = new TextEncoder().encode('123456789');
  assert.equal(crc16(ascii), 0x29B1); // CRC-16/CCITT-FALSE
  assert.equal(crc32(ascii), 0xCBF43926);
});

test('los trozos de una macro van alineados y en sectores enteros', () => {
  const r = layoutBlobs([new Uint8Array(5).fill(1), new Uint8Array(3).fill(2)]);
  assert.deepEqual(r.offsets, [0, ALIGN]);
  assert.equal(r.sectors, 1);
  assert.equal(r.image.length, SECTOR);
  assert.equal(r.image[5], 0xFF);
  assert.equal(r.image[ALIGN], 2);
});

test('el reparto deja a cada macro donde estaba si le cabe', () => {
  const owners = new Map([[0, 7], [1, 7], [2, 9]]);
  assert.equal(allocate(7, 2, owners, 10), 0);
  assert.equal(allocate(7, 1, owners, 10), 0);
  // No le caben 3 donde estaba (el 2 es de otra): al primer hueco libre.
  assert.equal(allocate(7, 3, owners, 10), 3);
  assert.equal(allocate(5, 1, owners, 10), 3);
  assert.equal(allocate(5, 8, owners, 10), null);
});

test('los sectores de una macro salen de sus pasos de pool', () => {
  const steps = [
    { type: STEP.KEY, a: 1, b: 2 },
    { type: STEP.TEXT, a: SECTOR / ALIGN, b: SECTOR + 1 }, // del sector 1 al 2
  ];
  assert.deepEqual([...sectorsUsedBy(steps, POOL_STEPS)].sort(), [1, 2]);
});

test('varint como LEB128', () => {
  assert.deepEqual(varint(0), [0]);
  assert.deepEqual(varint(127), [127]);
  assert.deepEqual(varint(300), [0xAC, 0x02]);
});

// --- Grabaciones ------------------------------------------------------------

const desk = { x: 0, y: 0, w: 1920, h: 1080 };

test('los extremos de la pantalla son los extremos del puntero absoluto', () => {
  assert.deepEqual(toAbs(0, 0, desk), [0, 0]);
  assert.deepEqual(toAbs(1919, 1079, desk), [32767, 32767]);
  // Con un monitor a la izquierda del principal, el origen es negativo.
  assert.deepEqual(toAbs(-1920, 0, { x: -1920, y: 0, w: 3840, h: 1080 }), [0, 0]);
});

test('las teclas de la grabadora se traducen a HID', () => {
  assert.equal(uiohookToHid(30), 0x04);    // A
  assert.equal(uiohookToHid(29), 0xE0);    // Control izquierdo
  assert.equal(uiohookToHid(57416), 0x52); // Arriba
  assert.equal(uiohookToHid(9999), null);
});

test('una grabación se convierte en registros con su espera', () => {
  const events = [
    { k: 'kdown', c: 30, t: 0 },
    { k: 'kup', c: 30, t: 30 },
    { k: 'mdown', b: 1, x: 0, y: 0, t: 60 },
    { k: 'wheel', r: -1, d: 3, t: 90 },
  ];
  const r = compileRecording(events, { desk, speed: 1 });
  assert.ok(r.ok);
  assert.deepEqual([...r.chunks[0]], [
    0, EVT.KEY_DOWN, 0x04,
    30, EVT.KEY_UP, 0x04,
    30, EVT.ABS, 0, 0, 0, 0,
    0, EVT.BTN_DOWN, 0x01,
    30, EVT.WHEEL, 1, // hacia arriba
  ]);
});

test('la velocidad acorta las esperas', () => {
  const r = compileRecording([{ k: 'kdown', c: 30, t: 0 }, { k: 'kup', c: 30, t: 300 }], { speed: 3 });
  assert.equal(r.chunks[0][3], 100);
});

test('sin pantalla no se puede reproducir un movimiento', () => {
  const r = compileRecording([{ k: 'move', x: 1, y: 1, t: 0 }], {});
  assert.equal(r.ok, false);
});

// --- Macros -----------------------------------------------------------------

const opts = { native: true, layout: 'es', desk };

test('una secuencia de siempre sube igual que antes', () => {
  const m = { id: 1, actions: [
    { type: 'hotkey', modifier: 1, keycode: 6 },
    { type: 'delay', ms: 50 },
    { type: 'mouse_click', button: 'right', count: 3, gap: 40 },
  ] };
  const r = compileMacro(m, { native: false });
  assert.ok(r.ok);
  assert.equal(r.preferPc, false);
  assert.deepEqual(r.steps, [
    { type: STEP.KEY, a: 1, b: 6, repeat: 1, gap: 0 },
    { type: STEP.DELAY, a: 50, b: 0, repeat: 1, gap: 0 },
    { type: STEP.CLICK, a: 2, b: 0, repeat: 3, gap: 40 },
  ]);
});

test('sin NATIVE=1 un texto se queda en el PC', () => {
  assert.equal(compileMacro({ actions: [{ type: 'text', text: 'hola' }] }, { native: false }).ok, false);
});

test('un texto lleva la marca de preferir el PC y su trozo', () => {
  const r = compileMacro({ actions: [{ type: 'text', text: 'hola' }] }, opts);
  assert.ok(r.ok);
  assert.equal(r.preferPc, true);
  assert.equal(r.steps[0].type, STEP.PREFER_PC);
  assert.equal(r.steps[1].type, STEP.TEXT);
  assert.equal(r.steps[1].blob.length, 8);
});

test('abrir una app es Win+R, esperar, escribir la ruta e Intro', () => {
  const r = compileMacro({ actions: [{ type: 'open_app', target: 'C:\\Program Files\\App\\app.exe' }] }, opts);
  assert.ok(r.ok);
  assert.deepEqual(r.steps.map((s) => s.type),
    [STEP.PREFER_PC, STEP.KEY, STEP.DELAY, STEP.TEXT, STEP.DELAY, STEP.KEY]);
  assert.deepEqual([r.steps[1].a, r.steps[1].b], [MOD.GUI, 0x15]);
  assert.deepEqual([r.steps[5].a, r.steps[5].b], [0, 0x28]);
});

test('las rutas con espacios van entre comillas, las URL no', () => {
  assert.equal(runCommandFor('C:\\Mis cosas\\a.txt'), '"C:\\Mis cosas\\a.txt"');
  assert.equal(runCommandFor('notepad'), 'notepad');
  assert.equal(runCommandFor('https://example.com/a b'), 'https://example.com/a b');
  assert.equal(runCommandFor('"ya entre comillas"'), '"ya entre comillas"');
});

test('energía: bloquear es Win+L, suspender es System Control, apagar va por shutdown', () => {
  const power = (mode) => compileMacro({ actions: [{ type: 'system_power', mode }] }, opts);
  assert.deepEqual(power('lock').steps.slice(1).map((s) => [s.type, s.a, s.b]), [[STEP.KEY, MOD.GUI, 0x0F]]);
  assert.deepEqual(power('sleep').steps.slice(1).map((s) => [s.type, s.a]), [[STEP.SYSTEM, 2]]);
  assert.equal(power('shutdown').steps[1].type, STEP.KEY);
});

test('una posición de ratón necesita saber de qué pantalla es', () => {
  const m = { actions: [{ type: 'mouse_position', x: 1919, y: 0 }] };
  assert.equal(compileMacro(m, { native: true, layout: 'es' }).ok, false);
  const r = compileMacro(m, opts);
  assert.deepEqual(r.steps[1], { type: STEP.ABS_MOVE, a: 32767, b: 0, repeat: 1, gap: 0 });
  // La que viaja con el paso manda sobre la del PC.
  const own = compileMacro({ actions: [{ ...m.actions[0], desk: { x: 0, y: 0, w: 3840, h: 1080 } }] }, opts);
  assert.equal(own.steps[1].a, Math.round(1919 * 32767 / 3839));
});

test('los complementos se quedan en el PC', () => {
  assert.equal(compileMacro({ actions: [{ type: 'plugin', plugin: 'x' }] }, opts).ok, false);
});

test('una grabación en bucle acaba en LOOP; «mientras se pulsa», en LOOP 1', () => {
  const rec = { kind: 'recording', events: [{ k: 'kdown', c: 30, t: 0 }], mode: 'hold' };
  const r = compileMacro(rec, opts);
  assert.deepEqual(r.steps.map((s) => s.type), [STEP.PREFER_PC, STEP.EVENTS, STEP.LOOP]);
  assert.equal(r.steps[2].a, 1);
  assert.equal(compileMacro({ ...rec, events: [] }, opts).ok, false);
});

test('una macro que no cabe en 48 pasos se queda en el PC', () => {
  const actions = Array.from({ length: MAX_STEPS + 1 }, () => ({ type: 'delay', ms: 1 }));
  assert.equal(compileMacro({ actions }, opts).ok, false);
});

// --- De acuerdo con el firmware ---------------------------------------------

test('los tipos de paso coinciden con MacroStepType', () => {
  for (const [name, value] of Object.entries(STEP)) {
    const m = new RegExp(`MSTEP_${name}\\s*=\\s*(\\d+)`).exec(firmware);
    assert.ok(m, `falta MSTEP_${name} en main.cpp`);
    assert.equal(Number(m[1]), value, `MSTEP_${name}`);
  }
});

test('la tabla multimedia coincide con get_consumer_key_from_index', () => {
  const body = /get_consumer_key_from_index\(uint8_t idx\) \{([\s\S]*?)default:/.exec(firmware)[1];
  const fw = {};
  for (const m of body.matchAll(/case (\d+):\s*return (0x[0-9A-Fa-f]+);/g)) fw[m[1]] = parseInt(m[2], 16);
  assert.deepEqual(Object.fromEntries(Object.entries(CONSUMER_USAGE).map(([k, v]) => [k, v])), fw);
});
