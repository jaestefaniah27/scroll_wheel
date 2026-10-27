// El bloque de macros que se guarda en el teclado (src/web/macro-blob.mjs).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { encode, decode, merge, canonical, crc32 } from '../src/web/macro-blob.mjs';

const MACROS = [
  { id: 3, actions: [{ type: 'open_app', path: 'C:\\Program Files\\Altium\\X2.EXE' }] },
  { id: 1, name: 'Saludo', actions: [{ type: 'text', text: 'Un saludo, ñandú' }] },
  { id: 7, kind: 'recording',
    events: Array.from({ length: 2000 }, (_, i) => ({ type: 'move', x: 100 + (i % 50), y: 200, t: i * 16 })) },
];

test('lo que se guarda se recupera igual', async () => {
  const back = await decode(await encode(MACROS));
  assert.equal(canonical(back), canonical(MACROS));
});

test('una grabación comprimida ocupa mucho menos que su JSON', async () => {
  const blob = await encode(MACROS);
  assert.ok(blob.length * 5 < canonical(MACROS).length, `${blob.length} B comprimido`);
});

test('el texto canónico no depende del orden', () => {
  const shuffled = [MACROS[2], MACROS[0], { ...MACROS[1] }];
  assert.equal(canonical(shuffled), canonical(MACROS));
});

test('un bloque ajeno se rechaza en vez de mezclarse', async () => {
  await assert.rejects(decode(new Uint8Array([1, 2, 3, 4, 5, 6])));
  const future = await encode(MACROS);
  future[3] = 99;
  await assert.rejects(decode(future));
});

test('al juntar, el teclado manda y lo del PC que no está se conserva', () => {
  const local = [{ id: 1, actions: [{ type: 'text', text: 'viejo' }] }, { id: 9, actions: [] }];
  const { macros, changed } = merge(local, MACROS);
  assert.deepEqual(macros.map((m) => m.id), [1, 3, 7, 9]);
  assert.equal(macros.find((m) => m.id === 1).name, 'Saludo');
  assert.equal(changed, 3);
  assert.equal(merge(macros, MACROS).changed, 0);
});

test('el CRC es el CRC32 estándar, el mismo que calcula el firmware', () => {
  assert.equal(crc32(new TextEncoder().encode('123456789')), 0xcbf43926);
});
