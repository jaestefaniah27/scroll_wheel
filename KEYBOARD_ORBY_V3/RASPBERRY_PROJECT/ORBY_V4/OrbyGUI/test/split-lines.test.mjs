import test from 'node:test';
import assert from 'node:assert/strict';
import { splitLines } from '../src/web/split-lines.mjs';

test('separa las líneas completas y guarda el resto', () => {
  const r = splitLines('', 'ORBY_V4:FW=4.3\nSTATE:PROFILE:0\nSTATE:PRO');
  assert.deepEqual(r.lineas, ['ORBY_V4:FW=4.3', 'STATE:PROFILE:0']);
  assert.equal(r.resto, 'STATE:PRO');
});

test('une el resto anterior con el trozo nuevo', () => {
  const r = splitLines('STATE:PRO', 'FILES:3:8\nSTATE:END\n');
  assert.deepEqual(r.lineas, ['STATE:PROFILES:3:8', 'STATE:END']);
  assert.equal(r.resto, '');
});

test('descarta el retorno de carro y las líneas en blanco', () => {
  // El firmware manda \n, pero un CDC puede entregar \r\n según cómo lo empaquete
  // el host. Una línea vacía colada en la cola de peticiones de device.js no
  // encaja con ningún match y se queda ahí.
  const r = splitLines('', 'PROFILE:OK:1\r\n\r\nSAVE:OK\r\n');
  assert.deepEqual(r.lineas, ['PROFILE:OK:1', 'SAVE:OK']);
  assert.equal(r.resto, '');
});

test('un trozo sin salto de línea no produce ninguna', () => {
  const r = splitLines('', 'MACRO:');
  assert.deepEqual(r.lineas, []);
  assert.equal(r.resto, 'MACRO:');
});
