import test from 'node:test';
import assert from 'node:assert/strict';
import { toggleKey, pasteTargets } from '../src/views/profiles/clipboard.mjs';

const item = (key) => ({ key, label: `T${key}` });

test('Ctrl+clic: la primera vez entra también la tecla que ya estaba elegida', () => {
  assert.deepEqual(toggleKey([], 4, 1), [1, 4]);
  assert.deepEqual(toggleKey([1, 4], 4, 7), [1, 4, 7]);
});

test('Ctrl+clic en una ya elegida la quita', () => {
  assert.deepEqual(toggleKey([1, 4, 7], 7, 4), [1, 7]);
  assert.deepEqual(toggleKey([1, 4], 4, 4), [1]);
});

test('Ctrl+clic sobre la propia tecla elegida, sin selección múltiple, no la duplica', () => {
  assert.deepEqual(toggleKey([], 3, 3), [3]);
  assert.deepEqual(toggleKey([], null, 3), [3]);
});

test('una tecla copiada va a la elegida, o a todas las elegidas con Ctrl+clic', () => {
  assert.deepEqual(pasteTargets([item(2)], [], 5), [{ item: item(2), key: 5 }]);
  assert.deepEqual(pasteTargets([item(2)], [0, 3, 6], 6).map((t) => t.key), [0, 3, 6]);
});

test('varias copiadas con tantas elegidas como copiadas: en orden', () => {
  const t = pasteTargets([item(5), item(1)], [7, 3], 7);
  assert.deepEqual(t.map((x) => [x.item.key, x.key]), [[1, 3], [5, 7]]);
});

test('varias copiadas sin una selección del mismo tamaño: a sus mismas posiciones', () => {
  const t = pasteTargets([item(0), item(4), item(8)], [], 2);
  assert.deepEqual(t.map((x) => x.key), [0, 4, 8]);
  assert.deepEqual(pasteTargets([item(0), item(4)], [1, 2, 3], 1).map((x) => x.key), [0, 4]);
});

test('sin nada copiado o sin tecla elegida no se pega nada', () => {
  assert.deepEqual(pasteTargets(null, [], 1), []);
  assert.deepEqual(pasteTargets([item(1)], [], null), []);
});
