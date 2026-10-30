import test from 'node:test';
import assert from 'node:assert/strict';
import { textToKeys, textToSteps } from '../src/web/text-keys.mjs';

const SHIFT = 0x02, ALTGR = 0x40;

test('letras, mayúsculas, espacio y saltos', () => {
  assert.deepEqual(textToKeys('aB \n\t'), [
    { mod: 0, key: 0x04 }, { mod: SHIFT, key: 0x05 }, { mod: 0, key: 0x2C },
    { mod: 0, key: 0x28 }, { mod: 0, key: 0x2B },
  ]);
});

test('símbolos de la distribución española', () => {
  assert.deepEqual(textToKeys('@'), [{ mod: ALTGR, key: 0x1F }]);
  assert.deepEqual(textToKeys('"'), [{ mod: SHIFT, key: 0x1F }]);
  assert.deepEqual(textToKeys('{'), [{ mod: ALTGR, key: 0x34 }]);
  assert.deepEqual(textToKeys('ñ'), [{ mod: 0, key: 0x33 }]);
  assert.deepEqual(textToKeys('€'), [{ mod: ALTGR, key: 0x22 }]);
});

test('un acento es la tecla muerta y luego la vocal', () => {
  assert.deepEqual(textToKeys('á'), [{ mod: 0, key: 0x34 }, { mod: 0, key: 0x04 }]);
  assert.deepEqual(textToKeys('É'), [{ mod: 0, key: 0x34 }, { mod: SHIFT, key: 0x08 }]);
  assert.deepEqual(textToKeys('ü'), [{ mod: SHIFT, key: 0x34 }, { mod: 0, key: 0x18 }]);
});

test('las teclas muertas sueltas llevan un espacio detrás', () => {
  assert.deepEqual(textToKeys('~'), [{ mod: ALTGR, key: 0x21 }, { mod: 0, key: 0x2C }]);
});

test('un carácter sin tecla invalida todo el texto', () => {
  assert.equal(textToKeys('hola 😀'), null);
  assert.equal(textToSteps('日本'), null);
});

test('tres teclas por paso, con (mod << 8) | tecla y 0 de relleno', () => {
  assert.deepEqual(textToSteps('abcd'), [
    { a: 0x04, b: 0x05, gap: 0x06 },
    { a: 0x07, b: 0, gap: 0 },
  ]);
  assert.deepEqual(textToSteps('A'), [{ a: (SHIFT << 8) | 0x04, b: 0, gap: 0 }]);
  assert.deepEqual(textToSteps(''), []);
});

test('todo carácter ASCII imprimible tiene tecla', () => {
  for (let c = 0x20; c < 0x7F; c++) {
    assert.ok(textToKeys(String.fromCharCode(c)), `sin tecla: ${String.fromCharCode(c)}`);
  }
});
