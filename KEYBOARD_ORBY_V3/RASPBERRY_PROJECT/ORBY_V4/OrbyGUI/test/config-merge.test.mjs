import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULTS, merge } from '../src/web/config-merge.mjs';

test('los valores por defecto traen las claves que leen las vistas', () => {
  assert.deepEqual(DEFAULTS.macros, []);
  assert.equal(DEFAULTS.macrosOnDevice, null);
  assert.equal(DEFAULTS.deviceMirror, null);
  assert.equal(DEFAULTS.wheelDial.offsetDeg, 62);
});

test('fusiona en profundidad sin tocar la base', () => {
  const base = { wheelDial: { invert: true, offsetDeg: 62, marker: 'dot' } };
  const out = merge(base, { wheelDial: { marker: 'line' } });
  assert.deepEqual(out.wheelDial, { invert: true, offsetDeg: 62, marker: 'line' });
  assert.equal(base.wheelDial.marker, 'dot');
});

test('el espejo del teclado se sustituye entero, no se fusiona', () => {
  // Fusionándolo, un icono borrado o un perfil que ya no existe seguirían en el
  // archivo para siempre: el espejo describe un estado completo, no un parche.
  const base = { deviceMirror: { savedAt: 'antes', icons: { '0:0:1': 'aa' } } };
  const out = merge(base, { deviceMirror: { savedAt: 'ahora', icons: {} } });
  assert.deepEqual(out.deviceMirror, { savedAt: 'ahora', icons: {} });
});

test('los arrays se sustituyen enteros', () => {
  const out = merge({ macros: [{ id: 1 }, { id: 2 }] }, { macros: [{ id: 3 }] });
  assert.deepEqual(out.macros, [{ id: 3 }]);
});
