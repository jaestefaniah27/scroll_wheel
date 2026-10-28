import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as U from '../src/oled-ui.mjs';
import { ENTRIES } from '../src/oled-ui-data.mjs';

// La app tiene que dibujar las pantallas sin icono subido EXACTAMENTE como el
// teclado. La referencia son los casos que tools/oled_ui/build.py saca de
// layout.py, los mismos con los que test_ui.cpp comprueba oled_ui.h.

const casos = JSON.parse(readFileSync(new URL('./fixtures/oled-ui-cases.json', import.meta.url), 'utf8'));
const hex = (fb) => Array.from(fb, (b) => b.toString(16).padStart(2, '0')).join('');
const icono = (key) => ENTRIES.find(([k]) => k === key);

function dibuja(what) {
  const [tipo, ...resto] = what.split(':');
  const arg = resto.join(':');
  if (tipo === 'icon') { const [, ic, cap] = icono(arg); return U.screenIcon(ic, cap); }
  if (tipo === 'text') return U.screenText(arg);
  if (tipo === 'toggle') return U.screenToggle(resto[0] === '1', resto.slice(1).join(':'));
  if (tipo === 'dots') return U.screenDots(Number(resto[0]), Number(resto[1]), resto.slice(2).join(':'));
  return null;
}

for (const c of casos) {
  const fb = dibuja(c.what);
  if (!fb) continue;   // la píldora de menú es del sistema: la app no la dibuja
  test(`pantalla igual que el firmware: ${c.what}`, () => {
    assert.equal(hex(fb), c.fb);
  });
}

test('el catálogo se busca sin mayúsculas, con la entrada del perfil delante', () => {
  assert.ok(U.lookup('', 'copy') >= 0);
  assert.equal(U.lookup('', 'copy'), U.lookup('', 'COPY'));
  // «PREM/CUT» es la cuchilla de Premiere; «CUT» a secas, las tijeras.
  assert.notEqual(U.lookup('PREM', 'CUT'), U.lookup('', 'CUT'));
  assert.equal(U.lookup('', 'NO-EXISTE'), -1);
});

test('una tecla sin entrada en el catálogo lleva su etiqueta en texto', () => {
  assert.equal(hex(U.screenKey('', 'F1')), hex(U.screenText('F1')));
  assert.equal(hex(U.screenKey('', '')), '00'.repeat(U.FB));
});
