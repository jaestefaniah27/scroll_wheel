import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as A from '../src/oled-anim.mjs';

// El dashboard tiene que animar las pantallas EXACTAMENTE como el teclado. La
// referencia es include/oled_anim.h: tools/test/anim_golden.cpp lo ejecuta y
// apunta el CRC de cada fotograma en el fichero de abajo. Aquí se hace lo mismo
// con la copia en JS y tienen que salir iguales, bit a bit.

const golden = readFileSync(new URL('./fixtures/oled-anim-golden.txt', import.meta.url), 'utf8')
  .trim().split('\n');

function crc32(c, bytes) {
  c = ~c >>> 0;
  for (const b of bytes) {
    c ^= b;
    for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (0xedb88320 & -(c & 1));
  }
  return ~c >>> 0;
}

function datos() {
  let seed = 12345;
  const rnd = () => { seed = (Math.imul(seed, 1103515245) + 12345) >>> 0; return (seed >>> 16) & 0xff; };
  const from = [], to = [];
  for (let i = 0; i < A.N; i++) {
    from.push(new Uint8Array(A.FB));
    to.push(new Uint8Array(A.FB));
    for (let b = 0; b < A.FB; b++) { from[i][b] = rnd(); to[i][b] = rnd(); }
  }
  return { from, to };
}

test('cada fotograma de cada transición coincide con oled_anim.h', () => {
  const { from, to } = datos();
  // El teclado calcula en float: el ritmo y el instante llegan redondeados así.
  const rates = [1, Math.fround(1.4)];
  const out = new Uint8Array(A.FB);
  let comparados = 0;
  for (const linea of golden) {
    const p = linea.split(' ');
    if (p[0] === 'press') continue;
    const [k, r, ti] = p.slice(0, 3).map(Number);
    let c = 0;
    for (let i = 0; i < A.N; i++) {
      A.render(k, ti * Math.fround(0.0125), i, from, to, out, rates[r]);
      c = crc32(c, out);
    }
    assert.equal(c.toString(16).padStart(8, '0'), p[3], `tipo ${k}, ritmo ${rates[r]}, t=${ti}`);
    comparados++;
  }
  assert.equal(comparados, 8 * 2 * 41);
});

test('la pulsación se hunde con el mismo muelle', () => {
  const { to } = datos();
  const p = A.newPress();
  A.pressStep(p, 1, Math.fround(0.03));
  const out = new Uint8Array(A.FB);
  A.renderPress(to[0], p, out);
  const esperado = golden.find((l) => l.startsWith('press ')).split(' ')[1];
  assert.equal(crc32(0, out).toString(16).padStart(8, '0'), esperado);
});

test('pasada su duración, la transición deja el destino tal cual', () => {
  const { from, to } = datos();
  const out = new Uint8Array(A.FB);
  for (const k of Object.values(A.KIND)) {
    A.render(k, A.duration(k, 1.3) + 0.001, 4, from, to, out, 1.3);
    assert.deepEqual(out, to[4]);
  }
});
