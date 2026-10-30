import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as A from '../src/oled-anim.mjs';

// El dashboard tiene que animar las pantallas EXACTAMENTE como el teclado. La
// referencia es include/oled_anim.h: tools/test/anim_golden.cpp lo ejecuta y
// apunta el CRC de cada fotograma en el fichero de abajo. Aquí se hace lo mismo
// con la copia en JS y tienen que salir iguales, bit a bit.

const golden = readFileSync(new URL('./fixtures/oled-anim-golden.txt', import.meta.url), 'utf8')
  .trim().split(/\r?\n/);   // git en Windows lo puede sacar con CRLF

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
  let comparados = 0, pick = 0;
  for (const linea of golden) {
    const p = linea.split(' ');
    if (p[0] === 'press') continue;
    // `pick <origen> <r> <ti> <crc>`: PICK depende además de la pantalla de origen.
    const esPick = p[0] === 'pick';
    const [k, origen, r, ti] = esPick ? [A.KIND.PICK, ...p.slice(1, 4).map(Number)]
      : [Number(p[0]), 4, Number(p[1]), Number(p[2])];
    let c = 0;
    for (let i = 0; i < A.N; i++) {
      A.render(k, ti * Math.fround(0.0125), i, from, to, out, rates[r], origen);
      c = crc32(c, out);
    }
    assert.equal(c.toString(16).padStart(8, '0'), p[esPick ? 4 : 3],
      `tipo ${k}, origen ${origen}, ritmo ${rates[r]}, t=${ti}`);
    if (esPick) pick++; else comparados++;
  }
  assert.equal(comparados, 9 * 2 * 41);
  assert.equal(pick, 2 * 2 * 41);
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
  for (const origen of [0, 4, 9]) {
    A.render(A.KIND.PICK, A.duration(A.KIND.PICK, 1.3) + 0.001, 6, from, to, out, 1.3, origen);
    assert.deepEqual(out, to[6]);
  }
});

test('un tipo desconocido dura 0 y deja el destino', () => {
  const { from, to } = datos();
  const out = new Uint8Array(A.FB);
  assert.equal(A.duration(99, 1), 0);
  A.render(99, 0, 2, from, to, out, 1);
  assert.deepEqual(out, to[2]);
});
