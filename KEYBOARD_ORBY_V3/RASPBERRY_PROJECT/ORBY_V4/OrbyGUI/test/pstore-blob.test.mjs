// El archivo de perfiles desactivados (src/web/pstore-blob.mjs).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { encode, decode, canonical, entryFrom, edited, newUid, macroIdsUsed } from '../src/web/pstore-blob.mjs';

// Un icono de 360 bytes casi vacío, como los de verdad.
const icon = (seed) => {
  const b = new Uint8Array(360);
  for (let i = 0; i < 40; i++) b[(i * 7 + seed) % 360] = (i * 31 + seed) & 0xff;
  return Array.from(b, (x) => x.toString(16).padStart(2, '0')).join('');
};

function perfil(name, pages = 4) {
  return {
    name,
    pageCount: pages,
    pages: Array.from({ length: pages }, (_, pg) => ({
      labels: Array.from({ length: 20 }, (_, s) => (s % 3 ? '' : `T${s}`)),
      keys: Array.from({ length: 24 }, (_, s) => ({ modifier: s % 4, keycode: 4 + s })),
      rotary: Array.from({ length: 16 }, () => ({ type: 1, modifier: 0, keycode: 0 })),
      scroll: [{ detentsPerRev: 24, invert: false }, { detentsPerRev: 24, invert: true }],
      oledMask: 0b1111,
      icons: { 0: icon(pg), 1: icon(pg + 9), 2: icon(pg + 17), 3: icon(pg + 25) },
    })),
    profileIcon: icon(99),
  };
}

test('lo que se archiva se recupera idéntico', async () => {
  const lista = [entryFrom(perfil('OFIM')), entryFrom(perfil('ALTM', 1))];
  const back = await decode(await encode(lista));
  assert.equal(canonical(back), canonical(lista));
});

test('un perfil de cuatro páginas con iconos ocupa poco', async () => {
  const lista = [entryFrom(perfil('OFIM'))];
  const bytes = await encode(lista);
  assert.ok(bytes.length < 6 * 1024, `${bytes.length} B`);
  assert.ok(bytes.length * 4 < canonical(lista).length);
});

test('caben decenas de perfiles en el banco del teclado', async () => {
  const lista = Array.from({ length: 40 }, (_, i) => entryFrom(perfil(`P${i}`)));
  const bytes = await encode(lista);
  assert.ok(bytes.length < 112 * 1024 - 256, `${bytes.length} B`);
});

test('editar conserva el uid y sube la revisión', () => {
  const e = entryFrom(perfil('OFIM'), { uid: 'abc', rev: 3 });
  const n = edited(e, perfil('OFIM2'));
  assert.equal(n.uid, 'abc');
  assert.equal(n.rev, 4);
  assert.equal(n.name, 'OFIM2');
});

test('los uid no se repiten', () => {
  const s = new Set(Array.from({ length: 500 }, () => newUid()));
  assert.equal(s.size, 500);
});

test('un bloque ajeno o de una versión nueva se rechaza', async () => {
  await assert.rejects(decode(new Uint8Array([1, 2, 3, 4, 5, 6])));
  const futuro = await encode([entryFrom(perfil('X', 1))]);
  futuro[3] = 99;
  await assert.rejects(decode(futuro));
});

test('las macros de los perfiles archivados cuentan como en uso', () => {
  const p = perfil('X', 1);
  p.pages[0].keys[0] = { modifier: 0xF0, keycode: 12 };
  p.pages[0].rotary[2] = { type: 2, modifier: 0xF0, keycode: 40 };
  p.pages[0].rotary[3] = { type: 1, modifier: 0xF0, keycode: 41 };   // otro tipo: no cuenta
  const usadas = macroIdsUsed([entryFrom(p)], 0xF0, 2);
  assert.deepEqual([...usadas].sort((a, b) => a - b), [12, 40]);
});
