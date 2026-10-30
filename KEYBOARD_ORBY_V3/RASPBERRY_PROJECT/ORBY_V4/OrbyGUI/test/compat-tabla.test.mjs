import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { compareFw, validar, masReciente, fwRecomendado, appNecesaria } from '../src/compat-tabla.mjs';

const leer = (ruta) => readFileSync(new URL(ruta, import.meta.url), 'utf8');
const tablaDeLaApp = JSON.parse(leer('../src/compat.json'));
const versionApp = JSON.parse(leer('../package.json')).version;
const versionFw = (() => {
  const h = leer('../../include/orby_version.h');
  const mayor = h.match(/ORBY_FW_MAJOR\s+(\d+)/)[1];
  const menor = h.match(/ORBY_FW_MINOR\s+(\d+)/)[1];
  return `${mayor}.${menor}`;
})();

const tabla = {
  schema: 1, rev: 3, fwMin: '2.0',
  firmware: { '4.9': { minApp: '1.0.0' }, '4.10': { minApp: '1.1.0' }, '4.11': { minApp: '1.2.0' } },
};

// --- La puerta: lo que obliga a sacar app nueva solo cuando hace falta -----------------

test('el firmware del repositorio tiene entrada en compat.json', () => {
  assert.ok(
    appNecesaria(tablaDeLaApp, versionFw),
    `compat.json no conoce el firmware ${versionFw}: añade su entrada (y sube "rev")`,
  );
});

test('la app del repositorio ofrece el firmware del repositorio', () => {
  // Si falla, el firmware pide una app que aún no existe: toca subir la versión
  // de la app (package.json, tauri.conf.json, Cargo.toml) y publicarla antes.
  assert.ok(
    compareFw(appNecesaria(tablaDeLaApp, versionFw), versionApp) <= 0,
    `el firmware ${versionFw} pide la app ${appNecesaria(tablaDeLaApp, versionFw)} y package.json dice ${versionApp}`,
  );
  assert.equal(fwRecomendado(tablaDeLaApp, versionApp), versionFw);
});

test('compat.json es una tabla válida', () => {
  assert.ok(validar(tablaDeLaApp));
});

// --- La lógica --------------------------------------------------------------------------

test('4.10 es posterior a 4.9', () => {
  assert.equal(compareFw('4.10', '4.9'), 1);
  assert.equal(compareFw('1.1.4', '1.1.3'), 1);
  assert.equal(compareFw('4.0', '4'), 0);
});

test('cada app ofrece hasta el último firmware que cumple', () => {
  assert.equal(fwRecomendado(tabla, '1.0.5'), '4.9');
  assert.equal(fwRecomendado(tabla, '1.1.0'), '4.10');
  assert.equal(fwRecomendado(tabla, '1.1.9'), '4.10');
  assert.equal(fwRecomendado(tabla, '1.2.0'), '4.11');
});

test('una app anterior a todo lo de la tabla se queda en el mínimo', () => {
  assert.equal(fwRecomendado(tabla, '0.9'), '2.0');
});

test('un firmware nuevo que no pide app nueva se ofrece a la app ya instalada', () => {
  const mas = { ...tabla, rev: 4, firmware: { ...tabla.firmware, '4.12': { minApp: '1.1.0' } } };
  assert.equal(fwRecomendado(tabla, '1.1.0'), '4.10');
  assert.equal(fwRecomendado(mas, '1.1.0'), '4.12');
});

test('se descartan las tablas mal formadas', () => {
  assert.equal(validar(null), null);
  assert.equal(validar({ ...tabla, schema: 2 }), null);
  assert.equal(validar({ ...tabla, rev: 0 }), null);
  assert.equal(validar({ ...tabla, firmware: { '4.x': { minApp: '1.0' } } }), null);
  assert.equal(validar({ ...tabla, firmware: { '4.9': {} } }), null);
  assert.equal(validar({ ...tabla, firmware: undefined }), null);
});

test('entre varias tablas gana la de rev más alta, y una rota no gana nunca', () => {
  const vieja = { ...tabla, rev: 1 };
  assert.equal(masReciente(vieja, tabla).rev, 3);
  assert.equal(masReciente(tabla, vieja).rev, 3);
  assert.equal(masReciente({ ...tabla, rev: 9, schema: 7 }, vieja).rev, 1);
  assert.equal(masReciente(null, undefined), null);
});
