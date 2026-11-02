import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { TIPS, CATEGORIAS } from '../src/tips.mjs';

// compat.js importa compat.json con atributos de importación, que Node 20 no admite; las
// claves de FEATURES se leen del fuente.
const featuresSrc = readFileSync(new URL('../src/compat.js', import.meta.url), 'utf8');
const FEATURES = new Set([...featuresSrc.slice(featuresSrc.indexOf('export const FEATURES')).matchAll(/^  (\w+):\s*\{ since/gm)].map((m) => m[1]));

const VISTAS = ['view-dashboard', 'view-profiles', 'view-settings', 'view-auto'];

test('hay 19 consejos con ids únicos', () => {
  assert.equal(TIPS.length, 19);
  assert.equal(new Set(TIPS.map((t) => t.id)).size, TIPS.length);
});

test('los ids sirven de nombre de fichero de vídeo', () => {
  for (const t of TIPS) assert.match(t.id, /^[a-z0-9-]+$/, t.id);
});

test('cada categoría es una de las seis', () => {
  assert.equal(CATEGORIAS.length, 6);
  for (const t of TIPS) assert.ok(CATEGORIAS.includes(t.categoria), t.id);
});

test('cada requiere existe en FEATURES', () => {
  assert.ok(FEATURES.size > 10, 'no se han leído las FEATURES');
  for (const t of TIPS) if (t.requiere) assert.ok(FEATURES.has(t.requiere), `${t.id}: ${t.requiere}`);
});

test('cada acción apunta a una vista real y tiene etiqueta', () => {
  for (const t of TIPS) {
    if (!t.accion) continue;
    assert.ok(VISTAS.includes(t.accion.vista), `${t.id}: ${t.accion.vista}`);
    assert.ok(t.accion.etiqueta?.length > 3, t.id);
  }
});

test('las tarjetas de Ajustes a las que se lleva existen en index.html', () => {
  const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
  for (const t of TIPS) {
    const card = t.accion?.params?.card;
    if (card) assert.ok(html.includes(`id="${card}"`), `${t.id}: falta #${card}`);
  }
});

test('textos y títulos no están vacíos', () => {
  for (const t of TIPS) {
    assert.ok(t.titulo.length > 5, t.id);
    assert.ok(t.texto.length > 40, t.id);
  }
});
