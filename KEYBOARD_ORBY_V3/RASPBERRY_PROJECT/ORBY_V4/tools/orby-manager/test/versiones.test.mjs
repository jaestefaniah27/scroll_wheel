// El número de versión de cada publicación sale de aquí.
//
// Estos tests existen por un fallo concreto: `subirSemver` no entendía las preversiones y
// se negaba con «no tiene forma x.y.z». Con la app en `1.0.0-alpha` —que era la versión que
// había puesta— eso significaba que **ninguna publicación era posible desde el panel**, y
// el error solo aparecía al pulsar el botón, nunca antes.

import test from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const { subirSemver } = require('../versiones.js');

test('a una preversión no se le suma: se gradúa', () => {
  // `1.0.0-alpha` ya «vale» 1.0.0 menos un pelo. Sumarle uno se saltaría la 1.0.0 sin
  // haberla publicado nunca.
  assert.equal(subirSemver('1.0.0-alpha', 'patch', 'test'), '1.0.0');
  assert.equal(subirSemver('1.0.0-alpha', 'minor', 'test'), '1.1.0');
  assert.equal(subirSemver('1.0.0-alpha', 'major', 'test'), '2.0.0');
});

test('sin preversión, se comporta como siempre', () => {
  assert.equal(subirSemver('1.0.0', 'patch', 'test'), '1.0.1');
  assert.equal(subirSemver('1.2.3', 'minor', 'test'), '1.3.0');
  assert.equal(subirSemver('1.2.3', 'major', 'test'), '2.0.0');
});

test('se cuenta por partes, no como decimal', () => {
  // El mismo fallo que ya tenía su prueba en orby-core: 1.0.10 va después de 1.0.9.
  assert.equal(subirSemver('1.0.9', 'patch', 'test'), '1.0.10');
});

test('los metadatos de build no cuentan', () => {
  assert.equal(subirSemver('1.0.0-rc.1+build7', 'patch', 'test'), '1.0.0');
});

test('una versión ilegible falla, y dice dónde estaba', () => {
  // Se nombra el fichero a propósito: con tres ficheros de versión, «no tiene forma x.y.z»
  // a secas obliga a abrirlos los tres.
  assert.throws(() => subirSemver('no.es.version', 'patch', 'OrbyGUI/package.json'),
    /OrbyGUI\/package\.json/);
});

test('un tipo de bump inventado falla', () => {
  assert.throws(() => subirSemver('1.0.0', 'enorme', 'test'), /Tipo de bump inválido/);
});
