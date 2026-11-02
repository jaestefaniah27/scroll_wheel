import test from 'node:test';
import assert from 'node:assert/strict';
import { puedeAvisar, siguienteConsejo, MIN_USO_MS, ENTRE_AVISOS_MS } from '../src/tips-pick.mjs';

const BASE = {
  ahora: 10_000_000, inicioApp: 10_000_000 - MIN_USO_MS - 1, ultimoAviso: null,
  activo: true, visible: true, conectado: true, sinGuardar: false, hayAviso: false,
};
const con = (cambios) => ({ ...BASE, ...cambios });

test('avisa pasados dos minutos de uso', () => {
  assert.equal(puedeAvisar(BASE), true);
});

test('no avisa antes de dos minutos', () => {
  assert.equal(puedeAvisar(con({ inicioApp: BASE.ahora - MIN_USO_MS + 1 })), false);
});

test('no avisa dos veces el mismo día', () => {
  assert.equal(puedeAvisar(con({ ultimoAviso: BASE.ahora - ENTRE_AVISOS_MS + 1 })), false);
});

test('avisa al día siguiente', () => {
  assert.equal(puedeAvisar(con({ ultimoAviso: BASE.ahora - ENTRE_AVISOS_MS })), true);
});

test('un último aviso en el futuro (reloj hacia atrás) no bloquea', () => {
  assert.equal(puedeAvisar(con({ ultimoAviso: BASE.ahora + 5 * ENTRE_AVISOS_MS })), true);
});

test('no avisa con cambios sin guardar', () => {
  assert.equal(puedeAvisar(con({ sinGuardar: true })), false);
});

test('no avisa sin teclado conectado', () => {
  assert.equal(puedeAvisar(con({ conectado: false })), false);
});

test('no avisa con la ventana oculta', () => {
  assert.equal(puedeAvisar(con({ visible: false })), false);
});

test('no avisa con otro aviso en pantalla', () => {
  assert.equal(puedeAvisar(con({ hayAviso: true })), false);
});

test('no avisa si los consejos están apagados', () => {
  assert.equal(puedeAvisar(con({ activo: false })), false);
});

const TIPS = [{ id: 'a' }, { id: 'b', requiere: 'helpMode' }, { id: 'c' }];

test('siguiente salta los vistos', () => {
  assert.equal(siguienteConsejo(TIPS, ['a'], () => true), 'b');
});

test('siguiente salta los que requieren algo que falta', () => {
  assert.equal(siguienteConsejo(TIPS, ['a'], (k) => k !== 'helpMode'), 'c');
});

test('siguiente ignora ids vistos que ya no existen', () => {
  assert.equal(siguienteConsejo(TIPS, ['viejo', 'a', 'b'], () => true), 'c');
});

test('siguiente devuelve null si no queda ninguno', () => {
  assert.equal(siguienteConsejo(TIPS, ['a', 'b', 'c'], () => true), null);
});

test('siguiente tolera vistos mal formados', () => {
  assert.equal(siguienteConsejo(TIPS, undefined, () => true), 'a');
  assert.equal(siguienteConsejo(TIPS, 'a', () => true), 'a');
});
