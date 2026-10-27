import test from 'node:test';
import assert from 'node:assert/strict';
import { motivoParaEsperar, INACTIVO_MS } from '../src/fw-auto.mjs';

const listo = {
  activado: true, conectado: true, disponible: true, version: '4.11', ocupado: false,
  sinGuardar: false, bootsel: true, inactivoMs: INACTIVO_MS, fallida: null,
};

test('con todo a favor se actualiza', () => {
  assert.equal(motivoParaEsperar(listo), null);
});

test('no se actualiza mientras el PC está en uso', () => {
  assert.ok(motivoParaEsperar({ ...listo, inactivoMs: INACTIVO_MS - 1 }));
});

test('sin dato de inactividad se espera, no se flashea a ciegas', () => {
  assert.ok(motivoParaEsperar({ ...listo, inactivoMs: null }));
});

test('los cambios sin guardar en Flash lo frenan', () => {
  assert.ok(motivoParaEsperar({ ...listo, sinGuardar: true }));
});

test('un firmware sin BOOTSEL por serie no se actualiza solo', () => {
  // Habría que desenchufarlo con el botón pulsado: imposible sin nadie delante.
  assert.ok(motivoParaEsperar({ ...listo, bootsel: false }));
});

test('una versión que ya falló no se reintenta sola, pero una más nueva sí', () => {
  assert.ok(motivoParaEsperar({ ...listo, fallida: '4.11' }));
  assert.equal(motivoParaEsperar({ ...listo, fallida: '4.10' }), null);
});

test('desactivado, sin teclado, sin versión nueva u ocupado: no', () => {
  for (const cambio of [{ activado: false }, { conectado: false }, { disponible: false },
    { version: null }, { ocupado: true }]) {
    assert.ok(motivoParaEsperar({ ...listo, ...cambio }), JSON.stringify(cambio));
  }
});
