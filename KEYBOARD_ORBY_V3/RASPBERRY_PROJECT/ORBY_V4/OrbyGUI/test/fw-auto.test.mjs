import test from 'node:test';
import assert from 'node:assert/strict';
import { motivoParaEsperar, esActividadDelTeclado, INACTIVO_MS } from '../src/fw-auto.mjs';

const listo = {
  activado: true, conectado: true, disponible: true, version: '4.11', ocupado: false,
  sinGuardar: false, sincronizando: false, bootsel: true, inactivoMs: INACTIVO_MS, fallida: null,
};

test('con todo a favor se actualiza', () => {
  assert.equal(motivoParaEsperar(listo), null);
});

test('no se actualiza mientras el teclado está en uso', () => {
  assert.ok(motivoParaEsperar({ ...listo, inactivoMs: INACTIVO_MS - 1 }));
});

test('hay que esperar 2 minutos sin tocar el teclado', () => {
  assert.equal(INACTIVO_MS, 2 * 60 * 1000);
});

test('cuenta como uso lo que se toca del teclado, no lo que manda solo', () => {
  for (const l of ['KEY_EV:3:1', 'ENC:1:-2', 'ENC_SW:2:1', 'WHEEL:5:2100', 'MODE:MENU', 'EV:CTX:OFIM:0:1']) {
    assert.ok(esActividadDelTeclado(l), l);
  }
  for (const l of ['EV:SCR:1:00ff', 'EV:INV:5', 'EV:DISP:0', 'STATE:MODE:NORMAL', 'HOST_APP:OK:1']) {
    assert.ok(!esActividadDelTeclado(l), l);
  }
});

test('sin dato de inactividad se espera, no se flashea a ciegas', () => {
  assert.ok(motivoParaEsperar({ ...listo, inactivoMs: null }));
});

test('no se reinicia a mitad de una sincronización con el teclado', () => {
  assert.ok(motivoParaEsperar({ ...listo, sincronizando: true }));
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
