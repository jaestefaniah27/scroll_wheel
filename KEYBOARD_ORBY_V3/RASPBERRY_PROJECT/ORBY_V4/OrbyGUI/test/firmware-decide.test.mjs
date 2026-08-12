// Cuándo se flashea el teclado solo.
//
// Estos tests valen por lo que impiden, no por lo que confirman: cada `porQueNo` que
// devuelve un motivo es un flasheo que NO se dispara. Durante la copia el teclado se
// reinicia en el cargador de la ROM y deja de existir unos segundos, así que un falso
// positivo aquí es un teclado que se muere a mitad de un atajo.

import test from 'node:test';
import assert from 'node:assert/strict';

import { porQueNo, OCIO_MS } from '../src/firmware-decide.mjs';

/// El caso bueno: todo en su sitio y el teclado quieto desde hace rato.
function listo(extra = {}) {
  return {
    conectado: true,
    deviceInfo: { fw: '4.5' },
    disponible: true,
    ultimaVersion: '4.6',
    ocupado: false,
    versionFallida: null,
    automatico: true,
    sinGuardar: false,
    grabando: false,
    reproduciendo: false,
    quietoMs: OCIO_MS + 1,
    ...extra,
  };
}

test('con todo en su sitio, se flashea', () => {
  assert.equal(porQueNo(listo()), null);
});

test('no se flashea sin teclado delante', () => {
  assert.match(porQueNo(listo({ conectado: false })), /no hay teclado/);
  // Conectado pero sin handshake todavía: no se sabe ni qué versión lleva.
  assert.match(porQueNo(listo({ deviceInfo: null })), /no hay teclado/);
});

test('no se flashea si no hay versión nueva', () => {
  assert.match(porQueNo(listo({ disponible: false })), /no hay firmware nuevo/);
  assert.match(porQueNo(listo({ ultimaVersion: null })), /no hay firmware nuevo/);
});

test('no se pisa un proceso ya en marcha', () => {
  assert.match(porQueNo(listo({ ocupado: true })), /ya hay algo en marcha/);
});

test('una versión que ya falló no se reintenta en bucle', () => {
  // Si el .uf2 de la 4.6 no se pudo copiar, volver a intentarlo cada minuto no lo arregla
  // y sí llena el log de errores.
  assert.match(porQueNo(listo({ versionFallida: '4.6' })), /ya falló/);
  // Pero una versión distinta sí se intenta: el fallo era de aquella, no de esta.
  assert.equal(porQueNo(listo({ versionFallida: '4.5' })), null);
});

test('el interruptor de Ajustes manda', () => {
  assert.match(porQueNo(listo({ automatico: false })), /automático está apagado/);
});

test('no se flashea con cambios sin guardar en Flash', () => {
  // El reinicio en el cargador se lleva lo que solo esté en RAM, y ahí es exactamente
  // donde viven las variaciones por aplicación.
  assert.match(porQueNo(listo({ sinGuardar: true })), /sin guardar/);
});

test('no se flashea a mitad de una grabación ni de una reproducción', () => {
  assert.match(porQueNo(listo({ grabando: true })), /grabando/);
  assert.match(porQueNo(listo({ reproduciendo: true })), /reproduciéndose/);
});

test('hace falta que el teclado lleve un rato quieto', () => {
  // Justo por debajo del umbral: todavía no.
  const recien = porQueNo(listo({ quietoMs: OCIO_MS - 1 }));
  assert.match(recien, /se ha usado hace/);
  // Y el mensaje dice cuánto, que es lo que hace el log legible.
  assert.match(recien, /\d+ s/);

  // Justo en el umbral: ya.
  assert.equal(porQueNo(listo({ quietoMs: OCIO_MS })), null);
});

test('acabar de teclear pesa más que llevar horas conectado', () => {
  // El caso que de verdad importa: el usuario está trabajando. Da igual todo lo demás.
  assert.match(porQueNo(listo({ quietoMs: 0 })), /se ha usado hace 0 s/);
});
