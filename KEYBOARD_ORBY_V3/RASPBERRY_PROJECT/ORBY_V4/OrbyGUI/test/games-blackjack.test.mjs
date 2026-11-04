import test from 'node:test';
import assert from 'node:assert/strict';
import blackjack, { valorMano, esBlackjack, liquidar, nuevoZapato } from '../src/games/blackjack.mjs';
import { lienzo, limpiarLienzo } from '../src/games/proto.mjs';

const C = (...rs) => rs.map((r) => ({ r, s: 0 }));

test('Blackjack: el as vale 11 o 1 según convenga', () => {
  assert.equal(valorMano(C(1, 13)), 21);
  assert.equal(valorMano(C(1, 1, 9)), 21);
  assert.equal(valorMano(C(1, 1)), 12);
  assert.equal(valorMano(C(1, 5, 9)), 15);
  assert.equal(valorMano(C(10, 12, 2)), 22);
  assert.ok(esBlackjack(C(1, 11)));
  assert.ok(!esBlackjack(C(7, 7, 7)), 'tres cartas que suman 21 no son blackjack');
});

test('Blackjack: lo que paga cada final de mano', () => {
  assert.deepEqual(liquidar(C(1, 13), C(10, 9), 20), { resultado: 'blackjack', devuelve: 50 });   // 3:2
  assert.deepEqual(liquidar(C(1, 13), C(1, 10), 20), { resultado: 'empate', devuelve: 20 });
  assert.deepEqual(liquidar(C(10, 9), C(1, 10), 20), { resultado: 'pierde', devuelve: 0 });
  assert.deepEqual(liquidar(C(10, 9), C(10, 8), 20), { resultado: 'gana', devuelve: 40 });
  assert.deepEqual(liquidar(C(10, 8), C(10, 8), 20), { resultado: 'empate', devuelve: 20 });
  assert.deepEqual(liquidar(C(10, 5), C(10, 6, 9), 20), { resultado: 'gana', devuelve: 40 });   // la banca se pasa
  assert.deepEqual(liquidar(C(10, 5, 9), C(10, 6, 9), 20), { resultado: 'pasa', devuelve: 0 }); // pasarse pierde antes
});

test('Blackjack: el zapato tiene seis barajas completas', () => {
  let s = 1;
  const z = nuevoZapato((tope) => { s = (s * 48271) % 2147483647; return s % tope; });
  assert.equal(z.length, 312);
  for (let r = 1; r <= 13; r++) assert.equal(z.filter((c) => c.r === r).length, 24);
});

test('Blackjack: apostar, repartir y plantarse mueve las fichas', () => {
  let reloj = 0;
  let s = 3;
  const partida = blackjack.crear({ ahora: () => reloj, aleatorio: (tope) => { s = (s * 48271) % 2147483647; return s % tope; } });
  const l = lienzo();
  const avanzar = (ms) => { for (let t = 0; t < ms; t += 8) { reloj += 8; partida.tick(8); limpiarLienzo(l); partida.dibujar(l); } };
  const pulsar = (p) => partida.tecla({ tipo: 'tecla', tecla: p, pantalla: p, pulsada: true });

  assert.equal(partida.puntos, 100);
  pulsar(9); pulsar(9);                 // apuesta 10 → 30
  partida.rueda({ delta: -40 });        // y la rueda la baja a 20
  pulsar(8);
  assert.equal(partida.puntos, 100, 'lo apostado sigue contando como tuyo hasta que se resuelve');
  avanzar(2000);
  pulsar(8);                            // plantarse (si no hubo blackjack, no hace nada)
  avanzar(8000);
  const fichas = partida.puntos;
  assert.ok([80, 100, 120, 130].includes(fichas), `con 20 apostados se pierde 20, se empata o se gana 20/30: ${fichas}`);
});
