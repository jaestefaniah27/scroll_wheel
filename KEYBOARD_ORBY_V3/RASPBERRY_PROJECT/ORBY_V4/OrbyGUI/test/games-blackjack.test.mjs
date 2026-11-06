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

// Una partida con el reloj y el azar controlados. `guardados` recoge cada aviso de progreso.
function montar(semilla, saldo = null) {
  let reloj = 0;
  let s = semilla;
  const guardados = [];
  const partida = blackjack.crear({
    ahora: () => reloj,
    aleatorio: (tope) => { s = (s * 48271) % 2147483647; return s % tope; },
    saldo,
    progreso: (p) => guardados.push(p),
  });
  const l = lienzo();
  const avanzar = (ms) => { for (let t = 0; t < ms; t += 8) { reloj += 8; partida.tick(8); limpiarLienzo(l); partida.dibujar(l); } };
  const pulsar = (p) => partida.tecla({ tipo: 'tecla', tecla: p, pantalla: p, pulsada: true });
  // Una mano entera apostando lo mínimo: reparte, se planta si hace falta y deja ver el resultado.
  const jugarMano = () => {
    const antes = partida.manos;
    pulsar(8);
    avanzar(3500);                 // el reparto es lento: cuatro cartas, una cada 650 ms
    if (partida.manos === antes) { pulsar(8); avanzar(20000); } else avanzar(6000);
  };
  return { partida, guardados, avanzar, pulsar, jugarMano, l };
}

test('Blackjack: apostar, repartir y plantarse mueve las fichas y se guarda al momento', () => {
  const { partida, guardados, avanzar, pulsar } = montar(3);

  assert.equal(partida.puntos, 100);
  pulsar(9); pulsar(9);                 // apuesta 10 → 30
  partida.rueda({ delta: -40 });        // y la rueda la baja a 20
  pulsar(8);
  assert.deepEqual(guardados.at(-1), { saldo: 80, pico: 100 }, 'al repartir ya se descuenta lo apostado');
  avanzar(3500);
  if (partida.manos === 0) { pulsar(8); avanzar(20000); }   // plantarse (si no hubo blackjack)
  assert.equal(partida.manos, 1);
  const { saldo, pico } = guardados.at(-1);
  assert.ok([80, 100, 120, 130].includes(saldo), `con 20 apostados se pierde 20, se empata o se gana 20/30: ${saldo}`);
  assert.equal(pico, Math.max(100, saldo));
  assert.equal(partida.puntos, pico, 'los puntos son el máximo alcanzado, no las fichas del final');
});

test('Blackjack: no tiene tope de manos', () => {
  const { partida, jugarMano } = montar(11, 100000);
  for (let i = 0; i < 25; i++) jugarMano();
  assert.equal(partida.terminado, false);
  assert.ok(partida.manos >= 25);
});

test('Blackjack: arranca con el saldo guardado y con 100 si no hay o es menor que la apuesta mínima', () => {
  assert.equal(montar(5, 340).partida.puntos, 340);
  assert.equal(montar(5).partida.puntos, 100);
  assert.equal(montar(5, 0).partida.puntos, 100);
  assert.equal(montar(5, 5).partida.puntos, 100);
  assert.equal(montar(5, null).partida.puntos, 100);
});

test('Blackjack: quedarse a cero da 100 más y la recarga no cuenta como récord', () => {
  let perdidas = 0;
  for (let semilla = 1; semilla <= 60; semilla++) {
    const { partida, guardados, jugarMano } = montar(semilla, 10);
    jugarMano();
    const { saldo, pico } = guardados.at(-1);
    assert.ok(saldo >= 10, `nunca se queda por debajo de la apuesta mínima (semilla ${semilla}): ${saldo}`);
    if (pico === 10 && saldo === 100) perdidas++;   // perdió la mano: sin fichas, recibe 100
    assert.equal(partida.puntos, pico);
    if (saldo === 100 && pico === 10) assert.ok(true);
    else assert.ok(pico >= 10 && pico <= 25, `ganó o empató: pico ${pico}`);
  }
  assert.ok(perdidas > 0, 'alguna de las 60 partidas tiene que perder la mano');
});

test('Blackjack: tras perder todo se ve el regalo y la siguiente apuesta ya descuenta del saldo nuevo', () => {
  for (let semilla = 1; semilla <= 60; semilla++) {
    const { partida, guardados, jugarMano, pulsar } = montar(semilla, 10);
    jugarMano();
    if (guardados.at(-1).saldo !== 100 || guardados.at(-1).pico !== 10) continue;
    pulsar(8);
    assert.deepEqual(guardados.at(-1), { saldo: 90, pico: 10 });
    assert.equal(partida.puntos, 10);
    return;
  }
  assert.fail('no hubo ninguna partida perdida para comprobar');
});

const tinta = (fb) => fb.reduce((n, b) => n + b.toString(2).replace(/0/g, '').length, 0);

test('Blackjack: las cartas salen despacio y la tapada de la banca se da la vuelta al destaparse', () => {
  for (let semilla = 1; semilla <= 40; semilla++) {
    const { partida, avanzar, pulsar, l } = montar(semilla, 1000);
    pulsar(8);
    // Cada carta tarda 650 ms: a los 700 ms solo se ha enseñado la primera.
    avanzar(700);
    assert.equal(tinta(l[1]), 0, 'la segunda pantalla de la banca sigue vacía');
    avanzar(2900);
    if (partida.manos !== 0) continue;            // blackjack al repartir: no hay turno que probar
    avanzar(300);                                  // la animación de abrirse ya acabó
    const tapada = tinta(l[0]);
    pulsar(8);                                     // planto: se destapa la carta
    avanzar(225);                                  // mitad de la vuelta: la carta es una línea
    const mitad = tinta(l[0]);
    avanzar(1000);                                 // vuelta acabada: la cara, con su índice y su palo
    const cara = tinta(l[0]);
    assert.ok(mitad < tapada, `a media vuelta la carta se estrecha (${mitad} < ${tapada})`);
    assert.ok(cara > mitad, `y al acabar vuelve a ensancharse (${cara} > ${mitad})`);
    return;
  }
  assert.fail('ninguna semilla repartió una mano sin blackjack');
});

test('Blackjack: el resultado se queda lo bastante para ver por qué se ganó o se perdió', () => {
  const { partida, avanzar, pulsar, guardados } = montar(7, 1000);
  pulsar(8);                                       // reparte
  let t = 0;
  while (partida.manos === 0 && t < 40000) {       // y se juega hasta que la mano se resuelve
    avanzar(8); t += 8;
    if (t === 3504) pulsar(8);                     // se planta (si no hubo blackjack)
  }
  assert.equal(partida.manos, 1, 'la mano se resolvió');
  const n = guardados.length;
  avanzar(1000);
  pulsar(8);                                       // a 1 s del resultado una tecla no lo salta
  avanzar(700);
  pulsar(8);                                       // a 1,7 s sí: pasa a apostar...
  assert.equal(guardados.length, n, 'saltar el resultado no reparte');
  pulsar(8);                                       // ...y ahora esta tecla reparte la mano siguiente
  assert.equal(guardados.length, n + 1);
});
