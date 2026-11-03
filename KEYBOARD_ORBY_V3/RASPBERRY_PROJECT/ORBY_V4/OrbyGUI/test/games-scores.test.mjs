import test from 'node:test';
import assert from 'node:assert/strict';
import { vacio, registrar, paraGuardar, resumen, serieGrafica, MAX_HISTORIAL, RECIENTES, formatearPuntos } from '../src/games/scores.mjs';

test('la primera partida es récord', () => {
  const e = registrar(undefined, 12, { ahora: 1000 });
  assert.equal(e.mejor, 12);
  assert.equal(e.partidas, 1);
  assert.equal(e.record, true);
  assert.deepEqual(e.historial, [{ t: 1000, p: 12 }]);
});

test('en un juego de puntos manda la mayor', () => {
  let e = registrar(vacio(), 10);
  e = registrar(paraGuardar(e), 7);
  assert.equal(e.mejor, 10);
  assert.equal(e.record, false);
  e = registrar(paraGuardar(e), 15);
  assert.equal(e.mejor, 15);
  assert.equal(e.record, true);
});

test('en un juego de tiempos manda la menor', () => {
  let e = registrar(vacio(), 300, { mejorSiMenor: true });
  e = registrar(paraGuardar(e), 450, { mejorSiMenor: true });
  assert.equal(e.mejor, 300);
  assert.equal(e.record, false);
  e = registrar(paraGuardar(e), 280, { mejorSiMenor: true });
  assert.equal(e.mejor, 280);
  assert.equal(e.record, true);
});

test('empatar el récord no cuenta como récord nuevo', () => {
  const e = registrar({ mejor: 10, partidas: 3, historial: [] }, 10);
  assert.equal(e.record, false);
});

test('el historial se recorta pero las partidas se siguen contando', () => {
  let e = vacio();
  for (let i = 0; i < MAX_HISTORIAL + 5; i++) e = registrar(paraGuardar(e), i, { ahora: i });
  assert.equal(e.historial.length, MAX_HISTORIAL);
  assert.equal(e.partidas, MAX_HISTORIAL + 5);
  assert.equal(e.historial[e.historial.length - 1].p, MAX_HISTORIAL + 4);
  assert.equal(e.historial[0].p, 5);
});

test('registrar no muta el estado de partida', () => {
  const antes = { mejor: 5, partidas: 1, historial: [{ t: 1, p: 5 }] };
  registrar(antes, 9);
  assert.deepEqual(antes, { mejor: 5, partidas: 1, historial: [{ t: 1, p: 5 }] });
});

test('lo que se guarda no lleva la marca de récord', () => {
  const g = paraGuardar(registrar(vacio(), 3));
  assert.deepEqual(Object.keys(g).sort(), ['historial', 'mejor', 'partidas']);
});

test('el resumen de un juego sin partidas', () => {
  const r = resumen(undefined);
  assert.deepEqual(r, { mejor: null, partidas: 0, ultima: null, recientes: [], detalle: [], totales: null, ultimaStats: null });
});

test('el desglose por partes se guarda y se resume aparte, y su récord no se pierde', () => {
  let e = registrar(vacio(), 300, { mejorSiMenor: true, detalle: [250, 300, 350], ahora: 1 });
  e = registrar(paraGuardar(e), 320, { mejorSiMenor: true, detalle: [270, 280, 410], ahora: 2 });
  const g = paraGuardar(e);
  assert.deepEqual(g.mejorDetalle, [250, 280, 350]);
  assert.deepEqual(g.historial[1].d, [270, 280, 410]);
  const r = resumen(g);
  assert.deepEqual(r.detalle.map((d) => d.ultima), [270, 280, 410]);
  assert.deepEqual(r.detalle.map((d) => d.mejor), [250, 280, 350]);
});

test('los tiempos de reacción se enseñan en milisegundos', () => {
  assert.equal(formatearPuntos('reaccion', 287.4), '287 ms');
  assert.equal(formatearPuntos('reaccion', 287.4, { corto: true }), '287ms');
});

test('el resumen enseña las más recientes primero', () => {
  let e = vacio();
  for (let i = 1; i <= RECIENTES + 3; i++) e = registrar(paraGuardar(e), i * 10, { ahora: i });
  const r = resumen(paraGuardar(e));
  assert.equal(r.recientes.length, RECIENTES);
  assert.equal(r.recientes[0].p, (RECIENTES + 3) * 10);
  assert.equal(r.ultima, (RECIENTES + 3) * 10);
  assert.equal(r.mejor, (RECIENTES + 3) * 10);
});

test('no se guardan ni se calculan medias de varias partidas', () => {
  let e = registrar(vacio(), 10, { detalle: [10, 10] });
  e = registrar(paraGuardar(e), 20, { detalle: [20, 20] });
  const g = paraGuardar(e);
  assert.deepEqual(Object.keys(g).sort(), ['historial', 'mejor', 'mejorDetalle', 'partidas']);
  const r = resumen(g);
  assert.equal(r.media, undefined);
  assert.equal(r.mediaSiempre, undefined);
  assert.equal(r.detalle[0].media, undefined);
});

test('la gráfica: sin partidas no hay puntos y con una queda centrada', () => {
  assert.deepEqual(serieGrafica([], 100, 30), []);
  assert.deepEqual(serieGrafica([{ t: 1, p: 5 }], 100, 30), [[50, 15]]);
});

test('la gráfica pone arriba lo mejor, según el juego', () => {
  const h = [{ t: 1, p: 10 }, { t: 2, p: 30 }];
  const mayor = serieGrafica(h, 100, 30, { margen: 0 });
  assert.equal(mayor[0][1], 30);   // 10 es lo peor: abajo
  assert.equal(mayor[1][1], 0);    // 30 es lo mejor: arriba
  const menor = serieGrafica(h, 100, 30, { mejorSiMenor: true, margen: 0 });
  assert.equal(menor[0][1], 0);    // 10 ms es lo mejor: arriba
  assert.equal(menor[1][1], 30);
});

test('la gráfica con todas las partidas iguales no divide entre cero', () => {
  const pts = serieGrafica([{ t: 1, p: 7 }, { t: 2, p: 7 }, { t: 3, p: 7 }], 100, 30);
  for (const [, y] of pts) assert.equal(y, 15);
});

test('las estadísticas de una partida suman entre partidas y el máximo se queda con el mayor', () => {
  let e = registrar(vacio(), 40, { ahora: 1, stats: { suma: { tienda: 2, tanque: 1, total: 3 }, max: { aLaVez: 2 } } });
  e = registrar(paraGuardar(e), 90, { ahora: 2, stats: { suma: { tienda: 1, camion: 4, total: 5 }, max: { aLaVez: 1 } } });
  const g = paraGuardar(e);
  assert.deepEqual(g.totales, { suma: { tienda: 3, tanque: 1, camion: 4, total: 8 }, max: { aLaVez: 2 } });
  // La de cada partida, aparte, para enseñar la última.
  assert.deepEqual(g.historial[1].s, { suma: { tienda: 1, camion: 4, total: 5 }, max: { aLaVez: 1 } });
  const r = resumen(g);
  assert.deepEqual(r.totales, g.totales);
  assert.deepEqual(r.ultimaStats.max, { aLaVez: 1 });
});

test('un juego sin estadísticas no guarda totales', () => {
  const g = paraGuardar(registrar(vacio(), 5));
  assert.equal(g.totales, undefined);
  assert.equal(resumen(g).totales, null);
});
