import test from 'node:test';
import assert from 'node:assert/strict';
import { recorrer } from '../src/games/flujo.mjs';
import { crearMenu, MAX_JUEGOS } from '../src/games/menu.mjs';
import { crearFinal } from '../src/games/final.mjs';
import { JUEGOS } from '../src/games/catalogo.mjs';
import { lienzo, limpiarLienzo, FB } from '../src/games/proto.mjs';
import { formatearPuntos } from '../src/games/scores.mjs';
import { etiquetaValor, subir } from '../src/games/dibujo.mjs';

const tinta = (fb) => fb.reduce((s, b) => s + b.toString(2).replace(/0/g, '').length, 0);

// ---------------------------------------------------------------- el recorrido

// Un `ops` de mentira: cada llamada saca el siguiente resultado de su guion, y todo queda
// anotado para comprobar el orden.
function ops(guion) {
  const traza = [];
  const siguiente = (clave) => {
    const r = guion[clave].shift();
    assert.ok(r, `el guion de ${clave} se quedó sin resultados`);
    return r;
  };
  return {
    traza,
    juegoPorId: (id) => ({ id, nombre: id }),
    menu: async () => { traza.push('menu'); return siguiente('menu'); },
    partida: async (j) => { traza.push(`partida:${j.id}`); return siguiente('partida'); },
    final: async (j, p) => { traza.push(`final:${j.id}:${p}`); return siguiente('final'); },
    cerrar: async () => { traza.push('cerrar'); },
  };
}

const fin = (valor, extra = {}) => ({ motivo: 'fin', valor, puntos: 0, ...extra });

test('menú → partida → final → otra vez → final → salir, y cierra el modo al acabar', async () => {
  const o = ops({
    menu: [fin({ accion: 'jugar', id: 'topos' })],
    partida: [fin(undefined, { puntos: 7 }), fin(undefined, { puntos: 9 })],
    final: [fin({ accion: 'otra' }), fin({ accion: 'salir' })],
  });
  await recorrer(o);
  assert.deepEqual(o.traza, ['menu', 'partida:topos', 'final:topos:7', 'partida:topos', 'final:topos:9', 'cerrar']);
});

test('desde el final se vuelve al menú y se elige otro juego', async () => {
  const o = ops({
    menu: [fin({ accion: 'jugar', id: 'topos' }), fin({ accion: 'jugar', id: 'simon' })],
    partida: [fin(undefined, { puntos: 3 }), fin(undefined, { puntos: 5 })],
    final: [fin({ accion: 'menu' }), fin({ accion: 'salir' })],
  });
  await recorrer(o);
  assert.deepEqual(o.traza, ['menu', 'partida:topos', 'final:topos:3', 'menu', 'partida:simon', 'final:simon:5', 'cerrar']);
});

test('empezar por un juego (desde el botón de la app) se salta el menú', async () => {
  const o = ops({
    menu: [],
    partida: [fin(undefined, { puntos: 4 })],
    final: [fin({ accion: 'salir' })],
  });
  await recorrer(o, { id: 'sintoniza', nombre: 'Sintoniza' });
  assert.deepEqual(o.traza, ['partida:sintoniza', 'final:sintoniza:4', 'cerrar']);
});

test('la tecla 10 en mitad de una partida vuelve al menú sin puntuar', async () => {
  const o = ops({
    menu: [fin({ accion: 'jugar', id: 'topos' }), fin({ accion: 'salir' })],
    partida: [{ motivo: 'salir', puntos: 3 }],
    final: [],
  });
  await recorrer(o);
  assert.deepEqual(o.traza, ['menu', 'partida:topos', 'menu', 'cerrar']);
});

test('la tecla 10 en el menú sale del modo juego', async () => {
  const o = ops({ menu: [{ motivo: 'salir' }], partida: [], final: [] });
  await recorrer(o);
  assert.deepEqual(o.traza, ['menu', 'cerrar']);
});

test('la tecla 10 en la pantalla final vuelve al menú', async () => {
  const o = ops({
    menu: [fin({ accion: 'jugar', id: 'simon' }), fin({ accion: 'salir' })],
    partida: [fin(undefined, { puntos: 2 })],
    final: [{ motivo: 'salir' }],
  });
  await recorrer(o);
  assert.deepEqual(o.traza, ['menu', 'partida:simon', 'final:simon:2', 'menu', 'cerrar']);
});

test('«Parar» desde la app cierra el modo juego', async () => {
  const o = ops({ menu: [{ motivo: 'cancelado' }], partida: [], final: [] });
  await recorrer(o);
  assert.deepEqual(o.traza, ['menu', 'cerrar']);
});

for (const motivo of ['menu', 'timeout', 'desconectado']) {
  test(`si el teclado sale solo (${motivo}), no se le manda GAME:0 de más`, async () => {
    const o = ops({ menu: [fin({ accion: 'jugar', id: 'topos' })], partida: [{ motivo }], final: [] });
    await recorrer(o);
    assert.deepEqual(o.traza, ['menu', 'partida:topos']);
  });
}

test('si algo falla a mitad, el modo juego se cierra igualmente', async () => {
  const o = ops({ menu: [], partida: [], final: [] });
  o.menu = async () => { o.traza.push('menu'); throw new Error('sin teclado'); };
  await assert.rejects(recorrer(o), /sin teclado/);
  assert.deepEqual(o.traza, ['menu', 'cerrar']);
});

// ---------------------------------------------------------------- el menú del teclado

function correrMenu(juegos, teclas) {
  let reloj = 0;
  const p = crearMenu(juegos).crear({});
  const l = lienzo();
  const paso = (ms) => { reloj += ms; p.tick(ms); limpiarLienzo(l); p.dibujar(l); };
  paso(500);   // pasa la guarda de las pulsaciones heredadas
  for (const pantalla of teclas) {
    p.tecla({ tipo: 'tecla', tecla: pantalla, pantalla, pulsada: true });
    paso(200);
  }
  return { p, l, paso };
}

test('menú: cada juego en su pantalla y SALIR en la décima', () => {
  const { l } = correrMenu(JUEGOS, []);
  for (let i = 0; i < JUEGOS.length; i++) assert.ok(tinta(l[i]) > 0, `la pantalla ${i + 1} debería llevar ${JUEGOS[i].nombre}`);
  for (let i = JUEGOS.length; i < 9; i++) assert.equal(tinta(l[i]), 0, `la pantalla ${i + 1} debería estar vacía`);
  assert.ok(tinta(l[9]) > 0, 'SALIR');
});

test('menú: pulsar la pantalla de un juego lo elige', () => {
  const { p } = correrMenu(JUEGOS, [2]);
  assert.equal(p.terminado, true);
  assert.deepEqual(p.valor, { accion: 'jugar', id: JUEGOS[1].id });
});

test('menú: la pantalla 10 sale', () => {
  const { p } = correrMenu(JUEGOS, [10]);
  assert.deepEqual(p.valor, { accion: 'salir' });
});

test('menú: una pantalla sin juego no hace nada', () => {
  const { p } = correrMenu(JUEGOS, [9]);
  assert.equal(p.terminado, false);
  assert.equal(p.valor, null);
});

test('menú: ignora las pulsaciones que llegan nada más abrirse', () => {
  const p = crearMenu(JUEGOS).crear({});
  p.tick(50);
  p.tecla({ tipo: 'tecla', tecla: 1, pantalla: 1, pulsada: true });
  assert.equal(p.valor, null);
});

test('menú: soltar una tecla no elige nada', () => {
  const p = crearMenu(JUEGOS).crear({});
  p.tick(500);
  p.tecla({ tipo: 'tecla', tecla: 1, pantalla: 1, pulsada: false });
  assert.equal(p.valor, null);
});

test('menú: nunca enseña más de nueve juegos', () => {
  const muchos = Array.from({ length: 12 }, (_, i) => ({ id: `j${i}`, nombre: `Juego ${i}` }));
  const { l, p } = correrMenu(muchos, []);
  assert.equal(MAX_JUEGOS, 9);
  assert.ok(tinta(l[8]) > 0);
  assert.ok(tinta(l[9]) > 0);      // sigue siendo SALIR, no el décimo juego
  assert.equal(p.valor, null);
});

// ---------------------------------------------------------------- la pantalla final

function correrFinal(datos, esperaMs, teclas = []) {
  const p = crearFinal({ juego: { nombre: 'Topos' }, textoPuntos: '12', textoMejor: '15', partidas: 4, record: false, ...datos }).crear({});
  const l = lienzo();
  const paso = (ms) => { p.tick(ms); limpiarLienzo(l); p.dibujar(l); };
  paso(esperaMs);
  for (const pantalla of teclas) {
    p.tecla({ tipo: 'tecla', tecla: pantalla, pantalla, pulsada: true });
    paso(200);
  }
  return { p, l, paso };
}

test('final: los datos arriba, el juego en el centro y los tres botones abajo', () => {
  const { l } = correrFinal({ juego: { nombre: 'Topos', icono: '#TOPOS' } }, 1000);
  for (const i of [0, 1, 2]) assert.ok(tinta(l[i]) > 0, `el dato de la pantalla ${i + 1}`);
  assert.ok(tinta(l[4]) > 0, 'el icono del juego');
  for (const i of [6, 7, 8]) assert.ok(tinta(l[i]) > 0, `el botón de la pantalla ${i + 1}`);
  for (const i of [3, 5, 9]) assert.equal(tinta(l[i]), 0, `la pantalla ${i + 1} debería estar vacía`);
});

test('final: las teclas 7, 8 y 9 son otra vez, menú y salir', () => {
  assert.deepEqual(correrFinal({}, 1000, [7]).p.valor, { accion: 'otra' });
  assert.deepEqual(correrFinal({}, 1000, [8]).p.valor, { accion: 'menu' });
  assert.deepEqual(correrFinal({}, 1000, [9]).p.valor, { accion: 'salir' });
});

test('final: el resto de teclas no eligen nada', () => {
  assert.equal(correrFinal({}, 1000, [1, 2, 3, 4, 5, 6, 10]).p.valor, null);
});

test('final: las pulsaciones del último segundo de la partida no cuentan', () => {
  const { p } = correrFinal({}, 300, [7]);
  assert.equal(p.valor, null, 'una tecla pulsada con prisa al acabar no puede dar a OTRA VEZ');
});

test('final: un récord nuevo parpadea y uno normal no', () => {
  const cambios = (record) => {
    const { l, paso } = correrFinal({ record }, 0);
    const vistos = new Set();
    for (let i = 0; i < 12; i++) { paso(100); vistos.add(tinta(l[2])); }
    return vistos.size;
  };
  assert.ok(cambios(true) > 1, 'el récord debería parpadear');
  assert.equal(cambios(false), 1, 'sin récord la pantalla no cambia');
});

// ---------------------------------------------------------------- piezas de dibujo y formato

test('formatearPuntos: tiempos en segundos y puntos enteros', () => {
  assert.equal(formatearPuntos('ms', 12345), '12.3 s');
  assert.equal(formatearPuntos('ms', 12345, { corto: true }), '12.3s');
  assert.equal(formatearPuntos('aciertos', 12.4), '12');
  assert.equal(formatearPuntos('aciertos', null), '—');
  assert.equal(formatearPuntos('aciertos', null, { corto: true }), '-');
});

test('etiquetaValor: valor grande arriba y rótulo pequeño debajo, sin pisarse', () => {
  const fb = new Uint8Array(FB);
  etiquetaValor(fb, 'PARTIDAS', 12);
  const solo = new Uint8Array(FB);
  etiquetaValor(solo, 'X', 12);
  assert.ok(tinta(fb) > tinta(solo), 'un rótulo más largo tiene que añadir tinta');
  // El valor vive en las tres primeras páginas (filas 0-23) y el rótulo en las siguientes.
  assert.ok(fb.slice(0, 72 * 3).some((b) => b), 'el valor grande debería estar arriba');
  assert.ok(fb.slice(72 * 3).some((b) => b), 'el rótulo debería estar debajo');
});

test('subir desplaza el contenido y descarta lo que sale por arriba', () => {
  const fb = new Uint8Array(FB);
  fb[0] = 0b0000_0001;            // píxel (0, 0)
  fb[72 * 1] = 0b0000_0010;       // píxel (0, 9)
  subir(fb, 5);
  assert.equal(fb[0] & 1, 0, 'el píxel de la fila 0 sale por arriba');
  assert.equal((fb[0] >> 4) & 1, 1, 'el de la fila 9 sube a la 4');
});
