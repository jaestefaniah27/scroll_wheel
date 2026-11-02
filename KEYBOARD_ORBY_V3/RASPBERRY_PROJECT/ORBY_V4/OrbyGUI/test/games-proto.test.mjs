import test from 'node:test';
import assert from 'node:assert/strict';
import {
  PANTALLAS, FB, lienzo, armarFotograma, pantallaDeTecla, teclaDePantalla,
  leerEvento, puedeEnviar, EN_VUELO, msEntre,
} from '../src/games/proto.mjs';

const texto = (bytes, hasta) => new TextDecoder().decode(bytes.slice(0, hasta));

test('el primer fotograma lleva las diez pantallas', () => {
  const l = lienzo();
  const f = armarFotograma(null, l, 0);
  assert.equal(f.mascara, 0x3ff);
  assert.equal(f.pantallas, 10);
  const cab = 'GAME_BIN:1023:0\n';
  assert.equal(texto(f.bytes, cab.length), cab);
  assert.equal(f.bytes.length, cab.length + 10 * FB);
});

test('solo viajan las pantallas que han cambiado, en orden', () => {
  const antes = lienzo();
  const ahora = lienzo();
  ahora[2].fill(0xaa);   // pantalla 3
  ahora[7].fill(0x55);   // pantalla 8
  const f = armarFotograma(antes, ahora, 41);
  assert.equal(f.mascara, (1 << 2) | (1 << 7));
  const cab = `GAME_BIN:${f.mascara}:41\n`;
  assert.equal(texto(f.bytes, cab.length), cab);
  assert.equal(f.bytes.length, cab.length + 2 * FB);
  // La carga va en orden creciente de pantalla.
  assert.equal(f.bytes[cab.length], 0xaa);
  assert.equal(f.bytes[cab.length + FB], 0x55);
});

test('un fotograma idéntico no se manda', () => {
  const l = lienzo();
  l[0][5] = 1;
  const copia = lienzo();
  copia[0][5] = 1;
  assert.equal(armarFotograma(l, copia, 3), null);
});

test('un solo byte distinto basta para mandar su pantalla', () => {
  const antes = lienzo();
  const ahora = lienzo();
  ahora[9][FB - 1] = 1;
  const f = armarFotograma(antes, ahora, 0);
  assert.equal(f.mascara, 1 << 9);
});

test('la tecla 11 es la pantalla 10 y la 10 y la 12 no tienen', () => {
  assert.equal(pantallaDeTecla(1), 1);
  assert.equal(pantallaDeTecla(9), 9);
  assert.equal(pantallaDeTecla(10), null);
  assert.equal(pantallaDeTecla(11), 10);
  assert.equal(pantallaDeTecla(12), null);
  for (let p = 1; p <= PANTALLAS; p++) assert.equal(pantallaDeTecla(teclaDePantalla(p)), p);
});

test('lee las líneas de entrada del teclado', () => {
  assert.deepEqual(leerEvento('KEY_EV:11:1'), { tipo: 'tecla', tecla: 11, pantalla: 10, pulsada: true, us: null });
  assert.deepEqual(leerEvento('KEY_EV:3:0'), { tipo: 'tecla', tecla: 3, pantalla: 3, pulsada: false, us: null });
  assert.deepEqual(leerEvento('ENC:2:-1'), { tipo: 'mando', mando: 2, delta: -1 });
  assert.deepEqual(leerEvento('ENC_SW:1:1'), { tipo: 'mando-boton', mando: 1, pulsado: true });
  assert.deepEqual(leerEvento('WHEEL:5:2100'), { tipo: 'rueda', delta: 5, angulo: 2100 });
  assert.deepEqual(leerEvento('GAME:SHOWN:17'), { tipo: 'acuse', n: 17, us: null });
});

test('en juego, teclas y acuses traen el instante del reloj del teclado', () => {
  assert.equal(leerEvento('KEY_EV:5:1:123456').us, 123456);
  assert.deepEqual(leerEvento('GAME:SHOWN:17:4000000000'), { tipo: 'acuse', n: 17, us: 4000000000 });
});

test('msEntre resta bien aunque el contador de µs haya dado la vuelta', () => {
  assert.equal(msEntre(1_000_000, 1_250_500), 250.5);
  assert.equal(msEntre(0xffff_ff00, 0x0003_0d40 - 0x100), 200);
});

test('el teclado puede sacar de la partida por dos caminos', () => {
  assert.equal(leerEvento('GAME:TIMEOUT').tipo, 'fin');
  assert.equal(leerEvento('MODE:MENU').tipo, 'fin');
  assert.equal(leerEvento('MODE:NORMAL').tipo, 'fin');
});

test('una línea de rueda pegada a otra se descarta en vez de inventar el ángulo', () => {
  assert.equal(leerEvento('WHEEL:5:WHEEL:-3:2100'), null);
  assert.equal(leerEvento('WHEEL:x:y'), null);
});

test('lo que no es de juego no lo es', () => {
  assert.equal(leerEvento('PROFILE:OK:2'), null);
  assert.equal(leerEvento('EV:CTX:1:0:1'), null);
});

test('el control de flujo deja EN_VUELO fotogramas sin acuse', () => {
  // Se manda el 0, 1 y 2 sin ningún acuse; el 3 tiene que esperar.
  assert.equal(puedeEnviar(-1, -1), true);
  assert.equal(puedeEnviar(0, -1), true);
  assert.equal(puedeEnviar(1, -1), true);
  assert.equal(puedeEnviar(EN_VUELO - 1, -1), false);
  assert.equal(puedeEnviar(EN_VUELO - 1, 0), true);
});
