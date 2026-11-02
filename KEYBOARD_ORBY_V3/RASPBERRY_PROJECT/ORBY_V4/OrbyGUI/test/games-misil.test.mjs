import test from 'node:test';
import assert from 'node:assert/strict';
import misil, { OBJETIVOS } from '../src/games/misil.mjs';
import { lienzo, limpiarLienzo } from '../src/games/proto.mjs';

const DT = 8;

function nueva(semilla = 3) {
  let s = semilla >>> 0;
  const aleatorio = (tope) => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return Math.floor((s / 0x100000000) * tope);
  };
  let reloj = 0;
  const partida = misil.crear({ ahora: () => reloj, aleatorio });
  const l = lienzo();
  const paso = (ms = DT) => { for (let t = 0; t < ms; t += DT) { reloj += DT; partida.tick(DT); limpiarLienzo(l); partida.dibujar(l); } };
  paso();
  return { partida, l, paso };
}

// La mira empieza en el centro del suelo (3 pantallas de 72 + 2 huecos de 16, por 3 de 40
// + 2 de 14) y cada paso lento del mando la mueve 4 px. Llevarla a un punto es contar pasos.
const CENTRO = { x: Math.round((3 * 72 + 2 * 16) / 2), y: Math.round((3 * 40 + 2 * 14) / 2) };

// El objetivo se encuentra mirando qué pantalla tiene tinta fuera de la cruz: no se expone
// el estado, así que el test lo apunta igual que una persona, por lo que ve.
function apuntar(partida, paso, destino) {
  const dx = Math.round((destino.x - CENTRO.x) / 4);
  const dy = Math.round((destino.y - CENTRO.y) / 4);
  for (let i = 0; i < Math.abs(dx); i++) { partida.mando({ mando: 1, delta: Math.sign(dx) }); paso(50); }
  for (let i = 0; i < Math.abs(dy); i++) { partida.mando({ mando: 2, delta: Math.sign(dy) }); paso(50); }
}

// Repite el sorteo del primer objetivo con la misma semilla: el estado no se expone, y
// buscarlo entre los píxeles de la cruz y el círculo sería más frágil que esto.
function sorteo(semilla) {
  let s = semilla >>> 0;
  const aleatorio = (tope) => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return Math.floor((s / 0x100000000) * tope); };
  const tipo = OBJETIVOS[aleatorio(OBJETIVOS.length)];
  for (let intento = 0; ; intento++) {
    const p = aleatorio(9);
    const x = (p % 3) * 88 + 3 + aleatorio(72 - tipo.w - 6);
    const y = Math.floor(p / 3) * 54 + 3 + aleatorio(40 - tipo.h - 6);
    if (Math.hypot(x + tipo.w / 2 - CENTRO.x, y + tipo.h / 2 - CENTRO.y) >= 60 || intento > 20) return { tipo, x, y };
  }
}

test('Misil: cada objetivo tiene un dibujo rectangular y vale más cuanto más pequeño', () => {
  for (const o of OBJETIVOS) for (const f of o.filas) assert.equal(f.length, o.w, o.nombre);
  const porArea = [...OBJETIVOS].sort((a, b) => b.w * b.h - a.w * a.h);
  for (let i = 1; i < porArea.length; i++) assert.ok(porArea[i].valor > porArea[i - 1].valor, porArea[i].nombre);
});

test('Misil: no hace falta dar en el píxel, basta con que la explosión alcance el objetivo', () => {
  // La mira se queda unos 4 px a la derecha del borde del objetivo (los pasos del mando son de
  // 4 y redondean hasta 2 más): fuera del dibujo, dentro del radio.
  for (const semilla of [3, 4, 5]) {
    const { partida, paso } = nueva(semilla);
    const { tipo, x, y } = sorteo(semilla);
    apuntar(partida, paso, { x: x + tipo.w - 1 + 4, y: y + Math.floor(tipo.h / 2) });
    for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
    assert.ok(partida.puntos > 0, `semilla ${semilla}: la explosión debería alcanzar al ${tipo.nombre}`);
  }
});

test('Misil: sin tocar nada, tres misiles fallan y la partida acaba con cero', () => {
  const { partida, paso } = nueva();
  paso(60_000);
  assert.ok(partida.terminado);
  assert.equal(partida.puntos, 0);
});

test('Misil: pulsar los dos mandos hace caer el misil antes que pulsar uno', () => {
  const tiempoHastaFallo = (botones) => {
    const { partida, paso } = nueva();
    for (const b of botones) partida.mandoBoton({ mando: b, pulsado: true });
    let t = 0;
    while (!/2 misiles/.test(partida.estado) && t < 20_000) { paso(); t += DT; }
    return t;
  };
  const nada = tiempoHastaFallo([]);
  const uno = tiempoHastaFallo([1]);
  const dos = tiempoHastaFallo([1, 2]);
  assert.ok(uno < nada && dos < uno, `${nada} / ${uno} / ${dos}`);
});

test('Misil: con la mira encima del objetivo, el misil suma puntos', () => {
  // Se prueba con varias semillas: el objetivo sale cada vez en un sitio y con otra forma.
  for (const semilla of [3, 4, 5]) {
    const { partida, paso, l } = nueva(semilla);
    const { tipo, x, y } = sorteo(semilla);
    const destino = { x: x + Math.floor(tipo.w / 2), y: y + Math.floor(tipo.h / 2) };
    apuntar(partida, paso, destino);
    for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
    assert.ok(partida.puntos > 0, `semilla ${semilla}: debería haber acertado al ${tipo.nombre}`);
    assert.ok(l[9].some((b) => b), 'el marcador se dibuja');
  }
});
