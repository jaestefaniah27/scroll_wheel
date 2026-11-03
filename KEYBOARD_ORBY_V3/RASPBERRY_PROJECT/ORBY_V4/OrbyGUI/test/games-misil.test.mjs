import test from 'node:test';
import assert from 'node:assert/strict';
import misil, { OBJETIVOS, orientar, elegirObjetivo, MOVILES_DESDE } from '../src/games/misil.mjs';
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

// Los mandos de Misil van al revés (SENTIDO_X/Y = -1): de ahí el signo cambiado.
// La mira empieza en el centro del suelo (3 pantallas de 72 + 2 huecos de 16, por 3 de 40
// + 2 de 14) y cada paso lento del mando la mueve 4 px. Llevarla a un punto es contar pasos.
const CENTRO = { x: Math.round((3 * 72 + 2 * 16) / 2), y: Math.round((3 * 40 + 2 * 14) / 2) };

// El objetivo se encuentra mirando qué pantalla tiene tinta fuera de la cruz: no se expone
// el estado, así que el test lo apunta igual que una persona, por lo que ve.
function apuntar(partida, paso, destino) {
  const dx = Math.round((destino.x - CENTRO.x) / 4);
  const dy = Math.round((destino.y - CENTRO.y) / 4);
  for (let i = 0; i < Math.abs(dx); i++) { partida.mando({ mando: 1, delta: -Math.sign(dx) }); paso(50); }
  for (let i = 0; i < Math.abs(dy); i++) { partida.mando({ mando: 2, delta: -Math.sign(dy) }); paso(50); }
}

// Repite el sorteo del primer objetivo con la misma semilla: el estado no se expone, y
// buscarlo entre los píxeles de la cruz y el círculo sería más frágil que esto.
function sorteo(semilla) {
  let s = semilla >>> 0;
  const aleatorio = (tope) => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return Math.floor((s / 0x100000000) * tope); };
  const tipo = OBJETIVOS[aleatorio(OBJETIVOS.length)];
  for (let intento = 0; ; intento++) {
    const p = aleatorio(9);
    const x = (p % 3) * 88 + 8 + aleatorio(72 - tipo.w - 16);
    const y = Math.floor(p / 3) * 54 + 8 + aleatorio(40 - tipo.h - 16);
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

// --- Fase 2: objetivos que se mueven -------------------------------------------------

function dado(semilla) {
  let s = semilla >>> 0;
  return (tope) => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return Math.floor((s / 0x100000000) * tope); };
}

test('Misil: girar un objetivo lo pone de cara a donde va, sin perder píxeles', () => {
  const tanque = OBJETIVOS.find((o) => o.nombre === 'tanque');
  const tinta = (s) => s.filas.join('').split('#').length - 1;
  for (const dir of ['der', 'izq', 'arr', 'aba']) {
    const g = orientar(tanque, dir);
    assert.equal(g.filas.length, g.h, dir);
    for (const f of g.filas) assert.equal(f.length, g.w, dir);
    assert.equal(tinta(g), tinta(tanque), dir);
  }
  // El cañón sobresale por la derecha en el original (fila 4, hasta la última columna).
  // Girado hacia abajo sobresale por abajo; hacia arriba, por arriba; hacia la izquierda, por la izquierda.
  const abajo = orientar(tanque, 'aba');
  const arriba = orientar(tanque, 'arr');
  assert.equal(abajo.w, tanque.h);
  assert.equal(abajo.filas[abajo.h - 1][4], '#');
  assert.equal(arriba.filas[0][4], '#');
  assert.equal(orientar(tanque, 'izq').filas[4][0], '#');
});

test('Misil: hasta unos aciertos nada se mueve; después, cada vez más y más deprisa', () => {
  const aleatorio = dado(17);
  const tirar = (aciertos, n = 400) => Array.from({ length: n }, () => elegirObjetivo(aciertos, aleatorio));
  for (let a = 0; a < MOVILES_DESDE; a++) assert.ok(tirar(a).every((o) => o.vel === 0), `con ${a} aciertos`);

  const moviles = (lista) => lista.filter((o) => o.vel > 0);
  const pocos = moviles(tirar(MOVILES_DESDE));
  const muchos = moviles(tirar(20));
  assert.ok(pocos.length > 0, 'en cuanto empieza la fase 2 ya sale alguno');
  assert.ok(muchos.length > pocos.length, 'con más aciertos salen más');
  const media = (l) => l.reduce((s, o) => s + o.vel / o.tipo.velocidad, 0) / l.length;
  assert.ok(media(muchos) > media(pocos), 'y van más deprisa');
  // La tienda nunca se mueve, y siempre queda algún objetivo quieto.
  assert.ok(tirar(50).filter((o) => o.tipo.nombre === 'tienda').every((o) => o.vel === 0));
  assert.ok(tirar(50).some((o) => o.vel === 0));
});

test('Misil: el dibujo del objetivo que se mueve mira hacia donde va', () => {
  const aleatorio = dado(23);
  for (let i = 0; i < 200; i++) {
    const o = elegirObjetivo(20, aleatorio);
    assert.deepEqual(o.forma.filas, orientar(o.tipo, o.dir).filas);
    if (o.vel === 0) assert.equal(o.dir, 'der');
  }
});

test('Misil: un objetivo fallado sigue vivo, en el mismo sitio, para el siguiente misil', () => {
  const semilla = 4;
  const { partida, paso, l } = nueva(semilla);
  const { tipo, x, y } = sorteo(semilla);
  // Si está entero el dibujo del objetivo en su sitio: todos sus píxeles encendidos. Contar
  // tinta en la caja no valía, porque la cruz de la mira puede pasar por ella.
  const entero = () => {
    let n = 0;
    for (let j = 0; j < tipo.h; j++) {
      for (let i = 0; i < tipo.w; i++) {
        if (tipo.filas[j][i] !== '#') continue;
        const gx = x + i, gy = y + j;
        const col = Math.floor(gx / 88), fila = Math.floor(gy / 54);
        const ox = gx - col * 88, oy = gy - fila * 54;
        const fb = l[fila * 3 + col];
        if (!((fb[(oy >> 3) * 72 + ox] >> (oy & 7)) & 1)) return false;
      }
    }
    return true;
  };
  assert.ok(entero(), 'al empezar, el objetivo está donde dice el sorteo');
  // Sin tocar nada, el primer misil falla.
  for (let t = 0; t < 10_000 && !/2 misiles/.test(partida.estado); t += DT) paso();
  paso(2000);   // pasa la explosión y ya cae el segundo misil
  assert.match(partida.estado, /^0 objetivos · 2 misiles/);
  assert.ok(entero(), `el ${tipo.nombre} fallado debería seguir dibujado donde estaba`);
});
