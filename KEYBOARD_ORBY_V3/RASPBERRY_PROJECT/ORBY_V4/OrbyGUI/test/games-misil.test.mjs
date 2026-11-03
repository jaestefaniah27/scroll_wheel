import test from 'node:test';
import assert from 'node:assert/strict';
import misil, { OBJETIVOS, orientar, MUNDO_W, MUNDO_H, recorrido, ESTANCIA_MS, TRANSICION_MS } from '../src/games/misil.mjs';
import { W, H } from '../src/games/dibujo.mjs';
import { NODOS, TRAMOS, ESTATICOS, ZONAS, MAPA_W, MAPA_H, enCarretera, construirMapa } from '../src/games/misil-mapa.mjs';
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

// Los mandos de Misil van al revés (SENTIDO_X/Y = -1): de ahí el signo cambiado. La mira
// empieza en el centro del suelo y cada paso lento del mando la mueve 4 px.
const CENTRO = { x: Math.round(MUNDO_W / 2), y: Math.round(MUNDO_H / 2) };

function apuntar(partida, paso, destino) {
  const dx = Math.round((destino.x - CENTRO.x) / 4);
  const dy = Math.round((destino.y - CENTRO.y) / 4);
  for (let i = 0; i < Math.abs(dx); i++) { partida.mando({ mando: 1, delta: -Math.sign(dx) }); paso(50); }
  for (let i = 0; i < Math.abs(dy); i++) { partida.mando({ mando: 2, delta: -Math.sign(dy) }); paso(50); }
}

const centro = (e) => ({ x: Math.round(e.x + e.forma.w / 2), y: Math.round(e.y + e.forma.h / 2) });
// Lo mismo, en coordenadas de la vista (lo que mueve la mira).
const enVista = (partida, p) => ({ x: p.x - partida.camara.x, y: p.y - partida.camara.y });
const tamano = (nombre, dir = 'der') => orientar(OBJETIVOS.find((o) => o.nombre === nombre), dir);
// ¿Se ve entero en las pantallas con la cámara en `z`? (Ni en un hueco ni a medias.)
function visible(z, x, y, w, h) {
  const vx = x - z.x, vy = y - z.y;
  const col = Math.floor(vx / 88), fila = Math.floor(vy / 54);
  return col >= 0 && col < 3 && fila >= 0 && fila < 3
    && vx - col * 88 + w <= 72 && vy - fila * 54 + h <= 40 && vx >= col * 88 && vy >= fila * 54;
}

test('Misil: el suelo del mapa mide lo mismo que las nueve pantallas con sus huecos', () => {
  assert.equal(MUNDO_W, 248);
  assert.equal(MUNDO_H, 148);
  assert.equal(W, 72);
  assert.equal(H, 40);
});

test('Misil: cada objetivo tiene un dibujo rectangular y vale más cuanto más pequeño', () => {
  for (const o of OBJETIVOS) for (const f of o.filas) assert.equal(f.length, o.w, o.nombre);
  const porArea = [...OBJETIVOS].sort((a, b) => b.w * b.h - a.w * a.h);
  for (let i = 1; i < porArea.length; i++) assert.ok(porArea[i].valor >= porArea[i - 1].valor, porArea[i].nombre);
});

test('Misil: girar un objetivo lo pone de cara a donde va, sin perder píxeles', () => {
  const tanque = OBJETIVOS.find((o) => o.nombre === 'tanque');
  const tinta = (s) => s.filas.join('').split('#').length - 1;
  for (const dir of ['der', 'izq', 'arr', 'aba']) {
    const g = orientar(tanque, dir);
    assert.equal(g.filas.length, g.h, dir);
    for (const f of g.filas) assert.equal(f.length, g.w, dir);
    assert.equal(tinta(g), tinta(tanque), dir);
  }
  const abajo = orientar(tanque, 'aba');
  assert.equal(abajo.w, tanque.h);
  assert.equal(abajo.filas[abajo.h - 1][4], '#');
  assert.equal(orientar(tanque, 'arr').filas[0][4], '#');
  assert.equal(orientar(tanque, 'izq').filas[4][0], '#');
});

test('Misil: la cabina del camión está delante, a la derecha, como el cañón del tanque', () => {
  const camion = OBJETIVOS.find((o) => o.nombre === 'camion');
  // La cabina sobresale por arriba y por abajo en la parte de atrás de la caja… y la parte
  // delantera (derecha) es la más ancha de verdad: la trasera tiene los huecos.
  assert.equal(camion.filas[0].slice(-3), '...');
  assert.equal(camion.filas[2][camion.w - 1], '#');
  assert.equal(orientar(camion, 'aba').filas[camion.w - 1].includes('#'), true);
});

test('Misil: las carreteras son rectas y paralelas a los ejes, y lo que hay quieto no las pisa', () => {
  for (const [p, q] of TRAMOS) assert.ok(NODOS[p][0] === NODOS[q][0] || NODOS[p][1] === NODOS[q][1], p + q);
  for (const [nombre, x, y, dir] of ESTATICOS) {
    const f = tamano(nombre, dir);
    assert.ok(x >= 0 && y >= 0 && x + f.w <= MAPA_W && y + f.h <= MAPA_H, `${nombre} ${x},${y} dentro del mapa`);
    for (const [cx, cy] of [[x, y], [x + f.w - 1, y], [x, y + f.h - 1], [x + f.w - 1, y + f.h - 1]]) {
      assert.ok(!enCarretera(cx, cy), `${nombre} ${x},${y} pisa una carretera`);
    }
  }
});

test('Misil: cada zona cabe en el mapa y enseña bastantes objetivos enteros, sin amontonarlos', () => {
  for (const z of ZONAS) {
    assert.ok(z.x >= 0 && z.y >= 0 && z.x + MUNDO_W <= MAPA_W && z.y + MUNDO_H <= MAPA_H, z.nombre);
    const vistos = ESTATICOS.filter(([n, x, y, d]) => { const f = tamano(n, d); return visible(z, x, y, f.w, f.h); });
    assert.ok(vistos.length >= 8 && vistos.length <= 20, `${z.nombre}: ${vistos.length} objetivos a la vista`);
  }
});

test('Misil: lo que hay quieto no se solapa con otro objetivo', () => {
  const cajas = ESTATICOS.map(([n, x, y, d]) => { const f = tamano(n, d); return { n, x, y, w: f.w, h: f.h }; });
  for (let i = 0; i < cajas.length; i++) {
    for (let j = i + 1; j < cajas.length; j++) {
      const a = cajas[i], b = cajas[j];
      const solapa = a.x <= b.x + b.w && b.x <= a.x + a.w && a.y <= b.y + b.h && b.y <= a.y + a.h;
      assert.ok(!solapa, `${a.n} ${a.x},${a.y} y ${b.n} ${b.x},${b.y}`);
    }
  }
});

test('Misil: el mapa es siempre el mismo y no hay dibujo debajo de lo destruible', () => {
  const caja = { x: 120, y: 33, w: 11, h: 11 };
  const a = construirMapa([caja]);
  assert.equal(a.length, MAPA_W * MAPA_H);
  assert.deepEqual(a, construirMapa([caja]));
  for (let y = caja.y; y < caja.y + caja.h; y++) for (let x = caja.x; x < caja.x + caja.w; x++) assert.equal(a[y * MAPA_W + x], 0);
  // Y el dibujo no es ruido: casi todo el mapa es negro.
  const tinta = a.reduce((n, v) => n + v, 0);
  assert.ok(tinta / a.length < 0.08, `tinta ${tinta}`);
});

test('Misil: se pinta el mapa, con carreteras, en las nueve pantallas', () => {
  const { l } = nueva();
  for (let i = 0; i < 9; i++) assert.ok(l[i].some((b) => b), `pantalla ${i + 1} vacía`);
});

test('Misil: los vehículos circulan por la carretera y siempre de cara a donde avanzan', () => {
  const { partida, paso } = nueva(5);
  // La mira empieza junto a la puerta del puesto: se aparta a las dunas para no destruir lo que se observa.
  apuntar(partida, paso, { x: 60, y: 100 });
  const moviles = () => partida.entidades.filter((e) => e.desde);
  assert.equal(moviles().length, 5);
  const dirDe = { der: [1, 0], izq: [-1, 0], aba: [0, 1], arr: [0, -1] };
  let giros = 0;
  const previas = moviles().map((e) => ({ ...centro(e), dir: e.dir }));
  for (let t = 0; t < 60_000; t += 200) {
    paso(200);
    moviles().forEach((e, i) => {
      const c = centro(e);
      const ant = previas[i];
      // Dentro de la calzada, y el movimiento neto va en el sentido del dibujo (salvo un giro).
      assert.ok(enCarretera(c.x, c.y), `${e.tipo.nombre} fuera de la carretera en ${c.x},${c.y}`);
      const [dx, dy] = dirDe[e.dir];
      if (e.dir === ant.dir) {
        assert.ok((c.x - ant.x) * dx + (c.y - ant.y) * dy >= -1, `${e.tipo.nombre} va marcha atrás`);
      } else giros++;
      assert.equal(e.forma.filas.join(), orientar(e.tipo, e.dir).filas.join(), 'el dibujo mira hacia donde va');
      previas[i] = { ...c, dir: e.dir };
    });
  }
  assert.ok(giros > 0, 'en un minuto algún vehículo toma una curva');
});

test('Misil: un misil sobre un campamento destruye lo que hay dentro y suma puntos', () => {
  const { partida, paso } = nueva(3);
  const blanco = partida.entidades.find((e) => !e.desde && e.tipo.nombre === 'tienda');
  const antes = partida.entidades.length;
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(partida.puntos > 0, 'debería haber acertado');
  assert.ok(partida.entidades.length < antes);
  assert.ok(!partida.entidades.includes(blanco));
});

test('Misil: lo destruido vuelve a aparecer pasado un rato', () => {
  const { partida, paso } = nueva(3);
  const blanco = partida.entidades.find((e) => !e.desde && e.tipo.nombre === 'tienda');
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(!partida.entidades.includes(blanco));
  // Los misiles siguientes vuelven a caer en el mismo sitio: basta verla de vuelta un instante.
  let vuelve = false;
  for (let t = 0; t < 20_000 && !vuelve; t += 100) {
    paso(100);
    vuelve = partida.entidades.some((e) => e !== blanco && e.tipo === blanco.tipo && e.x === blanco.x && e.y === blanco.y);
  }
  assert.ok(vuelve);
});

test('Misil: no hace falta dar en el píxel, basta con que la explosión alcance el objetivo', () => {
  const { partida, paso } = nueva(4);
  const blanco = partida.entidades.find((e) => !e.desde && e.tipo.nombre === 'tienda');
  apuntar(partida, paso, enVista(partida, { x: blanco.x + blanco.forma.w - 1 + 4, y: blanco.y + 5 }));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(partida.puntos > 0);
});

test('Misil: sin tocar nada, tres misiles fallan y la partida acaba con cero', () => {
  // La mira empieza junto a la puerta del puesto, por donde pasan vehículos: se aparta antes a
  // un descampado.
  const { partida, paso } = nueva();
  apuntar(partida, paso, { x: 150, y: 95 });
  paso(60_000);
  assert.ok(partida.terminado);
  assert.equal(partida.puntos, 0);
});

test('Misil: pulsar los dos mandos hace caer el misil antes que pulsar uno', () => {
  const tiempoHastaFallo = (botones) => {
    const { partida, paso } = nueva();
    for (const b of botones) partida.mandoBoton({ mando: b, pulsado: true });
    let t = 0;
    while (partida.estado.startsWith('0 objetivos · 3') && t < 20_000) { paso(); t += DT; }
    return t;
  };
  const nada = tiempoHastaFallo([]);
  const uno = tiempoHastaFallo([1]);
  const dos = tiempoHastaFallo([1, 2]);
  assert.ok(uno < nada && dos < uno, `${nada} / ${uno} / ${dos}`);
});

// --- Fase 3: el VTOL va de zona en zona -------------------------------------------------

test('Misil: el VTOL se queda en cada zona y luego vuela a la siguiente, en bucle', () => {
  const ciclo = ESTANCIA_MS + TRANSICION_MS;
  for (let n = 0; n < ZONAS.length + 1; n++) {
    const z = ZONAS[n % ZONAS.length];
    for (const t of [0, ESTANCIA_MS / 2, ESTANCIA_MS - 1]) {
      const r = recorrido(n * ciclo + t);
      assert.deepEqual([r.x, r.y, r.enVuelo], [z.x, z.y, false], `zona ${n}, ${t} ms`);
    }
    assert.ok(recorrido(n * ciclo + ESTANCIA_MS + 1).enVuelo);
  }
  assert.ok(TRANSICION_MS >= 3000 && TRANSICION_MS <= 4000, 'el vuelo dura tres o cuatro segundos');
});

test('Misil: el vuelo es suave: sin saltos, arrancando y frenando despacio', () => {
  for (let n = 0; n < ZONAS.length; n++) {
    const t0 = n * (ESTANCIA_MS + TRANSICION_MS) + ESTANCIA_MS;
    const pasos = [];
    let prev = recorrido(t0 - 8);
    for (let t = t0; t <= t0 + TRANSICION_MS + 8; t += 8) {
      const r = recorrido(t);
      pasos.push(Math.hypot(r.x - prev.x, r.y - prev.y));
      prev = r;
    }
    // A 8 ms por fotograma, nunca más de unos pocos píxeles de golpe.
    assert.ok(Math.max(...pasos) <= 4, `zona ${n}: salto de ${Math.max(...pasos)} px`);
    const tercio = Math.floor(pasos.length / 3);
    const suma = (l) => l.reduce((a, b) => a + b, 0);
    const medio = suma(pasos.slice(tercio, 2 * tercio));
    assert.ok(suma(pasos.slice(0, tercio)) < medio / 1.5 && suma(pasos.slice(2 * tercio)) < medio / 1.5, `zona ${n}: arranca y frena`);
    // Y el camino se queda dentro del mapa.
    for (let t = t0; t <= t0 + TRANSICION_MS; t += 100) {
      const r = recorrido(t);
      assert.ok(r.x >= 0 && r.y >= 0 && r.x + MUNDO_W <= MAPA_W && r.y + MUNDO_H <= MAPA_H);
    }
  }
});

// Una partida que acierta una tienda con el primer misil: así le quedan tres fallos de margen y
// dura hasta pasado el primer vuelo (cada misil fallado son unos 7,6 s).
function partidaQueLlegaAlPuente() {
  const { partida, paso } = nueva(3);
  const tienda = partida.entidades.find((e) => !e.desde && e.tipo.nombre === 'tienda');
  const mira = enVista(partida, centro(tienda));
  apuntar(partida, paso, mira);
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(partida.puntos > 0, 'el primer misil acierta la tienda');
  return { partida, paso, mira };
}

test('Misil: en partida la cámara sigue el recorrido del VTOL', () => {
  const { partida, paso } = partidaQueLlegaAlPuente();
  assert.equal(partida.zona, 'puesto');
  assert.deepEqual(partida.camara, { x: ZONAS[0].x, y: ZONAS[0].y });
  while (partida.zona === 'puesto') paso(100);
  assert.equal(partida.zona, null, 'a medio vuelo no está en ninguna zona');
  paso(TRANSICION_MS / 2);
  const c = partida.camara;
  assert.ok(c.x !== ZONAS[0].x || c.y !== ZONAS[0].y);
  paso(TRANSICION_MS / 2 + 100);
  assert.ok(!partida.terminado);
  assert.equal(partida.zona, 'puente');
  assert.deepEqual(partida.camara, { x: ZONAS[1].x, y: ZONAS[1].y });
});

test('Misil: después del vuelo se puede acertar en la zona nueva', () => {
  const { partida, paso, mira } = partidaQueLlegaAlPuente();
  while (partida.zona !== 'puente') paso(100);
  const antes = partida.puntos;
  const z = ZONAS[1];
  const blanco = partida.entidades.find((e) => !e.desde && e.tipo.nombre === 'camion'
    && e.x > z.x && e.y > z.y && e.x < z.x + MUNDO_W && e.y < z.y + MUNDO_H);
  assert.ok(blanco, 'en el puente hay camiones aparcados');
  // La mira va con el aparato: sigue en el mismo punto de la vista, y desde ahí se lleva al camión.
  const v = enVista(partida, centro(blanco));
  const dx = Math.round((v.x - mira.x) / 4), dy = Math.round((v.y - mira.y) / 4);
  for (let i = 0; i < Math.abs(dx); i++) { partida.mando({ mando: 1, delta: -Math.sign(dx) }); paso(50); }
  for (let i = 0; i < Math.abs(dy); i++) { partida.mando({ mando: 2, delta: -Math.sign(dy) }); paso(50); }
  for (let t = 0; t < 9000 && partida.puntos === antes; t += DT) paso();
  assert.ok(partida.puntos > antes, 'el misil acierta el camión del puente');
});
