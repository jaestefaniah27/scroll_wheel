import test from 'node:test';
import assert from 'node:assert/strict';
import misil, { OBJETIVOS, orientar, MUNDO_W, MUNDO_H, trayecto, hayObjetivosEn, avanzarVehiculo, espacioDelante, circular, armarConvoy, duracionVuelo, ESTANCIA_MS, BONO_VIDA, TRANSICION_MS, NIVELES, ESPACIO_DISPAROS_MS, LANZADOR_DESPLIEGUE_MS, DESEMBARCO_MS, DERROTA_MS, crearExplosion } from '../src/games/misil.mjs';
import { W, H } from '../src/games/dibujo.mjs';
import { NODOS, TRAMOS, SALIDAS, ESTATICOS, ZONAS, SENDAS, APARCAMIENTOS, MAPA_W, MAPA_H, enCarretera, construirMapa } from '../src/games/misil-mapa.mjs';
import { lienzo, limpiarLienzo } from '../src/games/proto.mjs';

const DT = 8;

function nueva(semilla = 3, opciones = {}) {
  let s = semilla >>> 0;
  const aleatorio = (tope) => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return Math.floor((s / 0x100000000) * tope);
  };
  let reloj = 0;
  // Sin vaivén por defecto: las pruebas que apuntan suponen una cámara quieta.
  const partida = misil.crear({ ahora: () => reloj, aleatorio, quieto: true, ...opciones });
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

// Una tienda de las que se ven desde la zona en la que empieza la partida.
const tiendaALaVista = (partida) => partida.entidades.find((e) => !e.desde && e.tipo.nombre === 'tienda' && hayObjetivosEn([e], ZONAS[0]));
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

test('Air Strike: el juego conserva el id de las puntuaciones y se llama Air Strike', () => {
  assert.equal(misil.id, 'misil');
  assert.equal(misil.nombre, 'Air Strike');
  assert.ok(!misil.descripcion.includes('Misil'));
});

test('Misil: el suelo del mapa mide lo mismo que las nueve pantallas con sus huecos', () => {
  assert.equal(MUNDO_W, 248);
  assert.equal(MUNDO_H, 148);
  assert.equal(W, 72);
  assert.equal(H, 40);
});

test('Misil: cada objetivo tiene un dibujo rectangular y vale más cuanto más pequeño', () => {
  for (const o of OBJETIVOS) for (const f of o.filas) assert.equal(f.length, o.w, o.nombre);
  // El helicóptero y el lanzador valen lo que valen por decisión, no por tamaño.
  const porArea = OBJETIVOS.filter((o) => !o.valorFijo && o.nombre !== 'lanzador').sort((a, b) => b.w * b.h - a.w * a.h);
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

test('Misil: la red es de sentido único, sin callejones: de cada nodo se sale, a cada nodo se llega y desde cualquiera se va a todos', () => {
  const nodos = Object.keys(NODOS);
  for (const nodo of nodos) {
    assert.ok(SALIDAS[nodo]?.length >= 1, `el nodo ${nodo} es un callejón sin salida`);
    assert.ok(TRAMOS.some((t) => t[1] === nodo), `al nodo ${nodo} no llega nadie`);
  }
  for (const [p, q] of TRAMOS) assert.ok(!TRAMOS.includes(q + p), `${p}${q} y ${q}${p}: dos sentidos en la misma carretera`);
  for (const origen of nodos) {
    const vistos = new Set([origen]);
    for (const n of vistos) for (const m of SALIDAS[n]) vistos.add(m);
    assert.equal(vistos.size, nodos.length, `desde ${origen} no se llega a todos`);
  }
  // Y se empieza en la base, con el puesto y el puente después.
  assert.deepEqual(ZONAS.map((z) => z.nombre), ['base', 'puesto', 'puente', 'oasis', 'aerodromo', 'cuartel']);
});

test('Misil: ningún vehículo se da la vuelta en seco, salga de donde salga y vaya adonde vaya', () => {
  const opuesto = { der: 'izq', izq: 'der', arr: 'aba', aba: 'arr' };
  const camion = OBJETIVOS.find((o) => o.nombre === 'camion');
  let s = 7;
  const azar = (tope) => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return Math.floor((s / 0x100000000) * tope); };
  let giros = 0;
  for (const [p, q] of TRAMOS) {
    for (const [desde, hasta] of [[p, q]]) {
      for (let intento = 0; intento < 4; intento++) {
        const v = { tipo: camion, vel: 0, desde, hasta, dist: 0 };
        avanzarVehiculo(v, 0, azar);
        let dir = v.dir;
        // 30.000 px a trozos de un píxel: pasa por todos los cruces muchas veces.
        for (let px = 0; px < 30_000; px++) {
          avanzarVehiculo(v, 1, azar);
          assert.notEqual(v.dir, opuesto[dir], `se da la vuelta en seco en ${v.desde}→${v.hasta}`);
          if (v.dir !== dir) giros++;
          dir = v.dir;
        }
      }
    }
  }
  assert.ok(giros > 100, 'sí que giran, solo que en las esquinas');
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
  const { partida, paso } = nueva(5, { inmortal: true });
  // La mira empieza en el centro de la base: se aparta a un claro para no destruir lo que se observa.
  apuntar(partida, paso, { x: 150, y: 95 });
  const moviles = () => partida.entidades.filter((e) => e.desde);
  assert.ok(moviles().length >= 2, 'la partida empieza con tráfico');
  const dirDe = { der: [1, 0], izq: [-1, 0], aba: [0, 1], arr: [0, -1] };
  let giros = 0;
  const previas = new Map();
  for (let t = 0; t < 60_000; t += 200) {
    paso(200);
    for (const e of moviles()) {
      const c = centro(e);
      const ant = previas.get(e);
      // Dentro de la calzada, y el movimiento neto va en el sentido del dibujo (salvo un giro).
      assert.ok(enCarretera(c.x, c.y), `${e.tipo.nombre} fuera de la carretera en ${c.x},${c.y}`);
      const [dx, dy] = dirDe[e.dir];
      if (ant && e.dir === ant.dir) {
        assert.ok((c.x - ant.x) * dx + (c.y - ant.y) * dy >= -1, `${e.tipo.nombre} va marcha atrás`);
      } else if (ant) giros++;
      assert.equal(e.forma.filas.join(), orientar(e.tipo, e.dir).filas.join(), 'el dibujo mira hacia donde va');
      previas.set(e, { ...c, dir: e.dir });
    }
  }
  assert.ok(giros > 0, 'en un minuto algún vehículo toma una curva');
});

// Dos cajas se solapan si comparten algún píxel (rozarse no cuenta).
const solapan = (a, b) => a.x < b.x + b.forma.w && b.x < a.x + a.forma.w && a.y < b.y + b.forma.h && b.y < a.y + a.forma.h;
const tipoDe = (n) => OBJETIVOS.find((o) => o.nombre === n);
const azarDe = (semilla) => {
  let s = semilla;
  return (tope) => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return Math.floor((s / 0x100000000) * tope); };
};

test('Misil: nueve vehículos con convoyes dan vueltas 2000 ticks sin pisarse nunca y sin atascarse', () => {
  const azar = azarDe(11);
  const vehiculos = [];
  // Tres convoyes de tres, en tramos distintos y con los tipos mezclados (el tanque tiene que frenar al camión).
  for (const [tramo, tipos] of [['af', ['camion', 'tanque', 'camion']], ['gq', ['tanque', 'camion', 'patrulla']], ['yd', ['camion', 'camion', 'tanque']]]) {
    vehiculos.push(...armarConvoy(tipos.map(tipoDe), tramo, 0.5, azar));
  }
  assert.equal(vehiculos.length, 9);
  for (const a of vehiculos) for (const b of vehiculos) if (a !== b) assert.ok(!solapan(a, b), 'ya salen pisándose');
  const recorrido = new Map(vehiculos.map((v) => [v, 0]));
  for (let t = 0; t < 2000; t++) {
    const antes = vehiculos.map((v) => [v.x, v.y]);
    circular(vehiculos, 50, azar);
    vehiculos.forEach((v, i) => recorrido.set(v, recorrido.get(v) + Math.hypot(v.x - antes[i][0], v.y - antes[i][1])));
    for (const a of vehiculos) for (const b of vehiculos) if (a !== b) assert.ok(!solapan(a, b), `tick ${t}: ${a.tipo.nombre} y ${b.tipo.nombre} se pisan`);
  }
  // 100 s de reloj: el tanque, el más lento, va a 12 px/s. Ninguno se queda parado para siempre.
  for (const [v, d] of recorrido) assert.ok(d > 300, `${v.tipo.nombre} solo ha recorrido ${Math.round(d)} px`);
});

test('Misil: espacioDelante frena al de atrás en su tramo y también pasado el cruce', () => {
  const camion = tipoDe('camion');
  const v = (desde, hasta, luego, dist) => { const x = { tipo: camion, vel: 0, desde, hasta, luego, dist }; avanzarVehiculo(x, 0, () => 0); return x; };
  // 'ba' mide 124. Dos camiones de 14 de largo: el hueco es 14 + 4 = 18.
  const atras = v('b', 'a', 'f', 10);
  const delante = v('b', 'a', 'f', 40);
  assert.equal(espacioDelante(atras, [atras, delante]), 30 - 18);
  assert.equal(espacioDelante(delante, [atras, delante]), Infinity);
  // El de delante ya está en el tramo siguiente: se cuenta por la carretera, no por tramo. Y como ahí hay una curva,
  // las cajas pueden rozarse en la esquina: el hueco suma también los anchos.
  const casi = v('b', 'a', 'f', 120);
  const lejos = v('a', 'f', 'h', 30);
  assert.equal(espacioDelante(casi, [casi, lejos]), 4 + 30 - 20);
  const pegado = v('a', 'f', 'h', 5);
  assert.equal(espacioDelante(casi, [casi, pegado]), 0);
});

test('Misil: un convoy sigue el camino de su cabeza y va a la velocidad del más lento', () => {
  const azar = azarDe(5);
  const convoy = armarConvoy([tipoDe('camion'), tipoDe('tanque'), tipoDe('camion')], 'gq', 0.5, azar);
  assert.equal(convoy.length, 3);
  const rutas = convoy.map(() => []);
  for (let t = 0; t < 4000; t++) {
    circular(convoy, 50, azar);
    convoy.forEach((v, i) => { if (rutas[i].at(-1) !== v.hasta) rutas[i].push(v.hasta); });
    assert.ok(convoy.every((v) => v.vel === 12), 'todos a la velocidad del tanque');
  }
  assert.ok(rutas[0].length > 3);
  assert.deepEqual(rutas[1], rutas[0].slice(0, rutas[1].length));
  assert.deepEqual(rutas[2], rutas[0].slice(0, rutas[2].length));
});

test('Misil: en un minuto con la cámara en la primera zona casi siempre hay vehículos a la vista', () => {
  const { partida, paso } = nueva(7, { inmortal: true });
  apuntar(partida, paso, { x: 150, y: 95 });
  let muestras = 0, con = 0;
  for (let t = 0; t < 60_000 && !partida.volando; t += 250) {
    paso(250);
    if (partida.volando) break;
    muestras++;
    if (partida.entidades.some((e) => e.desde && hayObjetivosEn([e], ZONAS[0]))) con++;
    assert.ok(partida.entidades.filter((e) => e.desde).length <= 12, 'no se acumulan vehículos');
  }
  assert.ok(muestras > 100, `solo ${muestras} muestras`);
  assert.ok(con / muestras >= 0.9, `solo hay vehículos a la vista el ${Math.round((100 * con) / muestras)} % del tiempo`);
});

test('Misil: la patrulla corre más que nadie, lleva luces que parpadean y se gira entera', () => {
  const patrulla = tipoDe('patrulla');
  assert.ok(patrulla.velocidad > Math.max(...OBJETIVOS.filter((o) => o.nombre !== 'patrulla').map((o) => o.velocidad)));
  assert.equal(patrulla.filas2.length, patrulla.h);
  for (const f of patrulla.filas2) assert.equal(f.length, patrulla.w);
  assert.notDeepEqual(patrulla.filas, patrulla.filas2);
  for (const dir of ['izq', 'arr', 'aba']) {
    const g = orientar(patrulla, dir);
    assert.equal(g.filas2.length, g.h, dir);
    for (const f of g.filas2) assert.equal(f.length, g.w, dir);
  }
});

test('Misil: un misil sobre un campamento destruye lo que hay dentro y suma puntos', () => {
  const { partida, paso } = nueva(3);
  const blanco = tiendaALaVista(partida);
  const quietos = () => partida.entidades.filter((e) => !e.desde).length;
  const antes = quietos();
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(partida.puntos > 0, 'debería haber acertado');
  assert.ok(quietos() < antes);
  assert.ok(!partida.entidades.includes(blanco));
});

test('Misil: lo destruido no vuelve, y deja un cráter que se queda', () => {
  const { partida, paso } = nueva(3);
  const blanco = tiendaALaVista(partida);
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(!partida.entidades.includes(blanco));
  assert.equal(partida.crateres, 1);
  // Mucho más de lo que tardaba en volver antes (14 s): ni la tienda ni un clon en su sitio.
  for (let t = 0; t < 30_000 && !partida.terminado; t += 100) {
    paso(100);
    assert.ok(!partida.entidades.some((e) => e.tipo === blanco.tipo && e.x === blanco.x && e.y === blanco.y), 'ha vuelto');
  }
  assert.ok(partida.crateres >= 1);
});

test('Misil: se cuentan los objetivos destruidos por tipo y el máximo con un solo misil', () => {
  const { partida, paso } = nueva(3);
  assert.deepEqual(partida.detalle, { suma: { total: 0 }, max: { aLaVez: 0, zona: 1 } });
  const blanco = tiendaALaVista(partida);
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  const d = partida.detalle;
  assert.equal(d.suma.tienda, 1);
  assert.equal(d.suma.total, Object.entries(d.suma).reduce((n, [k, v]) => n + (k === 'total' ? 0 : v), 0));
  assert.ok(d.max.aLaVez >= 1);
  assert.equal(d.max.aLaVez, d.suma.total, 'con un solo misil, el máximo a la vez es todo lo destruido');
  // Y lo que la vista enseña tiene una etiqueta para cada clave.
  const claves = new Set(misil.estadisticas.suma.map(([k]) => k));
  for (const k of Object.keys(d.suma)) assert.ok(claves.has(k), `falta etiqueta para ${k}`);
  assert.ok(misil.estadisticas.max.some(([k]) => k === 'aLaVez'));
});

test('Misil: una zona sin objetivos a la vista se detecta, y con uno solo ya no', () => {
  const zona = ZONAS[1];
  const dentro = { x: zona.x + 20, y: zona.y + 20, forma: { w: 5, h: 5 } };
  const fuera = { x: zona.x + MUNDO_W + 10, y: zona.y, forma: { w: 5, h: 5 } };
  assert.equal(hayObjetivosEn([], zona), false);
  assert.equal(hayObjetivosEn([fuera], zona), false);
  assert.equal(hayObjetivosEn([fuera, dentro], zona), true);
});

test('Misil: no hace falta dar en el píxel, basta con que la explosión alcance el objetivo', () => {
  const { partida, paso } = nueva(4);
  const blanco = tiendaALaVista(partida);
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

test('Misil: el trayecto sale de una zona y llega a la siguiente, en ruta lineal', () => {
  for (let n = 0; n < ZONAS.length - 1; n++) {
    const a = ZONAS[n], b = ZONAS[n + 1];
    assert.deepEqual(trayecto(n, 0), { x: a.x, y: a.y }, `zona ${n} al salir`);
    assert.deepEqual(trayecto(n, 1), { x: b.x, y: b.y }, `zona ${n} al llegar`);
  }
  assert.ok(ESTANCIA_MS >= 30000, 'se queda un rato largo sobre cada zona');
  assert.equal(TRANSICION_MS, 3500);
  // El vuelo crece con la distancia, y nunca baja del mínimo.
  const dist = (n) => Math.hypot(ZONAS[n + 1].x - ZONAS[n].x, ZONAS[n + 1].y - ZONAS[n].y);
  const duraciones = ZONAS.slice(1).map((_, n) => duracionVuelo(n));
  duraciones.forEach((d, n) => assert.equal(d, Math.max(3500, dist(n) * 12), `zona ${n}`));
  assert.ok(Math.max(...duraciones) > TRANSICION_MS, 'hay vuelos largos');
});

test('Misil: el vuelo es suave: sin saltos, arrancando y frenando despacio', () => {
  for (let n = 0; n < ZONAS.length - 1; n++) {
    const dur = duracionVuelo(n);
    const pasos = [];
    let prev = trayecto(n, 0);
    for (let t = DT; t <= dur; t += DT) {
      const r = trayecto(n, t / dur);
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
    for (let t = 0; t <= dur; t += 100) {
      const r = trayecto(n, t / dur);
      assert.ok(r.x >= 0 && r.y >= 0 && r.x + MUNDO_W <= MAPA_W && r.y + MUNDO_H <= MAPA_H);
    }
  }
});

test('Misil: el VTOL no se mueve mientras hay un misil en el aire ni antes de cumplir la estancia', () => {
  const { partida, paso } = nueva(3, { quieto: false });
  // Sin acertar, la partida acaba pronto (tres fallos); lo que se comprueba es que, mientras
  // dura, la cámara solo se aparta de su zona lo que dura el vaivén y que no vuela antes de tiempo.
  for (let t = 0; t < 60000 && !partida.terminado; t += DT) {
    const volabaAntes = partida.volando;
    paso();
    if (partida.volando && !volabaAntes) assert.ok(partida.estancia >= ESTANCIA_MS, 'vuela tras la estancia');
    if (!partida.volando) {
      const z = ZONAS.find((zz) => zz.nombre === partida.zona);
      assert.ok(Math.abs(partida.camara.x - z.x) <= 3 && Math.abs(partida.camara.y - z.y) <= 3, 'parado, la cámara está en su zona');
    }
  }
});

test('Misil: sin quieto, parado en una zona la cámara vaivenea, y nunca más de la amplitud', () => {
  for (const [zonaInicial, amp] of [[0, 2], [5, 7]]) conNivel(zonaInicial, { disparoMs: 0, lanzadorMs: 0 }, () => {  // sin impactos: sacuden más que la amplitud
    const { partida, paso } = nueva(9, { quieto: false, inmortal: true, zonaInicial });
    const z = ZONAS[zonaInicial];
    const vistas = new Set();
    for (let t = 0; t < 10_000; t += 50) {
      paso(50);
      const c = partida.camara;
      vistas.add(`${c.x},${c.y}`);
      assert.ok(Math.abs(c.x - z.x) <= amp + 1 && Math.abs(c.y - z.y) <= amp + 1, `zona ${zonaInicial}: ${c.x - z.x},${c.y - z.y}`);
    }
    assert.ok(vistas.size > 1, `zona ${zonaInicial}: la cámara no se ha movido`);
  });
});

test('Misil: la sexta zona acaba con victoria y las vidas que quedan suman; antes, se vuela a la siguiente', () => {
  const { partida, paso } = nueva(4, { inmortal: true, zonaInicial: 5 });
  assert.equal(partida.detalle.max.zona, 6);
  paso(ESTANCIA_MS + 10_000);
  assert.ok(partida.terminado && partida.victoria);
  assert.ok(partida.puntos >= 3 * BONO_VIDA, `puntos ${partida.puntos}`); // el bono, más lo que haya caído en la mira
  const arranque = nueva(4, { inmortal: true });
  assert.equal(arranque.partida.zona, 'base');
  arranque.paso(ESTANCIA_MS + 10_000);
  assert.ok(arranque.partida.detalle.max.zona >= 2 || arranque.partida.volando, 'pasa a la siguiente zona');
  assert.ok(!arranque.partida.terminado);
});

test('Misil: perder todas las vidas no es victoria y no da bono', () => {
  const { partida, paso } = nueva(3);
  apuntar(partida, paso, { x: 150, y: 95 });
  paso(60_000);
  assert.ok(partida.terminado && !partida.victoria);
  assert.equal(partida.puntos, 0);
});

// --- Fase 2: el helicóptero --------------------------------------------------------------

test('Misil: el helicóptero cabe en la H del helipuerto, tiene rotor que gira y cola', () => {
  const heli = tipoDe('helicoptero');
  assert.equal(heli.w, 13);
  assert.equal(heli.h, 13);
  assert.notDeepEqual(heli.filas, heli.filas2);
  for (const fs of [heli.filas, heli.filas2]) {
    fs.forEach((f, j) => [...f].forEach((c, i) => { if (c === '#') assert.ok(Math.hypot(i - 6, j - 6) <= 8, `píxel ${i},${j} fuera de la H`); }));
    assert.equal(fs[6][0], '#', 'la cola sale a la izquierda');
  }
  assert.equal(heli.valor, 150);
});

// Una partida con la que el helicóptero espera en la base al empezar (la probabilidad es 0,5).
function conHelicoptero(opciones = {}) {
  for (let semilla = 1; semilla < 60; semilla++) {
    const juego = nueva(semilla, opciones);
    if (juego.partida.entidades.some((e) => e.tipo.nombre === 'helicoptero')) return juego;
  }
  throw new Error('ninguna semilla saca helicóptero');
}

test('Misil: el helicóptero espera en la H, despega al segundo y al poco ya se ha ido', () => {
  const { partida, paso } = conHelicoptero();
  const heli = partida.entidades.find((e) => e.tipo.nombre === 'helicoptero');
  const [hx, hy] = ZONAS[0].helipuerto;
  assert.deepEqual({ x: heli.x + 6, y: heli.y + 6 }, { x: hx, y: hy });
  paso(900);
  assert.deepEqual({ x: heli.x + 6, y: heli.y + 6 }, { x: hx, y: hy }, 'sigue posado');
  assert.equal(heli.vel, 0);
  paso(600);
  assert.ok(heli.vel > 0, 'despega');
  assert.ok(Math.hypot(heli.x + 6 - hx, heli.y + 6 - hy) > 0, 'se mueve');
  paso(15_000);
  assert.ok(!partida.entidades.includes(heli), 'se ha ido');
  assert.equal(partida.crateres, 0);
});

test('Misil: acertar al helicóptero posado da como mínimo su valor', () => {
  const { partida, paso } = conHelicoptero();
  const heli = partida.entidades.find((e) => e.tipo.nombre === 'helicoptero');
  partida.mandoBoton({ mando: 1, pulsado: true });
  partida.mandoBoton({ mando: 2, pulsado: true });
  apuntar(partida, paso, enVista(partida, centro(heli)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  assert.ok(partida.puntos >= 150, `puntos ${partida.puntos}`);
  assert.equal(partida.detalle.suma.helicoptero, 1);
});

test('Misil: las sendas están a la vista de su zona, fuera de la carretera y sin pisar un estático', () => {
  const cajas = ESTATICOS.map(([n, x, y, d]) => { const f = tamano(n, d); return { x: x - 1, y: y - 1, x1: x + f.w, y1: y + f.h }; });
  for (const { zona, puntos } of SENDAS) {
    const z = ZONAS.find((zz) => zz.nombre === zona);
    for (const [x, y] of puntos) {
      assert.ok(x >= z.x && x < z.x + MUNDO_W && y >= z.y && y < z.y + MUNDO_H, `${zona} ${x},${y} a la vista`);
      assert.ok(!enCarretera(x, y, 2), `${zona} ${x},${y} en carretera`);
    }
    // Se muestrea cada píxel del recorrido menos el primer y el último punto, que son la puerta o el vehículo.
    for (let i = 0; i + 1 < puntos.length; i++) {
      const [a, b] = [puntos[i], puntos[i + 1]];
      const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]));
      for (let k = 0; k <= n; k++) {
        if ((i === 0 && k === 0) || (i === puntos.length - 2 && k === n)) continue;
        const x = a[0] + ((b[0] - a[0]) * k) / n, y = a[1] + ((b[1] - a[1]) * k) / n;
        assert.ok(!enCarretera(x, y, 2), `${zona} tramo ${i} en carretera`);
        assert.ok(!cajas.some((c) => x >= c.x && x <= c.x1 && y >= c.y && y <= c.y1), `${zona} tramo ${i} pisa un estático en ${x.toFixed(0)},${y.toFixed(0)}`);
      }
    }
  }
});

test('Misil: sale un grupo de 2 a 4 soldados por una puerta, camina, y al acabar se queda quieto o se va', () => {
  let vistos = 0, quietos = 0, idos = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6, 7, 8]) {
    // Zona del puesto (1), con un grupo cada ~1 s para no esperar.
    const { partida, paso } = nueva(semilla, { zonaInicial: 1, pieMs: 1000, inmortal: true });
    const andando = () => partida.entidades.filter((e) => e.pie);
    for (let t = 0; t < 4000 && !andando().length; t += DT) paso();
    const grupo = andando();
    assert.ok(grupo.length >= 2 && grupo.length <= 4, `semilla ${semilla}: ${grupo.length} soldados`);
    assert.ok(grupo.every((e) => e.tipo.nombre === 'soldado' && e.pie.ruta === grupo[0].pie.ruta));
    assert.ok(!hayObjetivosEn(grupo, ZONAS[1]), 'los que caminan no retienen al VTOL');
    const antes = grupo.map((e) => [e.x, e.y]);
    paso(1000);
    assert.ok(grupo.some((e, i) => e.x !== antes[i][0] || e.y !== antes[i][1]), 'se mueven');
    // Hasta que el grupo acabe (la senda más larga son ~100 px a 7 px/s): lanza otros, así que se sigue al primero.
    for (let t = 0; t < 30000 && grupo.some((e) => e.pie); t += DT) paso();
    vistos++;
    for (const e of grupo) {
      if (partida.entidades.includes(e)) { assert.equal(e.vel, 0); quietos++; } else idos++;
    }
    for (const e of partida.entidades) if (e.pie) assert.ok(e.vel > 0);
  }
  assert.equal(vistos, 8);
  assert.ok(quietos > 0 && idos > 0, `quietos ${quietos}, idos ${idos}`);
});

test('Misil: sin pieMs en la zona no sale nadie a pie, y en la base tampoco hay sendas', () => {
  const { partida, paso } = nueva(3, { inmortal: true });
  for (let t = 0; t < 30000; t += DT) paso();
  assert.ok(!partida.entidades.some((e) => e.pie));
});

// --- Fase 4: dificultad por zona y fuego enemigo ---

// Cambia la fila `i` de NIVELES mientras dura `f` (la tabla es la de verdad, no una copia).
function conNivel(i, cambios, f) {
  const original = NIVELES[i];
  NIVELES[i] = { ...original, ...cambios };
  try { return f(); } finally { NIVELES[i] = original; }
}
const indiceZona = (partida) => ZONAS.findIndex((z) => z.nombre === partida.zona);

// Deja pasar `ms` sin tocar nada y vigila las reglas del disparo en cada tick.
function vigilar(partida, ms) {
  const visto = { maxSimultaneas: 0, impactos: [], tirados: new Set(), amenazas: 0 };
  const conocidas = new Set();
  let antes = partida.impactos;
  for (let t = 0; t < ms; t += DT) {
    partida.tick(DT);
    const idx = indiceZona(partida);
    const am = partida.amenazas;
    if (idx >= 0) {
      assert.ok(am.length <= (NIVELES[idx].simultaneas ?? 1), `zona ${idx + 1}: ${am.length} amenazas a la vez`);
      visto.maxSimultaneas = Math.max(visto.maxSimultaneas, am.length);
    }
    for (const a of am) {
      const clave = `${a.desde}:${a.hasta}`;
      if (conocidas.has(clave)) continue;
      conocidas.add(clave);
      visto.amenazas++;
      visto.tirados.add(a.e.tipo.nombre);
      assert.ok(a.hasta - a.desde >= 3500, `aviso de ${a.hasta - a.desde} ms`);
      for (const o of am) if (o !== a) assert.ok(Math.abs(o.hasta - a.hasta) >= ESPACIO_DISPAROS_MS, 'dos amenazas con impactos juntos');
    }
    if (partida.impactos > antes) { visto.impactos.push(t); antes = partida.impactos; }
  }
  for (let i = 1; i < visto.impactos.length; i++) {
    assert.ok(visto.impactos[i] - visto.impactos[i - 1] >= ESPACIO_DISPAROS_MS - DT, 'impactos a menos de 3 s');
  }
  return visto;
}

test('Misil: NIVELES tiene una fila por zona, aprieta siempre más y el aviso nunca baja de 3500', () => {
  assert.equal(NIVELES.length, ZONAS.length);
  for (const col of ['vel', 'maxVehiculos', 'vaiven', 'puntos']) {
    for (let i = 1; i < NIVELES.length; i++) assert.ok(NIVELES[i][col] >= NIVELES[i - 1][col], `${col} baja en la fila ${i + 1}`);
  }
  for (const n of NIVELES) assert.ok(n.avisoMs === 0 || n.avisoMs >= 3500, `avisoMs ${n.avisoMs}`);
  assert.equal(NIVELES.filter((n) => n.simultaneas > 1).length, 1);
  assert.equal(NIVELES.at(-1).simultaneas, 2);
});

test('Misil: la velocidad de los vehículos sale de la fila de la zona y se puede empezar en otra', () => {
  const camion = OBJETIVOS.find((o) => o.nombre === 'camion');
  const v = { tipo: camion, vel: 0, desde: 'a', hasta: 'f', dist: 0 };
  circular([v], 1000, () => 0, 1.5);
  assert.equal(v.vel, camion.velocidad * 1.5);
  assert.equal(nueva(3, { zonaInicial: 0 }).partida.camara.x, ZONAS[0].x);
  assert.equal(nueva(3, { zonaInicial: 5 }).partida.camara.x, ZONAS[5].x);
});

test('Misil: en las zonas sin disparo no hay amenazas, ni lanzadores, y los RPG fijos son de la zona 6', () => {
  for (const zona of [0, 1]) {
    const { partida } = nueva(5, { inmortal: true, zonaInicial: zona });
    assert.equal(vigilar(partida, 40000).amenazas, 0, `zona ${zona + 1}`);
  }
  for (const zona of [0, 1, 2, 3]) {
    const { partida } = nueva(9, { inmortal: true, zonaInicial: zona });
    for (let t = 0; t < 40000; t += DT) {
      partida.tick(DT);
      assert.ok(!partida.entidades.some((e) => e.mision?.luego === 'lanzar' || e.tipo.nombre === 'lanzador'), `zona ${zona + 1}`);
    }
  }
  for (const [n, x, y, dir] of ESTATICOS.filter((e) => e[0] === 'rpg')) {
    assert.ok(hayObjetivosEn([{ x, y, forma: tamano(n, dir) }], ZONAS[5]), 'los RPG fijos son de la zona 6');
  }
});

test('Misil: con un antiaéreo a la vista y sin tocar nada, a los disparoMs + avisoMs se pierde una vida', () => {
  const { partida } = nueva(3, { zonaInicial: 2 });
  const n = NIVELES[2];
  assert.equal(partida.amenazas.length, 0);
  let vidasAntes = partida.vidas, cuando = null;
  for (let t = DT; t < 30000 && cuando === null; t += DT) {
    vidasAntes = partida.vidas;
    partida.tick(DT);
    if (partida.impactos === 1) cuando = t;
  }
  assert.ok(cuando >= n.disparoMs + n.avisoMs && cuando <= n.disparoMs + n.avisoMs + 3 * DT, `impacto a los ${cuando} ms`);
  assert.equal(partida.vidas, vidasAntes - 1);
});

test('Misil: destruir al tirador durante el aviso cancela la amenaza y suma puntos', () => {
  const { partida, paso } = nueva(3, { zonaInicial: 2, inmortal: true });
  while (!partida.amenazas.length) paso();
  const tirador = partida.amenazas[0].e;
  const antes = partida.puntos;
  apuntar(partida, paso, enVista(partida, centro(tirador)));
  partida.mandoBoton({ mando: 1, pulsado: true });
  partida.mandoBoton({ mando: 2, pulsado: true });
  for (let t = 0; t < 5000 && partida.entidades.includes(tirador); t += DT) paso();
  assert.ok(!partida.entidades.includes(tirador), 'destruido');
  assert.ok(partida.puntos > antes);
  assert.ok(!partida.amenazas.some((a) => a.e === tirador), 'sin amenaza');
  assert.equal(partida.impactos, 0);
});

test('Misil: las reglas del disparo valen en partidas largas, con varias semillas, en todas las zonas con fuego', () => {
  let impactos = 0, dobles = 0;
  const tirados = new Set();
  for (const semilla of [1, 2, 3, 4]) {
    for (const zona of [2, 3, 4, 5]) {
      // Con lanzadores frecuentes, para que se pisen con los turnos normales.
      conNivel(zona, zona >= 4 ? { lanzadorMs: 4000 } : {}, () => {
        const { partida } = nueva(semilla, { inmortal: true, zonaInicial: zona });
        const v = vigilar(partida, 100000);
        impactos += v.impactos.length;
        if (v.maxSimultaneas >= 2) dobles++;
        v.tirados.forEach((t) => tirados.add(t));
      });
    }
  }
  assert.ok(impactos > 20, `solo ${impactos} impactos`);
  assert.ok(tirados.has('antiaereo'));
  assert.ok(tirados.has('lanzador'), 'algún lanzador dispara');
  assert.ok(tirados.has('rpg'), 'algún RPG dispara');
  assert.ok(dobles > 0, 'en la zona 6 llega a haber dos amenazas a la vez');
});

test('Misil: un camión aparca, despliega el lanzador y, sin tocar nada, dispara a los LANZADOR_DESPLIEGUE_MS + avisoMs', () => {
  let atacados = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6]) {
    conNivel(4, { lanzadorMs: 2000, disparoMs: 0 }, () => {
      const { partida } = nueva(semilla, { inmortal: true, zonaInicial: 4 });
      let aparcado = null;
      for (let t = 0; t < 45000; t += DT) {
        partida.tick(DT);
        const l = aparcado === null && partida.entidades.find((e) => e.mision?.fase === 'despliega');
        if (l) {
          aparcado = t;
          assert.ok(APARCAMIENTOS.some((a) => a === l.mision.ap && a.zona === 'aerodromo'));
        }
        if (aparcado !== null && partida.impactos) {
          atacados++;
          const espera = t - aparcado;
          const esperado = LANZADOR_DESPLIEGUE_MS + NIVELES[4].avisoMs;
          assert.ok(espera >= esperado && espera <= esperado + 4 * DT, `${espera} ms desde aparcar`);
          break;
        }
      }
    });
  }
  assert.ok(atacados >= 3, `solo ${atacados} de 6 partidas llegaron a ser atacadas por un lanzador`);
});

test('Misil: el lanzador desplegado vale el doble que un camión y cambia de dibujo a mitad del despliegue', () => {
  assert.equal(OBJETIVOS.find((o) => o.nombre === 'lanzador').valor, 2 * OBJETIVOS.find((o) => o.nombre === 'camion').valor);
  conNivel(4, { lanzadorMs: 2000, disparoMs: 0 }, () => {
    const { partida } = nueva(2, { inmortal: true, zonaInicial: 4 });
    let visto = false;
    for (let t = 0; t < 45000 && !visto; t += DT) {
      partida.tick(DT);
      const l = partida.entidades.find((e) => e.mision?.fase === 'despliega');
      if (l && l.mision.t < LANZADOR_DESPLIEGUE_MS / 2 - DT) assert.equal(l.tipo.nombre, 'camion');
      if (l && l.mision.t >= LANZADOR_DESPLIEGUE_MS / 2 + DT) { assert.equal(l.tipo.nombre, 'lanzador'); visto = true; }
    }
    assert.ok(visto, 'se vio desplegado');
  });
});

test('Misil: en el cuartel algún soldado a pie lleva RPG, y solo es tirador al pararse', () => {
  let rpgs = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6]) {
    const { partida, paso } = nueva(semilla, { zonaInicial: 5, pieMs: 800, inmortal: true });
    for (let t = 0; t < 20000; t += DT) {
      paso();
      for (const e of partida.entidades) {
        if (e.tipo.nombre !== 'rpg') continue;
        if (e.pie) { rpgs++; assert.ok(!e.tirador, 'andando no dispara'); } else assert.ok(e.tirador);
      }
    }
  }
  assert.ok(rpgs > 0, 'ningún RPG a pie');
  const { partida, paso } = nueva(1, { zonaInicial: 4, pieMs: 800, inmortal: true });
  for (let t = 0; t < 10000; t += DT) { paso(); assert.ok(!partida.entidades.some((e) => e.pie && e.tipo.nombre === 'rpg')); }
});

// --- Fase 5.2 y 5.3: bajar de un vehículo y subir a uno ---

const SENTIDO = { der: [1, 0], izq: [-1, 0], arr: [0, -1], aba: [0, 1] };

// Avanza hasta que `cond` se cumple (o se agota `ms`); devuelve si se cumplió.
function hasta(partida, ms, cond) {
  for (let t = 0; t < ms; t += DT) { partida.tick(DT); if (cond()) return true; }
  return false;
}

test('Misil: un vehículo aparca, bajan 2-4 soldados, espera DESEMBARCO_MS y vuelve a la red de cara', () => {
  let vistos = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6]) {
    const { partida } = nueva(semilla, { zonaInicial: 1, pieMs: 800, inmortal: true });
    const baja = () => partida.entidades.find((e) => e.mision?.luego === 'desembarcar' && e.mision.fase === 'baja');
    if (!hasta(partida, 90000, baja)) continue;
    const v = baja();
    const m = v.mision;
    const soldados = () => partida.entidades.filter((e) => e.pie?.ruta === SENDAS.find((d) => d.zona === 'puesto' && d.de === 'aparcamiento').puntos);
    assert.ok(APARCAMIENTOS.includes(m.ap) && m.ap.zona === 'puesto');
    assert.ok([2, 3, 4].includes(soldados().length), `${soldados().length} soldados`);
    // De cara a la salida: mira hacia la carretera, no hacia la explanada.
    assert.equal(v.forma.w > v.forma.h, true);
    assert.ok(Math.sign(SENTIDO[v.dir][0]) === Math.sign(NODOS[m.ap.tramo[0]][0] - (m.ap.x + 10)), 'mira a la carretera');
    const aparcado = m.t;
    assert.ok(aparcado < DT * 3);
    let previo = [v.cx, v.cy], atras = 0;
    assert.ok(hasta(partida, DESEMBARCO_MS + 30000, () => {
      const [dx, dy] = [v.cx - previo[0], v.cy - previo[1]];
      if (v.mision && dx * SENTIDO[v.dir][0] + dy * SENTIDO[v.dir][1] < -1e-9) atras++;
      previo = [v.cx, v.cy];
      return !v.mision;
    }), 'vuelve a la red');
    assert.equal(atras, 0, 'nunca marcha atrás');
    assert.ok(v.desde && v.hasta, 'circula otra vez');
    assert.equal(v.desde + v.hasta, m.ap.tramo);
    assert.ok(v.dist >= m.ap.dist);
    vistos++;
  }
  assert.ok(vistos >= 3, `solo ${vistos} desembarcos`);
});

test('Misil: el desembarco espera DESEMBARCO_MS y una explanada ocupada no se usa para otra misión', () => {
  const { partida } = nueva(2, { zonaInicial: 1, pieMs: 800, inmortal: true });
  let inicio = null, fin = null;
  for (let t = 0; t < 120000 && fin === null; t += DT) {
    partida.tick(DT);
    const usadas = partida.entidades.filter((e) => e.mision?.ap).map((e) => e.mision.ap);
    assert.equal(new Set(usadas).size, usadas.length, 'dos misiones en la misma explanada');
    const v = partida.entidades.find((e) => e.mision?.fase === 'baja');
    if (v && inicio === null) inicio = t;
    if (inicio !== null && !partida.entidades.some((e) => e.mision?.fase === 'baja')) fin = t;
  }
  assert.notEqual(fin, null, 'no hubo desembarco');
  assert.ok(fin - inicio >= DESEMBARCO_MS - DT && fin - inicio <= DESEMBARCO_MS + 2 * DT, `${fin - inicio} ms`);
});

test('Misil: al volver a la red sin hueco, el vehículo espera y no pisa a nadie', () => {
  for (const semilla of [1, 2, 3]) {
    const { partida } = nueva(semilla, { zonaInicial: 1, pieMs: 800, inmortal: true });
    for (let t = 0; t < 90000; t += DT) {
      partida.tick(DT);
      const vs = partida.entidades.filter((e) => e.desde);
      for (const v of vs) assert.ok(espacioDelante(v, vs) >= 0);
      for (let i = 0; i < vs.length; i++) for (let j = i + 1; j < vs.length; j++) {
        const [a, b] = [vs[i], vs[j]];
        const solapa = a.x < b.x + b.forma.w && b.x < a.x + a.forma.w && a.y < b.y + b.forma.h && b.y < a.y + a.forma.h;
        assert.ok(!solapa, `semilla ${semilla}: ${a.tipo.nombre} y ${b.tipo.nombre} se pisan`);
      }
    }
  }
});

test('Misil: un grupo sube al vehículo del final: desaparecen y el vehículo deja de ser estático y se incorpora a la red', () => {
  let subidos = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6]) {
    const { partida } = nueva(semilla, { zonaInicial: 4, pieMs: 800, inmortal: true });
    const camion = partida.entidades.find((e) => e.arranque);
    assert.ok(camion && !camion.desde);
    const marcha = () => camion.mision?.luego === 'arrancar';
    if (!hasta(partida, 120000, marcha)) continue;
    assert.ok(!partida.entidades.some((e) => e.pie?.objetivo === camion), 'el grupo ha desaparecido');
    let previo = [camion.cx, camion.cy], atras = 0;
    assert.ok(hasta(partida, 30000, () => {
      const [dx, dy] = [camion.cx - previo[0], camion.cy - previo[1]];
      if (camion.mision && dx * SENTIDO[camion.dir][0] + dy * SENTIDO[camion.dir][1] < -1e-9) atras++;
      previo = [camion.cx, camion.cy];
      return !camion.mision;
    }));
    assert.equal(atras, 0, 'nunca marcha atrás');
    assert.equal(camion.desde + camion.hasta, camion.arranque.tramo);
    assert.ok(camion.dist >= camion.arranque.dist);
    subidos++;
  }
  assert.ok(subidos >= 2, `solo ${subidos} subidas`);
});

test('Misil: si el vehículo del final está destruido no se sortea su senda, y si lo destruyen en camino el grupo se queda quieto', () => {
  const quitar = (partida, v) => partida.entidades.splice(partida.entidades.indexOf(v), 1);
  for (const [zonaInicial, zona, tramo] of [[4, 'aerodromo', 'qr'], [5, 'cuartel', 'su']]) {
    const { partida } = nueva(7, { zonaInicial, pieMs: 500, inmortal: true });
    quitar(partida, partida.entidades.find((e) => e.arranque?.tramo === tramo));
    const subir = SENDAS.find((d) => d.zona === zona && d.fin === 'subir').puntos;
    for (let t = 0; t < 60000; t += DT) { partida.tick(DT); assert.ok(!partida.entidades.some((e) => e.pie?.ruta === subir)); }
  }
  let quietos = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6]) {
    const { partida } = nueva(semilla, { zonaInicial: 4, pieMs: 800, inmortal: true });
    const camion = partida.entidades.find((e) => e.arranque);
    const grupo = () => partida.entidades.filter((e) => e.pie?.objetivo === camion);
    if (!hasta(partida, 120000, () => grupo().length && !camion.mision)) continue;
    const miembros = grupo();
    quitar(partida, camion);
    assert.ok(hasta(partida, 30000, () => miembros.every((e) => !e.pie)));
    for (const e of miembros) { assert.ok(partida.entidades.includes(e)); assert.equal(e.vel, 0); }
    quietos++;
    break;
  }
  assert.equal(quietos, 1);
});

// --- Air Strike, fase 1: la derrota ---

// Pierde las tres vidas por fallos y devuelve la partida justo al entrar en la derrota.
function hastaDerrota(semilla = 3) {
  const j = nueva(semilla);
  apuntar(j.partida, j.paso, { x: 150, y: 95 });
  for (let t = 0; t < 60_000 && j.partida.fase !== 'derrota'; t += DT) j.paso();
  assert.equal(j.partida.fase, 'derrota');
  return j;
}

test('Air Strike: al perder por fallos primero sale «HAS PERDIDO» y a los DERROTA_MS se pasa a los resultados', () => {
  const { partida, paso } = hastaDerrota();
  assert.equal(partida.vidas, 0);
  assert.equal(partida.terminado, false);
  paso(DERROTA_MS - 4 * DT);
  assert.equal(partida.terminado, false);
  paso(6 * DT);
  assert.equal(partida.terminado, true);
});

test('Air Strike: en la derrota el mundo está congelado y la mira no responde', () => {
  const { partida, paso } = hastaDerrota();
  const antes = partida.entidades.map((e) => [e.x, e.y]);
  const mira = partida.mira;
  const camara = partida.camara;
  partida.mando({ mando: 1, delta: 5 });
  partida.mando({ mando: 2, delta: -5 });
  paso(1000);
  assert.deepEqual(partida.entidades.map((e) => [e.x, e.y]), antes);
  assert.deepEqual(partida.mira, mira);
  assert.deepEqual(partida.camara, camara);
  assert.equal(partida.amenazas.length, 0);
});

test('Air Strike: perdiste por fallos, sin culpable; por un impacto, el culpable es el tirador que disparó', () => {
  assert.equal(hastaDerrota().partida.culpable, null);
  let porImpacto = 0;
  for (const semilla of [1, 2, 3, 4, 5, 6]) {
    conNivel(2, { disparoMs: 500 }, () => {
      const { partida, paso } = nueva(semilla, { zonaInicial: 2 });
      apuntar(partida, paso, { x: 150, y: 95 });
      let tirador = null, impactos = partida.impactos;
      for (let t = 0; t < 60_000 && partida.fase !== 'derrota'; t += DT) {
        if (partida.amenazas.length) tirador = partida.amenazas[0].e;
        paso();
        if (partida.fase === 'derrota' && partida.impactos > impactos) {
          porImpacto++;
          assert.deepEqual(partida.culpable, { x: Math.round(tirador.x), y: Math.round(tirador.y), w: tirador.forma.w, h: tirador.forma.h });
        } else if (partida.fase === 'derrota') assert.equal(partida.culpable, null);
        impactos = partida.impactos;
      }
    });
  }
  assert.ok(porImpacto > 0, 'ninguna semilla acabó por un impacto');
});

test('Air Strike: la derrota sigue enseñando el mapa, atenuado a trama de ajedrez, con el texto y los puntos', () => {
  const { l } = hastaDerrota();
  const bit = (fb, x, y) => (fb[(y >> 3) * W + x] >> (y & 7)) & 1;
  for (let i = 0; i < 10; i++) assert.ok(l[i].some((b) => b), `pantalla ${i + 1} vacía en la derrota`);
  // En una pantalla sin texto (la 1) ningún píxel de la mitad atenuada queda encendido, y la otra mitad conserva el mapa.
  let vivos = 0;
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) { if ((x + y) % 2 === 0) assert.equal(bit(l[0], x, y), 0); else vivos += bit(l[0], x, y); }
  assert.ok(vivos > 0, 'el mapa sigue ahí');
});

// --- Air Strike, fase 2: explosiones ---

test('Air Strike: cada objetivo destruido crea su explosión, desfasadas, y ninguna sigue viva pasado su tiempo', () => {
  const { partida, paso } = nueva(3);
  const blanco = tiendaALaVista(partida);
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  const n = partida.detalle.suma.total;
  assert.ok(n >= 1);
  const ex = partida.explosiones;
  assert.equal(ex.length, n);
  ex.forEach((e, i) => assert.equal(e.t0, ex[0].t0 + 40 * i, 'desfase de 40 ms por orden'));
  for (const e of ex) assert.ok(e.fin - e.t0 <= 4000, 'dura lo que dura, no más');
  paso(5000);
  assert.equal(partida.explosiones.length, 0, 'se han ido todas');
});

test('Air Strike: la explosión de un depósito es grande y con chispas, la de un soldado pequeña, la de un vehículo a golpes escalonados', () => {
  const azar = azarDe(5);
  const dep = crearExplosion(10, 10, 'grande', 0, azar);
  const sol = crearExplosion(10, 10, 'pequena', 0, azar);
  const tan = crearExplosion(10, 10, 'media', 0, azar);
  assert.ok(dep.estallidos[0].r >= 12 && dep.chispas === 8);
  assert.equal(sol.estallidos.length, 1);
  assert.ok(tan.estallidos.length >= 3 && tan.estallidos.length <= 4, 'dos o tres y uno final');
  for (const e of [dep, sol, tan]) {
    assert.ok(e.fin > e.humoDesde && e.humoDesde >= Math.max(...e.estallidos.map((b) => b.t + b.dur)));
    assert.ok(e.humo.length >= 3 && e.humo.length <= 4);
  }
  assert.ok(tan.estallidos.every((b, i, l) => !i || b.t > l[i - 1].t), 'escalonados');
  // Misma semilla, misma explosión: el azar no sale al dibujar.
  assert.deepEqual(crearExplosion(1, 2, 'media', 0, azarDe(9)), crearExplosion(1, 2, 'media', 0, azarDe(9)));
});

test('Air Strike: la explosión se ve mientras dura y desaparece del dibujo cuando acaba', () => {
  const { partida, paso, l } = nueva(3);
  const blanco = tiendaALaVista(partida);
  apuntar(partida, paso, enVista(partida, centro(blanco)));
  for (let t = 0; t < 8000 && partida.puntos === 0; t += DT) paso();
  paso(100);
  const durante = l.map((fb) => fb.join());
  paso(5000);
  assert.ok(partida.explosiones.length === 0);
  assert.notDeepEqual(l.slice(0, 9).map((fb) => fb.join()), durante.slice(0, 9));
});
