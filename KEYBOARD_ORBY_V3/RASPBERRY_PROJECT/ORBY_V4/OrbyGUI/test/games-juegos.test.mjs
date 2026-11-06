import test from 'node:test';
import assert from 'node:assert/strict';
import { JUEGOS, juegoPorId } from '../src/games/catalogo.mjs';
import { lienzo, limpiarLienzo, PANTALLAS, FB } from '../src/games/proto.mjs';
import { Malla, W, H } from '../src/games/dibujo.mjs';

// Un generador con semilla: la partida simulada tiene que ser la misma cada vez para que un
// fallo se pueda repetir.
function aleatorioConSemilla(semilla) {
  let s = semilla >>> 0;
  return (tope) => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return Math.floor((s / 0x100000000) * tope);
  };
}

const DT = 8;   // ms por tic, como el reloj del motor

// Juega una partida sin teclado: teclas al azar en las pantallas de la rejilla y la rueda
// girando de un lado a otro. Devuelve cuántos tics duró.
function simular(juego, semilla, { maxTics = 120_000 } = {}) {
  const aleatorio = aleatorioConSemilla(semilla);
  let reloj = 0;
  const partida = juego.crear({ ahora: () => reloj, aleatorio });
  const l = lienzo();
  let tics = 0;
  while (!partida.terminado && tics < maxTics) {
    reloj += DT;
    partida.tick(DT);
    limpiarLienzo(l);
    partida.dibujar(l);
    if (aleatorio(40) === 0) {
      const pantalla = 1 + aleatorio(9);
      partida.tecla?.({ tipo: 'tecla', tecla: pantalla, pantalla, pulsada: true });
      partida.tecla?.({ tipo: 'tecla', tecla: pantalla, pantalla, pulsada: false });
    }
    partida.rueda?.({ tipo: 'rueda', delta: aleatorio(2) ? 40 : -40, angulo: 0 });
    tics++;
  }
  return { tics, partida, l };
}

test('el catálogo no repite identificadores y todos los juegos están completos', () => {
  const ids = JUEGOS.map((j) => j.id);
  assert.equal(new Set(ids).size, ids.length);
  for (const j of JUEGOS) {
    assert.ok(j.id && j.nombre && j.descripcion && j.unidad, `${j.id} incompleto`);
    assert.equal(typeof j.mejorSiMenor, 'boolean', `${j.id}: mejorSiMenor`);
    assert.equal(typeof j.crear, 'function');
    assert.equal(juegoPorId(j.id), j);
  }
});

for (const juego of JUEGOS) {
  test(`${juego.nombre}: una partida al azar termina y dibuja sin romperse`, () => {
    // Sintoniza exige mantener el cursor quieto dentro de una zona estrecha: un paseo
    // aleatorio tarda mucho más en lograrlo por azar que en acertar un tecleo suelto.
    const maxTics = juego.id === 'sintoniza' ? 2_000_000 : undefined;
    for (const semilla of [1, 2, 3]) {
      const { tics, partida, l } = simular(juego, semilla, { maxTics });
      // Un juego sin final (Blackjack) se sale con la tecla 10: aquí solo se comprueba que aguanta.
      if (juego.sinFin) assert.equal(partida.terminado, false, `${juego.id} no debería terminar solo`);
      else assert.ok(partida.terminado, `${juego.id} (semilla ${semilla}) no terminó en ${tics} tics`);
      assert.equal(l.length, PANTALLAS);
      for (const p of l) assert.equal(p.length, FB);
      assert.ok(Number.isFinite(partida.puntos), 'los puntos tienen que ser un número');
    }
  });

  test(`${juego.nombre}: dejar pasar el tiempo sin tocar nada también acaba`, () => {
    const aleatorio = aleatorioConSemilla(7);
    let reloj = 0;
    const partida = juego.crear({ ahora: () => reloj, aleatorio });
    const l = lienzo();
    let tics = 0;
    while (!partida.terminado && tics < 120_000) {
      reloj += DT;
      partida.tick(DT);
      limpiarLienzo(l);
      partida.dibujar(l);
      tics++;
    }
    // Sintoniza y Simon esperan al jugador para siempre; Topos acaba por tiempo.
    if (juego.id === 'topos') assert.ok(partida.terminado, `${juego.id} no acabó solo`);
  });
}

test('Topos: acertar suma, fallar resta y no baja de cero', () => {
  const juego = juegoPorId('topos');
  let reloj = 0;
  const partida = juego.crear({ ahora: () => reloj, aleatorio: aleatorioConSemilla(5) });
  const l = lienzo();
  const pulsar = (p) => partida.tecla({ tipo: 'tecla', tecla: p, pantalla: p, pulsada: true });
  assert.equal(partida.puntos, 0);
  // Se avanza hasta que se ve un topo y se pulsa en ese mismo instante: las pantallas sin
  // topo tienen todas los mismos píxeles (solo el agujero), y la del topo, más.
  const tinta = (fb) => fb.reduce((s, b) => s + b.toString(2).replace(/0/g, '').length, 0);
  let topo = null;
  for (let i = 0; i < 2000 && !topo; i++) {
    reloj += DT; partida.tick(DT); limpiarLienzo(l); partida.dibujar(l);
    const cuentas = [...Array(9).keys()].map((p) => tinta(l[p]));
    const base = Math.min(...cuentas);
    const m = cuentas.findIndex((c) => c > base + 8);
    if (m >= 0) topo = m + 1;
  }
  assert.ok(topo, 'debería haber salido un topo');
  pulsar(topo);
  assert.equal(partida.puntos, 1);
  // Pulsar una pantalla donde no hay topo resta uno, y no se baja de cero.
  pulsar(topo);
  assert.equal(partida.puntos, 0);
  pulsar(topo);
  assert.equal(partida.puntos, 0);
});

test('Simon: repetir la secuencia sube de ronda y fallar la termina', () => {
  const juego = juegoPorId('simon');
  let reloj = 0;
  const partida = juego.crear({ ahora: () => reloj, aleatorio: aleatorioConSemilla(11) });
  const l = lienzo();
  const avanzar = (ms) => { for (let t = 0; t < ms; t += DT) { reloj += DT; partida.tick(DT); limpiarLienzo(l); partida.dibujar(l); } };
  const pulsar = (p) => partida.tecla({ tipo: 'tecla', tecla: p, pantalla: p, pulsada: true });

  // Se ve la secuencia: la pantalla encendida (rellena) es la que toca. Se recoge
  // observando qué pantalla se enciende mientras dura la muestra.
  const vistas = [];
  for (let t = 0; t < 4000 && vistas.length < 1; t += DT) {
    avanzar(DT);
    // Página 3 (filas 24-31), centro: macizo en el pulsador encendido, hueco en el apagado.
    const llena = [...Array(9).keys()].find((i) => l[i][3 * 72 + 36] === 0xff);
    if (llena !== undefined) vistas.push(llena + 1);
  }
  assert.equal(vistas.length, 1, 'debería haber mostrado la primera tecla');
  avanzar(1500);              // termina la muestra y pasa a esperar al jugador
  pulsar(vistas[0]);
  assert.equal(partida.puntos, 1);
  avanzar(1200);              // pausa entre rondas
  // Ahora la secuencia tiene dos; se falla a propósito pulsando una que no es la primera.
  avanzar(4000);
  pulsar(vistas[0] === 9 ? 1 : vistas[0] + 1);
  const marcadorAntes = tinta(l[9]);
  avanzar(200);
  assert.equal(partida.puntos, 1);
  assert.equal(tinta(l[9]), marcadorAntes, 'fallar no puede cambiar el marcador: parecería que quita un punto');
});

test('Malla: la bola cruza el hueco sin dibujarse y cae en la pantalla que toca', () => {
  const l = lienzo();
  const m = new Malla(l, [[1, 2, 3]], 70, 0);
  assert.equal(m.ancho, 3 * W + 2 * 70);
  assert.equal(m.destino(0, 0).pantalla, 1);
  assert.equal(m.destino(W - 1, 0).pantalla, 1);
  assert.equal(m.destino(W, 0), null, 'el hueco no es de nadie');
  assert.equal(m.destino(W + 70, 0).pantalla, 2);
  assert.equal(m.destino(W + 70 + 5, 3).x, 5);
  assert.equal(m.destino(m.ancho, 0), null);
  m.px(W + 70 + 5, 0);
  assert.equal(l[1][5], 1);
  assert.equal(m.alto, H);
});

// --- Sintoniza: tiene que explicarse sola ----------------------------------------------

const tinta = (fb) => fb.reduce((s, b) => s + b.toString(2).replace(/0/g, '').length, 0);

function nuevaSintoniza(semilla = 9) {
  let reloj = 0;
  const partida = juegoPorId('sintoniza').crear({ ahora: () => reloj, aleatorio: aleatorioConSemilla(semilla) });
  const l = lienzo();
  const paso = (ms = DT) => { reloj += ms; partida.tick(ms); limpiarLienzo(l); partida.dibujar(l); };
  const girar = (delta) => partida.rueda({ tipo: 'rueda', delta, angulo: 0 });
  paso();
  return { partida, l, paso, girar };
}

// Gira hacia donde apunta la flecha hasta que las dos desaparecen: eso es estar dentro de
// la zona (la barra de sintonía, tecla 5, manda a partir de ahí).
function llevarADentro({ l, paso, girar }) {
  for (let i = 0; i < 400 && (tinta(l[3]) > 0 || tinta(l[5]) > 0); i++) {
    // Flecha en la 4 (índice 3) = zona a la izquierda. La rueda de Sintoniza va invertida
    // (SENTIDO), así que el giro positivo lleva el cursor hacia la izquierda.
    girar(tinta(l[3]) > 0 ? 40 : -40);
    paso();
  }
}

// Mantiene el cursor donde está (sin girar) hasta que se conquista la emisora o se agotan
// los tics: con DURACION_MS = 1200 y DT = 8, bastan unos 150.
function mantenerHastaConquistar({ partida, paso }, tics = 250) {
  const antes = partida.estado;
  for (let i = 0; i < tics && partida.estado === antes; i++) paso();
}

test('Sintoniza: al empezar escribe las instrucciones en la fila de abajo', () => {
  const { l } = nuevaSintoniza();
  for (const p of [6, 7, 8]) assert.ok(tinta(l[p]) > 0, `la pantalla ${p + 1} debería llevar una instrucción`);
});

test('Sintoniza: las instrucciones se quitan al girar y aparece la frecuencia', () => {
  const { l, girar, paso } = nuevaSintoniza();
  const antes = Array.from(l[6]);
  girar(10);
  paso();
  assert.notDeepEqual(Array.from(l[6]), antes, 'la pantalla 7 debería dejar de enseñar la instrucción');
  assert.ok(tinta(l[6]) > 0, 'la frecuencia debería verse en la pantalla 7');
  assert.ok(tinta(l[7]) > 0, 'el punto decimal debería verse en la pantalla 8');
});

test('Sintoniza: una flecha apunta hacia donde está la zona', () => {
  const { l } = nuevaSintoniza();
  const izquierda = tinta(l[3]) > 0;
  const derecha = tinta(l[5]) > 0;
  assert.ok(izquierda !== derecha, 'debe haber una flecha, y solo una');
});

test('Sintoniza: mantener el cursor dentro de la zona conquista la emisora sin pulsar nada', () => {
  const j = nuevaSintoniza();
  llevarADentro(j);
  assert.equal(j.partida.estado, 'Emisora 1 de 5');
  mantenerHastaConquistar(j);
  assert.equal(j.partida.estado, 'Emisora 2 de 5');
});

test('Sintoniza: la barra de sintonía se vacía desde el centro mientras se mantiene dentro', () => {
  const j = nuevaSintoniza();
  llevarADentro(j);
  const llena = tinta(j.l[4]);
  for (let i = 0; i < 50; i++) j.paso();   // dentro, pero sin llegar a conquistar
  assert.equal(j.partida.estado, 'Emisora 1 de 5', 'con 50 tics (400 ms) no debería haber conquistado ya');
  assert.ok(tinta(j.l[4]) < llena, 'con progreso la barra debería tener menos tinta (se vacía desde el centro)');
});

test('Sintoniza: salir de la zona reinicia la barra a llena', () => {
  const j = nuevaSintoniza();
  llevarADentro(j);
  const llena = tinta(j.l[4]);
  for (let i = 0; i < 50; i++) j.paso();
  assert.ok(tinta(j.l[4]) < llena, 'debería haberse vaciado algo antes de salir');
  j.girar(1_000_000);   // al límite de la barra, siempre fuera de la zona (hay margen)
  j.paso();
  assert.equal(tinta(j.l[4]), llena, 'al salir de la zona la barra debería volver a estar llena');
});

test('Sintoniza: cada emisora es más estrecha que la anterior', () => {
  const j = nuevaSintoniza();
  llevarADentro(j);
  mantenerHastaConquistar(j);
  assert.equal(j.partida.estado, 'Emisora 2 de 5');
  llevarADentro(j);
  const estrecha = (() => {
    // La zona rellena en las teclas 1-3 (la Malla) cuenta menos tinta cuanto más estrecha es.
    return [0, 1, 2].reduce((s, p) => s + tinta(j.l[p]), 0);
  })();
  const primera = nuevaSintoniza();
  llevarADentro(primera);
  const ancha = [0, 1, 2].reduce((s, p) => s + tinta(primera.l[p]), 0);
  assert.ok(estrecha < ancha, 'la zona de la segunda emisora debería ocupar menos píxeles que la primera');
});

// Conquista las cinco emisoras una tras otra: a cada una, busca la zona y se queda dentro
// hasta vaciar la barra.
function jugarPartidaCompleta(j) {
  for (let i = 0; i < 5; i++) {
    llevarADentro(j);
    mantenerHastaConquistar(j);
  }
}

test('Sintoniza: el tiempo no corre mientras se lee', () => {
  const j = nuevaSintoniza();
  for (let i = 0; i < 1000; i++) j.paso();       // 8 s sin tocar nada antes del primer giro
  jugarPartidaCompleta(j);
  assert.ok(j.partida.puntos < 10_000, `el reloj debería contar desde el primer giro, no desde que apareció (${j.partida.puntos} ms)`);
});

// --- Malla sin huecos (la que usa Sintoniza) ----------------------------------------------

test('Malla sin huecos: ningún punto cae fuera de una pantalla', () => {
  const l = lienzo();
  const m = new Malla(l, [[1, 2, 3], [4, 5, 6], [7, 8, 9]], 0, 0);
  assert.equal(m.ancho, 3 * W);
  assert.equal(m.alto, 3 * H);
  for (let x = 0; x < m.ancho; x++) assert.ok(m.destino(x, 5), `el punto x=${x} cae en un hueco`);
  for (let y = 0; y < m.alto; y++) assert.ok(m.destino(5, y), `el punto y=${y} cae en un hueco`);
});

// --- Reflejos: lo que importa es la medida ---------------------------------------------

// Una partida con el motor imitado: cada dibujar() sale como un fotograma y el teclado lo
// acusa con el instante `pintadoUs`. Devuelve la pantalla del objetivo en cuanto aparece.
function nuevaReflejos(semilla = 5) {
  const partida = juegoPorId('reflejos').crear({ aleatorio: aleatorioConSemilla(semilla) });
  const l = lienzo();
  let n = 0;
  const paso = (ms = 4) => { partida.tick(ms); limpiarLienzo(l); partida.dibujar(l); partida.enviado(n++); };
  // El objetivo es la única pantalla toda en blanco.
  const objetivo = () => [...Array(9).keys()].map((i) => i + 1).find((p) => l[p - 1].every((b) => b === 0xff)) ?? null;
  const esperarObjetivo = (pintadoUs) => {
    paso();   // el lienzo aún tiene el objetivo anterior: se redibuja antes de mirar
    for (let t = 0; t < 5000 && objetivo() === null; t += 4) paso();
    partida.acuse({ tipo: 'acuse', n: n - 1, us: pintadoUs });
    return objetivo();
  };
  const pulsar = (pantalla, us) => partida.tecla({ tipo: 'tecla', tecla: pantalla, pantalla, pulsada: true, us });
  return { partida, l, paso, objetivo, esperarObjetivo, pulsar };
}

test('Reflejos: empieza al pulsar la del centro, y no antes', () => {
  const j = nuevaReflejos();
  j.paso();
  j.pulsar(4, 1);
  for (let t = 0; t < 4000; t += 4) j.paso();
  assert.match(j.partida.estado, /para empezar/, 'otra tecla que no es la del centro no empieza');
  assert.equal(j.objetivo(), null);
  j.pulsar(5, 2);
  for (let t = 0; t < 400; t += 4) j.paso();
  assert.equal(j.objetivo(), null, 'tras empezar hay una espera antes del primer objetivo');
});

test('Reflejos: mide con el reloj del teclado, de pantalla pintada a pulsación', () => {
  const j = nuevaReflejos();
  j.paso();
  j.pulsar(5, 0);
  const p = j.esperarObjetivo(10_000_000);
  assert.ok([4, 5, 6].includes(p), 'el primer nivel es la fila del medio');
  // Lo que tarde el PC no cuenta: el reloj del juego avanza 900 ms y da igual.
  j.paso(900);
  j.pulsar(p, 10_287_000);
  assert.equal(j.partida.detalle.niveles[0], 287);
  assert.equal(j.partida.detalle.preciso, true);
});

test('Reflejos: una tecla equivocada suma 0,1 s, y pulsar antes de que salga también', () => {
  const j = nuevaReflejos();
  j.paso();
  j.pulsar(5, 0);
  j.paso(300);
  j.pulsar(4, 1);                                  // en la espera: castigo
  const p = j.esperarObjetivo(5_000_000);
  j.pulsar(p === 4 ? 6 : 4, 5_100_000);            // la que no es: castigo
  j.pulsar(p, 5_250_000);
  assert.equal(j.partida.detalle.niveles[0], 250 + 2 * 100);
});

test('Reflejos: tres niveles de tres, con 3, 6 y 9 teclas, y el resultado de cada uno', () => {
  const j = nuevaReflejos(21);
  j.paso();
  j.pulsar(5, 0);
  let us = 1_000_000;
  const vistas = [[], [], []];
  for (let k = 0; k < 9; k++) {
    const p = j.esperarObjetivo(us);
    vistas[Math.floor(k / 3)].push(p);
    j.pulsar(p, us + 200_000 + Math.floor(k / 3) * 100_000);
    us += 5_000_000;
  }
  assert.ok(vistas[0].every((p) => p >= 4 && p <= 6));
  assert.ok(vistas[1].every((p) => p >= 1 && p <= 6));
  assert.ok(vistas[2].every((p) => p >= 1 && p <= 9));
  assert.deepEqual(j.partida.detalle.niveles, [200, 300, 400]);
  assert.equal(j.partida.puntos, 300);
  for (let t = 0; t < 7000; t += 4) j.paso();
  assert.equal(j.partida.terminado, true);
});
