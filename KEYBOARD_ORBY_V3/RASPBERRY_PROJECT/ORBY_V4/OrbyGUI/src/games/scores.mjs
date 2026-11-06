// Puntuaciones de los juegos: qué se guarda de cada partida y cómo se resume.
//
// Sin acceso a disco ni a window.orby: el estado entra y sale como un objeto, y quien
// lo guarda (views/games.js, en la configuración local del PC) lo trata como un valor.
// Así se puede probar entero.
//
// Por cada juego:  { mejor, partidas, historial: [{ t, p, d?, s? }], mejorDetalle?, totales? }
//   mejor      la mejor puntuación de siempre (la mayor o la menor, según el juego)
//   partidas   cuántas se han jugado, aunque el historial ya no las guarde todas
//   historial  las últimas MAX_HISTORIAL, la más reciente al final; `t` es la fecha en
//              milisegundos y `p` la puntuación; `d`, si el juego lo da, el desglose de
//              esa partida (Reflejos: la media de cada nivel, en el orden de juego.detalle)
//   mejorDetalle  lo mejor de siempre de cada parte del desglose, por separado: el
//              historial se recorta y un récord de un nivel no se debe perder con él
//   totales    lo que el juego cuenta además de los puntos —en Misil, objetivos destruidos
//              de cada tipo y el máximo con un solo misil—: { suma: { clave: n }, max: { clave: n } }.
//              La suma se acumula entre partidas y el máximo se queda con el mayor; `s`, en
//              cada entrada del historial, es lo mismo de esa partida sola
//
// No se guardan medias de varias partidas: cada partida vale por sí misma (su resultado y su
// desglose por nivel), y de ellas solo se conserva el mejor.

export const MAX_HISTORIAL = 30;

// Cuántas partidas recientes enseña la vista.
export const RECIENTES = 8;

export function vacio() {
  return { mejor: null, partidas: 0, historial: [] };
}

// `mejorSiMenor`: en un juego de tiempos (reflejos) gana quien tarda menos.
// Junta lo que cuenta una partida (`stats`: { suma, max }) con lo de siempre.
function juntarTotales(previo, stats) {
  const suma = { ...(previo?.suma || {}) };
  const max = { ...(previo?.max || {}) };
  for (const [k, v] of Object.entries(stats.suma || {})) suma[k] = (suma[k] || 0) + v;
  for (const [k, v] of Object.entries(stats.max || {})) max[k] = Math.max(max[k] || 0, v);
  return { suma, max };
}

export function registrar(estado, puntos, { mejorSiMenor = false, ahora = Date.now(), detalle = null, stats = null } = {}) {
  const previo = estado && typeof estado === 'object' ? estado : vacio();
  const entrada = { t: ahora, p: puntos };
  if (Array.isArray(detalle)) entrada.d = detalle;
  if (stats) entrada.s = { suma: stats.suma || {}, max: stats.max || {} };
  const historial = [...(previo.historial || []), entrada].slice(-MAX_HISTORIAL);
  const elegir = (a, b) => (a == null ? b : b == null ? a : mejorSiMenor ? Math.min(a, b) : Math.max(a, b));
  const mejor = elegir(previo.mejor, puntos);
  const extra = {};
  if (Array.isArray(detalle)) {
    const antes = previo.mejorDetalle || [];
    extra.mejorDetalle = detalle.map((v, i) => elegir(antes[i] ?? null, v));
  } else if (previo.mejorDetalle) {
    extra.mejorDetalle = previo.mejorDetalle;
  }
  if (stats) extra.totales = juntarTotales(previo.totales, stats);
  else if (previo.totales) extra.totales = previo.totales;
  if (previo.saldo != null) extra.saldo = previo.saldo;
  return {
    mejor,
    partidas: (previo.partidas || 0) + 1,
    historial,
    ...extra,
    // Lo dice quien registra para que la vista no tenga que adivinar si la última fue
    // un récord: se compara aquí, con el valor anterior a la partida.
    record: previo.mejor == null || mejor !== previo.mejor,
  };
}

// El estado que se guarda no lleva `record`: es un dato de esa partida, no del juego.
export function paraGuardar({ mejor, partidas, historial, mejorDetalle, totales, saldo }) {
  const base = { mejor, partidas, historial };
  if (mejorDetalle) base.mejorDetalle = mejorDetalle;
  if (totales) base.totales = totales;
  if (saldo != null) base.saldo = saldo;
  return base;
}

// Para un juego que arrastra un saldo de una partida a otra (Blackjack): guarda el saldo y
// sube el récord si `pico` lo supera, sin contar una partida nueva ni tocar el historial.
// Se llama tras cada mano; la partida se anota al salir (con `registrar`).
export function conSaldo(estado, saldo, pico, { mejorSiMenor = false } = {}) {
  const previo = estado && typeof estado === 'object' ? estado : vacio();
  const mejor = previo.mejor == null ? pico : mejorSiMenor ? Math.min(previo.mejor, pico) : Math.max(previo.mejor, pico);
  return { ...previo, mejor, saldo };
}

export function resumen(estado) {
  const e = estado && typeof estado === 'object' ? estado : vacio();
  const historial = e.historial || [];
  const recientes = historial.slice(-RECIENTES).reverse();
  // El desglose: por cada parte, lo de la última partida y lo mejor de siempre. Es lo que
  // deja ver en qué parte se es mejor (en Reflejos, con cuántas teclas).
  const conD = historial.filter((x) => Array.isArray(x.d));
  const partes = Math.max(0, ...conD.map((x) => x.d.length), (e.mejorDetalle || []).length);
  const detalle = Array.from({ length: partes }, (_, i) => ({
    ultima: conD.length ? conD[conD.length - 1].d[i] ?? null : null,
    mejor: e.mejorDetalle?.[i] ?? null,
  }));
  const ultimas = historial.length ? historial[historial.length - 1] : null;
  return {
    mejor: e.mejor ?? null,
    partidas: e.partidas || 0,
    ultima: ultimas ? ultimas.p : null,
    recientes,
    detalle,
    totales: e.totales || null,
    ultimaStats: ultimas?.s || null,
  };
}

// Los puntos de una gráfica de las últimas partidas, ya en coordenadas de un SVG de
// `ancho` x `alto`: la partida más antigua a la izquierda, y arriba lo mejor (que es lo
// menor en un juego de tiempos). Con una sola partida no hay línea que trazar y queda un
// punto centrado. Sin partidas, lista vacía.
export function serieGrafica(historial, ancho, alto, { mejorSiMenor = false, margen = 3 } = {}) {
  const h = historial || [];
  if (!h.length) return [];
  const valores = h.map((x) => x.p);
  const min = Math.min(...valores);
  const max = Math.max(...valores);
  const util = alto - 2 * margen;
  return h.map((x, i) => {
    const px = h.length === 1 ? ancho / 2 : margen + (i * (ancho - 2 * margen)) / (h.length - 1);
    // Todas iguales: una línea recta a media altura en vez de dividir entre cero.
    const t = max === min ? 0.5 : (x.p - min) / (max - min);
    const py = margen + (mejorSiMenor ? t : 1 - t) * util;
    return [Math.round(px * 10) / 10, Math.round(py * 10) / 10];
  });
}

// Una puntuación como se enseña. `corto` es para las pantallas del teclado, donde cada
// carácter cuenta: sin el espacio antes de la unidad.
export function formatearPuntos(unidad, valor, { corto = false } = {}) {
  if (valor == null) return corto ? '-' : '—';
  // Tiempos de reacción: aquí cuentan los milisegundos y una décima de segundo no dice nada.
  if (unidad === 'reaccion') return corto ? `${Math.round(valor)}ms` : `${Math.round(valor)} ms`;
  if (unidad === 'ms') {
    const s = (valor / 1000).toFixed(1);
    return corto ? `${s}s` : `${s} s`;
  }
  return String(Math.round(valor));
}
