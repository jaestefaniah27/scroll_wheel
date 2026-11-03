// Juegos: los que trae la app y tus estadísticas de cada uno.
//
// Los juegos corren aquí, en el PC; el teclado es solo las pantallas y los mandos (modo
// GAME del firmware 4.29, ver src/games/). Se juega desde el propio teclado —«Juegos» en
// su selector de perfiles abre el menú— o desde aquí; la vista no lleva la partida, solo
// la enseña: quien manda es games/gestor.js, que también atiende al teclado cuando esta
// vista ni está abierta. Hace falta el teclado delante para jugar, y la vista lo dice en
// vez de esconderse: sin él, las estadísticas se siguen viendo.
//
// No replica las pantallas del teclado: se juega mirándolas, y el espacio que ocupaba el
// dibujo es mejor para las estadísticas, que son lo que solo se puede ver aquí. Una lista
// de juegos a la izquierda y, del elegido, un panel grande con todo lo que se guarda.

import { state, subscribe as suscribirStore } from '../store.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';
import * as compat from '../compat.js';
import { JUEGOS } from '../games/catalogo.mjs';
import * as gestor from '../games/gestor.js';
import * as almacen from '../games/almacen.js';
import { resumen, serieGrafica, formatearPuntos, RECIENTES } from '../games/scores.mjs';

const escape = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

const CLAVE_SELECCION = 'orby.juegos.seleccion';

let host = null;
let relojEstado = null;
let seleccion = null;

function leerSeleccion() {
  try { return localStorage.getItem(CLAVE_SELECCION); } catch { return null; }
}

function guardarSeleccion(id) {
  try { localStorage.setItem(CLAVE_SELECCION, id); } catch { /* sin almacenamiento, no se recuerda */ }
}

const juegoElegido = () => JUEGOS.find((j) => j.id === seleccion) || JUEGOS[0];

// ¿Se puede jugar con lo que hay conectado? Devuelve el motivo si no.
function motivoNo() {
  if (!state.connected) return 'Conecta el teclado para jugar: los juegos usan sus pantallas y sus mandos.';
  if (!compat.supports(state.deviceInfo, 'games')) {
    return `Tu firmware (${compat.fwOf(state.deviceInfo)}) no trae el modo juego. Actualízalo a la ${compat.FEATURES.games.since} o posterior desde Ajustes.`;
  }
  return null;
}

const fecha = (t) => new Date(t).toLocaleDateString('es-ES', { day: 'numeric', month: 'short' });
const entero = (n) => Math.round(n || 0).toLocaleString('es-ES');

function grafica(juego, datos) {
  const ancho = 600, alto = 120;
  const historial = datos?.historial || [];
  const pts = serieGrafica(historial, ancho, alto, { mejorSiMenor: juego.mejorSiMenor, margen: 8 });
  if (pts.length < 2) return '<p class="game-none">La gráfica aparece a partir de la segunda partida.</p>';
  const linea = pts.map(([x, y]) => `${x},${y}`).join(' ');
  const [ux, uy] = pts[pts.length - 1];
  const valores = historial.map((x) => x.p);
  const f = (v) => formatearPuntos(juego.unidad, v);
  return `
    <div class="game-chart-wrap">
      <svg class="game-chart" viewBox="0 0 ${ancho} ${alto}" preserveAspectRatio="none" aria-hidden="true">
        <polyline points="${linea}" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round" stroke-linecap="round" vector-effect="non-scaling-stroke"/>
        <circle cx="${ux}" cy="${uy}" r="3.2" fill="currentColor"/>
      </svg>
      <div class="game-chart-axis"><span>${escape(f(juego.mejorSiMenor ? Math.min(...valores) : Math.max(...valores)))}</span><span>${escape(f(juego.mejorSiMenor ? Math.max(...valores) : Math.min(...valores)))}</span></div>
    </div>`;
}

function baldosa(titulo, valor, nota = '') {
  return `<div class="game-tile"><dt>${escape(titulo)}</dt><dd>${escape(valor)}</dd>${nota ? `<small>${escape(nota)}</small>` : ''}</div>`;
}

// Por cada parte del juego (Reflejos: cada nivel): lo de la última partida y lo mejor. No hay
// medias de varias partidas: cada una vale por sí misma.
function desglose(juego, r, f) {
  if (!juego.detalle || !r.detalle.some((d) => d.ultima != null || d.mejor != null)) return '';
  const filas = juego.detalle.map((nombre, i) => {
    const d = r.detalle[i] || {};
    return `<tr><th scope="row">${escape(nombre)}</th><td>${escape(f(d.ultima))}</td><td>${escape(f(d.mejor))}</td></tr>`;
  }).join('');
  return `
    <section class="game-block">
      <h4>Por nivel</h4>
      <table class="game-detail">
        <thead><tr><th></th><th>Última partida</th><th>Mejor</th></tr></thead>
        <tbody>${filas}
          <tr class="game-total"><th scope="row">Media de la partida</th><td>${escape(f(r.ultima))}</td><td>${escape(f(r.mejor))}</td></tr>
        </tbody>
      </table>
    </section>`;
}

// Lo que el juego cuenta además de los puntos (juego.estadisticas, ver scores.mjs): sumas de
// siempre como barras proporcionales, y máximos como cifras. Con la última partida al lado.
function estadisticas(juego, r) {
  const def = juego.estadisticas;
  if (!def || !r.totales) return '';
  const suma = r.totales.suma || {};
  const ultima = r.ultimaStats?.suma || {};
  const tipos = def.suma.filter(([k]) => k !== 'total');
  const tope = Math.max(1, ...tipos.map(([k]) => suma[k] || 0));
  const barras = tipos.map(([k, etiqueta]) => {
    const n = suma[k] || 0;
    const ult = ultima[k] || 0;
    return `
      <li>
        <span class="game-bar-name">${escape(etiqueta)}</span>
        <span class="game-bar"><i style="width:${Math.round((n / tope) * 100)}%"></i></span>
        <span class="game-bar-num">${entero(n)}</span>
        <span class="game-bar-last" title="En la última partida">${ult ? `+${entero(ult)}` : ''}</span>
      </li>`;
  }).join('');
  const totalDef = def.suma.find(([k]) => k === 'total');
  const maximos = (def.max || []).map(([k, etiqueta]) =>
    baldosa(etiqueta, entero(r.totales.max?.[k]), r.ultimaStats ? `Última partida: ${entero(r.ultimaStats.max?.[k])}` : ''));
  return `
    <section class="game-block">
      <h4>Objetivos · totales de todas las partidas</h4>
      <dl class="game-tiles">
        ${totalDef ? baldosa(totalDef[1], entero(suma.total), r.ultimaStats ? `Última partida: ${entero(ultima.total)}` : '') : ''}
        ${maximos.join('')}
      </dl>
      <ul class="game-bars" aria-label="Objetivos destruidos por tipo">${barras}</ul>
    </section>`;
}

// Las últimas partidas una a una. En un juego con estadísticas cada una se abre y enseña lo suyo
// (objetivos de cada tipo y el máximo con un solo misil), aparte de los totales de arriba.
function partidasDetalladas(juego, r, f) {
  const def = juego.estadisticas;
  const filas = r.recientes.map((x) => {
    const suma = x.s?.suma || {};
    const max = x.s?.max || {};
    const tipos = def.suma.filter(([k]) => k !== 'total' && suma[k]).map(([k, et]) => `<li><span>${escape(et)}</span><b>${entero(suma[k])}</b></li>`).join('');
    const maximos = def.max.map(([k, et]) => `<li><span>${escape(et)}</span><b>${entero(max[k])}</b></li>`).join('');
    const cuerpo = x.s ? `<ul class="game-run-detail">${tipos || '<li><span>Ningún objetivo</span><b>0</b></li>'}${maximos}</ul>` : '<p class="game-none">Sin desglose: es anterior a las estadísticas.</p>';
    return `
      <details class="game-run">
        <summary><span>${escape(fecha(x.t))}</span><span>${entero(suma.total)} objetivos</span><b>${escape(f(x.p))}</b></summary>
        ${cuerpo}
      </details>`;
  }).join('');
  return `<div class="game-runs" aria-label="Últimas ${RECIENTES} partidas">${filas}</div>`;
}

function recientes(juego, r, f) {
  if (!r.recientes.length) return '<p class="game-none">Sin partidas todavía.</p>';
  if (juego.estadisticas) return partidasDetalladas(juego, r, f);
  const filas = r.recientes.map((x) => `<li><span>${escape(fecha(x.t))}</span><b>${escape(f(x.p))}</b></li>`).join('');
  return `<ul class="game-recent" aria-label="Últimas ${RECIENTES} partidas">${filas}</ul>`;
}

function panelEnVivo(juego) {
  const e = gestor.leerEstado();
  if (!e.activo) return '';
  return `
    <section class="game-live" id="game-live">
      <div class="game-live-info">
        <span class="game-live-tag">${escape(e.titulo || 'Juegos')}</span>
        <p id="game-live-estado">${escape(e.partida?.estado ?? 'Preparando…')}</p>
        <small>Se juega en el teclado. La tecla 10 (la de menú) vuelve atrás o sale.</small>
      </div>
      <button class="secondary-btn danger" data-act="parar">${icon('close', 15)} Cerrar juegos</button>
    </section>`;
}

function lista(motivo) {
  const e = gestor.leerEstado();
  return JUEGOS.map((j) => {
    const r = resumen(almacen.leer(j.id));
    const f = (v) => formatearPuntos(j.unidad, v);
    const jugando = e.activo && e.juego === j.id;
    return `
      <button class="game-item ${j.id === juegoElegido().id ? 'is-selected' : ''} ${jugando ? 'is-playing' : ''}" data-act="elegir" data-id="${j.id}">
        <span class="game-item-name">${escape(j.nombre)}${jugando ? ' <em>jugando</em>' : ''}</span>
        <span class="game-item-meta">${r.partidas ? `Récord ${escape(f(r.mejor))} · ${r.partidas} ${r.partidas === 1 ? 'partida' : 'partidas'}` : 'Sin partidas'}</span>
      </button>`;
  }).join('');
}

function detalle(juego, motivo) {
  const datos = almacen.leer(juego.id);
  const r = resumen(datos);
  const f = (v) => formatearPuntos(juego.unidad, v);
  const e = gestor.leerEstado();
  const jugando = e.activo && e.juego === juego.id;
  return `
    <article class="game-pane glass-panel" data-juego="${juego.id}">
      <header class="game-pane-head">
        <div>
          <h3>${escape(juego.nombre)}</h3>
          <p>${escape(juego.descripcion)}</p>
        </div>
        <button class="primary-btn game-play" data-act="jugar" data-id="${juego.id}" ${motivo || e.activo ? 'disabled' : ''}>
          ${icon('gamepad', 16)} ${jugando ? 'Jugando…' : 'Jugar'}
        </button>
      </header>
      ${panelEnVivo(juego)}
      <dl class="game-tiles game-tiles-main">
        ${baldosa('Récord', f(r.mejor))}
        ${baldosa('Última', f(r.ultima))}
        ${baldosa('Partidas', String(r.partidas))}
      </dl>
      ${desglose(juego, r, f)}
      ${estadisticas(juego, r)}
      <section class="game-block">
        <h4>Evolución</h4>
        ${grafica(juego, datos)}
      </section>
      <section class="game-block">
        <h4>Últimas partidas${juego.estadisticas ? ' · pulsa una para ver su desglose' : ''}</h4>
        ${recientes(juego, r, f)}
      </section>
    </article>`;
}

export function init() {
  host = document.getElementById('view-games');
  if (!host) return;
  seleccion = leerSeleccion();

  host.addEventListener('click', (e) => {
    const b = e.target.closest('[data-act]');
    if (!b) return;
    if (b.dataset.act === 'jugar') empezar(b.dataset.id);
    else if (b.dataset.act === 'parar') gestor.parar();
    else if (b.dataset.act === 'elegir') { seleccion = b.dataset.id; guardarSeleccion(seleccion); render(); }
  });

  // El teclado se conecta o se va: la vista tiene que reflejarlo (los botones de Jugar).
  suscribirStore(() => { if (visible() && !gestor.activo()) render(); });

  gestor.suscribir(alGestor);
  // A partir de aquí la app atiende los juegos que pida el teclado, esté esta vista
  // abierta o no.
  gestor.escuchar();
}

const visible = () => host?.classList.contains('active');

function alGestor(tipo, datos) {
  if (tipo === 'error') { toast(String(datos), 'error'); return; }
  if (tipo === 'partida') {
    const f = (v) => formatearPuntos(datos.juego.unidad, v);
    toast(datos.record ? `¡Récord! ${f(datos.puntos)}` : `${datos.juego.nombre}: ${f(datos.puntos)}`, 'success', 3600);
    // Las estadísticas se acaban de guardar: si la vista está abierta, se ven al momento.
    if (visible()) render();
    return;
  }
  // 'estado': empieza o acaba una sesión, o cambia de pantalla del teclado.
  if (gestor.activo() && !relojEstado) {
    relojEstado = setInterval(() => {
      const el = document.getElementById('game-live-estado');
      const p = gestor.leerEstado().partida;
      if (el && p) el.textContent = p.estado ?? '';
    }, 250);
  } else if (!gestor.activo() && relojEstado) {
    clearInterval(relojEstado);
    relojEstado = null;
  }
  // Si se juega desde el teclado, se enseña el juego que se está jugando.
  const jugando = gestor.leerEstado().juego;
  if (gestor.activo() && jugando && jugando !== seleccion) seleccion = jugando;
  if (visible()) render();
}

export async function render() {
  if (!host) return;
  await almacen.cargar();
  const motivo = motivoNo();
  const jugando = gestor.activo();
  host.innerHTML = `
    <div class="games-layout">
      <header class="pane-head">
        <h2>Juegos</h2>
        <p>Corren en el PC y se juegan en el teclado: sus pantallas son el tablero y los botones y la rueda, los mandos.
           Para jugar sin la app delante, elige «Juegos» en el selector de perfiles del teclado.</p>
      </header>
      ${motivo && !jugando ? `<p class="games-need glass-panel">${icon('info', 16)} ${escape(motivo)}</p>` : ''}
      <div class="games-split">
        <nav class="games-list" aria-label="Juegos">${lista(motivo)}</nav>
        ${detalle(juegoElegido(), motivo)}
      </div>
    </div>`;
}

async function empezar(id) {
  if (gestor.activo() || motivoNo()) return;
  try {
    await gestor.iniciar({ juegoId: id });
  } catch (err) {
    // El teclado no ha dejado entrar: sin conexión, o con un menú abierto.
    const msg = String(err?.message || err);
    toast(msg.includes('GAME_BUSY') ? 'Cierra el menú del teclado antes de jugar' : msg, 'error');
  }
}
