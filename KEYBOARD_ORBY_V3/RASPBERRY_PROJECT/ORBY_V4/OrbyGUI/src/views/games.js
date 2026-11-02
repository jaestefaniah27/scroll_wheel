// Juegos: los que trae la app, con tu récord, tu última partida y las recientes.
//
// Los juegos corren aquí, en el PC; el teclado es solo las pantallas y los mandos (modo
// GAME del firmware 4.29, ver src/games/). Se juega desde el propio teclado —«Juegos» en
// su selector de perfiles abre el menú— o desde aquí; la vista no lleva la partida, solo
// la enseña: quien manda es games/gestor.js, que también atiende al teclado cuando esta
// vista ni está abierta. Hace falta el teclado delante para jugar, y la vista lo dice en
// vez de esconderse: sin él, los récords se siguen viendo.

import { state, subscribe as suscribirStore } from '../store.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';
import * as compat from '../compat.js';
import { JUEGOS } from '../games/catalogo.mjs';
import * as gestor from '../games/gestor.js';
import * as almacen from '../games/almacen.js';
import { resumen, serieGrafica, formatearPuntos } from '../games/scores.mjs';

const escape = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

let host = null;
let relojEstado = null;
let pintando = false;

// El teclado dibujado con sus diez pantallas: la rejilla de 3x3 y la décima debajo de la
// del medio, que es donde está en el aparato.
const POSICION = {
  1: [0, 0], 2: [1, 0], 3: [2, 0],
  4: [0, 1], 5: [1, 1], 6: [2, 1],
  7: [0, 2], 8: [1, 2], 9: [2, 2],
  10: [1, 3],
};
const W = 72, H = 40, SEP = 8;

// ¿Se puede jugar con lo que hay conectado? Devuelve el motivo si no.
function motivoNo() {
  if (!state.connected) return 'Conecta el teclado para jugar: los juegos usan sus pantallas y sus mandos.';
  if (!compat.supports(state.deviceInfo, 'games')) {
    return `Tu firmware (${compat.fwOf(state.deviceInfo)}) no trae el modo juego. Actualízalo a la ${compat.FEATURES.games.since} o posterior desde Ajustes.`;
  }
  return null;
}

function grafica(juego, datos) {
  const pts = serieGrafica(datos?.historial, 120, 34, { mejorSiMenor: juego.mejorSiMenor });
  if (pts.length < 2) return '';
  const linea = pts.map(([x, y]) => `${x},${y}`).join(' ');
  const [ux, uy] = pts[pts.length - 1];
  return `
    <svg class="game-chart" viewBox="0 0 120 34" preserveAspectRatio="none" aria-hidden="true">
      <polyline points="${linea}" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linejoin="round" stroke-linecap="round" vector-effect="non-scaling-stroke"/>
      <circle cx="${ux}" cy="${uy}" r="2.4" fill="currentColor"/>
    </svg>`;
}

// El desglose de los juegos que lo tienen (Reflejos: cada nivel por separado). Sin
// partidas que lo traigan no se enseña: una tabla de rayas no dice nada.
function desglose(juego, r, f) {
  if (!juego.detalle || !r.detalle.some((d) => d.media != null)) return '';
  const filas = juego.detalle.map((nombre, i) => {
    const d = r.detalle[i] || {};
    return `<tr><th scope="row">${escape(nombre)}</th><td>${escape(f(d.media))}</td><td>${escape(f(d.mejor))}</td><td>${escape(f(d.ultima))}</td></tr>`;
  }).join('');
  return `
      <table class="game-detail">
        <thead><tr><th></th><th>Media</th><th>Mejor</th><th>Última</th></tr></thead>
        <tbody>${filas}</tbody>
      </table>`;
}

function tarjeta(juego, motivo) {
  const datos = almacen.leer(juego.id);
  const r = resumen(datos);
  const f = (v) => formatearPuntos(juego.unidad, v);
  const e = gestor.leerEstado();
  const jugando = e.activo && e.juego === juego.id;
  const recientes = r.recientes.length
    ? r.recientes.map((x) => `<li>${escape(f(x.p))}</li>`).join('')
    : '<li class="game-none">Sin partidas todavía</li>';
  return `
    <article class="game-card glass-panel ${jugando ? 'is-playing' : ''}" data-juego="${juego.id}">
      <header class="game-head">
        <h3>${escape(juego.nombre)}</h3>
        <p>${escape(juego.descripcion)}</p>
      </header>
      <dl class="game-stats">
        <div><dt>Récord</dt><dd>${escape(f(r.mejor))}</dd></div>
        <div><dt>Última</dt><dd>${escape(f(r.ultima))}</dd></div>
        <div><dt>Media</dt><dd>${escape(f(r.media))}</dd></div>
        <div><dt>Partidas</dt><dd>${r.partidas}</dd></div>
      </dl>
      ${desglose(juego, r, f)}
      ${grafica(juego, datos)}
      <ul class="game-recent" aria-label="Partidas recientes">${recientes}</ul>
      <button class="primary-btn game-play" data-act="jugar" data-id="${juego.id}"
              ${motivo || e.activo ? 'disabled' : ''}>
        ${icon('gamepad', 16)} ${jugando ? 'Jugando…' : 'Jugar'}
      </button>
    </article>`;
}

function panelEnVivo() {
  const e = gestor.leerEstado();
  if (!e.activo) return '';
  return `
    <section class="game-live glass-panel" id="game-live">
      <div class="game-live-info">
        <h2>${escape(e.titulo || 'Juegos')}</h2>
        <p id="game-live-estado">${escape(e.partida?.estado ?? 'Preparando…')}</p>
        <p class="game-live-hint">Se juega en el teclado. La tecla 10 (la de menú) vuelve atrás o sale.</p>
        <button class="secondary-btn danger" data-act="parar">${icon('close', 15)} Cerrar juegos</button>
      </div>
      <canvas class="game-live-canvas" id="game-live-canvas"
              width="${3 * W + 2 * SEP}" height="${4 * H + 3 * SEP}"></canvas>
    </section>`;
}

export function init() {
  host = document.getElementById('view-games');
  if (!host) return;

  host.addEventListener('click', (e) => {
    const b = e.target.closest('[data-act]');
    if (!b) return;
    if (b.dataset.act === 'jugar') empezar(b.dataset.id);
    else if (b.dataset.act === 'parar') gestor.parar();
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
  if (tipo === 'fotograma') { pedirPintado(); return; }
  if (tipo === 'error') { toast(String(datos), 'error'); return; }
  if (tipo === 'partida') {
    const f = (v) => formatearPuntos(datos.juego.unidad, v);
    toast(datos.record ? `¡Récord! ${f(datos.puntos)}` : `${datos.juego.nombre}: ${f(datos.puntos)}`, 'success', 3600);
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
      ${panelEnVivo()}
      <div class="games-grid">${JUEGOS.map((j) => tarjeta(j, motivo)).join('')}</div>
    </div>`;
  pedirPintado();
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

// --- Vista en vivo de las diez pantallas ---------------------------------------------
// Un repintado por cuadro de animación como mucho: el motor puede mandar cien fotogramas
// por segundo y el canvas no necesita más de los que muestra el monitor.
function pedirPintado() {
  if (pintando) return;
  pintando = true;
  requestAnimationFrame(pintar);
}

function pintar() {
  pintando = false;
  const canvas = document.getElementById('game-live-canvas');
  const lienzo = gestor.leerEstado().ultimo;
  if (!canvas || !lienzo) return;
  const ctx = canvas.getContext('2d');
  const color = getComputedStyle(document.documentElement).getPropertyValue('--primary').trim() || '#6fe3ff';
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = color;
  for (let p = 1; p <= 10; p++) {
    const [cx, cy] = POSICION[p];
    const ox = cx * (W + SEP), oy = cy * (H + SEP);
    const fb = lienzo[p - 1];
    for (let x = 0; x < W; x++) {
      for (let pagina = 0; pagina < 5; pagina++) {
        const byte = fb[pagina * W + x];
        if (!byte) continue;
        for (let bit = 0; bit < 8; bit++) {
          if ((byte >> bit) & 1) ctx.fillRect(ox + x, oy + pagina * 8 + bit, 1, 1);
        }
      }
    }
  }
}
