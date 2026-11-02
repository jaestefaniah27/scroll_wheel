// Consejos: lo que el teclado y la app saben hacer y no se descubre solo.
//
// Cada consejo lleva un vídeo corto (se baja una vez, ver src-tauri/src/tips.rs) y, si
// corresponde, un botón que lleva a donde se activa o configura eso mismo. Los vídeos se
// cargan al acercarse a la tarjeta, no todos de golpe: pesan poco, pero son diecinueve.

import { state } from '../store.js';
import { icon } from '../icons.js';
import { goTo } from '../nav.js';
import * as compat from '../compat.js';
import { TIPS, CATEGORIAS } from '../tips.mjs';
import * as tipsState from '../tips-state.js';

const escape = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

let host = null;
let vistos = [];
let observador = null;
// Lo que ya se pidió al backend: render() corre con cada cambio de vista y volver a leer
// un vídeo de disco cada vez sería trabajo tirado.
const fuentes = new Map();

// Sin teclado conectado no se sabe qué firmware tiene: se enseña todo como disponible
// y la tarjeta se atenúa solo cuando hay un firmware concreto que no lo trae.
function disponible(t) {
  if (!t.requiere || !state.deviceInfo) return true;
  return compat.supports(state.deviceInfo, t.requiere);
}

function tarjeta(t) {
  const ok = disponible(t);
  const since = compat.FEATURES[t.requiere]?.since;
  const visto = vistos.includes(t.id);
  const boton = t.accion && ok
    ? `<button class="secondary-btn tip-go" data-act="tip-go" data-id="${t.id}">${escape(t.accion.etiqueta)} ${icon('chevron', 14)}</button>`
    : '';
  return `
    <article class="tip-card glass-panel ${ok ? '' : 'is-off'}" id="tip-${t.id}" data-id="${t.id}">
      <div class="tip-media" data-video="${t.id}">
        <span class="tip-media-empty">${icon('record', 18)} Vídeo próximamente</span>
      </div>
      <div class="tip-body">
        <h3>${escape(t.titulo)}${visto ? '' : ' <span class="tip-new">Nuevo</span>'}</h3>
        <p>${escape(t.texto)}</p>
        ${ok ? boton : `<p class="tip-need">Necesita el firmware ${escape(since || '')} o posterior.</p>`}
      </div>
    </article>`;
}

async function cargarVideo(caja) {
  const id = caja.dataset.video;
  if (!fuentes.has(id)) {
    fuentes.set(id, window.orby.tips.videoSrc(id).catch(() => null));
  }
  const src = await fuentes.get(id);
  if (!src || caja.querySelector('video')) return;
  const v = document.createElement('video');
  v.muted = true;
  v.loop = true;
  v.autoplay = true;
  v.playsInline = true;
  v.preload = 'metadata';
  // Un 404 (aún sin grabar) o un fichero ilegible: se queda el hueco.
  v.addEventListener('error', () => v.remove());
  v.addEventListener('loadeddata', () => caja.classList.add('has-video'));
  v.src = src;
  caja.prepend(v);
}

export function init() {
  host = document.getElementById('view-tips');
  if (!host) return;

  host.addEventListener('click', (e) => {
    const b = e.target.closest('[data-act="tip-go"]');
    if (!b) return;
    const t = TIPS.find((x) => x.id === b.dataset.id);
    if (t?.accion) goTo(t.accion.vista, t.accion.params);
  });

  observador = new IntersectionObserver((entradas) => {
    for (const en of entradas) {
      if (!en.isIntersecting) continue;
      observador.unobserve(en.target);
      cargarVideo(en.target);
    }
  }, { rootMargin: '200px' });
}

export async function render() {
  if (!host) return;
  vistos = (await tipsState.leer()).vistos;

  host.innerHTML = `
    <div class="tips-layout">
      <header class="pane-head"><h2>Consejos</h2><p>Lo que puede hacer el teclado y la app, y que no se descubre solo.</p></header>
      ${CATEGORIAS.map((cat) => {
        const lista = TIPS.filter((t) => t.categoria === cat);
        return lista.length
          ? `<section class="tips-group"><h2 class="tips-cat">${escape(cat)}</h2><div class="tips-grid">${lista.map(tarjeta).join('')}</div></section>`
          : '';
      }).join('')}
    </div>`;

  observador?.disconnect();
  host.querySelectorAll('[data-video]').forEach((caja) => observador?.observe(caja));
}

// Desde el aviso: abre la vista en la tarjeta de ese consejo y lo da por visto.
export async function openTarget({ id = null } = {}) {
  await tipsState.marcarVisto(id).catch(() => {});
  await render();
  const el = id && document.getElementById(`tip-${id}`);
  if (!el) return;
  el.scrollIntoView({ block: 'center', behavior: 'smooth' });
  el.classList.remove('tip-flash');
  void el.offsetWidth;
  el.classList.add('tip-flash');
}
