// Pestañas de perfil de la barra superior.
//
// Son el selector de perfil del Inicio y no de un editor: pulsar una cambia el
// perfil del teclado, igual que su menú. Por eso la resaltada es siempre la del
// perfil activo, y solo esa enseña sus páginas: el firmware edita y cambia de
// página sobre el perfil que tiene puesto (ver selectPage en store.js), así que
// los puntos de otro perfil no podrían hacer nada al pulsarlos.
//
// Vive aparte de dashboard.js porque la barra es del armazón: se queda pintada
// aunque la vista cambie, y solo se esconde fuera de Inicio (main.js).

import * as device from './device.js';
import { state, notify, subscribe, profileMeta, hasPages, pageCountOf, selectPage } from './store.js';
import * as cache from './oled-cache.js';
import { icon } from './icons.js';
import { toast, requireDevice } from './ui.js';

const escape = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

let host = null;
// Lo último que se pintó. La barra se repinta con cada notify() del store, que
// llega en cada tecla pulsada y en cada guardado: rehacer el HTML entero cada
// vez reiniciaba la animación de la píldora y hacía parpadear los iconos.
let firma = '';

export function init() {
  host = document.getElementById('profile-strip');
  if (!host) return;

  host.addEventListener('click', (e) => {
    const dot = e.target.closest('[data-page]');
    if (dot) {
      // El punto va dentro de la pestaña: sin esto el clic seguiría hasta ella.
      e.stopPropagation();
      choosePage(Number(dot.dataset.page));
      return;
    }
    const tab = e.target.closest('[data-profile]');
    if (tab) activateProfile(Number(tab.dataset.profile));
  });

  subscribe(render);
  // El icono de cada perfil llega por el puerto serie después del primer pintado.
  cache.onChange(render);
  window.addEventListener('resize', fit);
  render();
}

// Con ocho perfiles y la ventana estrecha no caben todos con su nombre: los
// que no están activos se quedan en el icono. Se mide en vez de poner un
// ancho fijo porque los nombres van de una letra a siete.
function fit() {
  if (!host || host.classList.contains('hidden')) return;
  host.classList.remove('compact');
  if (host.scrollWidth > host.clientWidth + 1) host.classList.add('compact');
}

function activateProfile(idx) {
  if (idx === state.activeProfileIdx || !requireDevice()) return;
  device.setProfile(idx)
    .then(() => {
      state.activeProfileIdx = idx;
      notify();
    })
    .catch(() => toast('El teclado no confirmó el cambio de perfil', 'error'));
}

async function choosePage(idx) {
  if (idx === state.pageIdx || !requireDevice()) return;
  try {
    await selectPage(idx);
    // Los iconos son de cada página: los de la nueva hay que leerlos.
    cache.loadProfile(state.activeProfileIdx);
  } catch (e) {
    toast(`No he podido cambiar de página: ${e.message}`, 'error');
  }
}

function pageDots(prof) {
  if (!hasPages()) return '';
  const count = pageCountOf(prof);
  if (count <= 1) return '';
  const cur = Math.min(state.pageIdx, count - 1);
  let dots = '';
  for (let i = 0; i < count; i++) {
    // <span> y no <button>: van dentro del botón de la pestaña, y un botón
    // dentro de otro no es HTML válido (el navegador los separa al parsear).
    dots += `<span class="strip-dot ${i === cur ? 'on' : ''}" role="button" data-page="${i}"
                   title="Página ${i + 1} de ${count}" aria-label="Página ${i + 1}"
                   aria-current="${i === cur}"></span>`;
  }
  return `<span class="strip-dots">${dots}</span>`;
}

function tabIcon(i) {
  // El icono de verdad del perfil (el que enseña su menú en el teclado) va en
  // una pantallita negra, como en el aparato. Sin él, la baldosa de color.
  if (cache.profileIcon(i)) {
    return `<span class="strip-icon is-oled"><canvas data-bmp="${i}:0:20"></canvas></span>`;
  }
  const meta = profileMeta(i);
  return `<span class="strip-icon" style="--accent:${meta.accent}">${icon(meta.icon, 14)}</span>`;
}

export function render() {
  if (!host) return;
  const prof = state.profiles[state.activeProfileIdx];
  const nueva = JSON.stringify([
    state.profiles.map((p, i) => [p.name, Boolean(cache.profileIcon(i))]),
    state.activeProfileIdx, state.pageIdx, prof ? pageCountOf(prof) : 1, hasPages(),
  ]);
  if (nueva === firma) {
    cache.paintThumbs(host);
    return;
  }
  firma = nueva;

  host.innerHTML = state.profiles.map((p, i) => {
    const on = i === state.activeProfileIdx;
    return `
      <button class="strip-tab ${on ? 'on' : ''}" role="tab" aria-selected="${on}" data-profile="${i}"
              title="${on ? `${escape(p.name)} · perfil activo` : `Cambiar el teclado a ${escape(p.name)}`}">
        ${tabIcon(i)}
        <span class="strip-name">${escape(p.name)}</span>
        ${on ? pageDots(p) : ''}
      </button>`;
  }).join('');

  // Los <canvas> no se pueden serializar en la plantilla: se pintan después.
  cache.paintThumbs(host);
  fit();
}

// main.js la enseña al volver al Inicio: mientras estaba escondida no se
// podía medir.
export function refit() {
  fit();
}
