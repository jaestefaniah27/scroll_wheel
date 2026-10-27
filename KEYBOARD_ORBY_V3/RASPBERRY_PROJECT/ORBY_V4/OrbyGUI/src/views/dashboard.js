// Vista Dashboard: réplica del hardware con telemetría en vivo.
//
// La réplica se coloca con las coordenadas del prototipo, en milímetros y con
// el origen en el centro de la carcasa: son las mismas que usa el modelo de la
// web del producto (orvy.es). Doce teclas en 3x4 a 23 mm de paso a la
// izquierda, la rueda de 76 mm a la derecha y los dos mandos en el borde de
// arriba: el primero en el centro de la carcasa y el segundo en la esquina.

import * as device from '../device.js';
import { state, notify, labelForKey, labelSlot, profile, liveScroll, KEY_TO_SCREEN,
         KEY_SUPER, hasPages, pageCountOf } from '../store.js';
import * as wheelDial from '../wheel-dial.js';
import * as cache from '../oled-cache.js';

let wheelDecay = null;
let wheelTotal = 0;

const CASE_W = 210;
const CASE_D = 125;
const keyPos = (i) => ({ x: -57 + ((i % 3) - 1) * 23, z: -2.5 + (Math.floor(i / 3) - 1.5) * 23 });
const WHEEL_POS = { x: 52, z: 1.5 };
const KNOB_POS = [{ x: 1, z: -41 }, { x: 92, z: -41 }];

// Centro de una pieza en tanto por ciento de la carcasa, que es lo que lee el CSS.
function at({ x, z }) {
  const px = ((x + CASE_W / 2) / CASE_W) * 100;
  const py = ((z + CASE_D / 2) / CASE_D) * 100;
  return `--x:${px.toFixed(3)};--y:${py.toFixed(3)}`;
}

export function init() {
  const container = document.getElementById('keyboard-visualizer');

  let html = `
    <div class="orby-board" role="img" aria-label="Réplica del Orby">
      <div class="orby-case"></div>`;

  for (let i = 1; i <= 12; i++) {
    const hasScreen = KEY_TO_SCREEN[i - 1] !== 0;
    html += `
      <div class="hw-key ${hasScreen ? '' : 'no-screen'}" id="hw-key-${i}" style="${at(keyPos(i - 1))}">
        <div class="hw-cap">
          ${hasScreen ? `<div class="oled-screen" id="hw-oled-${i}">--</div>`
                      : `<span class="hw-key-role">${i === KEY_SUPER ? 'SUPER' : 'MENÚ'}</span>`}
        </div>
      </div>`;
  }

  html += `
      <div class="hw-wheel" id="hw-scroll" title="Rueda" style="${at(WHEEL_POS)}">
        <div class="hw-wheel-face">${wheelDial.markerHtml('hw-wheel-needle')}</div>
        <div class="hw-wheel-shine"></div>
      </div>
      ${KNOB_POS.map((p, n) => `
      <div class="hw-encoder" id="hw-enc-${n + 1}" title="Encoder ${n + 1}" style="${at(p)}">
        <div class="hw-encoder-face"><div class="hw-encoder-mark"></div></div>
      </div>`).join('')}
    </div>`;

  container.innerHTML = html;

  device.on('telemetry', handleTelemetry);
  // El marcador sigue el ángulo absoluto que manda el teclado, así que se queda
  // donde esté la rueda de verdad en vez de volver a cero al soltarla.
  wheelDial.onUpdate(paintMarker);
  // Los iconos llegan por su cuenta (preloadAll, cambio de perfil…), así que el
  // dashboard tiene que repintarse cuando la caché cambia, no solo cuando lo
  // hace el estado del store.
  cache.onChange(render);
  render();
}

function paintMarker(deg) {
  const marker = document.getElementById('hw-wheel-needle');
  if (marker) marker.style.transform = `rotate(${deg}deg)`;
}

// Rehace el marcador cuando cambian los ajustes de calibración (forma incluida).
export function refreshMarker() {
  const face = document.querySelector('.hw-wheel-face');
  if (!face) return;
  face.querySelector('.dial-marker')?.remove();
  face.insertAdjacentHTML('afterbegin', wheelDial.markerHtml('hw-wheel-needle'));
  paintMarker(wheelDial.angle());
}

function handleTelemetry(line) {
  if (line.startsWith('KEY_EV:')) {
    const [, id, pressed] = line.split(':');
    const keyId = parseInt(id, 10);
    const isPressed = pressed === '1';

    if (keyId === KEY_SUPER) {
      state.superActive = isPressed;
      notify();
      render();
    }
    if (isPressed) {
      pulse(`hw-key-${keyId}`);
      showEvent(`Tecla ${keyId}`, describeKey(keyId));
    } else {
      document.getElementById(`hw-key-${keyId}`)?.classList.remove('active');
    }
  } else if (line.startsWith('ENC:')) {
    const [, id, delta] = line.split(':');
    pulse(`hw-enc-${id}`);
    const d = parseInt(delta, 10);
    showEvent(`Encoder ${id}`, `${d > 0 ? '+' : ''}${d}`);
  } else if (line.startsWith('ENC_SW:')) {
    const [, id, pressed] = line.split(':');
    if (pressed === '1') {
      pulse(`hw-enc-${id}`);
      showEvent(`Encoder ${id}`, 'Click');
    }
  } else if (line.startsWith('WHEEL:')) {
    handleWheel(parseInt(line.slice(6), 10));
  } else if (line.startsWith('MODE:')) {
    state.deviceMode = line.split(':')[1];
    notify();
    render();
    showEvent('Modo', state.deviceMode);
  } else if (line.startsWith('PROFILE:OK:')) {
    state.activeProfileIdx = parseInt(line.split(':')[2], 10);
    notify();
    render();
  }
}

function describeKey(keyId) {
  const prof = profile();
  if (!prof) return '';
  const layer = state.superActive ? 'super' : 'normal';
  return labelForKey(prof, keyId - 1, layer) || '';
}

// La rueda emite bloques agregados a 20 Hz. El giro del disco lo lleva
// wheel-dial.js con el ángulo absoluto; aquí solo se cuentan los clics del
// tirón actual y se enciende el brillo del borde.
function handleWheel(delta) {
  if (!Number.isFinite(delta)) return;
  wheelTotal += delta;

  const el = document.getElementById('hw-scroll');
  if (!el) return;
  el.classList.add('active');

  const detentsPerRev = liveScroll().detentsPerRev || 60;
  const detents = (wheelTotal * detentsPerRev) / 4096;
  showEvent('Scroll', `${detents >= 0 ? '+' : ''}${detents.toFixed(2)} clics`);

  clearTimeout(wheelDecay);
  wheelDecay = setTimeout(() => {
    el.classList.remove('active');
    wheelTotal = 0;
  }, 600);
}

function pulse(id) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.add('active');
  setTimeout(() => el.classList.remove('active'), 160);
}

function showEvent(title, detail) {
  const feed = document.getElementById('live-event-feed');
  if (!feed) return;
  feed.innerHTML = `
    <div class="big-event">
      <span class="big-event-title">${title}</span>
      ${detail ? `<span class="big-event-detail">${detail}</span>` : ''}
    </div>`;
}

export function render() {
  const prof = profile();
  const layer = state.superActive ? 'super' : 'normal';

  document.getElementById('lbl-active-profile').textContent = prof ? prof.name : '--';
  document.getElementById('lbl-active-mode').textContent = state.deviceMode;

  // La página la cambia también el propio teclado (botón de menú, gestor de
  // páginas, tecla de salto), así que se enseña aquí para saber en cuál está.
  const pageRow = document.getElementById('row-active-page');
  if (pageRow) {
    const count = prof ? pageCountOf(prof) : 1;
    pageRow.hidden = !hasPages();
    document.getElementById('lbl-active-page').textContent =
      `${Math.min(state.pageIdx, count - 1) + 1} de ${count}`;
  }

  const superEl = document.getElementById('lbl-super-state');
  superEl.textContent = state.superActive ? 'Activa' : 'Inactiva';
  superEl.classList.toggle('is-on', state.superActive);

  // La rueda es un ajuste del perfil y de la capa, así que el dashboard enseña
  // la sensibilidad que está aplicando el teclado en este momento.
  const scrollEl = document.getElementById('lbl-scroll-mode');
  if (scrollEl) {
    const s = liveScroll();
    scrollEl.textContent = `${s.detentsPerRev} clics/vuelta · ${s.hires ? 'suave' : 'clásico'}`;
    scrollEl.classList.toggle('is-on', s.hires);
  }

  for (let i = 1; i <= 12; i++) {
    const oled = document.getElementById(`hw-oled-${i}`);
    if (!oled) continue;

    const lslot = labelSlot(i - 1, layer);
    const bmp = prof && lslot >= 0 ? cache.get(state.activeProfileIdx, lslot) : null;
    if (bmp) {
      oled.innerHTML = `<canvas class="okey-canvas" data-bmp="${cache.cacheKey(state.activeProfileIdx, lslot)}"></canvas>`;
    } else {
      oled.textContent = (prof && labelForKey(prof, i - 1, layer)) || '--';
    }
  }
  cache.paintThumbs(document.getElementById('keyboard-visualizer'));
}
