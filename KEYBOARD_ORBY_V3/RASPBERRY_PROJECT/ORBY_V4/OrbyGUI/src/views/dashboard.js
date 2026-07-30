// Vista Dashboard: réplica del hardware con telemetría en vivo.
//
// La disposición sigue al teclado real: las doce teclas a la izquierda y, a su
// derecha, los mandos —los dos encoders arriba y la rueda magnética, que es un
// disco grande, debajo de ellos.

import * as device from '../device.js';
import { state, notify, labelForKey, profile, liveScroll, KEY_TO_SCREEN,
         KEY_SUPER } from '../store.js';
import * as wheelDial from '../wheel-dial.js';

let wheelDecay = null;
let wheelTotal = 0;

export function init() {
  const container = document.getElementById('keyboard-visualizer');

  let html = `
    <div class="orby-board">
      <div class="board-keys">`;

  for (let i = 1; i <= 12; i++) {
    const hasScreen = KEY_TO_SCREEN[i - 1] !== 0;
    html += `
      <div class="hw-key ${hasScreen ? '' : 'no-screen'}" id="hw-key-${i}">
        ${hasScreen ? `<div class="oled-screen" id="hw-oled-${i}">--</div>`
                    : `<span class="hw-key-role">${i === KEY_SUPER ? 'SUPER' : 'MENU'}</span>`}
      </div>`;
  }

  html += `
      </div>
      <div class="board-controls">
        <div class="encoder-row">
          <div class="hw-encoder" id="hw-enc-1"><span class="hw-tag">ENC 1</span></div>
          <div class="hw-encoder" id="hw-enc-2"><span class="hw-tag">ENC 2</span></div>
        </div>
        <div class="hw-wheel" id="hw-scroll">
          <div class="hw-wheel-face">
            ${wheelDial.markerHtml('hw-wheel-needle')}
            <div class="hw-wheel-hub"></div>
          </div>
          <span class="hw-tag">RUEDA</span>
        </div>
      </div>
    </div>`;

  container.innerHTML = html;

  device.on('telemetry', handleTelemetry);
  // El marcador sigue el ángulo absoluto que manda el teclado, así que se queda
  // donde esté la rueda de verdad en vez de volver a cero al soltarla.
  wheelDial.onUpdate(paintMarker);
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
    if (oled) oled.textContent = (prof && labelForKey(prof, i - 1, layer)) || '--';
  }
}
