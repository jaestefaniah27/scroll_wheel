// Vista Dashboard: réplica del hardware con telemetría en vivo.

import * as device from '../device.js';
import { state, notify, labelForKey, profile, KEY_TO_SCREEN } from '../store.js';

let wheelDecay = null;
let wheelTotal = 0;

export function init() {
  const container = document.getElementById('keyboard-visualizer');

  let html = `
    <div class="orby-board">
      <div class="controls-area">
        <div class="hw-encoder" id="hw-enc-1"><span class="hw-tag">VOL</span></div>
        <div class="hw-scroll" id="hw-scroll"><div class="hw-scroll-fill" id="hw-scroll-fill"></div></div>
        <div class="hw-encoder" id="hw-enc-2"><span class="hw-tag">BRI</span></div>
      </div>
      <div class="key-matrix">`;

  for (let i = 1; i <= 12; i++) {
    const hasScreen = KEY_TO_SCREEN[i - 1] !== 0;
    html += `
      <div class="hw-key ${hasScreen ? '' : 'no-screen'}" id="hw-key-${i}">
        ${hasScreen ? `<div class="oled-screen" id="hw-oled-${i}">--</div>`
                    : `<span class="hw-key-role">${i === 10 ? 'SUPER' : 'MENU'}</span>`}
      </div>`;
  }

  html += `</div></div>`;
  container.innerHTML = html;

  device.on('telemetry', handleTelemetry);
  render();
}

function handleTelemetry(line) {
  if (line.startsWith('KEY_EV:')) {
    const [, id, pressed] = line.split(':');
    const keyId = parseInt(id, 10);
    const isPressed = pressed === '1';

    if (keyId === 10) {
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
    showEvent(id === '1' ? 'Volumen' : 'Brillo', `${d > 0 ? '+' : ''}${d}`);
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

// La rueda emite bloques agregados a 20 Hz; los convertimos en una barra de
// intensidad que decae, que se lee mucho mejor que un simple parpadeo.
function handleWheel(delta) {
  if (!Number.isFinite(delta)) return;
  wheelTotal += delta;

  const el = document.getElementById('hw-scroll');
  const fill = document.getElementById('hw-scroll-fill');
  if (!el || !fill) return;

  el.classList.add('active');
  const magnitude = Math.min(100, Math.abs(delta) / 2);
  fill.style.height = `${magnitude}%`;
  fill.style.background = delta > 0 ? 'var(--success)' : 'var(--info)';

  const detentsPerRev = state.scroll.detentsPerRev || 60;
  const detents = (wheelTotal * detentsPerRev) / 4096;
  showEvent('Scroll', `${detents >= 0 ? '+' : ''}${detents.toFixed(2)} clics`);

  clearTimeout(wheelDecay);
  wheelDecay = setTimeout(() => {
    el.classList.remove('active');
    fill.style.height = '0%';
    wheelTotal = 0;
  }, 300);
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

  const scrollEl = document.getElementById('lbl-scroll-mode');
  if (scrollEl) {
    scrollEl.textContent = state.scroll.hires ? 'Suave (alta res.)' : 'Clásico (3 líneas)';
    scrollEl.classList.toggle('is-on', state.scroll.hires);
  }

  for (let i = 1; i <= 12; i++) {
    const oled = document.getElementById(`hw-oled-${i}`);
    if (oled) oled.textContent = (prof && labelForKey(prof, i - 1, layer)) || '--';
  }
}
