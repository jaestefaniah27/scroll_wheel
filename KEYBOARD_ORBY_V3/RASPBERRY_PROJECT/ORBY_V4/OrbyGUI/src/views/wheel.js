// Calibración de la rueda de scroll.
//
// El firmware expresa la sensibilidad como "detents por vuelta": cuántos clics
// de rueda de ratón clásica equivale dar una vuelta completa a la rueda
// magnética. El AS5600 tiene 4096 cuentas por vuelta, así que:
//
//   unidades_alta_resolucion = cuentas * detents_por_vuelta * 120 / 4096
//
// El resto de esa división NO se descarta en el firmware, que es justamente lo
// que permite el desplazamiento fino.

import * as device from '../device.js';
import { state, notify } from '../store.js';
import { icon } from '../icons.js';
import { toast } from '../ui.js';

const PRESETS = [
  { value: 12,  name: 'Preciso',  desc: 'Timeline, PCB, edición fina' },
  { value: 30,  name: 'Suave',    desc: 'Lectura y navegación lenta' },
  { value: 60,  name: 'Estándar', desc: 'Equivalente a rueda de ratón' },
  { value: 120, name: 'Rápido',   desc: 'Documentos y logs largos' },
];

let liveTotal = 0;
let decayTimer = null;

export function init() {
  const root = document.getElementById('view-wheel');
  root.addEventListener('click', onClick);
  root.addEventListener('input', onInput);
  root.addEventListener('change', onChange);

  device.on('telemetry', (line) => {
    if (line.startsWith('WHEEL:')) onWheelTelemetry(parseInt(line.slice(6), 10));
  });

  render();
}

function onClick(e) {
  const el = e.target.closest('[data-act]');
  if (!el) return;

  if (el.dataset.act === 'preset') {
    apply(Number(el.dataset.value));
  } else if (el.dataset.act === 'invert') {
    device.setScrollInvert(!state.scroll.invert)
      .then((line) => { readScrollLine(line); render(); })
      .catch(() => toast('El teclado no respondió', 'error'));
  } else if (el.dataset.act === 'refresh') {
    device.getScroll()
      .then((line) => { readScrollLine(line); render(); })
      .catch(() => toast('El teclado no respondió', 'error'));
  } else if (el.dataset.act === 'save') {
    device.saveToFlash()
      .then(() => { state.dirty = false; notify(); toast('Calibración guardada en Flash'); })
      .catch(() => toast('El teclado no confirmó el guardado', 'error'));
  }
}

// Deslizar actualiza solo el número; el comando se envía al soltar para no
// inundar el CDC con un SET_SCROLL por cada píxel del slider.
function onInput(e) {
  if (e.target.id !== 'wheel-slider') return;
  document.getElementById('wheel-value').textContent = e.target.value;
  updateDerived(Number(e.target.value));
}

function onChange(e) {
  if (e.target.id === 'wheel-slider') apply(Number(e.target.value));
}

async function apply(detents) {
  try {
    const line = await device.setScroll(detents);
    readScrollLine(line);
    state.dirty = true;
    notify();
    render();
  } catch {
    toast('El teclado no respondió', 'error');
  }
}

function readScrollLine(line) {
  if (typeof line !== 'string' || !line.startsWith('SCROLL:OK:')) return;
  const [det, inv, hires] = line.slice(10).split(':').map(Number);
  state.scroll = { detentsPerRev: det, invert: !!inv, hires: !!hires };
  notify();
}

function onWheelTelemetry(delta) {
  if (!Number.isFinite(delta)) return;
  liveTotal += delta;

  const bar = document.getElementById('wheel-live-bar');
  const readout = document.getElementById('wheel-live-readout');
  if (!bar) return;

  const degrees = (liveTotal / 4096) * 360;
  const detents = (liveTotal * state.scroll.detentsPerRev) / 4096;
  bar.style.transform = `rotate(${degrees}deg)`;
  if (readout) {
    readout.textContent = `${degrees >= 0 ? '+' : ''}${degrees.toFixed(1)}°  ·  ${detents >= 0 ? '+' : ''}${detents.toFixed(2)} clics`;
  }

  clearTimeout(decayTimer);
  decayTimer = setTimeout(() => { liveTotal = 0; }, 1200);
}

function updateDerived(detents) {
  const el = document.getElementById('wheel-derived');
  if (!el) return;
  const degPerDetent = 360 / detents;
  const unitsPerCount = (detents * 120) / 4096;
  el.innerHTML = `
    <li><span class="lbl">Giro por clic</span><span class="val">${degPerDetent.toFixed(1)}°</span></li>
    <li><span class="lbl">Unidades HID por cuenta</span><span class="val">${unitsPerCount.toFixed(3)}</span></li>
    <li><span class="lbl">Líneas por vuelta (Windows)</span><span class="val">${(detents * 3).toFixed(0)}</span></li>`;
}

export function render() {
  const root = document.getElementById('view-wheel');
  if (!root) return;

  const s = state.scroll;
  root.innerHTML = `
    <header class="view-header">
      <h1>Rueda de scroll</h1>
      <p>Calibración de la rueda magnética AS5600 y estado del scroll de alta resolución</p>
    </header>

    <div class="wheel-grid">
      <div class="glass-panel wheel-main">
        <div class="card-header">${icon('wheel', 22)}<h2>Sensibilidad</h2></div>

        <div class="wheel-readout">
          <span id="wheel-value">${s.detentsPerRev}</span>
          <small>clics por vuelta completa</small>
        </div>

        <input type="range" id="wheel-slider" class="premium-slider"
               min="6" max="240" step="1" value="${s.detentsPerRev}">

        <div class="preset-row">
          ${PRESETS.map((p) => `
            <button class="preset-btn ${s.detentsPerRev === p.value ? 'on' : ''}" data-act="preset" data-value="${p.value}">
              <strong>${p.name}</strong>
              <span>${p.value} clics</span>
              <em>${p.desc}</em>
            </button>`).join('')}
        </div>

        <ul class="info-list" id="wheel-derived"></ul>

        <div class="wheel-actions">
          <button class="secondary-btn ${s.invert ? 'is-on' : ''}" data-act="invert">
            ${icon('reset', 16)} ${s.invert ? 'Dirección invertida' : 'Invertir dirección'}
          </button>
          <button class="secondary-btn" data-act="refresh">${icon('refresh', 16)} Releer del teclado</button>
          <button class="primary-btn" data-act="save">${icon('save', 16)} Guardar en Flash</button>
        </div>
      </div>

      <div class="glass-panel wheel-side">
        <div class="card-header">${icon('bolt', 22)}<h2>Alta resolución</h2></div>
        <div class="hires-state ${s.hires ? 'ok' : 'off'}">
          <span class="hires-dot"></span>
          <strong>${s.hires ? 'Activa' : 'No negociada'}</strong>
        </div>
        <p class="setting-desc">
          ${s.hires
            ? 'Windows ha solicitado el multiplicador de resolución: el desplazamiento viaja en unidades de 1/120 de clic, lo que da el scroll pixel a pixel en Chrome, Edge, el Explorador y VS Code.'
            : 'El sistema aún no ha pedido el multiplicador. Ocurre justo tras enumerar el dispositivo; si sigue apagado, desconecta y vuelve a conectar el teclado. Algunas aplicaciones Win32 antiguas ignoran el desplazamiento fraccionario y seguirán saltando de tres en tres líneas.'}
        </p>

        <div class="card-header mt-4">${icon('dashboard', 22)}<h2>Prueba en vivo</h2></div>
        <div class="wheel-dial">
          <div class="wheel-dial-face"><div class="wheel-dial-needle" id="wheel-live-bar"></div></div>
        </div>
        <p class="wheel-live-readout" id="wheel-live-readout">Gira la rueda para comprobar el recorrido</p>

        <div class="scroll-testbed">
          <div class="scroll-testbed-inner">
            ${Array.from({ length: 60 }, (_, i) =>
              `<div class="tb-row"><span>${String(i + 1).padStart(3, '0')}</span><i style="width:${20 + ((i * 37) % 70)}%"></i></div>`).join('')}
          </div>
        </div>
        <p class="setting-desc">Desplázate aquí con la rueda para notar la diferencia sin salir de la app.</p>
      </div>
    </div>`;

  updateDerived(s.detentsPerRev);
}
