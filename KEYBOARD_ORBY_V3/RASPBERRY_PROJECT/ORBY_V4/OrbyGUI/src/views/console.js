// Consola serie.
//
// La versión anterior hacía `out.innerHTML += ...` por cada línea recibida.
// Con la rueda emitiendo telemetría eso reconstruía el DOM entero decenas de
// veces por segundo y la app acababa congelándose. Ahora hay un búfer circular
// y un único repintado por frame de animación.

import * as device from '../device.js';

const MAX_LINES = 500;
const lines = [];
let paused = false;
let frame = null;
let filter = '';

export function init() {
  const input = document.getElementById('console-input');

  document.getElementById('btn-clear-console').addEventListener('click', () => {
    lines.length = 0;
    schedule();
  });

  const pauseBtn = document.getElementById('btn-pause-console');
  pauseBtn.addEventListener('click', () => {
    paused = !paused;
    pauseBtn.classList.toggle('is-on', paused);
    pauseBtn.title = paused ? 'Reanudar' : 'Pausar';
  });

  document.getElementById('console-filter').addEventListener('input', (e) => {
    filter = e.target.value.trim().toUpperCase();
    schedule();
  });

  const history = [];
  let historyIdx = -1;

  input.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' && input.value.trim()) {
      const cmd = input.value.trim();
      history.unshift(cmd);
      historyIdx = -1;
      device.send(cmd);
      input.value = '';
    } else if (e.key === 'ArrowUp' && history.length) {
      historyIdx = Math.min(historyIdx + 1, history.length - 1);
      input.value = history[historyIdx];
      e.preventDefault();
    } else if (e.key === 'ArrowDown') {
      historyIdx = Math.max(historyIdx - 1, -1);
      input.value = historyIdx >= 0 ? history[historyIdx] : '';
      e.preventDefault();
    }
  });

  device.on('rx', (line) => push(line, 'rx'));
  device.on('tx', (line) => push(line, 'tx'));
  device.on('error', (err) => push(`ERROR: ${err}`, 'error'));
  device.on('connected', (info) => push(`Conectado: ${info?.raw || 'ORBY_V4'}`, 'system'));
  device.on('disconnected', () => push('Dispositivo desconectado.', 'error'));

  push('Terminal OrbyGUI. Escribe GET_STATE para volcar la configuración.', 'system');
}

export function push(text, type = 'system') {
  if (paused && type === 'rx') return;
  const time = new Date().toLocaleTimeString('es-ES', { hour12: false });
  lines.push({ time, text, type });
  if (lines.length > MAX_LINES) lines.splice(0, lines.length - MAX_LINES);
  schedule();
}

function schedule() {
  if (frame) return;
  frame = requestAnimationFrame(() => {
    frame = null;
    paint();
  });
}

function escapeHtml(s) {
  return s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
}

function paint() {
  const out = document.getElementById('console-output');
  if (!out) return;

  const atBottom = out.scrollHeight - out.scrollTop - out.clientHeight < 40;
  const visible = filter ? lines.filter((l) => l.text.toUpperCase().includes(filter)) : lines;

  out.innerHTML = visible
    .map((l) => `<div class="console-line ${l.type}"><span class="ts">${l.time}</span>${escapeHtml(l.text)}</div>`)
    .join('');

  if (atBottom) out.scrollTop = out.scrollHeight;
}
