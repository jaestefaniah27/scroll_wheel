// Vista Inicio: la réplica del teclado, viva, y nada más.
//
// Debajo de cada zona del aparato va su botón de configurar, alineado con
// ella: el de las teclas bajo las teclas y el de la rueda y los mandos bajo la
// rueda. Pulsar una pieza de la réplica hace lo mismo que el botón de su zona,
// ya con esa pieza elegida.
//
// La réplica se coloca con las medidas del Orby montado, en milímetros y con el
// origen en el centro de la carcasa (x a la derecha, z hacia el usuario). Están
// tomadas de una foto cenital del teclado real, no del modelo de la web: allí
// las teclas iban a 23 mm de paso y los mandos y la rueda algo desplazados.
// En la foto el paso sale igual en las dos direcciones (24,4 y 24,5 mm), que
// es lo que confirma que la proporción de la carcasa, 210 x 125, está bien.
//
// Las pantallas no se dibujan aquí: son diez canvas que pinta oled-mirror.js
// con el mismo motor de animación que el firmware. Esta vista solo le da los
// lienzos y, cuando el teclado no puede mandar sus pantallas, lo que la app
// sabe que hay en ellas.

import * as device from '../device.js';
import { state, notify, labelForKey, labelSlot, profile, KEY_TO_SCREEN, KEY_SUPER,
         pageCountOf } from '../store.js';
import * as wheelDial from '../wheel-dial.js';
import * as cache from '../oled-cache.js';
import * as oledMirror from '../oled-mirror.js';
import { icon } from '../icons.js';
import { goTo } from '../nav.js';
import { ROTARY_SLOTS } from '../hid-keys.js';

let wheelDecay = null;

const CASE_W = 210;
const CASE_D = 125;
const KEY_PITCH = 24.4;
const keyPos = (i) => ({ x: -59.8 + ((i % 3) - 1) * KEY_PITCH, z: 0.6 + (Math.floor(i / 3) - 1.5) * KEY_PITCH });
const WHEEL_POS = { x: 42.5, z: 5 };
const KNOB_POS = [{ x: -2, z: -34.5 }, { x: 84, z: -34.5 }];

// Las dos zonas que se configuran por separado, con el centro de cada una en
// mm. La de la rueda se centra en la rueda y no en el conjunto con los mandos:
// es lo que el ojo toma por "esa parte" del aparato.
const ZONES = [
  { id: 'keys',  x: -59.8, label: 'Configurar teclas',  icon: 'keyboard' },
  { id: 'wheel', x: WHEEL_POS.x, label: 'Rueda y mandos', icon: 'wheel' },
];

// Ritmo de fábrica de cada grupo de animación, en % del muelle de Apple. Es
// ANIM_BASE_PCT de main.cpp: solo se usa sin espejo, porque con él el teclado
// ya manda el ritmo calculado en cada transición.
const ANIM_BASE = { menu: 200, carousel: 65, submenu: 200, profile: 140, fade: 100 };

// Centro de una pieza en tanto por ciento de la carcasa, que es lo que lee el CSS.
function at({ x, z }) {
  const px = ((x + CASE_W / 2) / CASE_W) * 100;
  const py = ((z + CASE_D / 2) / CASE_D) * 100;
  return `--x:${px.toFixed(3)};--y:${py.toFixed(3)}`;
}

const escape = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

export function init() {
  const container = document.getElementById('keyboard-visualizer');

  let html = `
    <div class="orby-rig">
    <div class="orby-board" aria-label="Réplica del Orvy">
      <div class="orby-case"></div>`;

  for (let i = 1; i <= 12; i++) {
    const screen = KEY_TO_SCREEN[i - 1];
    html += `
      <div class="hw-key ${screen ? '' : 'no-screen'}" id="hw-key-${i}" data-key="${i - 1}" style="${at(keyPos(i - 1))}">
        <div class="hw-cap">
          ${screen ? `<div class="oled-screen"><canvas class="oled-px" data-screen="${screen - 1}"></canvas><span class="oled-text" id="hw-oled-${i}"></span></div>`
                   : `<span class="hw-key-role">${i === KEY_SUPER ? 'SUPER' : 'MENÚ'}</span>`}
        </div>
      </div>`;
  }

  html += `
      <div class="hw-wheel" id="hw-scroll" data-rotary="${ROTARY_SLOTS.WHEEL_CW}" style="${at(WHEEL_POS)}">
        <div class="hw-wheel-face">${wheelDial.markerHtml('hw-wheel-needle')}</div>
        <div class="hw-wheel-shine"></div>
      </div>
      ${KNOB_POS.map((p, n) => `
      <div class="hw-encoder" id="hw-enc-${n + 1}" data-rotary="${n ? ROTARY_SLOTS.ENC2_CW : ROTARY_SLOTS.ENC1_CW}" style="${at(p)}">
        <div class="hw-encoder-face"></div>
      </div>`).join('')}
    </div>
    <div class="rig-actions">
      ${ZONES.map((z) => `
      <button class="rig-btn" data-zone="${z.id}" style="--x:${(((z.x + CASE_W / 2) / CASE_W) * 100).toFixed(3)}">
        ${icon(z.icon, 15)}<span>${z.label}</span>
      </button>`).join('')}
    </div>
    </div>`;

  container.innerHTML = html;

  const canvases = [];
  container.querySelectorAll('canvas.oled-px').forEach((cv) => { canvases[Number(cv.dataset.screen)] = cv; });
  oledMirror.attach(canvases);
  oledMirror.onLiveChange(render);

  device.on('telemetry', handleTelemetry);
  // El marcador sigue el ángulo absoluto que manda el teclado, así que se queda
  // donde esté la rueda de verdad en vez de volver a cero al soltarla.
  wheelDial.onUpdate(paintMarker);
  // Los iconos llegan por su cuenta (preloadAll, cambio de perfil…), así que el
  // dashboard tiene que repintarse cuando la caché cambia, no solo cuando lo
  // hace el estado del store.
  cache.onChange(render);

  container.addEventListener('click', onClick);

  render();
}

// Sin teclado se deja entrar igual: el editor enseña la copia del PC en solo
// lectura, que también sirve para mirar qué hace cada cosa.
function onClick(e) {
  const zone = e.target.closest('[data-zone]');
  if (zone) {
    goTo('view-profiles', zone.dataset.zone === 'keys' ? { key: 0 } : { rotary: ROTARY_SLOTS.WHEEL_CW });
    return;
  }
  const key = e.target.closest('[data-key]');
  if (key) {
    goTo('view-profiles', { key: Number(key.dataset.key) });
    return;
  }
  const rotary = e.target.closest('[data-rotary]');
  if (rotary) goTo('view-profiles', { rotary: Number(rotary.dataset.rotary) });
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
    // Sin espejo, el negativo y el muelle de la pulsación los pone la app; con
    // él llegan del teclado (EV:INV) y esto no hace nada.
    oledMirror.setPressed(KEY_TO_SCREEN[keyId - 1] - 1, isPressed);
    document.getElementById(`hw-key-${keyId}`)?.classList.toggle('active', isPressed);
  } else if (line.startsWith('ENC:')) {
    pulse(`hw-enc-${line.split(':')[1]}`);
  } else if (line.startsWith('ENC_SW:')) {
    const [, id, pressed] = line.split(':');
    if (pressed === '1') pulse(`hw-enc-${id}`);
  } else if (line.startsWith('WHEEL:')) {
    handleWheel(parseInt(line.slice(6), 10));
  } else if (line.startsWith('MODE:')) {
    state.deviceMode = line.split(':')[1];
    notify();
    render();
  } else if (line.startsWith('PROFILE:OK:')) {
    state.activeProfileIdx = parseInt(line.split(':')[2], 10);
    notify();
    render();
  }
}

// La rueda emite bloques agregados a 20 Hz. El giro del disco lo lleva
// wheel-dial.js con el ángulo absoluto; aquí solo se enciende el brillo del
// borde mientras dura el tirón.
function handleWheel(delta) {
  if (!Number.isFinite(delta)) return;
  const el = document.getElementById('hw-scroll');
  if (!el) return;
  el.classList.add('active');
  clearTimeout(wheelDecay);
  wheelDecay = setTimeout(() => el.classList.remove('active'), 600);
}

function pulse(id) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.add('active');
  setTimeout(() => el.classList.remove('active'), 160);
}

// Lo que la app sabe de cada pantalla sin preguntarle al teclado: el icono
// subido si lo hay. Las teclas con solo etiqueta llevan el texto encima.
function fallbackScreens(prof, layer) {
  const screens = new Array(10).fill(null);
  for (let ki = 0; ki < 12; ki++) {
    const screen = KEY_TO_SCREEN[ki];
    if (!screen) continue;
    const slot = labelSlot(ki, layer);
    screens[screen - 1] = prof && slot >= 0 ? cache.get(state.activeProfileIdx, slot, state.pageIdx) : null;
  }
  return screens;
}

function renderBoardFallback(prof, layer) {
  const live = oledMirror.isLive();
  const screens = live ? null : fallbackScreens(prof, layer);

  for (let ki = 0; ki < 12; ki++) {
    const el = document.getElementById(`hw-oled-${ki + 1}`);
    if (!el) continue;
    // Con espejo, lo que se ve es exactamente lo del teclado: nada encima.
    const text = live || screens[KEY_TO_SCREEN[ki] - 1] ? '' : (prof && labelForKey(prof, ki, layer)) || '';
    el.textContent = text;
  }

  if (live) return;
  const count = prof ? pageCountOf(prof) : 1;
  const rates = {};
  for (const [k, base] of Object.entries(ANIM_BASE)) rates[k] = (base * (state.animSpeed[k] ?? 100)) / 10000;
  oledMirror.fallback({
    profile: state.activeProfileIdx,
    page: Math.min(state.pageIdx, count - 1),
    pages: count,
    super: state.superActive,
  }, screens, rates);
}

export function render() {
  const prof = profile();
  renderBoardFallback(prof, state.superActive ? 'super' : 'normal');
}
