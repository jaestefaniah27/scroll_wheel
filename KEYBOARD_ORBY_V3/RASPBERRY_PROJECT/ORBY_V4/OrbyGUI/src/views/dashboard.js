// Vista Inicio: la réplica del teclado, viva, y nada más.
//
// Debajo de cada zona del aparato va su botón de configurar, alineado con
// ella: el de las teclas bajo las teclas y el de la rueda y los mandos bajo la
// rueda. Pulsar una pieza de la réplica hace lo mismo que el botón de su zona,
// ya con esa pieza elegida.
//
// Configurar es ampliar: la réplica se acerca a la zona, que se queda a la
// izquierda, y por la derecha entra el panel con el editor (profiles.js, en
// modo panel). Es el encuadre por zonas de la versión móvil de orvy.es: allí
// la cámara persigue su objetivo con un suavizado exponencial, y aquí lo hace
// una transición CSS con una curva de la misma forma. Tocar una pieza de la
// otra zona cambia de encuadre, igual que en la web.
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
import { ROTARY_SLOTS } from '../hid-keys.js';
import * as profiles from './profiles.js';

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

// Lo que encuadra cada zona al ampliarla, en mm. Entra todo el alto de la
// carcasa y su borde del lado de la zona: sin borde a la vista la ampliación
// se lee como una foto recortada y no como el aparato visto de cerca.
const FRAMES = {
  keys:  { x0: -108, x1: -14, z0: -64, z1: 64 },
  wheel: { x0: -14,  x1: 108, z0: -64, z1: 64 },
};
// Hueco entre la réplica ampliada y el panel, y margen con los bordes.
const ZOOM_GAP = 32;

// Zona ampliada ahora mismo, o null en reposo.
let zoom = null;

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
    </div>
    <!-- Fuera de .orby-rig: ampliada lleva transform, y dentro escalaría con ella. -->
    <div class="rig-layer">${icon('keyboard', 13)} Editando la capa SUPER</div>`;

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
  document.getElementById('zoom-panel').addEventListener('click', (e) => {
    if (e.target.closest('[data-act="panel-done"]')) closeZoom();
  });
  // Con una zona ampliada, pulsar fuera del aparato (el fondo del escenario)
  // vuelve al Inicio: es tocar fuera de una hoja de iOS. La carcasa cuenta como
  // aparato, para que un clic que se queda corto al apuntar a una tecla no
  // deshaga el zoom. Se mira .orby-rig y no solo la réplica porque el clic en
  // "Configurar teclas" sube hasta aquí ya con el zoom puesto; ampliado, esa
  // fila no recibe clics (pointer-events: none) y su hueco cuenta como fondo.
  //
  // Con composedPath() y no con closest(): el panel se repinta entero al
  // pulsar cualquiera de sus botones, y para cuando el clic sube hasta aquí el
  // botón ya no está en el documento y closest() no encuentra nada.
  document.getElementById('view-dashboard').addEventListener('click', (e) => {
    if (!zoom) return;
    const inside = e.composedPath().some((el) => el.id === 'zoom-panel' || el.classList?.contains('orby-rig'));
    if (!inside) closeZoom();
  });
  window.addEventListener('keydown', onKey);
  window.addEventListener('resize', () => { if (zoom) frameZone(); });

  render();
}

// Sin teclado se deja entrar igual: el editor enseña la copia del PC en solo
// lectura, que también sirve para mirar qué hace cada cosa.
function onClick(e) {
  const zone = e.target.closest('[data-zone]');
  if (zone) {
    openZoom(zone.dataset.zone, zone.dataset.zone === 'keys' ? { key: 0 } : { rotary: ROTARY_SLOTS.WHEEL_CW });
    return;
  }
  const key = e.target.closest('[data-key]');
  if (key) {
    openZoom('keys', { key: Number(key.dataset.key) });
    return;
  }
  const rotary = e.target.closest('[data-rotary]');
  if (rotary) openZoom('wheel', { rotary: Number(rotary.dataset.rotary) });
}

function openZoom(zone, target) {
  const dash = document.querySelector('#view-dashboard .dash');
  const panel = document.getElementById('zoom-panel');

  if (zoom === zone) {
    profiles.selectInPanel(target);
    return;
  }
  const opening = !zoom;
  zoom = zone;

  // Arriba del todo antes de medir: con la vista desplazada, el encuadre
  // saldría corrido lo mismo que el scroll.
  document.getElementById('view-dashboard').scrollTop = 0;
  dash.classList.add('editing');
  dash.dataset.zoom = zone;
  panel.setAttribute('aria-hidden', 'false');
  // Cada repintado del panel puede haber cambiado la capa (NORMAL/SUPER) o lo
  // que hay en una tecla: la réplica se pone al día con él.
  profiles.openPanel(panel, zone, target, { onRender: () => { paintSelection(); render(); } });
  frameZone();
  if (opening) panel.querySelector('.zp-scroll')?.focus({ preventScroll: true });
}

export function closeZoom() {
  if (!zoom) return;
  zoom = null;
  const dash = document.querySelector('#view-dashboard .dash');
  dash.classList.remove('editing');
  delete dash.dataset.zoom;
  document.getElementById('zoom-panel').setAttribute('aria-hidden', 'true');
  document.querySelector('#view-dashboard .orby-rig').style.transform = '';
  profiles.closePanel();
  paintSelection();
  render();
}

export function isZoomed() {
  return Boolean(zoom);
}

// Lleva la zona al hueco que deja el panel: escala para que quepa entera y
// traslada su centro al centro del hueco. Se mide con las cajas de maquetación
// (offset*), que no cambian con el transform: así se puede volver a encuadrar
// a mitad de animación (otra zona, ventana redimensionada) sin medir lo ya
// movido.
function frameZone() {
  const dash = document.querySelector('#view-dashboard .dash');
  const rig = dash.querySelector('.orby-rig');
  const board = rig.querySelector('.orby-board');
  const panel = document.getElementById('zoom-panel');
  const f = FRAMES[zoom];

  const u = board.offsetWidth / CASE_W;               // px por mm sin ampliar
  const zoneW = (f.x1 - f.x0) * u;
  const zoneH = (f.z1 - f.z0) * u;
  const zoneCx = ((f.x0 + f.x1) / 2 + CASE_W / 2) * u;  // en px desde la esquina de la réplica
  const zoneCy = ((f.z0 + f.z1) / 2 + CASE_D / 2) * u;

  const areaW = dash.clientWidth - panel.offsetWidth - ZOOM_GAP * 2;
  const areaH = dash.clientHeight - ZOOM_GAP * 2;
  const scale = Math.min(areaW / zoneW, areaH / zoneH, 3);
  const areaCx = ZOOM_GAP + areaW / 2;
  const areaCy = ZOOM_GAP + areaH / 2;

  // El origen del transform es la esquina de .orby-rig, que es la de la réplica.
  const tx = areaCx - rig.offsetLeft - zoneCx * scale;
  const ty = areaCy - rig.offsetTop - zoneCy * scale;
  rig.style.transform = `translate(${tx.toFixed(1)}px, ${ty.toFixed(1)}px) scale(${scale.toFixed(4)})`;
}

// El anillo de la pieza que se está editando, en la réplica.
function paintSelection() {
  const board = document.querySelector('#view-dashboard .orby-board');
  if (!board) return;
  board.querySelectorAll('.is-editing').forEach((el) => el.classList.remove('is-editing'));
  if (!zoom) return;
  const sel = profiles.panelSelection();
  if (!sel) return;
  if (sel.kind === 'key') {
    document.getElementById(`hw-key-${sel.index + 1}`)?.classList.add('is-editing');
  } else {
    const id = sel.slot <= ROTARY_SLOTS.ENC1_CLICK ? 'hw-enc-1'
      : sel.slot <= ROTARY_SLOTS.ENC2_CLICK ? 'hw-enc-2' : 'hw-scroll';
    document.getElementById(id)?.classList.add('is-editing');
  }
}

// Esc cierra y las flechas mueven la tecla elegida por la rejilla de 3 x 4,
// como en un teclado de verdad. Nada de esto mientras se escribe en un campo:
// ahí las flechas y el Esc son del campo. Capturar un atajo se queda el
// teclado antes (profiles.js escucha en captura y corta la propagación).
function onKey(e) {
  if (!zoom || !document.getElementById('view-dashboard').classList.contains('active')) return;
  if (e.target.closest?.('input, select, textarea, [contenteditable="true"]')) return;

  if (e.key === 'Escape') {
    e.preventDefault();
    // Dentro de un tipo de acción, Esc vuelve a la tecla; desde la tecla, al Inicio.
    if (!profiles.panelBack()) closeZoom();
    return;
  }

  const sel = profiles.panelSelection();
  if (zoom !== 'keys' || sel?.kind !== 'key') return;
  const moves = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] };
  const m = moves[e.key];
  if (!m) return;
  e.preventDefault();
  const col = Math.min(2, Math.max(0, (sel.index % 3) + m[0]));
  const row = Math.min(3, Math.max(0, Math.floor(sel.index / 3) + m[1]));
  profiles.selectInPanel({ key: row * 3 + col });
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

// `preview` es la capa SUPER mientras se edita en el panel: se pinta con lo
// que sabe la app aunque el teclado esté mandando sus pantallas, porque las
// suyas son las de la capa normal (ver preview() en oled-mirror.js).
function renderBoardFallback(prof, layer, preview = false) {
  const live = oledMirror.isLive() && !preview;
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
  oledMirror[preview ? 'preview' : 'fallback']({
    profile: state.activeProfileIdx,
    page: Math.min(state.pageIdx, count - 1),
    pages: count,
    super: preview || state.superActive,
  }, screens, animRates());
}

function animRates() {
  const rates = {};
  for (const [k, base] of Object.entries(ANIM_BASE)) rates[k] = (base * (state.animSpeed[k] ?? 100)) / 10000;
  return rates;
}

export function render() {
  const prof = profile();
  // Editando la capa SUPER, las pantallas la enseñan hasta que se acaba de
  // editar (vuelta a NORMAL o fuera del zoom): si no, lo que se ve a la
  // izquierda no es lo que se está tocando a la derecha.
  const editingSuper = Boolean(zoom) && profiles.panelLayer() === 'super';
  document.querySelector('#view-dashboard .dash')?.classList.toggle('layer-super', editingSuper);
  document.getElementById(`hw-key-${KEY_SUPER}`)?.classList.toggle('held', editingSuper);
  if (!editingSuper && oledMirror.isPreviewing()) oledMirror.endPreview(animRates());
  renderBoardFallback(prof, editingSuper || state.superActive ? 'super' : 'normal', editingSuper);
}

// Al volver al Inicio con una zona ampliada (de Ajustes, del editor de
// iconos…) el panel puede haberse quedado atrás: la vista de Perfiles comparte
// su selección y es ella quien se pinta mientras está abierta. No va en
// render(), que corre con cada icono que llega: repintar el panel ahí le
// quitaría el foco a lo que se esté escribiendo.
export function shown() {
  if (!zoom) return;
  profiles.render();
  frameZone();
}
