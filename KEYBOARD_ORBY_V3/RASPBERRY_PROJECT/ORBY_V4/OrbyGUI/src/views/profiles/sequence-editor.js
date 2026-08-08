// Pintado de la pestaña "Secuencia": la lista de pasos y los botones para
// añadir cada tipo. La lógica de los pasos vive en macro-tabs.js; aquí solo se
// monta el HTML.

import { state, hasPages, maxPages, pageCountOf } from '../../store.js';
import { MODIFIERS, GOTO_PAGE_MODIFIER, PAGE_STATE_MODIFIER, describeAction } from '../../hid-keys.js';
import { icon } from '../../icons.js';
import { DEFAULT_STEP_DELAY_MS } from './constants.js';
import { view } from './view-state.js';
import { macroById, macroDeviceEligible, DEVICE_STEP_TYPE, MACRO_MAX_STEPS_DEVICE,
         DEFAULT_REPEAT_GAP_MS } from './macros-store.js';
import { ensureInstalledApps, installedAppsDatalist, seqKeyBuilder,
         capturePosEditIndex, lastMousePos } from './macro-tabs.js';
import { escape, keycodeOptions } from './util.js';

// Editor de una macro (secuencia ejecutada en el PC): lista de acciones con
// botón de borrar cada una, botones para añadir las de un solo paso, y un modo
// de grabación que captura pulsaciones reales del teclado del PC.
export function renderSequenceEditor(macroId) {
  const macro = macroId === null ? null : macroById(macroId);
  const actions = macro?.actions || [];
  const recording = view.capturing === 'sequence';
  const capturingPos = view.capturing === 'position';

  // El buscador de apps instaladas (datalist) de cada paso "Abrir" necesita
  // la lista cargada, igual que en la pestaña App (ver ensureInstalledApps).
  if (actions.some((a) => a.type === 'open_app' && a.kind !== 'file')) ensureInstalledApps();

  const CLICK_LABELS = { left: 'Izquierdo', middle: 'Central', right: 'Derecho' };
  const lastRealIdx = actions.reduce((acc, a, i) => (a.type === 'delay' ? acc : i), -1);

  // Botones para subir/bajar un paso, deshabilitados en los extremos.
  const moveButtons = (idx, isFirst, isLast) => `
    <button class="tool-btn small" data-act="seq-move-up" data-index="${idx}" title="Subir paso" ${isFirst ? 'disabled' : ''}>
      ${icon('up', 14)}
    </button>
    <button class="tool-btn small" data-act="seq-move-down" data-index="${idx}" title="Bajar paso" ${isLast ? 'disabled' : ''}>
      ${icon('down', 14)}
    </button>`;

  // Cuántas veces se repite el gesto (mismo atajo o mismo clic seguido, ver
  // seqAddAction en macro-tabs.js): no añade pasos de verdad, solo repite este al ejecutarlo.
  const countField = (a, idx) => `
    <input type="number" class="text-input compact seq-count-input" min="1" max="99"
           data-act="seq-count" data-index="${idx}" value="${a.count || 1}" title="Veces">`;

  // Hueco entre cada repetición: solo tiene sentido a partir de 2 veces (con
  // 1 no hay nada que espaciar). Si la acción no trae uno propio (secuencias
  // guardadas antes de que esto existiera), se enseña el valor por defecto.
  const gapField = (a, idx) => a.count > 1 ? `
    <input type="number" class="text-input compact seq-gap-input" min="0" max="32767" step="5"
           data-act="seq-gap-ms" data-index="${idx}" value="${a.gap ?? DEFAULT_REPEAT_GAP_MS}" title="Espera entre repeticiones (ms)">` : '';

  let stepNum = 0;
  const items = actions.map((a, idx) => {
    // La espera entre dos pasos no es un paso más: es el hueco configurable
    // que pide siempre haber entre dos pasos consecutivos.
    if (a.type === 'delay') {
      return `
        <li class="seq-item seq-gap">
          <span>${icon('reset', 13)} Espera</span>
          <span class="row-inline" style="gap:4px">
            <input type="number" class="text-input compact seq-gap-input" min="0" step="10"
                   value="${a.ms}" data-act="seq-delay-ms" data-index="${idx}">
            <span class="setting-desc">ms</span>
          </span>
        </li>`;
    }

    stepNum++;

    if (a.type === 'mouse_position') {
      return `
        <li class="seq-item">
          <span>${stepNum}. Posición de ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-x" data-index="${idx}" value="${a.x}" title="x">
            <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-y" data-index="${idx}" value="${a.y}" title="y">
            <button class="tool-btn small" data-act="seq-recapture" data-index="${idx}" title="Recapturar con el ratón">
              ${icon('fit', 14)}
            </button>
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    // Formato antiguo, sin coordenadas propias: solo se puede borrar, no
    // editar ni recapturar (no hay paso nuevo de este tipo desde el editor).
    if (a.type === 'center_mouse') {
      return `
        <li class="seq-item">
          <span>${stepNum}. Centrar ratón</span>
          <span class="row-inline" style="gap:4px">
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    if (a.type === 'mouse_move') {
      return `
        <li class="seq-item">
          <span>${stepNum}. Mover ratón</span>
          <span class="row-inline" style="gap:6px">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dx" data-index="${idx}" value="${a.dx}" title="dx">
            <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
                   data-act="seq-move-dy" data-index="${idx}" value="${a.dy}" title="dy">
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    if (a.type === 'mouse_click') {
      const btn = a.button || 'left';
      return `
        <li class="seq-item">
          <span>${stepNum}. Clic ${a.count > 1 ? `×${a.count}` : ''}</span>
          <span class="row-inline" style="gap:6px">
            <select class="select-input compact" data-act="seq-click-button" data-index="${idx}">
              ${Object.entries(CLICK_LABELS).map(([value, label]) =>
                `<option value="${value}" ${btn === value ? 'selected' : ''}>${label}</option>`).join('')}
            </select>
            ${countField(a, idx)}
            ${gapField(a, idx)}
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    // Como el paso "Abrir", necesita apilar en vertical: el campo del texto va
    // debajo de la fila de título (de ahí seq-item-open, ver index.css).
    if (a.type === 'text') {
      return `
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${stepNum}. Escribir texto ${a.count > 1 ? `×${a.count}` : ''}</span>
            <span class="row-inline" style="gap:4px">
              ${countField(a, idx)}
              ${gapField(a, idx)}
              ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
                ${icon('trash', 14)}
              </button>
            </span>
          </div>
          <textarea class="text-input mt-4" rows="2" style="width:100%"
                    data-act="seq-text" data-index="${idx}"
                    placeholder="Texto que se escribirá">${escape(a.text || '')}</textarea>
        </li>`;
    }

    if (a.type === 'open_app') {
      const openKind = a.kind === 'file' ? 'file' : 'app';
      return `
        <li class="seq-item seq-item-open">
          <div class="row-inline" style="justify-content:space-between">
            <span>${stepNum}. Abrir</span>
            <span class="row-inline" style="gap:4px">
              ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
              <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
                ${icon('trash', 14)}
              </button>
            </span>
          </div>
          <div class="row-inline mt-4" style="gap:6px">
            <button class="type-chip small ${openKind === 'app' ? 'on' : ''}" data-act="seq-open-kind" data-index="${idx}" data-kind="app">Aplicación</button>
            <button class="type-chip small ${openKind === 'file' ? 'on' : ''}" data-act="seq-open-kind" data-index="${idx}" data-kind="file">Archivo</button>
          </div>
          ${openKind === 'app' ? `
            <div class="row-inline mt-4" style="gap:6px">
              <button class="tool-btn small" data-act="seq-open-focus" data-index="${idx}" title="Usar la app en primer plano ahora mismo">
                ${icon('fit', 14)} App en foco
              </button>
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${idx}" data-kind="app" title="Examinar…">
                ${icon('upload', 14)}
              </button>
            </div>
            <input type="text" class="text-input compact mt-4" style="width:100%" list="installed-apps-list"
                   data-act="seq-open-target" data-index="${idx}" value="${escape(a.target || '')}"
                   placeholder="Escribe para buscar entre las instaladas" title="${escape(a.target || '')}">
          ` : `
            <div class="row-inline mt-4" style="gap:6px">
              <input type="text" class="text-input compact" style="flex:1;min-width:120px"
                     data-act="seq-open-target" data-index="${idx}" value="${escape(a.target || '')}"
                     placeholder="Ruta del archivo" title="${escape(a.target || '')}">
              <button class="tool-btn small" data-act="seq-open-browse" data-index="${idx}" data-kind="file" title="Examinar…">
                ${icon('upload', 14)}
              </button>
            </div>
          `}
        </li>`;
    }

    if (a.type === 'hotkey') {
      return `
        <li class="seq-item">
          <span>${stepNum}. ${escape(describeAction(a.modifier, a.keycode))} ${a.count > 1 ? `×${a.count}` : ''}</span>
          <span class="row-inline" style="gap:4px">
            ${countField(a, idx)}
            ${gapField(a, idx)}
            ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
            <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
              ${icon('trash', 14)}
            </button>
          </span>
        </li>`;
    }

    // a.type === 'key' (grabado pulsando físicamente, sin modificadores)
    return `
      <li class="seq-item">
        <span>${stepNum}. Tecla ${escape(a.code)}</span>
        <span class="row-inline" style="gap:4px">
          ${moveButtons(idx, stepNum === 1, idx === lastRealIdx)}
          <button class="tool-btn danger small" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
            ${icon('trash', 14)}
          </button>
        </span>
      </li>`;
  }).join('');

  return `
    <div class="field">
      <span class="field-label">Pasos de la secuencia</span>
      ${actions.length ? `<ul class="seq-list">${items}</ul>`
                        : `<p class="setting-desc">Todavía no tiene ningún paso.</p>`}
      ${installedAppsDatalist()}

      <div class="row-inline mt-4">
        <button class="secondary-btn ${capturingPos ? 'is-capturing' : ''}" data-act="seq-add-position">
          ${icon('fit', 16)} ${capturingPos ? (capturePosEditIndex !== null ? 'Recapturando… (Esc fija)' : 'Capturando… (Esc fija)') : 'Posición de ratón'}
        </button>
        <button class="secondary-btn" data-act="seq-add-click">${icon('bolt', 16)} Clic</button>
        <button class="secondary-btn" data-act="seq-add-move">${icon('reset', 16)} Mover ratón</button>
        <button class="secondary-btn" data-act="seq-add-text">${icon('pencil', 16)} Escribir texto</button>
        <button class="secondary-btn" data-act="seq-add-open">${icon('upload', 16)} Abrir app/archivo</button>
      </div>
      ${capturingPos ? `<p class="setting-desc" id="seq-live-pos">(${lastMousePos.x}, ${lastMousePos.y}) — mueve el ratón y pulsa Esc para fijarla</p>` : ''}

      <span class="field-label mt-4">Tecla (con modificadores)</span>
      <div class="mod-grid">
        ${MODIFIERS.map((m) => `
          <button class="mod-chip ${seqKeyBuilder.modifier & m.bit ? 'on' : ''}"
                  data-act="seq-key-mod" data-bit="${m.bit}">${m.label}</button>`).join('')}
      </div>
      <div class="row-inline mt-4">
        <select class="select-input" data-act="seq-key-pick" style="flex:1">
          <option value="0" ${!seqKeyBuilder.keycode ? 'selected' : ''}>— ninguna —</option>
          ${keycodeOptions(seqKeyBuilder.keycode)}
        </select>
        <button class="secondary-btn" data-act="seq-add-hotkey">${icon('plus', 16)} Añadir</button>
      </div>

      <button class="primary-btn full mt-4 ${recording ? 'is-capturing' : ''}" data-act="seq-record">
        ${icon('key', 16)} ${recording ? 'Grabando… pulsa teclas (Esc termina)' : 'Grabar secuencia de teclas'}
      </button>

      ${macro ? renderSequenceLocation(macro) : ''}

      <p class="setting-desc">
        "Grabar secuencia" solo reconoce letras, dígitos, Enter y Espacio; para el resto de
        teclas o combinaciones con modificadores usa "Tecla" arriba, y para meter un texto tal
        cual (correo, firma, un trozo de código) "Escribir texto". Entre cada dos pasos se
        espera automáticamente lo que pongas en "Espera" (${DEFAULT_STEP_DELAY_MS} ms por
        defecto suele bastar).
      </p>

      <button class="secondary-btn full" data-act="seq-clear">${icon('trash', 16)} Quitar secuencia</button>
    </div>`;
}

// Dice si esta secuencia la toca el propio teclado (sigue funcionando aunque
// cierres la app) o si necesita el PC, y por qué: espera, tecla, clic (con
// repeticiones incluidas, ver macro_repeat_or_advance en main.cpp) y
// movimiento relativo sí se suben; posición de ratón absoluta, de momento no
// (ver TODO(homing-absoluto) junto a DEVICE_STEP_TYPE) — igual que el formato
// antiguo "centrar ratón" o tener más pasos de los que caben en el teclado.
export function renderSequenceLocation(macro) {
  if (macroDeviceEligible(macro)) {
    return `<p class="setting-desc">${icon('check', 13)} Se ejecuta en el propio teclado: sigue
              funcionando aunque cierres esta app.</p>`;
  }
  const needsOpen = (macro.actions || []).some((a) => a.type === 'open_app');
  const needsText = (macro.actions || []).some((a) => a.type === 'text');
  const needsPos = (macro.actions || []).some((a) => a.type === 'mouse_position' || a.type === 'center_mouse');
  const reason = needsOpen
    ? 'abre una app o un archivo, algo que solo sabe hacer el PC'
    : needsText
    ? 'escribe un texto, y eso el PC lo manda como unicode: el teclado solo sabe mandar códigos de tecla, que cambian con la distribución'
    : needsPos
    ? 'usa una posición de ratón absoluta, que de momento solo sabe reproducir el PC'
    : `tiene más de ${MACRO_MAX_STEPS_DEVICE} pasos, o alguno de un formato antiguo`;
  return `<p class="setting-desc">Se ejecuta en el PC, no en el teclado (${reason}): solo
            funciona con esta app abierta.</p>`;
}

// Las dos acciones de página. Como las multimedia, no llevan modificadores ni
