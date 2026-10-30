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
         capturePosEditIndex, lastMousePos, stepUnits, seqInsertAt } from './macro-tabs.js';
import { escape, keycodeOptions } from './util.js';
import * as platform from '../../platform.js';
import * as compat from '../../compat.js';

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
  const units = stepUnits(actions);
  const insertAt = seqInsertAt();

  // Cuántas veces se repite el gesto (mismo atajo o mismo clic seguido, ver
  // seqAddAction en macro-tabs.js): no añade pasos de verdad, solo repite este al ejecutarlo.
  const countField = (a, idx) => `
    <span class="seq-count" title="Veces">×<input type="number" class="text-input compact seq-count-input" min="1" max="99"
           data-act="seq-count" data-index="${idx}" value="${a.count || 1}"></span>`;

  // Hueco entre cada repetición: solo tiene sentido a partir de 2 veces (con
  // 1 no hay nada que espaciar). Si la acción no trae uno propio (secuencias
  // guardadas antes de que esto existiera), se enseña el valor por defecto.
  const gapField = (a, idx) => a.count > 1 ? `
    <input type="number" class="text-input compact seq-gap-input" min="0" max="32767" step="5"
           data-act="seq-gap-ms" data-index="${idx}" value="${a.gap ?? DEFAULT_REPEAT_GAP_MS}" title="Espera entre repeticiones (ms)">` : '';

  // Lo que cambia de un tipo de paso a otro: el nombre y sus campos.
  const body = (a, idx) => {
    if (a.type === 'mouse_position') return {
      title: 'Posición',
      fields: `
        <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-x" data-index="${idx}" value="${a.x}" title="x">
        <input type="number" class="text-input compact seq-pos-input" data-act="seq-pos-y" data-index="${idx}" value="${a.y}" title="y">
        <button class="tool-btn small" data-act="seq-recapture" data-index="${idx}" title="Recapturar con el ratón">
          ${icon('fit', 14)}
        </button>`,
    };
    // Formato antiguo, sin coordenadas propias: solo se puede borrar, no
    // editar ni recapturar (no hay paso nuevo de este tipo desde el editor).
    if (a.type === 'center_mouse') return { title: 'Centrar ratón', fields: '' };
    if (a.type === 'mouse_move') return {
      title: 'Mover ratón',
      fields: `
        <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
               data-act="seq-move-dx" data-index="${idx}" value="${a.dx}" title="dx">
        <input type="number" class="text-input compact seq-pos-input" min="-127" max="127"
               data-act="seq-move-dy" data-index="${idx}" value="${a.dy}" title="dy">`,
    };
    if (a.type === 'mouse_click') {
      const btn = a.button || 'left';
      return {
        title: 'Clic',
        fields: `
          <select class="select-input compact" data-act="seq-click-button" data-index="${idx}">
            ${Object.entries(CLICK_LABELS).map(([value, label]) =>
              `<option value="${value}" ${btn === value ? 'selected' : ''}>${label}</option>`).join('')}
          </select>
          ${countField(a, idx)}
          ${gapField(a, idx)}`,
      };
    }
    // El campo ocupa lo que sobre de la fila, en vez de ir debajo: una línea
    // basta para casi todo y se puede estirar si hace falta.
    if (a.type === 'text') return {
      title: 'Texto',
      wide: true,
      fields: `
        <textarea class="text-input compact seq-text-input" rows="1"
                  data-act="seq-text" data-index="${idx}"
                  placeholder="Texto que se escribirá">${escape(a.text || '')}</textarea>
        ${countField(a, idx)}
        ${gapField(a, idx)}`,
    };
    if (a.type === 'open_app') {
      const openKind = a.kind === 'file' ? 'file' : 'app';
      return {
        title: 'Abrir',
        wide: true,
        fields: `
          <span class="seq-seg">
            <button class="type-chip small ${openKind === 'app' ? 'on' : ''}" data-act="seq-open-kind" data-index="${idx}" data-kind="app">App</button>
            <button class="type-chip small ${openKind === 'file' ? 'on' : ''}" data-act="seq-open-kind" data-index="${idx}" data-kind="file">Archivo</button>
          </span>
          ${openKind === 'app' ? `
            <input type="text" class="text-input compact seq-open-input" list="installed-apps-list"
                   data-act="seq-open-target" data-index="${idx}" value="${escape(a.target || '')}"
                   placeholder="Busca entre las instaladas" title="${escape(a.target || '')}">
            <button class="tool-btn small" data-act="seq-open-focus" data-index="${idx}" title="Usar la app en primer plano ahora mismo">
              ${icon('fit', 14)}
            </button>
          ` : `
            <input type="text" class="text-input compact seq-open-input"
                   data-act="seq-open-target" data-index="${idx}" value="${escape(a.target || '')}"
                   placeholder="Ruta del archivo" title="${escape(a.target || '')}">
          `}
          <button class="tool-btn small" data-act="seq-open-browse" data-index="${idx}" data-kind="${openKind}" title="Examinar…">
            ${icon('upload', 14)}
          </button>`,
      };
    }
    if (a.type === 'hotkey') return {
      title: `<kbd class="seq-kbd">${escape(describeAction(a.modifier, a.keycode))}</kbd>`,
      fields: `${countField(a, idx)}${gapField(a, idx)}`,
    };
    // a.type === 'key' (grabado pulsando físicamente, sin modificadores)
    return { title: `<kbd class="seq-kbd">${escape(a.code)}</kbd>`, fields: '' };
  };

  // Cada paso es una sola fila: asa para arrastrarlo, número, nombre, sus
  // campos, la espera hasta el siguiente (dentro del paso, no en una fila
  // aparte) y borrar. En el borde de arriba lleva una franja invisible que, al
  // pasar por encima, ofrece "añadir aquí"; la del final va en el último.
  const items = units.map((u, k) => {
    const { action: a, idx } = u;
    const { title, fields, wide } = body(a, idx);
    const last = k === units.length - 1;
    const wait = last ? '' : `
      <label class="seq-wait" title="Espera antes del paso siguiente (ms)">
        ${icon('clock', 12)}
        <input type="number" class="text-input compact" min="0" step="10"
               data-act="seq-wait" data-index="${idx}" value="${(u.delayIdx !== null ? actions[u.delayIdx].ms || 0 : 0) + u.extra}">
        <span>ms</span>
      </label>`;
    const marks = `${insertAt === k ? 'insert-before' : ''} ${wide ? 'seq-item-wide' : ''}`;
    return `
      <li class="seq-item ${marks}" data-step="${k}">
        <button class="seq-insert-btn" data-act="seq-insert-at" data-at="${k}" title="Añadir los pasos nuevos aquí" tabindex="-1">
          <span>${icon('plus', 10)}</span>
        </button>
        ${last ? `
          <button class="seq-insert-btn after" data-act="seq-insert-at" data-at="end" title="Añadir los pasos nuevos al final" tabindex="-1">
            <span>${icon('plus', 10)}</span>
          </button>` : ''}
        <span class="seq-handle" data-drag="seq-step" data-step="${k}" title="Arrastra para mover el paso">${icon('grip', 14)}</span>
        <span class="seq-num">${k + 1}</span>
        <span class="seq-title">${title}</span>
        <span class="seq-fields">${fields}</span>
        ${wait}
        <button class="tool-btn danger small seq-del" data-act="seq-del" data-index="${idx}" title="Quitar este paso">
          ${icon('trash', 14)}
        </button>
      </li>`;
  }).join('');

  // Aviso de dónde caerán los pasos nuevos, junto a los botones de añadir:
  // si no, al elegir un hueco a mitad de una lista larga no se vería qué pasa.
  const insertHint = insertAt === null
    ? (units.length ? `<p class="seq-insert-hint">Los pasos nuevos se añaden al final. Pasa el ratón entre dos pasos y pulsa ${icon('plus', 11)} para meterlos en medio.</p>` : '')
    : `<p class="seq-insert-hint on">${icon('plus', 11)} Añadiendo ${insertAt === 0 ? 'al principio' : `después del paso ${insertAt}`}
         <button class="seq-link-btn" data-act="seq-insert-at" data-at="end">Añadir al final</button></p>`;

  return `
    <div class="field">
      <span class="field-label">Pasos de la secuencia</span>
      ${units.length ? `<ul class="seq-list" id="seq-list">${items}</ul>`
                      : `<p class="setting-desc">Todavía no tiene ningún paso.</p>`}
      ${installedAppsDatalist()}
      ${insertHint}

      <div class="row-inline mt-4">
        <button class="secondary-btn ${capturingPos ? 'is-capturing' : ''} ${platform.can('pcSequences') ? '' : 'unsupported'}"
                ${platform.can('pcSequences') ? '' : 'disabled title="Necesita Orvy de escritorio"'}
                data-act="seq-add-position">
          ${icon('fit', 16)} ${capturingPos ? (capturePosEditIndex !== null ? 'Recapturando… (Esc fija)' : 'Capturando… (Esc fija)') : 'Posición de ratón'}
        </button>
        <button class="secondary-btn" data-act="seq-add-click">${icon('bolt', 16)} Clic</button>
        <button class="secondary-btn" data-act="seq-add-move">${icon('reset', 16)} Mover ratón</button>
        <button class="secondary-btn" data-act="seq-add-text">${icon('pencil', 16)} Escribir texto</button>
        <button class="secondary-btn ${platform.can('openApp') ? '' : 'unsupported'}"
                ${platform.can('openApp') ? '' : 'disabled title="Necesita Orvy de escritorio"'}
                data-act="seq-add-open">${icon('upload', 16)} Abrir app/archivo</button>
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
        cual (correo, firma, un trozo de código) "Escribir texto". Cada paso espera lo que
        marque su reloj antes de pasar al siguiente (${DEFAULT_STEP_DELAY_MS} ms por defecto
        suele bastar). Arrastra un paso por su asa para cambiarlo de sitio.
      </p>

      <button class="secondary-btn full" data-act="seq-clear">${icon('trash', 16)} Quitar secuencia</button>
    </div>`;
}

// Por qué un texto no lo teclea el teclado: o el firmware es anterior a la 4.20, o
// lleva algún carácter sin tecla en la distribución española (emojis, otros
// alfabetos), o no cabe en los pasos de una secuencia.
function textoSinTeclado() {
  return compat.supports(state.deviceInfo, 'macroText')
    ? `escribe un texto con caracteres que el teclado no sabe teclear (solo la distribución española, sin emojis ni otros alfabetos) o demasiado largo para sus ${MACRO_MAX_STEPS_DEVICE} pasos`
    : 'escribe un texto, y el teclado solo aprendió a teclearlo en el firmware 4.20: el PC lo manda como unicode';
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
    ? textoSinTeclado()
    : needsPos
    ? 'usa una posición de ratón absoluta, que de momento solo sabe reproducir el PC'
    : `tiene más de ${MACRO_MAX_STEPS_DEVICE} pasos, o alguno de un formato antiguo`;
  return `<p class="setting-desc">Se ejecuta en el PC, no en el teclado (${reason}): solo
            funciona con esta app abierta.</p>`;
}

// Las dos acciones de página. Como las multimedia, no llevan modificadores ni
