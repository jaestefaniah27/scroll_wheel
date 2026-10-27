// Dónde se ejecuta una macro, en una frase para el inspector: en el propio
// teclado (sigue funcionando con la app cerrada), en el teclado salvo con la app
// abierta (cuando el PC lo hace mejor: pega el texto, abre sin el diálogo
// Ejecutar…), o solo en el PC, y en ese caso por qué. La decisión es de
// compileMacro (src/native/compile.mjs); aquí solo se cuenta.
//
// Va aparte para que la usen las pestañas Secuencia, Texto, App y Grabar sin
// importarse unas a otras.

import { icon } from '../../icons.js';
import { macroDeviceBlocker, macroPrefersPc } from './macros-store.js';
import { escape } from './util.js';

export function renderWhereItRuns(macro) {
  if (!macro) return '';
  const blocker = macroDeviceBlocker(macro);
  if (blocker === null && macroPrefersPc(macro)) {
    return `<p class="setting-desc">${icon('check', 13)} Con esta app abierta lo hace el PC; sin
              ella, el propio teclado, así que funciona igual con la app cerrada.</p>`;
  }
  if (blocker === null) {
    return `<p class="setting-desc">${icon('check', 13)} Lo hace el propio teclado: sigue
              funcionando aunque cierres esta app.</p>`;
  }
  return `<p class="setting-desc">Lo hace el PC, no el teclado (${escape(blocker)}): solo
            funciona con esta app abierta.</p>`;
}
