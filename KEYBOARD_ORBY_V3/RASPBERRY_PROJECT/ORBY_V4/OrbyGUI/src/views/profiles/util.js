// Ayudas sueltas del editor de perfiles: cosas que usan varios de sus módulos
// y que no son de ninguno en concreto.

import { KEY_GROUPS } from '../../hid-keys.js';

// Qué ventana está en primer plano. El detector lo lleva el proceso principal y
// puede no estar arrancado todavía, así que se le da un empujón y se le dan
// unos intentos antes de rendirse: es lo que pasa la primera vez que se usa.
export async function getCurrentWindowInfo() {
  let info = await window.orby.foreground.current();
  if (info) return info;

  await window.orby.foreground.start();
  for (let i = 0; i < 8 && !info; i++) {
    await new Promise((r) => setTimeout(r, 300));
    info = await window.orby.foreground.current();
  }
  return info;
}

// El <select> de teclas, agrupado igual que en la pestaña Atajo.
export function keycodeOptions(selectedCode) {
  return KEY_GROUPS.map((g) => `
    <optgroup label="${g.name}">
      ${g.keys.map((k) => `<option value="${k.code}" ${selectedCode === k.code ? 'selected' : ''}>${escape(k.label)}</option>`).join('')}
    </optgroup>`).join('');
}

export function escape(s) {
  return String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
