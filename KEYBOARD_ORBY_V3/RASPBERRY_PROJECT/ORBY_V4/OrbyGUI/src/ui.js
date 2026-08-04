// Avisos efímeros. La app no tenía forma de confirmar que un comando había
// llegado al teclado más allá de mirar la consola.

import { icon } from './icons.js';
import { state } from './store.js';

const ICONS = { success: 'check', error: 'close', info: 'info' };

export function toast(message, kind = 'success', ms = 2600) {
  let host = document.getElementById('toast-host');
  if (!host) {
    host = document.createElement('div');
    host.id = 'toast-host';
    document.body.appendChild(host);
  }

  const el = document.createElement('div');
  el.className = `toast toast-${kind}`;
  el.innerHTML = `${icon(ICONS[kind] || 'info', 16)}<span>${message}</span>`;
  host.appendChild(el);

  setTimeout(() => {
    el.classList.add('leaving');
    setTimeout(() => el.remove(), 200);
  }, ms);
}

// Portero de la edición. Sin el Orby conectado la app enseña la copia local del
// PC en modo lectura: dejar cambiarla crearía dos versiones distintas de la
// misma configuración, y al reconectar habría que decidir cuál se tira.
//
// Se comprueba ANTES de tocar el modelo, no después de que falle el comando:
// las vistas cambian primero lo suyo y lo mandan después, así que enterarse por
// el error dejaría la pantalla enseñando algo que el teclado no tiene.
export function requireDevice() {
  if (state.connected) return true;
  toast('Conecta el Orby por USB para poder editar la configuración', 'error', 4000);
  return false;
}
