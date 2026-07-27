// Avisos efímeros. La app no tenía forma de confirmar que un comando había
// llegado al teclado más allá de mirar la consola.

import { icon } from './icons.js';

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
