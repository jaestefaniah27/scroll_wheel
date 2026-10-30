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

// Pregunta de confirmación con el aspecto de una alerta de iOS.
//
// Sustituye a confirm(): en la app de escritorio WebView2 lo pinta como un
// diálogo de navegador con "localhost dice" en el título, y en la WebGUI cada
// navegador lo pinta a su manera. Devuelve una promesa, así que quien pregunta
// tiene que esperarla con await.
//
// Todo el texto entra con textContent y no con innerHTML: varios mensajes llevan
// el nombre de un perfil o de un complemento, que escribe el usuario.
export function confirmar({ titulo, mensaje = '', aceptar = 'Aceptar', cancelar = 'Cancelar', peligro = false }) {
  return new Promise((resolve) => {
    const antes = document.activeElement;

    const fondo = document.createElement('div');
    fondo.className = 'alert-backdrop';
    const caja = document.createElement('div');
    caja.className = 'alert';
    caja.setAttribute('role', 'alertdialog');
    caja.setAttribute('aria-modal', 'true');

    const h = document.createElement('h2');
    h.textContent = titulo;
    caja.appendChild(h);
    if (mensaje) {
      const p = document.createElement('p');
      p.textContent = mensaje;
      caja.appendChild(p);
    }

    const botones = document.createElement('div');
    botones.className = 'alert-btns';
    const no = document.createElement('button');
    no.textContent = cancelar;
    const si = document.createElement('button');
    si.textContent = aceptar;
    si.className = peligro ? 'is-danger' : 'is-default';
    botones.append(no, si);
    caja.appendChild(botones);
    fondo.appendChild(caja);
    document.body.appendChild(fondo);

    let hecho = false;
    const cerrar = (valor) => {
      if (hecho) return;
      hecho = true;
      document.removeEventListener('keydown', teclas, true);
      fondo.classList.add('leaving');
      setTimeout(() => fondo.remove(), 200);
      antes?.focus?.();
      resolve(valor);
    };
    // En captura y cortando la propagación: si no, Intro y Escape llegarían
    // también a la vista de debajo (la consola, el editor de iconos).
    const teclas = (e) => {
      if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); cerrar(false); }
      else if (e.key === 'Enter') { e.preventDefault(); e.stopPropagation(); cerrar(document.activeElement !== no); }
    };

    no.addEventListener('click', () => cerrar(false));
    si.addEventListener('click', () => cerrar(true));
    fondo.addEventListener('mousedown', (e) => { if (e.target === fondo) cerrar(false); });
    document.addEventListener('keydown', teclas, true);

    // Como en iOS, una acción destructiva nunca es la de por defecto: el foco
    // empieza en Cancelar y un Intro distraído no borra nada.
    (peligro ? no : si).focus();
  });
}

// Tramo recorrido de los deslizadores (ver `.premium-slider` en index.css).
// CSS no sabe leer el valor de un <input type=range>, así que se le pasa en una
// variable. Va por delegación y con un observador porque las vistas pintan sus
// controles con innerHTML en cada render: engancharse a cada input se perdería
// en el siguiente repintado. Se exporta para quien cambie el valor por código:
// eso no dispara `input` y el tramo se quedaría donde estaba.
export function pintarDeslizador(el) {
  const min = Number(el.min) || 0;
  const max = Number(el.max) || 100;
  const pct = max > min ? ((Number(el.value) - min) / (max - min)) * 100 : 0;
  el.style.setProperty('--fill-pct', `${Math.max(0, Math.min(100, pct))}%`);
}

function pintarDeslizadores(raiz) {
  raiz.querySelectorAll?.('input[type="range"]').forEach(pintarDeslizador);
}

document.addEventListener('input', (e) => {
  if (e.target instanceof HTMLInputElement && e.target.type === 'range') pintarDeslizador(e.target);
});

new MutationObserver((cambios) => {
  for (const c of cambios) {
    for (const n of c.addedNodes) {
      if (n.nodeType !== 1) continue;
      if (n.matches('input[type="range"]')) pintarDeslizador(n);
      else pintarDeslizadores(n);
    }
  }
}).observe(document.body, { childList: true, subtree: true });

pintarDeslizadores(document);
