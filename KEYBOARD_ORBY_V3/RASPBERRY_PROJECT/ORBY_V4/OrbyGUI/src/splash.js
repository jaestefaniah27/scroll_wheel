// Pantalla de carga del arranque.
//
// El marcado y sus estilos van EN LÍNEA en index.html: la hoja de estilos de la
// app la importa este mismo módulo de JavaScript, así que llega después del
// primer pintado y durante ese hueco se veía la interfaz sin estilar. Aquí solo
// está la lógica de cuándo quitarla.
//
// Se quita cuando el arranque ha terminado de verdad: la copia del PC ya está
// pintada y el teclado ha tenido su oportunidad de contestar. Así lo primero
// que se ve es la app entera y estable, sin perfiles apareciendo de uno en uno.

let el = null;
let msgEl = null;
let done = false;

// Tope duro. Si algo se atasca (el teclado no contesta, una vista revienta al
// pintarse) la pantalla de carga se quita igual: quedarse detrás de un logo es
// peor que ver la app a medias.
const DEADLINE_MS = 7000;

export function init() {
  el = document.getElementById('app-splash');
  msgEl = document.getElementById('splash-msg');
  if (el) setTimeout(hide, DEADLINE_MS);
}

export function visible() {
  return Boolean(el) && !done;
}

export function status(text) {
  if (msgEl && !done) msgEl.textContent = text;
}

export function hide() {
  if (done || !el) return;
  done = true;
  el.classList.add('gone');
  // Se espera a que acabe el fundido antes de sacarlo del DOM: quitarlo de
  // golpe se lleva la transición por delante.
  setTimeout(() => { el?.remove(); el = null; msgEl = null; }, 320);
}
