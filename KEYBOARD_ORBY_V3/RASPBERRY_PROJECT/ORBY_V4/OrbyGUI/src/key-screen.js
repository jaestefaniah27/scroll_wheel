// Qué enseña una pantalla del teclado, decidido como lo decide render_screen()
// en main.cpp, para pintarlo en la app sin preguntárselo al teclado.
//
// Hace falta donde no hay espejo de las pantallas: la réplica con un firmware
// sin MIRROR o con la copia del PC, la vista previa de la capa SUPER mientras
// se edita, la pantalla grande del panel y los iconos de perfil. Antes solo se
// sabía pintar el icono subido; lo demás —todos los perfiles de la biblioteca—
// salía como la etiqueta en texto (WIDTH, RTMODE…).

import { pageCountOf, labelSlot, keymapSlot } from './store.js';
import { PAGE_STATE_MODIFIER } from './hid-keys.js';
import * as cache from './oled-cache.js';
import * as fb from './oled-fb.js';
import * as ui from './oled-ui.mjs';

// Pantalla de la tecla `keyIndex` (base 0) del perfil `profileIdx` en su página
// `page` y la capa `layer`, o null si la tecla no tiene pantalla. `over` deja
// pasar la acción y la etiqueta de una variación en vez de las del perfil.
export function keyScreen(prof, profileIdx, keyIndex, layer, page, over = {}) {
  const slot = labelSlot(keyIndex, layer);
  if (slot < 0 || !prof) return null;
  const action = over.action ?? prof.keys?.[keymapSlot(keyIndex, layer)];
  // El estado de páginas manda sobre el icono: la tecla dice en qué página estás.
  if (action?.modifier === PAGE_STATE_MODIFIER) {
    const total = pageCountOf(prof);
    return ui.screenDots(page, total, `Página ${page + 1} de ${total}`);
  }
  const bmp = cache.get(profileIdx, slot, page);
  if (bmp) return bmp;
  return ui.screenKey(prof.name, over.label ?? prof.labels?.[slot] ?? '');
}

// Icono de un perfil: el subido, o el de fábrica si su nombre está en el
// catálogo (los de la biblioteca lo están). null si no tiene ninguno: entonces
// la app pone su baldosa de color, que se lee mejor que el nombre en pequeño.
export function profileScreen(prof, profileIdx) {
  const bmp = cache.profileIcon(profileIdx);
  if (bmp) return bmp;
  return prof && ui.lookupProfile(prof.name) >= 0 ? ui.screenProfile(prof.name) : null;
}

// Pinta en cada <canvas data-profile-screen="i"> el icono del perfil i. Va
// aparte de cache.paintThumbs porque el de fábrica no está en la caché.
export function paintProfileScreens(root, profiles) {
  root.querySelectorAll('canvas[data-profile-screen]').forEach((cv) => {
    const i = Number(cv.dataset.profileScreen);
    const s = profileScreen(profiles[i], i);
    if (s) fb.renderToCanvas(s, cv, { zoom: 1, grid: false });
  });
}
