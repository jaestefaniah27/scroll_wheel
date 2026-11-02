// Los juegos que trae la app. Añadir uno es crear su fichero (ver engine.js para lo que
// tiene que exponer) y ponerlo aquí; la vista y las puntuaciones no necesitan más.

import topos from './topos.mjs';
import simon from './simon.mjs';
import sintoniza from './sintoniza.mjs';
import reflejos from './reflejos.mjs';
import misil from './misil.mjs';

export const JUEGOS = [topos, simon, sintoniza, reflejos, misil];

export function juegoPorId(id) {
  return JUEGOS.find((j) => j.id === id) || null;
}
