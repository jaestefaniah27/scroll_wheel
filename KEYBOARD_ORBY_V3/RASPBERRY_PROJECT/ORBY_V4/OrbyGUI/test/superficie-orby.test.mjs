// La superficie `window.orby` es EL contrato entre el renderer y quien haya detrás.
// Hay tres implementaciones (Electron, Tauri y navegador) y el renderer no distingue:
// si una se deja un método, la vista que lo llame revienta solo en esa vía, y encima
// tarde, cuando el usuario pulsa el botón concreto.
//
// Este test las compara entre sí leyendo el código, sin arrancar ninguna: no hay forma
// de importar el preload de Electron fuera de Electron ni el de Tauri fuera de Tauri.

import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const raiz = join(dirname(fileURLToPath(import.meta.url)), '..');
const lee = (p) => readFileSync(join(raiz, p), 'utf8');

// Saca los nombres de método de un objeto literal, incluidos los de los espacios de
// nombres, que se marcan como `espacio.metodo` para no confundir `plugins.list` con un
// hipotético `list` suelto.
function superficie(codigo) {
  const nombres = new Set();
  let espacio = null;
  let profundidad = 0;

  for (const linea of codigo.split('\n')) {
    const limpia = linea.trim();
    if (limpia.startsWith('//')) continue;

    // Un espacio de nombres abre con `nombre: {` y nada más en la línea.
    const abre = limpia.match(/^(\w+):\s*\{$/);
    if (abre) { espacio = abre[1]; profundidad = 1; continue; }

    // Una propiedad es `nombre:` o la forma abreviada `nombre,` (orby-web.js reexporta
    // así varias funciones que importa de sus módulos).
    const propiedad = limpia.match(/^(\w+)\s*[:,]/) || limpia.match(/^(\w+),$/);

    if (espacio) {
      if (limpia.startsWith('}')) { espacio = null; profundidad = 0; continue; }
      if (propiedad) nombres.add(`${espacio}.${propiedad[1]}`);
      continue;
    }

    if (profundidad === 0 && propiedad) nombres.add(propiedad[1]);
  }
  return nombres;
}

const electron = superficie(lee('electron/preload.js'));
const tauri = superficie(lee('src/tauri/orby-tauri.js'));
const web = superficie(lee('src/web/orby-web.js'));

test('la superficie de Electron es la que se espera', () => {
  // Red de seguridad: si alguien añade un método al preload sin tocar las otras vías,
  // este número cambia y el test de abajo dice exactamente cuál falta.
  assert.ok(electron.has('sendCommand'), 'debería tener sendCommand');
  assert.ok(electron.has('plugins.list'), 'debería tener plugins.list');
  assert.ok(electron.has('firmware.onState'), 'debería tener firmware.onState');
  assert.ok(electron.size > 40, `esperaba más de 40 métodos, hay ${electron.size}`);
});

test('Tauri implementa toda la superficie de Electron', () => {
  const faltan = [...electron].filter((m) => !tauri.has(m));
  assert.deepEqual(faltan, [], `a orby-tauri.js le faltan: ${faltan.join(', ')}`);
});

test('Tauri no inventa métodos que Electron no tiene', () => {
  // Un método de más es tan malo como uno de menos: alguna vista acabaría usándolo y
  // dejaría de funcionar en las otras dos vías.
  const sobran = [...tauri].filter((m) => !electron.has(m));
  assert.deepEqual(sobran, [], `orby-tauri.js tiene de más: ${sobran.join(', ')}`);
});

test('el navegador sigue cubriendo la superficie de Electron', () => {
  // No es cosa de esta migración, pero si se rompe conviene enterarse aquí y no por un
  // usuario con la WebGUI abierta.
  const faltan = [...electron].filter((m) => !web.has(m));
  assert.deepEqual(faltan, [], `a orby-web.js le faltan: ${faltan.join(', ')}`);
});

test('las tres vías declaran su plataforma', () => {
  assert.match(lee('electron/preload.js'), /platform:\s*'electron'/);
  assert.match(lee('src/tauri/orby-tauri.js'), /platform:\s*'tauri'/);
  assert.match(lee('src/web/orby-web.js'), /platform:\s*'web'/);
});

test('entry.js comprueba Tauri antes que Electron', () => {
  // Si se invirtiera el orden y Tauri montara algún día su propio window.orby, la rama
  // de Electron se lo tragaría y arrancaría por la vía equivocada en silencio.
  // Sin los comentarios: la cabecera del fichero nombra las dos cosas al explicarlo.
  const codigo = lee('src/entry.js')
    .split('\n').filter((l) => !l.trim().startsWith('//')).join('\n');

  const posTauri = codigo.indexOf('__TAURI_INTERNALS__');
  const posOrby = codigo.indexOf('window.orby');
  assert.ok(posTauri !== -1, 'entry.js debería detectar Tauri');
  assert.ok(posOrby !== -1, 'entry.js debería detectar Electron');
  assert.ok(posTauri < posOrby, 'la comprobación de Tauri debe ir antes que la de Electron');
});
