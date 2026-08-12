// La superficie `window.orby` es EL contrato entre el renderer y quien haya detrás.
// Hay dos implementaciones (Tauri y navegador) y el renderer no distingue: si una se
// deja un método, la vista que lo llame revienta solo en esa vía, y encima tarde, cuando
// el usuario pulsa el botón concreto.
//
// **La referencia es `src/tauri/orby-tauri.js`**, y lo es desde que se retiró Electron
// (Tarea 13): antes lo era `electron/preload.js`, que era de donde venía el contrato.
// Al borrarlo hubo que reanclar esto y `tools/test/verifica_plan_tauri.sh`, que leían
// aquella carpeta como fuente de verdad.
//
// Este test las compara leyendo el código, sin arrancar ninguna: no hay forma de importar
// el puente de Tauri fuera de Tauri.

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

const tauri = superficie(lee('src/tauri/orby-tauri.js'));
const web = superficie(lee('src/web/orby-web.js'));

test('la superficie de Tauri es la que se espera', () => {
  // Red de seguridad: si alguien añade un método al puente sin tocar la vía navegador,
  // este número cambia y el test de abajo dice exactamente cuál falta.
  assert.ok(tauri.has('sendCommand'), 'debería tener sendCommand');
  assert.ok(tauri.has('plugins.list'), 'debería tener plugins.list');
  assert.ok(tauri.has('firmware.onState'), 'debería tener firmware.onState');
  assert.ok(tauri.size > 40, `esperaba más de 40 métodos, hay ${tauri.size}`);
});

test('el navegador cubre la superficie de Tauri', () => {
  // La vía navegador puede *no poder* hacer algo (ver PC_ONLY en platform.js), pero el
  // método tiene que existir igual y devolver vacío: si falta, la vista revienta al
  // llamarlo en vez de esconder el botón.
  const faltan = [...tauri].filter((m) => !web.has(m));
  assert.deepEqual(faltan, [], `a orby-web.js le faltan: ${faltan.join(', ')}`);
});

test('las dos vías declaran su plataforma', () => {
  assert.match(lee('src/tauri/orby-tauri.js'), /platform:\s*'tauri'/);
  assert.match(lee('src/web/orby-web.js'), /platform:\s*'web'/);
});

test('entry.js reparte entre las dos vías que quedan', () => {
  // Sin los comentarios: la cabecera del fichero nombra las dos cosas al explicarlo.
  const codigo = lee('src/entry.js')
    .split('\n').filter((l) => !l.trim().startsWith('//')).join('\n');

  assert.ok(codigo.includes('__TAURI_INTERNALS__'), 'entry.js debería detectar Tauri');
  assert.ok(codigo.includes('tauri-main.js'), 'entry.js debería arrancar la vía Tauri');
  assert.ok(codigo.includes('web-main.js'), 'entry.js debería arrancar la vía navegador');
  // Electron se retiró en la Tarea 13: si esta rama vuelve, es que alguien la resucitó
  // a medias.
  assert.ok(!codigo.includes('window.orby'), 'ya no debería quedar rama de Electron');
});
