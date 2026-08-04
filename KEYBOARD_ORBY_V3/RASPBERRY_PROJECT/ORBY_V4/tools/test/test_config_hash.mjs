// Compara la huella que calcula la app con la que calcula el firmware.
//
// Fabrica la misma configuración de mentira que test_config_hash.cpp (mismo
// generador, mismo orden), la pasa por OrbyGUI/src/hash.js y comprueba que
// salen los mismos números que el programa en C++.
//
// Uso:  node tools/test/test_config_hash.mjs [ruta al .exe]

import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import fs from 'node:fs';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const exe = process.argv[2] || path.join(here, 'test_config_hash.exe');

// OrbyGUI no declara "type": "module" (el proceso principal de Electron es
// CommonJS), así que Node no dejaría importar hash.js por su ruta. Como el
// módulo no importa nada, basta con cargar su texto: es el mismo archivo que
// usa la app, no una copia.
const hashSrc = fs.readFileSync(path.join(here, '..', '..', 'OrbyGUI', 'src', 'hash.js'), 'utf8');
const { profileHash, snapshotHash, toHex } =
  await import(`data:text/javascript,${encodeURIComponent(hashSrc)}`);

const PROFILES = 3;
const SLOTS = 20;
const FB_SIZE = 360;

// El mismo congruencial lineal que el .cpp. Math.imul multiplica en 32 bits con
// desbordamiento, que es lo que hace el uint32_t de C.
let rngState = 12345;
function rnd() {
  rngState = (Math.imul(rngState, 1103515245) + 12345) >>> 0;
  return (rngState >>> 24) & 0xff;
}

const LABELS = ['', 'A', 'COPY', 'abcdefgh', 'z', '~!@#', '0', '        '];
const NAMES = ['', 'OFIM', '12345678'];

// icons[perfil][pagina][hueco] -> Uint8Array(360)
const icons = [];

// Determinista a propósito: llamarla dos veces da exactamente lo mismo, que es
// lo que necesita la comprobación de "cambiar un bit mueve la huella".
function build() {
  rngState = 12345;
  icons.length = 0;
  const profiles = [];

  for (let p = 0; p < PROFILES; p++) {
    const pageCount = p + 1;
    const pages = [];
    icons.push([]);

    for (let pg = 0; pg < pageCount; pg++) {
      icons[p].push({});
      const page = {
        labels: [], keys: [], rotary: [], scroll: [], oledMask: 0,
      };

      for (let s = 0; s < 20; s++) page.labels.push(LABELS[(s + pg + p) % LABELS.length]);
      for (let s = 0; s < 24; s++) page.keys.push({ modifier: rnd(), keycode: rnd() });
      for (let s = 0; s < 16; s++) {
        page.rotary.push({ type: rnd() % 6, modifier: rnd(), keycode: rnd() });
      }
      for (let l = 0; l < 2; l++) {
        page.scroll.push({ detentsPerRev: 3 + (rnd() % 238), invert: (rnd() & 1) !== 0 });
      }

      const mask = (rnd() | (rnd() << 8) | (rnd() << 16)) & 0xfffff;
      page.oledMask = mask;

      for (let s = 0; s < SLOTS; s++) {
        if (!(mask & (1 << s))) continue;
        const bytes = new Uint8Array(FB_SIZE);
        for (let i = 0; i < FB_SIZE; i++) bytes[i] = rnd();
        icons[p][pg][s] = bytes;
      }

      pages.push(page);
    }

    profiles.push({ idx: p, name: NAMES[p], pageCount, pages });
  }

  return profiles;
}

const iconOf = (profileIdx, slot, page) => icons[profileIdx]?.[page]?.[slot] ?? null;

const profiles = build();

// El 5 es el tiempo de reposo por defecto del teclado, igual que en el .cpp.
const mine = snapshotHash(profiles, 5, iconOf);

// --- Lo que dice el firmware ------------------------------------------------
let theirs;
try {
  theirs = execFileSync(exe, { encoding: 'utf8' });
} catch (err) {
  console.error(`No se pudo ejecutar ${exe}: ${err.message}`);
  console.error('Compílalo antes:  g++ -I include -o tools/test/test_config_hash.exe tools/test/test_config_hash.cpp');
  process.exit(1);
}

const expected = new Map();
for (const line of theirs.trim().split(/\r?\n/)) {
  const [key, value] = line.trim().split(/\s+/);
  if (key && value) expected.set(key, value);
}

let failed = false;
function compare(key, got) {
  const want = expected.get(key);
  if (got === null) {
    console.error(`  ${key}: la app no pudo calcularlo (le faltan iconos)`);
    failed = true;
    return;
  }
  const hex = toHex(got);
  if (hex !== want) {
    console.error(`  ${key}: la app dice ${hex} y el firmware ${want}`);
    failed = true;
  } else {
    console.log(`  ${key}: ${hex}`);
  }
}

console.log('Huellas de la configuracion (app frente a firmware):');
for (let p = 0; p < PROFILES; p++) compare(`P${p}`, mine.per[p]);
compare('ALL', mine.all);

// Un cambio de un solo bit tiene que mover la huella; si no, la comparación de
// arriba podría estar cuadrando por casualidad (p.ej. dos ceros).
const tocado = build();
tocado[0].pages[0].keys[0].keycode ^= 1;
if (profileHash(tocado[0], (slot, page) => iconOf(0, slot, page)) === mine.per[0]) {
  console.error('  Cambiar un keycode no cambia la huella: el campo no entra en el CRC');
  failed = true;
}

if (failed) {
  console.error('\n[ FALLO ] Las dos implementaciones no calculan lo mismo.');
  process.exit(1);
}
console.log('[ OK ] app y firmware calculan la misma huella.');
