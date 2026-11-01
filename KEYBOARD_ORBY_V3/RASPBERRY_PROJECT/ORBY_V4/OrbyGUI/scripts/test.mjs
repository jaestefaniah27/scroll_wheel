// Lanza los tests de test/ con la lista de ficheros explícita: en Windows npm no expande
// comodines, y `node --test test/` ya no acepta carpeta desde Node 22. Tampoco vale
// `node --test` a secas: recoge test-serial.js, que es un script manual con teclado.
// --experimental-default-type=module hace que los src/*.js (ESM para Vite) se
// carguen como módulos en Node 20, que no detecta la sintaxis por sí solo.
import { readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const dir = fileURLToPath(new URL('../test/', import.meta.url));
const ficheros = readdirSync(dir).filter((f) => f.endsWith('.mjs')).map((f) => dir + f);
const r = spawnSync(process.execPath, ['--experimental-default-type=module', '--test', ...ficheros], { stdio: 'inherit' });
process.exit(r.status ?? 1);
