// Lanza los tests de test/ con la lista de ficheros explícita: en Windows npm no expande
// comodines, y `node --test test/` ya no acepta carpeta desde Node 22. Tampoco vale
// `node --test` a secas: recoge test-serial.js, que es un script manual con teclado.
// --experimental-default-type=module hace que los src/*.js (ESM para Vite) se
// carguen como módulos en Node 20/21, que no detecta la sintaxis por sí solo.
// Desde Node 22.7 detecta ESM solo y Node 24 eliminó el flag (daría "bad option"),
// así que solo se pasa si esta versión de Node lo admite.
import { readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const dir = fileURLToPath(new URL('../test/', import.meta.url));
const ficheros = readdirSync(dir).filter((f) => f.endsWith('.mjs')).map((f) => dir + f);
const flagTipo = process.allowedNodeEnvironmentFlags.has('--experimental-default-type')
  ? ['--experimental-default-type=module']
  : [];
const r = spawnSync(process.execPath, [...flagTipo, '--test', ...ficheros], { stdio: 'inherit' });
process.exit(r.status ?? 1);
