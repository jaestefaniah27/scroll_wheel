// Empaqueta cada complemento de plugins/ en un .zip listo para repartir.
//
// Los complementos NO van dentro del instalador de OrbyGUI (mira "files" en
// package.json: solo entran dist, electron, assets y el propio package.json).
// Se reparten aparte y se instalan desde Ajustes → Complementos, que es justo
// lo que los hace opcionales.
//
// Comprime con Compress-Archive en vez de con una dependencia de npm: es lo
// mismo que ya usa el anfitrión para descomprimir (ver electron/plugins.js) y
// no añade nada al árbol de dependencias por un script que se ejecuta a mano.

const fs = require('fs');
const path = require('path');
const { execFileSync } = require('child_process');

const raiz = path.join(__dirname, '..');
const origen = path.join(raiz, 'plugins');
const destino = path.join(raiz, 'release', 'plugins');

if (!fs.existsSync(origen)) {
  console.error('No hay carpeta plugins/ que empaquetar.');
  process.exit(1);
}

fs.mkdirSync(destino, { recursive: true });
const q = (s) => `'${String(s).replace(/'/g, "''")}'`;

const carpetas = fs.readdirSync(origen, { withFileTypes: true })
  .filter((d) => d.isDirectory() && fs.existsSync(path.join(origen, d.name, 'plugin.json')));

if (!carpetas.length) {
  console.error('No hay ningún complemento con plugin.json en plugins/.');
  process.exit(1);
}

for (const d of carpetas) {
  const dir = path.join(origen, d.name);
  const manifest = JSON.parse(fs.readFileSync(path.join(dir, 'plugin.json'), 'utf8'));
  const zip = path.join(destino, `${manifest.id}-${manifest.version || '0.0.0'}.zip`);

  // El comodín mete el CONTENIDO de la carpeta en la raíz del zip; el
  // instalador admite las dos formas, pero así el zip se puede descomprimir
  // encima de la carpeta del complemento sin anidarla.
  execFileSync('powershell.exe', [
    '-NoProfile', '-NonInteractive', '-Command',
    `Compress-Archive -Path ${q(path.join(dir, '*'))} -DestinationPath ${q(zip)} -Force`,
  ], { stdio: 'inherit' });

  console.log(`${manifest.name} ${manifest.version} → ${path.relative(raiz, zip)}`);
}
