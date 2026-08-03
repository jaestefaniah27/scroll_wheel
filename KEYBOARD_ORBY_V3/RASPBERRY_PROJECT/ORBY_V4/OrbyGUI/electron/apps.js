const fs = require('fs');
const path = require('path');
const os = require('os');
const { shell } = require('electron');

// Lista de apps instaladas para la pestaña "App" del inspector de tecla: en
// vez de leer el registro (frágil, y muchas apps modernas -Store, portables-
// ni siquiera se anuncian ahí), se recorren los accesos directos del menú
// Inicio, que es justo la lista que ya conoce y mantiene Windows por su
// cuenta (nombre bonito, y cada acceso apunta ya al ejecutable real).
const START_MENU_DIRS = [
  path.join(process.env.ProgramData || 'C:\\ProgramData', 'Microsoft\\Windows\\Start Menu\\Programs'),
  path.join(process.env.APPDATA || os.homedir(), 'Microsoft\\Windows\\Start Menu\\Programs'),
];

// Accesos que no llevan a una app de verdad (desinstalador, léeme, licencia,
// enlaces a una web...). Solo se filtra por nombre, no hay forma fiable de
// saberlo del todo, pero cubre el ruido más habitual.
const NOISE_RE = /uninstall|desinstal|read ?me|léeme|help|ayuda|website|sitio web|support|licen[cs]/i;

async function walk(dir, out) {
  let entries;
  try {
    entries = await fs.promises.readdir(dir, { withFileTypes: true });
  } catch {
    return; // la carpeta no existe (usuario sin accesos propios, etc.)
  }

  for (const entry of entries) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) await walk(full, out);
    else if (entry.name.toLowerCase().endsWith('.lnk')) out.push(full);
  }
}

// Se cachea en memoria: recorrer el menú Inicio entero (cientos de accesos en
// un PC cargado) no es instantáneo, y la lista de apps instaladas no cambia
// mientras la app está abierta.
let cache = null;

async function listInstalledApps(forceRefresh = false) {
  if (cache && !forceRefresh) return cache;

  const lnkFiles = [];
  for (const dir of START_MENU_DIRS) await walk(dir, lnkFiles);

  const seen = new Set();
  const apps = [];
  for (const lnkPath of lnkFiles) {
    const name = path.basename(lnkPath, '.lnk');
    if (NOISE_RE.test(name)) continue;

    let target = '';
    try {
      target = shell.readShortcutLink(lnkPath).target;
    } catch {
      continue; // acceso roto, o no resoluble (algunos apuntan a una URL)
    }
    if (!target || !target.toLowerCase().endsWith('.exe')) continue;

    const key = target.toLowerCase();
    if (seen.has(key)) continue; // el mismo exe puede tener varios accesos
    seen.add(key);
    apps.push({ name, target });
  }

  apps.sort((a, b) => a.name.localeCompare(b.name, 'es'));
  cache = apps;
  return apps;
}

module.exports = { listInstalledApps };
