# Orby WebGUI — Plan de implementación

> **Para quien lo ejecute (agente o persona):** usa `superpowers:executing-plans` o
> `superpowers:subagent-driven-development` para ir tarea a tarea. Los pasos llevan
> casilla (`- [ ]`) para ir marcándolos.

**Objetivo:** que OrbyGUI se pueda usar desde el navegador, sin instalar nada en el PC,
para configurar el teclado en un equipo corporativo donde no se puede instalar software.

**Arquitectura:** el teclado ya expone un CDC ACM y habla por líneas de texto. Chrome y
Edge saben abrir ese CDC con **Web Serial**, así que el firmware **no se toca**: el
protocolo, `device.js`, `store.js` y las vistas siguen siendo exactamente los mismos.
Lo único que falta en el navegador es `window.orby`, que en la app de escritorio monta
`electron/preload.js`. Se escribe un equivalente para navegador (Web Serial +
IndexedDB) y el mismo `src/` arranca en los dos sitios. Un único `index.html` y un
único build de vite sirven para Electron y para GitHub Pages.

**Stack:** JS vanilla con módulos ES (sin framework, sin TypeScript), vite 7, Web Serial
API, IndexedDB, GitHub Pages. Tests con `node --test` (viene en Node 20, sin dependencias
nuevas).

---

## Constantes globales

Todas las tareas heredan esto:

- **Todo en castellano de España**: comentarios, textos de interfaz, mensajes de commit,
  documentación. Nada de castellano latinoamericano ni de "pa'".
- Los comentarios explican **por qué**, no qué. Documentan la decisión y el fallo que
  evitan. Un comentario que repite el código sobra.
- **Sin framework en el renderer, sin TypeScript.** No introducir ninguno.
- **Nada de `src/` puede importar de `electron/` ni de Node.** `src/` es código de
  navegador y ahora se ejecuta literalmente en uno.
- **El firmware no se toca en todo este plan.** Ni `main.cpp`, ni `src/usb_descriptors.c`,
  ni `include/orby_version.h`.
- **No se bifurcan las vistas.** Si un módulo de `src/` necesita saber dónde corre,
  pregunta a `src/platform.js`. Nunca a `navigator.userAgent` y nunca comprobando si
  existe tal o cual método de `window.orby`.
- VID del teclado: `0xCafe` (51966 decimal). Se filtra **solo por VID, cualquier PID**,
  igual que `KNOWN_IDS` en `electron/serial.js:444`: el firmware sube el PID cada vez que
  cambia la estructura de un informe HID.
- Puerto serie: `baudRate: 115200`, DTR y RTS a `true` (TinyUSB no acepta RX sin ellos).
- Web Serial exige **contexto seguro** (HTTPS o `localhost`) y un **gesto del usuario**
  para `navigator.serial.requestPort()`. `navigator.serial.getPorts()` no lo exige.
- Firmware mínimo real: el mismo que la app de escritorio (`FW_MIN = '2.0'` en
  `src/compat.js`). La WebGUI no baja ni sube ese listón.
- **Directorio de trabajo de los comandos:** todos los `git` y `npm` se ejecutan desde
  `ORBY_V4/` salvo los de la Tarea 10, que toca `.github/workflows/` y va desde la raíz
  del repositorio (`git rev-parse --show-toplevel`, dos niveles por encima de `ORBY_V4/`).

---

## Lo que el TODO daba por pendiente y ya está hecho

Comprobado en el código el 2026-08-10. **No hay que implementar nada de esto**; el
ejecutor solo tiene que actualizar el TODO (Tarea 11).

| Punto del TODO | Estado real |
|---|---|
| «Habilitar un endpoint USB Raw HID para configuración» | **No hace falta.** Con Web Serial se usa el CDC que ya existe. Además Chrome bloquea WebHID sobre colecciones de teclado/ratón/consumer, así que haría falta una colección *vendor* nueva, subir el PID y un protocolo binario nuevo: semanas de trabajo para la misma cobertura de navegadores. |
| «Diseñar protocolo ligero de lectura/escritura de configuración» | **Ya existe.** El protocolo de texto `COMANDO:arg:arg` sobre CDC, con la capa de peticiones con promesa de `src/device.js`. |
| «Implementar guardado persistente (Flash) de la configuración completa» | **Ya está.** `main.cpp:914` (`flash_range_erase`/`flash_range_program` sobre `FLASH_TARGET_OFFSET`), comando `SAVE_STATE`, con migración de formatos V1→V7. |
| «Hacer que el firmware parsee y ejecute macros (device-eligible) sin el PC» | **Ya está** desde el firmware 4.0. `MACRO_MAX_COUNT 64` (`main.cpp:622`), `MACRO_MAX_STEPS 48` (`main.cpp:626`), bandera `MACROS=1` y `MAXMACROS` en el handshake (`main.cpp:2210`). |
| «Implementar método de cambio manual de perfil en el teclado» | **Ya está.** La tecla 12 abre el menú del teclado al mantenerla; el teclado avisa del cambio con la telemetría `EV:CTX:<perfil>:<pagina>:<total>`. |
| «Configurar repositorio y hosting estático» | El repositorio y `.github/workflows/pages.yml` ya existen; falta añadir la app al despliegue (Tarea 10). |

Fuera de alcance de este plan, aunque el TODO los tenga cerca:

- Marcar en las **pantallas OLED del teclado** las teclas que no funcionan sin la app
  abierta. Es firmware y pertenece al punto suelto del TODO sobre device-eligible, no a
  la WebGUI. La parte de interfaz de ese mismo punto sí entra aquí (Tarea 8), y como
  `src/` es compartido sale también en la app de escritorio.
- Complementos, biblioteca de perfiles y tienda de plugins.

---

## Estructura de ficheros

**Se crean:**

| Fichero | Responsabilidad |
|---|---|
| `OrbyGUI/src/entry.js` | Único punto de entrada del `index.html`. Decide si arranca la vía Electron o la vía navegador. |
| `OrbyGUI/src/platform.js` | Dónde corre la app y qué funciones tiene disponibles ahí. |
| `OrbyGUI/src/web-main.js` | Arranque de la vía navegador: monta `window.orby` y luego carga `main.js`. |
| `OrbyGUI/src/web/split-lines.mjs` | Trocear el flujo del puerto en líneas. Puro, con tests. |
| `OrbyGUI/src/web/config-merge.mjs` | Fusión de la configuración local y sus valores por defecto. Puro, con tests. |
| `OrbyGUI/src/web/transport-serial.js` | Web Serial: abrir, leer, escribir, reconectar, handshake. |
| `OrbyGUI/src/web/config-store.js` | `getConfig`/`setConfig` sobre IndexedDB. |
| `OrbyGUI/src/web/backup-file.js` | `saveBackup`/`loadBackup` con descarga y `<input type="file">`. |
| `OrbyGUI/src/web/orby-web.js` | Monta el objeto `window.orby` completo para el navegador. |
| `OrbyGUI/src/web/connect-gate.js` | Pantalla de "Conectar teclado" y avisos de navegador no compatible. |
| `OrbyGUI/test/split-lines.test.mjs` | Tests de `split-lines.mjs`. |
| `OrbyGUI/test/config-merge.test.mjs` | Tests de `config-merge.mjs`. |
| `OrbyGUI/docs/WEBGUI.md` | Qué es la WebGUI, qué no puede hacer y cómo se publica. |

**Se modifican:**

| Fichero | Cambio |
|---|---|
| `OrbyGUI/index.html` | El `<script>` apunta a `src/entry.js`; se añade el hueco de la pantalla de conexión. |
| `OrbyGUI/electron/preload.js` | Declara `platform: 'electron'`. |
| `OrbyGUI/src/main.js` | Oculta los controles de ventana y la vista "Cambio automático" cuando no hay PC detrás. |
| `OrbyGUI/src/views/settings.js` | Oculta las tarjetas que no aplican en navegador. |
| `OrbyGUI/src/views/profiles/macros-store.js` | Añade `keyNeedsApp` / `rotaryNeedsApp`. |
| `OrbyGUI/src/views/profiles.js` | Insignia en las teclas y mandos que necesitan la app; pestañas del inspector deshabilitadas en navegador. |
| `OrbyGUI/src/views/profiles/sequence-editor.js` | Apaga los pasos de secuencia que solo sabe tocar el PC. |
| `OrbyGUI/src/styles/index.css` | Estilos de la pantalla de conexión, la insignia y las pestañas deshabilitadas. |
| `OrbyGUI/package.json` | Scripts `test` y `preview:web`. |
| `.github/workflows/pages.yml` | Publica también la app en `/app/`. |
| `OrbyGUI/docs/TODO.md` | Marca lo hecho y lo que resultó no hacer falta. |
| `ORBY_V4/CLAUDE.md` | Documenta la vía navegador. |

---

## Tarea 1: Punto de entrada único y detección de plataforma

**Ficheros:**
- Crear: `OrbyGUI/src/platform.js`
- Crear: `OrbyGUI/src/entry.js`
- Modificar: `OrbyGUI/index.html` (la etiqueta `<script>` del final)
- Modificar: `OrbyGUI/electron/preload.js`

**Interfaces:**
- Produce: `platform.setPlatform(name)`, `platform.isWeb(): boolean`,
  `platform.can(feature: string): boolean`. Los nombres de función válidos son las
  cadenas de `PC_ONLY` más cualquier otra: `can` devuelve `true` para todo lo que no
  esté en esa lista.

- [ ] **Paso 1: crear `src/platform.js`**

```js
// Dónde corre esta copia de la app y, por tanto, qué puede hacer.
//
// La app de escritorio tiene un proceso principal detrás: puede abrir programas,
// mover el ratón, cargar complementos y vigilar qué ventana está en primer plano.
// La versión de navegador no tiene nada de eso —solo el puerto serie del teclado—,
// así que hay funciones del editor que ahí no van a funcionar.
//
// La diferencia se pregunta SIEMPRE aquí. Repartir comprobaciones del tipo
// "¿existe window.orby.recorder?" por las vistas es cómo se acaba con una función
// medio deshabilitada en cinco sitios y viva en el sexto.

let current = 'electron';

export function setPlatform(name) {
  current = name;
}

export function isWeb() {
  return current === 'web';
}

// Lo que necesita un proceso con permisos de escritorio. Cada nombre es lo que
// preguntan las vistas, no el módulo que lo implementa: si mañana las secuencias
// las tocara el navegador, se quita de aquí y ya está.
const PC_ONLY = new Set([
  'openApp',      // abrir una aplicación o un archivo
  'text',         // escribir un texto por el teclado del sistema
  'power',        // suspender, apagar, bloquear
  'recorder',     // grabar y reproducir ratón y teclado
  'plugins',      // complementos
  'autoProfile',  // cambio de perfil según la ventana en primer plano
  'autostart',    // arrancar con la sesión de Windows
  'appUpdate',    // actualizar la propia app
  'firmwareUpdate', // actualizar el firmware del teclado
  'windowChrome', // minimizar / maximizar / cerrar
  'pcSequences',  // secuencias con pasos que solo sabe tocar el PC
]);

export function can(feature) {
  return !isWeb() || !PC_ONLY.has(feature);
}
```

- [ ] **Paso 2: crear `src/entry.js`**

```js
// La misma app arranca en dos sitios: dentro de Electron y dentro de un navegador.
//
// En Electron, `electron/preload.js` ya ha montado `window.orby` antes de que este
// script corra, así que el renderer puede empezar directamente. En un navegador no
// hay preload ni proceso principal: hay que montar el equivalente (Web Serial +
// IndexedDB) antes de que ningún módulo de src/ toque `window.orby`, y por eso
// main.js entra por importación dinámica y no por `import` normal.

if (window.orby) {
  await import('./main.js');
} else {
  await import('./web-main.js');
}
```

- [ ] **Paso 3: apuntar `index.html` a la entrada nueva**

Buscar la etiqueta de script del final de `OrbyGUI/index.html` (`<script type="module"
src="/src/main.js"></script>`) y dejarla en:

```html
    <script type="module" src="/src/entry.js"></script>
```

- [ ] **Paso 4: que el preload declare la plataforma**

En `OrbyGUI/electron/preload.js`, dentro del objeto que se pasa a
`contextBridge.exposeInMainWorld('orby', { ... })`, añadir como primera clave:

```js
  // Con qué está hablando el renderer. La versión de navegador monta este mismo
  // objeto con 'web' y muchas de las claves de abajo en blanco (ver src/web/orby-web.js).
  platform: 'electron',
```

- [ ] **Paso 5: comprobar que la app de escritorio sigue igual**

```bash
cd OrbyGUI && npm run dev
```

Esperado: la ventana abre como siempre, la barra dice "Orby V4 conectado · fw 4.3" con
el teclado enchufado, y en la consola del renderer **no** aparece
`Failed to resolve module`.

> **Trampa conocida:** si Electron arranca como Node y no abre ventana, es
> `ELECTRON_RUN_AS_NODE`, que filtra VS Code. Limpia esa variable. Si `npm install`
> falla con ficheros en uso, es OneDrive sincronizando `node_modules/electron`.

- [ ] **Paso 6: commit**

```bash
git add OrbyGUI/src/platform.js OrbyGUI/src/entry.js OrbyGUI/index.html OrbyGUI/electron/preload.js
git commit -m "feat(web): punto de entrada único para escritorio y navegador"
```

---

## Tarea 2: Trocear el flujo del puerto en líneas (con tests)

**Ficheros:**
- Crear: `OrbyGUI/src/web/split-lines.mjs`
- Crear: `OrbyGUI/test/split-lines.test.mjs`
- Modificar: `OrbyGUI/package.json` (script `test`)

**Interfaces:**
- Produce: `splitLines(resto: string, trozo: string) => { lineas: string[], resto: string }`

> **Por qué `.mjs`:** `package.json` no lleva `"type": "module"` (lo necesitan los CJS de
> `electron/`), así que Node trataría un `.js` de `src/` como CommonJS y se atragantaría
> con `export`. La extensión `.mjs` lo fuerza a ESM sin tocar nada más, y vite la importa
> igual. Solo se usa en los dos módulos puros que tienen tests.

- [ ] **Paso 1: escribir el test que falla**

Crear `OrbyGUI/test/split-lines.test.mjs`:

```js
import test from 'node:test';
import assert from 'node:assert/strict';
import { splitLines } from '../src/web/split-lines.mjs';

test('separa las líneas completas y guarda el resto', () => {
  const r = splitLines('', 'ORBY_V4:FW=4.3\nSTATE:PROFILE:0\nSTATE:PRO');
  assert.deepEqual(r.lineas, ['ORBY_V4:FW=4.3', 'STATE:PROFILE:0']);
  assert.equal(r.resto, 'STATE:PRO');
});

test('une el resto anterior con el trozo nuevo', () => {
  const r = splitLines('STATE:PRO', 'FILES:3:8\nSTATE:END\n');
  assert.deepEqual(r.lineas, ['STATE:PROFILES:3:8', 'STATE:END']);
  assert.equal(r.resto, '');
});

test('descarta el retorno de carro y las líneas en blanco', () => {
  // El firmware manda \n, pero un CDC puede entregar \r\n según cómo lo empaquete
  // el host. Una línea vacía colada en la cola de peticiones de device.js no
  // encaja con ningún match y se queda ahí.
  const r = splitLines('', 'PROFILE:OK:1\r\n\r\nSAVE:OK\r\n');
  assert.deepEqual(r.lineas, ['PROFILE:OK:1', 'SAVE:OK']);
  assert.equal(r.resto, '');
});

test('un trozo sin salto de línea no produce ninguna', () => {
  const r = splitLines('', 'MACRO:');
  assert.deepEqual(r.lineas, []);
  assert.equal(r.resto, 'MACRO:');
});
```

- [ ] **Paso 2: añadir el script de tests a `package.json`**

En `"scripts"`, junto a los demás:

```json
    "test": "node --test test/",
```

- [ ] **Paso 3: ejecutarlo y ver que falla**

```bash
cd OrbyGUI && npm test
```

Esperado: FALLA con `Cannot find module ... src/web/split-lines.mjs`.

- [ ] **Paso 4: escribir `src/web/split-lines.mjs`**

```js
// El puerto entrega trozos de bytes, no líneas: un mensaje del teclado puede
// llegar partido en dos lecturas y dos mensajes pueden llegar en la misma.
//
// Es lo mismo que hace el ReadlineParser de electron/serial.js, pero aquí no hay
// ningún parser de serialport que lo haga por nosotros. Se saca a su propio fichero
// porque es la única parte de todo el transporte que se puede probar sin un teclado
// enchufado, y porque equivocarse aquí se manifiesta como peticiones que caducan a
// los 4 s sin ninguna pista de por qué.

export function splitLines(resto, trozo) {
  const partes = (resto + trozo).split('\n');
  // El último elemento no termina en \n: o es una línea a medias o es cadena vacía.
  const cola = partes.pop() ?? '';
  const lineas = [];
  for (const parte of partes) {
    const limpia = parte.trim();
    if (limpia) lineas.push(limpia);
  }
  return { lineas, resto: cola };
}
```

- [ ] **Paso 5: ejecutar los tests y ver que pasan**

```bash
cd OrbyGUI && npm test
```

Esperado: `# pass 4` y `# fail 0`.

- [ ] **Paso 6: commit**

```bash
git add OrbyGUI/src/web/split-lines.mjs OrbyGUI/test/split-lines.test.mjs OrbyGUI/package.json
git commit -m "feat(web): trocear el flujo del puerto en líneas"
```

---

## Tarea 3: Fusión de la configuración local (con tests)

**Ficheros:**
- Crear: `OrbyGUI/src/web/config-merge.mjs`
- Crear: `OrbyGUI/test/config-merge.test.mjs`

**Interfaces:**
- Consume: nada.
- Produce: `DEFAULTS` (objeto), `merge(base, patch) => objeto nuevo`.

> Es un duplicado a propósito de `electron/config.js`: aquel es CommonJS y corre en el
> proceso principal, este es ESM y corre en el navegador. Son doce líneas; compartirlas
> obligaría a meter un módulo neutro que las dos plataformas puedan cargar, que es más
> maquinaria de la que ahorra. **Si cambias `REPLACE_WHOLE` o `DEFAULTS` en uno, cámbialo
> en el otro.**

- [ ] **Paso 1: escribir el test que falla**

Crear `OrbyGUI/test/config-merge.test.mjs`:

```js
import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULTS, merge } from '../src/web/config-merge.mjs';

test('los valores por defecto traen las claves que leen las vistas', () => {
  assert.deepEqual(DEFAULTS.macros, []);
  assert.equal(DEFAULTS.macrosOnDevice, null);
  assert.equal(DEFAULTS.deviceMirror, null);
  assert.equal(DEFAULTS.wheelDial.offsetDeg, 62);
});

test('fusiona en profundidad sin tocar la base', () => {
  const base = { wheelDial: { invert: true, offsetDeg: 62, marker: 'dot' } };
  const out = merge(base, { wheelDial: { marker: 'line' } });
  assert.deepEqual(out.wheelDial, { invert: true, offsetDeg: 62, marker: 'line' });
  assert.equal(base.wheelDial.marker, 'dot');
});

test('el espejo del teclado se sustituye entero, no se fusiona', () => {
  // Fusionándolo, un icono borrado o un perfil que ya no existe seguirían en el
  // archivo para siempre: el espejo describe un estado completo, no un parche.
  const base = { deviceMirror: { savedAt: 'antes', icons: { '0:0:1': 'aa' } } };
  const out = merge(base, { deviceMirror: { savedAt: 'ahora', icons: {} } });
  assert.deepEqual(out.deviceMirror, { savedAt: 'ahora', icons: {} });
});

test('los arrays se sustituyen enteros', () => {
  const out = merge({ macros: [{ id: 1 }, { id: 2 }] }, { macros: [{ id: 3 }] });
  assert.deepEqual(out.macros, [{ id: 3 }]);
});
```

- [ ] **Paso 2: ejecutarlo y ver que falla**

```bash
cd OrbyGUI && npm test
```

Esperado: FALLA con `Cannot find module ... src/web/config-merge.mjs`.

- [ ] **Paso 3: escribir `src/web/config-merge.mjs`**

```js
// Los valores por defecto de la configuración local y cómo se fusiona un parche.
//
// Es la misma lógica que electron/config.js, repetida porque aquel es CommonJS del
// proceso principal y este es ESM que corre en el navegador. Doce líneas duplicadas
// salen más baratas que un módulo neutro que carguen los dos mundos. Al tocar
// REPLACE_WHOLE o DEFAULTS aquí, hay que tocarlo también allí.
//
// Se han quitado las claves que en navegador no puede rellenar nadie (complementos,
// cambio automático por ventana): dejarlas sería prometer una función que no existe.

export const DEFAULTS = {
  // Variaciones de perfil por aplicación. En navegador nadie las escribe —no hay
  // detector de ventanas—, pero variants.js las lee al arrancar y sin la clave
  // tendría que aprender a vivir sin ella.
  profileVariants: [],

  // Secuencias. En navegador solo se editan las que el teclado sabe tocar solo
  // (ver macroDeviceEligible en views/profiles/macros-store.js); el resto se ven
  // pero no se tocan.
  macros: [],
  macrosOnDevice: null,

  // Calibración del dibujo de la rueda. Es del PC, no del firmware: depende de cómo
  // estén montados el imán y el marcador de la tapa.
  wheelDial: {
    invert: true,
    offsetDeg: 62,
    marker: 'dot',
  },

  // Espejo de lo que hay en el teclado, en el mismo formato que una copia de
  // seguridad. Permite abrir la app sin el Orby (en modo lectura) y, con él
  // conectado, no descargar los perfiles cuya huella no ha cambiado.
  deviceMirror: null,
};

// Claves que se sustituyen enteras en vez de fusionarse.
const REPLACE_WHOLE = new Set(['deviceMirror']);

export function merge(base, patch) {
  const out = { ...base };
  for (const [key, value] of Object.entries(patch || {})) {
    out[key] = (value && typeof value === 'object' && !Array.isArray(value) && !REPLACE_WHOLE.has(key))
      ? merge(base[key] || {}, value)
      : value;
  }
  return out;
}
```

- [ ] **Paso 4: ejecutar los tests y ver que pasan**

```bash
cd OrbyGUI && npm test
```

Esperado: `# pass 8` y `# fail 0` (los 4 de la tarea anterior más estos 4).

- [ ] **Paso 5: commit**

```bash
git add OrbyGUI/src/web/config-merge.mjs OrbyGUI/test/config-merge.test.mjs
git commit -m "feat(web): valores por defecto y fusión de la configuración local"
```

---

## Tarea 4: Configuración local sobre IndexedDB

**Ficheros:**
- Crear: `OrbyGUI/src/web/config-store.js`

**Interfaces:**
- Consume: `DEFAULTS`, `merge` de `./config-merge.mjs`.
- Produce: `initConfig(): Promise<void>`, `getConfig(): Promise<object>`,
  `setConfig(patch: object): Promise<object>`. Las dos últimas son exactamente la firma
  que `electron/preload.js` expone como `window.orby.getConfig`/`setConfig`.

- [ ] **Paso 1: escribir `src/web/config-store.js`**

```js
// La configuración local, que en la app de escritorio vive en un JSON de
// %APPDATA%, aquí vive en IndexedDB del navegador.
//
// No vale localStorage: el espejo del teclado lleva los iconos OLED en hexadecimal
// (360 bytes por pantalla, veinte por página) y con unos cuantos perfiles se pasa
// del límite de 5 MB. IndexedDB no tiene ese techo.
//
// Se guarda un único registro con todo el objeto, no una clave por ajuste: quien
// llama pide y escribe la configuración entera (mirror.js vuelca el espejo completo
// en cada guardado), así que partirlo solo añadiría transacciones.

import { DEFAULTS, merge } from './config-merge.mjs';

const DB_NAME = 'orby-webgui';
const DB_VERSION = 1;
const STORE = 'config';
const KEY = 'local';

let db = null;
let cache = null;
let saveTimer = null;

function openDb() {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION);
    req.onupgradeneeded = () => req.result.createObjectStore(STORE);
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
}

function read() {
  return new Promise((resolve) => {
    const req = db.transaction(STORE, 'readonly').objectStore(STORE).get(KEY);
    req.onsuccess = () => resolve(req.result ?? null);
    req.onerror = () => resolve(null);
  });
}

function write(value) {
  return new Promise((resolve) => {
    const tx = db.transaction(STORE, 'readwrite');
    tx.objectStore(STORE).put(value, KEY);
    tx.oncomplete = () => resolve(true);
    tx.onerror = () => resolve(false);
  });
}

// Hay que llamarlo antes de montar window.orby: getConfig no puede devolver una
// promesa que espere a la base sin más, porque mirror.js la llama en el arranque y
// una configuración vacía haría creer a la app que no hay copia local.
export async function initConfig() {
  try {
    db = await openDb();
    cache = merge(DEFAULTS, (await read()) || {});
  } catch {
    // Navegación privada, IndexedDB deshabilitada por política del navegador… La
    // app funciona igual, pero cada recarga empieza de cero: mejor eso que no
    // arrancar.
    console.warn('[web] sin IndexedDB: la configuración no sobrevivirá a la recarga');
    db = null;
    cache = structuredClone(DEFAULTS);
  }
}

export async function getConfig() {
  return cache;
}

// Escribe agrupando: mirror.js guarda el espejo entero después de cada cambio y
// durante la precarga de iconos eso son decenas de guardados seguidos con el mismo
// objeto grande dentro.
const SAVE_DELAY_MS = 300;

export async function setConfig(patch) {
  cache = merge(cache, patch);
  if (!db) return cache;

  if (saveTimer) clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    saveTimer = null;
    write(cache).catch(() => {});
  }, SAVE_DELAY_MS);

  return cache;
}
```

- [ ] **Paso 2: commit**

```bash
git add OrbyGUI/src/web/config-store.js
git commit -m "feat(web): configuración local en IndexedDB"
```

---

## Tarea 5: Transporte Web Serial

**Ficheros:**
- Crear: `OrbyGUI/src/web/transport-serial.js`

**Interfaces:**
- Consume: `splitLines` de `./split-lines.mjs`.
- Produce:
  - `disponible(): boolean`
  - `on(evento, cb): void` — eventos `'connected'` (con `info`), `'disconnected'`,
    `'data'` (con la línea), `'error'` (con el texto), `'searching'`.
  - `estado(): 'connected' | 'searching' | 'disconnected'`
  - `info(): object | null` — el objeto de identidad del teclado.
  - `enviar(comando: string): Promise<void>`
  - `pedirPuerto(): Promise<boolean>` — **exige gesto del usuario**.
  - `arrancar(): Promise<boolean>` — intenta abrir un puerto ya autorizado, sin gesto.
  - `reconectar(): Promise<void>`

El objeto de identidad tiene que tener **exactamente la misma forma** que el de
`electron/serial.js:331` (`_parseDeviceInfo`), porque `src/compat.js` y
`views/profiles/macros-store.js` leen sus claves: cada `CLAVE=valor` del handshake pasa
a `info[clave.toLowerCase()] = valor` (`fw`, `keys`, `oleds`, `encoders`, `profiles`,
`maxprofiles`, `maxpages`, `macros`, `maxmacros`, `hash`, `bootsel`, `mode`), más
`raw` con la línea entera y `port` con algo que identifique el puerto.

- [ ] **Paso 1: escribir `src/web/transport-serial.js`**

```js
// Web Serial: el mismo CDC que abre electron/serial.js, pero desde el navegador.
//
// El teclado no necesita nada nuevo. Expone un CDC ACM y habla por líneas de texto,
// así que aquí basta con abrir el puerto, ponerle las señales y trocear el flujo.
// El protocolo, la cola de peticiones de device.js y las vistas se quedan igual.

import { splitLines } from './split-lines.mjs';

// El VID de los ejemplos de TinyUSB, en decimal: Web Serial pide números, no las
// cadenas hexadecimales que devuelve serialport. Debe seguir a KNOWN_IDS de
// electron/serial.js. No se filtra por PID a propósito: el firmware lo sube cada vez
// que cambia la estructura de un informe HID, y filtrarlo dejaría fuera a los
// teclados con firmware anterior.
export const ORBY_USB_VID = 0xCafe;

const BAUD_RATE = 115200;

const oyentes = new Map();
let puerto = null;
let escritor = null;
let lector = null;
let identidad = null;
let conectado = false;
let buscando = false;
let cerrandoAdrede = false;
let reintento = null;

export function disponible() {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

export function on(evento, cb) {
  if (!oyentes.has(evento)) oyentes.set(evento, new Set());
  oyentes.get(evento).add(cb);
}

function emitir(evento, dato) {
  const set = oyentes.get(evento);
  if (set) for (const cb of set) cb(dato);
}

export function estado() {
  if (conectado) return 'connected';
  return buscando ? 'searching' : 'disconnected';
}

export function info() {
  return identidad;
}

// --- Identidad -------------------------------------------------------------

// Misma forma que _parseDeviceInfo en electron/serial.js: compat.js y macros-store.js
// leen estas claves en minúscula, y una identidad con otra forma haría que la app
// tratara a un teclado 4.3 como si fuera anterior a las páginas y las huellas.
function parsearIdentidad(linea) {
  const out = { raw: linea, port: 'Web Serial' };
  for (const parte of linea.split(':')) {
    if (!parte.includes('=')) continue;
    const [clave, valor] = parte.split('=');
    out[clave.toLowerCase()] = valor;
  }
  return out;
}

// --- Apertura --------------------------------------------------------------

async function abrir(p) {
  await p.open({ baudRate: BAUD_RATE });
  // TinyUSB no acepta lo que le mandamos si el host no ha levantado DTR: sin esto
  // el teclado nunca ve el ACK y la presentación no llega jamás.
  try { await p.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch { /* algún backend no las soporta */ }

  puerto = p;
  escritor = p.writable.getWriter();
  cerrandoAdrede = false;
  bucleLectura();

  // El primer ACK se pierde a menudo si el CDC acaba de enumerarse, igual que en
  // la app de escritorio: se insiste unas cuantas veces hasta que se presenta.
  for (let i = 0; i < 5 && !conectado; i++) {
    await enviar('ACK');
    await new Promise((r) => setTimeout(r, 400 + i * 200));
  }
  return conectado;
}

async function bucleLectura() {
  const decodificador = new TextDecoder();
  let resto = '';

  try {
    lector = puerto.readable.getReader();
    for (;;) {
      const { value, done } = await lector.read();
      if (done) break;
      const trozo = decodificador.decode(value, { stream: true });
      const r = splitLines(resto, trozo);
      resto = r.resto;
      for (const linea of r.lineas) manejarLinea(linea);
    }
  } catch (err) {
    // Desenchufar el teclado hace que la lectura reviente con NetworkError. No es
    // un fallo que contarle al usuario: es la vía normal por la que nos enteramos
    // de que se ha ido, porque el evento 'disconnect' de navigator.serial no
    // siempre llega antes.
    if (!cerrandoAdrede) console.warn('[web] la lectura del puerto se ha cortado:', err.message);
  } finally {
    try { lector?.releaseLock(); } catch { /* ya liberado */ }
    lector = null;
    if (!cerrandoAdrede) await caida();
  }
}

function manejarLinea(linea) {
  if (linea.startsWith('ORBY_V4:')) {
    // El teclado repite su presentación cada vez que se le manda un ACK, así que
    // esta línea llega también con la conexión ya hecha. Anunciarla otra vez haría
    // a la app resincronizarse entera.
    const yaConectado = conectado && identidad?.raw === linea;
    identidad = parsearIdentidad(linea);
    conectado = true;
    buscando = false;
    if (!yaConectado) emitir('connected', identidad);
    return;
  }
  emitir('data', linea);
}

async function caida() {
  if (!conectado && !puerto) return;
  conectado = false;
  identidad = null;
  try { escritor?.releaseLock(); } catch { /* ya liberado */ }
  escritor = null;
  try { await puerto?.close(); } catch { /* ya cerrado, o se lo llevó el desenchufe */ }
  puerto = null;
  emitir('disconnected');
  reintentar();
}

// --- Reconexión ------------------------------------------------------------
// El permiso lo concede el usuario una vez y se queda concedido para ese puerto,
// así que volver a abrirlo tras un desenchufe NO necesita otro gesto: se puede
// insistir solo, igual que hace el escaneo de la app de escritorio.
const REINTENTO_MS = 3000;

function reintentar() {
  if (reintento) return;
  buscando = true;
  emitir('searching');
  reintento = setInterval(async () => {
    if (conectado) { clearInterval(reintento); reintento = null; return; }
    const p = await puertoAutorizado();
    if (!p) return;
    try { await abrir(p); } catch { /* sigue sin estar listo */ }
  }, REINTENTO_MS);
}

async function puertoAutorizado() {
  const puertos = await navigator.serial.getPorts();
  return puertos.find((p) => p.getInfo?.().usbVendorId === ORBY_USB_VID) || puertos[0] || null;
}

// --- API pública -----------------------------------------------------------

export async function enviar(comando) {
  if (!escritor) return;
  try {
    await escritor.write(new TextEncoder().encode(`${comando}\n`));
  } catch (err) {
    emitir('error', `No se pudo escribir en el puerto: ${err.message}`);
  }
}

// Pide un puerto nuevo. Solo se puede llamar desde el manejador de un clic: Chrome
// rechaza requestPort() fuera de un gesto del usuario.
export async function pedirPuerto() {
  try {
    const p = await navigator.serial.requestPort({ filters: [{ usbVendorId: ORBY_USB_VID }] });
    return await abrir(p);
  } catch (err) {
    // NotFoundError = el usuario cerró el diálogo sin elegir nada. No es un error
    // que merezca un aviso rojo.
    if (err.name === 'NotFoundError') return false;
    // En Windows solo un proceso puede tener el COM abierto: con OrbyGUI de
    // escritorio corriendo, esto falla siempre y el motivo no se adivina.
    emitir('error', err.name === 'NetworkError'
      ? 'No se pudo abrir el puerto. Si tienes OrbyGUI de escritorio abierta, ciérrala: solo un programa puede usar el teclado a la vez.'
      : `No se pudo abrir el puerto: ${err.message}`);
    return false;
  }
}

// Intenta abrir un puerto ya autorizado en una visita anterior. No necesita gesto,
// así que es lo que se llama al cargar la página.
let oyenteDesconexion = false;

export async function arrancar() {
  if (!disponible()) return false;

  // Una sola vez: reconectar() vuelve a pasar por aquí, y con un oyente por llamada
  // un desenchufe acabaría disparando caida() tantas veces como reconexiones hubiera.
  if (!oyenteDesconexion) {
    oyenteDesconexion = true;
    navigator.serial.addEventListener('disconnect', (ev) => {
      if (ev.target === puerto) caida();
    });
  }

  const p = await puertoAutorizado();
  if (!p) return false;
  try {
    return await abrir(p);
  } catch {
    return false;
  }
}

export async function reconectar() {
  cerrandoAdrede = true;
  try { lector?.cancel(); } catch { /* nada que cancelar */ }
  await caida();
  cerrandoAdrede = false;
  await arrancar();
}
```

- [ ] **Paso 2: commit**

```bash
git add OrbyGUI/src/web/transport-serial.js
git commit -m "feat(web): transporte Web Serial sobre el CDC del teclado"
```

---

## Tarea 6: Copias de seguridad en el navegador

**Ficheros:**
- Crear: `OrbyGUI/src/web/backup-file.js`

**Interfaces:**
- Produce: `saveBackup(data): Promise<{ok:boolean, canceled?:boolean, error?:string}>`,
  `loadBackup(): Promise<{ok:boolean, canceled?:boolean, data?:object, error?:string}>`.
  Es la forma exacta que espera `src/backup.js:195` y `:208` (`res.canceled`, `res.ok`,
  `res.data`, `res.error`).

- [ ] **Paso 1: escribir `src/web/backup-file.js`**

```js
// Copias de seguridad sin diálogo nativo.
//
// En el escritorio esto lo hacen dialog.showSaveDialog / showOpenDialog del proceso
// principal. En el navegador se resuelve con una descarga y un <input type="file">,
// que funcionan en todos los navegadores sin pedir permisos.
//
// La forma de lo que devuelven tiene que ser la misma que la del proceso principal
// ({ ok, canceled, data, error }): src/backup.js no debe enterarse de dónde corre.

function nombreArchivo() {
  const d = new Date();
  const dos = (n) => String(n).padStart(2, '0');
  return `orby-backup-${d.getFullYear()}-${dos(d.getMonth() + 1)}-${dos(d.getDate())}`
       + `-${dos(d.getHours())}-${dos(d.getMinutes())}.json`;
}

export async function saveBackup(data) {
  try {
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = nombreArchivo();
    a.click();
    // Sin esto el blob se queda en memoria toda la sesión, y una copia con todos
    // los iconos no es precisamente pequeña.
    setTimeout(() => URL.revokeObjectURL(url), 10_000);
    return { ok: true };
  } catch (err) {
    return { ok: false, error: err.message };
  }
}

export function loadBackup() {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'application/json,.json';

    input.addEventListener('change', async () => {
      const file = input.files?.[0];
      if (!file) { resolve({ ok: false, canceled: true }); return; }
      try {
        resolve({ ok: true, data: JSON.parse(await file.text()) });
      } catch (err) {
        resolve({ ok: false, error: `El archivo no es un JSON válido: ${err.message}` });
      }
    });

    // 'cancel' no lo emiten todos los navegadores; cuando no llega, la promesa se
    // queda colgada y el botón de restaurar se queda en "Restaurando…" para
    // siempre. El foco volviendo a la ventana es la señal que sí llega en todos.
    window.addEventListener('focus', () => {
      setTimeout(() => {
        if (!input.files?.length) resolve({ ok: false, canceled: true });
      }, 500);
    }, { once: true });

    input.click();
  });
}
```

- [ ] **Paso 2: commit**

```bash
git add OrbyGUI/src/web/backup-file.js
git commit -m "feat(web): copias de seguridad por descarga y selector de archivo"
```

---

## Tarea 7: Montar `window.orby` en el navegador y arrancar de punta a punta

**Ficheros:**
- Crear: `OrbyGUI/src/web/orby-web.js`
- Crear: `OrbyGUI/src/web/connect-gate.js`
- Crear: `OrbyGUI/src/web-main.js`
- Modificar: `OrbyGUI/index.html` (hueco de la pantalla de conexión)
- Modificar: `OrbyGUI/src/styles/index.css`
- Modificar: `OrbyGUI/package.json` (script `preview:web`)

**Interfaces:**
- Consume: `initConfig`/`getConfig`/`setConfig` de `./config-store.js`; todo el módulo
  `./transport-serial.js`; `saveBackup`/`loadBackup` de `./backup-file.js`;
  `setPlatform` de `../platform.js`.
- Produce: `instalarOrbyWeb(): Promise<void>` (en `orby-web.js`),
  `montarGate(): void` (en `connect-gate.js`).

Esta es la tarea que hace que la app arranque en el navegador. Al final de ella la
WebGUI ya lee perfiles del teclado.

- [ ] **Paso 1: escribir `src/web/orby-web.js`**

```js
// El equivalente de electron/preload.js para el navegador.
//
// El renderer entero está escrito contra window.orby: 55 llamadas repartidas por
// quince ficheros. Reimplementar esa superficie sale infinitamente más barato que
// reescribir las vistas, y sobre todo evita tener dos interfaces que divergen desde
// el primer commit.
//
// Lo que aquí no puede existir (abrir programas, mover el ratón, complementos,
// detectar la ventana en primer plano) devuelve un valor vacío en vez de reventar:
// las vistas ya lo esconden preguntando a src/platform.js, y esto es la segunda red
// por si alguna se deja.

import { initConfig, getConfig, setConfig } from './config-store.js';
import { saveBackup, loadBackup } from './backup-file.js';
import * as serie from './transport-serial.js';

// Los eventos que en Electron llegan por ipcRenderer.on aquí son suscripciones al
// transporte. La firma se mantiene: un único callback, sin objeto de evento.
function puente(evento) {
  return (cb) => serie.on(evento, cb);
}

export async function instalarOrbyWeb() {
  await initConfig();

  window.orby = {
    platform: 'web',

    // Comunicación serie
    sendCommand: (cmd) => serie.enviar(cmd),
    getDeviceInfo: async () => serie.info(),
    getStatus: async () => serie.estado(),
    reconnect: () => serie.reconectar(),

    onConnected: puente('connected'),
    onDisconnected: puente('disconnected'),
    onData: puente('data'),
    onError: puente('error'),
    onSearching: puente('searching'),

    // Configuración local
    getConfig,
    setConfig,

    // Copias de seguridad
    saveBackup,
    loadBackup,

    // Nada de esto existe sin un proceso con permisos de escritorio. Devuelven
    // valores vacíos, no excepciones: una vista que se deje una comprobación tiene
    // que quedarse sin la función, no romper la página entera.
    getMousePosition: async () => null,
    pickAppOrFile: async () => ({ ok: false, canceled: true }),
    listInstalledApps: async () => [],

    plugins: {
      list: async () => [],
      install: async () => ({ ok: false }),
      uninstall: async () => ({ ok: false }),
      setEnabled: async () => ({ ok: false }),
      getSettings: async () => ({}),
      setSettings: async () => ({ ok: false }),
      test: async () => ({ ok: false, error: 'Los complementos necesitan la app de escritorio' }),
      openFolder: async () => ({ ok: false }),
    },

    autostart: {
      get: async () => false,
      set: async () => false,
    },

    recorder: {
      toggle: async () => ({ ok: false }),
      stop: async () => ({ ok: false }),
      status: async () => ({ recording: false }),
      onState: () => {},
    },

    foreground: {
      start: async () => ({ ok: false }),
      stop: async () => ({ ok: false }),
      current: async () => null,
      available: async () => false,
      onChange: () => {},
      onError: () => {},
    },

    updater: {
      get: async () => ({ status: 'idle' }),
      check: async () => null,
      install: async () => false,
      onState: () => {},
    },

    firmware: {
      get: async () => ({ status: 'idle' }),
      check: async () => null,
      update: async () => ({ ok: false }),
      cancel: async () => null,
      onState: () => {},
    },

    // No hay marco de ventana que controlar: la pestaña es del navegador.
    minimize: () => {},
    maximize: () => {},
    close: () => {},
  };
}
```

- [ ] **Paso 2: escribir `src/web/connect-gate.js`**

```js
// La pantalla que pide conectar el teclado.
//
// Web Serial no deja elegir puerto sin un gesto del usuario, así que la primera
// visita necesita sí o sí un botón. En las siguientes el permiso ya está concedido
// y el puerto se abre solo al cargar, sin que se vea nada de esto.

import * as serie from '../web/transport-serial.js';

const NAVEGADOR_NO_VALE =
  'Este navegador no sabe hablar con puertos serie. La WebGUI necesita Chrome, Edge '
  + 'u otro navegador basado en Chromium, en un ordenador (en móvil no existe la API).';

let host = null;

// El último motivo por el que falló una apertura. El transporte lo cuenta por su
// evento 'error', que llega antes de que pedirPuerto() devuelva false: guardarlo aquí
// es la forma de poder enseñarlo en la tarjeta en vez de un "no se pudo" a secas.
let ultimoError = null;

function pintar({ mensaje, error, ocupado }) {
  host.innerHTML = `
    <div class="gate-card">
      <h1>Orby WebGUI</h1>
      <p class="gate-desc">${mensaje}</p>
      ${error ? `<p class="gate-error">${error}</p>` : ''}
      <button class="primary-btn gate-btn" id="gate-connect" ${ocupado ? 'disabled' : ''}>
        ${ocupado ? 'Conectando…' : 'Conectar teclado'}
      </button>
      <p class="gate-note">
        Elige <strong>Orby V4 Control</strong> en la lista del navegador. Si no aparece,
        comprueba que el cable está enchufado y que no tienes abierta la app de
        escritorio: solo un programa puede usar el teclado a la vez.
      </p>
    </div>`;

  const btn = document.getElementById('gate-connect');
  if (btn) btn.addEventListener('click', conectar);
}

async function conectar() {
  pintar({ mensaje: 'Elige el teclado en la lista del navegador.', ocupado: true });
  const ok = await serie.pedirPuerto();
  if (ok) { ocultar(); return; }
  pintar({
    mensaje: 'No se ha conectado ningún teclado.',
    error: ultimoError || 'Vuelve a intentarlo.',
  });
}

export function ocultar() {
  host?.classList.add('hidden');
}

export function mostrar() {
  host?.classList.remove('hidden');
}

export function montarGate() {
  host = document.getElementById('web-gate');
  serie.on('error', (err) => { ultimoError = err; });

  if (!serie.disponible()) {
    host.classList.remove('hidden');
    host.innerHTML = `
      <div class="gate-card">
        <h1>Orby WebGUI</h1>
        <p class="gate-desc">${NAVEGADOR_NO_VALE}</p>
      </div>`;
    return;
  }

  pintar({ mensaje: 'Conecta el teclado por USB para empezar a configurarlo.' });
  host.classList.remove('hidden');

  // Con el teclado ya autorizado en una visita anterior, la conexión se hace sola y
  // esta pantalla no llega a verse.
  serie.on('connected', ocultar);
}
```

- [ ] **Paso 3: escribir `src/web-main.js`**

```js
// Arranque de la versión de navegador.
//
// El orden importa: window.orby y la plataforma tienen que estar puestos ANTES de
// que se cargue main.js, porque sus módulos consultan las dos cosas nada más
// importarse. Por eso main.js entra por importación dinámica.

import { setPlatform } from './platform.js';
import { instalarOrbyWeb } from './web/orby-web.js';
import { montarGate } from './web/connect-gate.js';
import * as serie from './web/transport-serial.js';

setPlatform('web');
await instalarOrbyWeb();

await import('./main.js');

// El gate se monta después de main.js: main.js pinta la pantalla de carga y engancha
// los eventos del teclado, y el gate tiene que quedar por encima de todo eso.
montarGate();

// Un puerto autorizado en una visita anterior se abre sin pedir nada. Si no hay
// ninguno, esto no hace nada y se queda el botón del gate.
serie.arrancar();
```

- [ ] **Paso 4: añadir el hueco del gate a `index.html`**

Justo antes de la etiqueta `<script type="module" src="/src/entry.js"></script>`:

```html
  <!-- Pantalla de conexión de la versión de navegador. Vive aquí y no en un
       fichero aparte porque el index.html es el mismo para las dos plataformas;
       en Electron nadie la desoculta nunca. -->
  <div id="web-gate" class="web-gate hidden"></div>
```

- [ ] **Paso 5: estilos del gate en `src/styles/index.css`**

Al final del fichero:

```css
/* --- Pantalla de conexión de la WebGUI ---------------------------------- */
/* Tapa la app entera: sin teclado no hay nada que enseñar, y dejar ver el
   editor de fondo invita a hacer clic en cosas que todavía no van. */
.web-gate {
  position: fixed;
  inset: 0;
  z-index: 900;
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--bg-app, #12141a);
}

.web-gate.hidden { display: none; }

.gate-card {
  max-width: 26rem;
  padding: 2.5rem;
  text-align: center;
}

.gate-card h1 { margin-bottom: 1rem; }
.gate-desc { margin-bottom: 1.5rem; opacity: 0.85; }
.gate-error { margin-bottom: 1rem; color: var(--danger, #ff6b6b); font-size: 0.9rem; }
.gate-btn { margin-bottom: 1.5rem; }
.gate-note { font-size: 0.8rem; opacity: 0.6; line-height: 1.5; }
```

> Comprueba los nombres de las variables CSS (`--bg-app`, `--danger`) contra los que
> ya define `src/styles/index.css`. Si no existen, usa las equivalentes que sí estén;
> los valores de reserva del `var()` solo son la red.

- [ ] **Paso 6: añadir el script de vista previa a `package.json`**

En `"scripts"`:

```json
    "preview:web": "vite build && vite preview --port 5174",
```

- [ ] **Paso 7: probarlo de punta a punta**

```bash
cd OrbyGUI && npm run dev
```

Con el teclado enchufado y **la app de escritorio cerrada** (Electron se queda el
puerto COM y el navegador no podrá abrirlo), abrir `http://localhost:5173` en Chrome.

Comprobar, en orden:

1. Sale la tarjeta "Orby WebGUI" con el botón "Conectar teclado".
2. Al pulsarlo, Chrome enseña el diálogo de puertos con **Orby V4 Control**.
3. Al elegirlo, la tarjeta desaparece y la barra superior dice
   "Orby V4 conectado · fw 4.3".
4. La vista Perfiles enseña los perfiles del teclado, con sus etiquetas e iconos OLED.
5. Cambiar el atajo de una tecla y ver que la pantalla del teclado lo refleja.
6. Recargar la página (F5): se conecta **sin** volver a pedir el puerto.
7. Desenchufar el teclado: la barra pasa a "Desconectado" y luego a "Buscando USB…".
   Al volver a enchufarlo, se reconecta solo.

> Si el paso 2 falla con `NetworkError`, hay otro proceso con el puerto abierto:
> cierra OrbyGUI de escritorio (bandeja incluida) y vuelve a probar.

- [ ] **Paso 8: commit**

```bash
git add OrbyGUI/src/web/orby-web.js OrbyGUI/src/web/connect-gate.js OrbyGUI/src/web-main.js \
        OrbyGUI/index.html OrbyGUI/src/styles/index.css OrbyGUI/package.json
git commit -m "feat(web): la app arranca en el navegador con Web Serial"
```

---

## Tarea 8: Marcar lo que no funciona sin la app abierta

**Ficheros:**
- Modificar: `OrbyGUI/src/views/profiles/macros-store.js` (añadir al final)
- Modificar: `OrbyGUI/src/views/profiles.js:1121-1152` (`renderKeyGridInner`)
- Modificar: `OrbyGUI/src/styles/index.css`

**Interfaces:**
- Consume: `macroDeviceEligible`, `macroById` (ya existen en `macros-store.js:135` y `:53`);
  `MACRO_MODIFIER`, `ROTARY_TYPES` de `../../hid-keys.js` (ya importados ahí).
- Produce: `keyNeedsApp(action): boolean`, `rotaryNeedsApp(action): boolean`.

> Esto cubre el punto de UX del TODO («advertencias visuales sobre qué acciones son
> device-eligible»). Como `src/` es compartido, la insignia sale **también en la app de
> escritorio**, que es justo lo que pide el otro punto suelto del TODO. La parte de
> marcarlo en las pantallas OLED del teclado es firmware y queda fuera de este plan.

- [ ] **Paso 1: añadir los predicados a `macros-store.js`**

Al final de `OrbyGUI/src/views/profiles/macros-store.js`:

```js
// Si esta acción la tiene que ejecutar el PC. Es lo contrario de "el teclado sabe
// tocarla solo": una secuencia con pasos que el firmware no conoce (abrir una app,
// escribir un texto, un complemento, una grabación), o una que no cabe en su
// memoria de secuencias.
//
// Sin la app abierta esa tecla no hace nada, y hasta ahora no había forma de saber
// cuál era cuál mirando la rejilla. En la WebGUI la diferencia es permanente: ahí
// no hay app de escritorio detrás, nunca.
export function keyNeedsApp(action) {
  if (!action || action.modifier !== MACRO_MODIFIER) return false;
  const m = macroById(action.keycode);
  if (!m) return false;
  return !macroDeviceEligible(m);
}

// Lo mismo para un mando giratorio. Solo las acciones de tipo "atajo" pueden llevar
// una macro dentro; el resto (multimedia, desplazar, zoom) las toca el firmware.
export function rotaryNeedsApp(action) {
  if (!action || action.type !== ROTARY_TYPES.KEY) return false;
  return keyNeedsApp(action);
}
```

- [ ] **Paso 2: importarlos en `profiles.js`**

En el bloque de importación de `./profiles/macros-store.js` (`OrbyGUI/src/views/profiles.js:36-38`),
añadir `keyNeedsApp` y `rotaryNeedsApp` a la lista de nombres importados.

- [ ] **Paso 3: pintar la insignia en la rejilla de teclas**

En `renderKeyGridInner` (`OrbyGUI/src/views/profiles.js:1122`), después de la línea
`const assigned = action.modifier || action.keycode;`, añadir:

```js
    // Esta tecla la ejecuta el PC, no el teclado: sin OrbyGUI abierta no hace nada.
    const necesitaApp = keyNeedsApp(action);
```

Y sustituir la línea del `okey-action` por:

```js
        <span class="okey-action ${assigned ? 'assigned' : ''}">${escape(describeKey(action))}${
          necesitaApp
            ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esta tecla no hace nada">${icon('plug', 11)}</i>`
            : ''}</span>
```

- [ ] **Paso 4: pintar la insignia en los mandos giratorios**

Mismo problema en las tarjetas de los encoders y de la rueda. En
`OrbyGUI/src/views/profiles.js:1194`, dentro de `.rp-body`, sustituir:

```js
              <strong>${escape(describeRotaryFull(action))}</strong>
```

por:

```js
              <strong>${escape(describeRotaryFull(action))}${
                rotaryNeedsApp(action)
                  ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${icon('plug', 11)}</i>`
                  : ''}</strong>
```

Repetir el mismo cambio en las dos líneas de la tarjeta de la rueda
(`OrbyGUI/src/views/profiles.js:1277` y `:1282`), donde las variables se llaman `cw` y
`ccw` en lugar de `action`:

```js
              <span class="rp-body"><em>Hacia abajo</em><strong>${escape(describeRotaryFull(cw))}${
                rotaryNeedsApp(cw)
                  ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${icon('plug', 11)}</i>`
                  : ''}</strong></span>
```

```js
              <span class="rp-body"><em>Hacia arriba</em><strong>${escape(describeRotaryFull(ccw))}${
                rotaryNeedsApp(ccw)
                  ? `<i class="okey-pc" title="La ejecuta el PC: sin OrbyGUI abierta esto no hace nada">${icon('plug', 11)}</i>`
                  : ''}</strong></span>
```

- [ ] **Paso 5: estilos de la insignia**

Al final de `OrbyGUI/src/styles/index.css`:

```css
/* Marca de "esto lo ejecuta el PC" en la rejilla de teclas. Va pegada al texto de
   la acción y no en una esquina: lo que califica es la acción, no la tecla. */
.okey-pc {
  display: inline-flex;
  vertical-align: -1px;
  margin-left: 0.25rem;
  opacity: 0.65;
}
```

- [ ] **Paso 6: comprobarlo**

```bash
cd OrbyGUI && npm run dev
```

En la app de escritorio, con el teclado conectado:

1. Asignar a la tecla 1 una acción de la pestaña **App** ("Abrir…"): aparece el
   iconito de enchufe junto al texto de la acción.
2. Asignar a la tecla 2 una **Secuencia** de solo teclas y esperas (la sube el
   teclado): **no** aparece el iconito.
3. Asignar a la tecla 3 un atajo normal de la pestaña **Atajo**: **no** aparece.
4. Pasar el ratón por encima del iconito: sale el globo "La ejecuta el PC…".
5. Asignar a la pulsación del encoder izquierdo una acción de **App**: el iconito
   aparece también en su tarjeta.
6. Asignar a un giro una acción de **Multimedia**: **no** aparece.

- [ ] **Paso 7: commit**

```bash
git add OrbyGUI/src/views/profiles/macros-store.js OrbyGUI/src/views/profiles.js OrbyGUI/src/styles/index.css
git commit -m "feat(perfiles): marcar las teclas que necesitan la app abierta"
```

---

## Tarea 9: Recortar la interfaz de lo que en navegador no existe

**Ficheros:**
- Modificar: `OrbyGUI/src/main.js`
- Modificar: `OrbyGUI/src/views/settings.js`
- Modificar: `OrbyGUI/src/views/profiles.js:1524-1532` (pestañas del inspector)
- Modificar: `OrbyGUI/src/styles/index.css`

**Interfaces:**
- Consume: `isWeb`, `can` de `src/platform.js` (Tarea 1).

- [ ] **Paso 1: ocultar el marco de ventana y la vista "Cambio automático"**

En `OrbyGUI/src/main.js`, añadir a las importaciones:

```js
import * as platform from './platform.js';
```

Y en `initChrome()`, al principio de la función:

```js
  // En el navegador la pestaña ya tiene su marco y su botón de cerrar, y no hay
  // proceso que vigile la ventana en primer plano: el cambio automático de perfil
  // no puede funcionar, así que su entrada no se enseña en vez de dejarla llevando
  // a una pantalla que no hace nada.
  if (!platform.can('windowChrome')) {
    document.querySelector('.titlebar-controls')?.classList.add('hidden');
  }
  if (!platform.can('autoProfile')) {
    document.querySelector('.nav-item[data-target="view-auto"]')?.classList.add('hidden');
  }
  if (!platform.can('appUpdate')) {
    document.getElementById('btn-update')?.classList.add('hidden');
  }
```

En `initUpdateBadge()`, envolver el cuerpo con una salida temprana como primera línea:

```js
  // Sin autoactualización no hay insignia que pintar, y updater.init() llamaría a
  // un window.orby.updater que en navegador no hace nada.
  if (!platform.can('appUpdate')) return;
```

En `renderUpdateBadge()`, como primera línea:

```js
  if (!platform.can('appUpdate')) return;
```

En `onDeviceConnected(info)`, envolver la comprobación de firmware del final:

```js
  // Actualizar el firmware exige copiar un .uf2 en la unidad USB que aparece al
  // reiniciar en modo carga: eso el navegador no puede hacerlo.
  if (platform.can('firmwareUpdate')) {
    firmware.check()?.then((st) => {
      if (st?.available) {
        toast(`Hay firmware nuevo para el teclado (${st.latest.version}). Ajustes → Firmware del teclado`, 'info', 8000);
      }
    }).catch(() => {});
  }
```

- [ ] **Paso 2a: dar `id` a las dos tarjetas de Ajustes que no lo tienen**

Las tarjetas de "Aplicación" (`id="settings-app"`) y "Firmware del teclado"
(`id="settings-firmware"`) ya lo llevan; las de Autoarranque y Complementos no. En
`OrbyGUI/index.html`, sobre los `<div class="settings-card glass-panel">` que contienen
`<h2>Autoarranque</h2>` y `<h2>Complementos</h2>`, dejarlos como:

```html
          <div class="settings-card glass-panel" id="settings-autostart">
```

```html
          <div class="settings-card glass-panel" id="settings-plugins">
```

- [ ] **Paso 2b: ocultar las tarjetas de Ajustes que no aplican**

En `OrbyGUI/src/views/settings.js`, añadir a las importaciones:

```js
import * as platform from '../platform.js';
```

Y al principio de `init()`, **antes** de enganchar ningún botón:

```js
  // Las tarjetas que dependen del PC se ocultan enteras en vez de deshabilitarse: un
  // interruptor de autoarranque de Windows dentro de una pestaña de Chrome no es un
  // ajuste apagado, es un ajuste que no significa nada.
  //
  // Va antes de los addEventListener porque los botones de dentro dejan de existir a
  // efectos prácticos: engancharlos igual no rompe nada, pero deja manejadores vivos
  // que llaman a un window.orby que aquí no hace nada.
  const ocultarSiFalta = (capacidad, selector) => {
    if (platform.can(capacidad)) return true;
    document.querySelector(selector)?.classList.add('hidden');
    return false;
  };

  const hayAutostart = ocultarSiFalta('autostart', '#settings-autostart');
  const hayPlugins   = ocultarSiFalta('plugins', '#settings-plugins');
  ocultarSiFalta('appUpdate', '#settings-app');
  ocultarSiFalta('firmwareUpdate', '#settings-firmware');
```

Después, envolver con `if (hayAutostart) { ... }` las líneas de `init()` que enganchan
`#btn-autostart` y leen `window.orby.autostart.get()`, y con `if (hayPlugins) { ... }`
las que enganchan `#btn-plugin-install`, `#btn-plugin-folder` y llaman a `plugins.init()`
o `plugins.onChange(...)`.

En `render()`, salir temprano de los tramos que pintan `#plugin-list`,
`#plugin-settings-cards`, `#app-version`, `#app-update-status` y las de `#fw-*`
comprobando la misma capacidad: `render()` se llama en cada `notify()` y buscaría
elementos que ya no están en el DOM.

- [ ] **Paso 3: deshabilitar las pestañas del inspector que necesitan el PC**

En `OrbyGUI/src/views/profiles.js`, añadir a las importaciones:

```js
import * as platform from '../platform.js';
```

Sustituir el bloque `<div class="inspector-tabs">` (`OrbyGUI/src/views/profiles.js:1524-1532`) por:

```js
      <div class="inspector-tabs">
        ${[
          { id: 'shortcut', label: 'Atajo',      cap: null },
          { id: 'sequence', label: 'Secuencia',  cap: null },
          { id: 'text',     label: 'Texto',      cap: 'text' },
          { id: 'record',   label: 'Grabar',     cap: 'recorder' },
          { id: 'app',      label: 'App',        cap: 'openApp' },
          { id: 'media',    label: 'Multimedia', cap: null },
        ].map(({ id, label, cap }) => {
          // Una pestaña que aquí no puede funcionar se deja a la vista pero apagada:
          // esconderla dejaría al usuario preguntándose dónde está la opción que sí
          // tenía en el PC, y "abrir una app" no es algo que un navegador vaya a poder
          // hacer nunca.
          const apagada = cap && !platform.can(cap);
          return `<button class="inspector-tab ${tab === id ? 'active' : ''} ${apagada ? 'unsupported' : ''}"
                          ${apagada ? 'disabled title="Necesita OrbyGUI de escritorio"' : ''}
                          data-act="set-tab" data-tab="${id}">${label}</button>`;
        }).join('')}
        ${hasPages() ? `<button class="inspector-tab ${tab === 'pages' ? 'active' : ''}" data-act="set-tab" data-tab="pages">Páginas</button>` : ''}
      </div>
```

> La pestaña **Secuencia** se queda activa: una secuencia de teclas, esperas, clics y
> desplazamientos relativos la toca el propio teclado (`DEVICE_STEP_TYPE` en
> `macros-store.js:97`). Lo que hay que apagar dentro de ella son los pasos que el
> firmware no conoce.

- [ ] **Paso 4: apagar los pasos no soportados dentro de la pestaña Secuencia**

La pestaña Secuencia sigue viva, pero tres de sus seis botones de "añadir paso" meten
pasos que el firmware no sabe tocar. Están en
`OrbyGUI/src/views/profiles/sequence-editor.js:242-248` y `:263`:

| Botón | `data-act` | Tipo de paso | ¿Lo toca el teclado? |
|---|---|---|---|
| Posición de ratón | `seq-add-position` | `mouse_position` | **No** → `pcSequences` |
| Clic | `seq-add-click` | `mouse_click` | Sí |
| Mover ratón | `seq-add-move` | `mouse_move` | Sí |
| Escribir texto | `seq-add-text` | `text` | **No** → `text` |
| Abrir app/archivo | `seq-add-open` | `open_app` | **No** → `openApp` |
| Añadir (atajo) | `seq-add-hotkey` | `hotkey` | Sí |

Añadir a las importaciones de `sequence-editor.js`:

```js
import * as platform from '../../platform.js';
```

Y a los tres botones que no valen, el mismo tratamiento que a las pestañas. Por ejemplo,
el de "Escribir texto":

```js
        <button class="secondary-btn ${platform.can('text') ? '' : 'unsupported'}"
                ${platform.can('text') ? '' : 'disabled title="Necesita OrbyGUI de escritorio"'}
                data-act="seq-add-text">${icon('pencil', 16)} Escribir texto</button>
```

El de "Abrir app/archivo" igual con `openApp`, y el de "Posición de ratón" con
`pcSequences` (ese además captura la posición del cursor con `getMousePosition`, que en
navegador devuelve `null`).

> Los otros tres se quedan como están: clic, mover ratón y atajo son justo los tipos de
> `DEVICE_STEP_TYPE` (`macros-store.js:97`), los que el teclado reproduce él solo. Una
> secuencia hecha solo con ellos funciona en la WebGUI y sigue funcionando después con
> el teclado desenchufado del PC.

- [ ] **Paso 5: estilo de las pestañas apagadas**

Al final de `OrbyGUI/src/styles/index.css`:

```css
/* Pestaña o botón que en esta plataforma no puede funcionar. Se ve, pero no se
   pulsa: el usuario tiene que poder averiguar que la opción existe y por qué no
   está disponible aquí. */
.inspector-tab.unsupported,
.secondary-btn.unsupported {
  opacity: 0.4;
  cursor: not-allowed;
}
```

- [ ] **Paso 6: comprobar las dos plataformas**

```bash
cd OrbyGUI && npm run dev
```

En **Electron**: nada ha cambiado. Están los botones de ventana, la entrada "Cambio
automático", las tarjetas de Autoarranque, Complementos, Aplicación y Firmware, y las
siete pestañas del inspector activas.

En **Chrome** (`http://localhost:5173`, con la app de escritorio cerrada):

1. No hay botones de minimizar/maximizar/cerrar.
2. La barra lateral no tiene "Cambio automático".
3. Ajustes solo enseña: Reposo automático, Copia de seguridad, Información del
   dispositivo y Calibración de la rueda.
4. En el inspector de una tecla, "Texto", "Grabar" y "App" están en gris y no se
   pueden pulsar; "Atajo", "Secuencia", "Multimedia" y "Páginas" funcionan.
5. Dentro de "Secuencia", los botones "Posición de ratón", "Escribir texto" y
   "Abrir app/archivo" están en gris; "Clic", "Mover ratón" y "Añadir" funcionan.
6. Montar una secuencia de atajo + espera + clic, asignarla a una tecla, guardarla en
   Flash y comprobar que la tecla la reproduce **con la pestaña del navegador cerrada**:
   es la prueba de que la WebGUI configura algo que luego vive sin ella.
7. La consola del navegador **no** tiene errores.

- [ ] **Paso 7: commit**

```bash
git add OrbyGUI/src/main.js OrbyGUI/src/views/settings.js OrbyGUI/src/views/profiles.js \
        OrbyGUI/src/views/profiles/sequence-editor.js OrbyGUI/index.html OrbyGUI/src/styles/index.css
git commit -m "feat(web): esconder en el navegador lo que necesita el PC"
```

---

## Tarea 10: Publicar la WebGUI en GitHub Pages

**Ficheros:**
- Modificar: `.github/workflows/pages.yml` (en la raíz del repositorio, no dentro de `ORBY_V4/`)
- Modificar: `ORBY_V4/web/index.html` (enlace a la app)

La landing sigue en `/` y la app pasa a estar en `/app/`. El build es el mismo que ya
usa Electron (`npm run build` → `dist/`), porque `index.html` y el bundle son idénticos
en las dos plataformas: la única diferencia la decide `src/entry.js` en tiempo de
ejecución. `base: './'` ya está puesto en `vite.config.mjs`, así que la app funciona
igual colgada de un subdirectorio.

- [ ] **Paso 1: reescribir el workflow**

Sustituir el contenido de `.github/workflows/pages.yml` por:

```yaml
name: Publicar la landing y la WebGUI de ORBY

# La landing es un único index.html estático; la WebGUI sí necesita build (vite).
# Salen juntas: la landing en la raíz y la app en /app/. Este workflow no hace nada
# hasta que en Ajustes → Pages se seleccione "GitHub Actions" como origen.
on:
  push:
    branches: [main]
    paths:
      - 'KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/web/**'
      - 'KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/src/**'
      - 'KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/index.html'
      - 'KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/package*.json'
      - '.github/workflows/pages.yml'
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

# Una publicación a la vez: dos a la vez se pisan y Pages rechaza la segunda.
concurrency:
  group: pages
  cancel-in-progress: true

env:
  APP_DIR: KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI
  WEB_DIR: KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/web

jobs:
  publicar:
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.despliegue.outputs.page_url }}
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-node@v4
        with:
          node-version: '20'
          cache: npm
          cache-dependency-path: ${{ env.APP_DIR }}/package-lock.json

      # --ignore-scripts se salta el postinstall (electron-builder install-app-deps)
      # y la descarga del binario de Electron: ~100 MB que aquí no pinta nada, porque
      # lo único que se compila es el renderer.
      - name: Instalar dependencias
        working-directory: ${{ env.APP_DIR }}
        run: npm ci --ignore-scripts

      - name: Tests
        working-directory: ${{ env.APP_DIR }}
        run: npm test

      - name: Compilar la WebGUI
        working-directory: ${{ env.APP_DIR }}
        run: npm run build

      - name: Montar el sitio
        run: |
          mkdir -p _site
          cp -r "$WEB_DIR"/. _site/
          mkdir -p _site/app
          cp -r "$APP_DIR"/dist/. _site/app/

      - uses: actions/configure-pages@v5

      - uses: actions/upload-pages-artifact@v3
        with:
          path: _site

      - id: despliegue
        uses: actions/deploy-pages@v4
```

- [ ] **Paso 2: enlazar la app desde la landing**

En `ORBY_V4/web/index.html`, junto al botón de descarga que ya haya, añadir un enlace a
`./app/` con un texto en la línea de:

```html
        <a class="btn secondary" href="./app/">
          Configurarlo desde el navegador
          <small>Sin instalar nada. Necesita Chrome o Edge.</small>
        </a>
```

Ajusta las clases a las que use la landing.

- [ ] **Paso 3: comprobar el build en local antes de empujar**

```bash
cd OrbyGUI && npm test && npm run build && npx vite preview --port 5174
```

Abrir `http://localhost:5174` en Chrome: la WebGUI arranca igual que en `npm run dev`.
Esperado de `npm run build`: termina con `built in ...` y sin ningún `error`.

> Este paso comprueba lo importante que `npm run dev` no comprueba: que el bundle de
> producción, con las rutas relativas de `base: './'`, sigue arrancando.

- [ ] **Paso 4: commit**

```bash
git add .github/workflows/pages.yml KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/web/index.html
git commit -m "ci: publicar la WebGUI junto a la landing"
```

> **Después de empujar:** hay que entrar a Settings → Pages del repositorio y poner
> Source: **GitHub Actions**. Sin eso el workflow corre pero no publica nada. La app
> quedará en `https://<usuario>.github.io/<repo>/app/`.

---

## Tarea 11: Documentación

**Ficheros:**
- Crear: `OrbyGUI/docs/WEBGUI.md`
- Modificar: `OrbyGUI/docs/TODO.md`
- Modificar: `ORBY_V4/CLAUDE.md`

- [ ] **Paso 1: escribir `OrbyGUI/docs/WEBGUI.md`**

Con estas secciones, en castellano:

- **Qué es**: la misma app, corriendo en el navegador contra el mismo CDC, para
  configurar el teclado en un PC donde no se puede instalar nada.
- **Qué necesita**: Chrome o Edge de escritorio (Web Serial no existe en Firefox,
  Safari ni en móvil), HTTPS o localhost, y que la app de escritorio esté cerrada
  (en Windows solo un proceso puede tener el COM abierto).
- **Qué no puede hacer y por qué**: abrir aplicaciones, escribir textos, grabar ratón
  y teclado, complementos, cambio automático por ventana, autoarranque, actualizar la
  app o el firmware. Todo eso necesita un proceso con permisos de escritorio.
- **Qué sí puede hacer**: perfiles, páginas, etiquetas, atajos, mandos, rueda, iconos
  OLED, copias de seguridad, y las secuencias que el propio teclado sabe tocar
  (`macroDeviceEligible`).
- **Cómo está montada**: `src/entry.js` decide la vía; `src/web/orby-web.js` monta el
  mismo `window.orby` que el preload de Electron; `src/platform.js` es el único sitio
  donde se pregunta dónde corre la app.
- **Al añadir una función que necesite el PC**: añadir su nombre a `PC_ONLY` en
  `src/platform.js` y un valor vacío en `src/web/orby-web.js`.
- **Dónde se publica**: `.github/workflows/pages.yml`, en `/app/`.
- **Chrome corporativo**: la política `DefaultSerialGuardSetting` puede bloquear Web
  Serial en un equipo gestionado. Si el diálogo de puertos no aparece nunca, es eso.

- [ ] **Paso 2: actualizar `OrbyGUI/docs/TODO.md`**

En la sección "Orby WebGUI", marcar como `[X]`:

- Toda la sección "Investigación y Pruebas de Concepto", anotando que se resolvió con
  **Web Serial** en lugar de WebHID y por qué (Chrome bloquea WebHID sobre colecciones
  de teclado/ratón/consumer; con Web Serial no hay que tocar el firmware).
- Toda la sección "Adaptación del Firmware", anotando que **no hizo falta ninguna**: el
  guardado en Flash y la reproducción autónoma de secuencias ya existían desde el
  firmware 4.0, y el endpoint Raw HID sobra con Web Serial.
- Toda la sección "Desarrollo de la Web App".
- Los dos puntos de "Experiencia de Usuario", anotando que el cambio manual de perfil
  desde el teclado ya existía (menú de la tecla 12 + telemetría `EV:CTX`).

Y en el punto suelto sobre device-eligible (el de "que haya un pequeño simbolito"),
marcar como hecha la parte de la interfaz y dejar pendiente la del OLED del teclado.

- [ ] **Paso 3: actualizar `ORBY_V4/CLAUDE.md`**

- En la cabecera, junto a la descripción de OrbyGUI, decir que la misma app corre en
  el navegador por Web Serial.
- En "Comandos → OrbyGUI", añadir `npm test` y `npm run preview:web`.
- En "Arquitectura", una subsección corta "OrbyGUI — versión de navegador" que apunte
  a `docs/WEBGUI.md` y diga la regla: **al añadir algo que necesite el proceso
  principal, hay que tocar `src/platform.js` (`PC_ONLY`) y `src/web/orby-web.js`**.
- En "Trampas conocidas": la app de escritorio y la WebGUI **no pueden usar el teclado
  a la vez** — en Windows solo un proceso puede tener abierto el puerto COM.

- [ ] **Paso 4: commit**

```bash
git add OrbyGUI/docs/WEBGUI.md OrbyGUI/docs/TODO.md CLAUDE.md
git commit -m "docs: la versión de navegador de OrbyGUI"
```

---

## Repaso final antes de dar el trabajo por hecho

- [ ] `cd OrbyGUI && npm test` → todo pasa.
- [ ] `cd OrbyGUI && npm run build` → sin errores.
- [ ] `npm run dev` + Electron → la app de escritorio se comporta exactamente igual que
      antes de empezar: perfiles, iconos, secuencias, complementos, cambio automático,
      autoarranque y actualizaciones.
- [ ] Chrome contra `localhost:5173` → conecta, lee perfiles, edita una tecla, guarda en
      Flash, hace una copia de seguridad y la restaura.
- [ ] Recargar la pestaña reconecta sin volver a pedir permiso.
- [ ] Desenchufar y volver a enchufar el teclado reconecta solo.
- [ ] `grep -rn "window.orby" OrbyGUI/src/` → todas las claves que aparecen existen
      también en `src/web/orby-web.js`. Es la comprobación que evita el fallo más
      probable de todo el plan: una vista que llama a algo que en navegador no está.
