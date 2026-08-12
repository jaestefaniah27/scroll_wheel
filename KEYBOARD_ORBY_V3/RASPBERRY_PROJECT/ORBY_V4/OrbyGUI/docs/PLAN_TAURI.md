# OrbyGUI sobre Tauri — Plan de implementación

> **Para quien lo ejecute (agente o persona):** las tareas van en orden y cada una es
> autosuficiente. **Lee solo la tarea que vas a hacer**, no el plan entero: todo lo que
> necesitas saber de las demás está resumido en «Constantes globales» y en las tablas de
> referencia de aquí arriba. Los pasos llevan casilla (`- [ ]`) para ir marcándolos.
>
> **No explores el código para «entender el contexto».** Las tablas de este documento ya
> traen extraído lo que hace falta (los 39 canales, la superficie de `window.orby`, las
> constantes del protocolo). Si algo no cuadra con el código real, **para y avisa**: es un
> fallo del plan, no una invitación a improvisar.

**Objetivo:** que OrbyGUI deje de ser Electron y pase a ser Tauri, con la misma
funcionalidad, un instalador mucho más pequeño y menos RAM en reposo.

**Arquitectura:** el renderer entero está escrito contra un único objeto global,
`window.orby`. Ya hay **dos** implementaciones de esa superficie: `electron/preload.js`
(escritorio) y `src/web/orby-web.js` (navegador, Web Serial). Tauri es la **tercera**.
Por eso las vistas, `device.js`, `store.js`, `mirror.js` y `compat.js` **no se tocan**:
se reimplementa el objeto, no la aplicación.

**Stack:** Rust estable + Tauri 2, con el mismo frontend de siempre (JS vanilla con
módulos ES, vite 7). Tests de Rust con `cargo test`; tests de JS con `node --test`.

---

## Constantes globales

Todas las tareas heredan esto:

- **Todo en castellano de España**: comentarios, textos de interfaz, mensajes de commit,
  documentación.
- Los comentarios explican **por qué**, no qué. Documentan la decisión y el fallo que
  evitan. Un comentario que repite el código sobra.
- **Sin framework en el renderer, sin TypeScript.** No introducir ninguno.
- **El firmware no se toca en todo este plan.** Ni `main.cpp`, ni `src/usb_descriptors.c`,
  ni `include/orby_version.h`. El protocolo serie tampoco cambia.
- **No se bifurcan las vistas.** Si un módulo de `src/` necesita saber dónde corre,
  pregunta a `src/platform.js`. Nunca a `navigator.userAgent` ni comprobando si existe
  tal o cual método de `window.orby`.
- **Electron sigue funcionando hasta la última tarea.** No se borra nada de `electron/`
  hasta la Tarea 13. Durante todo el plan conviven las tres vías.
- **Objetivo solo Windows.** No hay que hacer que compile en Linux ni en macOS.
- **Rama de trabajo:** `claude/react-native-app-efficiency-tek728`.
- **Directorio de trabajo:** los `cargo` se ejecutan desde `OrbyGUI/src-tauri/`; los `npm`
  y `node` desde `OrbyGUI/`; los `git` desde la raíz del repositorio.

### Constantes del protocolo y del puerto

Copiadas del código real. **No hay que ir a buscarlas.**

| Constante | Valor | Por qué |
|---|---|---|
| VID del teclado | `0xCafe` | VID de los ejemplos de TinyUSB |
| PID | **cualquiera** | Sube con cada cambio de informe HID; filtrar por PID deja fuera firmwares viejos |
| Baudios | `115200` | |
| DTR y RTS | `true` | TinyUSB no acepta RX sin ellos |
| Primer `ACK` | +400 ms tras abrir | |
| Espera de handshake | `1200` ms, dos intentos | Si no contesta, **soltar el puerto** |
| Watchdog: silencio | `12 000` ms → mandar `ACK` | |
| Watchdog: sin respuesta | `4 000` ms → desconectar | |
| Caducidad de `HOST_APP` | 25 s en el teclado | Por eso se reenvía en **cada** handshake |
| Intervalo de rastreo de puertos | `3 000` ms | |
| Tiempo máximo de apertura | `6 000` ms | Y suelta el puerto al agotarse |

**Reglas que rompen la app en silencio si se pierden:**

1. `serial:data` entrega **todas** las líneas al renderer, también la telemetría no
   solicitada (`EV:CTX:`, `ENC:`, `KEY_EV:`). `device.js` depende de ello.
2. Nunca dejar abierto un puerto cuyo handshake falló: Windows da «Access denied» con los
   handles huérfanos.
3. `HOST_APP:1` se reenvía en cada handshake, **incluido el del watchdog**.
4. Un `MACRO:<id>` con solo dígitos es un disparo; `MACRO:OK:…`, `MACRO:<id>:STEP:…` y
   `MACRO:<id>:END` son respuestas. Ya está resuelto en `orby_core::protocolo::macro_trigger`.
5. `deviceMirror` se **sustituye entero**, no se fusiona. Ya está en `orby_core::config::merge`.
6. Las versiones se comparan **por partes**: `4.10 > 4.9`. Ya está en `orby_core::fw::compare_fw`.

---

## Lo que ya está hecho

Comprobado el 2026-08-11. **No hay que reimplementarlo.**

| Pieza | Dónde | Estado |
|---|---|---|
| Partido de líneas del CDC | `src-tauri/crates/orby-core/src/lines.rs` | Hecho, 4 tests |
| Fusión de configuración + valores por defecto | `.../src/config.rs` | Hecho, 5 tests |
| Comparación de versiones de firmware | `.../src/fw.rs` | Hecho, 4 tests |
| Saludo, `MACRO:<id>`, `CAMBIA_ALGO` | `.../src/protocolo.rs` | Hecho, 4 tests |
| Costura del frontend (**Tarea 1 entera**) | `src/entry.js`, `src/tauri-main.js`, `src/tauri/orby-tauri.js`, `src/platform.js` | Hecho, con `test/superficie-orby.test.mjs` |
| Manifiesto de complementos y descriptor | `.../src/plugins/manifiesto.rs` | Hecho, 18 tests |
| Plantillas `{{...}}` | `.../src/plugins/plantilla.rs` | Hecho, 12 tests |
| Agrupado de giros (`coalesce`) | `.../src/plugins/coalesce.rs` | Hecho, 8 tests |
| Manifiesto de lampdesk en API 2 | `.../tests/fixtures/lampdesk-api2.plugin.json` | Escrito; la Tarea 10 lo pone en su sitio |
| Máquina de estados del serie (**Tarea 3, paso 1**) | `.../src/serie.rs` | Hecho, 16 tests |
| Montaje de peticiones de complemento | `.../src/plugins/peticion.rs` | Hecho, 14 tests |
| Presupuesto de refresco de las OLED (**Tarea 11, paso 4**) | `.../src/plugins/oled.rs` | Hecho, 9 tests |
| Filtrado de releases de firmware (**Tarea 9, paso 1**) | `.../src/releases.rs` | Hecho, 9 tests |
| Crate de la app y ventana (**Tarea 2 entera**) | `src-tauri/{Cargo.toml,build.rs,tauri.conf.json,capabilities/,src/main.rs}` | Hecho, ventana comprobada a mano |
| Traducción de teclas HID → Windows (**Tarea 6, paso 4**) | `.../src/teclas.rs` | Hecho, 7 tests |
| Grabadora: filtrado, recorte y ritmo (**Tarea 7**) | `.../src/grabacion.rs` | Hecho, 10 tests |
| Puente de códigos de tecla de uiohook (**Tarea 7**) | `.../src/uiohook.rs` | Hecho, 9 tests |

Las pruebas del primer bloque se contrastaron ejecutando las implementaciones JS actuales
con las mismas entradas: coinciden caso por caso. Y el descriptor de complementos se
compara contra un fichero de referencia **generado desde el propio `electron/plugins.js`**,
así que la igualdad con lo que ve hoy el renderer está comprobada, no supuesta.

```bash
cd OrbyGUI/src-tauri/crates/orby-core && cargo test
# Esperado: 126 passed (lib) + 5 passed (descriptor_lampdesk)

cd OrbyGUI && node --test test/*.mjs
# Esperado: # pass 14
```

---

## Referencia: la superficie `window.orby`

Es **el contrato**. Tauri tiene que producir este objeto con estos nombres exactos y
estas formas de dato exactas. Origen: `electron/preload.js`.

**Nivel superior** (18 funciones + `platform`):

| Nombre | Canal | Devuelve |
|---|---|---|
| `platform` | — | la cadena `'tauri'` |
| `sendCommand(cmd)` | `serial:send` | `boolean` |
| `getDeviceInfo()` | `serial:getInfo` | objeto de info \| `null` |
| `getStatus()` | `serial:getStatus` | `'connected'\|'searching'\|'disconnected'` |
| `reconnect()` | `serial:reconnect` | `boolean` |
| `onConnected(cb)` | evento `serial:connected` | cb(info) |
| `onDisconnected(cb)` | evento `serial:disconnected` | cb() |
| `onData(cb)` | evento `serial:data` | cb(línea) |
| `onError(cb)` | evento `serial:error` | cb(texto) |
| `onSearching(cb)` | evento `serial:searching` | cb() |
| `getMousePosition()` | `mouse:getPosition` | `{x,y}` |
| `getConfig()` | `config:get` | configuración completa |
| `setConfig(patch)` | `config:set` | configuración **ya fusionada** |
| `saveBackup(data)` | `backup:save` | `{ok,path}` \| `{ok:false,canceled\|error}` |
| `loadBackup()` | `backup:load` | `{ok,data}` \| `{ok:false,canceled\|error}` |
| `pickAppOrFile(kind)` | `dialog:pickAppOrFile` | `{ok,path}` \| `{ok:false,canceled}` |
| `listInstalledApps()` | `apps:listInstalled` | `[{name,target}]` |
| `minimize()` / `maximize()` / `close()` | `window:*` | nada |

**Espacios de nombres:**

| Espacio | Métodos |
|---|---|
| `plugins` | `list, install, uninstall, setEnabled, getSettings, setSettings, test, read, openFolder` |
| `autostart` | `get, set` |
| `recorder` | `toggle(id), stop, status, onState(cb)` |
| `foreground` | `start, stop, current, available, onChange(cb), onError(cb)` |
| `updater` | `get, check, install, onState(cb)` |
| `firmware` | `get, check(opts), update(opts), cancel, onState(cb)` |

**Los 10 eventos** que van del backend al renderer: `serial:connected`,
`serial:disconnected`, `serial:error`, `serial:searching`, `serial:data`,
`firmware:state`, `recorder:state`, `updater:state`, `foreground:change`,
`foreground:error`.

**Detalles del contrato que no se ven en la tabla:**

- Los `on*` **no tienen forma de darse de baja**. Se suscriben y ya. No inventar una.
- Los callbacks reciben **solo la carga**, nunca el objeto de evento.
- `recorder:state` lleva `{id, phase}` donde `phase` es una **cadena**
  (`'reset'|'saved'|'empty'|'playing'|'idle'|'recording'`).
- `updater:state` y `firmware:state` mandan el **estado entero**, no diferencias.
- Todo lo que en Electron es `ipcMain.handle` es asíncrono: en `orby-tauri.js` todos esos
  métodos devuelven promesa.

---

## Estructura de ficheros

**Se crean:**

| Fichero | Responsabilidad |
|---|---|
| `OrbyGUI/src/tauri-main.js` | Arranque de la vía Tauri: monta `window.orby` y luego carga `main.js` |
| `OrbyGUI/src/tauri/orby-tauri.js` | La superficie `window.orby` sobre `invoke()` y `listen()` |
| `OrbyGUI/src-tauri/Cargo.toml` | Crate de la app |
| `OrbyGUI/src-tauri/tauri.conf.json` | Configuración de Tauri |
| `OrbyGUI/src-tauri/src/main.rs` | Punto de entrada y registro de comandos |
| `OrbyGUI/src-tauri/src/*.rs` | Un módulo por área (serie, config, macros, …) |
| `OrbyGUI/src-tauri/crates/orby-core/src/plugins/*.rs` | Motor de complementos |

**Se modifican:**

| Fichero | Cambio |
|---|---|
| `OrbyGUI/src/entry.js` | Detecta Tauri antes que Electron y que navegador |
| `OrbyGUI/src/platform.js` | Acepta la plataforma `'tauri'` |
| `OrbyGUI/plugins/lampdesk/` | Se reescribe como manifiesto declarativo |
| `OrbyGUI/package.json` | Scripts de Tauri |
| `ORBY_V4/CLAUDE.md`, `docs/PLUGINS.md`, `docs/WEBGUI.md`, `docs/PUBLICACION.md` | Documentación (Tarea 13) |

**No se tocan, en ninguna tarea:**

`src/views/**`, `src/device.js`, `src/store.js`, `src/mirror.js`, `src/compat.js`,
`src/oled-*.js`, `src/variants.js`, `index.html` (salvo la CSP en la Tarea 2),
`src/web/**`, y todo el firmware.

---

## Cómo se verifica cada tarea

Tres niveles. **Una tarea no está hecha hasta que pasa el nivel que le toque.**

1. **`cargo test`** — corre en cualquier máquina, incluidos los contenedores Linux de CI.
   Todo lo que sea lógica pura va aquí, en `orby-core`.
2. **Comprobación sin hardware** — `cargo test` con dobles: servidor HTTP de mentira,
   proceso de mentira, dúplex en memoria en vez del puerto.
3. **Sobre el teclado, en Windows** — solo lo que no se puede simular. Cada tarea dice
   exactamente qué mirar. **Si no tienes un teclado delante, deja la casilla sin marcar y
   dilo**; no la des por buena.

> **Aviso sobre el entorno de desarrollo:** en Linux no se puede compilar el crate de la
> app (necesita `webkit2gtk`), pero **sí** `orby-core`. Por eso toda la lógica vive ahí.
> Si estás en Linux, usa `cargo test -p orby-core` o entra en `crates/orby-core/`.

---

## Tarea 1: La costura del frontend

Montar la tercera vía de arranque. Al acabar, la app **sigue funcionando igual** en
Electron y en navegador, y existe el esqueleto de la de Tauri.

**Ficheros:**
- Modificar: `OrbyGUI/src/entry.js`
- Modificar: `OrbyGUI/src/platform.js`
- Crear: `OrbyGUI/src/tauri-main.js`
- Crear: `OrbyGUI/src/tauri/orby-tauri.js`

**Interfaces:**
- Produce: `instalarOrbyTauri(): Promise<void>`, que deja `window.orby` montado.
- Produce: `platform.isTauri(): boolean`.

- [x] **Paso 1: `src/platform.js`**

Hoy `current` vale `'electron'` o `'web'`. Añadir `'tauri'`. La clave es que `can()`
siga devolviendo `true` para todo salvo en navegador: Tauri aspira a paridad, así que
**no entra en `PC_ONLY`**.

Añadir junto a `isWeb()`:

```js
export function isTauri() {
  return current === 'tauri';
}
```

Y dejar `can()` exactamente como está: `return !isWeb() || !PC_ONLY.has(feature);`. Como
`isWeb()` es falso en Tauri, `can()` da `true` para todo. Es lo que queremos.

> **Mientras la migración esté a medias**, si hace falta esconder algo aún no portado,
> añadir un `TAURI_PENDIENTE` (un `Set` como `PC_ONLY`) y consultarlo en `can()`. Se borra
> en la Tarea 13. No esparcir comprobaciones por las vistas.

- [x] **Paso 2: `src/entry.js`**

Tauri inyecta `window.__TAURI_INTERNALS__` antes de que corra ningún script de la página,
igual que el preload de Electron inyecta `window.orby`. Queda:

```js
// La misma app arranca en tres sitios: dentro de Tauri, dentro de Electron y dentro de
// un navegador.
//
// Electron y Tauri traen su puente puesto antes de que este script corra; el navegador
// no tiene ninguno y hay que montarlo (Web Serial + IndexedDB) antes de que ningún
// módulo de src/ toque `window.orby`. Por eso main.js entra siempre por importación
// dinámica y no por `import` normal.

if (window.__TAURI_INTERNALS__) {
  await import('./tauri-main.js');
} else if (window.orby) {
  await import('./main.js');
} else {
  await import('./web-main.js');
}
```

> **El orden importa.** Tauri se comprueba **primero**: si algún día montara también un
> `window.orby` propio, la rama de Electron se lo tragaría y arrancaría por la vía
> equivocada sin decir nada.

- [x] **Paso 3: `src/tauri-main.js`**

Espejo de `web-main.js`:

```js
// Arranque de la vía Tauri. El puente lo monta este módulo (no hay preload que lo haga
// por nosotros), así que tiene que estar puesto antes de que main.js importe nada de
// src/, y por eso main.js entra por importación dinámica.

import { setPlatform } from './platform.js';
import { instalarOrbyTauri } from './tauri/orby-tauri.js';

setPlatform('tauri');
await instalarOrbyTauri();
await import('./main.js');
```

- [x] **Paso 4: `src/tauri/orby-tauri.js`**

La superficie completa, sobre `invoke` y `listen`. Usa la tabla «Referencia: la
superficie `window.orby`» de este documento: **los 39 canales, con sus nombres exactos**.

Reglas al escribirlo:

- Los nombres de canal se conservan tal cual (`'serial:send'`, `'plugins:list'`, …). No
  renombrar a `snake_case` aunque sea lo idiomático en Rust: el coste de traducir dos
  veces no lo paga nadie y el contrato es el de Electron.
- `invoke` recibe **un objeto** de argumentos. Convención de este proyecto: el comando de
  Rust declara los parámetros con el mismo nombre que usa el renderer.
- Los `on*` envuelven `listen(evento, e => cb(e.payload))`: el renderer nunca ve el objeto
  de evento, solo la carga.
- Nada de dar de baja suscripciones: el contrato de Electron no lo tiene.

Estructura (abreviada; complétala con la tabla):

```js
// La API sale del global que inyecta Tauri, no del paquete `@tauri-apps/api`.
const { core, event } = window.__TAURI__ ?? {};
const invoke = (cmd, args) => core.invoke(cmd, args);
const puente = (evento) => (cb) => { event.listen(evento, (e) => cb(e.payload)); };

export async function instalarOrbyTauri() {
  window.orby = {
    platform: 'tauri',

    sendCommand: (cmd) => invoke('serial_send', { cmd }),
    getDeviceInfo: () => invoke('serial_get_info'),
    getStatus: () => invoke('serial_get_status'),
    reconnect: () => invoke('serial_reconnect'),

    onConnected: puente('serial:connected'),
    onDisconnected: puente('serial:disconnected'),
    onData: puente('serial:data'),
    onError: puente('serial:error'),
    onSearching: puente('serial:searching'),

    // … el resto, siguiendo la tabla de referencia
  };
}
```

> **Por qué el global y no `import { invoke } from '@tauri-apps/api'`:** ese `import` lo
> tendría que resolver vite en **los tres** builds, incluido el de navegador, así que una
> dependencia que solo usa una de las vías obligaría a instalarla para compilar
> cualquiera de las otras. Con el global, el frontend sigue compilando sin tocar
> `package.json` ni el `package-lock.json`. **Exige `withGlobalTauri: true` en
> `tauri.conf.json`** (Tarea 2); si se olvida, `window.__TAURI__` llega `undefined` y la
> app arranca en blanco sin decir por qué.

> **Sobre los nombres de comando:** Tauri no admite `:` en el nombre de un comando de
> Rust, así que el **comando** se llama `serial_send` y el **evento** sigue llamándose
> `serial:connected`. Los eventos sí admiten `:` y hay que conservarlos idénticos porque
> son los que ya conoce el renderer.

- [x] **Paso 5: comprobar que no se ha roto nada**

```bash
cd OrbyGUI && node --test test/*.mjs && npx vite build
```

Esperado: `# pass 8`, y el build de vite termina sin error. Esto solo demuestra que el
JS es sintácticamente válido y que las otras dos vías siguen en pie; la vía de Tauri no
arranca todavía porque no existe el backend.

- [x] **Paso 6: commit**

```bash
git add OrbyGUI/src/entry.js OrbyGUI/src/platform.js OrbyGUI/src/tauri-main.js OrbyGUI/src/tauri/
git commit -m "feat(tauri): la tercera vía de arranque del frontend"
```

---

## Tarea 2: El crate de la app y una ventana en blanco

**Ficheros:**
- Crear: `OrbyGUI/src-tauri/Cargo.toml`, `tauri.conf.json`, `build.rs`, `src/main.rs`
- Modificar: `OrbyGUI/package.json` (scripts)
- Modificar: `OrbyGUI/index.html` (solo la CSP)

- [x] **Paso 1: el crate**

`src-tauri/Cargo.toml` declara el paquete de la app y toma `orby-core` por ruta. Hay que
**quitar** el `[workspace]` vacío de `crates/orby-core/Cargo.toml` y declarar aquí el
espacio de trabajo:

```toml
[package]
name = "orby-app"
version = "0.5.1"
edition = "2021"

[workspace]
members = ["crates/orby-core"]

[build-dependencies]
tauri-build = { version = "2", features = [] }

[dependencies]
orby-core = { path = "crates/orby-core" }
tauri = { version = "2", features = ["tray-icon"] }
tauri-plugin-dialog = "2"
tauri-plugin-notification = "2"
serde = { version = "1", features = ["derive"] }
serde_json = "1"
```

- [x] **Paso 1 bis: los permisos** *(no estaba en el plan; sin esto no arranca nada)*

Tauri 2 deniega por defecto **hasta sus propios comandos**, los del núcleo incluidos. Sin
`src-tauri/capabilities/default.json` las ocho suscripciones de `device.init()` se rechazan
con `event.listen not allowed. Permissions associated with this command:
core:event:allow-listen, core:event:default`, y la app se queda sorda a todo lo que emita
el backend sin decir por qué (los rechazos son de promesa: no rompen el pintado).

```json
{
  "identifier": "default",
  "windows": ["main"],
  "permissions": ["core:default", "dialog:default", "notification:default"]
}
```

`"main"` es la etiqueta que Tauri le pone a la ventana cuando `tauri.conf.json` no le da una.

- [x] **Paso 2: la CSP de `index.html`**

Hoy la cabecera dice `default-src 'self'; script-src 'self'`. Tauri necesita poder hablar
con su IPC. Añadir `ipc:` y `http://ipc.localhost` a `connect-src`. **No relajar
`script-src`**: es lo que impide que un complemento o una respuesta de la red inyecte
código en la interfaz.

- [x] **Paso 3: ventana en blanco**

`tauri.conf.json` con: `frontendDist` a `../dist`, `devUrl` a `http://localhost:5173`,
ventana de 1280×820, mínimo 1024×700, `decorations: false` (el marco lo dibuja la app,
igual que hoy) y fondo `#0a0a0f` para que no pegue un fogonazo blanco al abrir.

**Y `"withGlobalTauri": true`**, que es de lo que depende `src/tauri/orby-tauri.js`
(Tarea 1) para tener `window.__TAURI__`. Sin esa línea la app abre en blanco y no dice
por qué: es el fallo más tonto y más caro de este plan.

`src/main.rs` mínimo: `tauri::Builder::default().run(...)`.

- [x] **Paso 4: comprobación** — hecha el 2026-08-11

En Windows:

```bash
cd OrbyGUI && npm run tauri:dev
```

Esperado: abre una ventana y se ve la interfaz de OrbyGUI (sin teclado detectado, porque
aún no hay puerto serie). En Linux **esto no compila** y es lo esperado: comprueba al
menos que `cargo test -p orby-core` sigue en verde.

Salió así: la interfaz se pinta entera y el aviso de "solo lectura" aparece. Lo que **no**
funciona todavía es cualquier cosa que se pulse, porque no hay comandos registrados.

Con `decorations: false` eso incluye **cerrar la ventana**: minimizar, maximizar y cerrar
son `invoke('window_*')`, y sin ellos la única salida es el administrador de tareas. Por eso
los tres se adelantan aquí desde la Tarea 5 (en `src/main.rs`). `window_close` de momento
cierra de verdad; pasa a esconder a la bandeja cuando la Tarea 5 monte la bandeja.

Cómo se leyó la consola del webview sin depender de que alguien mire la pantalla: WebView2
acepta el protocolo de depuración de Chrome, así que
`WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS=--remote-debugging-port=9222` antes de `tauri dev`
deja escuchar los errores y hasta capturar la ventana en PNG desde un script.

**Toolchain**: no hacen falta las Build Tools de MSVC. Con
`rustup ... --default-host x86_64-pc-windows-gnu` compila y enlaza (probado con 1.97.1).

- [x] **Paso 5: commit**

```bash
git add OrbyGUI/src-tauri OrbyGUI/package.json OrbyGUI/index.html
git commit -m "feat(tauri): crate de la app y ventana"
```

---

## Tarea 3: El puerto serie

La tarea más importante del plan: con ella el editor ya sirve para algo.

**Ficheros:**
- Crear: `OrbyGUI/src-tauri/crates/orby-core/src/serie.rs` (la máquina de estados, pura)
- Crear: `OrbyGUI/src-tauri/src/serial.rs` (el puerto de verdad y los comandos)

**Interfaces:**
- Comandos: `serial_send(cmd) -> bool`, `serial_get_info() -> Option<Value>`,
  `serial_get_status() -> String`, `serial_reconnect() -> bool`.
- Eventos: `serial:connected`, `serial:disconnected`, `serial:error`, `serial:searching`,
  `serial:data`.

> **La clave del diseño:** la correlación entre petición y respuesta **no se implementa**.
> Vive en `src/device.js` (cola de `pending`, `match`/`collect`/`fail`, 4 s de espera) y
> ahí se queda. Rust solo descubre el puerto, lo abre, saluda, trocea en líneas, vigila
> que siga vivo y **reenvía todo**. Si te encuentras escribiendo lógica de `GET_PROFILE`
> en Rust, te has salido del plan.

- [x] **Paso 1: la máquina de estados, en `orby-core` y con tests** — **YA ESTÁ HECHO**
      en `src-tauri/crates/orby-core/src/serie.rs`, con 16 pruebas. Léelo antes de seguir:
      la interfaz real es la de abajo y los plazos ya están puestos.

> **Un fallo que solo salió al escribir el Paso 3** (corregido el 2026-08-11): cuando el
> vigilante daba una conexión por muerta, emitía `Desconectado` y `Buscando` pero **no**
> `SoltarPuerto`. Como la cáscara solo cierra el puerto cuando la máquina se lo pide, y el
> rastreo no abre nada mientras haya uno cogido, la app se quedaba encallada enseñando
> «buscando» sin buscar. No se veía con los 16 tests porque ninguno miraba esa salida:
> ahora `si_no_contesta_al_vigilante_se_suelta_la_conexion` la comprueba.

> **Dos correcciones deliberadas respecto a `electron/serial.js`**, y hay que conservarlas:
> mientras se persigue el saludo por telemetría, **la cadena del saludo se aparta** (allí
> las dos mandaban `ACK` a la vez), y **un puerto del que ha llegado telemetría no se
> suelta nunca** (allí se soltaba a los 2,8 s, llevándose por delante la persecución que
> estaba a punto de conectar). Las dos tienen su prueba.

Va en `orby-core` **sin tocar el puerto**, para poder probarla sin hardware. Modela:

```rust
pub enum Estado { Desconectado, Buscando, Saludando, Conectado }

pub enum Salida {              // lo que la máquina pide que se haga
    Mandar(String),            // escribir esto en el puerto
    Emitir(Evento),            // avisar al renderer
    SoltarPuerto,              // cerrar, sin emitir «desconectado»
}

pub struct Maquina { /* … */ }

impl Maquina {
    pub fn al_abrir(&mut self) -> Vec<Salida>;
    pub fn al_recibir(&mut self, linea: &str) -> Vec<Salida>;
    pub fn al_pasar_tiempo(&mut self, ms: u64) -> Vec<Salida>;
}
```

El tiempo entra por parámetro (`al_pasar_tiempo`), **no** se lee del reloj: es lo que
permite probar el watchdog en microsegundos en vez de esperar 16 segundos reales.

Las pruebas que cubre (todas escritas ya):

- [x] Al abrir, manda `ACK` a los 400 ms; si no contesta, otro a los 1600 ms; si sigue sin
      contestar, `SoltarPuerto` y **no** emite `desconectado` (no llegó a estar conectado).
- [x] Con el saludo `ORBY_V4:FW=4.5:…:HOSTAPP=1`, emite `connected` con la info y manda
      `HOST_APP:1`.
- [x] Con el saludo **sin** `HOSTAPP=1`, emite `connected` y **no** manda `HOST_APP:1`.
- [x] Estando conectado, a los 12 000 ms de silencio manda `ACK`; si contesta, sigue
      conectado; y **vuelve a mandar `HOST_APP:1`** (el teclado lo caduca a los 25 s).
- [x] Estando conectado, 12 000 ms de silencio + 4 000 ms sin respuesta → emite
      `disconnected` y vuelve a `Buscando`.
- [x] Toda línea recibida estando conectado se emite como `serial:data`, **incluidas**
      `KEY_EV:3:1`, `ENC:1:-2` y `EV:CTX:0:1:3`.
- [x] Si llega `KEY_EV:` o `ENC:` sin haber saludado, reintenta el `ACK` cuatro veces cada
      600 ms y luego se da por conectado con una identidad mínima
      (`device: "ORBY_V4"`, `keys: 12`, `oleds: 10`).

- [x] **Paso 2: ejecutar los tests** — en verde:

```bash
cd OrbyGUI/src-tauri/crates/orby-core && cargo test serie
# Esperado: 16 passed
```

- [x] **Paso 3: el puerto de verdad, en `src-tauri/src/serial.rs`** — hecho el 2026-08-11

Solo fontanería alrededor de la máquina: un hilo que rastrea puertos cada 3 s con
`serialport::available_ports()`, filtra por VID `0xCafe` (**cualquier PID**), abre a
115200, fuerza DTR y RTS, y bombea bytes → `orby_core::lines::split_lines` → `Maquina`.
Las `Salida` se traducen a escrituras en el puerto y a `app.emit(...)`.

Cuidado con esto, que son fallos ya vividos:

- Quitar los manejadores de cierre **antes** de cerrar un puerto que falló, o se emite un
  `disconnected` falso.
- El tiempo máximo de apertura (6 s) también suelta el puerto.
- No hay «prueba con cualquier COM»: solo los que casan por VID. Probar a ciegas cuelga
  el hilo con puertos de otros cacharros.

Cómo quedó, para no tener que releer el fichero:

- **Un solo hilo posee el puerto.** Los comandos del renderer no escriben en él: encolan
  en `Compartido.salientes` y el hilo drena. Así no hay dos escritores sobre el mismo
  handle ni hace falta duplicarlo con `try_clone`.
- **El pulso del hilo es el plazo de la lectura** (20 ms): duerme dentro del `read` en vez
  de girar en vacío, y esos 20 ms son la latencia máxima de un comando (frente a los 4 s
  que espera `device.js`, no cuenta). Con el puerto cerrado el reposo sube a 200 ms.
- **Se rota entre los candidatos** en vez de probar siempre el primero: con dos puertos del
  mismo VID (una Pico de repuesto en modo carga), probar siempre el mismo dejaría el
  teclado bueno sin abrir para siempre.
- **El objeto de info se aplana** a la forma exacta de `_parseDeviceInfo`: claves en
  minúscula + `device`, `raw` y `port`. `compat.js` y las vistas la leen así.
- `serial_get_status` nunca devuelve `'disconnected'`: el hilo rastrea siempre mientras la
  app vive. Se conserva en el contrato porque las otras dos vías sí lo usan.
- En desarrollo salen por el terminal tanto los comandos del renderer (`> …`) como lo que
  manda la máquina por su cuenta (`>> ACK`, `>> HOST_APP:1`): sin lo segundo no hay forma
  de ver que una conexión en reposo se está renovando en vez de estar a punto de caerse.

- [x] **Paso 4: comprobación sobre el teclado (Windows)** — completada el 2026-08-11

- [x] Cierra la app de Electron antes (en Windows solo un proceso tiene el COM).
- [x] `npm run tauri:dev` → detecta el teclado y la interfaz se puebla con los perfiles.
      Traza: `probando COM7` → `>> ACK` → `teclado detectado: fw 4.6 en COM7` →
      `>> HOST_APP:1`, y detrás los 106 comandos de la sincronización completa
      (`GET_HASH`, `GET_STATE`, `GET_PROFILE:0/1`, todos los `GET_OLED`/`GET_OLED_PG`).
      Que lleguen los 106 ya demuestra que las respuestas vuelven: la cola de `pending`
      de `device.js` habría abortado en el primero que no contestara.
- [x] Desenchufa el teclado: la interfaz pasa a «buscando» en menos de 20 s.
      Salió mejor: `disconnected` y `searching` se emiten **en el mismo instante**, no a los
      20 s. La cuenta atrás del vigilante no llega a correr porque desaparecer el puerto se
      ve al leer, no al agotar el plazo.
- [x] Vuelve a enchufarlo: reconecta solo, sin tocar nada. Tardó 40 s con el cable fuera y
      volvió sin intervención.
- [x] Con la consola de la app abierta, gira el mando: se ven llegar líneas `ENC:`.
      Comprobado sobre la build de release: 24 líneas, `ENC:1:±1` y `ENC:2:±1`, los dos
      mandos y los dos sentidos. Con ellas llegan también `KEY_EV:` (pares pulsar/soltar),
      `MODE:MENU`/`MODE:NORMAL`, `EV:CTX:` y los `MACRO:<id>` de las teclas de PC.
- [x] Déjala conectada **un minuto sin tocar nada**: no se desconecta (el watchdog está
      renovando bien). Este es el que más fallos pilla.
      Salió así: a los 12 s de silencio `>> ACK`, el teclado contesta con su presentación
      y se renueva `>> HOST_APP:1`, **sin** un segundo `teclado detectado` (no se
      re-anuncia una conexión que ya estaba) y sin desconexión.

> **Un `MACRO:` que no salta no siempre es un fallo.** Al comprobar esto se dio por perdida
> una tecla que no disparaba su secuencia; lo que pasaba es que el teclado estaba en otro
> **perfil**, donde esa misma tecla es una tecla normal. Antes de buscar el fallo en la app,
> mirar el `EV:CTX:<perfil>:<pagina>:<total>` que hay justo antes en la traza.

- [x] **Paso 5: commit**

```bash
git commit -am "feat(tauri): descubrimiento, handshake y vigilancia del puerto serie"
```

---

## Tarea 4: Configuración local y copias de seguridad

**Ficheros:**
- Crear: `OrbyGUI/src-tauri/src/config.rs`, `OrbyGUI/src-tauri/src/backup.rs`

**Comandos:** `config_get`, `config_set(patch)`, `backup_save(data)`, `backup_load`.

La lógica ya está hecha (`orby_core::config`). Aquí solo falta el fichero y los diálogos.

> **Síntoma que provoca no tener esto** (visto el 2026-08-11 con la Tarea 3 recién hecha):
> el teclado guarda las acciones que no son teclas —abrir una app, un complemento, energía,
> una secuencia— como un **puntero a macro**, y lo que significa cada puntero vive en el
> PC. Sin `config_get`, la app abre, se conecta y pinta **todas** esas teclas como
> secuencias vacías: parece que se ha perdido la configuración. No se pierde nada —el
> autoguardado a Flash solo se dispara con ediciones explícitas, y aun así escribiría los
> mismos punteros—, pero asusta y es indistinguible de una pérdida de verdad.

- [x] **Paso 1: el fichero** — hecho el 2026-08-11

`%APPDATA%\OrbyGUI\orby-config.json`, el **mismo** que usa Electron.

**La ruta NO sale de `app.path().app_config_dir()`.** Ese resuelve a
`%APPDATA%\<identifier>`, y el identificador es `com.orby.gui` —reverse-DNS, como piden
los empaquetadores y como necesita el instalador de la Tarea 12—, así que dejaría la
configuración en `%APPDATA%\com.orby.gui` y el usuario abriría la app nueva sin ninguna de
sus secuencias. Se compone a mano desde `%APPDATA%`.

Dos diferencias deliberadas con `electron/config.js`, las dos a mejor:

- **Se escribe por fichero temporal y `rename`.** Electron escribe directo sobre el
  original, que es lo único que hay: si la app muere a mitad, un `orby-config.json`
  truncado se lleva por delante todas las secuencias del usuario.
- **Hay un cerrojo alrededor de leer-fusionar-escribir.** Los comandos de Tauri pueden
  correr a la vez en hilos distintos y dos guardados casi simultáneos se pisarían. En
  Electron no podía pasar porque todo iba en un hilo.

Y una diferencia a peor que no importa: Electron cachea la configuración en memoria y aquí
se relee del disco en cada consulta. Son 160 KB, y a cambio un cambio hecho por fuera se ve
sin reiniciar.

- `config_get` lee, y ante cualquier error de lectura o de parseo devuelve
  `orby_core::config::defaults()`. Un JSON corrupto **no** puede impedir que la app abra.
- `config_set` fusiona con `merge` y devuelve **la configuración ya fusionada** (el
  renderer se queda con lo devuelto).
- Escribir con sangría, que estos ficheros se acaban leyendo a mano.

- [x] **Paso 2: las copias** — hecho el 2026-08-11

`backup_save` abre un diálogo de guardar (`tauri-plugin-dialog`) y escribe el JSON.
`backup_load` abre uno de abrir y lo lee. Las dos formas de respuesta tienen que ser
**exactamente** `{ok:true,path}` / `{ok:true,data}` / `{ok:false,canceled:true}` /
`{ok:false,error}`, porque `src/backup.js` ya las distingue y es común a las tres vías.

Detalles que sí importan y que se comprobaron contra `electron/main.js`: `src/backup.js`
mira `canceled` **antes** que `ok`, así que cerrar el diálogo no puede devolver `error` o
le echaría la bronca al usuario por cerrar una ventana; el nombre propuesto es
`orby-backup-<AAAA-MM-DD-HH-MM>.json` en Documentos, con el filtro «Copia de seguridad
Orby»; y `backup_load` devuelve el JSON **ya interpretado**, no el texto.

El diálogo contesta por callback desde otro hilo, así que los dos comandos son `async` y
lo esperan por un canal: bloquear ahí colgaría la interfaz mientras el diálogo está
abierto.

- [x] **Paso 3: la prueba de oro de la paridad (Windows, con teclado)** — pasada el
      2026-08-11 sobre el teclado real (COM7, fw 4.5), con la build de **release** de Tauri

Esta prueba vale por diez:

- [x] Con la app de **Electron**, haz una copia de seguridad.
- [x] Ciérrala. Abre la de **Tauri** y restaura esa copia.
- [x] Comprueba que los perfiles, etiquetas, atajos y mandos quedan igual.
- [x] Haz ahora una copia con la de Tauri y compárala con la de Electron: deben coincidir
      salvo la marca de tiempo.

Las tres copias (una de Tauri, una de Electron y una de Tauri **después** de restaurar la
de Electron) miden **54 122 bytes exactos** y tienen el mismo contenido: `orby-backup` v4,
2 perfiles, 4 páginas y 20 iconos en el primero, iconos de perfil incluidos.

**Una diferencia real que hay que conocer, y que no es un fallo:** las dos copias no son
idénticas byte a byte. `serde_json` serializa los objetos con las **claves ordenadas
alfabéticamente**, así que donde Electron escribe `{"modifier":1,"keycode":6}` Tauri escribe
`{"keycode":6,"modifier":1}`. Mismos valores, mismo tamaño, orden distinto. Lo mismo le pasa
a `deviceMirror` y al objeto de info del teclado. Comparar copias entre las dos vías exige
normalizar el orden de las claves antes; un `diff` a secas marca las 80 entradas de
`keys`+`rotary` de cada perfil y no significa nada.

> **Trampa vivida al hacer esta prueba, y es de Electron.** Al cerrar la app de Tauri y
> abrir la de Electron acto seguido, el primer intento de apertura de Electron pilla el
> COM todavía ocupado, **se queda con el handle** y a partir de ahí registra
> `no se pudo abrir COM7: Access denied` cada 3 s **contra sí mismo**, para siempre. Con
> Electron corriendo, el puerto tampoco se abre desde PowerShell; al matarlo, se libera.
> Es exactamente el fallo que la constante «nunca dejar abierto un puerto cuyo handshake
> falló» manda evitar, y la máquina de estados de `serie.rs` sí lo hace. Al alternar entre
> las dos apps hay que esperar a que el puerto quede libre de verdad.

- [x] **Paso 4: commit**

```bash
git commit -am "feat(tauri): configuración local y copias de seguridad"
```

---

## Tarea 5: Ventana, bandeja, diálogos, apps instaladas y ratón

Un montón de piezas pequeñas, todas independientes.

**Comandos:** `window_minimize`, `window_maximize`, `window_close`,
`dialog_pick_app_or_file(kind)`, `apps_list_installed`, `mouse_get_position`.

- [x] **Ventana.** El marco lo dibuja la app (`decorations: false`), así que los tres
      botones llaman a la ventana de Tauri. **`close` no cierra: esconde a la bandeja**,
      igual que hoy. Cerrar de verdad solo desde el menú de la bandeja.
      El cierre se intercepta **además** en `on_window_event`: Alt+F4 no pasa por el
      comando, y sin eso se llevaría la app por delante con el teclado esperando que
      ejecute secuencias. La bandera `SALIENDO` es lo que distingue esconder de salir de
      verdad; sin ella, «Salir» de la bandeja tampoco podría cerrar.
      La primera vez que se esconde se avisa por notificación («OrbyGUI sigue activo»),
      una sola vez por sesión, igual que el globo de Electron.
- [x] **Bandeja.** Icono con menú «Abrir OrbyGUI» / separador / «Salir». Clic simple
      alterna visibilidad; doble clic muestra.
      El icono sale de `default_window_icon()`, **no** de `assets/orby-icon.png`: ese
      fichero es un JPEG con extensión `.png` y no todos los cargadores se lo tragan.
      Y `show_menu_on_left_click(false)`, o el clic izquierdo abriría el menú y no
      quedaría forma de esconder la ventana desde la bandeja.
- [x] **Diálogo de fichero.** Filtro `exe, lnk, bat, cmd` cuando `kind` no es `'file'`.
      El **orden** de los filtros importa y cambia según `kind`: el primero es el que sale
      preseleccionado, y pidiendo una aplicación con «todos los archivos» delante hay que
      rebuscar el `.exe` entre todo lo demás.
- [x] **Apps instaladas.** Recorre los dos menús de Inicio
      (`%ProgramData%\Microsoft\Windows\Start Menu\Programs` y el de `%APPDATA%`),
      recoge los `.lnk`, resuelve su destino con `IShellLink` (COM), descarta los que no
      apunten a un `.exe`, quita duplicados por destino en minúsculas, filtra los que
      casen con `uninstall|desinstal|read ?me|léeme|help|ayuda|website|sitio web|support|licen[cs]`
      y ordena con criterio español. Devuelve `[{name, target}]`.
      Tres detalles: el filtrado se hace con comparaciones literales en vez de traerse el
      crate `regex`; **no** se llama a `IShellLink::Resolve`, que con un destino que ya no
      existe se pone a buscarlo por disco y por red y convierte la lista en minutos; y el
      orden se le pregunta a Windows con `CompareStringEx` en `es-ES`, porque comparar
      bytes deja la «Ñ» y los acentos donde no toca.
- [x] **Posición del ratón.** Tiene que salir del **mismo espacio de coordenadas con DPI**
      que usará la reproducción de la Tarea 7. Si la captura y la reproducción no
      coinciden, las secuencias con posiciones absolutas pinchan en sitios distintos en
      pantallas con escalado. Es el fallo más difícil de diagnosticar de toda la
      migración.
      Resuelto usando `GetCursorPos`, que da píxeles físicos del escritorio virtual: el
      mismo espacio en el que trabaja `SendInput` con coordenadas absolutas. **La Tarea 7
      tiene que usar `SendInput` absoluto y no ninguna capa que hable en píxeles
      lógicos.**

- [x] **Comprobación (Windows):** minimizar, maximizar, cerrar a bandeja y recuperar desde
      la bandeja; la pestaña «App» de una tecla lista programas de verdad; el botón
      «Posición de ratón» del editor de secuencias devuelve la posición correcta **en un
      monitor con escalado al 150 %**.

  **Escalado al 150 %, comprobado el 2026-08-11** con el monitor principal a 144 DPI
  (0,0–1920,1080) y el secundario a 96 DPI (−1920,98–0,1178). `getMousePosition()` devuelve
  **exactamente** lo mismo que `GetCursorPos` en los cuatro puntos probados, incluidos los
  bordes y el monitor de coordenadas negativas: `1700,900`, `50,50`, `1919,1079`,
  `−1500,600` y `−1,1100`. Ni un píxel de diferencia: no hay virtualización de por medio.

  > **Al comprobar esto, la sonda tiene que declararse consciente del DPI.** Un PowerShell
  > normal **no** lo es, y Windows le virtualiza las coordenadas: en el monitor al 150 %
  > lee 1280×720 donde de verdad hay 1920×1080, y `EnumDisplayMonitors` le dice que todo
  > está al 100 %. Comparar eso contra la app —que sí es consciente— inventa una diferencia
  > que no existe. Hay que llamar a `SetProcessDpiAwarenessContext(-4)` antes de nada.

  Comprobado el 2026-08-11 sin tocar la pantalla: `listInstalledApps()` devuelve **188**
  programas con el destino resuelto (p. ej. `Altium Designer` →
  `C:\Program Files\Altium\AD24\X2.EXE`) y ordenados; `getMousePosition()` devuelve
  coordenadas del escritorio virtual, negativas incluidas, que es lo que hay que ver con
  un monitor a la izquierda; y `orby.close()` deja el proceso vivo con la ventana en
  `IsWindowVisible: False`.

  Comprobado a mano el 2026-08-11: la lista de programas instalados y el diálogo de elegir
  aplicación o archivo se comportan igual que en Electron.

  **Bandeja, comprobada a mano el 2026-08-11 sobre la build de release**: el ciclo entero
  (X → bandeja, clic simple para mostrar, clic simple para esconder, doble clic para
  mostrar, y «Abrir OrbyGUI» del menú) funciona, con la notificación de «sigue activo» la
  primera vez. Lo que importa medir de eso: el proceso **sigue vivo** y **el puerto serie
  sobrevive** al ciclo — al volver, `getStatus()` da `connected` en COM7 sin reconectar.
  La RAM pasa de 39,8 MB con la ventana abierta a 49,3 MB después de tres reconstrucciones
  del WebView2, y los hilos de 20 a 27; el WebView2 se reconstruye entero cada vez, así que
  los ganchos que hubiera puestos en la página se pierden (esperado, no es fuga de la app).

  Queda por comprobar a mano: el escalado al 150 %.

- [x] **Commit:** `git commit -am "feat(tauri): ventana, bandeja, diálogos y apps"`

---

## Tarea 6: Secuencias (lo que el PC ejecuta)

Cuando el teclado manda `MACRO:<id>` y la secuencia tiene pasos que solo sabe hacer el
PC, los ejecuta la app.

**Ficheros:** `OrbyGUI/src-tauri/src/macros.rs`

**Tipos de paso** que hay que cubrir (salen de `config.macros[].actions[]`):

| Tipo | Campos | Qué hace |
|---|---|---|
| `mouse_position` | `x, y` | Mover el ratón |
| `mouse_click` | `button, count, gap` | Clic |
| `delay` | `ms` | Esperar |
| `key` | `code, count, gap` | Pulsar una tecla |
| `hotkey` | `modifier, keycode, count, gap` | Combinación |
| `text` | `text, count, gap` | Escribir texto |
| `open_app` | `target` | Abrir programa o fichero |
| `system_power` | `mode` | Suspender, apagar, bloquear… |
| `plugin` | `plugin, op, value` | Delegar en el complemento (Tarea 10) |
| `center_mouse` | — | Legado: centro de la pantalla |

> **`mouse_move` no está en esta tabla y no es un olvido.** Aparece en configuraciones de
> verdad, pero `runAction` de Electron **tampoco lo ejecuta**: es uno de los cuatro tipos
> que hace el firmware por su cuenta (`DEVICE_STEP_TYPE` en `src/macros-store.js`), así que
> nunca llega por `MACRO:<id>`. Implementarlo aquí lo ejecutaría dos veces.

- [x] **Paso 1: la vía rápida del portapapeles** — hecho el 2026-08-11

Un texto de **5 caracteres o más** no se teclea letra a letra: se guarda el portapapeles,
se escribe el texto, se espera 30 ms, se manda `Ctrl+V`, se espera 120 ms y **se restaura
el portapapeles en un bloque que se ejecute pase lo que pase**. Por debajo de 5
caracteres se teclea con 4 ms entre letras. Los saltos de línea y los tabuladores van
como pulsaciones de verdad, no como caracteres.

> Perder la restauración del portapapeles es de los fallos que más molestan y que nadie
> asocia con el teclado.

Añadido que Electron no tenía: **un cerrojo alrededor del portapapeles**. Dos secuencias
pueden solaparse —el teclado no espera a que acabe una para mandar la siguiente, y en
Electron tampoco se esperaba—, pero el portapapeles es uno solo: sin el cerrojo la segunda
lo pisa mientras la primera no ha pegado todavía y el usuario acaba con el texto
equivocado **y** con el portapapeles de otro.

Y las letras se escriben por `KEYEVENTF_UNICODE` en vez de por tecla virtual: así salen
igual sea cual sea la distribución del teclado del usuario.

- [x] **Paso 2: acciones de energía** (`std::process::Command`, todas de Windows) — hecho:

| `mode` | Comando |
|---|---|
| `sleep` | `rundll32.exe powrprof.dll,SetSuspendState 0,1,0` |
| `hibernate` | `shutdown /h` |
| `restart` | `shutdown /r /t 0` |
| `shutdown` | `shutdown /s /t 0` |
| `lock` | `rundll32.exe user32.dll,LockWorkStation` |
| `logoff` | `shutdown /l` |

- [x] **Paso 3: retardos entre repeticiones.** Por defecto **20 ms** entre repeticiones.
      Y al escribir, poner el retardo automático de la librería de entrada **a cero**: los
      300/100 ms que traen por defecto añaden más de medio segundo por repetición.
      Con `SendInput` esto último desaparece solo: no hay retardo automático que quitar,
      los eventos salen cuando se mandan. Se conservan los 4 ms entre letras al teclear,
      que sí hacen falta: con 0 hay aplicaciones que se comen caracteres sueltos.

- [x] **Paso 4: tests** de la traducción de códigos HID a teclas (tabla de la página de
      uso `0x07`) y de los bits de modificador (`0x01..0x80` → Control/Shift/Alt/Super,
      izquierda y derecha). Van en `orby-core`, son puros.
      Hechos en `orby-core/src/teclas.rs`, 7 pruebas. La que más vale es la de las teclas
      **extendidas**: sin esa bandera, con Bloq Num puesto la flecha arriba escribe un 8, y
      el Intro del numérico y el normal son el mismo código virtual —solo los distingue
      esa bandera—.

- [x] **Comprobación (Windows):** una secuencia con texto largo (comprobar que el
      portapapeles vuelve a lo que había), otra con combinación, otra que abra un
      programa, y una con posición absoluta de ratón **en un monitor con escalado**.

  Comprobado a mano el 2026-08-11 sobre el teclado: secuencias, abrir programas y elegir
  ficheros se comportan igual que en Electron.

  **El caso del escalado, cerrado el 2026-08-11** con el monitor principal al 150 %: se
  grabó un recorrido que cruza los dos monitores y se reprodujo midiendo el cursor con una
  sonda consciente del DPI. La x mínima grabada (**−1121**, en el monitor de la izquierda)
  se reproduce **idéntica**, y el cursor arranca y acaba en `337,960`, que es exactamente la
  coordenada del clic que se grabó. Las únicas diferencias están donde el gancho de bajo
  nivel vio posiciones **fuera** del escritorio virtual (x=1970, y=−43): son las que Windows
  recorta al borde al reproducir, y no son un desajuste de escala.

- [x] **Commit:** `git commit -am "feat(tauri): ejecución de secuencias en el PC"`

---

## Tarea 7: Grabadora de ratón y teclado

**Ficheros:** `OrbyGUI/src-tauri/src/recorder.rs`

**Comandos:** `recorder_toggle(id)`, `recorder_stop`, `recorder_status`.
**Evento:** `recorder:state` con `{id, phase}`, donde `phase` es una **cadena**.

> **El hallazgo que cambia el diseño** (2026-08-11): lo que hay guardado en `c` **no es
> una tecla virtual de Windows**, es el código de `UiohookKey`. Electron capturaba con
> uiohook-napi y reproducía con nut-js, y en el disco quedó el número del primero. Los
> ganchos de bajo nivel dan teclas virtuales, así que hace falta un puente en los **dos**
> sentidos: al grabar, para seguir escribiendo el mismo formato; al reproducir, para
> entender las grabaciones que el usuario ya tiene. Sin él, una grabación vieja reproduce
> teclas equivocadas y no hay ningún error que lo delate.
>
> Los códigos de uiohook son los scancodes del juego 1 con tres codificaciones:
> `sc` a secas, `0xE00 | sc` para las extendidas, `0xE000 | sc` **solo para las flechas** y
> `0xEE00 | sc` para el bloque numérico con Bloq Num apagado. La incoherencia entre
> `0xE00` y `0xE000` es de libuiohook y se copia tal cual. La tabla está en
> `orby-core/src/uiohook.rs`, sacada del volcado de `require('uiohook-napi').UiohookKey`
> de esta misma instalación (124 entradas), y **no del enum de nut-js**: la traducción de
> nut-js a teclas de Windows vive dentro de un `.node` compilado que no se distribuye.
>
> El puente se hace contra `(vkCode, LLKHF_EXTENDED)`, que es la pareja que sí distingue
> el Inicio del 7 del numérico: comparten tecla virtual y solo se diferencian en esa
> bandera.

- [x] **Paso 1: la captura.** Ganchos de bajo nivel de Windows (`WH_KEYBOARD_LL` y
      `WH_MOUSE_LL`) en un **hilo propio con su bombeo de mensajes**: sin bombeo no llega
      ningún evento y el fallo es silencioso.
      Se instalan al empezar a grabar y se quitan al parar, no al abrir la app: un gancho
      de bajo nivel puesto siempre mete a este proceso en el camino de **cada** pulsación
      del equipo. El hilo se para mandándole un `WM_QUIT` con `PostThreadMessageW`, y por
      eso hay que guardar el identificador de hilo **de Windows** (`GetCurrentThreadId`),
      que no es el `ThreadId` de Rust.

- [x] **Paso 2: forma de los eventos grabados.** Conservada tal cual, con sus tests de ida
      y vuelta: `{k:'kdown'|'kup', c, t}`, `{k:'mdown'|'mup', b, x, y, t}`,
      `{k:'wheel', r, d, t}`, `{k:'move', x, y, t}`. Un evento que no se entienda se
      descarta **uno a uno**: si fallara la lista entera, una versión futura que añadiera
      un tipo dejaría ilegibles todas las grabaciones al abrirlas con una versión vieja.

- [x] **Paso 3: filtrado del movimiento.** 16 ms y 3 px, con las comparaciones estrictas
      del original y el umbral **por eje** (basta con que uno se mueva). Los clics no pasan
      por el filtro.

- [x] **Paso 4: recortar el silencio inicial.** Al parar, restando a todos los tiempos el
      del primer evento.

- [x] **Paso 5: reproducción.** Modos `once`, `loop` y `hold`; `speed` divide los tiempos y
      el objetivo de cada evento se mide contra el arranque de **esa pasada**, no sumando
      esperas. Antes de un clic se recoloca el ratón.
      Las teclas y botones hundidos los suelta un `Drop`, que es el equivalente del
      `finally` de Electron y cubre también el caso de que el hilo muera por pánico, que es
      justo cuando peor viene quedarse con el Control pulsado.
      La espera va troceada en 10 ms: sin eso, parar un bucle con un hueco de tres segundos
      tardaría hasta tres segundos en obedecer, y en modo `hold` son tres segundos de
      teclas cayendo después de soltar.

- [x] **Paso 6: apagado.** `recorder::apagar()` en `RunEvent::Exit`. Lo importante no es
      que el proceso termine (los hilos de Rust no lo retienen), es **soltar lo que
      estuviera hundido**: salir a mitad de una reproducción sin esto deja el Control
      pulsado, y ahí ya no queda ninguna app viva que pueda arreglarlo.

**Cuatro desviaciones deliberadas respecto a Electron**, todas a mejor y todas comentadas
en el código:

1. **No se reproducen los botones laterales del ratón.** Electron los pasaba por
   `BUTTON_TO_NUT[b] || 'LEFT'` y acababa haciendo un clic **izquierdo** donde el usuario
   había pulsado el de «atrás». Un clic izquierdo que nadie pidió cae sobre lo que haya
   debajo del puntero; no hacer nada, no.
2. **Se ignoran los eventos inyectados mientras hay una reproducción en marcha.** Allí,
   empezar a grabar con otra grabación en bucle se grababa a sí misma. Solo mientras
   reproduce: por escritorio remoto **toda** la entrada llega marcada como inyectada, y
   filtrar sin condición dejaría la grabadora muerta en esa sesión sin decir por qué.
3. **El giro de la rueda se redondea a una muesca como mínimo.** Los ratones de precisión
   mandan giros menores que `WHEEL_DELTA`; con una división a secas quedan en cero, y un
   cero se toma como una muesca **hacia abajo**, así que el gesto salía del revés.
4. **Un modo desconocido da una sola pasada.** Electron repetía con cualquier modo que no
   fuera `'once'`, así que un valor raro en la configuración era un bucle infinito.

- [x] **Comprobación (Windows)** — hecha el 2026-08-11, sobre la app y con el teclado
      conectado (COM7, fw 4.6), sin necesidad de mirar la pantalla: se lanzó `tauri:dev`
      con `--remote-debugging-port=9222` y se condujo `window.orby` por el protocolo de
      depuración, generando entrada real con `SendInput` desde PowerShell (movimientos de
      ratón y F15, que no hace nada en ninguna aplicación).

  - Las **seis fases** salen y con el `id` que toca: `recording`, `saved`, `empty`,
    `playing`, `idle` y `reset`.
  - Grabado un gesto de 20 movimientos y una pulsación: quedan **22 eventos**, los
    movimientos espaciados ~30 ms y ~8 px (el filtro deja pasar lo que debe), la tecla como
    `c: 93` —que es F15 en la tabla de uiohook, o sea que el puente de códigos funciona— y
    el primer evento en `t: 0`, que es el recorte del silencio inicial.
  - Reproducida en `once`: el ratón recorre el camino grabado (de 860 a 1025) y el estado
    vuelve a `playingId: null` solo.
  - **La prueba que importa:** una grabación en bucle que hunde F15 y no la suelta.
    Con la reproducción en marcha, `GetAsyncKeyState(0x7E)` da hundida; se corta a mitad
    con `recorder.stop()` y pasa a **suelta**. No queda nada pegado.
  - La tecla de borrado (`recording-reset`) avisa con el `id` de su `target`, no con el
    suyo, y deja la grabación en cero eventos.
  - La configuración se dejó como estaba: ninguna macro con eventos.

- [x] **El corte de un `hold` por la telemetría del teclado** — comprobado el 2026-08-11
      sobre la build de release, con una grabación temporal que hunde F15 y no la suelta,
      colgada de una tecla física del Orby:

  | t | Qué pasa |
  |---|---|
  | 120,10 s | `KEY_EV:11:1` y `MACRO:140` |
  | 120,12 s | `recorder {id:140, phase:"playing"}` — arranca en **20 ms** |
  | 126,65 s | `KEY_EV:11:0` (soltada tras 6,5 s) |
  | 126,66 s | `recorder {id:140, phase:"idle"}` — corta en **10 ms** |

  Repetido con una pulsación corta (2,3 s), mismo resultado. Ni F15 ni Control quedan
  hundidas después (`GetAsyncKeyState`).

- [x] **Una grabación con clics de verdad y rueda** — comprobado el 2026-08-11: 437 eventos
      (409 `move`, 3 `mdown`/3 `mup`, 10 `wheel`, 6 `kdown`/6 `kup`).
  - Primer evento en `t = 0`: el recorte del silencio inicial funciona.
  - Clics con su botón (`b=1` y `b=2`) y sus coordenadas, uno de ellos en **x = −284**:
    escritorio virtual, con el monitor de la izquierda incluido.
  - Rueda en los dos sentidos (`r=-1` y `r=1`) y **siempre** con `d=3`: nunca cae a cero,
    que es la desviación deliberada nº 3.
  - Huecos entre movimientos: mínimo y mediana **16 ms** clavados, que es el filtro.
  - Reproducida, el cursor recorre de **x=−642 a x=1213**, cruzando los dos monitores: la
    captura y la reproducción comparten espacio de coordenadas.

  > **La reproducción dura un tercio de la grabación y está bien.** Una grabación de 38,8 s
  > se reproduce en 13,6 s porque la velocidad por defecto es **3×**
  > (`VELOCIDAD_POR_DEFECTO` en `recorder.rs`, `macro.speed || 3` en `electron/main.js:314`).
  > Al medir tiempos de reproducción hay que dividir por ahí antes de gritar.

- [x] **Commit:** `git commit -am "feat(tauri): grabadora de ratón y teclado"`

---

## Tarea 8: Ventana en primer plano y cambio automático de perfil

**Ficheros:** `OrbyGUI/src-tauri/src/foreground.rs`

**Comandos:** `foreground_start`, `foreground_stop`, `foreground_current`,
`foreground_available`. **Eventos:** `foreground:change`, `foreground:error`.

Hoy esto es un **PowerShell permanente** al que se le pasa un script en base64 y que hace
P/Invoke contra `user32.dll`. En Rust son cuatro llamadas directas y desaparece el
proceso hijo: `GetForegroundWindow`, `GetWindowThreadProcessId`, `GetWindowText` y
`QueryFullProcessImageName`.

- [x] Sondeo cada **400 ms**, y emitir **solo cuando cambia** la ventana o el título.
- [x] Devolver `{process, title, path}`.
- [x] `foreground_available` da `true` en Windows.
- [x] Arrancar el vigilante al iniciar **solo si** `config.autoProfile.enabled`.

Cómo quedó, para no tener que releer el fichero:

- **`process` es el nombre del ejecutable sin extensión**, sacado del tallo de la ruta que
  da `QueryFullProcessImageNameW`. Es lo que daba `Get-Process.ProcessName` en Electron y
  con lo que casan las reglas que el usuario ya tiene guardadas («altium», no
  «altium.exe»). Ponerle la extensión las rompería todas de golpe y sin aviso.
- **El proceso se abre con `PROCESS_QUERY_LIMITED_INFORMATION`**, no con
  `PROCESS_QUERY_INFORMATION`: es el permiso que dejan pedir los procesos de más
  integridad, así que la ruta también sale para lo que corra como administrador. Con el
  otro, cualquier programa elevado aparecería sin nombre y sus reglas no encajarían nunca.
- **Se compara por ventana y por título**, como el script de PowerShell. Dentro de un
  navegador o de un editor el programa no cambia, cambia el título, y ahí encaja la mitad
  de las reglas.
- **Sin ventana con el foco no se emite nada.** Emitir un hueco al bloquear la pantalla
  haría saltar el perfil de reserva, y el usuario se encontraría otro perfil al volver.
- **La bandera de vida es un `Arc` por hilo, no un atómico global**: un `parar()` seguido
  de un `arrancar()` deja al hilo viejo con su propia bandera, ya apagada, y se muere solo
  sin que los dos se pisen. Y se vuelve a mirar justo antes de emitir: un cambio emitido
  por un vigilante ya apagado le movería el perfil al teclado sin que nadie lo pida.
- **El estado de comparación es del hilo**, así que parar y volver a arrancar re-emite la
  ventana que haya delante. Es de lo que depende el botón «App en foco» del editor, que
  arranca el vigilante y pregunta a los 300 ms.
- **`foreground:error` no se emite nunca**, y no es un olvido: allí los errores eran del
  proceso de PowerShell (no arrancaba, se moría, escupía por stderr) y ese proceso ya no
  existe. Lo que puede fallar ahora —una ventana sin nombre resoluble— se resuelve
  devolviendo campos vacíos, que es lo que hacía Electron también. El renderer sigue
  suscrito porque el contrato es el suyo.

- [x] **Comprobación (Windows, con teclado)** — hecha el 2026-08-11 sobre el teclado
      (COM7, fw 4.6), conduciendo `window.orby` por el protocolo de depuración del webview
      y moviendo el foco de verdad con `SetForegroundWindow` desde PowerShell.

  - Alternando entre VS Code y el Bloc de notas llegan **tres** `foreground:change`: el de
    VS Code, el del Bloc de notas y **un tercero con la misma ventana y otro título**
    («Bloc de notas» → «Sin título: Bloc de notas»), que es la prueba de que se compara
    también por título. Mientras nada cambia no llega nada.
  - `{process, title, path}` con la forma exacta: `process: "Notepad"`,
    `path: "C:\Program Files\WindowsApps\...\Notepad.exe"`.
  - **La prueba de oro:** con la regla que ya tenía el usuario (`altium` → perfil 0) y el
    teclado puesto a mano en el perfil 1, abrir un fichero llamado `altium.txt` en el Bloc
    de notas devuelve el teclado al **perfil 0** solo. La cadena entera —vigilante,
    `auto.js`, `SET_PROFILE`— funciona sin tocar el renderer.
  - Tras `foreground.stop()`: cero eventos aunque cambie el foco, el recuento de hilos del
    proceso baja de 15 a 14, **cero `powershell.exe`** en la máquina y `current()` sigue
    devolviendo la última ventana vista, igual que el `this.current` de Electron.
  - Arranque automático: reiniciada la app con `autoProfile.enabled` en `true`, `current()`
    ya trae la ventana **sin que nadie haya llamado a `start()`**.

- [x] **Commit:** `git commit -am "feat(tauri): ventana en primer plano sin PowerShell"`

---

## Tarea 9: Actualización del firmware

**Ficheros:** `OrbyGUI/src-tauri/src/firmware.rs`

**Comandos:** `firmware_get`, `firmware_check(opts)`, `firmware_update(opts)`,
`firmware_cancel`. **Evento:** `firmware:state`.

Estado: `{status, current, latest:{version,asset}, available, percent, manual, error}`,
donde `status` es `idle|checking|downloading|bootsel|flashing|done|error`.

- [x] **Buscar versiones.** YA HECHO en `orby_core::releases` (9 tests). Aquí solo falta la llamada HTTP. `GET https://api.github.com/repos/jaestefaniah27/scroll_wheel/releases?per_page=50`
      con cabecera `User-Agent: OrbyGUI`. Quedarse con las etiquetas que empiecen por
      `fw-v`, **conservando las prereleases** (las de firmware se publican así a
      propósito) y descartando los borradores. Filtrar por `compare_fw(version, maxFw) <= 0`
      y ordenar de mayor a menor.
- [x] **Descargar** el `.uf2` con barra de progreso, validando el tamaño. — hecho el
      2026-08-11 en `src-tauri/src/firmware.rs`.
- [x] **Entrar en modo carga.** Si no hay unidad de arranque, mandar `BOOTSEL` por el
      puerto y esperar a que aparezca la unidad (hasta 90 s, mirando cada 400 ms).
- [x] **Encontrar la unidad**: recorrer de `C:` a `Z:` buscando un `INFO_UF2.TXT` que
      contenga `RP2`. **Por etiqueta no**: la letra la asigna Windows y cambia.
- [x] **Copiar el fichero y tragarse los errores** `EIO`, `EPERM`, `ENOENT`, `EBUSY` e
      `EINVAL`. La Pico se reinicia a mitad de la copia y el volumen desaparece: **eso es
      el éxito**, no un fallo. Tratarlos como error es el fallo clásico aquí.
      En Rust no hay código de error de libc que mirar: `std::io::copy` los traduce a
      `ErrorKind`, así que se toleran `UnexpectedEof` (EIO), `PermissionDenied` (EPERM),
      `NotFound` (ENOENT), `WouldBlock` (EBUSY) e `InvalidInput` (EINVAL).

  Cómo quedó, para no tener que releer el fichero: el reqwest de descarga es el
  **bloqueante** (`reqwest::blocking`), no el asíncrono, para no meter un runtime de tokio
  en un comando que ya corre en su propio hilo — es el mismo estilo que `serial.rs` y
  `recorder.rs`. El backend TLS es `rustls` y no `native-tls`: es TLS puro en Rust, sin
  ningún componente en C que enlazar (`native-tls` sí lo necesitaría en otras
  plataformas), lo que evita depender del toolchain de C del sistema — aunque
  `aws-lc-sys`, el proveedor de criptografía por defecto de `rustls` en esta versión, sí
  se compila con `cc`/`cmake`; compiló limpio en esta máquina pero **si algún día falla en
  otra por falta de compilador de C, la salida es pasar a `rustls-no-provider` +
  `rustls::crypto::ring::default_provider()`**, que es criptografía pura en Rust. El envío
  del `BOOTSEL` reutiliza `serial::serial_send` llamándolo directamente (no por IPC): los
  comandos de Tauri son funciones normales y `firmware.rs` puede pedir el `State<Serie>` al
  `AppHandle` igual que hace el propio `main.rs`.

- [x] **Comprobación (Windows, con teclado):** actualizar por la vía `BOOTSEL`. Confirmar
      que al acabar el teclado vuelve con la versión nueva.

> **La vía manual (enchufar con BOOTSEL pulsado) se descarta a propósito**, decisión del
> 2026-08-11. La automática funciona y es la que usa cualquiera con un firmware 4.2 o
> posterior; la manual solo hace falta para recuperar un teclado que se quedó a medias, y
> para eso está `flash.ps1`. Esta tarea se da por cerrada sin ella.

  - [x] **Vía `BOOTSEL`, hecha de verdad el 2026-08-11**: el teclado subió de la 4.5 a la
        4.6 sobre la build de release, y volvió solo con `FW=4.6` y todas sus banderas
        (`MACROS=1:MAXMACROS=64:HASH=1:BOOTSEL=1:PICON=1:HOSTAPP=1`). La secuencia de
        `firmware:state` fue `downloading` 0→100 % (0,3–0,9 s) → `bootsel` (0,9 s) →
        `flashing` (1,8 s) → `done` (4,3 s) → recomprobación automática → `idle` (8,2 s).
        Nadie tocó el teclado: ni desenchufarlo, ni el botón.
  - [x] `firmware_check` con los dos topes: con `maxFw: '4.5'` devuelve `available: false`
        y `latest: 4.5`; con `maxFw: '4.6'`, `available: true` y el asset correcto
        (`ORBY_V4-fw-4.6.uf2`, 260 608 bytes, el mismo tamaño que publica GitHub).
  - Vía manual (enchufar con BOOTSEL pulsado): **descartada**, ver el aviso de arriba.

  > **Ojo con el tope de versión al probar esto en esta rama.** La rama de trabajo va por
  > detrás de `main` en el firmware: aquí `orby_version.h` y `compat.js` dicen **4.5** y
  > `main` dice **4.6**. Como el tope sale de `FW_RECOMMENDED`, la app en esta rama solo
  > ofrece la 4.5, así que a un teclado con la 4.6 le diría «al día» y a uno con la 4.5 no
  > le ofrecería nada. Para probar la subida de verdad se le pasó `maxFw: '4.6'` a
  > `firmware.check` a mano, sin tocar el código. Al traer `main` esto se arregla solo.

  Comprobado en seco antes, el 2026-08-11: `cargo check` compila limpio con la dependencia
  nueva, `cargo test -p orby-core` sigue en 127 pruebas verdes y el verificador del plan no
  encuentra discrepancias.

- [x] **Commit:** `git commit -am "feat(tauri): actualización del firmware"`

---

## Tarea 10: Complementos, tipo `http`

Los complementos se reescriben de cero. Un complemento de tipo `http` es **un
`plugin.json` y nada más**: no hay código de terceros ejecutándose.

**Ficheros:**
- Crear: `OrbyGUI/src-tauri/crates/orby-core/src/plugins/{mod,manifiesto,plantilla,http}.rs`
- Crear: `OrbyGUI/src-tauri/src/plugins.rs` (instalar, listar, ajustes)
- Reescribir: `OrbyGUI/plugins/lampdesk/plugin.json`; **borrar** `plugins/lampdesk/main.js`

**Comandos:** `plugins_list`, `plugins_install`, `plugins_uninstall(id)`,
`plugins_set_enabled(id, enabled)`, `plugins_get_settings(id)`,
`plugins_set_settings(id, patch)`, `plugins_test(id, values)`, `plugins_read(id, op)`,
`plugins_open_folder`.

> **El frontend no se toca.** Rust tiene que producir el **mismo descriptor** que hoy
> produce `electron/plugins.js`, porque `src/plugins.js` y la tarjeta de Ajustes ya lo
> consumen: `{id, name, version, description, author, icon, enabled, error, actions[],
> views[], hasRead, settings, values}`. Con `actions[]` de forma
> `{op, label, targets, step, hint, value:{min,max,step,default}}` y `views[]` de forma
> `{op, label, icon}`.

- [x] **Paso 1: el manifiesto y su validación (con tests)** — ya estaba hecho (ver «Lo que
      ya está hecho»): `crates/orby-core/src/plugins/manifiesto.rs`, 18 tests.

Campos: `id` (contra `^[a-z0-9][a-z0-9-]{1,38}$`), `name`, `apiVersion` (debe ser `2`),
`kind` (`"http"` o `"process"`), `version`, `description`, `author`, `icon`, `settings`,
`actions`, `views`, y para `http`: `request` global (`base`, `timeoutMs`) y un `request`
por acción (`path`, `method`, `query`).

Tests a escribir primero:
- [ ] Un `id` con mayúsculas o con espacios se rechaza.
- [ ] `apiVersion: 1` se rechaza **con un mensaje claro** (no se cuelga ni revienta).
- [ ] Un manifiesto sin `name` se rechaza.
- [ ] Un manifiesto válido produce el descriptor esperado.
- [ ] **El descriptor de lampdesk coincide campo a campo con el que produce hoy Electron.**
      Es la prueba de que el frontend no nota el cambio.

- [x] **Paso 2: las plantillas (con tests)** — ya estaba hecho:
      `crates/orby-core/src/plugins/plantilla.rs`, 12 tests.

Solo sustitución, sin expresiones ni lógica: `{{settings.x}}`, `{{value}}`,
`{{response.x}}`.

- [ ] `{{value}}` con `0` y con negativos (un giro hacia atrás es `-5`).
- [ ] `{{response.x}}` con el campo ausente.
- [ ] Una plantilla desconocida **falla**, no interpola vacío. Interpolar vacío manda una
      petición mal formada que el cacharro del otro lado interpreta como quiere.
- [ ] La URL y la cadena de consulta se montan bien, con codificación incluida.

- [x] **Paso 3: agrupar los giros (`coalesce`), con tests** — ya estaba hecho:
      `crates/orby-core/src/plugins/coalesce.rs`, 8 tests. Lo que faltaba de este paso era
      engancharlo a algo que avance el reloj de verdad: eso se hizo en el paso 4 con el
      «hilo del agrupador» de `src-tauri/src/plugins.rs`.

`coalesce: {ms, mode:"sum"}`. El firmware manda un `MACRO:<id>` **por cada muesca**, así
que un giro rápido son veinte disparos; agruparlos no es un lujo.

- [ ] Dos muescas de `+5` producen **una** petición con `10`.
- [ ] Si llegan más muescas mientras una petición está en vuelo, se encadena otra ronda.
- [ ] Una acción **sin** `coalesce` manda una petición por muesca (encender y apagar no se
      pueden fundir: dos pulsaciones son dos órdenes).

- [x] **Paso 4: peticiones, contra un servidor de mentira levantado en el propio test** —
      hecho el 2026-08-11 en `src-tauri/src/plugins.rs` (función `ejecutar`, tests en el
      propio fichero contra un `TcpListener` de mentira: no hacía falta traerse un crate
      de servidor HTTP para cuatro respuestas fijas).

- [x] `timeoutMs` corta de verdad. Sin él, un cacharro desenchufado deja la ejecución
      colgada más de un minuto esperando a que agote el TCP.
      Probado con un servidor que acepta la conexión y no contesta nunca: con
      `timeout_ms: 500` en la `Peticion`, `ejecutar` corta en menos de 2 s de verdad
      (`el_tiempo_maximo_corta_de_verdad`).
- [x] Un `500` se reporta como error, no se propaga como pánico.
- [x] `test` con `retries: 1` reintenta **una vez y solo una**
      (`con_un_reintento_hace_como_mucho_dos_intentos`: dos peticiones, ni una ni tres).
- [x] `run` **no reintenta nunca**: un incremento repetido se aplicaría dos veces
      (`sin_reintentos_declarados_un_solo_intento`).

  Cómo quedó, para no tener que releer el fichero: `ejecutar()` usa `reqwest::blocking`
  (el mismo backend que ya trajo la Tarea 9 para el firmware, con `rustls`) y no hay
  hilo dedicado al puerto como en `serial.rs` — cada llamada abre su propio cliente con
  el `timeout_ms` de la `Peticion` y se ejecuta en el hilo que la pidió (el de la tecla
  pulsada, o uno del hilo del agrupador). El cuerpo se interpreta como JSON si se puede
  y como `Value::Null` si no: no todos los cacharros contestan JSON a una escritura, y
  no tener nada que leer no es un error.

  El agrupado de giros (paso 3) se engancha aquí: `plugins::iniciar` lanza un hilo que
  cada 15 ms llama a `Agrupador::avanzar` y manda lo que venza, con la misma idea que el
  «pulso» de `serial.rs`. `disparar_paso`, que es donde entra el paso `{type:"plugin"}`
  de una secuencia (`macros.rs`), empuja al agrupador y solo dispara la petición al
  momento si la acción no declara `coalesce`.

- [x] **Paso 5: poner lampdesk en su sitio** — hecho el 2026-08-11.

  Copiado a `plugins/lampdesk/plugin.json` y borrado `plugins/lampdesk/main.js`. El test
  de descriptor (`descriptor_lampdesk.rs`) ya no lee de `tests/fixtures/`: lee del
  manifiesto de verdad con `include_str!("../../../../plugins/lampdesk/plugin.json")`,
  así que no puede quedarse verde comparando un fichero que ya no es el que instala la
  app. La copia en `tests/fixtures/lampdesk-api2.plugin.json` se queda donde estaba,
  porque los tests de `peticion.rs` siguen leyendo de ahí.

  **Aviso deliberado:** el lampdesk de la API 1 **deja de funcionar en la app de
  Electron** a partir de este commit, porque su `main.js` ha desaparecido y su
  `plugin.json` ahora declara `apiVersion: 2`, que el cargador de complementos de
  Electron rechaza. Es una excepción consciente a la constante «Electron sigue
  funcionando hasta la Tarea 13»: aplica a la app en general, no a este complemento de
  ejemplo en concreto, y el propio texto de este paso ya avisaba de que «un mismo
  fichero no puede declarar las dos versiones». Si hace falta seguir usando el lampdesk
  desde la app de Electron mientras dura la migración, hay que recuperar la versión
  anterior de `plugins/lampdesk/` de git.

Ya está resuelto en él: `timeoutMs: 4000`, `coalesce.ms: 120` en los dos `_delta`,
`retries: 1` en `test`, y las equivalencias `toggle`→`on=toggle`, `on`→`on=1`,
`off`→`on=0`, `brightness_delta`→`dbrightness={{value}}`, `color_delta`→`dcolor={{value}}`,
`brightness_set`→`brightness={{value}}`, `color_set`→`color={{value}}`, con los visores
leyendo `/state`. Su lista de `HOSTS_MUERTOS` **no se portó**: era un apaño de la época
de desarrollo.

- [x] **Paso 6: instalación** — hecho el 2026-08-11 en `src-tauri/src/plugins.rs`.

Descomprimido con el crate `zip` (**no** con `Expand-Archive` de PowerShell), con
`features = ["deflate"]` a propósito y sin los demás métodos de compresión del crate
(bzip2, lzma, zstd): esos enlazan una librería en C, y el toolchain de este proyecto es
GNU sin las Build Tools de MSVC. Un `.zip` puede traer el `plugin.json` en la raíz o
dentro de una única carpeta, igual que admitía Electron. Se protege contra una entrada
`../../algo` dentro del zip (`ruta_seria`), que Electron no comprobaba porque
`Expand-Archive` ya lo hacía por dentro.

Sin el diálogo de «¿zip o carpeta?» de Electron: un complemento `http` no ejecuta nada,
así que no hay ningún permiso que pedir ni ninguna decisión de seguridad que el usuario
tenga que tomar a propósito. `plugins_install` abre directamente el selector de fichero
filtrado a `.zip`/`.orbyplugin`.

Los ajustes siguen viviendo en `config.plugins["<id>"]`, con la misma forma, para que
sobrevivan a desinstalar y volver a instalar.

- [x] **Comprobación (Windows, con la lámpara):** instalar el `.zip`, ajustar la
      dirección, «Probar», asignar acciones a una tecla y a un mando, y **girar el mando
      rápido**: el brillo tiene que subir liso, sin tirones ni retraso acumulado.

  Comprobada por el usuario el 2026-08-11 contra la lámpara real: el complemento
  declarativo funciona. Antes ya estaban en verde el `cargo check`, los 7 tests de
  `plugins.rs` (ejecución HTTP contra un servidor de mentira) y los 127 + 5 de `orby-core`.

- [x] **Commit:** `git commit -am "feat(tauri): complementos declarativos"`

---

## Tarea 11: Complementos, tipo `process` (esqueleto)

> **Aplazada a propósito (decisión del 2026-08-11).** No hay ningún complemento de
> proceso que escribir todavía, así que construir el esqueleto ahora es trabajo sin
> nadie que lo use. Se retoma cuando toque desarrollar de verdad el primero de la lista
> del TODO (OBS, monitor de hardware, audio, Chrome o Altium): en ese momento el
> protocolo se diseña **contra las necesidades reales de ese complemento concreto**, en
> vez de a ciegas. Hasta entonces, **saltar directamente a la Tarea 12**.

Aquí no se busca paridad con Electron —esto no existe hoy— sino **dejar la puerta
abierta** a los complementos del TODO (OBS, monitor de hardware, audio, Chrome, Altium)
para que se puedan escribir después **sin volver a tocar OrbyGUI**.

**Ficheros:** `OrbyGUI/src-tauri/src/plugins/proceso.rs`, y el protocolo documentado en
`docs/PLUGINS.md`.

- [ ] **Paso 1: lanzar y hablar.** OrbyGUI lanza el ejecutable declarado en
      `process.exe` y habla por su entrada/salida con **JSON por líneas**.

*App → plugin:* `hello` (versión de API y capacidades del teclado), `settings`,
`run {op, value}`, `read {op}`, `test`, `shutdown`.

*Plugin → app:* `ready {actions, views}`, `event {op, value}`, y las que empujan:
`setPage`, `setProfile`, `setOled {slot, frame}`, `setLabel`, `notify`, `log`.

- [ ] **Paso 2: acciones en caliente.** Cuando llega `ready`, se **refresca** el
      descriptor y se avisa al frontend. Hace falta porque las escenas de OBS no se
      conocen hasta conectar: no pueden salir del manifiesto.

- [ ] **Paso 3: pintar en las OLED.** Reutiliza la vía que **ya existe** en
      `src/live-oled.js`: `device.uploadOled()` → `OLED_CHUNK`, que escribe **en la RAM
      del teclado**. Solo `SAVE_STATE` toca la Flash, así que un dashboard puede repintar
      cuanto quiera **sin desgastarla**. Y sus dos reglas ya probadas: solo se pinta en
      teclas de la **página que el teclado tiene puesta ahora mismo**, y **nunca** se
      fuerza un cambio de página para pintar de fondo.

- [ ] **Paso 4: el presupuesto de refresco.** Esto es lo único de esta tarea que no se
      puede dejar para después. El CDC son ~11,5 KB/s **compartidos con las pulsaciones**,
      y una pantalla son 360 bytes que viajan en 4 `OLED_CHUNK`: unos 800 bytes por frame,
      o sea ~14 pantallas por segundo **si no circulara nada más**. Por tanto:
  - Frecuencia máxima por complemento (empezar por 4 frames/s y medir).
  - **Coalescencia por hueco**: de un mismo slot solo importa el último frame; los
    intermedios se tiran sin mandarlos.
  - Las dos cosas van **en la app, no en el plugin**: un complemento de un tercero no
    puede tener la potestad de dejar el teclado pastoso.

- [ ] **Paso 5: supervisión.** Un plugin que se muere se marca con su `error` en la lista
      y se reintenta con **espera creciente**, nunca en bucle cerrado. `shutdown` al cerrar
      la app y al desactivarlo, con matarile por tiempo si no se va solo.

- [ ] **Paso 6: tests con un proceso de mentira** escrito en el propio test:
  - [ ] `ready` refresca el descriptor.
  - [ ] `run` llega al plugin.
  - [ ] `setPage` y `setOled` salen por donde deben.
  - [ ] El presupuesto descarta los frames intermedios del mismo hueco y conserva el
        último.
  - [ ] Un plugin que muere se marca y se reintenta con espera creciente.

- [ ] **Paso 7: el ejemplo.** Un complemento mínimo que empuje una OLED, como prueba de
      que la plataforma queda abierta y como referencia para escribir los del TODO.

> **Fuera de alcance de esta tarea**, pero **sí documentado** en `docs/PLUGINS.md`: el
> modo `socket` (para lo que vive dentro de otro programa: la DLL de Altium, el Native
> Messaging Host de Chrome) y el registro de ese host en Chrome.

- [ ] **Comprobación (Windows, con teclado):** el complemento de ejemplo pintando una OLED
      **mientras tecleas y giras el mando**: las pulsaciones no se pueden atascar. Es la
      prueba del presupuesto, y solo se ve sobre el enlace real.

- [ ] **Commit:** `git commit -am "feat(tauri): esqueleto de complementos de proceso"`

---

## Tarea 12: Actualizador, autoarranque y empaquetado

**El punto más delicado de todo el plan.** Aquí es donde se puede dejar tirada a la gente
que ya tiene la app instalada.

> **Es lo único que le queda a la migración.** Comprobado el 2026-08-11: todas las demás
> tareas están cerradas sobre el teclado real, salvo la 11 (aplazada a propósito) y la 13
> (que va después). Esta es la que bloquea que Tauri pase a ser el modelo.

> ### Hecha el 2026-08-12
>
> Los cinco comandos que faltaban están implementados y registrados: Rust registra ya los
> **39 de 39**. Lo que se decidió y por qué está en los apartados de cada paso, más abajo.
> Lo que **no** se hizo, a propósito: el actualizador que descarga e instala solo (ver
> Paso 4).
>
> El instalador sigue costando lo mismo: **7 843 560 bytes (7,48 MB)**, 25 KB más que el del
> 2026-08-11, que es lo que pesa `tauri-plugin-single-instance`. Frente a los 97,6 MB de
> Electron, la comparación de la Tarea 13 no se mueve.

### Punto de partida, medido el 2026-08-11

**Lo que ya está**: el empaquetado NSIS **funciona**. `npm run tauri:build` produce
`target/release/bundle/nsis/OrbyGUI_0.5.1_x64-setup.exe`, **7,5 MB** frente a los 97,6 MB
del de Electron, con `identifier: com.orby.gui` y el nombre de producto `OrbyGUI`. El paso 3
está, en la práctica, hecho: falta comprobar que instala **encima** de la versión existente
en vez de dejar dos apps.

**Lo que falta, y falla en caliente**: de los 39 comandos del contrato, Rust registra **34**.
Los cinco que no existen son justo los de esta tarea:

| Comando | Lo llama | Qué se ve hoy |
|---|---|---|
| `autostart_get` | `src/views/settings.js:244` | `Command autostart_get not found` |
| `autostart_set` | `src/views/settings.js:247` | idem |
| `updater_get` | `src/updater.js` (desde `init`) | `Command updater_get not found` |
| `updater_check` | `src/views/settings.js:119` | idem |
| `updater_install` | `src/views/settings.js:124` | idem |

**Y no fallan de forma visible, que es lo peor.** Los dos sitios donde se usan están
**declarados en `src/tauri/orby-tauri.js`**, así que `platform.can()` da `true` y
`ocultarSiFalta` **no** esconde las tarjetas: el usuario ve el interruptor de «Arrancar con
Windows» y la tarjeta de actualización de la app como si funcionaran.

- El interruptor de autoarranque queda **muerto**: `initAutostart()` revienta en el
  `await window.orby.autostart.get()` de su primera línea, que está **antes** del
  `addEventListener`, así que el botón nunca llega a tener manejador. Se pulsa y no pasa
  nada, sin ningún mensaje.
- La insignia y la tarjeta de actualización se quedan en blanco: `updater.init()` se llama
  igual (`src/main.js:101` solo se protege del caso navegador) y su promesa se rechaza.
- Los rechazos son de promesa y nadie los espera, así que **no rompen el pintado** del resto
  de Ajustes. Por eso esto ha llegado hasta aquí sin verse.

Al implementarlo, la decisión que hay que tomar de paso: si `updater` se va a quedar sin
implementar durante un tiempo, **quitar los dos espacios de nombres de `orby-tauri.js` y
meterlos en un `TAURI_PENDIENTE`**, para que las tarjetas se escondan en vez de mentir.
(El plan daba por hecho que la Tarea 1 había dejado ese `Set` previsto en `platform.js`;
no está. Al implementar los cinco comandos dejó de hacer falta, pero si algún día se
aparca una función de Tauri, hay que crearlo: `can()` solo pregunta por `isWeb()`.)

- [x] **Paso 1: autoarranque.** `src-tauri/src/autostart.rs`, sobre la clave
      `HKCU\Software\Microsoft\Windows\CurrentVersion\Run` con el crate `windows` que ya
      estaba en el árbol. Sin plugin: la trampa de este paso está en cómo se **lee**, y
      escribiendo la entrada uno mismo se controla la comparación.

      Lo que decide si está activado es la **ruta**, no los argumentos, al revés de lo que
      pedía este plan. El motivo: una entrada sin `--hidden` arranca la app igual, así que
      contestar «desactivado» es mentir tanto como contestar «activado» sin entrada. Esas
      entradas a medias se **reparan** al arrancar (`normalizar()`), que es lo que hacía
      también electron/main.js. Y la ruta se compara resuelta con `canonicalize`, porque
      el mismo fichero escrito de dos formas (mayúsculas, enlaces, nombres 8.3) compararía
      distinto como texto crudo y ahí sí saldría siempre «desactivado».

- [x] **Paso 2: arranque escondido.** `main.rs` mira `--hidden` en los argumentos.

      La ventana pasa a declararse `"visible": false` en `tauri.conf.json` y enseñarla es
      un paso explícito del `setup`: dejarla visible y esconderla después la deja asomar un
      instante al iniciar sesión. Con `--hidden` no solo no se enseña, se **destruye**
      (`window::arrancar_en_bandeja`), por lo mismo que el cierre a la bandeja: si no, el
      WebView2 se queda cargado toda la sesión sin que nadie lo mire.

      De paso, `tauri-plugin-single-instance`. No estaba en el plan y lo trae este paso:
      con el autoarranque puesto la app ya está corriendo escondida, y un doble clic en el
      acceso directo abriría un segundo proceso que pelearía por el puerto COM con el
      primero (en Windows solo lo puede tener abierto uno). Ahora ese segundo arranque solo
      saca la ventana de la instancia que ya vive. Electron tenía lo mismo
      (`requestSingleInstanceLock`) y por el mismo motivo.

> **`tauri build` falla si la app está abierta.** Con un `orby-app.exe` corriendo —la build
> anterior, lanzada a mano o dejada en la bandeja—, Cargo no puede sobrescribir
> `target/release/orby-app.exe` y el error no dice cuál es el problema:
> ```
> error: failed to run custom build command for `orby-app v0.5.1`
>   El proceso no tiene acceso al archivo porque está siendo utilizado por otro proceso. (os error 32)
> ```
> Ciérrala antes (menú de la bandeja → Salir, o `taskkill /PID <pid> /F`). Y ojo al
> encadenar la salida por una tubería (`| tee`, `| grep`): el código de salida pasa a ser el
> del último eslabón, así que un build fallido se lee como exitoso.

- [x] **Paso 3: empaquetado NSIS** con Tauri, conservando el nombre de producto
      `OrbyGUI` y el mismo identificador, para que actualice **sobre** la instalación
      existente en vez de dejar dos apps.
      **Hecho, y el resto de este paso se descarta.** El instalador sale de
      `npm run tauri:build` (7,5 MB) y los nombres son los correctos. Comprobar que instala
      **encima** de la de Electron ya no aplica: ver el aviso de abajo.

- [x] **Paso 4: el actualizador. Avisa, no instala** (decisión del 2026-08-12).

  > **Revertido el 2026-08-12, el mismo día**, a petición del autor: la app se actualiza
  > sola y en silencio. Lo de abajo se deja como está porque explica lo que costaba y por
  > qué se aplazó —que sigue siendo verdad y es justo lo que hubo que pagar—. Lo que se
  > hizo en su lugar está al final de este documento, en «Actualizaciones automáticas».

  `src-tauri/src/updater.rs`. Los tres comandos existen y la tarjeta dice la verdad, pero
  **no** sobre `tauri-plugin-updater`: se consulta `releases/latest` y, si hay versión más
  nueva, el estado pasa a `available` y el botón abre la página de la release en el
  navegador. La instalación sigue siendo a mano.

  Por qué no el actualizador de verdad: cuesta un par de claves minisign, la privada
  disponible al compilar y un `latest.json` firmado publicado **en cada release** además
  del `setup.exe`. Saltárselo una vez deja el actualizador mudo sin que nadie se entere.
  Para un único usuario que ya instala a mano no sale a cuenta. Si algún día hay a quién
  actualizar, se cambia el interior de `updater.rs` y el frontend no se entera más que en
  el texto del botón.

  El apaño de las **releases de firmware como *prerelease*** sigue en pie y ahora tiene red:
  si `releases/latest` devuelve una etiqueta `fw-v*` —es decir, si alguien publicó una de
  firmware como definitiva—, el avisador no ofrece el `.uf2` como si fuera la app: da error
  diciendo exactamente eso. Ya pasó una vez, está documentado en
  `ORBY_V4/docs/COMPATIBILIDAD.md` (ojo: ese está un nivel por encima, en `ORBY_V4/docs/`,
  no en `OrbyGUI/docs/` como los demás que cita este plan).

  En el frontend esto es un estado nuevo, `available`, junto a los `downloading` /
  `downloaded` de Electron. Lo pintan `src/updater.js` (texto), `src/main.js` (la insignia
  de la barra) y `src/views/settings.js` (la tarjeta, que además reescribe el botón y su
  explicación cuando corre sobre Tauri). Al revés que el de Electron, **funciona también en
  desarrollo**: aquel necesitaba un instalador que sustituir, y este solo abre una página.

> ### El relevo de Electron se descarta (decisión del 2026-08-11)
>
> Este plan se escribió suponiendo un parque de usuarios instalados con
> `electron-updater` al que no se podía dejar tirado, y de ahí salían el puente («una
> última versión de Electron cuya actualización instale la de Tauri») y la comprobación de
> instalar una encima de la otra.
>
> **No hay tal parque: el único usuario es el autor**, que ya tiene la de Tauri instalada
> a mano y la de Electron desinstalada. Así que:
>
> - **No se publica ninguna versión puente de Electron.**
> - **No se comprueba** que las dos convivan ni que una sustituya a la otra: con una a la
>   vez basta.
> - La instalación se hace **a mano**, con el `.exe` que produce `npm run tauri:build`.
>
> Lo que sí sigue en pie de este paso es el actualizador en sí, si se quiere que la app
> siga avisando de versiones nuevas. Si se decide que tampoco hace falta, la salida
> limpia **no** es dejar los comandos sin registrar —que es lo que hay hoy y miente en la
> interfaz— sino quitar el espacio `updater` de `src/tauri/orby-tauri.js` y meterlo en el
> `TAURI_PENDIENTE` de `platform.js`, para que la tarjeta se esconda.

- [ ] **Comprobación (Windows):** activar el autoarranque, cerrar sesión, volver a entrar y
      comprobar que la app arranca **escondida en la bandeja**, con el teclado detectado y
      sin ventana. Y que el interruptor de Ajustes refleja el estado real del registro
      después de reiniciar (es donde se ve si se comparó la ruta sin los argumentos).

      **Pendiente: es lo único de esta tarea que no se puede comprobar sin cerrar sesión.**
      Lo que sí está comprobado: `cargo test` y `node --test` en verde,
      `verifica_plan_tauri.sh` con 0 discrepancias y los 39 comandos registrados, y el
      instalador NSIS saliendo de `npm run tauri:build`. El aviso de versión nueva se puede
      ver sin cerrar sesión: la app instalada es la 0.5.1 y en el repositorio está publicada
      la release `v0.5.2`, así que la tarjeta de Ajustes tiene que decir «Hay una versión
      nueva: 0.5.2» y el botón abrir su página.

      Para ver el registro sin abrir regedit:
      ```powershell
      Get-ItemProperty 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Run' -Name OrbyGUI
      # Esperado con el interruptor encendido: "...\OrbyGUI.exe" --hidden
      ```

- [ ] **Commit:** `git commit -am "feat(tauri): actualizador, autoarranque y empaquetado"`

---

## Tarea 13: Retirar Electron y documentar

Solo cuando **todo lo anterior** esté comprobado sobre el teclado.

- [x] **Paso 1: medir**, que es lo que justifica la migración. Con las dos builds contra el
      mismo teclado, anotar: tamaño del instalador y RAM en reposo. Poner los números en el
      mensaje del commit.

  Medido el 2026-08-11 con las dos builds conectadas al mismo teclado (COM7). **El tiempo
  de arranque se descarta a propósito**: con un instalador trece veces menor y una décima
  parte de RAM, no hay ninguna decisión que dependa de esa tercera cifra.

  | | Electron 0.5.2 | Tauri 0.5.1 |
  |---|---|---|
  | Instalador NSIS | **97,6 MB** | **7,5 MB** |
  | Procesos | 4 | 1 |
  | RAM con la ventana abierta | **616,6 MB** (310,4 + 133,3 + 121,0 + 51,9) | **39,8 MB** |
  | RAM escondida en la bandeja | no medida | ~8 MB (ver commit `513e47d`) |

- [x] **Paso 2: quitar** `electron/`, las dependencias de Electron de `package.json`
      (`electron`, `electron-builder`, `electron-updater`, `serialport`, `uiohook-napi`,
      `@nut-tree-fork/nut-js`) y sus scripts.

      **Hecho el 2026-08-12.** Fuera la carpeta entera (11 ficheros, ~2 900 líneas), el
      `"main"`, los scripts `dev`/`start`/`dist`/`release`/`postinstall`, el bloque
      `build` de electron-builder (ya duplicado en `tauri.conf.json`) y las cuatro
      dependencias de ejecución: `dependencies` se queda **vacío** y `devDependencies`
      en dos entradas. `concurrently` se va con `dev`, que era quien lo usaba.
      `TAURI_PENDIENTE` **no existía**: nunca llegó a crearse porque los cinco comandos
      que lo iban a necesitar se implementaron en la Tarea 12. `PC_ONLY` se queda, que es
      de la vía navegador.

      **Antes de borrar hubo que reanclar dos cosas que leían `electron/` como fuente de
      verdad**, o el commit habría dejado la suite en rojo:

      - `test/superficie-orby.test.mjs` comparaba Tauri y navegador **contra
        `electron/preload.js`**. La referencia pasa a ser `src/tauri/orby-tauri.js`, y con
        ella se van los dos tests que solo tenían sentido con tres vías (14 → 12 pruebas).
      - `tools/test/verifica_plan_tauri.sh` tenía ~90 comprobaciones apuntando a
        `electron/`: las constantes de serie, secuencias, grabadora, firmware, primer
        plano y apps, los 39 canales, la superficie del preload y el descriptor de
        complementos. Todas reancladas a `src-tauri/src/*.rs` y a
        `src-tauri/crates/orby-core/`. **Esto contradice lo que decía el repaso final de
        este mismo plan** («el de Electron se deja: sigue siendo la referencia»): deja de
        ser cierto aquí, porque la referencia ya no existe.

      De propina, dos cosas que el plan no había mirado y que también dependían de
      Electron en **código vivo**, no en comentarios:

      - **`OrbyGUI.bat`** lanzaba `npx electron .`. Reescrito para arrancar `tauri:dev`.
      - **`tools/orby-manager/`**, el panel de control: dos de sus cinco botones de lanzar
        (`npm run dev`, `npm start`), el pipeline entero de release de la app
        (electron-builder + `latest.yml`) y el bump de versión, que trataba «vía Electron»
        y «vía Tauri» como dos números independientes. Ahora la app lleva **un** número en
        tres ficheros (`tauri.conf.json`, `Cargo.toml`, `package.json`) y el bump los toca
        a la vez o a ninguno. Y su lectura del teclado usaba `serialport` **prestado de
        `OrbyGUI/node_modules`**, que ya no lo tiene: el panel pasa a declararlo en un
        `package.json` propio.

- [x] **Paso 3: `entry.js`** se queda con dos vías: Tauri y navegador.

      Hecho. Fuera la rama `else if (window.orby)`; `platform.js` arranca en `'tauri'` en
      vez de en `'electron'`. La pregunta de `can()` sigue siendo «¿es navegador?» y no
      «¿es Tauri?»: lo que recorta funciones es la falta de backend, no cómo se llame
      quien lo pone.

- [x] **Paso 4: documentación.** Hecha el 2026-08-12.
  - `ORBY_V4/CLAUDE.md`: la tabla de arquitectura y los comandos.
  - `docs/WEBGUI.md`: la regla de «añadir a `PC_ONLY` y dar valor vacío» ahora tiene otro
    sitio donde mirar.
  - `docs/PLUGINS.md`: **reescrito entero**. Pasa de documentar un formato de programa a
    documentar un formato de fichero. Los dos tipos, los campos del manifiesto, las
    plantillas, `coalesce`, el protocolo completo de `process` (incluido el modo `socket`,
    aunque no esté implementado) y el presupuesto de las OLED. Este documento es lo que
    permite escribir el complemento de OBS o el de Altium sin volver a abrir OrbyGUI.
  - `docs/PUBLICACION.md`: empaquetado y canal de actualización.
  - `docs/TODO.md`: apuntar junto a cada complemento de la lista qué tipo le toca
    (`http` para las API REST; `process`+`stdio` para OBS, hardware y audio;
    `process`+`socket` para Altium y Chrome).

- [x] **Commit:** `git commit -am "feat(tauri): retirar Electron"`

---

## Actualizaciones automáticas y silenciosas (2026-08-12)

Fuera del plan original: se pidió después de cerrar la Tarea 13. **La app y el firmware se
instalan solos, sin intervención del usuario.** Las decisiones, tomadas con el autor:

| Punto | Decisión |
|---|---|
| Canal de la app | `tauri-plugin-updater`, con claves minisign y `latest.json` firmado |
| Publicación | **Manual desde el PC del autor**, documentada en `PUBLICACION.md`. Sin CI |
| Momento | **Inmediato**: se descarga, se instala y la app se reinicia sola |
| Firmware | Solo con el teclado **ocioso** (5 min sin actividad); nunca a media faena |
| Teclados sin BOOTSEL por serie | **No existen**, así que el camino manual no lo contempla el automático |
| Opt-out | Dos interruptores en Ajustes, encendidos de fábrica |

### La app: `src-tauri/src/updater.rs`

La forma exterior del estado **no cambia** —el renderer ya consumía
`{status, version, newVersion, percent, error, url}`—, solo el interior. `percent` pasa de
ser un campo que nadie movía a llevar la descarga de verdad.

**Se consultan dos sitios, y no es redundancia.** El plugin pide el `latest.json` de la
release, que es lo único que sabe firmar y verificar; pero eso no distingue «no hay nada
nuevo» de «la última release es de firmware». Por eso antes se mira la etiqueta por la API
de GitHub y se conserva la guarda de las `fw-v*`: si no, el fallo del `--prerelease` sale
como un 404 que no explica nada.

**Las guardas antes de reiniciar.** «Inmediato» no puede significar «a media faena»:
no se reinicia con un firmware instalándose (dejaría el teclado a medias, y de ahí solo se
sale con el botón BOOTSEL), ni con la grabadora grabando, ni con una secuencia en marcha.
En esos casos se queda instalada en disco y espera, reintentando cada minuto.

Y el reinicio **tiene que pasar por `window::marcar_saliendo()`**: `CloseRequested` está
interceptado para el cierre a la bandeja, así que sin eso `app.restart()` se quedaría en un
intento silencioso de esconder la ventana.

`updater_check` deja de ser bloqueante: se iba hasta 20 s del hilo de comandos con la
interfaz esperando el `invoke`, y con el automático encadenando descarga e instalación
encima eso ya no valía.

### El firmware: `src/firmware-auto.js`

El motor ya estaba entero en `firmware.rs`. Lo único nuevo es **quién aprieta el botón y
cuándo**, y vive en el renderer porque las condiciones solo se conocen ahí.

La señal de ocio es `device.on('telemetry')`, que `device.js` ya separaba de `response`:
solo cuenta lo que el teclado manda **por su cuenta**. Contando todo el tráfico, el latido
`HOST_APP` de cada ocho segundos haría que no hubiera ocio jamás.

Se flashea con teclado conectado, versión nueva disponible, **5 minutos sin telemetría**,
sin cambios pendientes de Flash (el reinicio se lleva lo que solo esté en RAM: es el mismo
motivo del aviso del botón manual) y sin grabación ni reproducción en marcha. Un fallo
**no se reintenta en bucle**: se anota la versión y no se vuelve a intentar en esa sesión.

El botón manual de Ajustes **se queda incluso con el automático puesto**, al revés que el
de la app: reinstalar el mismo firmware es la forma de recuperar un teclado que se quedó a
medias, y eso el automático no lo hará nunca porque para él ya está al día.

### El fallo latente que había que arreglar antes

`.github/workflows/firmware.yml` publicaba **sin `--prerelease`**, pese a que
`COMPATIBILIDAD.md` dice que no es opcional. Cada firmware publicado por CI se convertía en
`releases/latest` y dejaba el actualizador de la app en error. Con la actualización
silenciosa eso pasa de molestia a fallo mudo: nadie mira la pantalla. Arreglado.

### Lo que queda por hacer a mano

Tres cosas, y todas necesitan Windows y el teclado delante: **generar el par de claves
minisign**, **comprobar sobre el teclado** que la app se actualiza sola y que el firmware
se flashea estando ocioso, y **publicar** la primera release con los tres ficheros.

**Está todo escrito paso a paso en [PLAN_PUBLICAR.md](PLAN_PUBLICAR.md)**, con los comandos
exactos, lo que tiene que salir en cada uno y qué hacer si sale otra cosa. Ese documento es
el que hay que seguir; esta sección solo dice por qué existe.

Lo que no se pudo dejar hecho desde aquí es la clave: la privada no puede vivir en el
repositorio. Hasta que se genere, `tauri.conf.json` lleva un marcador y
`verifica_plan_tauri.sh` lo canta como **PENDIENTE** en cada ejecución.

Y un aviso sobre el orden, que no es el obvio: **publicar exige estar en `main`** (el
pipeline se niega en el primer paso), y `main` todavía no tiene la migración. Así que
fusionar va **antes** de publicar y **después** de comprobarlo todo sobre el teclado.

> **El riesgo que hay que decir en voz alta:** el instalador **no va firmado con
> certificado de código**. La minisign protege la descarga, no la reputación del binario
> ante SmartScreen. Una actualización que se instala sola, con `installMode: passive` y sin
> nadie mirando la pantalla, puede toparse con SmartScreen o con el antivirus y quedarse
> ahí. Si la actualización silenciosa se queda muda, ese es el primer sitio donde mirar.
> Lo barato que lo arreglaría está en `docs/TODO.md`: la cuenta de la Microsoft Store.

---

## Repaso final antes de dar el trabajo por hecho

Repasado el 2026-08-11 contra la build de **release** y el teclado real (COM7).

- [x] `cargo test` en verde, y **ningún test marcado como ignorado** sin explicar por qué.
      127 (orby-core) + 5 (descriptor de lampdesk) + 9 (orby-app), 0 fallos, 0 ignorados.
      Los dos nuevos de `orby-app` son de `autostart.rs`: sacar la ruta de la línea del
      registro y reconocer `--hidden` sin tragarse `--hidden-cosas`.
- [x] `node --test test/*.mjs` en verde. 14 pruebas.
- [x] Los 39 comandos existen y ninguno devuelve un valor de mentira.
      Los cinco que faltaban se implementaron en la Tarea 12 el 2026-08-12: **39 de 39**.

      > **`verifica_plan_tauri.sh` no comprobaba esto, y por eso se le añadió.** Su bloque
      > «Los 39 canales del plan existen» los busca en **`electron/main.js`**, que es la
      > *referencia* de la migración, no en `src-tauri/src/main.rs`: daba «0 discrepancias»
      > con cinco comandos sin implementar. Ahora hay un segundo bloque, «Los mismos 39
      > comandos existen en el invoke_handler de Rust», que lee el `generate_handler!` de
      > `main.rs`, que es lo único que decide si un `invoke` existe. El de Electron se deja:
      > sigue siendo la referencia de qué tiene que haber.
- [x] Ninguna vista, ni `device.js`, ni `store.js`, ni `compat.js` han cambiado.
- [x] La copia de seguridad hecha con Electron se restaura con Tauri sin perder nada.
      Ver la prueba de oro de la Tarea 4: las tres copias, 54 122 bytes, mismo contenido.
- [x] La app aguanta **cinco minutos conectada sin tocarla** sin desconectarse.
      Aguantó **6,0 minutos**: cero `disconnected`, cero `searching`, cero errores y 45
      renovaciones de `HOST_APP` por el camino.
- [x] Parar una reproducción a mitad no deja ninguna tecla pegada.
      Comprobado dos veces: cortando un `loop` con `recorder.stop()` y cortando un `hold`
      soltando la tecla física. `GetAsyncKeyState` da F15 y Control sueltas después.
- [x] Los números de la mejora (tamaño y RAM) están medidos y escritos.
      Ver Tarea 13, paso 1.

**Lo que queda para que Tauri pueda ser el modelo**, a fecha del 2026-08-12: la Tarea 12
está escrita y compila, y falta **la única comprobación que no se puede hacer sin cerrar
sesión**: activar el autoarranque, salir, volver a entrar y ver que arranca escondida en la
bandeja con el teclado detectado, y que al abrir Ajustes el interruptor sigue encendido.
La Tarea 11 está aplazada a propósito y no bloquea nada; la 13 es retirar Electron y
documentar, y va después.

**Descartado a propósito, no se va a hacer** (decisión del 2026-08-11):

| Qué | Por qué |
|---|---|
| Firmware por la vía **manual** (enchufar con BOOTSEL pulsado) | La vía automática funciona y es la que usa todo el mundo |
| Tiempo de arranque de las dos vías | El tamaño del instalador y la RAM ya justifican la migración de sobra |
| El **puente de Electron a Tauri** y que las dos convivan | No hay parque instalado que relevar: el único usuario ya tiene la de Tauri puesta a mano y la de Electron desinstalada. Ver la Tarea 12 |

## Si algo no cuadra

Este plan se escribió leyendo el código el 2026-08-11, y **todos** los datos concretos
que cita (rutas, constantes, los 39 canales, los 10 eventos, la superficie de
`preload.js`) se comprobaron contra el código real. Puedes volver a comprobarlo en
cualquier momento:

```bash
bash ORBY_V4/tools/test/verifica_plan_tauri.sh
# Esperado: TODO CUADRA: 0 discrepancias
```

**Lánzalo antes de empezar.** Si sale alguna discrepancia, el código ha derivado desde
que se escribió el plan: **para y dilo**. No improvises una alternativa ni «arregles» el
código para que encaje con el plan. Es mucho más barato corregir el plan.
