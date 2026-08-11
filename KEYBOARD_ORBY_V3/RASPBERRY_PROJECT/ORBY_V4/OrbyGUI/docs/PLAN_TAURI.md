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

Las pruebas del primer bloque se contrastaron ejecutando las implementaciones JS actuales
con las mismas entradas: coinciden caso por caso. Y el descriptor de complementos se
compara contra un fichero de referencia **generado desde el propio `electron/plugins.js`**,
así que la igualdad con lo que ve hoy el renderer está comprobada, no supuesta.

```bash
cd OrbyGUI/src-tauri/crates/orby-core && cargo test
# Esperado: 100 passed (lib) + 5 passed (descriptor_lampdesk)

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

- [ ] **Paso 1: `src/platform.js`**

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

- [ ] **Paso 2: `src/entry.js`**

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

- [ ] **Paso 3: `src/tauri-main.js`**

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

- [ ] **Paso 4: `src/tauri/orby-tauri.js`**

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

- [ ] **Paso 5: comprobar que no se ha roto nada**

```bash
cd OrbyGUI && node --test test/*.mjs && npx vite build
```

Esperado: `# pass 8`, y el build de vite termina sin error. Esto solo demuestra que el
JS es sintácticamente válido y que las otras dos vías siguen en pie; la vía de Tauri no
arranca todavía porque no existe el backend.

- [ ] **Paso 6: commit**

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

- [ ] **Paso 5: commit**

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

- [ ] **Paso 4: comprobación sobre el teclado (Windows)** — parcial, 2026-08-11

- [x] Cierra la app de Electron antes (en Windows solo un proceso tiene el COM).
- [x] `npm run tauri:dev` → detecta el teclado y la interfaz se puebla con los perfiles.
      Traza: `probando COM7` → `>> ACK` → `teclado detectado: fw 4.6 en COM7` →
      `>> HOST_APP:1`, y detrás los 106 comandos de la sincronización completa
      (`GET_HASH`, `GET_STATE`, `GET_PROFILE:0/1`, todos los `GET_OLED`/`GET_OLED_PG`).
      Que lleguen los 106 ya demuestra que las respuestas vuelven: la cola de `pending`
      de `device.js` habría abortado en el primero que no contestara.
- [ ] Desenchufa el teclado: la interfaz pasa a «buscando» en menos de 20 s.
- [ ] Vuelve a enchufarlo: reconecta solo, sin tocar nada.
- [ ] Con la consola de la app abierta, gira el mando: se ven llegar líneas `ENC:`.
- [x] Déjala conectada **un minuto sin tocar nada**: no se desconecta (el watchdog está
      renovando bien). Este es el que más fallos pilla.
      Salió así: a los 12 s de silencio `>> ACK`, el teclado contesta con su presentación
      y se renueva `>> HOST_APP:1`, **sin** un segundo `teclado detectado` (no se
      re-anuncia una conexión que ya estaba) y sin desconexión.

> Las tres casillas que quedan sin marcar necesitan una mano: desenchufar el cable y girar
> el mando. Todo lo demás se comprobó sobre el teclado de verdad (COM7, firmware 4.6).

- [ ] **Paso 5: commit**

```bash
git commit -am "feat(tauri): descubrimiento, handshake y vigilancia del puerto serie"
```

---

## Tarea 4: Configuración local y copias de seguridad

**Ficheros:**
- Crear: `OrbyGUI/src-tauri/src/config.rs`, `OrbyGUI/src-tauri/src/backup.rs`

**Comandos:** `config_get`, `config_set(patch)`, `backup_save(data)`, `backup_load`.

La lógica ya está hecha (`orby_core::config`). Aquí solo falta el fichero y los diálogos.

- [ ] **Paso 1: el fichero**

`%APPDATA%\OrbyGUI\orby-config.json`, el **mismo** que usa Electron. En Tauri sale de
`app.path().app_config_dir()`; comprueba que resuelve a esa ruta exacta y **no** a
`OrbyGUI-tauri` ni similar: si cambia, se pierde la configuración del usuario al migrar.
El `identifier` de `tauri.conf.json` es lo que manda ahí.

- `config_get` lee, y ante cualquier error de lectura o de parseo devuelve
  `orby_core::config::defaults()`. Un JSON corrupto **no** puede impedir que la app abra.
- `config_set` fusiona con `merge` y devuelve **la configuración ya fusionada** (el
  renderer se queda con lo devuelto).
- Escribir con sangría, que estos ficheros se acaban leyendo a mano.

- [ ] **Paso 2: las copias**

`backup_save` abre un diálogo de guardar (`tauri-plugin-dialog`) y escribe el JSON.
`backup_load` abre uno de abrir y lo lee. Las dos formas de respuesta tienen que ser
**exactamente** `{ok:true,path}` / `{ok:true,data}` / `{ok:false,canceled:true}` /
`{ok:false,error}`, porque `src/backup.js` ya las distingue y es común a las tres vías.

- [ ] **Paso 3: la prueba de oro de la paridad (Windows, con teclado)**

Esta prueba vale por diez:

- [ ] Con la app de **Electron**, haz una copia de seguridad.
- [ ] Ciérrala. Abre la de **Tauri** y restaura esa copia.
- [ ] Comprueba que los perfiles, etiquetas, atajos y mandos quedan igual.
- [ ] Haz ahora una copia con la de Tauri y compárala con la de Electron: deben coincidir
      salvo la marca de tiempo.

- [ ] **Paso 4: commit**

```bash
git commit -am "feat(tauri): configuración local y copias de seguridad"
```

---

## Tarea 5: Ventana, bandeja, diálogos, apps instaladas y ratón

Un montón de piezas pequeñas, todas independientes.

**Comandos:** `window_minimize`, `window_maximize`, `window_close`,
`dialog_pick_app_or_file(kind)`, `apps_list_installed`, `mouse_get_position`.

- [ ] **Ventana.** El marco lo dibuja la app (`decorations: false`), así que los tres
      botones llaman a la ventana de Tauri. **`close` no cierra: esconde a la bandeja**,
      igual que hoy. Cerrar de verdad solo desde el menú de la bandeja.
- [ ] **Bandeja.** Icono con menú «Abrir OrbyGUI» / separador / «Salir». Clic simple
      alterna visibilidad; doble clic muestra.
- [ ] **Diálogo de fichero.** Filtro `exe, lnk, bat, cmd` cuando `kind` no es `'file'`.
- [ ] **Apps instaladas.** Recorre los dos menús de Inicio
      (`%ProgramData%\Microsoft\Windows\Start Menu\Programs` y el de `%APPDATA%`),
      recoge los `.lnk`, resuelve su destino con `IShellLink` (COM), descarta los que no
      apunten a un `.exe`, quita duplicados por destino en minúsculas, filtra los que
      casen con `uninstall|desinstal|read ?me|léeme|help|ayuda|website|sitio web|support|licen[cs]`
      y ordena con criterio español. Devuelve `[{name, target}]`.
- [ ] **Posición del ratón.** Tiene que salir del **mismo espacio de coordenadas con DPI**
      que usará la reproducción de la Tarea 7. Si la captura y la reproducción no
      coinciden, las secuencias con posiciones absolutas pinchan en sitios distintos en
      pantallas con escalado. Es el fallo más difícil de diagnosticar de toda la
      migración.

- [ ] **Comprobación (Windows):** minimizar, maximizar, cerrar a bandeja y recuperar desde
      la bandeja; la pestaña «App» de una tecla lista programas de verdad; el botón
      «Posición de ratón» del editor de secuencias devuelve la posición correcta **en un
      monitor con escalado al 150 %**.

- [ ] **Commit:** `git commit -am "feat(tauri): ventana, bandeja, diálogos y apps"`

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

- [ ] **Paso 1: la vía rápida del portapapeles**

Un texto de **5 caracteres o más** no se teclea letra a letra: se guarda el portapapeles,
se escribe el texto, se espera 30 ms, se manda `Ctrl+V`, se espera 120 ms y **se restaura
el portapapeles en un bloque que se ejecute pase lo que pase**. Por debajo de 5
caracteres se teclea con 4 ms entre letras. Los saltos de línea y los tabuladores van
como pulsaciones de verdad, no como caracteres.

> Perder la restauración del portapapeles es de los fallos que más molestan y que nadie
> asocia con el teclado.

- [ ] **Paso 2: acciones de energía** (`std::process::Command`, todas de Windows):

| `mode` | Comando |
|---|---|
| `sleep` | `rundll32.exe powrprof.dll,SetSuspendState 0,1,0` |
| `hibernate` | `shutdown /h` |
| `restart` | `shutdown /r /t 0` |
| `shutdown` | `shutdown /s /t 0` |
| `lock` | `rundll32.exe user32.dll,LockWorkStation` |
| `logoff` | `shutdown /l` |

- [ ] **Paso 3: retardos entre repeticiones.** Por defecto **20 ms** entre repeticiones.
      Y al escribir, poner el retardo automático de la librería de entrada **a cero**: los
      300/100 ms que traen por defecto añaden más de medio segundo por repetición.

- [ ] **Paso 4: tests** de la traducción de códigos HID a teclas (tabla de la página de
      uso `0x07`) y de los bits de modificador (`0x01..0x80` → Control/Shift/Alt/Super,
      izquierda y derecha). Van en `orby-core`, son puros.

- [ ] **Comprobación (Windows):** una secuencia con texto largo (comprobar que el
      portapapeles vuelve a lo que había), otra con combinación, otra que abra un
      programa, y una con posición absoluta de ratón **en un monitor con escalado**.

- [ ] **Commit:** `git commit -am "feat(tauri): ejecución de secuencias en el PC"`

---

## Tarea 7: Grabadora de ratón y teclado

**Ficheros:** `OrbyGUI/src-tauri/src/recorder.rs`

**Comandos:** `recorder_toggle(id)`, `recorder_stop`, `recorder_status`.
**Evento:** `recorder:state` con `{id, phase}`, donde `phase` es una **cadena**.

- [ ] **Paso 1: la captura.** Ganchos de bajo nivel de Windows (`WH_KEYBOARD_LL` y
      `WH_MOUSE_LL`) en un **hilo propio con su bombeo de mensajes**: sin bombeo no llega
      ningún evento y el fallo es silencioso.

- [ ] **Paso 2: forma de los eventos grabados.** Conservarla, que es lo que ya guarda la
      configuración del usuario: `{k:'kdown'|'kup', c, t}`, `{k:'mdown'|'mup', b, x, y, t}`,
      `{k:'wheel', r, d, t}`, `{k:'move', x, y, t}`.

- [ ] **Paso 3: filtrado del movimiento.** Un evento de movimiento como mucho cada
      **16 ms** y si se ha movido **3 px** o más. Sin esto, un gesto de dos segundos son
      miles de eventos.

- [ ] **Paso 4: recortar el silencio inicial.** Al parar, restar a todos los tiempos el
      del primer evento: si no, la reproducción empieza esperando lo que tardaste en
      empezar a moverte.

- [ ] **Paso 5: reproducción.** Modos `once`, `loop` y `hold`. `speed` **divide** los
      tiempos. Antes de un clic, recolocar el ratón (un movimiento pudo caer en el
      filtrado). Y lo importante: **llevar la cuenta de teclas y botones pulsados y
      soltarlos siempre al terminar**, también si se para a mitad. Si no, te quedas con el
      Ctrl pegado y el PC inservible hasta reiniciar.

- [ ] **Paso 6: apagado.** Parar los ganchos al cerrar la app o el proceso no termina.

- [ ] **Comprobación (Windows):** grabar un gesto con clics y escritura; reproducir en los
      tres modos; **parar a mitad de un bucle y comprobar que no queda ninguna tecla
      pegada** (probar escribiendo en el bloc de notas después).

- [ ] **Commit:** `git commit -am "feat(tauri): grabadora de ratón y teclado"`

---

## Tarea 8: Ventana en primer plano y cambio automático de perfil

**Ficheros:** `OrbyGUI/src-tauri/src/foreground.rs`

**Comandos:** `foreground_start`, `foreground_stop`, `foreground_current`,
`foreground_available`. **Eventos:** `foreground:change`, `foreground:error`.

Hoy esto es un **PowerShell permanente** al que se le pasa un script en base64 y que hace
P/Invoke contra `user32.dll`. En Rust son cuatro llamadas directas y desaparece el
proceso hijo: `GetForegroundWindow`, `GetWindowThreadProcessId`, `GetWindowText` y
`QueryFullProcessImageName`.

- [ ] Sondeo cada **400 ms**, y emitir **solo cuando cambia** la ventana o el título.
- [ ] Devolver `{process, title, path}`.
- [ ] `foreground_available` da `true` en Windows.
- [ ] Arrancar el vigilante al iniciar **solo si** `config.autoProfile.enabled`.

- [ ] **Comprobación (Windows):** crear una regla que cambie de perfil al pasar a otro
      programa; alternar entre dos programas y ver que el perfil cambia. Comprobar también
      que con el vigilante parado no queda ningún proceso suelto.

- [ ] **Commit:** `git commit -am "feat(tauri): ventana en primer plano sin PowerShell"`

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
- [ ] **Descargar** el `.uf2` con barra de progreso, validando el tamaño.
- [ ] **Entrar en modo carga.** Si no hay unidad de arranque, mandar `BOOTSEL` por el
      puerto y esperar a que aparezca la unidad (hasta 90 s, mirando cada 400 ms).
- [ ] **Encontrar la unidad**: recorrer de `C:` a `Z:` buscando un `INFO_UF2.TXT` que
      contenga `RP2`. **Por etiqueta no**: la letra la asigna Windows y cambia.
- [ ] **Copiar el fichero y tragarse los errores** `EIO`, `EPERM`, `ENOENT`, `EBUSY` e
      `EINVAL`. La Pico se reinicia a mitad de la copia y el volumen desaparece: **eso es
      el éxito**, no un fallo. Tratarlos como error es el fallo clásico aquí.

- [ ] **Comprobación (Windows, con teclado):** actualizar por la vía `BOOTSEL` y por la
      manual. Confirmar que al acabar el teclado vuelve con la versión nueva.

- [ ] **Commit:** `git commit -am "feat(tauri): actualización del firmware"`

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

- [ ] **Paso 1: el manifiesto y su validación (con tests)**

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

- [ ] **Paso 2: las plantillas (con tests)**

Solo sustitución, sin expresiones ni lógica: `{{settings.x}}`, `{{value}}`,
`{{response.x}}`.

- [ ] `{{value}}` con `0` y con negativos (un giro hacia atrás es `-5`).
- [ ] `{{response.x}}` con el campo ausente.
- [ ] Una plantilla desconocida **falla**, no interpola vacío. Interpolar vacío manda una
      petición mal formada que el cacharro del otro lado interpreta como quiere.
- [ ] La URL y la cadena de consulta se montan bien, con codificación incluida.

- [ ] **Paso 3: agrupar los giros (`coalesce`), con tests**

`coalesce: {ms, mode:"sum"}`. El firmware manda un `MACRO:<id>` **por cada muesca**, así
que un giro rápido son veinte disparos; agruparlos no es un lujo.

- [ ] Dos muescas de `+5` producen **una** petición con `10`.
- [ ] Si llegan más muescas mientras una petición está en vuelo, se encadena otra ronda.
- [ ] Una acción **sin** `coalesce` manda una petición por muesca (encender y apagar no se
      pueden fundir: dos pulsaciones son dos órdenes).

- [ ] **Paso 4: peticiones, contra un servidor de mentira levantado en el propio test**

- [ ] `timeoutMs` corta de verdad. Sin él, un cacharro desenchufado deja la ejecución
      colgada más de un minuto esperando a que agote el TCP.
- [ ] Un `500` se reporta como error, no se propaga como pánico.
- [ ] `test` con `retries: 1` reintenta **una vez y solo una**.
- [ ] `run` **no reintenta nunca**: un incremento repetido se aplicaría dos veces.

- [ ] **Paso 5: poner lampdesk en su sitio**

**Ya está escrito**, en
`src-tauri/crates/orby-core/tests/fixtures/lampdesk-api2.plugin.json`, y el test de
descriptor lo usa. Está ahí y no en `plugins/lampdesk/` porque el de allí sigue siendo
el de la API 1: **la app de Electron tiene que seguir funcionando** hasta la Tarea 13, y
un mismo fichero no puede declarar las dos versiones.

Lo que toca aquí es: copiarlo a `plugins/lampdesk/plugin.json`, **borrar
`plugins/lampdesk/main.js`** y cambiar en el test de descriptor el `include_str!` para
que apunte al manifiesto de verdad en vez de a la copia.

Ya está resuelto en él: `timeoutMs: 4000`, `coalesce.ms: 120` en los dos `_delta`,
`retries: 1` en `test`, y las equivalencias `toggle`→`on=toggle`, `on`→`on=1`,
`off`→`on=0`, `brightness_delta`→`dbrightness={{value}}`, `color_delta`→`dcolor={{value}}`,
`brightness_set`→`brightness={{value}}`, `color_set`→`color={{value}}`, con los visores
leyendo `/state`. Su lista de `HOSTS_MUERTOS` **no se portó**: era un apaño de la época
de desarrollo.

- [ ] **Paso 6: instalación**

Descomprimir con el crate `zip` (**no** con `Expand-Archive` de PowerShell). Un
complemento `http` **se instala sin ningún aviso**: no hay programa que instalar. Los
ajustes siguen viviendo en `config.plugins["<id>"]`, con la misma forma, para que
sobrevivan a desinstalar y volver a instalar.

- [ ] **Comprobación (Windows, con la lámpara):** instalar el `.zip`, ajustar la
      dirección, «Probar», asignar acciones a una tecla y a un mando, y **girar el mando
      rápido**: el brillo tiene que subir liso, sin tirones ni retraso acumulado.

- [ ] **Commit:** `git commit -am "feat(tauri): complementos declarativos"`

---

## Tarea 11: Complementos, tipo `process` (esqueleto)

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

- [ ] **Paso 1: autoarranque.** Entrada en el registro de usuario (clave `Run`) que
      conserve el argumento `--hidden`. Y al **leerla**, comparar ruta **y argumentos**:
      comparando solo la ruta siempre devuelve que está desactivado.

- [ ] **Paso 2: arranque escondido.** Con `--hidden` la app arranca sin mostrar la
      ventana, solo el icono de la bandeja.

- [ ] **Paso 3: empaquetado NSIS** con Tauri, conservando el nombre de producto
      `OrbyGUI` y el mismo identificador, para que actualice **sobre** la instalación
      existente en vez de dejar dos apps.

- [ ] **Paso 4: el relevo del actualizador.** Leer esto entero antes de tocar nada:

  Hoy los usuarios tienen `electron-updater` mirando las releases de GitHub. Si la app de
  Tauri sale con su propio actualizador y ya está, **los que tengan la de Electron no se
  enterarán nunca** y se quedan varados en la última versión de Electron.

  El orden correcto es:
  1. Publicar una **última versión de Electron** cuya actualización instale la app de
     Tauri.
  2. Solo después, pasar las siguientes releases al formato del actualizador de Tauri.

  Y hay que **conservar** el apaño de que **las releases de firmware van como
  *prerelease***: si una release `fw-v*` se publica como definitiva, pasa a ser
  `releases/latest` y rompe la actualización de **todo** el parque instalado. Ya pasó una
  vez, está documentado en `ORBY_V4/docs/COMPATIBILIDAD.md` (ojo: ese está un nivel por
  encima, en `ORBY_V4/docs/`, no en `OrbyGUI/docs/` como los demás que cita este plan).

- [ ] **Comprobación (Windows):** instalar la versión de Electron anterior, actualizar y
      comprobar que queda la de Tauri, **con la configuración y los perfiles intactos**.
      Activar el autoarranque, reiniciar sesión y comprobar que arranca escondida.

- [ ] **Commit:** `git commit -am "feat(tauri): actualizador, autoarranque y empaquetado"`

---

## Tarea 13: Retirar Electron y documentar

Solo cuando **todo lo anterior** esté comprobado sobre el teclado.

- [ ] **Paso 1: medir**, que es lo que justifica la migración. Con las dos builds contra el
      mismo teclado, anotar: tamaño del instalador, RAM en reposo y tiempo de arranque.
      Poner los números en el mensaje del commit.

- [ ] **Paso 2: quitar** `electron/`, las dependencias de Electron de `package.json`
      (`electron`, `electron-builder`, `electron-updater`, `serialport`, `uiohook-napi`,
      `@nut-tree-fork/nut-js`) y sus scripts. Borrar el `TAURI_PENDIENTE` de
      `platform.js` si quedó alguno.

- [ ] **Paso 3: `entry.js`** se queda con dos vías: Tauri y navegador.

- [ ] **Paso 4: documentación.**
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

- [ ] **Commit:** `git commit -am "feat(tauri): retirar Electron"`

---

## Repaso final antes de dar el trabajo por hecho

- [ ] `cargo test` en verde, y **ningún test marcado como ignorado** sin explicar por qué.
- [ ] `node --test test/*.mjs` en verde.
- [ ] Los 39 comandos existen y ninguno devuelve un valor de mentira.
- [ ] Ninguna vista, ni `device.js`, ni `store.js`, ni `compat.js` han cambiado.
- [ ] La copia de seguridad hecha con Electron se restaura con Tauri sin perder nada.
- [ ] La app aguanta **cinco minutos conectada sin tocarla** sin desconectarse.
- [ ] Parar una reproducción a mitad no deja ninguna tecla pegada.
- [ ] Los números de la mejora (tamaño, RAM, arranque) están medidos y escritos.

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
