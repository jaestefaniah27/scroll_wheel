# ORBY_V4

Teclado macro con rueda. Dos mitades que hablan por USB CDC:

- **Firmware** — Raspberry Pi Pico (RP2040), C++17, Pico SDK 2.2.0 + TinyUSB. Raíz del proyecto.
- **OrbyGUI** — app de escritorio (Tauri + Vite, JS vanilla) que configura el teclado. `OrbyGUI/`.
  La misma app corre también en el navegador (Chrome/Edge) por Web Serial, para
  equipos donde no se puede instalar nada: ver [docs/WEBGUI.md](OrbyGUI/docs/WEBGUI.md).

El backend de escritorio **era Electron y ahora es Tauri** (migración cerrada en la
v1.0.0-alpha, ver [docs/PLAN_TAURI.md](OrbyGUI/docs/PLAN_TAURI.md)). `OrbyGUI/electron/`
sigue en el repositorio y **no es material muerto**: su `preload.js` es la definición
de referencia del contrato `window.orby`, y hay un test que obliga a las otras dos vías
a cubrirlo entero. Lo que se compila y se publica hoy es la vía de Tauri.

El teclado funciona solo: la configuración vive en su Flash. OrbyGUI es un editor,
no un runtime — salvo para las acciones que solo el PC puede ejecutar (secuencias,
abrir apps, complementos).

## Comandos

### Firmware
```powershell
.\flash.ps1          # compila + mete la Pico en modo carga + copia el .uf2
cmake -B build -G Ninja && cmake --build build    # solo compilar
```
Salida: `build/ORBY_V4.uf2`.

**No hace falta pulsar BOOTSEL.** `flash.ps1` busca el puerto CDC del teclado por
VID (`VID_CAFE`, cualquier PID) y le manda el comando `BOOTSEL`, igual que hace
OrbyGUI. Cae al método manual si el teclado lleva un firmware anterior al 4.2, si
no está enchufado, o si **OrbyGUI tiene el puerto cogido** — ciérrala antes.
El volumen se busca por etiqueta `RPI-RP2` o por su `INFO_UF2.TXT`, no por letra
(la asigna Windows y cambia).

### OrbyGUI
```bash
cd OrbyGUI
npm install
npm run tauri:dev    # LA VÍA DE HOY: vite + Tauri, con recarga
npm run tauri:build  # instalador NSIS en src-tauri/target/release/bundle/nsis/
npm test             # node --test: módulos puros de src/web/ + el contrato window.orby
npm run preview:web  # build + servidor local para probar la vía navegador en Chrome

npm run dev          # vía Electron (anterior): vite + electron en paralelo
npm start            # vía Electron: build + electron, sin recarga
npm run dist         # vía Electron: instalador en release/
```

Los tests del backend son de Rust y van aparte — son la mayoría de los que hay:
```bash
cd OrbyGUI/src-tauri && cargo test -p orby-core   # 132 tests
```

**`npm run tauri:build` a secas produce una release rota.** Sin las variables de
firma, el `.sig` que necesita el actualizador automático no se genera. Ver
[docs/PUBLICACION.md](OrbyGUI/docs/PUBLICACION.md) antes de publicar nada.

## Arquitectura

### Firmware (`main.cpp`, ~3400 líneas)
Un solo fichero grande + cabeceras en `include/` (drivers: OLED, AS5600, encoder,
HID, pinout) y `src/` (descriptores USB, bitmaps).

- `pico_multicore`: núcleo 0 USB/HID, núcleo 1 entrada+OLED.
- `encoder.pio` genera la cabecera del PIO del encoder en tiempo de build.
- stdio por UART y USB **desactivados** — el CDC es el canal de control, no una consola.
- El dispatch de comandos serie está sobre la línea ~2180 de `main.cpp`: cadena de
  `strncmp(cmd, "...")`. Al añadir un comando, tócalo ahí **y** en `OrbyGUI/src/device.js`.

### Protocolo serie
Líneas de texto sobre CDC, `COMANDO:arg:arg`. No correlaciona petición y respuesta:
eso lo pone encima [device.js](OrbyGUI/src/device.js) con una cola de `pending`, cada
una con su `match(line)` y opcionalmente `collect(line)` para respuestas multilínea
(`GET_PROFILE`, `GET_STATE`). Timeout 4 s.

El teclado emite telemetría no solicitada: `EV:CTX:<perfil>:<pagina>:<total>` cuando
cambia de contexto por su cuenta (menú, gestor de páginas, tecla de salto).

### Versiones y compatibilidad
La versión del firmware sale de `include/orby_version.h` y de ahí van el `FW=`
del handshake y el `bcdDevice` del USB. Lo que la app sabe usar está en
`OrbyGUI/src/compat.js` (`FW_MIN`, `FW_RECOMMENDED`, tabla `FEATURES`).
La app degrada, no rompe. La tabla de qué trajo cada versión y cómo se publica
una release de firmware: [docs/COMPATIBILIDAD.md](docs/COMPATIBILIDAD.md).

Las vistas preguntan por función (`compat.supports(info, 'macros')`), nunca por
número de versión: cuando el handshake trae la bandera (`MACROS=1`, `HASH=1`,
`MAXMACROS`), manda la bandera sobre la versión.

### Publicar la app
Desde la 1.0.2, **OrbyGUI se actualiza sola**: descarga la versión nueva de fondo y la
instala sin preguntar (solo espera si hay cambios sin escribir en la Flash del teclado).
Eso convierte el procedimiento de publicación en algo que no se puede improvisar:

- La versión va en **tres** ficheros: `OrbyGUI/package.json`, `src-tauri/tauri.conf.json`
  y `src-tauri/Cargo.toml`.
- Cada release necesita el `.exe`, su `.sig` y un `latest.json`. **Sin ellos el
  actualizador de todo el parque instalado se queda mudo y no avisa de nada.**
- La firma sale de una clave minisign que vive fuera del repositorio y **no se puede
  sustituir**: cambiarla obliga a reinstalar a mano en todas partes.

Todo el detalle, incluido por qué las releases de firmware van como *prerelease*, en
[docs/PUBLICACION.md](OrbyGUI/docs/PUBLICACION.md) y [docs/COMPATIBILIDAD.md](docs/COMPATIBILIDAD.md).

### OrbyGUI — backend (`OrbyGUI/src-tauri/src/`)
Rust. Un módulo por asunto, con el mismo reparto que tenía la vía de Electron:

| Fichero | Qué hace |
|---|---|
| `main.rs` | registra plugins y comandos, arranca todo lo demás en `setup` |
| `window.rs` | ventana y bandeja. Cerrar **destruye** la ventana (no la esconde) y la app sigue viva |
| `serial.rs` | descubre el puerto, reconecta, parte el flujo en líneas |
| `macros.rs` | ejecuta secuencias en el PC |
| `recorder.rs` | graba y reproduce ratón/teclado |
| `foreground.rs` | ventana en primer plano → dispara variaciones por app |
| `plugins.rs` | carga, ajustes y ejecución de complementos |
| `apps.rs` | lista apps instaladas del menú Inicio |
| `config.rs` | configuración local del PC (no del firmware) |
| `firmware.rs` | actualiza el firmware del teclado: releases `fw-v*`, `BOOTSEL`, copia del `.uf2` |
| `updater.rs` | actualiza la propia app: descarga e instala sola, con firma minisign |
| `autostart.rs` | arrancar con la sesión de Windows |
| `backup.rs` · `dialog.rs` · `mouse.rs` | copias de seguridad, diálogos de fichero, posición del cursor |
| `log.rs` | registro del ciclo de vida de la conexión en `%APPDATA%\OrbyGUI\orby.log` |

Cada uno expone `#[tauri::command]`; el renderer los llama con `invoke` a través de
`src/tauri/orby-tauri.js`, que es quien monta `window.orby`. Nunca toca el backend directamente.

#### `orby-core`, la parte que se puede probar
`src-tauri/crates/orby-core/` es una biblioteca **pura**: sin red, sin disco, sin Tauri.
Ahí vive todo lo que *decide* algo —partir líneas del CDC, comparar versiones de firmware,
filtrar releases, fusionar configuración, mapear teclas, descodificar la grabadora, armar
las peticiones de un complemento— precisamente para poder comprobarlo sin un teclado
delante. Son **132 tests** y es donde toca añadir lógica nueva: los módulos de arriba
deberían quedarse con lo que necesita un socket o un fichero.

### OrbyGUI — renderer (`OrbyGUI/src/`)
JS vanilla con módulos ES, sin framework. Vistas en `src/views/`, cada una con
`init()` y `render()`; `main.js` las registra en `VIEWS` y conmuta con `switchView`.

- `store.js` — estado único (`state`) + `subscribe`/`notify`. `markDirty()` programa
  el autoguardado a Flash (1500 ms de agrupación).
- `device.js` — capa sobre `window.orby`, eventos con nombre + peticiones con promesa.
- `mirror.js` — copia local de los perfiles en disco. Sin teclado la app abre en
  **solo lectura** con esta copia; no se edita offline (evita dos versiones en conflicto).
- `oled-cache.js` / `oled-fb.js` — iconos OLED (framebuffer 1 bit, 360 bytes/pantalla).
- `variants.js` — variaciones de perfil por aplicación. Viven **solo en la RAM** del
  teclado: hay que revertir a base antes de escribir a Flash (`variants.withBase`).
- `plugins.js` — espejo en renderer de la lista de complementos.
- `compat.js` — qué firmware sabe usar esta app (`FW_MIN`, `FW_RECOMMENDED`, `FEATURES`).
- `firmware.js` — espejo de `src-tauri/src/firmware.rs`; pone el tope de versión desde `compat.js`.
- `updater.js` — espejo de `src-tauri/src/updater.rs`; le cuenta al backend si hay
  cambios sin guardar, para que no reinicie la app a media escritura en Flash.

### OrbyGUI — las tres vías
El mismo renderer arranca en tres sitios, y `src/entry.js` decide cuál montando el
`window.orby` que toque **antes** de que ningún módulo de `src/` lo mire:

| Vía | Quién monta `window.orby` | Cómo se detecta |
|---|---|---|
| Tauri | `src/tauri/orby-tauri.js` (`invoke` + eventos) | `window.__TAURI_INTERNALS__` |
| Electron | `electron/preload.js`, ya puesto al arrancar | `window.orby` existe |
| Navegador | `src/web/orby-web.js` (Web Serial + IndexedDB) | ninguna de las dos |

Tauri se comprueba **primero** a propósito: ver el comentario de `entry.js`.

`src/platform.js` es el único sitio donde se pregunta dónde corre la app. Regla al
añadir algo que necesite el backend: tocar `PC_ONLY` en `src/platform.js` **y** darle
un valor vacío en `src/web/orby-web.js`. Detalle en [docs/WEBGUI.md](OrbyGUI/docs/WEBGUI.md).

**El contrato es el mismo en las tres**, y `test/superficie-orby.test.mjs` lo hace
cumplir leyendo el código: un método que exista en una vía y no en las otras revienta
el test. Un método **de más** falla igual que uno de menos, porque alguna vista acabaría
usándolo y dejaría de funcionar en las otras dos.

### Complementos
Se instalan en `%APPDATA%\OrbyGUI\plugins\<id>\`, fuera de la app, para sobrevivir
actualizaciones. Ejemplo de referencia: `OrbyGUI/plugins/lampdesk`.

El modelo cambió con Tauri. Un complemento **de API 2** (`"apiVersion": 2`) es
declarativo: un `plugin.json` y nada más, sin código. El de tipo `http` describe qué
peticiones mandar, y quien las arma es `orby_core::plugins` —sin red, y por tanto con
tests—; `src-tauri/src/plugins.rs` solo abre el socket. Los de tipo `process` se cargan
y se ven en la lista, pero **todavía no hay quién los ejecute**: devuelven un error claro.

> [docs/PLUGINS.md](OrbyGUI/docs/PLUGINS.md) sigue describiendo la **API 1** de Electron
> (`plugin.json` + `main.js` de Node corriendo sin caja de arena dentro del proceso
> principal). Eso ya no es lo que hace la vía de Tauri. Al tocar complementos, la
> referencia es el código y `plugins/lampdesk/plugin.json`, no ese documento.

## Convenciones

- **Todo en castellano**: comentarios, textos de UI, mensajes de commit, docs.
- Los comentarios explican **por qué**, no qué. Documentan la decisión y el fallo que
  evitan. Ejemplo real de `device.js`:
  > `// Se lleva aquí, y no en store.js, para no crear una dependencia circular`

  Mantén ese estilo: si añades un comentario que solo repite el código, sobra.
- Sin framework en el renderer, sin TypeScript. No los introduzcas sin pedirlo.
- Los cambios de protocolo tocan **tres** sitios: `main.cpp` (dispatch),
  `device.js` (petición) y la vista que lo use.
- Un método nuevo en `window.orby` toca **las tres vías**: `src/tauri/orby-tauri.js`,
  `electron/preload.js` y `src/web/orby-web.js` (ahí, vacío). El test lo exige.
- La lógica que se pueda probar sin teclado ni red va en `orby-core`, no en los módulos
  de `src-tauri/src/`.

## Trampas conocidas

- **El firmware no tiene tests automatizados.** `tools/test/*.py` son comprobaciones
  sueltas (descriptor USB, mapa de Flash, teclas) que se lanzan a mano. La app sí los
  tiene: 132 en `cargo test -p orby-core` y 3 ficheros en `npm test`.
- **La app instalada no tiene consola.** Cuando no detecta el teclado, el rastro
  está en `%APPDATA%\OrbyGUI\orby.log` (`src-tauri/src/log.rs`). En desarrollo,
  `npm run tauri:dev` saca además la consola por el terminal.
- **OneDrive bloquea `node_modules/electron`.** Si `npm install` o el arranque de la vía
  de Electron fallan con ficheros en uso, es sincronización, no un bug del código.
- **VS Code filtra `ELECTRON_RUN_AS_NODE`.** Si Electron arranca como Node y no abre
  ventana, limpia esa variable antes de lanzar. Solo afecta a la vía de Electron.
- **`npm install` está roto de raíz.** `archiver@5.3.2`, peer de electron-builder, pide
  `async@^0.0.1`, versión que no existe en npm: cualquier instalación limpia falla con
  `ETARGET`. Con `--legacy-peer-deps` pasa; solo se queda fuera el árbol de
  `electron-builder-squirrel-windows`, que no se usa (el target es nsis).
- **El toolchain de Rust es GNU, sin las Build Tools de MSVC.** Por eso las dependencias
  se eligen sin nada en C: `rustls` y no `native-tls`, `zip` solo con `deflate`, Win32
  llamado directo en vez de módulos nativos de Node.
- **`assets/orby-icon.png` es un JPEG** con extensión `.png`. Electron-builder lo traga;
  el generador de iconos de Tauri no. Hay que convertirlo antes de dárselo.
- **Rutas largas de Windows.** El repo está anidado y bajo OneDrive: operaciones de
  ficheros recursivas pueden fallar con "Filename too long". Usa el prefijo `\\?\`.
- **La app de escritorio y la WebGUI no pueden usar el teclado a la vez.** En Windows
  solo un proceso puede tener abierto el puerto COM: con OrbyGUI de escritorio
  corriendo, la WebGUI no consigue abrir el puerto. Ciérrala antes de probar la vía
  navegador.
- **Cerrar la ventana NO cierra la app, y por tanto no suelta el puerto.** La X esconde
  a la bandeja (destruye la ventana, pero el proceso sigue vivo ejecutando secuencias).
  Para soltar el COM —al flashear con `flash.ps1`, o al probar la WebGUI— hay que usar
  **Salir** en el icono de la bandeja. Es la causa habitual de un "el puerto está
  ocupado" con la ventana ya cerrada.
- `orby-backup-*.json` en la raíz son copias de seguridad del usuario, no fixtures.
- `old/`, `build/` y `main_diag.cpp` son material muerto o de diagnóstico: no son
  la referencia de nada.
