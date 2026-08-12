# ORBY_V4

Teclado macro con rueda. Dos mitades que hablan por USB CDC:

- **Firmware** — Raspberry Pi Pico (RP2040), C++17, Pico SDK 2.2.0 + TinyUSB. Raíz del proyecto.
- **OrbyGUI** — app de escritorio (Tauri 2 + Vite, cáscara en Rust y renderer JS vanilla)
  que configura el teclado. `OrbyGUI/`.
  La misma app corre también en el navegador (Chrome/Edge) por Web Serial, para
  equipos donde no se puede instalar nada: ver [docs/WEBGUI.md](OrbyGUI/docs/WEBGUI.md).

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
npm run tauri:dev    # vite + la cáscara de Rust, con recarga
npm run tauri:build  # instalador NSIS en src-tauri/target/release/bundle/nsis/
npm run build        # solo el frontend, a dist/
npm test             # node --test sobre los módulos puros y el contrato window.orby
npm run preview:web  # build + servidor local para probar la vía navegador en Chrome
```

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

### OrbyGUI — cáscara de escritorio (`OrbyGUI/src-tauri/`)
Rust. El crate de la app es `src-tauri/src/`; lo que no depende de la plataforma vive
aparte en `src-tauri/crates/orby-core/`, que compila solo y es donde están los tests.

| Fichero | Qué hace |
|---|---|
| `main.rs` | ventana, bandeja, una sola instancia y el `generate_handler!` con los 39 comandos |
| `serial.rs` | descubre el puerto por VID, reconecta, parte el flujo en líneas |
| `macros.rs` | ejecuta secuencias en el PC (Win32 directo: `SendInput`, portapapeles) |
| `recorder.rs` | graba y reproduce ratón/teclado (ganchos de bajo nivel de Win32) |
| `foreground.rs` | ventana en primer plano → dispara variaciones por app |
| `plugins.rs` | carga, ajustes y ejecución de complementos declarativos |
| `apps.rs` | lista apps instaladas del menú Inicio (`IShellLinkW` por COM) |
| `config.rs` | configuración local del PC (no del firmware) |
| `firmware.rs` | actualiza el firmware del teclado: releases `fw-v*`, `BOOTSEL`, copia del `.uf2` |
| `updater.rs` | actualiza la propia app, sola y en silencio (`tauri-plugin-updater`) |
| `autostart.rs` | entrada en la clave `Run` del registro, con `--hidden` |
| `window.rs` | cierre a la bandeja, arranque escondido y salida de verdad |
| `log.rs` | registro del ciclo de vida de la conexión en `%APPDATA%\OrbyGUI\orby.log` |

Todo va por `#[tauri::command]` + `invoke`; el renderer no toca el sistema directamente.
El contrato completo, con los nombres que ve el renderer, está en
`OrbyGUI/src/tauri/orby-tauri.js`, y `test/superficie-orby.test.mjs` lo compara con la
vía navegador.

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
- `firmware-auto.js` — decide **cuándo** se puede flashear solo: solo con el teclado ocioso.

### OrbyGUI — versión de navegador
`src/entry.js` arranca la vía Tauri o la vía navegador según exista
`window.__TAURI_INTERNALS__`. En navegador el puente lo monta `src/web/orby-web.js`
(Web Serial + IndexedDB en vez del backend de Rust), y `src/platform.js` es el único sitio donde se
pregunta dónde corre la app. Regla al añadir algo que necesite el proceso principal:
tocar `PC_ONLY` en `src/platform.js` **y** darle un valor vacío en
`src/web/orby-web.js`. Detalle completo en [docs/WEBGUI.md](OrbyGUI/docs/WEBGUI.md).

### Complementos
Carpeta con un `plugin.json` **declarativo**: describe peticiones, no ejecuta código.
Se instalan en `%APPDATA%\OrbyGUI\plugins\<id>\`, fuera de la app, para sobrevivir
actualizaciones. Ver [docs/PLUGINS.md](OrbyGUI/docs/PLUGINS.md).
Ejemplo de referencia: `OrbyGUI/plugins/lampdesk`.

## Convenciones

- **Todo en castellano**: comentarios, textos de UI, mensajes de commit, docs.
- Los comentarios explican **por qué**, no qué. Documentan la decisión y el fallo que
  evitan. Ejemplo real de `device.js`:
  > `// Se lleva aquí, y no en store.js, para no crear una dependencia circular`

  Mantén ese estilo: si añades un comentario que solo repite el código, sobra.
- Sin framework en el renderer, sin TypeScript. No los introduzcas sin pedirlo.
- Los cambios de protocolo tocan **tres** sitios: `main.cpp` (dispatch),
  `device.js` (petición) y la vista que lo use.

## Trampas conocidas

- **El firmware no tiene tests automatizados.** `tools/test/*.py` son comprobaciones
  sueltas (descriptor USB, mapa de Flash, teclas) que se lanzan a mano. OrbyGUI sí:
  `cargo test` en `src-tauri/`, `npm test` en `OrbyGUI/`, y
  `bash tools/test/verifica_plan_tauri.sh`, que comprueba que lo que dice
  `OrbyGUI/docs/PLAN_TAURI.md` sigue existiendo en el código.
- **La app instalada no tiene consola.** Cuando no detecta el teclado, el rastro
  está en `%APPDATA%\OrbyGUI\orby.log` (`src-tauri/src/log.rs`). En desarrollo,
  `npm run tauri:dev` saca además la consola del renderer y cada comando enviado.
- **`tauri build` falla si la app está abierta.** Cargo no puede sobrescribir
  `target/release/orby-app.exe` y el error no dice cuál es el problema (`os error 32`).
  Ciérrala antes: menú de la bandeja → Salir. Y ojo al encadenar la salida por una
  tubería (`| tee`, `| grep`): el código de salida pasa a ser el del último eslabón, así
  que un build fallido se lee como exitoso.
- **`assets/orby-icon.png` es un JPEG** con extensión `.png`. El generador de iconos de
  Tauri no lo traga: hay que convertirlo antes de dárselo.
- **El toolchain de Rust es GNU, sin las Build Tools de MSVC.** Por eso `reqwest` va con
  `rustls` y `zip` solo con `deflate`: nada que enlazar en C.
- **Rutas largas de Windows.** El repo está anidado y bajo OneDrive: operaciones de
  ficheros recursivas pueden fallar con "Filename too long". Usa el prefijo `\\?\`.
- **La app de escritorio y la WebGUI no pueden usar el teclado a la vez.** En Windows
  solo un proceso puede tener abierto el puerto COM: con OrbyGUI de escritorio
  corriendo, la WebGUI no consigue abrir el puerto. Ciérrala antes de probar la vía
  navegador.
- `orby-backup-*.json` en la raíz son copias de seguridad del usuario, no fixtures.
- `old/`, `build/` y `main_diag.cpp` son material muerto o de diagnóstico: no son
  la referencia de nada.
