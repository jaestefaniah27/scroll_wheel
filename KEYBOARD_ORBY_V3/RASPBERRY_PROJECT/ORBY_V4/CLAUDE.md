# ORBY_V4

Teclado macro con rueda. Dos mitades que hablan por USB CDC:

- **Firmware** — Raspberry Pi Pico (RP2040), C++17, Pico SDK 2.2.0 + TinyUSB. Raíz del proyecto.
- **OrbyGUI** — app de escritorio (Electron + Vite, JS vanilla) que configura el teclado. `OrbyGUI/`.

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
npm run dev          # vite + electron en paralelo, con recarga
npm start            # build + electron (sin recarga)
npm run dist         # instalador NSIS en release/
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

### OrbyGUI — proceso principal (`OrbyGUI/electron/`)
| Fichero | Qué hace |
|---|---|
| `main.js` | ventana, IPC, autoupdate (electron-updater) |
| `serial.js` | descubre el puerto, reconecta, parte el flujo en líneas |
| `preload.js` | única superficie expuesta al renderer: `window.orby` |
| `macros.js` | ejecuta secuencias en el PC (nut-js) |
| `recorder.js` | graba y reproduce ratón/teclado (uiohook-napi) |
| `foreground.js` | ventana en primer plano → dispara variaciones por app |
| `plugins.js` | carga, ajustes y ejecución de complementos |
| `apps.js` | lista apps instaladas del menú Inicio |
| `config.js` | configuración local del PC (no del firmware) |
| `firmware.js` | actualiza el firmware del teclado: releases `fw-v*`, `BOOTSEL`, copia del `.uf2` |

Todo IPC va por `ipcMain.handle`; el renderer nunca toca Node directamente.

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
- `firmware.js` — espejo de `electron/firmware.js`; pone el tope de versión desde `compat.js`.

### Complementos
Carpeta con `plugin.json` + `main.js` (CommonJS, Node). Se instalan en
`%APPDATA%\OrbyGUI\plugins\<id>\`, fuera de la app, para sobrevivir actualizaciones.
Corren **sin sandbox** dentro del proceso principal. Ver [docs/PLUGINS.md](OrbyGUI/docs/PLUGINS.md).
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

- **No hay tests automatizados.** `tools/test/*.py` son comprobaciones sueltas
  (descriptor USB, mapa de Flash, teclas) que se lanzan a mano.
- **OneDrive bloquea `node_modules/electron`.** Si `npm install` o el arranque fallan
  con ficheros en uso, es sincronización, no un bug del código.
- **VS Code filtra `ELECTRON_RUN_AS_NODE`.** Si Electron arranca como Node y no abre
  ventana, limpia esa variable antes de lanzar.
- **Rutas largas de Windows.** El repo está anidado y bajo OneDrive: operaciones de
  ficheros recursivas pueden fallar con "Filename too long". Usa el prefijo `\\?\`.
- `orby-backup-*.json` en la raíz son copias de seguridad del usuario, no fixtures.
- `old/`, `build/` y `main_diag.cpp` son material muerto o de diagnóstico: no son
  la referencia de nada.
