# Orby Manager — panel de control del proyecto

## Contexto

ORBY_V4 se publica a mano. GitHub Actions está bloqueado por facturación (ver
[docs/COMPATIBILIDAD.md](COMPATIBILIDAD.md#L54-L61)) y `gh` no está instalado en
esta máquina, así que cada release —de app o de firmware— es una secuencia de ocho o
diez comandos copiados de la documentación, con cuatro ficheros de versión que hay que
mantener sincronizados a mano y ningún sitio donde ver de un vistazo qué versión hay en
el repo, cuál está instalada y cuál lleva el teclado enchufado.

El objetivo es un panel local, arrancable desde un icono del escritorio, que haga esas
cuatro cosas: lanzar la app en sus tres vías, hacer bump de versión, publicar releases con
progreso en vivo, y enseñar el estado de versiones y releases.

**No sustituye a nada existente.** No toca el firmware, ni OrbyGUI, ni `flash.ps1`. Es una
herramienta de desarrollo que vive en `tools/orby-manager/` y llama a los scripts que ya hay.

## Decisiones tomadas

| Decisión | Elección |
|---|---|
| Stack | Node (módulos nativos) + HTML servido en el navegador por defecto |
| Alcance del release | Automático de punta a punta (build → tag → push → release → assets) |
| Bump | Solo edita ficheros y enseña el diff. Ni commit ni tag |
| Token de GitHub | `process.env.GH_TOKEN` (ya definido). No se guarda ni se muestra |

Cero dependencias nuevas: `http`, `child_process`, `fs`, `crypto` son de Node. La única
importación externa es `serialport`, que se resuelve desde `OrbyGUI/node_modules` y es
opcional (sin él, la tarjeta del teclado dice "no disponible" y el resto funciona).

## Ficheros a crear

```
tools/orby-manager/
  server.js            HTTP + SSE + enrutado de la API
  versiones.js         leer/escribir las fuentes de versión
  github.js            API REST de GitHub con GH_TOKEN
  tareas.js            spawn de procesos con log en vivo y pasos
  teclado.js           lectura del handshake por CDC
  index.html           la UI entera, un solo fichero
  Orby Manager.cmd     lanzador (node server.js + abre navegador)
  crear-acceso-directo.ps1
  README.md
```

Y una línea en `.gitignore` si hiciera falta para artefactos temporales (no se prevé:
todo lo que genera va a `build/` y `OrbyGUI/release/`, ya ignorados).

## Arquitectura

Servidor HTTP en `localhost:7788`. Si el puerto ya responde, el lanzador solo abre el
navegador en vez de arrancar otra instancia — una sola por máquina, porque las tareas
mutan el repositorio y dos paneles a la vez podrían lanzar dos builds simultáneos.

El estado vive en memoria del servidor: procesos hijo lanzados, tarea en curso, últimas
versiones leídas. La UI es un cliente tonto que pinta lo que llega por `GET /api/eventos`
(SSE) y hace POST para actuar.

### API

| Ruta | Qué hace |
|---|---|
| `GET /api/estado` | versiones (repo fw, repo app, instalada, teclado), procesos vivos, tarea en curso |
| `GET /api/eventos` | SSE: `log`, `paso`, `progreso`, `estado`, `fin` |
| `POST /api/lanzar` | `{modo}`: `electron-dev`, `electron`, `tauri-dev`, `tauri-build`, `web` |
| `POST /api/parar` | mata el proceso lanzado (árbol entero) |
| `POST /api/bump` | `{componente:'fw'\|'app', tipo, nota}` → edita ficheros, devuelve el diff |
| `POST /api/release` | `{componente:'fw'\|'app'}` → pipeline con pasos |
| `GET /api/releases` | listado de GitHub, con veredicto de "completada" |

## 1. Lanzar OrbyGUI

Cinco botones, todos `spawn` con `cwd: OrbyGUI/`, `shell: true` (son scripts de npm en
Windows), salida por SSE a un panel de log:

| Botón | Comando |
|---|---|
| Electron (dev) | `npm run dev` |
| Electron | `npm start` |
| Tauri (dev) | `npm run tauri:dev` |
| Tauri (build) | `npm run tauri:build` |
| Web | `npm run preview:web` + abrir `http://localhost:5174` |

Un solo proceso lanzado a la vez, con botón de parar. Se mata el árbol entero
(`taskkill /pid <pid> /T /F` en Windows): `npm run dev` arranca vite y electron con
`concurrently` y matar solo el padre deja los hijos huérfanos.

**Aviso en la UI, no en la consola**: mientras haya un OrbyGUI de escritorio corriendo,
el puerto COM está cogido y la lectura del teclado (punto 4) falla. Es la trampa
documentada en [CLAUDE.md](../CLAUDE.md). El panel lo dice en la tarjeta del teclado en vez
de enseñar un error.

## 2. Releases con barra de progreso

`tareas.js` expone un `Pipeline` con pasos declarados de antemano. La barra es
`pasosHechos / pasosTotales`; dentro de cada paso, las líneas de salida van al log. La
subida de assets emite progreso por bytes.

### Firmware (`fw-vX.Y`)

Réplica exacta de los pasos de [docs/COMPATIBILIDAD.md](COMPATIBILIDAD.md#L63-L81):

1. **Comprobaciones**: árbol limpio (`git status --porcelain`), rama `main`, y que la
   etiqueta `fw-vX.Y` no exista ya. La versión sale de `include/orby_version.h`.
   La comprobación "etiqueta y `ORBY_FW_MINOR` dicen lo mismo" —la que hacía el workflow
   muerto y que hoy no valida nadie— se hace aquí.
2. **Compilar**: `cmake -B build -G Ninja` si no hay `build/CMakeCache.txt`, luego
   `cmake --build build`. Misma lógica de caché inválida que [flash.ps1](../flash.ps1#L10-L16).
3. **Empaquetar**: copiar `build/ORBY_V4.uf2` a `build/ORBY_V4-fw-X.Y.uf2` y escribir
   `build/ORBY_V4-fw-X.Y.uf2.sha256` (`crypto.createHash('sha256')`).
4. **Etiquetar**: `git tag -a fw-vX.Y -m "Firmware X.Y"`, `git push origin main`,
   `git push origin fw-vX.Y`.
5. **Publicar**: `POST /repos/jaestefaniah27/scroll_wheel/releases` con
   **`prerelease: true`**. No es opcional y el pipeline no ofrece desmarcarlo: una release
   de firmware que no sea prerelease rompe el autoupdate de la app
   (ver [COMPATIBILIDAD.md](COMPATIBILIDAD.md#L86-L100)).
6. **Subir assets**: `.uf2` y `.uf2.sha256` al `upload_url` de la release.
7. **Verificar**: releer la release y comprobar que el `.uf2` está y su tamaño coincide.

### App (`vX.Y.Z`)

1. **Comprobaciones**: árbol limpio, `GH_TOKEN` presente, versión de `package.json`.
2. **Build + publicación**: `npm run release` (`vite build` + `electron-builder --publish always`).
   `publish.releaseType` ya es `release` en [package.json](../OrbyGUI/package.json#L68-L73),
   así que electron-builder crea y publica la release y sube `latest.yml` y el `.exe`.
   `GH_TOKEN` se le pasa por entorno.
3. **Huella**: SHA256 de `OrbyGUI/release/OrbyGUI-Setup-*.exe`, mostrado para pegarlo en
   las notas ([PUBLICACION.md](../OrbyGUI/docs/PUBLICACION.md#L147-L172)). Botón de copiar el
   bloque de notas ya formateado, con el aviso de SmartScreen incluido.
4. **Verificar**: la release `vX.Y.Z` existe, no es draft y lleva `latest.yml`.

Si un paso falla, el pipeline para, marca ese paso en rojo y deja el log completo. No
deshace nada: un tag empujado o media release se arreglan a mano y con el log delante.

## 3. Bump de versión

Solo edita ficheros. Al terminar muestra `git diff` para que se revise antes de commitear.

**Firmware** — `major` o `minor` (el firmware no tiene patch):
- `include/orby_version.h`: `ORBY_FW_MAJOR` / `ORBY_FW_MINOR`.
- `OrbyGUI/src/compat.js`: `FW_RECOMMENDED`.
- `docs/COMPATIBILIDAD.md`: fila nueva insertada justo bajo la cabecera de la tabla, entre
  los marcadores `<!-- tabla:inicio -->` y `<!-- tabla:fin -->` que ya existen. El texto de
  "qué trajo" lo escribe el usuario en un campo del formulario.

**App** — `major`, `minor` o `patch`:
- `OrbyGUI/package.json` → `version`
- `OrbyGUI/src-tauri/tauri.conf.json` → `version`
- `OrbyGUI/src-tauri/Cargo.toml` → `version` del paquete `orby-app`

`Cargo.lock` no se toca: cargo lo actualiza solo en el siguiente build, y editarlo a mano
es la vía rápida a un lock incoherente.

Las escrituras son quirúrgicas (regex sobre la línea concreta), no reserializaciones: volver
a escribir `package.json` con `JSON.stringify` reordena claves y destroza el diff.

## 4. Panel de versiones

Cuatro tarjetas, refrescadas al abrir y con botón de recargar:

| Tarjeta | De dónde sale |
|---|---|
| Firmware en el repo | `include/orby_version.h` |
| App en el repo | `OrbyGUI/package.json` |
| App instalada | registro `HKCU\Software\Microsoft\Windows\CurrentVersion\Uninstall`, entrada con `DisplayName` que contenga "Orby" → `DisplayVersion`. Comprobado: devuelve `0.5.2` |
| Teclado conectado | CDC a 115200, se manda `ACK\n` y se espera una línea `ORBY_V4:FW=…`, 1,5 s de margen |

La lectura del teclado replica lo que hace [electron/serial.js](../OrbyGUI/electron/serial.js#L458-L470):
puerto por VID `cafe` (cualquier PID) o fabricante que contenga "orby", `ACK\n`, parsear
`ORBY_V4:FW=4.5:KEYS=12:…`. Se abre y se cierra al momento; el panel nunca se queda con el
puerto cogido, o dejaría a OrbyGUI sin poder conectar.

Debajo, un semáforo con los desfases que importan: repo vs. instalada (¿hay que publicar?),
teclado vs. `FW_RECOMMENDED` (¿hay que flashear?), `FW_RECOMMENDED` vs. `orby_version.h`
(¿se olvidó `compat.js` en el último bump?).

## 5. Estado de las releases en GitHub

`GET /repos/jaestefaniah27/scroll_wheel/releases?per_page=20` con
`Authorization: Bearer $GH_TOKEN`. Tabla con etiqueta, fecha, `draft`/`prerelease` y assets.

Veredicto de "completada" por tipo de etiqueta:
- `fw-v*` → no es draft, **es** prerelease, y hay un asset `.uf2` de tamaño > 0.
- `v*` → no es draft, no es prerelease, y hay `latest.yml` **y** un `.exe`.

Cualquier otra combinación sale marcada, con el motivo. En particular un `fw-v*` que no sea
prerelease: es el fallo que dejó a las copias instaladas sin poder actualizarse.

Mientras hay un pipeline de release corriendo, la tabla se autorrefresca cada 15 s.

## 6. Acceso directo en el escritorio

- `Orby Manager.cmd`: arranca `node server.js` y abre `http://localhost:7788`. Si el puerto
  ya responde, solo abre el navegador.
- `crear-acceso-directo.ps1`: crea `%USERPROFILE%\Desktop\Orby Manager.lnk` con
  `WScript.Shell`, apuntando al `.cmd`, `WindowStyle = 7` (minimizado) e icono
  `OrbyGUI/src-tauri/icons/icon.ico`.

Se usa el `.ico` de Tauri a propósito: `assets/orby-icon.png` es un JPEG con extensión
equivocada y Windows no lo acepta como icono de acceso directo.

## Diseño de la UI

Un `index.html` con CSS y JS en línea, sin build. Cuatro secciones apiladas en una columna:
**Versiones** (las cuatro tarjetas + semáforo), **Lanzar** (los cinco botones + parar),
**Publicar** (bump y release, firmware y app, con la barra de pasos), **Releases** (la tabla).
Debajo, un panel de log fijo que recibe todo el SSE.

Paleta oscura tomada del `backgroundColor` de [tauri.conf.json](../OrbyGUI/src-tauri/tauri.conf.json#L21)
(`#0a0a0f`) para que no parezca una herramienta de otro proyecto. Textos en castellano,
como el resto del repositorio.

## Verificación

Sin tests automatizados: el repositorio no tiene infraestructura para ellos y meterla aquí
sería desproporcionado. Se comprueba a mano, en este orden:

1. `node tools/orby-manager/server.js` → abre en `http://localhost:7788`.
2. **Versiones**: las cuatro tarjetas se rellenan. Con el teclado enchufado y OrbyGUI
   cerrada debe salir `4.5`. Con OrbyGUI abierta, debe decir "puerto ocupado" y no romper.
3. **Lanzar**: cada uno de los cinco botones arranca lo suyo, el log corre, y "parar" deja
   cero procesos `node`/`electron` colgando (`Get-Process node,electron`).
4. **Bump** (en una rama de usar y tirar): bump de firmware a 4.6 y `git diff` debe tocar
   exactamente `orby_version.h`, `compat.js` y `COMPATIBILIDAD.md`. Bump de app a 0.5.3 debe
   tocar `package.json`, `tauri.conf.json` y `Cargo.toml`. `git checkout .` para revertir.
5. **Releases**: la tabla debe listar las `fw-v*` y `v*` reales, todas las de firmware
   marcadas como completadas.
6. **Release de firmware**: es la prueba cara. Se hace **solo cuando toque publicar de
   verdad** una versión. Antes, un ensayo en seco: un interruptor `?dry=1` que ejecuta todo
   menos `git push` y las llamadas de escritura a GitHub, y enseña qué habría hecho.
7. **Acceso directo**: ejecutar `crear-acceso-directo.ps1`, doble clic en el icono del
   escritorio, el panel abre.

## Riesgos conocidos

- **`npm install` está roto de raíz** en este repositorio (`ETARGET` por `archiver`). No
  afecta: el gestor no instala nada, y las dependencias ya están en `node_modules`.
- **OneDrive** puede bloquear ficheros durante el build de Electron. El pipeline enseñará el
  error tal cual; no es un fallo del gestor.
- **`git push` puede pedir credenciales**. Si el proceso se queda esperando en un prompt, el
  paso se corta por timeout (120 s) con un mensaje que lo explica, en vez de colgarse.
