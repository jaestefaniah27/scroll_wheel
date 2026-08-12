# Orby Manager

Panel local de control para ORBY_V4: versiones, lanzar la app en sus vías y publicar
releases. Nació porque cada publicación —de app o de firmware— era una secuencia de
ocho o diez comandos copiados a mano de la documentación, con cuatro ficheros de
versión que sincronizar y ningún sitio donde ver de un vistazo qué versión hay en el
repositorio, cuál está instalada y cuál lleva el teclado enchufado.

## Qué es y qué no es

Es una herramienta de desarrollo local, no un sustituto de nada. No toca el firmware,
ni OrbyGUI, ni `flash.ps1`: llama a los mismos scripts y comandos que ya existían
(`cmake`, `flash.ps1`, `npm run tauri:dev`, `npm run tauri:build`, `git`, la API de
GitHub). Si
algo falla aquí, falla igual a mano.

Se publica a mano porque GitHub Actions está bloqueado por facturación en esta cuenta
(ver [docs/COMPATIBILIDAD.md](../../docs/COMPATIBILIDAD.md#publicar-una-versión-de-firmware))
y `gh` no está instalado en esta máquina. El panel no rodea ese bloqueo, solo evita
teclear los mismos comandos y olvidar uno de los ficheros de versión.

## Cómo se arranca

```powershell
cd tools\orby-manager
node server.js
```

O `Orby Manager.cmd` (mismo efecto, más la apertura del navegador), o el acceso
directo del escritorio que crea `crear-acceso-directo.ps1`:

```powershell
.\crear-acceso-directo.ps1
```

Sirve en `http://localhost:7788`. El servidor escucha solo en `127.0.0.1` y, además,
`server.js` rechaza con 403 cualquier petición cuyo `Host` no sea `localhost`/
`127.0.0.1`/`[::1]` o cuyo `Origin` (si lo trae) no apunte a ese mismo puerto. No es
paranoia gratuita: este panel compila, empuja a git y publica en GitHub con el
`GH_TOKEN` del usuario, así que una página cualquiera de internet que consiguiera
resolver su dominio a `127.0.0.1` y hacerle un POST estaría publicando en su nombre.

Una sola instancia por máquina: dos paneles a la vez podrían lanzar dos builds sobre
los mismos ficheros o publicar la misma release dos veces. `Orby Manager.cmd` mira con
`netstat` si el puerto 7788 ya está escuchando antes de arrancar `node`; si lo está,
solo abre la pestaña del navegador. `server.js` también se niega a arrancar si el
puerto está ocupado (`EADDRINUSE`) y lo dice por consola.

## Requisitos

- **Node** (el mismo que usa OrbyGUI), y un `npm install` en esta carpeta. Todo lo
  demás que usa el panel (`http`, `child_process`, `fs`, `crypto`, `https`) es de
  Node. `npm test` corre las pruebas de `subirSemver`, que es de donde sale el número
  de todas las publicaciones.
- **`GH_TOKEN` en el entorno**, para leer y publicar releases. `github.js` es el único
  módulo que lo lee (`process.env.GH_TOKEN`); no se guarda en disco, no se registra en
  el log y no se enseña en la UI. Sin él, el panel funciona igual salvo para publicar:
  el semáforo de Versiones lo avisa en rojo al abrir y el primer paso de cada pipeline
  de release corta con un mensaje claro.
- **`serialport`**, su única dependencia, para leer el firmware del teclado por CDC.
  La tenía prestada de `OrbyGUI/node_modules` mientras OrbyGUI era una app de Node;
  ahora la app abre el puerto desde Rust y ya no la instala, así que el panel la
  declara en su propio `package.json`. Sigue intentando primero la de OrbyGUI por si
  quedó instalada de antes. Si no encuentra ninguna, la tarjeta del teclado dice que
  no está disponible y el resto del panel sigue funcionando.

## Layout

La página es de dos columnas a pantalla completa, sin scroll en `body`. La
izquierda es el trabajo: las seis secciones de abajo, cada una en su
`<section>`, con scroll propio en la columna. La derecha es una columna de
actividad fija —`#columna-actividad`—, con el panel de la tarea en curso
(nombre, barra de progreso, lista de pasos, notas) arriba y el registro
debajo; cualquier botón de cualquier sección (un bump, una release, una
herramienta, el flasheo, un commit) llena ese mismo panel y ese mismo log,
porque todos comparten el mismo `Pipeline` por debajo. Por debajo de 1100 px
las dos columnas se apilan, con la de actividad limitada al 42 % de la altura.

Antes el registro era una franja fija bajo una sola columna. Se ha movido a un
panel lateral porque se consulta *mientras* se trabaja: ocupando la mitad
inferior obligaba a hacer scroll para ver a la vez lo que se lanza arriba y lo
que sale abajo.

## Las seis secciones

### Versiones

Cinco tarjetas, refrescadas al abrir y con botón «Recargar»:

| Tarjeta | De dónde sale |
|---|---|
| Firmware en el repo | `include/orby_version.h` (`ORBY_FW_MAJOR`/`ORBY_FW_MINOR`) |
| App en el repo | `OrbyGUI/src-tauri/tauri.conf.json`, `OrbyGUI/src-tauri/Cargo.toml` y `OrbyGUI/package.json`; si los tres ficheros no coinciden, la tarjeta enseña los tres valores separados por «/» y se marca en rojo, en vez de elegir uno y callarse los otros |
| App instalada | Registro de Windows, `HKCU\...\Uninstall`, la entrada cuyo `DisplayName` contiene «Orby» |
| Teclado conectado | Handshake por CDC (`ACK\n` → `ORBY_V4:FW=…`), abriendo y cerrando el puerto al momento. Lleva el botón «Flashear el teclado» (ver más abajo) |

La app lleva un solo número, repartido en tres ficheros que tienen que decir
lo mismo: `tauri.conf.json` (el que acaba en el instalador y en el
`latest.json` que mira el actualizador), `Cargo.toml` (el que compila el
binario) y `package.json` (el del frontend). La vía navegador no tiene versión
propia a propósito: sale del mismo `vite build` y se sirve como ficheros
estáticos, sin nada que instalar ni que actualizar.

Debajo, un semáforo con los desfases que importan, cada uno con nivel
ok/aviso/error:

- **Repo vs. instalada** — la versión del repositorio contra lo que hay
  instalado: ¿hay que publicar o instalar?
- **Los tres ficheros de versión entre sí** — si se han desincronizado, es un
  error: el instalador, el binario compilado y el frontend dirían versiones
  distintas.
- **Teclado vs. `FW_RECOMMENDED`** (de `OrbyGUI/src/compat.js`) — ¿hay que flashear
  este teclado?
- **`orby_version.h` vs. `FW_RECOMMENDED`** — ¿se olvidó `compat.js` en el último
  bump? Es el desfase silencioso: el firmware sube de versión, la app sigue
  ofreciendo la anterior y nadie se entera hasta que el instalador de firmware no
  encuentra la release.

Si el teclado no contesta porque el puerto está cogido (`Access denied`/`Permission
denied`, típicamente OrbyGUI de escritorio abierta), el aviso lo dice tal cual, en
vez de comparar contra una versión que no se ha podido leer.

### Flashear el teclado

Botón «Flashear el teclado» en la propia tarjeta del teclado, `POST
/api/flashear`. Tres pasos, siempre en un `Pipeline` sin ensayo en seco (no
tiene sentido ensayar algo que, de fallar a medias, deja al teclado sin
firmware que anunciar):

1. **Comprobar el teclado** — se niega si hay un OrbyGUI lanzado desde la
   sección Lanzar (tiene el puerto cogido) o si el puerto está ocupado por
   otra cosa: `flash.ps1` necesita el puerto libre para mandarle el comando
   `BOOTSEL` al teclado, y si no lo consigue cae al método manual y se queda
   esperando hasta 120 s a que alguien lo enchufe con el botón físico pulsado.
2. **Compilar y flashear** — ejecuta `flash.ps1` tal cual (`powershell
   -NoProfile -ExecutionPolicy Bypass -File flash.ps1`). No se reimplementa
   nada de lo que ese script ya hace y tiene probado (pedir el BOOTSEL,
   encontrar la unidad `RPI-RP2`, copiar el `.uf2`).
3. **Verificar** — el teclado se reinicia y tiene que volver a enumerarse: se
   relee el handshake hasta 6 veces cada 2,5 s. Si vuelve anunciando una
   versión distinta de la que se acababa de instalar, el paso falla.

La UI avisa con un `confirm()` de qué versión se va a instalar sobre cuál, y
en particular si es una **bajada** de versión (comparando con la misma regla
que usa el semáforo).

### Lanzar

Tres botones, uno a la vez, todos `spawn` con `cwd: OrbyGUI/` y salida en vivo al
panel de log:

| Botón | Comando |
|---|---|
| Tauri (dev) | `npm run tauri:dev` |
| Tauri (build) | `npm run tauri:build` |
| Web | `npm run preview:web`, con el navegador abierto solo a los 6 s (margen para que vite esté escuchando) |

Solo puede haber un proceso lanzado a la vez: lanzar un segundo modo con otro ya
corriendo falla con un mensaje que dice cuál es. «Parar» mata el árbol de procesos
entero (`taskkill /pid <pid> /T /F` en Windows), no solo el proceso hijo directo:
`npm run tauri:dev` levanta vite y la cáscara de Rust, y matar solo al padre deja
los dos corriendo por detrás, con el puerto COM del teclado cogido y sin ventana
desde la que cerrarlos.

Si el servidor del panel se cierra (Ctrl+C, cerrar la consola) con algo lanzado, mata
el árbol antes de salir por la misma razón.

Este candado —un proceso de OrbyGUI lanzado a la vez— es independiente del
candado de tareas largas (bump, release, herramienta, flasheo, commit): cada
uno tiene el suyo, salvo el caso concreto de Flashear el teclado, que exige
explícitamente que no haya nada lanzado porque necesita el puerto libre (ver
más arriba).

### Herramientas

Un botón por cada entrada de `HERRAMIENTAS` en `tareas.js`, servido en
`/api/estado` (campo `herramientas`) para que añadir una sea tocar un sitio y
no dos: el HTML no lista los nombres a mano, los pinta el cliente a partir de
lo que devuelve el servidor. `POST /api/herramienta {clave}`.

| Clave | Qué hace |
|---|---|
| `compilar-fw` | `cmake -B build -G Ninja` (si no hay caché) + `cmake --build build` |
| `limpiar-build` | Borra `build/` entera. Pide confirmación antes: es la única destructiva de la lista |
| `dist` | `npm run dist` (`--publish never`): empaqueta el instalador sin publicar nada. Al terminar enseña la ruta del `.exe` y su SHA256 |
| `pack-plugins` | `npm run pack:plugins` |
| `git-pull` | `git pull --ff-only` — si la rama ha divergido, mejor enterarse aquí que quedarse con un merge automático hecho a espaldas de nadie |
| `git-fetch` | `git fetch --tags --prune` |

Cada herramienta corre en su propio `Pipeline` de un solo paso, así que
comparte barra, log y el candado de «una tarea a la vez» con las releases y el
flasheo: compilar el firmware a la vez que corre una release sería pedir un
lío que luego no hay forma de leer en el log.

### Publicar

Bump de versión y release, cada uno por separado para firmware y para la app.
El bump se cubre en su propia sección más abajo.

La release corre como un `Pipeline` con pasos declarados de antemano: una barra de
progreso (`pasos hechos / pasos totales`) y, dentro de cada paso, el log línea a
línea. Un checkbox «ensayo en seco» por cada botón de publicar (marcado por defecto
para firmware, sin marcar para app) — ver más abajo. Publicar de verdad (checkbox sin
marcar) pide confirmación con un `window.confirm` antes de arrancar; en ensayo, no.

### Git

`GET /api/git` — rama actual, rama remota (o `null` si la rama local no tiene
seguimiento, en cuyo caso la UI dice que el push la va a crear), commits de
adelanto/retraso respecto a esa remota, los cambios sin commitear y los
últimos 8 commits (`git log -8`, con el separador `\x1f` para no confundir un
espacio del mensaje con el separador de columnas).

Los cambios sin commitear marcan cuáles cuelgan de `ORBY_V4/` y cuáles no
(`delProyecto`): el repositorio de git es `scroll_wheel` entero (ver
«Repositorio de git» más abajo), y esta sección lo enseña también en la
UI — lo que no es de este proyecto sale atenuado, con una nota explicándolo.

**Commit y push** — `POST /api/subir-cambios {mensaje, bump}`. Con `bump`
opcional (`{componente: 'fw'|'tauri', tipo, nota}`), la secuencia
es: bump (si lo hay) → `git add -A` limitado a la carpeta de `ORBY_V4` →
commit → `git push -u origin <rama actual>`. Dos motivos para que el bump
vaya en el mismo paso que el commit y no sea otro botón aparte:

- El `git add -A` acotado a `ORBY_V4` importa: el repositorio es
  `scroll_wheel` entero, y un `git add -A` a secas metería en un commit de
  este proyecto lo que hubiera a medias en cualquier subproyecto hermano.
- Subir la versión y olvidarse de commitearla deja el repositorio anunciando
  una versión que no está publicada en ningún sitio; hacerlo en el mismo paso
  que el commit es la manera de que no se pueda olvidar.

El mensaje de commit vacío es un `400`, no un `500` (ver «API» más abajo).

### Releases

Tabla con las últimas releases de GitHub (`GET /repos/jaestefaniah27/scroll_wheel/releases`,
20 más recientes): etiqueta, fecha, si es borrador o prerelease, sus assets y un
veredicto de «completada» por tipo de etiqueta (ver más abajo). Mientras hay un
pipeline de release corriendo, la tabla se autorrefresca cada 15 s.

## Bump de versión

Solo edita ficheros, con escrituras quirúrgicas (regex sobre la línea concreta, no
reserializaciones): reescribir `package.json` o `tauri.conf.json` con
`JSON.stringify` reordena claves y destroza el diff que hay que revisar después.
**No hace commit ni tag** (para eso está «Commit y push» en la sección Git, con
bump opcional incluido). Al terminar, la respuesta trae el `git diff` de
exactamente los ficheros tocados —no un `git diff` a secas, ver la sección de
«Repositorio de git» de más abajo— para revisarlo antes de commitear. Como el
resto de tareas largas, comparte el candado global: no se puede lanzar mientras
hay algo más corriendo.

**Firmware** — `major` o `minor` (el firmware no tiene patch):
- `include/orby_version.h` (`ORBY_FW_MAJOR`/`ORBY_FW_MINOR`)
- `OrbyGUI/src/compat.js` (`FW_RECOMMENDED`)
- `docs/COMPATIBILIDAD.md` — inserta una fila nueva justo bajo la cabecera de la
  tabla, entre los marcadores `<!-- tabla:inicio -->` y `<!-- tabla:fin -->`; el
  texto de «qué trajo» lo escribe quien pide el bump, en un campo del formulario.

**App** — `major`, `minor` o `patch`, los tres ficheros a la vez o ninguno.
**A una preversión no se le suma: se gradúa.** `1.0.0-alpha` con `patch` da `1.0.0`,
que es lo que semver dice que viene después; con `minor` da `1.1.0` y con `major`,
`2.0.0`. En los tres casos la etiqueta desaparece, porque el panel solo publica
releases definitivas. Antes esto ni siquiera se intentaba: el bump se negaba con «no
tiene forma x.y.z» y **no había forma de publicar nada** estando en `1.0.0-alpha`, que
era justo la versión que había puesta.

Los ficheros:
- `OrbyGUI/src-tauri/tauri.conf.json` → `version`
- `OrbyGUI/src-tauri/Cargo.toml` → `version` del paquete, y solo dentro del bloque
  `[package]` (más abajo en el mismo fichero hay una `version` por cada dependencia,
  y tocar una de esas rompe el build)
- `OrbyGUI/package.json` → `version`

Uno acaba en el instalador, otro compila el binario y el tercero es el del
frontend, así que si se separan el bump ya no sabría a cuál hacerle caso: si al
empezar el bump los tres no coinciden entre sí, se niega y pide igualarlos a
mano en vez de elegir por su cuenta cuál era el bueno. El mismo desfase lo
avisa el semáforo de Versiones.

`Cargo.lock` no se toca a propósito: cargo lo pone al día solo en el siguiente build,
y editarlo a mano es la vía rápida a un lock incoherente con lo que de verdad se
compiló.

Los ficheros de cada bump (tres para firmware, tres para la app) se validan y
calculan antes de escribir ninguno: si un
patrón no encaja, no se toca nada. Un bump a medias dejaría el repositorio con
dos números de versión distintos y sin forma fácil de saber cuál manda.

## Publicar una release

Rutas de `tareas.js`, `pipelineFirmware` y `pipelineApp`. Si un paso falla en una
release de verdad, el pipeline **para**, marca ese paso en rojo y no deshace nada: un
tag ya empujado o una release a medias se arreglan a mano, con el log completo
delante.

### Firmware (`fw-vX.Y`), 7 pasos

1. **Comprobaciones** — `GH_TOKEN` presente, árbol de git limpio, rama `main`, la
   etiqueta `fw-vX.Y` no existe ya en local, `FW_RECOMMENDED` de `compat.js` coincide
   con la versión de `orby_version.h`, y `docs/COMPATIBILIDAD.md` tiene fila para esa
   versión. Es la comprobación «etiqueta y `ORBY_FW_MINOR` dicen lo mismo» que hacía
   el workflow muerto de GitHub Actions, más las dos desincronizaciones que ese
   workflow nunca miró.
2. **Compilar** — `cmake -B build -G Ninja` si no hay caché válida (misma detección de
   caché generada desde otra ruta que usa `flash.ps1`), luego `cmake --build build`.
3. **Empaquetar** — copia `build/ORBY_V4.uf2` a `build/ORBY_V4-fw-X.Y.uf2` y escribe su
   `.sha256` en formato `sha256sum` (huella, dos espacios, nombre).
4. **Etiquetar y empujar** — `git tag -a fw-vX.Y`, `git push origin main`,
   `git push origin fw-vX.Y`.
5. **Publicar la release** — `POST /repos/jaestefaniah27/scroll_wheel/releases` con
   **`prerelease: true`**. No es un parámetro que se pueda desmarcar en la UI: va
   escrito en el código porque `releases/latest` de GitHub devuelve la más reciente
   sin mirar la etiqueta, y el actualizador de la app pregunta justo por ahí. Una
   release de firmware que no sea prerelease deja a todas las copias instaladas de
   OrbyGUI buscando un `latest.json` que no existe en esa release y sin poder
   actualizarse —ya pasó una vez, es el fallo documentado en COMPATIBILIDAD.md.
   El workflow `firmware.yml` publica con `--prerelease` por lo mismo.
6. **Subir los ficheros** — el `.uf2` y su `.sha256` al `upload_url` de la release, con
   progreso por bytes leídos del fichero (no por bytes confirmados por GitHub, que
   Node no expone sin esperar a que termine la subida entera).
7. **Verificar** — relee la release por su etiqueta y comprueba que el `.uf2` está y
   que su tamaño coincide con el local (una subida cortada deja un asset del tamaño
   que sea, no del que debería).

### App (`vX.Y.Z`), 6 pasos

1. **Comprobaciones** — `GH_TOKEN` presente, **`TAURI_SIGNING_PRIVATE_KEY` presente**,
   árbol de git limpio, rama `main`, y que la etiqueta `vX.Y.Z` no exista ya como
   release en GitHub. Lo de la clave se comprueba antes de compilar a propósito: sin
   ella `tauri build` termina bien pero no firma ni genera el `latest.json`, y sale
   un `.exe` perfecto con una release que ningún actualizador va a ver.
2. **Compilar el instalador** — `npm run tauri:build`. Se niega si hay un OrbyGUI
   lanzado desde el panel: Cargo no puede sobrescribir su propio `.exe` y el error
   (`os error 32`) no dice cuál es el problema.
3. **Reunir los ficheros** — el `*-setup.exe` más reciente de
   `src-tauri/target/release/bundle/nsis/`, su `.sig` y el `latest.json`. Si falta
   alguno de los dos últimos, se corta: los tres van juntos o la release no sirve.
   Calcula además el SHA256 y las notas ya formateadas (con el aviso de SmartScreen)
   para el botón «Copiar notas».
4. **Crear la release** — `POST /repos/.../releases` con `prerelease: false`, al revés
   que el firmware: esta *es* la que tiene que salir por `releases/latest`.
5. **Subir los ficheros** — los tres al `upload_url`, con progreso por bytes.
6. **Verificar** — relee la release por su etiqueta y aplica el mismo veredicto de
   «completada» que la tabla de Releases, que ahora exige `.exe`, `.exe.sig` y
   `latest.json`.

## El ensayo en seco

Checkbox «ensayo en seco» → `POST /api/release` con `{"dry": true}`. La regla exacta
que implementa `tareas.js` (funciones `ensayado` y la clase `Pipeline`):

- **Nada que compile, escriba o empuje se ejecuta de verdad.** Ni `cmake`, ni
  `npm run release`, ni `git tag`/`git push`, ni las llamadas de escritura a GitHub
  (crear la release, subir assets). Esos pasos, en ensayo, solo registran
  `[ensayo] no se ejecuta: <lo que habría hecho>` y siguen.
- **Las lecturas sí se hacen de verdad.** `git status --porcelain`, `git rev-parse`,
  `git tag -l`, listar y leer releases de GitHub, calcular la huella SHA256 de un
  `.exe` que ya esté en `OrbyGUI/release/` de un build anterior: todo eso corre igual
  con o sin ensayo, porque es lo único capaz de avisar de un problema real antes de
  publicar. El paso de comprobaciones (el primero de cada pipeline) es puramente
  lectura y por eso corre siempre completo, acumulando todos los problemas que
  encuentre en vez de cortar en el primero, para enseñarlos todos juntos de una vez
  en vez de uno por intento.
- **Un paso que falla en ensayo no corta el pipeline.** Se marca como «aviso»
  (no como error) y se sigue con el siguiente paso, precisamente para poder ver la
  secuencia entera de una sentada. En una release de verdad, en cambio, el primer
  fallo para el pipeline sin más: seguir después de un paso roto solo serviría para
  dejar el repositorio a medias.
- El paso «Verificar» del firmware, en ensayo, comprueba si la etiqueta **ya existe**
  en GitHub de un intento anterior; si existe, lo marca como fallo del ensayo («la
  release YA existe: publicarla de verdad fallaría»), justo el tipo de aviso que este
  modo existe para dar.

## Repositorio de git

El repositorio de git no es `ORBY_V4/`: es `scroll_wheel` entero, tres niveles por
encima. Todas las comprobaciones de árbol limpio, rama y etiquetas (`git status`,
`git rev-parse --abbrev-ref HEAD`, `git tag -l`) operan sobre ese repositorio
completo, así que «árbol limpio» exige que no haya cambios sin commitear en
*ningún* subproyecto hermano de ORBY_V4, no solo en este. Por el mismo motivo, el
`git diff` que enseña el bump se pide **solo de los ficheros que ha tocado el bump**
(`git diff -- <ficheros>`), no un `git diff` a secas: de lo contrario arrastraría
cualquier cambio ajeno que hubiera en el resto del repositorio.

Por la misma razón, el commit de la sección Git (`git add -A -- <ruta absoluta
de ORBY_V4>`) se pide **solo de la carpeta ORBY_V4** y no del repositorio
entero: sin ese acotamiento, un commit hecho desde este panel se llevaría por
delante cualquier cambio a medias que hubiera en un subproyecto hermano.

## API

| Ruta | Qué hace |
|---|---|
| `GET /api/estado` | versiones (`firmwareRepo`, `appTauri`, `appTauriCargo`, `appPkg`, `fwRecomendado`, `appInstalada`, teclado), proceso lanzado, tarea en curso, lista de herramientas, si hay `GH_TOKEN` |
| `GET /api/eventos` | SSE: `estado`, `log`, `paso`, `progreso`, `fin` |
| `POST /api/lanzar` | `{modo}` — uno de `tauri-dev`, `tauri-build`, `web` |
| `POST /api/parar` | mata el árbol del proceso lanzado |
| `POST /api/bump` | `{componente:'fw'\|'tauri', tipo, nota}` → edita ficheros y devuelve `{anterior, nueva, ficheros, diff}` |
| `POST /api/release` | `{componente:'fw'\|'app', dry}` → arranca el pipeline; responde `202` de inmediato y el progreso va por SSE |
| `POST /api/flashear` | arranca el pipeline de flasheo del teclado (sin `dry`: no tiene ensayo en seco) |
| `POST /api/herramienta` | `{clave}` — una de las claves de `HERRAMIENTAS` en `tareas.js` |
| `GET /api/git` | rama, remota, adelanto/retraso, cambios sin commitear, últimos 8 commits |
| `POST /api/subir-cambios` | `{mensaje, bump}` — commit y push, con bump opcional delante |
| `GET /api/releases` | últimas 20 releases de GitHub con su veredicto |

`componente` en el bump y en la release **no es el mismo conjunto de
valores**: el bump usa `fw`/`tauri` y la release `fw`/`app`. Nombran lo mismo
—el firmware y la app— y la discrepancia es histórica, de cuando había dos
vías de escritorio con número propio.

Los errores de validación (mensaje de commit vacío, herramienta o componente
que no existe) devuelven **400**, no 500. Un fallo dentro de un pipeline ya
arrancado no pasa por aquí: se ve en el panel de tarea y en el log, no como
respuesta HTTP, porque la petición que lo lanzó ya respondió `202` antes de
que el pipeline corriera.

## Trampas conocidas

- **Con OrbyGUI de escritorio abierta, el puerto COM está cogido.** La tarjeta del
  teclado lo dice como aviso, no como fallo: es el comportamiento esperado de
  Windows (solo un proceso puede tener el puerto abierto a la vez), no un problema
  del panel.
- **OneDrive puede bloquear ficheros durante el build.** El pipeline no lo detecta
  ni lo repara: enseña el error de `npm run tauri:build` tal cual salga, en el log.
- **`tauri build` falla si hay un OrbyGUI corriendo**, con un `os error 32` que no
  explica nada. El pipeline lo comprueba antes, pero solo sabe de los procesos que
  ha lanzado él: una app abierta a mano o dejada en la bandeja se le escapa.
- **Un `git push` que se queda esperando credenciales.** Los comandos de git tienen
  un tope de 120 s; si se cortan por eso, el mensaje lo dice explícitamente y sugiere
  ejecutar el `git push` una vez a mano en una consola, para que Windows guarde las
  credenciales y el pipeline no vuelva a tropezar con lo mismo.

## Comprobado a mano

- Las tarjetas de Versiones leen fw 4.5, app 0.5.1/0.5.1/0.5.1, instalada 0.5.2
  y el teclado real en COM7 con la 4.6. (Medido antes de retirar Electron: la
  tarjeta de la app enseñaba entonces dos vías separadas.)
- `GET /api/git` devuelve rama, adelanto y cambios correctos.
- La herramienta `git-fetch` corre entera por el pipeline y termina en «hecho».
- El bump de la app toca exactamente `tauri.conf.json`, `Cargo.toml` y
  `package.json`, revertido después.
- Los errores de validación devuelven 400.
- Lo que ya estaba comprobado de antes sigue en pie: los bumps, lanzar/parar,
  el ensayo en seco y la tabla de releases.

**Sin comprobar todavía**: el flasheo real desde el panel. En la rama de
trabajo el repositorio va por la 4.5 y el teclado lleva la 4.6, así que
probarlo ahí le habría bajado la versión al teclado.

**La release de firmware de verdad no se ha ejecutado nunca**: se hará cuando toque
publicar una versión real.
