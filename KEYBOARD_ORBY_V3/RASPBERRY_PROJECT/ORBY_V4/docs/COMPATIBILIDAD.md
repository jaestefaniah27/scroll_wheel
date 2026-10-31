# Compatibilidad entre firmware y OrbyGUI

El teclado y la app se publican por separado y se actualizan cuando le toca a
cada uno, así que en cualquier momento hay gente con combinaciones distintas.
Esta es la tabla de qué funciona con qué.

La regla de fondo: **la app degrada, no rompe**. Con un firmware anterior al que
conoce, OrbyGUI quita las funciones que ese teclado no tiene y lo dice al
conectar. Con un firmware posterior, sigue funcionando lo de siempre y las
novedades no se ven hasta actualizar la app.

## Quién guarda cada número

| Dato | Sitio |
|---|---|
| Versión del firmware | [`include/orby_version.h`](../include/orby_version.h) — de ahí salen el `FW=` del handshake y el `bcdDevice` del USB |
| Qué firmware ofrece cada app | [`OrbyGUI/src/compat.json`](../OrbyGUI/src/compat.json) — por firmware, la app mínima que lo aprovecha (`minApp`). **La app lo baja de GitHub al arrancar** |
| Qué trajo cada firmware | [`OrbyGUI/src/compat.js`](../OrbyGUI/src/compat.js) — la tabla `FEATURES`, solo para funciones que la app tiene que saber usar |
| Versión de la app | `OrbyGUI/package.json` |

Al subir la versión del firmware hay que tocar `orby_version.h` y `compat.json`
en el mismo commit, más este documento. `compat.js` solo si el firmware trae una
función nueva.

## Firmware y app se publican por separado

La app no lleva compilado hasta qué firmware ofrece: lo lee de `compat.json`, que
descarga de `raw.githubusercontent.com` (rama `main`) al arrancar. Si no hay red
usa la última copia guardada o, en su defecto, la que trae compilada; entre
varias gana la de `rev` más alto.

```json
{ "schema": 1, "rev": 7, "fwMin": "2.0",
  "firmware": { "4.19": { "minApp": "1.1.3" }, "4.20": { "minApp": "1.1.3" } } }
```

Cada app ofrece hasta el firmware más alto cuyo `minApp` ya cumple. Dos casos:

- **El firmware no cambia nada que la app use** (un arreglo, un ajuste de
  fábrica): se pone `minApp` igual al de la anterior, se sube `rev`, se sube
  `compat.json` a `main` y se publica el firmware. Las apps ya instaladas lo
  ofrecen sin actualizarse.
- **El firmware trae una función que la app tiene que saber usar**: se añade a
  `FEATURES`, `minApp` es la versión de app que la estrena, y esa app **hay que
  publicarla antes**. Hasta entonces las apps viejas no lo ofrecen y, si el
  teclado ya lo lleva, avisan de que la app es vieja.

`OrbyGUI/test/compat-tabla.test.mjs` hace de puerta: falla si el firmware de
`orby_version.h` no tiene entrada en `compat.json`, o si su `minApp` es mayor que
la versión de `package.json` (firmware que pide una app sin publicar).

**Orden al publicar:** primero `compat.json` en `main`, después la release de
firmware. Al revés, la app vería el `.uf2` y no la tabla, y no lo ofrecería.

Esto solo funciona desde OrbyGUI 1.1.4: las anteriores llevan el tope
compilado y siguen necesitando app nueva.

<!-- tabla:inicio -->
| Firmware | Qué trajo | OrbyGUI |
|---|---|---|
| **4.22** | **La intro de arranque lleva la marca nueva.** El dial de la pantalla central da una vuelta, se encienden las otras nueve de dentro afuera, giran las diez a la vez y la central se queda con el logotipo. Sale de `tools/anim/intro_dial.py`, que rasteriza la geometría de `OrbyGUI/assets/brand/`; el reproductor (`oled_intro.h`) no cambia. Sin cambios de protocolo | Cualquiera |
| **4.24** | **El zoom de la rueda es un pellizco de touchpad, no Ctrl + rueda.** El teclado anuncia una segunda interfaz HID (`Orby V4 Touchpad`, Windows Precision Touchpad, PID 0x4009, los dos dedos en un solo informe) y el giro de una rueda o mando en ZOOM abre o cierra dos dedos simétricos, sin Ctrl pulsado: en escala logarítmica (cada clic, el mismo zoom), detrás de un muelle críticamente amortiguado (sin tirones) y, con la rueda magnética, en alta resolución. Si el host no activa el modo touchpad (Linux, macOS), cae al Ctrl + rueda de siempre. Sale de `include/ptp_out.h`; el blob de certificación es el que Microsoft publica para dispositivos sin firmar. Sin cambios de protocolo serie | Cualquiera |
| **4.21** | **Las secuencias con texto, como el texto suelto.** Con la app viva (`HOST_APP`), cualquier secuencia que lleve un paso de texto —no solo las de texto puro— la ejecuta el PC entera (`MACRO:<id>`), que pega el texto de golpe en vez de pulsarlo tecla a tecla. Además los pasos de tecla esperan a que la cola HID esté libre: antes, varias teclas seguidas o una tecla justo tras un texto perdían pulsaciones. Sin cambios de protocolo ni bandera nueva | Cualquier Orvy con la 4.20; sin app todo sigue igual |
| **4.20** | **El teclado escribe textos él solo.** Paso de secuencia nuevo `MSTEP_TEXT` (tipo 6) y bandera `MACROTEXT=1`: `a`, `b` y `gap` llevan cada uno una tecla (`(modificador << 8) \| tecla`, 0 = fin), tres por paso, y el reproductor las pulsa de una en una con la cola de salida vacía. La app traduce el texto a teclas con la distribución **española** (`src/web/text-keys.mjs`); lo que no se pueda traducir, o no quepa en 48 pasos (unos 144 caracteres), sigue escribiéndolo el PC. Con la app viva (`HOST_APP`) y una secuencia que es solo texto (la 4.21 lo amplía a cualquiera con texto), el teclado avisa por CDC (`MACRO:<id>`) y escribe el PC, que es más rápido; sin app teclea el teclado. Un firmware anterior salta el paso sin hacer nada, así que la app no sube texto sin la bandera. En la WebGUI la pestaña «Escribir texto» ya no está desactivada | Orvy 1.1.5 o posterior para usarlo; con una anterior la macro de texto sigue yendo por el PC |
| **4.19** | **Un solo dueño del menú.** La tecla 10 o un mando: con un mando de menú (`SET_MENUENC` ≠ 0) la tecla 10 queda libre, y volver a la tecla (`SET_MENUKEY:0`) suelta el mando. Con dueño, el «Abrir menú» que un perfil tenga en cualquier otro mando se ignora; las páginas (`ROT_PAGE`) valen en cualquiera. Al cargar de la Flash se corrige lo guardado por la 4.16-4.18, que permitían ambos a la vez. Sin comando ni bandera nueva | Orvy 1.1.3 o posterior; la app oculta «Abrir menú» en los demás mandos con la próxima versión |
| **4.18** | **Los mandos giran al revés por defecto.** El sentido de fábrica de los dos encoders pasa a ser el contrario al de antes (el que había que marcar con `SET_ROTINV`). Los bits de `ROTINV` siguen queriendo decir «al revés de lo normal», ahora sobre ese sentido nuevo: un teclado que tenía marcados los dos mandos (máscara 3) vuelve a girar como antes hasta que se desmarcan. La rueda y la telemetría `ENC:` no cambian. Sin cambios de protocolo ni bandera nueva | Orvy 1.1.4 o posterior para que la app lo ofrezca como actualización; con cualquiera que tenga `ROTINV` se usa igual |
| **4.17** | **Animación al elegir perfil en el menú.** Al elegir un perfil en el menú físico, los iconos de las demás pantallas son succionados hacia la pantalla del perfil elegido y, cuando llegan todos, salen disparadas las imágenes del perfil nuevo, cada una hacia su pantalla. Es el tipo de transición `PICK` (9), del grupo de velocidad «perfil». Sin cambios de protocolo: con el espejo encendido (`MIRROR:1`), `EV:SCR:GO` lleva un campo más, `<origen>` (0-9, la pantalla del perfil elegido), que las apps anteriores ignoran. No hay bandera nueva | Orvy 1.1.3 o posterior para verla también en el Inicio; con una anterior el teclado la hace igual y el Inicio salta al destino sin animar |
| **4.16** | **Sentido de mandos y rueda, y el mando de menú.** `SET_ROTINV:<0..7>` (`ROTINV=1`) invierte el giro del encoder izquierdo (1), el derecho (2) y la rueda (4) para todo lo demás —acciones, menús y páginas—, sin tocar la telemetría `ENC:`/`WHEEL:`; es independiente de la inversión de la rueda por perfil. `SET_MENUENC:<0\|1\|2>` (`MENUENC=1`) deja un mando como menú: girarlo cambia de página y mantenerlo pulsado (250 ms) abre el selector de perfiles —con SUPER, el gestor de páginas—, se gira sin soltar hasta el elemento y **al soltar se elige**; un toque corto lo deja abierto hasta confirmar con otro clic. Lo que los perfiles tengan en ese mando se ignora. El mismo gesto sirve para cualquier clic con `ROT_MENU` (gestor de páginas, menú principal) y para los dos mandos pulsados 2 s (girando antes de soltar se entra en la opción). El gestor de páginas ya tiene cursor que mueven los mandos. **Con el menú abierto desde un mando, las teclas hacen lo que dice su página** en vez de ser atajos de selección. Ambos ajustes van en `GET_STATE` (`STATE:ROTINV`, `STATE:MENUENC`) y en bytes de relleno de los ajustes: no cambia el formato de la Flash | Orvy 1.1.3 o posterior para usar lo nuevo; con una anterior funciona como la 4.15 |
| **4.15** | **Los mandos llevan páginas y menús, y la tecla de menú se puede dejar libre** (`MENUKEY=1`). Dos tipos de acción de mando nuevos: `ROT_PAGE` (6, el giro pasa de página; como los desplazamientos, una acción cubre los dos sentidos) y `ROT_MENU` (7, el clic abre un menú: keycode 0 perfiles, 1 gestor de páginas, 2 menú principal). `SET_MENUKEY:<0\|1>` deja la tecla 10 como la de menú (0) o como una tecla normal (1); va en `GET_STATE` (`STATE:MENUKEY`) y se guarda en el byte de relleno de los ajustes, así que no cambia el formato de la Flash. Dentro de los menús navegan los dos mandos, no solo el izquierdo, y los dos mandos pulsados 2 s abren el menú principal desde cualquier menú | Orvy 1.1.3 o posterior para usar lo nuevo; con una anterior funciona como la 4.14 |
| **4.14** | Caben **255 secuencias** en el teclado en vez de 64 (`MAXMACROS=255`). La app no libera el id de una macro que ya no usa ninguna tecla, así que con el uso los ids pasaban de 63, y una secuencia rehecha a partir de ahí se quedaba tachada sin la app aunque solo llevara teclas, clics y esperas. Las secuencias se mudan en la Flash a 896-1024 KB; las de la 4.13 se traen solas al arrancar | Orvy 1.1.3 o posterior para que la app lo ofrezca como actualización; cualquiera para usarlo (la app lee el tope del handshake) |
| **4.13** | Sin cambios de protocolo. Con SUPER sujeta, la pulsación corta de la tecla de menú **retrocede** de página (sin SUPER sigue avanzando) y la larga abre el **menú principal** (ajustes) en vez del de perfiles. Mientras se mantiene el menú para cambiar de perfil, el espejo sigue mandando las pantallas: la app ya ve abrirse y cerrarse ese menú | Orvy 1.1.2 o posterior para que la app lo ofrezca como actualización; cualquiera para usarlo |
| **4.12** | `MIRROR:1`/`MIRROR:0` y bandera `MIRROR=1`: con el espejo encendido el teclado manda, en cada transición de las pantallas, el destino de las diez (`EV:SCR:<n>:<720 hex>`) y el tipo y ritmo de la animación (`EV:SCR:GO:<tipo>:<ritmo x1000>`), además del negativo de las teclas sujetas (`EV:INV:<máscara>`) y el reposo (`EV:DISP:<0/1>`). La app los anima con una copia en JS de `oled_anim.h` y el Inicio enseña las pantallas exactamente como las del teclado | Orvy 1.1.0 o posterior para verlo; con una app anterior no cambia nada (no pide el espejo) |
| **4.11** | Sin cambios de protocolo. El menú de perfiles que se abre manteniendo la tecla de menú ya no lleva la pantalla de Volver: soltar la tecla ya cancela. Desde el menú principal (Perfiles) sigue llevándola | Orvy 1.0.11 o posterior para que la app lo ofrezca como actualización; cualquiera para usarlo |
| **4.10** | La velocidad de las animaciones cuenta sobre un ritmo de fábrica propio de cada grupo (los valores que se eligieron con la 4.8/4.9 pasan a ser el 100%) y la bandera pasa a `ANIMSPEED=2`; `SET_ANIMSPEED` admite de 25 a 250. Abrir y cerrar menú con más zoom: al abrir, lo de antes se agranda y se va, y después lo nuevo crece desde un punto; al cerrar, el menú se encoge hasta un punto y la pantalla normal llega desde grande. La capa SUPER se abre y se cierra con esa misma animación | OrbyGUI 1.0.10 o posterior para elegir la velocidad con la escala nueva; cualquiera para lo demás |
| **4.9** | Sin cambios de protocolo. El carrusel de páginas respeta el hueco real entre pantallas (9,75 mm, 70 px a la escala del panel) en vez del comprimido de antes: un icono cruza de una tecla a la vecina a la misma velocidad que se mueve el resto | OrbyGUI 1.0.9 o posterior para que la app lo ofrezca como actualización; cualquiera para usarlo |
| **4.8** | Velocidad configurable de las transiciones animadas de las pantallas, por grupo: menú, carrusel de páginas, submenú y cambio de perfil llevan ahora un multiplicador (`SET_ANIMSPEED`/`GET_ANIMSPEED`, bandera `ANIMSPEED=1`). De fábrica salen más rápidas en todo salvo abrir/cerrar menú y el fundido, que ya iban bien | OrbyGUI 1.0.8 o posterior |
| **4.7** | Comandos `MSTORE_*` y bandera `MSTORE=1`: el teclado guarda una copia de lo que hace cada macro del PC (abrir una app, un texto, una grabación...), para que la app de otro ordenador la recupere al conectar. Pantallas rediseñadas (iconos, fuente con tildes, menús con icono y nombre) y transiciones animadas; iconos para los perfiles de la biblioteca | OrbyGUI 1.0.6 o posterior para recuperar macros; cualquiera para lo demás |
| **4.6** | Sin funciones nuevas. Arregla el antirrebote de las teclas, que era por bloqueo y daba por buena una pulsación entera a partir de dos picos de ruido: en la tecla de menú eso salía como páginas pasando solas sin parar. Añade `STATE:MENUGLITCH:<n>` a `GET_STATE`, el contador de pulsos descartados por demasiado cortos | Cualquiera |
| **4.5** | Comando `HOST_APP:<0\|1>` y bandera `HOSTAPP=1`: el teclado tacha con una barra diagonal las pantallas de las teclas que solo puede ejecutar la app de PC mientras esta no esté anunciada | OrbyGUI 0.4.x o posterior |
| **4.4** | Icono propio por perfil, visible en el menú de perfiles del teclado | OrbyGUI 0.4.x o posterior |
| **4.3** | Sin cambios de protocolo respecto a la 4.2. Es la primera versión que sale como release publicada, y por tanto la primera que se puede instalar desde la propia app | 0.4.1 o posterior |
| **4.2** | Comando `BOOTSEL`: el teclado se reinicia en el cargador de la ROM cuando la app se lo pide, así que se actualiza desde Ajustes sin desenchufar nada. El handshake anuncia `MAXMACROS` (cuántas secuencias caben) y `BOOTSEL=1` | 0.4.1 o posterior |
| **4.1** | `GET_HASH` y `GET_OLED_PG`: sincronización por huella y precarga de iconos de cualquier página. Identidad USB propia (número de serie único por teclado, `bcdDevice` real) | 0.4.0 o posterior |
| **4.0** | Páginas dentro de un perfil (`MAXPAGES`). Secuencias reproducidas por el propio teclado (`MACROS=1`, `SET_MACRO_STEP` y compañía) | 0.4.0 o posterior |
| **3.0** | Crear, duplicar y borrar perfiles. Mandos en la capa SUPER. Rueda ajustable por perfil (`SET_PSCROLL`) | 0.3.0 o posterior |
| **2.0** | `GET_STATE` y `GET_PROFILE`: la app puede leer la configuración del teclado. **Mínimo para usar el editor** | 0.2.0 o posterior |
| **1.0** | Solo HID y el handshake. La app no puede leer nada: **no se admite** | — |
<!-- tabla:fin -->

La app no exige la última: cualquier OrbyGUI reciente habla con cualquier
firmware de la 2.0 en adelante. La columna de la derecha dice desde cuándo la
app *aprovecha* esa versión de firmware, no desde cuándo se conecta.

## Cómo lo decide la app

Dos cosas distintas, y conviene no confundirlas:

- **La versión** (`FW=4.2`) dice qué firmware es. Sirve para el veredicto que se
  le enseña al usuario y para las funciones que no llevan bandera propia.
- **Las banderas del handshake** (`MACROS=1`, `HASH=1`, `MAXMACROS=255`,
  `MAXPAGES=…`) dicen qué trae *este* aparato. Cuando existe la bandera, manda
  ella: sobrevive a un firmware recompilado con otras opciones.

`compat.supports(info, 'macros')` aplica esa preferencia solo. Las vistas
preguntan por función, nunca por número de versión.

## Publicar una versión de firmware

> **GitHub Actions no se ejecuta en esta cuenta.** Los workflows del repositorio
> mueren al arrancar con `The job was not started because your account is locked
> due to a billing issue`. El de firmware
> ([`.github/workflows/firmware.yml`](../../../../.github/workflows/firmware.yml))
> se queda como documentación de los pasos y como red de seguridad por si algún
> día vuelve a funcionar, pero **hoy se publica a mano**.

```powershell
# 1. Subir ORBY_FW_MINOR (o MAJOR) en include/orby_version.h
# 2. Añadir la entrada a OrbyGUI/src/compat.json y subir su "rev"
#    (y FEATURES en compat.js solo si hay una función nueva)
# 3. Añadir la fila de la tabla de arriba
# 4. Commit, subir compat.json a main y etiquetar
git tag -a fw-v4.3 -m "Firmware 4.3"
git push origin main
git push origin fw-v4.3

# 5. Compilar y nombrar el .uf2 con su versión
cmake --build build
Copy-Item build\ORBY_V4.uf2 build\ORBY_V4-fw-4.3.uf2
Get-FileHash build\ORBY_V4-fw-4.3.uf2 -Algorithm SHA256

# 6. Crear la release y subir el .uf2 (con gh, o por la web)
#    --prerelease NO es opcional: ver abajo.
gh release create fw-v4.3 --title "Firmware 4.3" --notes-file notas.md --prerelease `
   build\ORBY_V4-fw-4.3.uf2 build\ORBY_V4-fw-4.3.uf2.sha256
```

La app solo exige dos cosas de la release: que la etiqueta empiece por `fw-v` y
que haya un asset `.uf2`. Todo lo demás son notas para quien las lea.

### Por qué las releases de firmware van como prerelease

Porque comparten repositorio con las de la app, y `releases/latest` de GitHub
devuelve la más reciente **sin mirar la etiqueta**. electron-updater pregunta por
ahí: publicada la `fw-v4.4` dos minutos después de la `v0.5.0`, todas las copias
instaladas de OrbyGUI se quedaron mostrando

> `Error: Cannot find latest.yml in the latest release artifacts (…/fw-v4.4/latest.yml)`

y sin poder actualizarse. Una prerelease no cuenta para `releases/latest`, así que
el actualizador de la app vuelve a ver la suya. El instalador de firmware de la
app no se entera: lista `releases?per_page=50`, que las incluye.

Si alguna vez se publica un firmware **sin** `--prerelease`, se arregla marcándolo
después; no hace falta rehacer la release.

Desde OrbyGUI 1.0.2 hay un motivo más, y más grave: la app **se actualiza sola**
(`tauri-plugin-updater`) pidiendo el `latest.json` de `releases/latest`. Una
release de firmware publicada sin `--prerelease` pasa a ser la última, no lleva
ese fichero, y todo el parque instalado se queda sin actualizarse **sin decir
nada**. El actualizador detecta ese caso concreto y lo nombra en el mensaje de
error, pero solo lo ve quien abra Ajustes.

### Y por qué las de la app NO van como prerelease, aunque el número diga «alpha»

La `v1.0.0-alpha` se publicó como release normal a propósito. La bandera de
GitHub y el número de versión son cosas distintas: marcarla como prerelease la
sacaría de `releases/latest`, que es justo donde mira el avisador de la app
(`OrbyGUI/src-tauri/src/updater.rs`), y nadie se enteraría de que existe. Lo de
«alpha» va en el número, no en la bandera.

> **Trampa de los sufijos.** La comparación de versiones
> (`orby_core::fw::compare_fw`) parte por puntos y lee cada trozo como número, así
> que de `1.0.0-alpha` saca `[1, 0, 0]`: **`1.0.0-alpha`, `1.0.0-beta` y `1.0.0`
> le parecen la misma versión.** Eso está bien para lo que se escribió —firmware,
> donde `4.10` tiene que ser posterior a `4.9`— pero significa que quien tenga
> puesta la `1.0.0-alpha` **no verá** una `1.0.0` final: hay que subir la parte
> numérica (`1.0.1`, `1.1.0`…). Si algún día hacen falta sufijos de verdad, el
> sitio donde tocarlo es `compare_fw`, y hay que tener en cuenta que lo comparte
> con las versiones de firmware.

**Comprobar a mano lo que comprobaba el workflow:** que la etiqueta y
`ORBY_FW_MINOR` dicen lo mismo. Nadie lo valida ya, y la app decide qué comandos
manda mirando ese número: si el `.uf2` de `fw-v4.3` se presenta como 4.2, la app
se cree la versión que anuncia el teclado y no la de la etiqueta, así que el
único perjudicado es quien lea la lista de releases. Al revés es peor.

## Actualizar desde la app

**Funciona, probado sobre el teclado real.** Con un Orby en la 4.2 y la 4.3
publicada como release, OrbyGUI detectó la versión nueva al conectar, la
descargó y la instaló entera **sin tocar el botón BOOTSEL**. Es el camino
recomendado: `flash.ps1` solo hace falta para probar una compilación local.

**Ajustes → Firmware del teclado.** La app mira las releases `fw-v*` del
repositorio, se queda con la más alta que no pase de `fwRecomendado()` (nunca
ofrece un firmware que ella no sabría manejar), la descarga y la instala.

Por dentro, en [`OrbyGUI/electron/firmware.js`](../OrbyGUI/electron/firmware.js):

1. Descarga el `.uf2` a un temporal y **comprueba el tamaño**. Una descarga
   cortada sigue siendo un fichero válido para el cargador: copiaría medio
   firmware y dejaría el teclado sin arrancar.
2. Manda `BOOTSEL` por el CDC. El teclado contesta `BOOTSEL:OK` y se reinicia en
   el cargador de la ROM. Con un firmware anterior al 4.2 no existe ese comando:
   la app pide al usuario que enchufe el cable con el botón pulsado y espera
   igual.
3. Busca la unidad del cargador por el `INFO_UF2.TXT` que la Pico deja en la
   raíz — no por la letra, que la asigna Windows y cambia, ni por la etiqueta,
   que obligaría a llamar a PowerShell.
4. Copia el `.uf2`. **El error al cerrar el fichero es lo normal**: la Pico se
   reinicia en cuanto recibe el último bloque y la unidad desaparece bajo los
   pies de Windows. Lo que delata un fallo de verdad es que la unidad siga ahí.

La app **no guarda por su cuenta** antes de actualizar: si hay cambios sin
escribir en Flash, se niega y lo dice. Guardar por sorpresa escribiría lo que
hubiera en RAM, variaciones por aplicación incluidas.

## Instalar el firmware a mano

`.\flash.ps1` compila y flashea sin que haya que pulsar nada: manda el mismo
comando `BOOTSEL` que la app (busca el puerto por `VID_CAFE`) y copia el `.uf2`
en cuanto sale la unidad. **Cierra OrbyGUI antes**, o tendrá el puerto cogido y
el script caerá al método manual.

Ese método manual sigue ahí para un firmware anterior al 4.2, o para un teclado
que se quedó a medias:

1. Conectar el teclado con **BOOTSEL** pulsado.
2. Copiar el `.uf2` a la unidad `RPI-RP2`.
3. Se reinicia solo.

La configuración vive en la Flash del teclado y **sobrevive al reflasheo**, con
una excepción: la Flash de secuencias sí se queda vacía. La app las reenvía sola
al conectar (`syncAllMacrosToDevice`).
