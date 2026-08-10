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
| Hasta dónde llega la app | [`OrbyGUI/src/compat.js`](../OrbyGUI/src/compat.js) — `FW_MIN`, `FW_RECOMMENDED` y la tabla `FEATURES` |
| Versión de la app | `OrbyGUI/package.json` |

Al subir la versión del firmware hay que tocar los tres sitios en el mismo
commit, más este documento.

<!-- tabla:inicio -->
| Firmware | Qué trajo | OrbyGUI |
|---|---|---|
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
- **Las banderas del handshake** (`MACROS=1`, `HASH=1`, `MAXMACROS=64`,
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
# 2. Actualizar FW_RECOMMENDED y FEATURES en OrbyGUI/src/compat.js
# 3. Añadir la fila de la tabla de arriba
# 4. Commit y etiqueta
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
repositorio, se queda con la más alta que no pase de `FW_RECOMMENDED` (nunca
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
