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
commit, más este documento. Es lo que comprueba el workflow de publicación:
si la etiqueta `fw-vX.Y` no coincide con `orby_version.h`, no compila.

<!-- tabla:inicio -->
| Firmware | Qué trajo | OrbyGUI |
|---|---|---|
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

```powershell
# 1. Subir ORBY_FW_MINOR (o MAJOR) en include/orby_version.h
# 2. Actualizar FW_RECOMMENDED y FEATURES en OrbyGUI/src/compat.js
# 3. Añadir la fila de arriba
# 4. Commit, y entonces:
git tag fw-v4.2
git push origin fw-v4.2
```

El workflow [`.github/workflows/firmware.yml`](../../../../.github/workflows/firmware.yml)
compila el `.uf2` en limpio, comprueba que la etiqueta y el código dicen lo
mismo, publica la release con el SHA-256 y pega esa tabla en las notas.

Para probar que compila sin publicar nada: *Actions → Publicar firmware de ORBY
→ Run workflow*. Deja el `.uf2` como artefacto y no crea release.

## Actualizar desde la app

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

Con el `.uf2` de la release, o con el que sale de `.\flash.ps1` en local:

1. Conectar el teclado con **BOOTSEL** pulsado.
2. Copiar el `.uf2` a la unidad `RPI-RP2`.
3. Se reinicia solo.

La configuración vive en la Flash del teclado y **sobrevive al reflasheo**, con
una excepción: la Flash de secuencias sí se queda vacía. La app las reenvía sola
al conectar (`syncAllMacrosToDevice`).
