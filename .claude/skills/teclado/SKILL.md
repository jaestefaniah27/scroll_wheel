---
name: teclado
description: Habla con el teclado ORBY conectado al PC — leer qué firmware lleva, flashearle un .uf2, ponerlo en modo de carga (RP2 Boot / RPI-RP2) o averiguar en qué puerto COM está. Úsala cuando pidan flashear, cargar el firmware en la Pico, saber la versión del teclado, o cuando aparezca RP2 Boot / RPI-RP2.
---

# Hablar con el teclado ORBY

El teclado es una Raspberry Pi Pico (RP2040) con firmware propio. En el PC aparece de dos formas:

| Estado | Cómo lo ve Windows | Para qué |
|---|---|---|
| Normal | Teclado HID + puerto serie **COMx**, VID `CAFE`, PID variable (`4007`, `4009`…) | uso diario; la app habla por el COM |
| Modo de carga | **RP2 Boot**: VID `2E8A`, PID `0003`, más una unidad **RPI-RP2** con `INFO_UF2.TXT` | recibir un `.uf2` |

El modo de carga es el cargador de la ROM de la Pico: no se instala ni se configura nada, Windows
solo lo registra la primera vez que aparece. Se entra:
- mandando `BOOTSEL` por el COM (firmware 4.2 o posterior), como hacen la app al actualizar y
  `flash.ps1`;
- o enchufando el cable con el botón BOOTSEL pulsado.

Al copiar el `.uf2`, la Pico se reinicia sola y vuelve a modo normal.

## Script: `teclado.ps1` (en esta carpeta)

```powershell
$S = "D:\ORVY\scroll_wheel\.claude\skills\teclado\teclado.ps1"
powershell -ExecutionPolicy Bypass -File $S -Version                  # ORBY_V4:FW=4.31:KEYS=...
powershell -ExecutionPolicy Bypass -File $S -Flashear <ruta\al.uf2>   # BOOTSEL, copia y lee la versión
```

- El puerto lo busca en el registro, entre los VID_CAFE con `PortName`, y solo se queda con los
  que existen ahora.
  - **`Get-CimInstance Win32_PnPEntity` falla en este PC** («Clase no válida»).
  - Por eso la búsqueda automática de `ORBY_V4/flash.ps1` no encuentra el teclado y pide pulsar
    BOOTSEL a mano.
- La versión se lee mandando `ACK\n`. La respuesta empieza por
  `ORBY_V4:FW=X.Y:KEYS=..:OLEDS=..:ENCODERS=..:MODE=..`. Se reintenta una vez, porque el primer
  ACK tras enchufar se pierde a menudo.

## Flujo para probar un firmware recién compilado

1. Compilar: ver la sección 2 de la skill `publicar`, o `ORBY_V4/flash.ps1`, que compila y flashea.
   Sale `ORBY_V4/build/ORBY_V4.uf2`.
2. Comprobar la versión dentro del binario:
   `grep -a -o "ORBY_V4:FW=[0-9.]*" ORBY_V4/build/ORBY_V4.elf`.
3. **Cerrar Orvy**, también desde el icono de la bandeja: tiene el COM cogido.
4. `teclado.ps1 -Flashear ORBY_V4\build\ORBY_V4.uf2`.
5. Volver a abrir Orvy.

Para el usuario final no hace falta nada de esto: la app ofrece el firmware publicado en
**Ajustes → Firmware**.

## Antes de flashear

- **Pide confirmación al usuario.** Flashear reinicia el teclado y corta lo que esté haciendo.
- Un firmware que necesita una app más nueva que la instalada deja funciones a medias: mira el
  `minApp` de su versión en `OrbyGUI/src/compat.json`.
- **Si se queda en RPI-RP2 y no arranca, no es grave:** se copia otro `.uf2` (por ejemplo el
  de la última release `fw-v*`) y vuelve. El cargador está en ROM y no se puede romper.

## Trampas

- **«Acceso denegado» al abrir el COM:** otro programa lo tiene abierto (Orvy, un monitor serie,
  otra sesión). Solo puede abrirlo un programa a la vez.
- **El número de COM cambia** cuando cambia el PID del firmware. Por ejemplo, PID 4007 era COM5
  y 4009 es COM6. No lo des por fijo.
- **Sin ningún COM** (`[System.IO.Ports.SerialPort]::GetPortNames()` vacío): el teclado está
  desenchufado o se ha quedado en modo de carga. Mira si hay una unidad con `INFO_UF2.TXT`.
- **Comandos de juego, OLED y demás:** el protocolo serie completo está en
  `ORBY_V4/main.cpp`, en la sección CALLBACKS CDC, y del lado de la app en
  `OrbyGUI/electron/serial.js`. No mandes `GAME:1` ni comandos que escriben en Flash sin saber
  qué hacen.
