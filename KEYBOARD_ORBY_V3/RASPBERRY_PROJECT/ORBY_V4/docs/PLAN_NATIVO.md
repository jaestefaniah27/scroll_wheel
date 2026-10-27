# Plan: hacer nativas en el teclado las funciones que hoy ejecuta el PC

Estudio de qué parte de lo que hoy hace OrbyGUI en el PC se puede llevar al firmware,
cómo, y qué coste tiene. Punto de partida: la lista `PC_ONLY` de
`OrbyGUI/src/platform.js` y los pasos que `src-tauri/src/macros.rs` ejecuta cuando el
teclado manda `MACRO:<id>`.

## Dónde estamos

El firmware ya reproduce solo los pasos `delay`, `hotkey`, `mouse_move` y `mouse_click`
(`MacroStepType` en `main.cpp`, `DEVICE_STEP_TYPE` en `macros-store.js`). Una secuencia
con **cualquier** otro paso entera se queda en el camino del PC, y sin la app abierta la
tecla sale tachada en la OLED (`key_needs_app` + `host_app_alive`).

Lo que hoy necesita la app, y si puede dejar de necesitarla:

| Función | Hoy | ¿Nativa? | Cómo |
|---|---|---|---|
| Escribir texto (`text`) | PC: `SendInput` Unicode / pegar | **Sí** | La app compila el texto a pulsaciones HID con la distribución del usuario; el teclado las reproduce |
| Bloquear (`lock`) | PC: `LockWorkStation` | **Sí, ya** | Es `Win+L`: un `hotkey` normal |
| Suspender / apagar | PC: `rundll32` / `shutdown` | **Sí** | HID *System Control* (Sleep / Power Down) |
| Reiniciar / hibernar / cerrar sesión | PC: `shutdown /r /h /l` | **Sí** (con texto) | `Win+R` → `shutdown /r /t 0` → Intro |
| Despertar el PC | — no existe | **Sí, y solo nativo** | *System Wake Up* + *remote wakeup* USB |
| Posición absoluta del ratón | PC: `SetCursorPos` | **Sí** | Puntero HID **absoluto** (no el homing relativo aparcado) |
| Abrir app / archivo / URL | PC: `ShellExecuteW` | **Mayoría sí** | Teclas *AL* de consumo, `Win+1..9`, o `Win+R` + ruta |
| Reproducir grabación | PC: `SendInput` | **Sí** | Convertir eventos a pasos y guardarlos en un pool en Flash |
| Grabar ratón/teclado | PC: ganchos de bajo nivel | No | El teclado no ve el ratón ni el otro teclado |
| Complementos (HTTP) | PC | No (con este hardware) | Necesita red: Pico W / ESP32 al lado |
| Perfil por ventana activa | PC | No | Solo el PC sabe qué ventana tiene el foco |
| Autoarranque, actualizar app, ventana | PC | No | Son de la propia app |
| Actualizar firmware | PC | — | Ya existe `BOOTSEL`; no tiene sentido «nativo» |

Efecto colateral importante: **todo lo que pase a nativo funciona también en la WebGUI**,
porque solo hay que subirlo por serie. `text`, `power` y buena parte de `pcSequences`
saldrían de `PC_ONLY`.

---

## 1. Texto nativo (la pieza que desbloquea casi todo lo demás)

### El problema
El teclado manda *keycodes* HID (posiciones físicas), no caracteres. Qué carácter sale
lo decide la distribución de teclado que tenga Windows. `@` en ES es `AltGr+2`; en US es
`Shift+2`. El firmware no puede saber la distribución.

### La solución: compilar en la app, reproducir en el teclado
La app, al subir la secuencia, traduce el texto a una lista de pares
`(modificador, keycode)`:

- **Tauri**: con la distribución real del usuario, `VkKeyScanExW(c, GetKeyboardLayout(..))`
  da tecla + Shift/Ctrl/Alt para cada carácter; `MapVirtualKeyExW` pasa a scancode y de
  ahí a uso HID. Teclas muertas (`´`, `` ` ``, `^`, `¨`): `á` = `´` + `a`, se detectan con
  `ToUnicodeEx`. Esta lógica es pura una vez obtenidas las tablas → **`orby-core`**,
  con tests (tabla ES y US como datos).
- **WebGUI**: no hay API para la distribución completa (`navigator.keyboard.getLayoutMap()`
  solo da el carácter sin Shift). Tablas estáticas ES / US / ES-LATAM en JS y un selector
  en Ajustes. Compartir la tabla con `orby-core` (JSON generado) para que no diverjan.
- **Caracteres que no están en la distribución** (`€` en US, `ñ` en US, `«`…):
  `Alt` mantenido + código en el teclado numérico (`Alt+0241` = `ñ`), que cubre
  Windows‑1252 en casi todas las apps. Fuera de eso (emoji, `∑`, CJK) no hay vía
  nativa fiable → la secuencia sigue yendo por el PC (`macroDeviceEligible` = false).

### Firmware
- **Pool de texto en Flash.** Hay ~456 KB libres entre el final de las macros
  (`FLASH_MACROS_OFFSET + 32 KB` = 1592 KB) y los 2 MB. Una región de, p. ej.,
  64 KB = ~32 000 pulsaciones a 2 bytes cada una. No meterlo en `MacroStep`: con
  `MACRO_MAX_STEPS = 48` no cabe ni una frase.
- **Nuevo paso `MSTEP_TEXT`**: `a`/`b` = desplazamiento y longitud dentro del pool.
- **Codificación**: `[mod, keycode]` por pulsación; un bit del byte de modificador marca
  «no soltar el modificador» para poder hacer `Alt` mantenido + varios dígitos del
  teclado numérico.
- **Velocidad**: pulsar + soltar = 2 informes; con `bInterval = 1 ms` son ~500 car/s.
  Dejar un hueco configurable (2–4 ms) porque hay apps que se comen letras (el mismo
  motivo que `RETARDO_ENTRE_LETRAS_MS` en `macros.rs`).
- **LEDs del host**: hoy `tud_hid_set_report_cb` solo mira el *Feature* del ratón.
  Hay que leer el informe de **salida** del teclado (NumLock/CapsLock):
  - Con **Bloq Mayús** activo, `Shift+a` sale `a`: pulsar CapsLock antes y después.
  - Los códigos `Alt+numpad` exigen **NumLock** activo: igual.
- **Protocolo**: `TEXT_POOL_WRITE:<off>:<hex>` por trozos + `TEXT_POOL_COMMIT` (escritura
  a Flash una vez). Bandera nueva en el handshake (`TEXT=1`) → `compat.js`.

### Lo que se pierde frente al camino del PC
El PC **pega** por el portapapeles los textos de ≥5 caracteres. Escribir letra a letra
tiene un efecto que pegar no tiene: en editores con autocompletado (VS Code, IDEs) se
cierran solos paréntesis y comillas y los saltos de línea se autoindentan. Propuesta:
**si la app está viva, el texto lo sigue haciendo el PC; si no, el teclado.** Ver
«Política híbrida» abajo.

---

## 2. Energía

| Acción | Vía nativa | Notas |
|---|---|---|
| Bloquear | `Win+L` | Ya posible hoy como `hotkey`. Solo hay que hacer que «Bloquear» en la pestaña Multimedia guarde un `hotkey` en vez de un `system_power`. |
| Suspender | HID Generic Desktop `0x82` *System Sleep* | Windows lo obedece directamente. |
| Apagar | HID `0x81` *System Power Down* | Windows hace lo que diga «Al presionar el botón de inicio/apagado» (en sobremesa suele ser apagar, en portátil suspender). Avisarlo en la UI. |
| Despertar | HID `0x83` *System Wake Up* | **Algo que la app no puede hacer nunca.** Exige declarar *remote wakeup* en el descriptor de configuración (hoy `attribute = 0` en `TUD_CONFIG_DESCRIPTOR`) y llamar `tud_remote_wakeup()` con el bus suspendido. Además el usuario tiene que dejar «Permitir que este dispositivo reactive el equipo» activo (Windows suele activarlo solo para teclados). |
| Reiniciar / hibernar / cerrar sesión | `Win+R` → `shutdown /r /t 0` → Intro | Depende del texto nativo (§1). Es independiente del idioma de Windows, a diferencia de navegar el menú `Win+X`. Falla si una directiva de empresa bloquea Ejecutar. |

**Descriptor**: añadir una colección *System Control* con su `REPORT_ID` (1 byte, 3 bits
+ relleno). Cambiar el descriptor HID está bien porque Windows cachea por
VID/PID/`bcdDevice` y el `bcdDevice` ya cambia con cada versión de firmware.

---

## 3. Posición absoluta del ratón

El `TODO(homing-absoluto)` de `main.cpp` y `macros-store.js` está bloqueado porque con un
ratón **relativo** el resultado depende de la «velocidad del puntero» de Windows. La
salida limpia no es calibrar, es **añadir un puntero absoluto**:

- Segunda colección en el descriptor: *Generic Desktop / Pointer*, X e Y con
  `Logical 0..32767`, `Input (Data,Var,Abs)`, más los botones. Es lo que usan las
  tabletas USB de las máquinas virtuales.
- Windows no aplica velocidad ni aceleración a coordenadas absolutas: `(x, y)` llega
  exacto.
- La app convierte píxeles a 0..32767 **al subir la secuencia**, con los límites del
  escritorio virtual (`SM_XVIRTUALSCREEN`, `SM_CXVIRTUALSCREEN`…). Si luego cambia la
  disposición de monitores la posición se desplaza, igual que hoy con `SetCursorPos`.
- **A verificar con una prueba antes de construir encima**: que en Windows 10/11 el
  rango absoluto se reparta por todo el escritorio virtual y no solo por el monitor
  principal con varias pantallas. Si fuera solo el principal, se puede ofrecer igual
  limitado a ese monitor.
- Nuevo paso `MSTEP_ABS_MOVE`. `MSTEP_HOME_MOVE` y `MACRO_TEST` pasan a ser material
  muerto.

Esto mismo es la base de la reproducción nativa de grabaciones (§5).

---

## 4. Abrir aplicaciones

Tres niveles, de más robusto a menos:

1. **Teclas de consumo *AL*** (ya existe la tabla `get_consumer_key_from_index`):
   explorador (`0x194`), navegador (`0x196`), correo (`0x18A`), calculadora, búsqueda.
   Solo sirven para esas apps fijas.
   > **Revisar**: la calculadora está como `0x011A`; en la HID Usage Tables *AL
   > Calculator* es `0x0192`. Si en tu PC no abre la calculadora, es eso.
2. **`Win+1..9`**: abre (o enfoca, si ya está abierta) la app anclada en esa posición de
   la barra de tareas. Nativo hoy mismo como `hotkey`, instantáneo y sin depender del
   idioma. Merece un acceso directo en la UI: «App anclada n.º N».
3. **`Win+R` + destino + Intro** (necesita §1): vale para `.exe`, documentos, carpetas,
   URLs y apps de la Tienda con `shell:AppsFolder\<AUMID>` (el AUMID se puede sacar en
   `apps.rs`, que ya lista el menú Inicio). Rutas con espacios entre comillas. Hace
   falta una espera (~300 ms) a que aparezca el diálogo. Se ve el diálogo un instante.

Con la app abierta, `ShellExecuteW` sigue siendo mejor (sin diálogo visible, sin
depender del foco) → caso claro para la política híbrida.

---

## 5. Grabaciones

- **Grabar** no puede ser nativo: el teclado es un dispositivo, no ve lo que hacen el
  ratón ni el teclado del usuario. Sigue en `recorder.rs`.
- **Reproducir** sí. Al terminar la grabación, la app la convierte:
  - `kdown`/`kup` (código uiohook) → uso HID. Hacen falta pasos **mantener/soltar
    tecla** (`MSTEP_KEY_DOWN` / `MSTEP_KEY_UP`) en vez de la pulsación breve de
    `MSTEP_KEY`; `hid_out.h` ya lleva un estado de 6 teclas, así que encaja.
  - Movimiento → puntero absoluto (§3).
  - Botones abajo/arriba y rueda → informe de ratón actual.
  - Tiempos `t` → esperas.
- Una grabación son cientos o miles de eventos: van a un **pool** en Flash como el del
  texto (≈5–6 bytes por evento; 128 KB ≈ 20 000 eventos), referenciado desde un paso.
- La velocidad de reproducción (`velocidad` en `reproducir`) se puede aplicar en el
  firmware escalando las esperas.

---

## 6. Lo que se queda en el PC

- **Complementos**: son peticiones HTTP a otros aparatos. Sin red en la Pico no hay vía.
  Solo con hardware nuevo (Pico W / Pico 2 W, o un ESP32 por UART) y moviendo
  `orby_core::plugins` a C++. Proyecto aparte.
- **Perfil según la ventana activa** y variaciones por app: solo el PC sabe qué ventana
  tiene el foco.
- **Grabar**, autoarranque, actualizar la app, controles de ventana.

---

## Política híbrida: «nativo si no hay app»

El firmware ya sabe si la app está viva (`host_app_alive`). Propuesta para los pasos
donde el PC lo hace **mejor** (texto largo → pega; abrir app → sin diálogo):

- La app sube **siempre** la versión nativa si la secuencia es elegible.
- Una bandera por macro, `prefer_pc`: si está puesta **y** `host_app_alive()`, el teclado
  manda `MACRO:<id>` como hoy; si no, la reproduce él.
- La tecla deja de salir tachada sin app, porque ya tiene vía nativa.

Así no se empeora nada con la app abierta y todo sigue funcionando con ella cerrada, en
la pantalla de inicio de sesión, en otro PC o desde la WebGUI.

---

## Orden propuesto

| Fase | Qué | Firmware | App | Coste |
|---|---|---|---|---|
| 0 | `Bloquear` como `Win+L`; acceso «App anclada N» (`Win+N`); revisar el uso de la calculadora; paso `MSTEP_CONSUMER` para meter multimedia dentro de secuencias | poco | poco | **horas** |
| 1 | Texto nativo: pool, `MSTEP_TEXT`, LEDs, compilador de distribución en `orby-core` + tablas web | medio | medio | **días** |
| 2 | Encima de 1: abrir app con `Win+R`, reiniciar/hibernar/cerrar sesión con `shutdown` | nada | poco | horas |
| 3 | Descriptor: *System Control* (+ wake-up remoto) y puntero absoluto; `MSTEP_ABS_MOVE` | medio | poco | días (primero la prueba multimonitor) |
| 4 | Política híbrida (`prefer_pc`) | poco | poco | horas |
| 5 | Reproducción nativa de grabaciones: `KEY_DOWN/UP`, pool de eventos, conversión | medio | medio | días |

Cada fase añade su bandera al handshake y su fila en `compat.js` / `docs/COMPATIBILIDAD.md`:
con firmware viejo la app sigue mandando esas secuencias por el PC, como ahora.

Riesgos a tener en cuenta en todas:

- **Flash**: cada pool nuevo es una región fija; reservar el mapa completo de una vez
  (texto + grabaciones) para no tener que migrar dos veces.
- **Desgaste**: escribir el pool solo al confirmar, no por cada trozo recibido.
- **El firmware no tiene tests**: la lógica de compilar (distribuciones, conversión de
  grabaciones, normalizar coordenadas) en `orby-core`, que sí los tiene; el firmware
  solo reproduce bytes.
