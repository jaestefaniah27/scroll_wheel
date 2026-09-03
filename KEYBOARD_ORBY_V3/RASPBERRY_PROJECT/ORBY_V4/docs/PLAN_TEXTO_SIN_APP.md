# Escribir texto sin la app — plan de implementación

**Objetivo:** que una tecla de "Escribir texto" funcione **siempre**. Con OrbyGUI
delante, el texto lo sigue metiendo el PC como hasta ahora (unicode y portapapeles,
exacto para cualquier carácter). Sin ella, lo escribe el propio teclado con lo que
puede mandar: usages HID traducidos con la distribución que tenga configurada.

Hoy no es así: la escritura de texto es lo único que se quedó como acción exclusiva
del PC, y con la app cerrada esa tecla no hace nada (su pantalla sale tachada, ver
`key_needs_app` en [`main.cpp:1784`](../main.cpp)).

---

## De dónde se parte

| Pieza | Estado hoy |
|---|---|
| Dónde vive el texto | Solo en la configuración del PC (`cfg.macros[].actions[0].text`). El teclado **no lo tiene** |
| Qué guarda el teclado | El hueco de la tecla con `MACRO_MODIFIER` (0xFB) + el id de la macro, nada más |
| Qué pasa al pulsar | `trigger_macro` ([`main.cpp:1613`](../main.cpp)) ve que no hay copia jugable y manda `MACRO:<id>` por CDC |
| Quién lo escribe | `escribir_texto` en [`src-tauri/src/macros.rs`](../OrbyGUI/src-tauri/src/macros.rs): portapapeles + Ctrl+V si es largo, `KEYEVENTF_UNICODE` letra a letra si es corto |
| Sin app | Nada. `host_app_alive()` es falso → la pantalla se tacha con una barra diagonal |
| Pasos que el teclado ya sabe tocar | Espera, tecla, clic y movimiento relativo (`MacroStepType`, [`main.cpp:639`](../main.cpp)). Escribir texto no está |

El motor de reproducción ya existe y funciona (`macro_player`), igual que el reparto
PC/teclado (`trigger_macro`). Lo que falta es **el texto dentro del teclado** y **saber
traducirlo a teclas**.

---

## La decisión difícil: la distribución

Un informe HID no lleva letras, lleva *usages*: el número de la tecla física. Qué
carácter sale de un usage lo decide la distribución que tenga puesta el host, y el
teclado no puede leerla. Por eso la escritura de texto se dejó en el PC en su día, y
es la única dificultad real de esta feature. Tres caminos:

| Opción | Cómo | Por qué sí | Por qué no |
|---|---|---|---|
| **A. Tabla en el firmware** | El texto se guarda en UTF-8 en la Flash del teclado y se traduce al escribir, con una tabla por distribución (ES, US) | El texto vive en el teclado: se lo puedes llevar a otro PC, se lee de vuelta con `GET_TEXT`, ocupa 1 byte por letra y editarlo no recompila nada | Hay que mantener la tabla, y acierta solo si la distribución configurada es la del host |
| **B. La app precompila las teclas** | Al subir, la app traduce el texto a pasos `MSTEP_KEY` con `VkKeyScanEx` (Windows sabe la distribución exacta) | Traducción perfecta sin tabla propia; el firmware casi no cambia | Queda **congelada**: cambiar la distribución del PC, o enchufar el teclado en otro, escribe basura sin avisar. La WebGUI no puede consultar la distribución. Y son 8 bytes por letra contra 1, con el tope de 48 pasos |
| **C. Alt + código numérico** | `Alt`+numpad, independiente de distribución en Windows | No necesita tabla | Necesita Bloq Num, va lentísimo, y no funciona en media aplicación ni fuera de Windows |

**Se elige la A.** La B optimiza justo lo contrario de lo que se pide: el caso de uso
es *usar el teclado sin app de forma continuada*, y en ese escenario nadie va a
resubir los textos cuando cambie algo. Con la A el teclado es autónomo de verdad.

La distribución configurada se guarda **en el teclado** (no en la configuración del
PC): es una propiedad del aparato, y tiene que sobrevivir a conectarlo en un equipo
donde no hay app.

---

## Diseño

### 1. Quién escribe, y cuándo

Regla nueva, y es la que pidió el usuario: **el texto prefiere el PC; todo lo demás
prefiere el teclado.**

| Situación | Quién escribe |
|---|---|
| La macro lleva texto y la app está anunciada (`host_app_alive()`) | El PC, como hoy: `MACRO:<id>` por CDC. Camino exacto, sin tocar nada |
| La macro lleva texto y la app **no** está anunciada | El teclado, con su tabla |
| La macro no lleva texto (secuencia normal) | El teclado, como hoy (más fiable y más rápido; funciona hasta en la pantalla de bloqueo) |
| La macro lleva texto que no le cabe al teclado, o el firmware es anterior a la 4.7 | Exactamente lo de hoy: solo PC, y pantalla tachada sin app |

La preferencia se invierte solo para el texto porque los dos caminos **no son
equivalentes**: el del PC mete unicode y sale cualquier carácter con cualquier
distribución; el del teclado es lo mejor que se puede hacer sin saber nada del host.
Entre uno exacto y uno aproximado, se usa el exacto cuando está disponible.

Una secuencia mixta (texto + clics + teclas) se ejecuta **entera** en un sitio, no se
puede repartir: si lleva algún paso de texto, se aplica la regla del texto.

Esto toca dos sitios que tienen que decir lo mismo, y el comentario de
[`main.cpp:1782`](../main.cpp) ya avisa de ello: `trigger_macro` y `key_needs_app`
(el tachado de la pantalla). Si se cambia uno y no el otro, aparece una tecla tachada
que funciona perfectamente.

### 2. Dónde se guarda el texto

Región propia de Flash, detrás de la de secuencias:

```
1024 KB  FLASH_OLED_OFFSET     iconos OLED
1536 KB  FLASH_TARGET_OFFSET   ajustes            (6 sectores, 24 KB)
1560 KB  FLASH_MACROS_OFFSET   secuencias         (8 sectores, 32 KB)
1592 KB  FLASH_TEXTS_OFFSET    textos  ← NUEVO    (8 sectores, 32 KB)
1624 KB  ...                   libre (2 MB de módulo: quedan 424 KB)
```

- `TEXT_MAX_BYTES = 512` por texto, 64 huecos (uno por cada id de macro posible).
  512 bytes UTF-8 son ~512 letras ASCII o ~256 con acentos: da para una firma o un
  bloque de código, que es para lo que se usa.
- **Los textos no se copian a RAM.** `macros[64]` ya cuesta ~24 KB de RAM más su blob
  de escritura; repetir ese patrón con los textos serían 64 KB más en un chip de 264 KB.
  Se leen directamente de la Flash mapeada (`XIP_BASE + FLASH_TEXTS_OFFSET + id*512`),
  que es lo mismo que ya se hace con los iconos.
- Para escribir uno: búfer de preparación de 512 bytes (patrón de `oled_staging`), y al
  cerrar se reescribe **solo el sector** que lo contiene (8 textos por sector de 4 KB),
  leyéndolo antes de la Flash para no perder a sus vecinos.
- El commit a Flash va donde ya van los demás: el comando `SAVE` ([`main.cpp:2816`](../main.cpp)),
  que hoy hace `save_settings()` + `save_macros()`.
- La distribución y el número de versión del formato van en una cabecera propia de esta
  región, **no en `Settings`**: así no hay que tocar la cadena de migraciones
  (`SettingsV1`..`SettingsV6` → `Settings`) de `load_settings`, que es de lo más delicado
  que tiene el firmware.

### 3. Protocolo nuevo

Calcado de `OLED_CHUNK`, que es el precedente de subida troceada. El texto va en hex
porque el parser corta por `:` y las líneas acaban en `\n`: texto crudo es imposible.
`CMD_BUF_SIZE` son 512 bytes, así que un texto de 512 entra en 3 trozos.

```
SET_TEXT:<hueco>:<offset>:<hex>   → TEXT:OK:<hueco>:<offset>:<escritos>
TEXT_END:<hueco>:<bytes>          → TEXT:OK:<hueco>:<bytes>:<nomapeables>
GET_TEXT:<hueco>                  → TEXTDATA:<hueco>:<hex>
TEXT_CLEAR:<hueco>                → TEXT:OK:<hueco>:0:0
SET_LAYOUT:<es|us>                → LAYOUT:OK:<código>
```

- Handshake (`ACK`): añade `TEXT=1:MAXTEXT=512:LAYOUT=es`. Igual que `MAXMACROS`, el
  tope lo dice el teclado y la app no lo hardcodea.
- `GET_STATE`: añade `STATE:LAYOUT:<código>`.
- **`<nomapeables>`** es el número de caracteres que la distribución configurada no
  sabe producir (un emoji, un carácter chino). Lo cuenta el teclado al cerrar el texto,
  y así **la app avisa sin duplicar la tabla**: el teclado es la autoridad sobre lo que
  sabe escribir, la app solo enseña lo que le conteste.
- Paso nuevo de secuencia: `MSTEP_TEXT = 6`, con `a` = hueco de texto. Los huecos son un
  fondo común de 64 que **reparte la app** (como `nextMacroId`), porque una secuencia
  puede llevar varios pasos de texto. Un hueco fuera de rango se contesta con
  `ERR:BAD_ARGS`, que es justo el error que en su día se comió 4 s por macro al conectar.

### 4. El motor de escritura

Un `text_player` hermano de `macro_player`: máquina de estados que recorre el texto de
la Flash carácter a carácter y va empujando pulsaciones a `HidOut`.

**La trampa gorda está aquí.** `HidOut::push` ([`hid_out.h:149`](../include/hid_out.h))
**descarta en silencio** cuando la cola está llena, y la cola es de 32 eventos. Un
texto de 200 caracteres empujado de golpe pierde la mayoría sin decir nada. Hace falta:

- `HidOut::room()` (o que `push` devuelva `bool`), y que el reproductor empuje solo
  mientras haya sitio, quedándose donde iba hasta la vuelta siguiente.
- Un retardo entre caracteres (~8 ms para empezar, a afinar midiendo): hay aplicaciones
  que se comen pulsaciones sintéticas demasiado seguidas. El camino del PC ya tiene su
  `RETARDO_ENTRE_LETRAS_MS` por lo mismo.
- Tope de duración total y aborto al pulsar una tecla física: un texto largo son
  segundos, y un reproductor colgado dejaría el teclado inútil.
- `\n` → Intro (0x28) y `\t` → Tab (0x2B), igual que hace `escribir_texto` en el PC:
  pegados o como carácter no los interpreta media aplicación.

### 5. Las tablas de distribución

Dos por ahora, ES (España) y US. Formato compacto: array directo para el ASCII
imprimible (0x20–0x7E) y tabla pequeña para lo que añade el castellano (`á é í ó ú ü
ñ Ñ ¿ ¡ € ç ª º`…). Entradas de 4 bytes: `{ mod, usage, mod_muerta, usage_muerta }`.
Unos 700 bytes por distribución — irrelevante al lado de los 424 KB libres.

Tres mecanismos que la tabla tiene que cubrir:

- **Directo**: `ñ` = usage 0x33 sin modificador (donde el US tiene `;`).
- **AltGr** (bit 0x40): `@` = AltGr + `2`.
- **Teclas muertas**: `á` = la muerta `´` y después `a`, en dos pulsaciones separadas.
  Es lo que más se puede torcer según la aplicación, y por eso la Tarea 8 existe.

Un carácter que no esté en la tabla **se salta y se cuenta**; nunca se cuelga ni
inventa otro.

---

## Tareas

### Fase 1 — Firmware (lo que hace que funcione)

- [x] **1. Región de textos en Flash.** `FLASH_TEXTS_HEADER_OFFSET`/`FLASH_TEXTS_DATA_OFFSET`,
      cabecera con magia y distribución en su propio sector, `load_texts()` / `save_texts()`
      con búfer de preparación de un sector entero (8 textos, patrón `oled_staging`). Los
      `static_assert` de siempre para que no se pise nada.
      *Verificación:* no se tocó `tools/test/check_flash_map.py` —su `define()` solo sabe
      leer literales, y estos offsets se derivan de otros `#define` (`FLASH_MACROS_OFFSET +
      FLASH_MACROS_BYTES`)—, pero **sí hay toolchain real en este entorno**: firmware
      compilado y enlazado de punta a punta con Pico SDK 2.2.0 + TinyUSB (`cmake --build`),
      limpio con `-Wall -Wextra` salvo dos avisos preexistentes ajenos a este cambio. Los
      `static_assert` de disposición de Flash pasaron al compilar, que es una comprobación
      más fuerte que la que haría un script en Python sin compilador delante.
      **Trampa real que se cazó así, no a mano:** la Flash borrada de fábrica lee `0xFF`,
      no `0`. Sin nada más, un hueco de texto nunca escrito habría dado `len=0xFFFF` —un
      texto fantasma de 65535 bytes tecleando lo que hubiera en la Flash de ahí en
      adelante—. `load_texts()` graba la región de datos entera a ceros la primera vez
      que arranca este firmware (borrar sola no basta, deja `0xFF`), y dispara con un
      hueco fuera de rango en `MSTEP_TEXT` de una secuencia corrupta (igual que ya se
      recorta `MSTEP_MOVE`).
- [x] **2. Tablas de distribución.** `include/layout.h` (tipos comunes), `layout_us.h`
      (reutiliza `HID_ASCII_TO_KEYCODE` de TinyUSB tal cual, cero tablas propias) y
      `layout_es.h` (hereda de la US para letras/dígitos/espacio —posición física
      idéntica— y solo redefine lo que cambia de verdad: fila de símbolos con Mayús,
      ñ/Ñ, ¿¡, vocales con tilde y ü vía tecla muerta, y AltGr). Comprobado con un
      programa suelto (no entra en el firmware) que no hay duplicados en las tablas y
      que ES y US coinciden carácter a carácter en todo lo que tienen que coincidir.
      Los símbolos de AltGr (`@ # ~ € \ | [ ] {`) llevan su propio aviso en el fichero:
      la posición es la esperable, pero sin teclado delante no se pueden jurar — es
      justo lo que la Tarea 8 existe para comprobar. `^ \` }` se dejan fuera a
      propósito (no mapeables) en vez de arriesgar una posición inventada.
- [x] **3. Reproductor de texto.** `text_player_tick` con contrapresión real contra la
      cola de `HidOut` (`HidOut::room()`, nuevo), retardo entre caracteres (8 ms), teclas
      muertas (dos pulsaciones reales, comprobando hueco para las dos a la vez), Intro y
      Tab, y tope de duración total (30 s). **Sin implementar:** abortar al pulsar una
      tecla física — necesita enganchar el detector de pulsaciones del núcleo 1, y se ha
      preferido dejarlo pendiente antes que meter un enganche a medio entender en el
      pipeline de entrada. El tope de 30 s acota el peor caso mientras tanto.
- [x] **4. Comandos nuevos.** `SET_TEXT`, `TEXT_END`, `GET_TEXT`, `TEXT_CLEAR`,
      `SET_LAYOUT` en el dispatch de `process_command`; `TEXT=1`, `MAXTEXT` y `LAYOUT`
      en el handshake y en `GET_STATE`; `MSTEP_TEXT = 6` en `MacroStepType` y en
      `macro_player_tick`. Y `save_texts()` colgado del comando `SAVE`.
- [x] **5. El reparto.** `trigger_macro` prefiere el PC solo cuando la macro lleva texto
      Y la app está anunciada; sin ella, siempre el teclado. `key_needs_app` **no cambia
      de lógica** —sigue preguntando solo si el teclado tiene copia jugable—, pero su
      comentario sí: ya no es "espejo exacto" de `trigger_macro`, porque una macro con
      texto SIEMPRE hace algo (aproximado sin app, exacto con ella) y por tanto nunca se
      tacha, aunque `trigger_macro` decida mandarla al PC por tenerlo más a mano.

### Fase 2 — App

- [ ] **6. Subida y comparación.** `text: 6` en `DEVICE_STEP_TYPE`; `macroDeviceEligible`
      acepta pasos de texto **solo si** el texto cabe en `MAXTEXT` y el firmware anuncia
      `TEXT=1` (si no, un firmware viejo contesta `ERR:UNKNOWN_CMD` a un comando que
      nadie espera y cada intento se muere de viejo a los 4 s). `syncMacroToDevice` sube
      el texto además de los pasos, y `matchesDevice` lo compara con `GET_TEXT` antes de
      escribir: sin eso, cada conexión gasta una escritura de Flash del teclado sin que
      haya cambiado nada — la misma lección que ya se aprendió con las secuencias.
      Repartidor de huecos de texto (`nextTextSlot`) y limpieza al borrar.
- [ ] **7. Ajuste de distribución.** Selector en Ajustes (Español / US) que escribe
      `SET_LAYOUT` en el teclado y lee el valor del handshake. No se guarda en la
      configuración del PC: es del aparato.
- [ ] **8. Comprobador de distribución.** Botón "Comprobar escritura": la app pide al
      teclado que escriba un texto de prueba con todos los caracteres raros **por su
      propio camino** (comando `TEXT_TEST`, que se salta la preferencia por el PC) dentro
      de un campo de la propia app, y compara lo que llega con lo que esperaba. Es la
      única forma de validar la tabla de verdad —el teclado no puede saber qué vio el
      host— y de paso le sirve al usuario para comprobar su equipo antes de fiarse.
- [ ] **9. Textos de la interfaz.** La pestaña "Texto" ([`macro-tabs.js:406`](../OrbyGUI/src/views/profiles/macro-tabs.js))
      dice hoy "necesita esta app abierta": pasa a explicar los dos caminos y el límite
      de tamaño. Igual `renderSequenceLocation` ([`sequence-editor.js:290`](../OrbyGUI/src/views/profiles/sequence-editor.js)),
      cuyo motivo "el teclado solo sabe mandar códigos de tecla" deja de ser cierto.
      Aviso cuando el texto no quepa o traiga caracteres no mapeables (con el número que
      contesta el teclado).

### Fase 3 — Cierre

- [ ] **10. WebGUI.** `text` sale de `PC_ONLY` en [`platform.js`](../OrbyGUI/src/platform.js)
      cuando el teclado anuncia `TEXT=1`: en el navegador no hay backend, pero ahora el
      que escribe es el teclado. Es la primera acción "de PC" que gana la vía navegador.
      Actualizar [`docs/WEBGUI.md`](../OrbyGUI/docs/WEBGUI.md), que la lista como imposible.
- [ ] **11. Versión y documentación.** Firmware **4.7**: `include/orby_version.h`,
      `FW_RECOMMENDED` y una entrada `deviceText` en `FEATURES` de `compat.js`, fila en
      [`docs/COMPATIBILIDAD.md`](COMPATIBILIDAD.md), y el `CLAUDE.md` (que describe el
      texto como acción solo-PC). Los tres primeros en el mismo commit, como manda la regla.
- [ ] **12. Publicación.** Release `fw-v4.7` como *prerelease*, a mano: GitHub Actions
      sigue bloqueado por facturación (ver [COMPATIBILIDAD.md](COMPATIBILIDAD.md)).

### Comprobación sobre el teclado real

El firmware no tiene tests automáticos, así que esta lista se pasa a mano:

1. Texto corto (correo) con la app abierta → lo escribe el PC, idéntico a hoy.
2. El mismo, con la app cerrada del todo (**Salir** desde la bandeja, no la X) → lo
   escribe el teclado.
3. Texto de 400 caracteres sin app → llega **entero** (es la prueba de la contrapresión
   de la cola: si faltan letras, la Tarea 3 está mal).
4. `áéíóúüñÑ¿¡@€\` en Bloc de notas, navegador y VS Code — las teclas muertas son lo
   que más se tuerce entre aplicaciones.
5. Con la app abierta y cerrándola a media escritura: no debe escribirse dos veces.
6. Secuencia mixta (texto + Ctrl+C + espera) por los dos caminos.
7. Reiniciar el teclado y repetir sin app: el texto tiene que seguir en la Flash.

---

## Límites que hay que aceptar (y contar en la interfaz)

- **La distribución es una suposición.** Si el PC tiene otra puesta, sale otra cosa. Es
  inherente al camino HID, no un fallo: por eso el camino del PC sigue siendo el
  preferente cuando está disponible.
- **Los dos caminos no se parecen en velocidad.** El PC pega 300 caracteres de golpe con
  Ctrl+V; el teclado los teclea a ~8 ms cada uno (2,4 s) y se ve teclear. A cambio, el
  del teclado funciona donde Ctrl+V no pega (consola clásica, algún juego).
- **512 bytes por texto.** Lo que no quepa se queda como está hoy: solo PC y pantalla
  tachada. Nunca truncar en silencio.
- **Emojis y alfabetos no latinos no se pueden.** Ninguna distribución los produce con
  pulsaciones; se saltan y la app lo avisa con el número que da el teclado.
- **Desgaste de Flash**: cada edición de un texto reescribe un sector. Por eso la
  comparación previa de la Tarea 6 no es un lujo.

## Aparcado a propósito

- **Confirmación de ejecución del PC.** Sería lo ideal para el caso "la app se cuelga a
  medio camino": el teclado manda `MACRO:<id>`, espera un `MACRO_DONE:<id>` y, si no
  llega en X ms, lo escribe él. Se deja fuera porque un ACK perdido escribe el texto
  **dos veces**, que es peor fallo que el que arregla. Retomar solo si aparece el caso
  de verdad.
- **Más distribuciones** (FR, DE, PT). La tabla es un fichero por distribución; se añaden
  cuando alguien las necesite.
- **Elegir el camino por tecla** ("esta siempre por el teclado"). Complica la interfaz
  para un caso que la regla automática ya cubre.
