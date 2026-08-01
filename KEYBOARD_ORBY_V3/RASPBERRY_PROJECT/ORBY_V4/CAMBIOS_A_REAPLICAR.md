# Cambios de este intento (para reaplicar desde limpio)

Todo esto es lo que hay sin commitear desde `383ef6d` (Add initial backup
configuration for Orby keyboard profiles). Lista pensada para reimplementar
feature por feature, más despacio, en vez de recuperar el diff roto.

## 1. Feature: Macros / Secuencias ejecutadas en el PC

Idea: una tecla o mando puede disparar una secuencia (centrar ratón, clic,
pausa, tecla) que se ejecuta en el PC, no en el teclado.

- **Firmware (`main.cpp`)**: nuevo opcode `KEYACT_MACRO = 0xFB`. Cuando una
  tecla o mando tiene ese modificador, en vez de `HidOut::tap(...)` manda por
  CDC `MACRO:<id>\n`.
- **GUI (`src/hid-keys.js`)**: `MACRO_MODIFIER = 0xFB` + entrada en
  `describeAction()`.
- **`electron/main.js`**: el handler de `serial.on('data', ...)` mira si la
  línea empieza por `MACRO:` y llama a `executeMacro(id)` (además de seguir
  reenviando la línea al renderer como siempre).
- **`electron/macros.js` (nuevo archivo)**: `executeMacro(macroId)` lee
  `config.load().macros`, busca la macro por id y reproduce sus acciones con
  `@nut-tree-fork/nut-js` (mouse/teclado). Acciones soportadas:
  `center_mouse`, `mouse_click`, `delay`, `key` (con `mapCodeToNutKey`, solo
  cubre letras, dígitos, Enter/Escape/Space — habría que ampliarlo).
- **`electron/config.js`**: nuevo campo `macros: []` en `DEFAULTS`.
- **`src/views/profiles.js`**:
  - Estado del módulo `pcMacros` (array), cargado al iniciar con
    `window.orby.getConfig()` y guardado con `savePCMacros()` →
    `window.orby.setConfig({ macros: pcMacros })`.
  - Pestaña **"Secuencia"** en el inspector de tecla y de mando (al lado de
    Atajo / Multimedia / Páginas).
  - `renderSequenceEditor(macroId)`: lista de acciones con botón de borrar
    cada una, botones para añadir "Centrar ratón" / "Clic" / "Pausa", botón
    "Grabar secuencia de teclas" (modo `view.capturing = 'sequence'`) que
    captura `e.code` de las pulsaciones reales y las va añadiendo (con
    debounce de guardado), y botón "Quitar secuencia".
  - Al pasar del tab "Atajo" al tab "Secuencia" por primera vez se crea una
    macro nueva con id autoincremental y se asigna esa tecla/mando a
    `MACRO_MODIFIER` + ese id.

**Pendiente / a vigilar si se reaplica**: `@nut-tree-fork/nut-js` se instaló
por accidente en la carpeta raíz del repo (`ORBY_V4/package.json`,
`package-lock.json`, `node_modules`) en vez de dentro de `OrbyGUI/`. Hay que
moverlo al `package.json` de `OrbyGUI` como dependencia real y borrar los
sobrantes de la raíz (`ORBY_V4/package.json`, `ORBY_V4/package-lock.json`,
`ORBY_V4/node_modules`, y el `txt.txt` vacío que tampoco pinta nada).

## 2. Rediseño del inspector de tecla/mando: pestañas

Antes: modificadores + tecla + acción multimedia + acciones de página, todo
visible a la vez, apilado.

Ahora: 4 pestañas — **Atajo / Secuencia / Multimedia / Páginas**
(`inspector-tabs` / `inspector-tab` en CSS). Cambiar de pestaña con
`data-act="set-tab"` decide qué modificador especial se pone (o se limpia) y
qué bloque se pinta. Se quitó el resumen final `<code>` con el resultado
("Resultado: ...") porque ya no hacía falta con las pestañas.

## 3. Rueda de scroll: calibración movida a Ajustes

Antes: la calibración de cómo se dibuja la rueda en pantalla (forma del
marcador, inversión del giro, desfase en grados, aguja en vivo con telemetría
`WHEEL:`) vivía dentro de la vista de Perfiles, en la tarjeta de la rueda.

Ahora: se movió entera a **Ajustes → nueva tarjeta "Calibración de la
rueda"** (`#settings-wheel-calib` en `index.html`, pintada por
`settings.js`). `settings.js` pasó a:
- Registrar los listeners de click/input para `dial-marker`, `dial-invert`,
  `dial-nudge`, `dial-align`, `dial-offset`.
- Escuchar la telemetría `WHEEL:` y mover la aguja (`onWheelTelemetry`).

En Perfiles, la tarjeta de la rueda se quedó solo con: velocidad
(detentsPerRev, con presets), invertir, y dos botones nuevos para asignar la
acción de **"Hacia abajo" / "Hacia arriba"** del giro (antes esto vivía como
una entrada más dentro de `ROTARY_GROUPS`, "Rueda de scroll", con el mismo
selector de tipo genérico que los encoders; ahora son botones directos que
seleccionan el slot y abren el inspector de mando normal).

⚠️ **Aquí fue donde se coló el bug gordo**: al mover la calibración fuera de
`profiles.js`, se borraron `paintDialMarker`, `onWheelTelemetry`,
`applyDialSetting` y `renderDialCalibration` del archivo — pero quedó un
`paintDialMarker(wheelDial.angle())` huérfano al final de la función
`render()` principal de Perfiles. Como `render()` se llama sync antes de
cualquier `await device.set...()` + `markDirty()` en cada `apply*`, esa
excepción abortaba el resto del handler cada vez: por eso nada se guardaba ni
se marcaba "Guardar en Flash", en cualquier parte de la vista de Perfiles
(teclas, mandos, rueda, variaciones — todo pasa por el mismo `render()`).

Si se reaplica: mover la calibración está bien, pero hay que revisar a mano
que no queden llamadas colgando a funciones borradas — buscar en todo
`profiles.js` cualquier referencia a lo que se elimina antes de dar por
bueno el cambio.

## 4. Otros cambios estéticos (`src/styles/index.css`, `src/icons.js`)

- Icono de "wheel" cambiado de una pastilla/cápsula vertical a un
  círculo con punto central.
- `.editor-layout`: columna del inspector ensanchada de 380px a 520px
  (para que quepan las pestañas y la secuencia de macro).
- `.editor-board-head`: pasó de una fila con wrap a dos filas apiladas
  (`head-row`): nombre + botón "Activar en el teclado" en una fila, capa
  NORMAL/SUPER + selector de páginas en la otra.
- Grupos de encoders renombrados: "Encoder izquierdo/derecho" → simplemente
  "Izquierdo" / "Derecho". Cabecera "Mandos giratorios" → "Encoders".
- Slider de velocidad de scroll: añadidas etiquetas "Más lento" / "Más
  rápido" a los lados (`.slider-row`).
- Se quitó el párrafo descriptivo bajo el botón de icono ("La pantalla
  muestra este icono...").

## 5. Arreglo del selector "La app de delante" en Variaciones

No es un cambio estético/nuevo, es un bug real preexistente que salió a la
luz: el botón "La app de delante" en el editor de variaciones solo
funcionaba si la función "Auto" (cambio de perfil por app) ya se había
activado alguna vez, porque solo entonces se arranca el proceso PowerShell
que vigila la ventana en primer plano. Si nunca se había tocado "Auto", el
botón siempre decía "no se ha detectado ninguna ventana" aunque hubiera algo
enfocado.

Arreglo aplicado: nueva función `getCurrentWindowInfo()` en `profiles.js`
que, si `foreground.current()` no tiene nada, llama a
`foreground.start()` y reintenta cada 300ms hasta ~2.4s antes de rendirse.
Usada tanto en `addCurrentApp()` como en `createVariant()` (para
autorrellenar el nombre con la app activa al crear la variación).

Merece la pena mantener esto al reaplicar aunque no sea parte del pack de
funcionalidades nuevas — es una mejora real e independiente.

## 6. Endurecimiento de `store.js` (no es una feature, es robustez)

`notify()` ahora envuelve cada suscriptor en `try/catch` para que si una
vista revienta al pintar, no bloquee a las demás (en particular, que el
indicador de "Guardar en Flash" siga actualizándose pase lo que pase en
otra vista). Merece la pena mantenerlo tal cual al reaplicar: es una red de
seguridad barata contra el mismo tipo de fallo que causó todo este lío.

## Resumen de archivos tocados

| Archivo | Qué cambió |
|---|---|
| `main.cpp` | opcode `KEYACT_MACRO`, envío de `MACRO:<id>` por CDC |
| `OrbyGUI/electron/config.js` | default `macros: []` |
| `OrbyGUI/electron/main.js` | intercepta líneas `MACRO:` y llama a `executeMacro` |
| `OrbyGUI/electron/macros.js` | **nuevo**: ejecuta macros con nut.js |
| `OrbyGUI/src/hid-keys.js` | `MACRO_MODIFIER` + `describeAction` |
| `OrbyGUI/src/icons.js` | icono de rueda rediseñado |
| `OrbyGUI/src/store.js` | `notify()` con try/catch por suscriptor (mantener) |
| `OrbyGUI/src/styles/index.css` | ver punto 4 |
| `OrbyGUI/src/views/profiles.js` | pestañas del inspector, secuencias, botones de rueda, `getCurrentWindowInfo`, **y el bug de `paintDialMarker` ya corregido** |
| `OrbyGUI/src/views/settings.js` | nueva tarjeta de calibración de rueda |
| `OrbyGUI/index.html` | marcado de `#settings-wheel-calib` |
| `ORBY_V4/package.json`, `package-lock.json` | **sobrantes**, instalados en el sitio equivocado — no reaplicar tal cual, mover la dependencia dentro de `OrbyGUI` |
| `ORBY_V4/txt.txt` | vacío, sobrante, se puede borrar sin más |

## Config del PC (`%APPDATA%\orbygui\orby-config.json`)

Ojo: este archivo NO se toca con el `git reset`/`checkout` porque vive fuera
del repo, en `AppData`. Ahí quedaron mezclados datos reales tuyos (macros 1-3
con secuencias T/G/A que pintan a pruebas tuyas) con basura de mis pruebas
automatizadas (una variación "electron" duplicada, macros 4-7 vacías). Sigue
pendiente que lo revises y me digas si lo limpio o lo dejas como está — no
lo he tocado desde el aviso anterior.
