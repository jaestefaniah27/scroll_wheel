# Orby WebGUI

## Qué es

La misma app, corriendo en el navegador contra el mismo CDC del teclado, para poder
configurar Orby en un PC donde no se puede instalar nada: un equipo corporativo, un
portátil prestado, un aula. No es una reescritura ni un subconjunto recortado a mano:
es el mismo `src/` de OrbyGUI arrancando por una vía distinta.

## Qué necesita

- Chrome o Edge **de escritorio**. Web Serial no existe en Firefox, en Safari ni en
  ningún navegador móvil.
- HTTPS o `localhost`. Web Serial exige contexto seguro.
- Que la app de escritorio esté cerrada. En Windows solo un proceso puede tener
  abierto el puerto COM del teclado a la vez; con OrbyGUI de escritorio corriendo, la
  WebGUI no consigue abrirlo.

## Qué no puede hacer y por qué

Todo esto necesita un proceso con permisos de escritorio, y el navegador no lo tiene:

- Abrir aplicaciones o archivos.
- Escribir texto por el teclado del sistema.
- Grabar y reproducir ratón y teclado.
- Cargar complementos.
- Cambiar de perfil según la ventana en primer plano.
- Autoarranque con la sesión de Windows.
- Actualizar la propia app o el firmware del teclado.
- Capturar la posición del cursor dentro del editor de secuencias (el botón
  "Posición de ratón", `src/views/profiles/sequence-editor.js`): queda
  deshabilitado porque `getMousePosition()` devuelve `null` sin el backend de
  escritorio detrás. Es distinto del módulo de grabación de ratón/teclado del punto
  anterior — este es un botón concreto dentro del editor de una secuencia.

## Qué sí puede hacer

Perfiles, páginas, etiquetas, atajos, mandos, rueda, iconos OLED, copias de
seguridad, y las secuencias que el propio teclado sabe tocar sin el PC
(`macroDeviceEligible`, ver `src/views/profiles/macros-store.js`).

## Cómo está montada

- `src/entry.js` es el único punto de entrada: si existe `window.__TAURI_INTERNALS__`
  arranca la vía Tauri (`src/tauri-main.js`); si no, la de navegador
  (`src/web-main.js`). Las dos montan `window.orby` antes de cargar el renderer.
- `src/web/orby-web.js` monta a mano ese mismo `window.orby` para el navegador, con
  Web Serial (`src/web/transport-serial.js`) como transporte e IndexedDB
  (`src/web/config-store.js`) como configuración local.
- `src/platform.js` es el único sitio del código donde se pregunta dónde corre la
  app (`isWeb()`, `can(feature)`). Las vistas nunca comprueban `navigator.userAgent`
  ni si existe tal o cual método de `window.orby`.

## Al añadir una función que necesite el PC

Añadir su nombre a `PC_ONLY` en `src/platform.js` y darle un valor vacío en
`src/web/orby-web.js`. Sin el segundo paso, la vista que la llame revienta en
navegador en vez de esconder el control.

## Dónde se publica

`.github/workflows/pages.yml`, junto con la landing. La landing sale en la raíz del
sitio y la WebGUI compilada en `/app/`.

## Chrome corporativo

En un equipo gestionado, la política `DefaultSerialGuardSetting` puede bloquear Web
Serial entero. Si el diálogo de selección de puertos no aparece nunca al pulsar
"Conectar teclado", es esa política, no un fallo de la app.
