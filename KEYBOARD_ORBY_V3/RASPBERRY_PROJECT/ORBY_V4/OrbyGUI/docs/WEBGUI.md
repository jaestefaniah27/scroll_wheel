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

- Elegir qué aplicación o archivo abre una tecla (la lista de apps instaladas y el
  selector de archivos son del PC). Lo que ya esté configurado lo abre el teclado.
- Grabar ratón y teclado (reproducir lo grabado sí: lo hace el teclado).
- Cargar complementos.
- Cambiar de perfil según la ventana en primer plano.
- Autoarranque con la sesión de Windows.
- Actualizar la propia app o el firmware del teclado.
- Capturar la posición del cursor dentro del editor de secuencias (el botón
  "Posición de ratón", `src/views/profiles/sequence-editor.js`): queda
  deshabilitado porque `getMousePosition()` devuelve `null` sin el proceso de
  Electron detrás. Es distinto del módulo de grabación de ratón/teclado del punto
  anterior — este es un botón concreto dentro del editor de una secuencia.

## Qué sí puede hacer

Perfiles, páginas, etiquetas, atajos, mandos, rueda, iconos OLED, copias de
seguridad, y las secuencias que el propio teclado sabe tocar sin el PC
(`macroDeviceEligible`, ver `src/views/profiles/macros-store.js`).

Con el firmware 4.7 eso incluye escribir texto, las acciones de energía, la
multimedia dentro de una secuencia y reproducir grabaciones: el navegador solo
las sube, y las ejecuta el teclado (`src/native/`). Los textos se traducen con la
distribución de teclado elegida en Ajustes.

## Cómo está montada

- `src/entry.js` es el único punto de entrada: si `window.orby` ya existe (lo montó
  `electron/preload.js`) arranca la vía Electron; si no, carga `src/web-main.js`.
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
