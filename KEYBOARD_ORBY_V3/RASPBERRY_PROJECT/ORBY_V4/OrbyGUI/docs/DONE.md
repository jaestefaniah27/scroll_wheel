# DONE

Tareas completadas, extraídas del TODO.md 

* [x] Identidad USB en constantes (`ORBY_USB_VID` / `ORBY_USB_PID`) en un solo sitio, y
      `bcdDevice` con la versión real de firmware (0x0410) en vez del 1.0 de plantilla.
* [x] Número de serie único desde el ID de la flash. Estaba fijo en `"123456"`, así que dos
      teclados enchufados a la vez compartían identidad y `serial.js` no los distinguía.
* [x] Strings USB acordes a la marca: `Orby` / `Orby V4` / `Orby V4 Control` / `Orby V4 Input`.
      Fuera el "Composite" (detalle interno) y el "Corp" (sociedad que no existe).
* [x] `serial.js` descubre el puerto por VID **y** PID. Antes bastaba con que el PID
      coincidiera, viniera del fabricante que fuera: abría puertos ajenos.
* [x] Landing estática en `web/index.html` + workflow `.github/workflows/pages.yml`.
      Solo falta poner Settings → Pages → Source: GitHub Actions.
* [x] Guía de SmartScreen y checklist de release con SHA-256, en la landing y en
      PUBLICACION.md.
* [x] Las releases de firmware (`fw-v*`) van como prerelease. Comparten repositorio con
      las de la app y `releases/latest` devuelve la más reciente sea de lo que sea: la
      `fw-v4.4` dejó a todas las copias instaladas de OrbyGUI con "Cannot find latest.yml
      in the latest release artifacts" y sin poder actualizarse.
   * **Estuvo a medias hasta el 2026-08-12**, y esta casilla mentía: la regla estaba
     escrita en COMPATIBILIDAD.md pero `.github/workflows/firmware.yml` publicaba **sin**
     `--prerelease`, así que cada firmware publicado por CI volvía a romper lo mismo. Se
     descubrió al montar la actualización automática, donde ese fallo pasa de molestia a
     fallo mudo. Ahora la bandera está en el workflow y `verifica_plan_tauri.sh` la
     comprueba.
* [x] Avisar de las teclas que no funcionan sin la app abierta, en los dos sitios:
   * [x] La parte de interfaz: la insignia en teclas y mandos de `src/views/profiles.js`
         que avisa cuando una acción necesita la app abierta.
   * [x] La parte del teclado (firmware 4.5): la pantalla OLED de esas teclas sale
         tachada con una barra diagonal mientras la app de escritorio no esté delante.
         El teclado no lo deduce de tener el puerto abierto —la WebGUI también lo abre
         y no puede ejecutar nada en el PC—: la app se anuncia con `HOST_APP:1` en cada
         presentación y el aviso caduca a los 25 s si deja de llegar.
* [X] Mejorar plugin de lampara: permitir ver el brillo que estás configurando en una tecla que configures. así puedes tener si quieres una página que veas lo que estás configurando en la lámpara. También poder ver el color que estás configurando. También añadir controles para poder poner con una tecla un brillo determinado, y también un color determinado.
* [X] Poder asignar un icono a un perfíl. Así, Desde el teclado cuando quieres cambiar de perfil, puedes identificar rápidamente cada perfil.

### Orby WebGUI (Versión Portátil / Entornos Corporativos)
Objetivo: crear una versión web de la app que funcione mediante WebHID/Web Serial,
permitiendo configurar el teclado sin instalar software en el PC host.

* [x] **Investigación y Pruebas de Concepto (PoC):**
   * [x] Validar viabilidad de comunicación bidireccional vía WebHID con el firmware actual en Chrome/Edge.
   * [x] Diseñar protocolo ligero de lectura/escritura de configuración a través de un endpoint HID Raw.
   * Resuelto con **Web Serial**, no con WebHID: Chrome bloquea WebHID sobre colecciones
     de teclado/ratón/consumer, así que habría hecho falta una colección *vendor*
     nueva, subir el PID y un protocolo binario aparte. Con Web Serial se reutiliza el
     CDC que el teclado ya expone y no hay que tocar el firmware para nada de esto.
* [x] **Adaptación del Firmware (Raspberry Pi Pico):**
   * [x] Habilitar un endpoint USB Raw HID para configuración (si no existe ya).
   * [x] Implementar guardado persistente (Flash/EEPROM) de la configuración completa (macros, colores, perfiles).
   * [x] Hacer que el firmware parsee y ejecute macros (Device-Eligible) de manera autónoma sin depender del PC.
   * No hizo falta ninguna adaptación: el guardado en Flash y la reproducción
     autónoma de secuencias *device-eligible* ya existían desde el firmware 4.0, y el
     endpoint Raw HID sobra en cuanto se usa Web Serial sobre el CDC existente.
* [x] **Desarrollo de la Web App:**
   * [x] Configurar repositorio y hosting estático (ej. GitHub Pages o Vercel).
   * [x] Migrar/Recrear la UI de OrbyGUI para web (React/Vue/Vanilla).
   * [x] Implementar la API de conexión y solicitud de permisos USB/HID al navegador.
   * [x] Crear las funciones de subida (Push) y bajada (Pull) de configuración desde el hardware.
* [x] **Experiencia de Usuario (UX):**
   * [x] Añadir advertencias visuales en la WebGUI sobre qué acciones son 'Device-Eligible' y cuáles no (ya que no habrá daemon corriendo).
   * [x] Implementar método de cambio manual de perfil integrado en el teclado (pantalla OLED o combinación de teclas).
   * El cambio manual de perfil desde el propio teclado ya existía: el menú de la
     tecla 12 y la telemetría `EV:CTX:<perfil>:<pagina>:<total>` que el teclado emite
     por su cuenta al cambiar de contexto.

### Mejoras de perfiles

* [x] Cambiar funcionalidad del botón menú. Una pulsación corta avanza a la siguiente página. Mantener pulsado te muestra los perfiles que tienes: si mientras mantienes, pulsas otro perfil, cambias a ese perfil. de esta manera se puede cambiar de perfiles fácilmente.

  Hecho en `main.cpp` (bloque "TECLA DE MENÚ"): mientras se mantiene el menú
  pulsado (tras el segundo que lo abre), se sigue sondeando el resto de teclas
  sin esperar a soltar. Si se toca la que tiene un perfil debajo (misma
  correspondencia tecla↔pantalla que ya pintan los OLEDs), se activa ese
  perfil al momento y se vuelve a `MODE_NORMAL`; si se suelta el menú sin
  tocar nada, queda como antes: abierto en `MODE_MENU_PERF` para elegir a
  mano. Se espera también a que se suelte la tecla del perfil elegido para que
  el bucle normal no la lea como una pulsación nueva y dispare su atajo.

## Migración a Tauri (2026-08-12)

* [x] **Tarea 13: retirar Electron.** Fuera `OrbyGUI/electron/` (11 ficheros, ~2 900
      líneas), las cuatro dependencias de ejecución y el bloque de electron-builder.
      Los números que la justificaban, medidos con las dos builds contra el mismo teclado:
      instalador **97,6 MB → 7,5 MB**, procesos **4 → 1**, RAM con la ventana abierta
      **616,6 MB → 39,8 MB** (y ~8 MB escondida en la bandeja).
   * Hubo que reanclar antes `test/superficie-orby.test.mjs` y
     `tools/test/verifica_plan_tauri.sh`, que leían `electron/` como *referencia* de la
     migración. La referencia del contrato `window.orby` pasa a ser
     `src/tauri/orby-tauri.js`.
   * Y arreglar dos cosas que el plan no había mirado y tenían código vivo colgando de
     Electron: `OrbyGUI.bat` y el panel `tools/orby-manager` (botones de lanzar, pipeline
     de release y bump de versión).
   * `docs/PLUGINS.md` reescrito entero: documentaba complementos como módulos de Node
     cuando el real es declarativo desde la API 2.

* [x] **La app se actualiza sola y en silencio** (`tauri-plugin-updater`), y **el firmware
      también** cuando el teclado lleva cinco minutos sin usarse. Dos interruptores en
      Ajustes para volver al modo manual, encendidos de fábrica.
   * El reinicio pasa por `window::marcar_saliendo()`: sin eso, el cierre a la bandeja se
     comía el `app.restart()` sin dar ningún error.
   * No se reinicia con un firmware instalándose, ni grabando, ni con una secuencia en
     marcha.
   * La decisión de flashear vive aparte en `src/firmware-decide.mjs`, con sus pruebas:
     valen por lo que impiden, porque un falso positivo ahí es un teclado que se muere a
     mitad de un atajo.
   * De paso, dos fallos mudos: `compare_fw` leía `1.0.0-alpha` como igual a `1.0.0` (la
     versión instalada **es** una preversión, así que publicar la v1.0.0 no habría
     disparado nada), y el bump del panel se negaba ante esa misma versión, o sea que no
     había forma de publicar.

* [ ] **Falta hacerlo a mano en Windows**: generar el par de claves de firma, comprobarlo
      sobre el teclado y publicar la primera release. Guion paso a paso en
      [PLAN_PUBLICAR.md](PLAN_PUBLICAR.md).
