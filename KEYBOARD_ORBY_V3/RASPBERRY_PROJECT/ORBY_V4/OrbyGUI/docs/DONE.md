# DONE

Tareas completadas, extraídas del TODO.md (evaluación "que ORBY parezca un producto" del 2026-08-08).

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
* [x] Hecha la parte de interfaz: la insignia en teclas y mandos de `src/views/profiles.js`
      que avisa cuando una acción necesita la app abierta.

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
