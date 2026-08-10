* [ ] Añadir biblioteca de perfiles para que los usuarios no tengan que crear los suyos desde cero. Los usuarios pueden aportar al pool de perfiles en el apartado perfiles de la comunidad. Los usuarios tienen que aportar Nombre del perfil y descripción. EN la biblioteca de perfiles sale el número total de atajos, el número de páginas, etc.
* [ ] He pensado en monetizar perfiles o paquetes de perfiles: el software básico es gratis pero hay que pagar una baja cantidad de € para obtener acceso a un paquete de perfiles para unos cuantos programas como Altium, excel, word, etc, o pagar cada uno por separado. Estudiar viavilidad de esto.
* [ ] **Que ORBY parezca un producto, no un proyecto.** La app ya es algo que usaría a diario
  y por lo que pagaría. Hace falta: que Windows no avise al instalar, que el teclado no se
  presente como un cacharro de pruebas al enchufarlo, y una web para venderlo.
  Evaluado el 2026-08-08 y partido en las tareas de abajo.
  Guía operativa: [OrbyGUI/docs/PUBLICACION.md](OrbyGUI/docs/PUBLICACION.md).

  **Hecho (0 €)**

  * [X] Identidad USB en constantes (`ORBY_USB_VID` / `ORBY_USB_PID`) en un solo sitio, y
    `bcdDevice` con la versión real de firmware (0x0410) en vez del 1.0 de plantilla.
  * [X] Número de serie único desde el ID de la flash. Estaba fijo en `"123456"`, así que dos
    teclados enchufados a la vez compartían identidad y `serial.js` no los distinguía.
  * [X] Strings USB acordes a la marca: `Orby` / `Orby V4` / `Orby V4 Control` / `Orby V4 Input`.
    Fuera el "Composite" (detalle interno) y el "Corp" (sociedad que no existe).
  * [X] `serial.js` descubre el puerto por VID **y** PID. Antes bastaba con que el PID
    coincidiera, viniera del fabricante que fuera: abría puertos ajenos.
  * [X] Landing estática en `web/index.html` + workflow `.github/workflows/pages.yml`.
    Solo falta poner Settings → Pages → Source: GitHub Actions.
  * [X] Guía de SmartScreen y checklist de release con SHA-256, en la landing y en
    PUBLICACION.md.

  **Se puede hacer ya, pero depende de terceros (0 €)**

  * [ ] Pedir un PID gratis bajo el VID de Raspberry Pi (`0x2E8A`) con una PR a
    `raspberrypi/usb-pid`. El texto está redactado en PUBLICACION.md § 1.
    Sigue con `0xCafe`, el VID de los *ejemplos* de TinyUSB: no es nuestro y no se puede
    distribuir así. Ocupar un PID del VID de Raspberry Pi sin concesión sería peor.
    Hacerlo **antes** de vender la primera unidad: cambiar el VID después deja a las ya
    vendidas en otro puerto COM.
  * [ ] Elegir licencia del repositorio. No tiene ninguna, y SignPath exige licencia OSI.
    Decisión con consecuencias: permisiva deja que cualquiera fabrique y venda el
    teclado; copyleft obliga a publicar los cambios. Bloquea lo de abajo.
  * [ ] Escribir a SignPath Foundation: certificados OV **gratis** para proyectos open source.
    Borrador listo en PUBLICACION.md § 2, con la pregunta sobre si los paquetes de
    perfiles de pago chocan con su criterio.

  **Casi gratis, cuando haya algo suelto**

  * [ ] 💰 Cuenta de desarrollador de Microsoft Store: ~19 €, **pago único**. La Store firma
    con el certificado de Microsoft → cero SmartScreen, sin certificado propio. Requiere
    empaquetar en MSIX (`electron-builder` ya trae target `appx`). Mejor relación
    resultado/dinero de toda la lista.
  * [ ] 💰 Dominio propio, ~12 €/año.

  **💰 BLOQUEADO — sin presupuesto ahora mismo**

  * [ ] Certificado de firma propio, si SignPath no sale. Desde junio de 2023 la clave privada
    tiene que vivir en hardware (FIPS 140-2 nivel 2): ya no vale un `.pfx` en disco.
    Azure Trusted Signing ~10 €/mes es lo barato; OV con token 250–400 €/año;
    EV 400–700 €/año. Todos exigen entidad legal verificada (autónomo vale).
  * [ ] Subir `electron-builder` de 24.13.3 a 26 (soporte de `azureSignOptions`) y añadir
    `win.publisherName` en `package.json`, que `electron-updater` lo necesita para validar
    la firma de las actualizaciones. El trabajo es gratis pero no sirve de nada sin
    certificado: hacerlo el mismo día que se contrate.
  * [ ] Registrar la marca "Orby". La CA verifica el nombre legal al emitir el certificado, y
    si se vende conviene tenerla antes.
  * [ ] Marcado CE para vender el teclado en la UE: directiva EMC 2014/30/UE + RoHS. La
    declaración la firmas tú, pero necesitas informe de laboratorio (EN 55032 emisiones
    clase B, EN 55035 inmunidad): 1.500–4.000 €. Es el mayor coste del proyecto entero.
    Solo cuando el hardware esté congelado.
  * [ ] Registro RAEE en España y Ecoembes por el embalaje. Va con lo anterior.
  * [ ] FCC Part 15 clase B si se vende en EEUU.
  * [ ] Kickstarter: exige entidad legal, cuenta bancaria, vídeo y coste unitario cerrado.
    Y enviar a la UE sin CE no es una opción.
* [ ] Poder asignar un icono a un perfíl. Así, Desde el teclado cuando quieres cambiar de perfil, puedes identificar rápidamente cada perfil.
* [ ] Mejorar plugin de lampara: permitir ver el brillo que estás configurando en una tecla que configures. así puedes tener si quieres una página que veas lo que estás configurando en la lámpara.
* [ ] Añadir la opción a multimedia de una tecla que sea la hora. Añadir también la opción de una tecla que sea la fecha.
* [ ] Que en la vusta de perfiles y macros, si una tecla no se reproducirá desde el teclado sino desde la app al no ser device elgible, que haya un pequeño simbolito para que se indique que sin la app abierta no funciona. Además, quiero que en el teclado, si no tienes la app abierta, las teclas cuya acción no sea debice elegible se marquen para que sepas que no tienes la app conectada y que justo esa tecla no va a funcionar.
  * [X] Hecha la parte de interfaz: la insignia en teclas y mandos de `src/views/profiles.js`
    que avisa cuando una acción necesita la app abierta.
  * [ ] Pendiente la parte del teclado: marcar en las pantallas OLED del propio Orby
    las teclas que no van a funcionar sin la app conectada. Es firmware, fuera del
    alcance de la WebGUI.
* [ ] Necesito investigar si gracias a la nueva funcionalidad de plugins, se puede hacer un plugin para que en chrome poder ejecutar ciertas acciones como: ir a la página número N del documento que tengo abierto ahora mismo. Y de manera similar a esto, si se podría hacer un plugin por ejemplo para altium, y poder realizar acciones de altium más fácilmente gracias al plugin, por ejemplo que el plugin detecte automáticamente si estás en un esquemático, en un diseño pcb, etc, y cambie de página de un perfil automáticamente (por ejemplo tener una página para esquemáticos, otra para layout, etc, y que el plugin ayude con la experiencia de crear los atajos ofreciendo ayudas). Lo mismo con un plugin para work o excel y cosas así. Después, evaluar si merece la pena que la existencia de estos plugins venga directamente en la app como funcionalidades disponibles, o es mejor que se mantengan como plugins separados disponibles a descargar desde la propia app por ejemplo.
  * **Investigación (2026-08-10):**
    * **Chrome:** Es posible mediante una *Chrome Extension* utilizando **Native Messaging** para comunicarse con la app de escritorio (OrbyGUI). La app registra un "Native Messaging Host" en el registro de Windows. La extensión puede leer la URL actual, inyectar scripts para leer el contexto (HTML) y comunicarse bidireccionalmente. *Nota:* Interactuar con el visor de PDFs nativo de Chrome para ir a una página concreta es muy limitado (sandboxing), aunque sí se puede controlar el scroll y estado de páginas web normales.
    * **Altium Designer:** Altium no expone fácilmente su contexto en tiempo real a ejecutables externos de forma directa. Para lograrlo (saber si estás en esquemático o PCB), hay que desarrollar un **Custom Plugin (Extensión DLL)** usando el Altium SDK (Delphi/C#) que corra *dentro* de Altium y acceda a `SchServer` y `PCBServer`. Este plugin podría comunicarse con OrbyGUI vía WebSockets o IPC para avisarle de cambios de contexto y que OrbyGUI cambie de página de perfil automáticamente.
    * **Word/Excel (Office):** Windows ofrece la interfaz **COM (ActiveX)**. Se pueden usar scripts (PowerShell) o pequeños binarios C# que se conecten a la instancia de Word/Excel, lean el estado actual del documento e intercepten eventos.
    * **¿Integrados o separados?** Dado que requieren componentes de terceros (una extensión en la Chrome Web Store, una DLL compilada para Altium, interacciones COM), **es mejor mantenerlos como plugins separados**. Incluirlos en la app base aumentaría el peso, la probabilidad de falsos positivos en antivirus y la complejidad de mantenimiento. OrbyGUI debería ofrecer un sistema de "Marketplace" o descargas bajo demanda.

### Ideas Estratégicas para la Tienda de Plugins (Marketplace)
Para hacer que el ecosistema Orby resulte muy atractivo "out-of-the-box" aprovechando su hardware único (10 OLEDs, encoders rotativos, rueda magnética alta resolución), se priorizarán los siguientes plugins de terceros:

* [ ] **Hardware Monitor (CPU / GPU / RAM):**
   * Leer estadísticas vía API de Windows (WMI/LibreHardwareMonitor).
   * Aprovechar las pantallas OLED para dibujar gráficos de barras, porcentajes y temperaturas en tiempo real. Un "dashboard" de hardware autónomo.

* [ ] **Control de Streaming (OBS Studio):**
   * Conectarse vía WebSocket a OBS.
   * Actualizar texto/iconos OLED según las escenas disponibles. Feedback de luces intermitentes en la OLED al estar grabando (REC) o transmitiendo.

* [ ] **Integración de Teletrabajo / Audio (Teams / Discord / Spotify):**
   * Muteo universal a nivel de sistema/aplicación, con un icono en la OLED mostrando si el micrófono está capturando audio para dar seguridad.
   * Uso de los encoders para controlar los niveles de volumen de aplicaciones específicas de manera separada (audio routing).

* [ ] **Desarrollo de la Tienda de Plugins (Marketplace):**
   * Crear la interfaz de usuario dentro de OrbyGUI para explorar, descargar e instalar plugins creados por la comunidad o de forma oficial.
   * Diseñar la arquitectura del repositorio/backend donde se alojarán los plugins para su descarga.

### Orby WebGUI (Versión Portátil / Entornos Corporativos)
Objetivo: Crear una versión web de la app que funcione mediante WebHID/Web Serial, permitiendo configurar el teclado sin instalar software en el PC host.

* [X] **Investigación y Pruebas de Concepto (PoC):**
   * [X] Validar viabilidad de comunicación bidireccional vía WebHID con el firmware actual en Chrome/Edge.
   * [X] Diseñar protocolo ligero de lectura/escritura de configuración a través de un endpoint HID Raw.
   * Resuelto con **Web Serial**, no con WebHID: Chrome bloquea WebHID sobre colecciones
     de teclado/ratón/consumer, así que habría hecho falta una colección *vendor*
     nueva, subir el PID y un protocolo binario aparte. Con Web Serial se reutiliza el
     CDC que el teclado ya expone y no hay que tocar el firmware para nada de esto.
* [X] **Adaptación del Firmware (Raspberry Pi Pico):**
   * [X] Habilitar un endpoint USB Raw HID para configuración (si no existe ya).
   * [X] Implementar guardado persistente (Flash/EEPROM) de la configuración completa (macros, colores, perfiles).
   * [X] Hacer que el firmware parsee y ejecute macros (Device-Eligible) de manera autónoma sin depender del PC.
   * No hizo falta ninguna adaptación: el guardado en Flash y la reproducción
     autónoma de secuencias *device-eligible* ya existían desde el firmware 4.0, y el
     endpoint Raw HID sobra en cuanto se usa Web Serial sobre el CDC existente.
* [X] **Desarrollo de la Web App:**
   * [X] Configurar repositorio y hosting estático (ej. GitHub Pages o Vercel).
   * [X] Migrar/Recrear la UI de OrbyGUI para web (React/Vue/Vanilla).
   * [X] Implementar la API de conexión y solicitud de permisos USB/HID al navegador.
   * [X] Crear las funciones de subida (Push) y bajada (Pull) de configuración desde el hardware.
* [X] **Experiencia de Usuario (UX):**
   * [X] Añadir advertencias visuales en la WebGUI sobre qué acciones son 'Device-Eligible' y cuáles no (ya que no habrá daemon corriendo).
   * [X] Implementar método de cambio manual de perfil integrado en el teclado (pantalla OLED o combinación de teclas).
   * El cambio manual de perfil desde el propio teclado ya existía: el menú de la
     tecla 12 y la telemetría `EV:CTX:<perfil>:<pagina>:<total>` que el teclado emite
     por su cuenta al cambiar de contexto.
