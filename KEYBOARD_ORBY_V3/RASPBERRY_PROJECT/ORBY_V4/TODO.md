* [ ] Añadir biblioteca de perfiles para que los usuarios no tengan que crear los suyos desde cero. Los usuarios pueden aportar al pool de perfiles en el apartado perfiles de la comunidad. Los usuarios tienen que aportar Nombre del perfil y descripción. EN la biblioteca de perfiles sale el número total de atajos, el número de páginas, etc.
* [ ] He pensado en monetizar perfiles o paquetes de perfiles: el software básico es gratis pero hay que pagar una baja cantidad de € para obtener acceso a un paquete de perfiles para unos cuantos programas como Altium, excel, word, etc, o pagar cada uno por separado. Estudiar viavilidad de esto.
* [ ] **Que ORBY parezca un producto, no un proyecto.** La app ya es algo que usaría a diario
  y por lo que pagaría. Hace falta: que Windows no avise al instalar, que el teclado no se
  presente como un cacharro de pruebas al enchufarlo, y una web para venderlo.
  Evaluado el 2026-08-08 y partido en las tareas de abajo.
  Guía operativa: [OrbyGUI/docs/PUBLICACION.md](OrbyGUI/docs/PUBLICACION.md).

  **Hecho (0 €)**
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