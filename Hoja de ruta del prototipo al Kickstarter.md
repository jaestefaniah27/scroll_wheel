# Hoja de ruta: del prototipo al Kickstarter

2026-09-19 · @Someone

## Resumen

El objetivo es saber en 6 semanas, gastando casi nada, si hay gente dispuesta a pagar unos 200 € por el teclado. Solo si la respuesta es sí se pasa a reservas, forma jurídica y Kickstarter.

La ruta tiene dos puertas de decisión con criterios escritos de antemano: semana 6 (hacia el 31 de octubre) y semana 12 (hacia el 12 de diciembre). Entre puertas no se añaden funciones al hardware.

Ritmo para dos personas con trabajo:

- 8-10 horas semanales cada uno y una reunión fija de 30 minutos.
- Cada tarea tiene un único responsable y una fecha.
- Una cosa pública por semana: un post, un vídeo corto o un correo a la lista.

| Fase | Cuándo | Resultado |
| --- | --- | --- |
| 0. Cimientos | Semana 0-1 | Acuerdo firmado, nombre, nicho, hardware congelado |
| 1. Material | Semanas 1-2 | Vídeo, fotos, landing, formulario |
| 2. Enseñarlo | Semanas 2-5 | Lista de espera y respuestas del formulario |
| Proveedores (en paralelo) | Semanas 1-5 | Coste real a 100 y 300 unidades |
| Puerta 1 | Semana 6 | Seguir, tirada pequeña o parar |
| 3. Reservas | Semanas 7-12 | Forma jurídica, marca, depósitos, unidades beta |
| Puerta 2 | Semana 12 | Kickstarter o venta directa |
| 4. Preparar campaña | Meses 4-5 | Página, presupuesto, suministro confirmado |
| 5. Campaña y entrega | Meses 6-12 | Unidades entregadas |

## Fase 0 · Cimientos (semana 0-1)

Nada de esta fase se ve desde fuera, pero evita los dos problemas que más rompen proyectos de dos personas: el reparto y la propiedad.

- [ ] Acuerdo escrito y firmado con tu socio, de 1-2 páginas: porcentaje de cada uno, quién hace qué (hardware, firmware, app, marketing), horas semanales, cómo se decide y qué pasa si uno lo deja.
- [ ] Revisar tu contrato de trabajo: exclusividad, no competencia y propiedad intelectual. Revisar también las normas de la universidad sobre trabajos de asignatura. Si hay dudas, una consulta con un abogado laboralista.
- [ ] Elegir nombre y comprobar que está libre: dominio, redes y marcas en TMview, el buscador de marcas de la EUIPO. Ojo con "Orbi": Netgear vende routers con esa marca, así que conviene comprobar el conflicto antes de encariñarse.
- [ ] Elegir un nicho principal, solo uno: CAD o edición de vídeo. Criterio: el que usáis vosotros y podéis demostrar mejor en vídeo.
- [ ] Congelar el hardware v1 tal como está. Como mucho, añadir la pantalla de los diales. El resto de ideas va a una lista "v2".
- [ ] Escribir ya los criterios de la Puerta 1 y no tocarlos después.

## Fase 1 · Material para enseñarlo (semanas 1-2)

Todo va en inglés, porque el mercado de este producto es internacional.

- [ ] Vídeo de 30-45 segundos, con luz de día y mesa limpia, sin voz y con subtítulos. Guion: 3 s de la rueda girando libre y moviendo un modelo; 12 s de teclas cambiando de icono según la aplicación; 15 s de uso real en el nicho elegido; cierre con el nombre y "join the waitlist". Grabad versión horizontal y vertical.
- [ ] Cinco fotos buenas: producto entero, detalle de las teclas OLED, detalle de la rueda, en uso e interior.
- [ ] Landing de una página (Carrd o Framer) con el vídeo, tres beneficios, el precio previsto de campaña (199-219 €) y un campo de email (MailerLite o similar).
- [ ] Formulario de Interest Check, máximo 8 preguntas: programa que usan, qué dispositivo tienen ahora, 12 o 16 teclas, pantalla para los diales sí o no, cuánto pagarían por tramos, importancia del código abierto, país y email.
- [ ] Servidor de Discord para los interesados.

Enseñad qué hace el teclado, no cómo está hecho: esquemas, CAD y firmware se publican en el lanzamiento.

## Fase 2 · Enseñarlo y medir (semanas 2-5)

El orden importa: primero los foros expertos, donde el feedback mejora el mensaje, y después los sitios de más alcance.

1. Interest Check en Geekhack y KeebTalk. Es el mecanismo habitual de esa comunidad para macropads: hilo, formulario y Discord ([ejemplos de IC en Geekhack](https://buttondown.com/Minnericht/archive/group-buys-and-interest-checks-of-week-7/), [un macropad europeo vendido a 125 € en KeebTalk](https://keebtalk.com/t/gb-nofaff-macropad/17650)).
2. Reddit: r/MechanicalKeyboards y los subreddits del programa elegido. Contadlo como proyecto (dos ingenieros, cómo está hecho, qué feedback buscáis) y leed antes las normas de autopromoción de cada uno. Responded a todos los comentarios en las primeras 24 horas.
3. Prensa maker: enviarlo a Hackaday y a KBD.news, que ya han publicado proyectos de teclas con OLED como Keybon y PolyKybd.
4. Un vídeo corto por semana en YouTube Shorts, TikTok e Instagram, con el proceso de construcción.
5. Un correo cada dos semanas a la lista de espera con los avances.

Cada domingo, apuntad en una hoja: visualizaciones, visitas a la landing, emails nuevos, respuestas al formulario y precio que dicen que pagarían.

## En paralelo · Proveedores y costes (semanas 1-5)

El objetivo es bajar el coste de 80 € a 50-55 € por unidad en una tirada de 300, y saberlo con presupuestos reales antes de la Puerta 1.

| Pieza | A quién pedir precio | Qué pedir |
| --- | --- | --- |
| Pantalla OLED | [BuyDisplay / EastRising](https://www.buydisplay.com/white-0-42-inch-oled-display-panel-72x40-iic-i2c-serial-spi-ssd1306), [Golden Morning LCD](https://goldenmorninglcd.com/oled-display/0.42-inch-oled-72x40-ssd1306-white-gme7240-01/), [DisplayModule](https://www.displaymodule.com/products/0-42-inch-oled-graphic-display-72x40-with-i2c) | Panel desnudo, precio a 1.000 y 4.000 unidades, muestras de dos proveedores |
| PCB montada | JLCPCB y PCBWay | Presupuesto para 100 y 300 placas con todo el SMD montado |
| Carcasa | Servicios de impresión MJF o SLS en nailon | Precio a 100 unidades, y un presupuesto de molde de inyección solo como referencia |
| Rodamiento | Un distribuidor industrial y un fabricante por volumen | Medida estándar y muestras de dos fuentes |
| Ensayos CE | Un laboratorio de compatibilidad electromagnética | Presupuesto de ensayo para un periférico USB |

He supuesto pantallas de 0,42 pulgadas (72×40, SSD1306). Si son de 0,49 pulgadas, los mismos proveedores las tienen.

Son 12 pantallas por teclado: 300 teclados son 3.600 paneles, un volumen que ya merece precio de fábrica. Con las muestras, comprobad que el pinout del cable plano coincide entre proveedores, para poder cambiar de uno a otro.

- [ ] Hoja de costes con tres columnas: ahora, a 100 unidades y a 300 unidades.
- [ ] Añadir embalaje, envío por zona y un 3-5 % de unidades de reposición.

## Puerta 1 · Decisión de la semana 6 (hacia el 31 de octubre)

Se decide con números, no con sensaciones. Estos umbrales son una propuesta: ajustadlos ahora, no cuando lleguen los datos.

| Resultado | Señal | Qué hacer |
| --- | --- | --- |
| Verde | 500 emails o más, y al menos un 30 % del formulario dispuesto a pagar 199 € o más | Pasar a la Fase 3 |
| Ámbar | 150-500 emails, o interés claro pero a menor precio | Tirada de 50 unidades en venta directa a la comunidad, sin Kickstarter |
| Rojo | Menos de 150 emails tras 20.000 visualizaciones o más | Parar, o repetir el test con el otro nicho una sola vez |

De una lista de emails gratuita suele comprar un 1-3 %, y de una lista con depósito, un 20-40 %. Son cifras orientativas de agencias de crowdfunding. Por eso los emails solo abren la puerta: lo que valida de verdad son los depósitos de la Fase 3.

## Fase 3 · Forma jurídica, protección y reservas (semanas 7-12)

Desde aquí se cobra dinero, así que primero va lo legal.

- [ ] Sesión con un gestor: autónomo o sociedad limitada, compatibilidad con tu empleo e IVA. Abrir una cuenta bancaria del proyecto.
- [ ] Registrar el nombre como marca en la EUIPO.
- [ ] Registrar el diseño en la EUIPO: [cuesta 350 € y hay un año de plazo desde la primera vez que se enseña](https://euipo.europa.eu/tunnel-web/secure/webdav/guest/document_library/contentPdfs/Design-Get-Protection_en.pdf). Mientras tanto protege el diseño no registrado, 3 años y solo contra copias. Cubre el aspecto, no la función.
- [ ] Página de reserva con depósito reembolsable de 5 €, que asegura el precio más bajo de campaña (189 €).
- [ ] Fabricar 10 unidades de preserie y dejar 5-10 a usuarios beta del Discord. Corregir lo que rompan.
- [ ] Enviar 3-5 unidades a creadores pequeños del nicho: canales de CAD, de edición o de teclados.
- [ ] Estabilizar firmware y app. Preparar la documentación de la parte abierta: protocolo, SDK de plugins y firmware.

Puerta 2, semana 12 (hacia el 12 de diciembre): con 150-200 depósitos, preparar Kickstarter. Con menos, tirada pequeña en venta directa y repetir más adelante con una lista mayor.

## Fase 4 · Preparar Kickstarter (meses 4-5)

La meta de la campaña es el mínimo con el que fabricar es viable: unas 100 unidades, entre 20.000 y 22.000 €. Si no se alcanza, a nadie se le cobra. Si se alcanza, [hay obligación de completar el proyecto y entregar cada recompensa](https://help.kickstarter.com/hc/en-us/articles/115005048173).

| Tramo | Precio | Para quién |
| --- | --- | --- |
| Reserva anticipada | 189 € | Quienes dejaron depósito, unidades limitadas |
| Early bird | 199 € | Primeras 48-72 horas |
| Campaña | 219 € | Resto de la campaña |
| Tienda | 249 € | Precio de referencia tras la campaña |

- [ ] Presupuesto completo: piezas, montaje, carcasa, embalaje, ensayos CE, comisiones (8-10 %), IVA del 21 % en ventas a la UE, envío por zona, aranceles de EE. UU. y un 10-15 % de imprevistos.
- [ ] Decidir quién paga el arancel en los envíos a EE. UU. y decirlo claro en la página.
- [ ] Suministro confirmado por escrito para 300 unidades o más de OLED y rodamiento, con dos proveedores. Sin esto no se lanza.
- [ ] Página de campaña: vídeo de 2 minutos, preguntas frecuentes, sección de riesgos honesta y fecha de entrega con 3 meses de margen.
- [ ] Evitar lanzar en agosto o a finales de diciembre. Preparar el aviso a la lista y al Discord para el primer día.

## Fase 5 · Campaña, fabricación y entrega (meses 6-12)

El trabajo de verdad empieza cuando la campaña termina: una entrega tardía o mala es lo que hunde la reputación de un proyecto de hardware.

1. Campaña de 30 días, con una actualización por semana y respuesta a todos los comentarios.
2. Al cerrar: encuesta de direcciones, cobro de envíos e impuestos, y pedido de componentes el mismo mes.
3. Tirada piloto de 20 unidades. Probarlas todas, corregir, y solo entonces lanzar la tirada completa.
4. Envíos empezando por la UE, que no tiene aduanas, y después el resto.
5. Actualización mensual a los mecenas hasta la última entrega, también cuando haya retrasos.

Si algo impide terminar, las condiciones de Kickstarter piden explicar en qué se usó el dinero y ofrecer la devolución de lo que quede.

## Qué no hacer

- Añadir funciones antes de la Puerta 1.
- Gastar en publicidad antes de la Puerta 1.
- Pagar moldes de inyección antes de tener 500 unidades validadas.
- Cobrar dinero antes de tener forma jurídica.
- Publicar esquemas, CAD o firmware antes del lanzamiento.
- Lanzar la campaña sin el suministro de OLED y rodamiento confirmado.
- Cambiar los criterios de las puertas cuando lleguen los datos.
