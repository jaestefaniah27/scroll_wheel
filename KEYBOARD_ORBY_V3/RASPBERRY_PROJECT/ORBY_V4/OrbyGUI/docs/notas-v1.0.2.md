# OrbyGUI 1.0.2

## La app vuelve a actualizarse sola

Desde el paso a Tauri, OrbyGUI solo avisaba de que había una versión nueva y
abría la página de descarga en el navegador. Vuelve a hacer lo que hacía la
versión de Electron, y un poco más: comprueba al arrancar y cada seis horas,
**descarga en segundo plano e instala sola**. La app se cierra un momento y
vuelve a abrirse, sin que haya que pulsar nada.

Lo único que la detiene es que haya cambios sin escribir en la Flash del
teclado: el guardado automático es un temporizador de segundo y medio, y
cerrar el proceso a mitad se llevaría por delante esa escritura. En ese caso
espera, y aparece el aviso de siempre en la barra superior por si quieres
instalarla ya.

Las actualizaciones van **firmadas**: la app rechaza cualquier instalador que
no venga firmado con la clave del proyecto.

## Esta versión hay que instalarla a mano

La 1.0.1 y anteriores no llevan el actualizador dentro, así que no pueden
traerse esta sola. Es la última vez: a partir de la 1.0.2 el resto llegan solas.

## Instalación

Windows mostrará el aviso de SmartScreen: el instalador todavía no va firmado
para Windows (la firma de las actualizaciones y el certificado de SmartScreen
son cosas distintas; ver `docs/PUBLICACION.md`). Para continuar: **Más
información → Ejecutar de todas formas**.

Antes de ejecutarlo, comprueba que el fichero es el que publicamos:

    Get-FileHash .\OrbyGUI_1.0.2_x64-setup.exe -Algorithm SHA256
