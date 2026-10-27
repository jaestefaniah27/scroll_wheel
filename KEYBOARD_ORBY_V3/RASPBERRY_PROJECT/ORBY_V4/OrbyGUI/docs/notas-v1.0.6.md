# OrbyGUI 1.0.6

Va con el **firmware 4.7**: la app avisa al conectar y lo instala desde Ajustes.

(La 1.0.5 se publicó solo con el cambio de número: todo lo de estas notas llega
en la 1.0.6.)

## Diseño nuevo

- La app tiene el aspecto de macOS e iOS: controles, listas, avisos y alertas
  de confirmación como los de un iPhone, y transiciones con la misma curva.
- **Modo claro.** Sigue al modo del sistema y cambia con él; en
  Ajustes → Apariencia se puede fijar Claro u Oscuro.
- El dashboard dibuja el Orby real, con sus medidas: carcasa blanca, teclas con
  cap transparente, la rueda y los dos mandos donde están de verdad.

## Biblioteca de perfiles

En **Perfiles → Biblioteca** hay perfiles listos para doce programas, con una
página por tarea y más órdenes con SUPER:

- **Ingeniería:** Altium (esquemático / PCB / vista y salida), KiCad, Fusion 360 y LTspice.
- **Programación:** VS Code (edición / paneles / depuración), Windows y el navegador.
- **Office en español:** Excel, Word y PowerPoint.
- **Creativos:** Photoshop y Premiere Pro.

«Añadir» crea el perfil con todas sus páginas, y además la regla para que se
ponga solo al abrir su programa. Con el firmware 4.7 cada tecla sale con su
icono en las pantallas.

## Las macros viajan con el teclado

Hasta ahora, lo que hacía una macro (abrir una app, escribir un texto, una
grabación) solo lo sabía el PC donde se creó: en otro ordenador, esas teclas no
hacían nada. Con el firmware 4.7 el teclado guarda una copia, y la app de
cualquier PC las recupera al conectar. Al editar una macro, la copia del
teclado se actualiza sola.

## Arreglos

- Restaurar una copia de seguridad escribía solo una página de cada perfil y
  dejaba las demás como estaban. Ahora restaura todas.
