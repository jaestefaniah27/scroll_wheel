# Orvy 1.1.3

Interfaz nueva: las teclas, la rueda y los mandos se configuran sobre la réplica
del teclado. Reconoce hasta el **firmware 4.17** y lo instala sola.

## Novedades

- **Configurar es ampliar la réplica.** En el Inicio se pulsa una tecla, la
  rueda o un mando y se abre su panel a la derecha, en dos niveles como Ajustes
  de iOS. Editando la capa SUPER o una variación por app, la réplica enseña lo
  que cambia.
- **Perfiles, solo para gestionar:** lista a la izquierda y ficha a la derecha
  (nombre, icono, páginas, variaciones por app, duplicar, eliminar). La
  biblioteca se abre ahí mismo.
- **Deshacer** en el panel de teclas y mandos: botón de la cabecera o Ctrl+Z,
  hasta 50 pasos.
- **Secuencias más compactas:** la espera va dentro de cada paso, los pasos se
  reordenan arrastrando y se pueden insertar entre dos.
- **Reordenar teclas arrastrando:** con «Configurar teclas» abierto, mantener
  pulsada una tecla de la réplica la levanta y se lleva su acción, etiqueta e
  icono.
- **Mandos con páginas y menús** (firmware 4.15): girar para cambiar de página,
  pulsar para abrir un menú. La tecla 10 puede quedar como tecla normal.
- **Invertir mandos y rueda, mando de menú y mantener-girar-soltar** para
  elegir perfil (firmware 4.16).
- **Animación al elegir perfil** en el menú (firmware 4.17), repetida en las
  pantallas del Inicio.
- **El firmware se actualiza solo:** al conectar el teclado o tras 2 minutos
  sin tocarlo.
- **Pantallas iguales que en el teclado:** los perfiles de la biblioteca salen
  con el icono del catálogo y su nombre, no con la etiqueta en texto.
- **Limpieza de macros sin usar** al conectar; lo retirado vuelve solo si
  alguna tecla lo pide.
- Ventana estrecha (WebGUI): el panel sube desde abajo como una hoja.

## Arreglos

- La app leía mal la velocidad de la animación del menú guardada en el teclado.
- Con la lista de apps instaladas vacía, el editor de secuencias colgaba la ventana.
- Con el firmware 4.14 caben 255 secuencias en el teclado, no 64.

## Compatibilidad

Reconoce hasta el firmware 4.17. Con uno anterior funciona, sin las funciones
que ese firmware no tiene.
