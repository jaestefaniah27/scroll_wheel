# Iconos, fuente y pantallas del teclado

Todo lo que se ve en las diez pantallas sale de aquí: la fuente, los iconos y
qué icono lleva cada etiqueta. Está dibujado para lo que son las pantallas,
72×40 a 1 bit, no reducido desde otra cosa.

| Fichero | Qué es |
|---|---|
| `font.py` | La fuente, píxel a píxel: proporcional, con minúsculas, tildes y ñ. La grande es la misma pasada por EPX. |
| `icons.py` | Los iconos (24×22, trazo de 2 px) y las piezas de interfaz (interruptor, puntos de página). |
| `catalog.py` | Etiqueta de tecla → icono y nombre en castellano. También las pantallas de los menús. |
| `layout.py` | Dónde va cada cosa en la pantalla. Es el modelo que copia `include/oled_ui.h`. |
| `build.py` | Genera `include/oled_ui_data.h` y el fixture de `tools/test/test_ui.cpp`, y lo mismo para la app: `OrbyGUI/src/oled-ui-data.mjs` y `OrbyGUI/test/fixtures/oled-ui-cases.json`. |
| `preview.py` | Hojas PNG con los cuatro perfiles de fábrica y todos los iconos. |

## Cambiar un icono o añadir uno

1. Dibújalo en `icons.py` (o toca el que hay).
2. Si es para una etiqueta nueva, añádela a `catalog.py`.
3. `python3 tools/oled_ui/preview.py /tmp/orby` y mira las hojas.
4. `python3 tools/oled_ui/build.py` y compila el firmware.

`include/oled_ui_data.h` va al repositorio: compilar el firmware no necesita
Python. Lo mismo `OrbyGUI/src/oled-ui-data.mjs`: la app dibuja con él, en
`src/oled-ui.mjs` (copia en JS de `oled_ui.h`), las pantallas que el teclado no
le manda, y `test/oled-ui.test.mjs` comprueba que salen iguales píxel a píxel. Lo que sí hace `tools/test/run.ps1` es comprobar con `build.py --check`
que nadie ha tocado un dibujo sin regenerar.

## Cómo elige el firmware qué enseñar en una tecla

1. Si la tecla tiene un icono subido desde la app, ese.
2. Si su etiqueta está en el catálogo, el icono con su nombre debajo. Se
   busca por etiqueta y no por posición, así que el icono sigue a la acción si
   se mueve de tecla. Unas pocas etiquetas cambian de significado según el
   programa y llevan entrada propia (`PREM/CUT` es la cuchilla de Premiere;
   `CUT` a secas, las tijeras).
3. Si no, la etiqueta en texto: en grande si cabe, en pequeño si no.

## Por qué está dibujado a mano

A 7 px de alto ninguna fuente vectorial aguanta el umbral a 1 bit: salen trazos
de 1 y 2 px mezclados. Los iconos sí se dibujan como vectores, pero con reglas
para que caigan en la rejilla: líneas de 2 px sobre coordenadas enteras,
siluetas rellenas donde un contorno no se leería, y supermuestreo ×8.
