# Editor de la intro de arranque

Diseña la animación de las diez pantallas, reprodúcela, y cuando esté a tu
gusto expórtala. **No hace falta compilar el firmware para probarla.**

## Cómo se usa

1. Doble clic en `index.html`. No necesita servidor, ni npm, ni internet.
2. Toca los parámetros del panel de la derecha y dale a ▶ Reproducir. Todo se
   recalcula al instante y los ajustes se guardan solos en el navegador.
3. Cuando te guste, pulsa **Exportar orby_intro_data.h**.
4. Guarda ese archivo en `include/orby_intro_data.h` (sobrescribiendo el que
   haya) y compila con `flash.ps1`.

La primera vez que aparece el archivo, puede que la compilación diga «no work to
do»: toca `main.cpp` (o guárdalo sin cambios) y vuelve a compilar. A partir de
ahí ya se rastrea solo y cada exportación dispara la recompilación.

## Por qué la vista previa es de fiar

La animación se dibuja sobre un lienzo virtual que coloca las diez pantallas
donde están en el teclado, y se **hornea a píxeles**: se trocea en los diez
framebuffers de 360 bytes del SSD1306 y eso es lo que se graba. El firmware no
calcula nada, solo descomprime y vuelca.

La vista previa se pinta desde esos mismos framebuffers ya binarizados, no desde
el dibujo en gris. Por eso lo que ves es exactamente lo que van a enseñar las
pantallas, con su tramado y sus bordes duros incluidos. No hay dos
implementaciones que se puedan desincronizar.

Está comprobado de punta a punta: se calcula una huella FNV‑1a de los 95×10
framebuffers en el editor y otra en C descomprimiendo el flujo, y dan el mismo
número.

## Los huecos entre pantallas

Los huecos reales entre teclas son de 86 px en horizontal y 118 en vertical
(paso de 19 mm con un área activa de 8,6 × 4,8 mm). Es decir, cada pantalla es
una ventana pequeña a una escena mucho más grande, y una figura fina se pierde
entre medias. Por eso:

- Los valores por defecto (40 y 34) están **comprimidos**: conservan la
  disposición pero hacen que la escena se lea como una sola cosa. Súbelos a
  86/118 si prefieres la verdad métrica.
- Lo que une las diez pantallas es el **movimiento**, no la continuidad exacta
  de los píxeles. Cosas gruesas y que crucen todas a la vez.
- Las partículas quieren ser muchas: las pantallas ocupan menos de un tercio del
  lienzo, así que la mayoría viaja por el hueco y no se ve.

## Coste

Con los valores por defecto: 95 fotogramas a 30 fps (3,2 s) y **38 KB** de
Flash, de los ~1400 KB libres que hay antes de la zona de ajustes. El fotograma
más caro repinta las diez pantallas, 3600 bytes = 2,9 ms de SPI sobre los 33 ms
que dura un fotograma. De media solo cambian 3,7 pantallas por fotograma, así
que se ahorra el 63 % del SPI.

Si te pasas de tamaño, lo que más pesa es la duración y los fps; el número de
partículas casi no influye porque el comprimido es por longitud de carrera.

## Archivos

| Archivo | Qué es |
|---|---|
| `index.html` | El editor. Es lo único que necesitas. |
| `engine.js` | El motor: geometría, fases, arena, binarizado y compresión. |
| `_bake.html` | Hornea sin abrir el navegador (para automatizar o comprobar). |

Los ajustes con los que se generó una animación quedan guardados dentro del
propio `.h`, en la línea `// PRESET {...}`. Puedes cargar ese mismo `.h` con el
botón **Cargar ajustes** para seguir donde lo dejaste.
