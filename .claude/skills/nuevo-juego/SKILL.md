---
name: nuevo-juego
description: Crea un juego nuevo para las pantallas del teclado ORBY (lógica en la app OrbyGUI, sin tocar el firmware), con su icono, su alta en el catálogo y sus tests. Úsala cuando pidan un juego, un minijuego o una partida para el teclado, o cambiar uno de los que ya hay (Topos, Simon, Sintoniza, Reflejos, Misil, Blackjack).
---

# Crear un juego para el teclado ORBY

Rutas relativas a `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/`.

## Cómo funciona (no hace falta tocar el firmware)

El firmware (modo GAME, desde 4.29) solo **pinta lo que le manda la app y le pasa las entradas**.
Toda la lógica corre en la app: `OrbyGUI/src/games/engine.js` llama al juego a ritmo fijo, le
pasa las entradas y manda al teclado solo lo que ha cambiado. Un juego nuevo es **un fichero
`.mjs` más una línea en el catálogo**.

Lo que hay que saber del teclado:

- **10 pantallas OLED de 72x40, 1 bit.** La rejilla de teclas 3x3 son las pantallas 1-9
  (de izquierda a derecha y de arriba abajo). La 10 queda fuera y sirve para el marcador.
- **La tecla 10 está reservada para salir.** El juego nunca la recibe, pero sí puede dibujar
  en su pantalla.
- **Entradas:** las teclas, los dos mandos (encoders) con su botón y la rueda magnética.
- **Sin prisas de fotogramas:** el motor hace control de flujo. Un juego por turnos o con
  animaciones de 100-300 ms va sobrado. Uno de reflejos también funciona (ver `reflejos.mjs`),
  pero mide los tiempos con `ev.us`, el reloj del teclado.

## El contrato

```js
export default {
  id: 'mijuego',            // único, en minúsculas
  nombre: 'Mi juego',
  descripcion: 'Una frase: qué hay que hacer.',
  icono: '#MIJUEGO',        // etiqueta del icono (máx. 8 caracteres con la #)
  unidad: 'puntos',         // cómo se llaman los puntos en el récord
  mejorSiMenor: false,      // true si gana el que menos tiene (tiempos, fallos)
  crear({ aleatorio, ahora }) {   // aleatorio(n) → entero 0..n-1; NO usar Math.random
    return {
      get terminado() {},   // true cuando ha acabado (la pantalla de final la pone final.mjs)
      get puntos() {},
      get estado() {},      // texto corto para la vista de la app («3 rondas»)
      tick(dt) {},          // avanza dt ms
      dibujar(lienzo) {},   // lienzo[0..9] = pantallas 1..10, ya en negro
      tecla(ev) {},         // { tecla, pantalla, pulsada, t, us }  (pulsada true/false)
      rueda(ev) {},         // { delta, angulo }
      mando(ev) {},         // { mando: 0|1, delta }
      mandoBoton(ev) {},    // { mando, pulsado }
    };
  },
};
```

Las entradas son todas opcionales. Usa siempre el `aleatorio` del contexto: los tests lo
siembran para que una partida fallida se pueda repetir.

## Dibujar: `OrbyGUI/src/games/dibujo.mjs`

| Función | Para qué |
|---|---|
| `px`, `rect`, `marco`, `linea`, `disco`, `anillo`, `elipse`, `pastilla(fb,x,y,w,h,r,v)` | primitivas sobre una pantalla (`fb`) |
| `texto(fb, s)` | texto lo más grande que quepa, centrado (marcadores, «FIN») |
| `etiquetaValor(fb, 'FICHAS', 120)` | número grande con su rótulo pequeño debajo |
| `icono(fb, '#ETIQ')` | el icono grande del catálogo |
| `invertir(fb)`, `subir(fb, dy)` | resaltar una pantalla y desplazar su contenido |
| `new Malla(lienzo, [[1,2,3],[4,5,6]])` | varias pantallas como un lienzo continuo, contando el hueco entre ellas |

Ejemplos reales:
- `simon.mjs` es el más corto: fases, temporizadores y teclas.
- `blackjack.mjs` es por turnos, con cartas en tira por varias pantallas, rueda y mando.
- `misil.mjs` es de acción continua, con `Malla`.

## Pasos

1. **Crear `OrbyGUI/src/games/<id>.mjs`** siguiendo el contrato.
   - Organízalo como una máquina de fases (`'jugando' | 'final'`…).
   - Ignora las teclas que no tocan en cada fase, por ejemplo con
     `if (fase !== 'x' || !ev.pulsada) return;`.
   - Exporta también las funciones puras (puntuar, barajar…) para poder probarlas.
2. **Darlo de alta** en `OrbyGUI/src/games/catalogo.mjs`: import y añadirlo a `JUEGOS`.
3. **Icono** (si no se hace, sale el texto de la etiqueta en su lugar):
   - `tools/oled_ui/catalog_juegos.py` → `'#MIJUEGO': ('nombre_icono', 'Mi juego'),`
   - `tools/oled_ui/icons_juegos.py` → `@icon('nombre_icono')`, en un lienzo de 24x22, con la
     silueta rellena y los detalles en hueco (`v=0`). Copia el estilo de `cards` o de `simon`.
   - Regenerar con `python tools/oled_ui/build.py`. Escribe `OrbyGUI/src/oled-ui-data.mjs` **y**
     `include/oled_ui_data.h`.
     - La app pinta el icono por su cuenta, así que el juego funciona sin flashear.
     - Aun así, el cambio en `include/` hay que subirlo para que `build.py --check` pase. El
       firmware lo incluirá en su próxima versión.
   - Para ver el icono, usa la vista de juegos de la app o el menú de juegos del teclado.
4. **Tests** en `OrbyGUI/test/games-<id>.test.mjs`:
   - las funciones puras;
   - un flujo con teclas concretas.
   `games-juegos.test.mjs` ya simula partidas al azar de **todos** los juegos del catálogo y
   exige que terminen en un número razonable de tics. Si el tuyo puede no acabar nunca con
   teclas al azar, añade un límite (rondas, tiempo, manos), o márcalo con `sinFin: true` si
   está pensado para no acabar (el Blackjack: se sale con la tecla 10 y guarda su saldo con
   `persistente: true`, ver `gestor.js`); entonces el test solo exige que aguante sin romperse.
5. **Comprobar:** `cd OrbyGUI && node --test test/*.mjs` (`npm test` está roto con Node 24),
   o la skill `comprobar`.
6. **Probar en el teclado:** con la app de desarrollo (`OrbyGUI.bat` o `npm run tauri:dev`) y
   el teclado conectado, ir a Juegos y jugar unas partidas. Sin teclado no se puede jugar: el
   motor se niega a arrancar, así que quedan los tests.
7. **Publicar:** sale en la siguiente versión de la app (skill `publicar`). Sube la versión de
   la app y no la del firmware, salvo que hayas cambiado algo más en `main.cpp`.

## Trampas

- **Etiquetas de 8 caracteres como máximo, contando la `#`:** es lo que guarda el firmware
  por etiqueta. Por eso `#BLACKJ` y no `#BLACKJACK`.
- **No uses `Math.random` ni `Date.now`:** usa `aleatorio` y acumula el tiempo con el `dt` de
  `tick`. Si no, los tests dejan de ser repetibles.
- **`dibujar` recibe las pantallas en negro cada vez:** dibuja el estado completo, no solo lo
  que ha cambiado. El motor ya se encarga de mandar solo lo que cambia.
- **La partida tiene que acabar en `terminado`:** la pantalla de resultados con «Otra vez» y
  «Salir» la pone `final.mjs`. No dibujes la tuya.
- **Fuente del teclado (`tools/oled_ui/font.py`):**
  - Las minúsculas con tilde y la ñ existen.
  - Las mayúsculas con tilde salen sin ella (`Á` → `A`), porque a 7 px no cabe.
  - Un carácter que no está sale como `?`.
