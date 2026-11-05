# Plan: Air Strike (antes «Misil»), mejoras

**Estado (2026-11-05): plan terminado.** Fases 0-5 hechas en la rama `airstrike` (04-nov) y fase 6 completa: L1 y L3 (`OLED_REINIT`), L2 (vigilante), L4 (reset completo al `GAME:1` y al llegar la app) y más margen en el bit-bang del 74HC595 (sospecha 1). Todo en firmware 4.32. Falta probarlo en el teclado: si la 9 se sigue apagando con esto, es hardware (alimentación o la línea CS9) y toca el analizador lógico.

Plan del 2026-11-03, sobre `main` en 8dd4480 (Orvy 1.1.19). Sigue a `PLAN_MISIL.md`, que está
cerrado: las reglas de la casa son las mismas (castellano de España, comentarios que explican
**por qué**, nada de `Math.random()` en `misil.mjs` —todo el azar sale de `aleatorio(tope)`—,
sin dependencias nuevas, `node --test test/*.mjs` en verde, un commit por fase con el formato
`feat(misil): ...`). **No empieces una fase sin cerrar la anterior.** Las pruebas que este plan
rompe a propósito se actualizan, no se borran.

Ficheros que se tocan:

| Fichero | Qué cambia |
|---|---|
| `OrbyGUI/src/games/misil.mjs` | nombre, explosiones, cráteres, aviso en la mira, misil enemigo, derrota |
| `OrbyGUI/test/games-misil.test.mjs` | pruebas de todo lo anterior |
| `tools/oled_ui/catalog_juegos.py` | etiqueta del menú (`'#MISIL': ('missile', 'Misil')`) |
| `include/hardware_oled.h`, `main.cpp` | el bug de la pantalla 9 (fase 6) |
| `tools/misil/ver-mapa.mjs` | mirar el resultado: **tras tocar un dibujo, vuelca el PNG y míralo** |

## Qué ha pedido el usuario (resumen)

0. Cambiar el nombre del juego a **Air Strike**.
1. Más explosiones al destruir vehículos.
2. Que los tanques se distingan de los cráteres que dejan tus impactos.
3. Mejorar la imagen de las defensas aéreas (feas y poco explicativas) y que se note que te
   apuntan: una **señal de peligro en la mira** para que mires alrededor, y la defensa más
   visible.
4. Cuando te disparan, que **se vea salir un misil hacia ti**; al impactar, que diga que te han
   dado y que pierdes una vida.
5. Al morir (por fallos o por impacto) el mapa no desaparece de golpe: primero **«HAS PERDIDO»**
   y a los 2 s, la pantalla de después de la partida. Si te mató una defensa, un **círculo
   parpadeando** señala a quien te dio el golpe final.
6. Un fallo de hardware: la pantalla 9 se apaga y solo vuelve reiniciando el teclado. Idea del
   usuario: un protocolo que detecte el fallo y reinicie la pantalla.

## Cómo está hoy (lo que manda el código)

- Antiaéreos, RPG y lanzadores son `tirador` (`dispara: true` en `OBJETIVOS`). `amenazar(e)`
  mete `{ e, desde, hasta }` en `amenazas`; `hasta` es al menos `desde + nivel().avisoMs`, y
  `avisoMs` **nunca baja de 3500 ms** (hay un test). `impactar(a)` resta una vida, pone el aviso
  `'TOCADO'` y `sacudir()`.
- El aviso de hoy es solo que el tirador **desaparece y reaparece** (`pintarEntidad`, periodo
  300 → 80 ms) y un «!» de 2×5 px junto a las vidas en la pantalla 10. No hay misil enemigo, ni
  nada en la mira. El tirador que se apaga a ratos tampoco ayuda a verlo.
- `estallar()` solo dibuja la onda del misil propio (`EXPLOSION_MS = 650`). Cada objetivo
  destruido deja `crateres.push({ x, y })`; se pinta como un anillo de radio 4 sobre una caja
  en blanco de 9×9, igual para todo. El depósito, que también es un anillo, se le parece.
- Al morir, `impactar` y `tick` ponen `fase = 'final'`, y `terminado` es `fase === 'final'`: el
  motor pasa **directo** a `final.mjs` (PARTIDAS/PUNTOS/RECORD y OTRA VEZ/MENU/SALIR). El «FIN»
  de `dibujar` ya no lo ve nadie. Por eso el mapa desaparece de golpe.
- `texto(fb, s)` escribe lo más grande que quepa en **una** pantalla, centrado.

---

## Fase 0. Cambio de nombre

- `misil.mjs`: `nombre: 'Air Strike'` y la `descripcion` (menciona «Misil» solo si hace falta).
  **Deja `id: 'misil'`**: es la clave de las puntuaciones guardadas (`scores.mjs`); cambiarla
  borraría los récords. Dilo en un comentario de una línea.
- `tools/oled_ui/catalog_juegos.py`: la etiqueta pasa a «Air Strike». Comprueba que cabe en
  72 px con la fuente; si no, dos líneas o «AIRSTRIKE». Regenera con `tools/oled_ui/build.py`
  (genera `include/oled_ui_data.h` y `OrbyGUI/src/oled-ui-data.mjs`; no se editan a mano) y mira
  la hoja con `preview.py`. Si el firmware pinta esa etiqueta, hay que reflashear.
- Comentario de `main.cpp` (~4638, «Misil acelera…»), cabecera de `misil.mjs` y de
  `games-misil.test.mjs`. Las filas 4.29/4.30 de `docs/COMPATIBILIDAD.md` son historia: no se tocan.
- Prueba: `JUEGOS` sigue teniendo el id `'misil'` y `nombre === 'Air Strike'`.

## Fase 1. Derrota con «HAS PERDIDO» (y señalar al culpable)

Primero la derrota, porque la fase 4 se apoya en ella.

- Fase nueva `'derrota'`, `DERROTA_MS = 2000`. Una función `perder(culpable)` la inicia y la
  llaman `impactar` (con el tirador que ha dado) y `tick` (cuando acaba la explosión y
  `vidas <= 0`, sin culpable). Guarda `culpable = { x, y, w, h }` en coordenadas del territorio
  y `hasta = tiempo + DERROTA_MS`.
- El mundo **se congela**: en `tick`, `moverMundo`, `vaivenear`, `disparar`, la estancia y el
  vuelo no corren en `'derrota'`; `mover` ignora los mandos (hoy solo mira `'final'`). Las
  amenazas se vacían.
- Al cumplirse `hasta`, `fase = 'final'` y `terminado` sigue siendo `fase === 'final'`: nada
  cambia para `flujo.mjs` ni `final.mjs`. Es lo que cumple «a los 2 segundos, fuera».
- `dibujar` en `'derrota'`: el mapa **sigue pintado** y atenuado (trama de ajedrez al 50 % sobre
  lo ya dibujado, para que el texto destaque); «HAS» en la pantalla 4 y «PERDIDO» en la 5 (cada
  `texto` ocupa una pantalla entera), y la 10 conserva los puntos (sin cuadraditos de vida: ya
  no quedan). Los primeros 150 ms, destello (`invertir`) de las nueve pantallas si fue por
  impacto.
- **Culpable**: si hay, dos círculos alternos (`anillo`, radios 9 y 13, cada 250 ms) centrados
  en su caja, sobre la atenuación. Si perdiste por fallos, solo el texto. Para que sea «en
  claro», la caja del culpable se vuelve a pintar sin atenuar.
- El récord se registra como hoy: `final` lo llama después con `puntos`.
- Pruebas: (a) tres fallos → `terminado` es falso al terminar la explosión y verdadero
  `DERROTA_MS` después; (b) durante la derrota las entidades no se mueven y la mira no responde;
  (c) con un impacto mortal, `culpable` es el tirador que disparó; con fallos, `null`;
  (d) `dibujar` en derrota no deja las diez pantallas en blanco (el mapa sigue ahí). Hay que
  **actualizar** las de `games-misil.test.mjs` que esperan `terminado` justo tras la última vida
  (líneas ~358 y ~464): ahora hay que avanzar `DERROTA_MS` más.

## Fase 2. Explosiones al destruir

- Lista `explosiones` en coordenadas del territorio, como `crateres`: así siguen en su sitio
  aunque la cámara se mueva o se vaya. `estallar()` crea una por objetivo destruido
  (`{ x, y, t0, tipo }`) además de la onda del misil propio. `tick` las envejece **siempre**
  (también en `'cayendo'` y `'derrota'`), no solo durante `'explosion'`.
- Cada una dura lo suyo según el tipo: depósito, bola de fuego grande (radio ~12, ~900 ms) y
  chispas (8 píxeles que salen en radio, posición fija por índice); camión, tanque y lanzador,
  2-3 estallidos escalonados (60-120 ms de desfase, ±3 px) y uno final; caza y antiaéreo, como
  el camión; tienda y soldado, uno pequeño. Que los destruidos de un mismo misil **no**
  estallen todos a la vez: desfase de 40 ms por orden.
- Un bit: el fogonazo alterna disco blanco y anillo negro con halo, que es lo que se ve sobre
  el terreno. Tras la última, humo: 3-4 píxeles que suben desde el cráter durante ~1,5 s.
- Todo el azar (desfases, chispas) sale de `aleatorio` en `estallar()`, no al dibujar, para que
  sea reproducible. `dibujar` solo lee.
- Prueba: destruir N objetivos crea N explosiones, ninguna sigue viva pasado su tiempo y
  `explosiones` no crece sin límite en una partida larga.

## Fase 3. Cráteres que no se confunden con tanques

(Interpretado: «cráteres de tus impactos» frente a los vehículos, sobre todo el tanque y el
depósito.)

- Regla: **nada recto en un cráter, nada redondo y hueco en un vehículo.**
- `crateres` guarda `{ x, y, w, h, tipo }` (la caja del destruido; el tamaño sale de ahí,
  topado entre 7 y 15 px). La semilla del dibujo sale de `(x * 31 + y)`, sin azar nuevo.
- Dibujo: mancha irregular rellena con **trama de puntos** (un píxel sí y otro no) en lugar de
  anillo limpio, con el contorno discontinuo y unos píxeles sueltos de chamusquina alrededor.
  Se limpia solo la mancha, no una caja 9×9. El del depósito es el mayor.
- `deposito`: revisa su dibujo para que no sea un anillo limpio (soporte, tubería o una
  segunda tapa). El del tanque conserva orugas y cañón.
- Prueba: el dibujo de un cráter y el de cualquier objetivo de `OBJETIVOS` no coinciden píxel a
  píxel, y la tinta del cráter no sube del 60 %. **Vuelca el PNG y míralo** con tanques,
  depósitos y cráteres juntos.

## Fase 4. Defensas aéreas visibles y aviso de peligro

### 4.1 Imagen del antiaéreo

- `antiaereo` pasa de 7×7 a unos 11×10: base ancha con una fila de ruedas, **dos tubos de misil
  inclinados** hacia arriba y un **plato de radar** a la izquierda, sobre un mástil. `sprite()`
  ya admite `filas2`: el segundo fotograma gira el plato, y `pintarEntidad` alterna cada 200 ms.
  Es un borrador: dibújalo, vuelca el PNG y ajusta hasta que se lea como lanzador antiaéreo
  y no como la tienda (hueca) ni el depósito (redondo).
- Más grande baja su valor (`sprite()` lo saca del área). Ponle `valorFijo: true` y el valor
  que tiene hoy, como el helicóptero, y conserva el orden por área del test.
- Comprueba con el test de solapes de los estáticos que sigue cabiendo donde está; si no, muévelo
  (con el PNG delante).

### 4.2 Que se vea quién te apunta

- Mientras dura la amenaza el tirador **no desaparece**: alterna entre su dibujo y su negativo
  (`invertir` sobre su caja), con el mismo periodo que hoy (300 → 80 ms).
- **Marco de blanco fijado**: cuatro esquinas de 3 px a 2 px de su caja, parpadeando a la vez, y
  una **línea de puntos** (uno de cada tres píxeles) desde el tirador hasta el VTOL (el centro
  de la vista, donde empieza la mira). Es lo que dice «este te apunta a ti».

### 4.3 Señal de peligro en la mira

- En `dibujar`, si `amenazas.length`, la mira lleva: cuatro esquinas de 4 px a 12 px de su
  centro y un triángulo con «!» (7×6) encima, todo parpadeando con el periodo de la amenaza más
  urgente (el mismo cálculo que el tirador; sácalo a una función `periodoAviso(a)`). Se pinta
  con halo negro para que se lea sobre el terreno. Funciona en `'cayendo'` y en `'explosion'`.
- Pantalla 10: el «!» de hoy (2×5) pasa a un triángulo de peligro de 8×7 que parpadea al mismo
  ritmo. Conviven con el aviso (`+N`, `FALLO`, `TOCADO`) sin taparse.
- Pruebas: con una amenaza activa, el dibujo de la mira difiere del de sin amenaza en algún
  fotograma del ciclo y es idéntico en el otro; sin amenazas, `dibujar` no cambia.

## Fase 5. El misil enemigo

- Cada amenaza gana `lanzaEn = hasta - VUELO_ENEMIGO_MS`, con `VUELO_ENEMIGO_MS = 1800`. Como
  `avisoMs ≥ 3500`, hay **al menos 1,7 s de apunte** (fase 4) antes de que salga nada: primero
  se ve que te apuntan y luego sale el misil. No hace falta tocar `NIVELES`.
- Entre `lanzaEn` y `hasta` hay un misil en vuelo: sale del centro del tirador y llega al VTOL
  (centro de la vista). Su posición es `lerp(origen, destino, k²)` con `k` lineal en el tiempo
  (acelera, como un misil de verdad), **recalculada cada tick** con la cámara actual: el
  vaivén mueve la vista y el misil no puede quedarse atrás.
- Dibujo: cabeza de 2×2 con halo negro y estela de 4 píxeles separados por la trayectoria; chispa
  en la cola. Pasa por los huecos entre pantallas sin dibujarse (es lo que hacen la mira y todo lo
  demás); no es un fallo, pero pruébalo con el PNG: la trayectoria no debe cruzar un hueco
  justo cuando llega.
- **Destruir al tirador con el misil ya en el aire**: hoy cancela la amenaza. Se queda así (es
  el premio por reaccionar), y el misil enemigo **se desvanece con una explosión pequeña en el
  aire** en lugar de desaparecer: reutiliza una explosión de la fase 2.
- Impacto (`impactar`): destello de las nueve pantallas (`invertir`, ~150 ms), texto
  **«TE HAN DADO»** en las pantallas 4 y 5 durante `AVISO_MS` (cada `texto` ocupa una pantalla:
  «TE HAN» / «DADO») y `-1` vida: el cuadradito de vida que se pierde **parpadea 3 veces y
  desaparece** en la pantalla 10 en lugar de quitarse de golpe. `sacudir()` se queda. Si era la
  última vida, el destello va con la derrota de la fase 1 (`culpable` = ese tirador).
- Las vidas siguen siendo comunes a fallos e impactos.
- Decisión que queda **abierta** para el usuario: ¿el misil enemigo va siempre al centro (el
  VTOL), o a donde apunta la mira cuando sale? Hoy no hay forma de esquivarlo; ir al centro lo
  deja así, y es lo que propone este plan. Si se quisiera esquivar, sería otra fase.
- Pruebas: (a) con una amenaza, a `lanzaEn` aparece un misil y a `hasta` hay impacto, no antes;
  (b) a 1 ms de `hasta` el misil está a menos de 3 px del centro de la vista; (c) destruir al
  tirador en vuelo quita el misil y no resta vida; (d) las dos reglas de 4.3 de `PLAN_MISIL.md`
  (una amenaza a la vez, 3 s de espacio) siguen en pie.

## Fase 6. La pantalla 9 que se apaga

**Síntoma:** la 9 se queda apagada y solo vuelve reiniciando el teclado.

**Por qué solo se arregla reiniciando.** `init_all_screens()` es lo único que manda la secuencia
de arranque del SSD1306 y solo se llama al inicio. `paint_screen()` manda posición y datos,
nunca la configuración. Si un glitch deja al controlador en un estado raro (display apagado,
modo de direccionamiento cambiado, bomba de carga caída), nada lo corrige. Y el SPI es solo de
escritura: **el teclado no puede leer el estado de la pantalla**, así que no hay detección
directa; hay que curar a ciegas.

**Sospechas, por orden (sin confirmar):**

1. Temporización del 74HC595. `apply_cs()` usa **un solo `nop`** entre flancos de reloj (~7 ns a
   133 MHz), por debajo del ancho de pulso típico del 74HC595 a 3,3 V (~20-25 ns). La pantalla 9
   es el bit 15 (`CS_BITS`), el primero en salir y el que recorre las dos etapas (U3 → U4): la
   más expuesta. Un CS espurio bajo durante el volcado de otra le mete bytes basura.
2. Ruido o ESD en SCK/MOSI, o caída de tensión con las diez pantallas cambiando (en juego, hasta
   ~120 fps).

**Primero medir (barato):**

- Cuando se apague, mandarle un reinit solo a ella (L3). Si vuelve, era estado del controlador
  y sirve el protocolo de abajo. Si no, es hardware (alimentación o CS).
- Subir los `nop` de `apply_cs()` a 4-5 y jugar unas horas. Si desaparece, era (1).
- Analizador lógico en CS9 durante un juego.

**Protocolo de recuperación, por capas** (de la más barata a la más cara):

- **L1, en cada fotograma:** `paint_screen()` manda `0x20 0x00` antes de `0x21/0x22`. El volcado
  deja de depender del modo de direccionamiento. Cuesta 2 bytes por pantalla.
- **L2, vigilante a ciegas:** en reposo, core1 refresca **una** pantalla cada ~250 ms en
  rotación (cada una, cada ~2,5 s): configuración ligera (`0x8D 0x14`, `0x20 0x00`, `0xA4`,
  `0xA6/0xA7` según inversión, `0xAF`) **sin `0xAE`**, para que no parpadee, y repinta su
  framebuffer entero (core1 solo manda diferencias, así que una pantalla estática no se
  repintaría nunca).
- **L3, reinit de una pantalla:** `reinit_screen(n)` con la secuencia completa y el CS solo de
  esa pantalla, **sin tocar RST** (es común a las diez). Comando serie `OLED_REINIT[:n]`.
- **L4, todo:** `init_all_screens()` completo, a petición (botón «Reiniciar pantallas» en la
  vista Consola) y automático al `GAME:1` y al conectar la app.
- Un contador de reinits por pantalla en el saludo o en `EV:*`, para ver si es siempre la 9.

**Si se toca el firmware** (reglas de sincronización): versión nueva en `include/orby_version.h`,
`OrbyGUI/src/compat.js` (FEATURES) y `docs/COMPATIBILIDAD.md`, en el mismo commit; el comando
nuevo toca `main.cpp`, `src/device.js` y la vista. Pruebas de PC en `tools/test/`. Firmware
publicado siempre como prerelease (ver `CLAUDE.md`).

---

## Orden sugerido

1. Fase 0 (nombre) y fase 1 (derrota): rápidas y sin riesgo.
2. Fase 6 (L1, `OLED_REINIT` y la prueba de los `nop`): el bug molesta ya y no depende del resto.
3. Fases 2 y 3 (explosiones y cráteres): solo dibujo.
4. Fases 4 y 5 (defensas y misil enemigo), que alimentan al culpable de la fase 1.
5. L2/L4 de la fase 6, según lo que dé la medición.

## Trampas

- **`crateres` pasa de `{ x, y }` a `{ x, y, w, h, tipo }`**: busca todos sus usos (el getter
  `crateres` solo cuenta, pero el dibujo y las pruebas lo leen).
- **`camara` indexa el mapa de bits**: entera y topada. Las explosiones y el culpable van en
  coordenadas del **territorio**; al dibujarlas, restar `camara`, como hacen los cráteres.
- **El orden de las llamadas a `aleatorio` cambia los resultados de las pruebas con semilla.**
  Sortea lo nuevo (desfases de explosión) siempre en el mismo punto y revisa qué comprueba una
  prueba antes de cambiarle la semilla.
- **Un bit por píxel**: nada de grises ni colores; la atenuación es trama.
- **`final.mjs` ignora las pulsaciones durante 900 ms** (`ARMA_MS`): sigue valiendo con la derrota
  de 2 s, pero no la acortes.
