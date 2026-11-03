# Plan: Misil, campaña de seis zonas

Plan para implementar por fases. Cada fase deja el juego jugable y `npm test` en verde
(desde `OrbyGUI/`: `node --test test/*.mjs`). **No empieces una fase sin cerrar la anterior.**

Ficheros que se tocan:

| Fichero | Qué tiene |
|---|---|
| `src/games/misil.mjs` | el juego: objetivos (`OBJETIVOS`), movimiento, VTOL, disparo, dibujo |
| `src/games/misil-mapa.mjs` | el territorio: tamaño, `ZONAS`, carreteras (`NODOS`, `TRAMOS`), `ESTATICOS`, dibujo fijo (`construirMapa`) |
| `test/games-misil.test.mjs` | sus pruebas (33 hoy). Las que este plan rompe a propósito se **actualizan**, no se borran |
| `tools/misil/ver-mapa.mjs` | **nuevo** (fase 0): vuelca el mapa a PNG para verlo |

Reglas de la casa que valen para todo el plan (ver `CLAUDE.md` del proyecto):

- Todo en castellano de España: código, comentarios, textos y commits.
- Los comentarios explican **por qué**, nunca qué. Mira la cabecera de `misil.mjs`: cada decisión
  nueva va a su lista «Decisiones» con el motivo, y la «Fase 2/3» de la cabecera se reescribe para
  que describa el juego tal y como queda.
- Nada de `Math.random()` en `misil.mjs`: todo el azar sale de `aleatorio(tope)` (entero en
  `[0, tope)`), que es lo que hace las pruebas reproducibles. Para un real en [0, 1):
  `aleatorio(1000) / 1000`.
- Sin dependencias nuevas.

---

## Qué ha pedido el usuario (resumen)

1. Los vehículos no se acaban: siguen apareciendo siempre.
2. Los vehículos no se atraviesan. Hay convoyes que van en fila sin pisarse.
3. El mapa pasa de 3 a 6 zonas, en **ruta lineal**, y la partida acaba al terminar la sexta.
   Las variaciones del mapa quedan para más adelante (**no** entran en este plan).
4. Cada zona es más difícil que la anterior.
5. La estancia en cada zona es de **40 s fijos**. Desaparece la condición de soltar un mínimo de
   misiles (`DISPAROS_PARA_VOLAR`): la gracia es apuntar y disparar rápido para aprovechar el tiempo.
6. Desde la zona 3 el enemigo dispara al VTOL.
7. Coches patrulla (vehículo nuevo, rápido).
8. Cazas de combate, quietos, delante de los hangares.
9. A veces hay un helicóptero en la H del helipuerto: al segundo de llegar el VTOL a la zona,
   despega y se va. Si lo aciertas, muchos puntos.
10. Vaivén del VTOL: pequeños deslizamientos aleatorios aunque esté parado, que obligan a
    corregir la puntería. Así los soldados, que dan muchos puntos, cuestan más.
11. Desde la zona 5, camiones lanzamisiles: un camión **cualquiera** de los que circulan se sale
    de la carretera, aparca en un sitio fijo, despliega el lanzador, apunta y dispara. No se sabe
    cuál ni cuándo.
12. En la zona 6, soldados con RPG que también disparan.
13. Soldados que se mueven, solos o en grupo: bajan de vehículos y suben a ellos, salen de los
    edificios y entran en ellos.

### Fallo de hoy que conviene entender

En una partida el usuario lo destruyó todo y el VTOL siguió rotando entre zonas sin nada que hacer.
La causa: `hayObjetivosEn` solo mira lo que cabe en la vista de cada zona, y los vehículos que
circulan pueden estar en tramos de carretera que no ve ninguna zona. Entonces `llegar()` ve la zona
vacía y vuelve a volar, y así sin fin. Y la partida no acaba porque `entidades.length` no es 0.
La fase 1 (aparición continua) y la fase 3 (ruta lineal con final) lo resuelven entre las dos.
No hace falta un parche aparte.

---

## Estado

**Hecho (Opus):** la fase 0 y todo el mapa de la fase 3.2. `misil-mapa.mjs` ya tiene las seis
zonas en `ZONAS` (con `helipuerto` donde hay), `MAPA_W = 1200`, la red dirigida en `TRAMOS` y
`SALIDAS` (`ADYACENTES` sigue, sin dirección, solo hasta que la fase 1 lo quite), los estáticos de
las zonas nuevas, cazas y antiaéreos incluidos, `APARCAMIENTOS` (con `EXPLANADA`), `SENDAS` y el
quinto campo de `ESTATICOS` para los vehículos que arrancan (5.3). Los dibujos `caza`, `antiaereo`
y `rpg` ya están en `OBJETIVOS`. **No muevas nada del mapa** salvo que un test lo exija, y si lo
haces, vuelca el PNG y míralo.

Diferencias con lo que dice el plan más abajo, porque manda el código:

- Solo hay **una** explanada por zona (puesto, puente, aeródromo y cuartel), y ninguna en el oasis:
  junto a una carretera horizontal no caben (ver el comentario de `APARCAMIENTOS`). Lanzadores y
  desembarcos comparten la de su zona. Si hay un lanzador aparcado, la explanada está ocupada.
- `SENDAS` lleva `fin` (`'quedarse' | 'entrar' | 'subir'`) además de `de`.
- Los vehículos que arrancan son dos: el camión del cuartel y el del aeródromo. Su senda es la que
  tiene `fin: 'subir'` en su zona, y acaba junto a ellos.

**Hecha la fase 1 (Sonnet):** sentido único (`ADYACENTES` fuera, `SALIDAS`), `luego`, `espacioDelante`,
convoyes (`armarConvoy`, `circular`), aparición continua y la patrulla (`sprite(filas, filas2)`). Desviaciones:

- `TRAFICO` es una constante provisional que la fase 4 sustituye por `NIVELES`; no hay multiplicador de
  velocidad (`factorVelocidad` fuera, cada tipo va a la suya).
- Los convoyes entran **cerca de la vista**, no en cualquier tramo: a más de 20 px y a menos de 80 del
  rectángulo de la cámara, y con un camino que llega a la vista en menos de 220 px (`armarConvoy` sortea
  4 cruces de antemano para poder saberlo). `maxVehiculos` cuenta solo los de menos de 200 px; los de más
  de 400 se borran.
- **El tope y el temporizador no esperan si la vista se queda vacía** (ni hay nadie a punto de entrar en
  ella, a menos de 40 px de camino): los circuitos son cortos y los vehículos lentos (el tanque, 12 px/s) dan
  vueltas cerca durante minutos, así que con el tope lleno la zona 1 pasaba 15-20 s sin nadie. Por eso
  puede haber hasta `maxVehiculos + convoyMax` vehículos cerca. Con esto, en 40 s sobre la base hay
  vehículos a la vista el 95 % del tiempo o más (varias semillas).
- Huecos en las esquinas: cuando un vehículo gira respecto al de delante el hueco suma también los
  anchos (`(w + w' + h + h') / 2 + 1`), no solo `HOLGURA`: con solo el largo las cajas se rozaban en la curva.
- `crear` admite `inmortal` (solo para pruebas: los fallos no gastan vidas), para medir un minuto de tráfico.
- `estadisticas.suma` ya lleva `patrulla`; el resto de etiquetas sigue siendo de la fase 6.
- Los tests `macro-prune` y `macro-store-sync` fallan con Node 20 (`Unexpected token 'export'`); piden Node 22
  y no tienen que ver con Misil.

**Hechas las fases 2 y 3.1 (Sonnet):** cazas (ya estaban), helicóptero, vaivén, ruta lineal con final.
Desviaciones:

- `HELI_PROB = 0.5` y `VAIVEN_PX = [2, 3, 4, 5, 6, 7]` (por zona) son constantes provisionales; la fase 4 las mete en `NIVELES`.
- `crear` admite además `quieto` y `zonaInicial` (solo pruebas). En el test, `nueva()` pasa `quieto: true` por defecto;
  las pruebas del vaivén lo desactivan.
- El helicóptero lleva `valorFijo: true` en `OBJETIVOS` (el test de «vale más cuanto más pequeño» lo salta) y `volador: true`
  en la entidad. Se borra al salir de la vista; un posado que queda lejos se borra con el tráfico (`LEJOS`). No cuenta para
  `hayObjetivos`. Su aspa gira con el mismo `filas2` de la patrulla (cada 200 ms).
- `TRANSICION_MS = 3500` es ahora el mínimo; la duración real es `duracionVuelo(zona)` = `max(3500, distancia · 12)`.
  El test del vuelo suave recorre de la zona n a la n+1.
- `estadisticas.suma` lleva `helicoptero` y `max` lleva `zona` (la zona más lejana, de 1 a 6). `victoria` es un getter más.
- Pantalla 10 en vuelo: `Z<n>` en lugar del marcador, y sin marco ni barra (las vidas siguen).


**Hecha la 5.1 (Sonnet, rama `misil-pie`):** entidad a pie (`e.pie = { ruta, s, tope, alFinal }`), `proximoPie` y
`lanzarPie`/`andar` en `misil.mjs`. Desviaciones:

- `PIE_MS` es una constante provisional por zona (índice = zona); la fase 4 la mete en `NIVELES`. Va a `tipo.velocidad`
  sin `nivel().vel`, que aún no existe. `tipoAPie()` siempre da soldado: ahí entran las RPG de 4.5.
- `crear` admite `pieMs` y `zonaInicial`, solo para pruebas.
- Los que acaban con `'quedarse'` se paran en fila contra el final de la senda (6 px entre sí), no todos en el mismo punto.
- Los que caminan no cuentan en `hayObjetivosEn`. Se ignoran las sendas `de: 'aparcamiento'` y `fin: 'subir'` (5.2 y 5.3).
- Test de `SENDAS` añadido: pasan todas tal como están, el mapa no se ha tocado.


**Hecha la fase 4 (Sonnet, rama `misil-fuego`):** `NIVELES` (con `simultaneas`) sustituye a `TRAFICO`; `vel` es el
cuarto argumento de `circular` (factor), `puntos` multiplica cada acierto. Antiaéreo y RPG llevan `dispara: true` (en vez
de `tirador`) y `crearEstatica` lo copia a `e.tirador`. Amenazas, `tiradores()`, camión lanzamisiles (`lanzador`, valor
2 × camión puesto a mano tras `TIPO`) y las dos reglas de espacio van en `misil.mjs`. Desviaciones:

- `crear` admite `zonaInicial` e `inmortal` también vale para los impactos (solo para pruebas). Getters nuevos para
  probar: `amenazas`, `impactos`, `vidas`.
- El impacto llama a `sacudir()`, vacía hasta que el vaivén (2.3) la llene. En vuelo se descartan las amenazas.
- `ultimoImpacto` es el del último impacto **real**: una amenaza cancelada no cuenta para el espacio de 3 s.
- Con el temporizador a punto y sin hueco (`simultaneas` llenas) el tirador espera su turno; sin candidatos, se
  vuelve a sortear.
- El camión asignado conserva la clave `mision` (`red`, `entra`, `despliega`, `listo`); al salir pierde `desde`/`hasta`
  y su convoy, así que `vehiculos()` ya no lo cuenta. El RPG que camina (fase 5) y `nivel().rpg` quedan sin usar.
- `heli`, `vaiven` y `pieMs` están en la tabla pero no se usan aún. El test de orden de valor por área excluye el
  `lanzador`, que vale el doble por decisión.


**Juntadas (Opus):** las ramas de 2+3.1, 4 y 5.1. Al juntarlas, `HELI_PROB`, `VAIVEN_PX` y `PIE_MS` pasan a `nivel().heli/vaiven/pieMs`, `sacudir()` da el golpe de deriva (±10, ±6) y los grupos a pie de la zona 6 llevan un RPG de cada cuatro, que pasa a tirador al pararse.

**Hechas la 5.2 y la 5.3 (Sonnet, rama `misil-bajar`):** `v.mision = { ap, luego, fase, t }` con `luego` en
`'lanzar' | 'desembarcar' | 'arrancar'` (la clave es `ap`, no `aparcamiento`, como ya usaban el lanzador y sus tests).
`candidatos`, `asignarMision`, `salirDeLaRed`, `incorporar` y `salir` son comunes; `lanzarPie` elige al azar entre
edificio, desembarco y subida. Desviaciones:

- El desembarco lo pueden hacer camiones y patrullas; al aparcar se gira ya (fase `baja`, `DESEMBARCO_MS`) y sale recto
  hacia la carretera de cara a ella. Una explanada con misión (lanzador aparcado incluido) no se vuelve a usar.
- Volver a la red o arrancar solo ocurre si hay hueco (`holgura` por pares con el resto); si no, espera en el eje.
- `crearEstatica` copia el quinto campo a `e.arranque`. La senda `subir` solo se sortea si ese vehículo sigue vivo, quieto
  y sin otro grupo yendo; el vehículo es el de `arranque` más cercano al último punto de la senda. Suben todos a la vez
  cuando llega el último; si destruyen el vehículo antes, el grupo se queda en fila (`'quedarse'`).
- Al arrancar se gira hacia el eje en que más le falta (el aparcamiento de ambos está de lado respecto a la carretera) y
  sin cambios de eje: sale recto, se incorpora en `{ tramo, dist }` y gira de golpe al tramo.
- Los vehículos que vuelven a la red no llevan convoy: `llegaALaVista` ya lo admite.
- Tests existentes tocados: el del vaivén desactiva disparos y lanzadores (el impacto sacude más que la amplitud y
  dependía de la semilla); el de «zonas sin disparo» ya no prohíbe `mision` a secas, solo la de lanzar.

**Pendiente:** fase 6.

## Fase 0. Herramienta para ver el mapa

Sin ver el mapa no se puede diseñar ninguna zona nueva. Crea `tools/misil/ver-mapa.mjs`
(Node puro, sin dependencias) que:

- importa `construirMapa`, `ESTATICOS`, `ZONAS`, `MAPA_W`, `MAPA_H` de `../../OrbyGUI/src/games/misil-mapa.mjs`
  y `OBJETIVOS`, `orientar`, `MUNDO_W`, `MUNDO_H`, `HUECO_X`, `HUECO_Y` de `misil.mjs`;
- pinta el mapa a escala ×2, blanco sobre negro, con los estáticos encima (con su halo negro,
  como `pintarEntidad`) y el borde de la vista de cada zona en gris, con los huecos entre pantallas
  marcados;
- escribe un PNG en la ruta que reciba como argumento. El PNG se arma a mano: firma, `IHDR`,
  `IDAT` con `zlib.deflateSync` y `IEND`. Para el CRC de cada bloque usa `crc32` de
  `OrbyGUI/src/web/macro-blob.mjs` (ya existe, no lo reescribas). Escala de grises de 8 bits.

Uso: `node tools/misil/ver-mapa.mjs <ruta>.png`, y luego abrir el PNG con la herramienta de leer
ficheros, que enseña las imágenes. **Cada vez que toques el dibujo del mapa, vuelca y mira.**

---

## Fase 1. Tráfico: sentido único, sin solapes, convoyes, aparición continua, patrulla

### 1.1 Carreteras de sentido único

La carretera mide 9 px (`SEMI = 4`) y el tanque girado mide 9 de ancho: **no caben dos carriles**.
Por eso la red pasa a ser de sentido único y nunca hay dos vehículos de frente.

En `misil-mapa.mjs`, `TRAMOS` pasa a ser **dirigido**: `'pq'` significa «de p a q». Los dos
circuitos de hoy ya encajan en un solo sentido (comparten `c→b`):

```js
// circuito del puesto: a→f→h→c→b→a; el de la base: b→g→q→d→c→b
export const TRAMOS = ['af', 'fh', 'hc', 'cb', 'ba', 'bg', 'gq', 'qd', 'dc'];
```

- `ADYACENTES` pasa a llamarse `SALIDAS` y solo guarda las salidas: `SALIDAS[p] = [q, ...]`.
- `SEGMENTOS` no cambia: `segmento()` ya ordena los extremos, así que el dibujo no se entera.
- En `avanzarVehiculo` desaparece el «salvo volver por donde vino»: se elige entre `SALIDAS[v.hasta]`.
- Actualiza los tests de callejones y de «no se da la vuelta» para que comprueben la red dirigida:
  cada nodo tiene al menos una salida y una entrada, y desde cualquier nodo se llega a todos
  los demás (un recorrido en anchura desde cada nodo).

### 1.2 Cada vehículo decide su siguiente tramo antes de llegar al cruce

Añade `v.luego` al vehículo: el nodo al que irá después de `hasta`. Se elige **al entrar** en un
tramo, no al salir. Hace falta para mirar qué hay delante pasado el cruce.

### 1.3 No pisarse

Función nueva en `misil.mjs`: `espacioDelante(v, vehiculos)`. Devuelve los px que puede avanzar
`v` sin acercarse a otro más de lo permitido. Usa la distancia **por la carretera** entre centros.
`largo(t)` es la longitud del tramo y `restante(v) = largo(tramo de v) - v.dist`.

Para cada otro vehículo `o`:

- en el mismo tramo y por delante (`o.dist > v.dist`): distancia `o.dist - v.dist`;
- en el tramo siguiente (`o.desde === v.hasta && o.hasta === v.luego`): `restante(v) + o.dist`;
- en otro tramo que llega al mismo cruce (`o.hasta === v.hasta` y no en el mismo tramo), y más
  cerca del cruce que `v` (`restante(o) < restante(v)`, o empate y `o` creado antes): cuenta como
  si fuera delante, a `restante(v) - restante(o)`. Así se resuelven las incorporaciones, sin
  semáforos.

El hueco que hay que respetar es `(largo de v + largo de o) / 2 + HOLGURA` (`HOLGURA = 4`). El largo
de un vehículo es el `w` de su dibujo sin girar (`v.tipo.w`): los dibujos miran a la derecha.
El avance del tick pasa a ser `Math.max(0, Math.min(deseado, distancia - hueco))` con la menor
distancia encontrada. En `moverMundo` se mueven en orden de creación (da igual cuál, mientras sea
siempre el mismo).

Prueba: 2000 ticks con 9 vehículos y convoyes; en ningún tick se solapan las cajas de dos
vehículos (la misma comprobación de solape que el test de los estáticos).

### 1.4 Convoyes

Un convoy es `{ camino: [nodo, nodo, ...] }` compartido por sus vehículos. Cada vehículo lleva
`v.convoy` y `v.paso`, su posición en `camino`. Al elegir `luego` (1.2):

- si `convoy.camino[v.paso + 2]` existe, lo toma (sigue al de delante);
- si no, lo elige al azar entre `SALIDAS[v.hasta]` y lo añade a `camino`.

No hay un «líder» explícito: el primero que llega a un cruce sin decidir es el que decide. Si
destruyen al primero, el siguiente pasa a decidir él solo, sin código aparte.

Todo el convoy va a la velocidad de su vehículo más lento. Si no, el tanque frena al camión y
el convoy se desordena.

### 1.5 Aparición continua

Fuera `Array(4).fill(...)` de `poblar()`: al empezar ya no hay vehículos sueltos. Un temporizador
`proximaLlegada` mete un convoy cada `NIVELES[zona].llegadaMs`, siempre que haya menos de
`NIVELES[zona].maxVehiculos` en el mapa. Para meterlo:

- tramo al azar, **dirigido**;
- el punto de entrada no puede verse desde la cámara (con 20 px de margen): que no aparezca de
  la nada delante del jugador;
- tiene que haber sitio para todo el convoy (los huecos de 1.3) dentro de ese tramo. Si no cabe,
  se prueba en el siguiente tick (no hay bucle de reintentos);
- tamaño del convoy al azar entre 1 y `NIVELES[zona].convoyMax`. El tipo de cada vehículo se
  sortea: tanque o camión, y patrulla con probabilidad `NIVELES[zona].patrulla`. Las patrullas
  van **solas**, nunca en convoy (son las rápidas).

Los vehículos aparcados de `ESTATICOS` se quedan como están: son estáticos.

`velocidadPorAcierto` desaparece: la velocidad la marca la zona (fase 4).

### 1.6 Coche patrulla

Objetivo nuevo en `OBJETIVOS`, que circula. Dibujo mirando a la derecha, 9×5, con la barra de
luces en el techo:

```js
{
  // Más corto que el camión y con la barra de luces encima: lo que más corre del mapa.
  nombre: 'patrulla', velocidad: 34,
  ...sprite([
    '.######..',
    '#########',
    '##.#.####',
    '#########',
    '.######..',
  ]),
  // Las luces parpadean: el segundo fotograma cambia la barra.
  filas2: ['.######..', '#########', '###.#.###', '#########', '.######..'],
},
```

`sprite()` no conoce `filas2`. Añade a `sprite()` un segundo argumento opcional con las filas del
segundo fotograma, que pase por el mismo `padEnd`. A `orientar()`, que también lo gire: extrae la
trasposición y el volteo a una función que reciban `filas` y úsala para las dos. En `pintarEntidad`,
si la forma trae `filas2`, se pinta uno u otro según `Math.floor(tiempo / 200) % 2`. El helicóptero
(fase 2) usa lo mismo.

---

## Fase 2. Cazas, helicóptero y vaivén

### 2.1 Cazas en los hangares

Objetivo quieto nuevo, `caza`, de ala delta y vista cenital, con el morro a la derecha (11×11):

```js
'..##.......',
'..###......',
'...###.....',
'#..####....',
'##########.',
'###########',
'##########.',
'#..####....',
'...###.....',
'..###......',
'..##.......',
```

En `ESTATICOS`, dos cazas delante de los hangares de la base (`hangar(b, 74, 288, 24, 14)` y
`hangar(b, 104, 288, ...)`), entre el hangar (acaba en y=302) y la pista (empieza en y=334),
con el morro hacia la pista (`'aba'`). El test «lo que hay quieto no se solapa» manda: si choca
con el soldado de (112, 322), mueve el soldado. Vuelca el mapa (fase 0) y míralo.

### 2.2 Helicóptero

Objetivo nuevo `helicoptero`. Dibujo de 13×13: el rotor es una cruz grande que en `filas2` va en
aspa, para que se vea girar. Fuselaje macizo en el centro y cola hacia la izquierda. **Valor fijo**
`VALOR_HELI = 150`, que se pone sobre lo que dé `sprite()` (que lo topa en `VALOR_MAX`).

- Cada zona puede tener `helipuerto: [cx, cy]` en `ZONAS` (la base: `[158, 292]`, el centro de
  `helipuerto(b, 158, 292)`).
- Al llegar a una zona con helipuerto (y también al empezar la partida, que es «llegar» a la 1),
  con probabilidad `NIVELES[zona].heli` se crea el helicóptero posado, centrado en la H.
- Entidad nueva de tipo «volador»: `{ tipo, forma, x, y, vx, vy, vel, despegaEn }`. No usa
  `desde`/`hasta`, así que `moverMundo` la mueve aparte. `vel = Math.hypot(vx, vy)` para el bono
  de velocidad de `estallar`, que ya existe.
- Posado `HELI_POSADO_MS = 1000`. Luego despega hacia un ángulo al azar, acelerando
  (`HELI_ACEL = 45` px/s², tope `HELI_VEL = 70` px/s). Se orienta (`orientar`) según el eje en que
  más se mueve.
- Cuando su caja sale de la vista de la cámara por completo, se borra: se ha ido.
- No cuenta para `hayObjetivosEn` (que no retenga al VTOL en una zona vacía) ni deja cráter si
  ya está volando: hacer el cráter solo si `vel === 0`.

Pruebas: (a) con `heli = 1` aparece en la H y a los 1000 ms empieza a moverse; (b) a los pocos
segundos ya no está en `entidades`; (c) un misil sobre él posado suma al menos `VALOR_HELI`.

### 2.3 Vaivén del VTOL

La cámara deja de estar clavada en la zona: `camara = zona + deriva`.

- `deriva = { x, y }` (reales) persigue a `objetivo = { x, y }` con un suavizado exponencial:
  `deriva.x += (objetivo.x - deriva.x) * (1 - Math.exp(-dt / VAIVEN_TAU))`, con `VAIVEN_TAU = 450`.
- Cada `1500 + aleatorio(2500)` ms se sortea un objetivo nuevo dentro de `±NIVELES[zona].vaiven` px
  en cada eje.
- La cámara usa `Math.round` (indexa el mapa de bits) y se topa para no salirse del mapa.
- La mira **no** se mueve con la deriva: está en coordenadas de la vista, así que el suelo se
  desliza por debajo, que es justo lo que se busca.
- Durante el vuelo se sigue sumando: no estorba.
- **Para las pruebas**: `crear({ ahora, aleatorio, quieto })`. Con `quieto: true` la amplitud es 0.
  Pásaselo en el `nueva()` del test a las pruebas que apuntan a un objetivo concreto. Sin eso
  fallarían por azar. Es una opción solo para pruebas: dilo en un comentario de una línea.
- Actualiza el test «parado, la cámara está en su zona»: ahora es «a menos de `vaiven` + 1 px».

Prueba nueva: sin `quieto`, parado en una zona, la cámara cambia al menos una vez en 10 s y nunca
se aleja más de la amplitud.

---

## Fase 3. Mapa de seis zonas en ruta lineal

### 3.1 La ruta y el final

- `trayecto(zona, k)` deja de dar la vuelta: de la zona `n` a la `n + 1`. El `% ZONAS.length` se va.
- `debeVolar()` pasa a ser `estancia >= ESTANCIA_MS || !hayObjetivosEn(...)`.
  **Fuera `DISPAROS_PARA_VOLAR`** (constante, contador `disparosZona`, su getter y su test).
  `quedaParaVolar()` solo mira el tiempo.
- Al cumplirse `debeVolar()` en la **última** zona, la partida acaba con victoria: `fase = 'final'`
  y `puntos += vidas * BONO_VIDA` (`BONO_VIDA = 200`). Lo mismo si la última zona está vacía.
- El «se acaba si `entidades.length === 0`» de `tick` se va: con tráfico continuo ya no pasa nunca.
- Nueva estadística `max: [['zona', 'Zona más lejana']]` (la que se alcanzó, de 1 a 6) en
  `estadisticas` y en `detalle`. `scores.mjs` las trata por clave: no hay que tocarlo, pero
  compruébalo con grep.
- En la pantalla 10, mientras vuela, en lugar del marco que parpadea: el número de la zona a la
  que va (`texto(lienzo[9], 'Z' + n)` durante el vuelo y el marcador al llegar).

### 3.2 El mapa crece hacia el este

`MAPA_W` pasa de 600 a **1200**; `MAPA_H` sigue en 440. Las tres zonas de hoy no se mueven. Las
nuevas van en x ≥ 600. La vista mide `MUNDO_W × MUNDO_H` = 248×148.

| # | `nombre` | Qué tiene | Dispara |
|---|---|---|---|
| 1 | `base` | la de hoy + 2 cazas + helipuerto | no |
| 2 | `puesto` | la de hoy | no |
| 3 | `puente` | la de hoy + 2 antiaéreos | sí |
| 4 | `oasis` | lagunas con palmeras, tiendas, soldados, camiones, 2 antiaéreos | sí |
| 5 | `aerodromo` | pista enemiga, 3 hangares con 4 cazas, helipuerto, depósitos, 3 antiaéreos | sí |
| 6 | `cuartel` | gran recinto amurallado, tanques en fila, búnkeres, helipuerto, 3-4 antiaéreos | sí |

Orden de vuelo: base (abajo a la izquierda) → puesto (arriba a la izquierda) → puente (centro) →
oasis (arriba al este) → aeródromo (abajo al este) → cuartel (al fondo, centro-este). Los vuelos
largos están bien: el terreno pasa por debajo, pero `TRANSICION_MS` debe crecer con la distancia
para que la velocidad no se dispare. Pon `TRANSICION_MS` por distancia: `max(3500, distancia * 12)`
ms. El test de «el vuelo es suave» sigue valiendo.

Reglas para dibujar las zonas nuevas (las mismas que cumplen las de hoy, y las vigilan los tests
que ya existen):

- solo con las funciones de dibujo que ya hay: `colina`, `duna`, `roca`, `palmera`, `recinto`,
  `valla`, `nido`, `torre`, `pista`, `hangar`, `helipuerto`, `carreteras`. Si te hace falta otra
  (por ejemplo una laguna, o un búnker de trazo), hazla de **trazo**, nunca maciza: lo macizo con
  halo negro es lo destruible (ver la cabecera de `misil-mapa.mjs`);
- el río se alarga hacia el este: añade puntos al final de su `curva(...)` y quita el
  `MAPA_W + 40` del final;
- las carreteras siguen siendo horizontales y verticales, de sentido único (1.1), sin callejones,
  con cada carretera en el centro de una fila de pantallas y no en un hueco. Las del este se
  conectan con la red de hoy saliendo de `q` o de `d` y volviendo a la otra, como un circuito más;
- entre 8 y 20 estáticos a la vista de cada zona (el test ya lo exige), alejados del borde de su
  pantalla (`MARGEN`) y nunca en un hueco entre pantallas;
- la tinta del mapa entero, por debajo del 8 % (test de «no es ruido»);
- **vuelca el PNG y míralo zona a zona antes de dar la fase por cerrada.**

---

## Fase 4. Dificultad por zona y fuego enemigo

### 4.1 La tabla

En `misil.mjs`, una sola tabla, indexada por zona (0 a 5). Todo lo que varía con la zona sale de
aquí. Ningún número suelto en el código:

```js
// Lo que cambia de una zona a la siguiente. Cada fila aprieta un poco más en todo a la vez:
// subir una sola cosa (la velocidad, por ejemplo) se notaba como un salto, no como una escalada.
export const NIVELES = [
  //  vel   llegadaMs maxVeh convoyMax patrulla heli vaiven puntos disparoMs avisoMs lanzadorMs rpg pieMs
  { vel: 1.0, llegadaMs: 8000, maxVehiculos: 4, convoyMax: 1, patrulla: 0,   heli: 0.5, vaiven: 2, puntos: 1.0, disparoMs: 0,     avisoMs: 0,    lanzadorMs: 0,     rpg: false, pieMs: 0 },
  { vel: 1.15, llegadaMs: 7000, maxVehiculos: 5, convoyMax: 2, patrulla: 0.2, heli: 0,   vaiven: 3, puntos: 1.2, disparoMs: 0,     avisoMs: 0,    lanzadorMs: 0,     rpg: false, pieMs: 20000 },
  { vel: 1.3, llegadaMs: 6000, maxVehiculos: 6, convoyMax: 2, patrulla: 0.3, heli: 0,   vaiven: 4, puntos: 1.4, disparoMs: 12000, avisoMs: 5000, lanzadorMs: 0,     rpg: false, pieMs: 15000 },
  { vel: 1.5, llegadaMs: 5000, maxVehiculos: 7, convoyMax: 3, patrulla: 0.4, heli: 0,   vaiven: 5, puntos: 1.6, disparoMs: 10000, avisoMs: 4500, lanzadorMs: 0,     rpg: false, pieMs: 12000 },
  { vel: 1.7, llegadaMs: 4000, maxVehiculos: 8, convoyMax: 3, patrulla: 0.5, heli: 0.6, vaiven: 6, puntos: 1.8, disparoMs: 8000,  avisoMs: 4000, lanzadorMs: 15000, rpg: false, pieMs: 10000 },
  { vel: 1.9, llegadaMs: 3500, maxVehiculos: 9, convoyMax: 4, patrulla: 0.6, heli: 0.7, vaiven: 7, puntos: 2.0, disparoMs: 6000,  avisoMs: 3500, lanzadorMs: 12000, rpg: true,  pieMs: 8000, simultaneas: 2 },
];
```

`simultaneas` vale 1 si la fila no lo pone. **`avisoMs` nunca baja de 3500**: es apuntar más los
1,6-1,8 s de caída con los dos mandos, y puede pillarte con otro misil en el aire. Test: ninguna
fila tiene `avisoMs` entre 1 y 3499.

(`heli` solo vale en zonas con `helipuerto`; en las demás da igual lo que ponga. `lanzadorMs`
y `pieMs` a 0 quieren decir «en esta zona no».) Dentro de la partida,
`const nivel = () => NIVELES[zonaIdx]`.

- `vel`: sustituye a `factorVelocidad()`. `e.vel = e.tipo.velocidad * nivel().vel`.
- `puntos`: multiplica `ganados` en `estallar`, antes del `Math.round`.
- Test: hay tantas filas como `ZONAS`, y cada columna de las de «más difícil» (`vel`,
  `maxVehiculos`, `vaiven`, `puntos`) nunca baja de una fila a la siguiente.

### 4.2 Antiaéreo

Objetivo quieto nuevo, `antiaereo`: un parapeto cuadrado con dos cañones hacia arriba. No se
orienta. Su silueta no se confunde con el depósito, que es redondo, ni con la tienda, que es hueca:

```js
'..#.#..',
'..#.#..',
'..#.#..',
'#######',
'#.###.#',
'#.###.#',
'#######',
```

Lleva `dispara: true`. Se coloca en `ESTATICOS` de las zonas 3 a 6 (ver 3.2). Antes de colocarlos
en el puente, comprueba que caben con el test de solapes.

### 4.3 El disparo

Solo con el VTOL **parado**. Estado nuevo: `amenazas = [{ e, hasta }]`.

**Tiene que dar tiempo a responder.** Con los dos mandos pulsados el misil tarda 1,6-1,8 s en caer,
y la amenaza te puede pillar con un misil a medias sobre otra cosa. Por eso hay dos reglas, que
mandan sobre todo lo demás de esta fase (lanzadores y RPG incluidos):

- **Una sola amenaza a la vez**, salvo en la zona 6, donde puede haber 2 (`NIVELES[z].simultaneas`:
  1 en todas menos la última).
- **Al menos `ESPACIO_DISPAROS_MS = 3000` entre un disparo y el siguiente**, contando el momento en
  que cada uno impactaría (su `hasta`). Toda amenaza nueva lleva
  `hasta = Math.max(tiempo + avisoMs, ultimoHasta + ESPACIO_DISPAROS_MS)`, siendo `ultimoHasta` el
  mayor `hasta` de las amenazas activas, o el del último disparo si ya no queda ninguna. Así, en la
  zona 6, la segunda amenaza llega al menos 3 s después de la primera, y nunca en el mismo instante.
- Si no se puede crear una amenaza ahora (porque ya hay `simultaneas` activas), el tirador espera
  su turno. **No se descarta**.

Prueba: en una partida larga en la zona 6 sin tocar nada, nunca hay más de 2 amenazas, y dos
impactos seguidos están siempre separados 3 s o más. En la zona 5, nunca más de 1.

**Tirador** es cualquier entidad viva, a la vista y con `e.tirador === true`. Lo son los antiaéreos
desde que se crean, los camiones lanzadores al desplegar (4.4) y los soldados con RPG (4.5). Una
función `tiradores()` los devuelve. Así el disparo no sabe de tipos.

- Cada `disparoMs` (temporizador `proximoDisparo`, que arranca al llegar a la zona; si `disparoMs`
  es 0 no corre), se elige al azar un tirador que no esté ya en `amenazas` y se añade con las
  reglas de arriba. Si no hay ninguno, no pasa nada: destruirlos todos es la forma de librarse.
- Mientras dura su amenaza, el tirador parpadea: se pinta un tick sí y otro no, cada vez más
  rápido (periodo de 300 ms al principio y 80 ms al final). En la pantalla 10 se pone un `!`
  parpadeando junto a los cuadraditos de las vidas, si hay al menos una amenaza.
- Si se destruye antes de `hasta`, su amenaza se quita, y el aviso suma `+N` como cualquier acierto.
- Si llega `hasta` con él vivo, **impacto**: `vidas--`, aviso `'TOCADO'`, y un golpe de vaivén
  (`deriva.x += ±10`, `deriva.y += ±6`, al azar) para que se note. Si las vidas llegan a 0, la
  partida acaba como con los fallos.
- Las vidas son las mismas que las de los fallos («misiles de margen»): el marcador ya las enseña.

Pruebas: (a) en la zona 1 nunca hay amenaza; (b) en una zona con disparo y un antiaéreo a la vista,
sin tocar nada, a los `disparoMs + avisoMs` se pierde una vida; (c) destruirlo durante el aviso
la cancela. Para llegar a la zona 3 sin esperar dos estancias: exporta solo para pruebas una forma
de empezar en otra zona, `crear({ ..., zonaInicial })`, con su comentario de una línea.

### 4.4 Camión lanzamisiles (zonas 5 y 6)

Lo que asusta es no saber cuál ni cuándo: cualquier camión de los que circulan puede serlo, y
hasta que se sale de la carretera parece uno más.

**Mapa.** Nuevo en `misil-mapa.mjs`: `APARCAMIENTOS`, explanadas junto a la carretera, 2 o 3 en
cada una de las zonas 5 y 6:

```js
// { tramo: 'pq', dist, x, y }: el punto del tramo (a `dist` de p) donde se sale, y la esquina
// de arriba a la izquierda donde se queda aparcado.
export const APARCAMIENTOS = [ /* ... */ ];
```

Cada uno se dibuja como un acceso corto, igual que los de `ACCESOS`: una carretera que se pinta
pero no se recorre, perpendicular al tramo, de unos 14 px, con la explanada al final (un rectángulo
de trazo con `marco`). Añade un test: cada aparcamiento cae en un tramo que existe, a la vista de
su zona, y su explanada no pisa ningún estático.

**El juego.** Temporizador `proximoLanzador`, que se sortea entre `0.5·lanzadorMs` y
`1.5·lanzadorMs` (ahí está el «no sabes cuándo»). Al saltar:

1. Se buscan los camiones **en circulación** cuyo tramo actual o `luego` lleve a un aparcamiento
   libre de esta zona que todavía no hayan pasado. Se elige uno al azar y se le apunta el
   aparcamiento (`v.mision`). Si no hay ninguno, se vuelve a sortear el temporizador.
2. Al llegar a `dist` en ese tramo, el camión **deja la red**: se quita de su convoy (los de
   detrás siguen; el hueco de 1.3 deja de contarlo porque ya no está en ningún tramo) y avanza en
   línea recta por el acceso hasta la explanada, de cara al acceso (`orientar`).
3. Aparcado, **despliega**: `LANZADOR_DESPLIEGUE_MS = 2000`. A mitad cambia su dibujo por el del
   lanzador: un objetivo nuevo, `lanzador`, que es el camión con la caja sustituida por una rampa
   de dos tubos que sobresalen. Es el único aviso de que viene algo.
4. Al acabar de desplegar, `e.tirador = true` y se pone el primero de la cola, sin esperar a
   `proximoDisparo`: en cuanto las reglas de 4.3 lo permitan (que no haya otra amenaza, o 2 en la
   zona 6, y con los 3 s de espacio), entra en `amenazas`. Mientras espera, se le ve desplegado y
   quieto: es tiempo extra para destruirlo. Si no se le destruye, impacto (4.3). Después sigue
   siendo tirador y vuelve a disparar por el turno normal de `disparoMs`.

Hay que destruirlo en cualquiera de los pasos. Desplegado vale el doble que un camión
(`valor` del `lanzador` = 2 × el del camión, puesto a mano como el del helicóptero).

Pruebas: (a) con `lanzadorMs` pequeño, en la zona 5, algún camión acaba aparcado y desplegado;
(b) sin tocar nada, a los `LANZADOR_DESPLIEGUE_MS + avisoMs` de aparcar se pierde una vida;
(c) en las zonas 1 a 4 nunca sale de la carretera ningún camión.

### 4.5 Soldados con RPG (zona 6)

Objetivo nuevo, `rpg`: el soldado con el tubo al hombro. Tiene que verse distinto del soldado de
3×4:

```js
'.#..',
'####',
'.#..',
'#.#.',
```

- Lleva `tirador: true`. Entra en el turno de `disparoMs` como los antiaéreos.
- Hay unos pocos fijos en `ESTATICOS` de la zona 6 (3 o 4, en nidos y torres).
- Con `nivel().rpg`, uno de cada cuatro soldados que se mueven (fase 5) es un RPG. Uno que camina
  **no dispara** hasta pararse: así se le puede cazar antes, si te das prisa.
- Pequeño y móvil: vale más que nada salvo el helicóptero. Su valor sale de `sprite()`, como el del
  soldado, sin tocar.

Prueba: en la zona 6, un RPG a la vista acaba en `amenazas`; en las zonas 1 a 5 nunca hay RPG.

---

## Fase 5. Soldados que se mueven

Hasta ahora el soldado tiene `velocidad: 7` pero siempre es estático. Aquí pasan a moverse a pie,
**fuera de la carretera**, en tres situaciones. Hazlas en este orden y cierra cada una (con su test)
antes de la siguiente: la tercera es la más delicada.

**Entidad «a pie».** `{ tipo, forma, x, y, vel, ruta: [[x, y], ...], i, alFinal }`: va en línea
recta de un punto de `ruta` al siguiente, a `tipo.velocidad · nivel().vel`. No se orienta (el
soldado es simétrico). `alFinal` dice qué pasa al llegar: `'quedarse'` (se vuelve estático),
`'entrar'` (desaparece sin dar puntos: se ha escapado) o `'subir'` (ver 5.3). `moverMundo` la mueve
aparte, como al helicóptero. Un grupo son 2-4 soldados con la misma ruta, separados 6 px a lo largo
de ella (el segundo empieza 6 px detrás del primero, y así).

**Mapa.** Nuevo en `misil-mapa.mjs`: `SENDAS`, caminos a pie de cada zona:

```js
// { zona, de: 'puerta' | 'aparcamiento' | 'estatico', puntos: [[x, y], ...] }
```

Sin dibujo propio: el camino no se ve, se ve al que camina. Test: ningún punto de una senda pisa
la carretera (`enCarretera`, con margen 2) ni un estático (salvo su extremo, cuando es la puerta de
un edificio o el lado de un vehículo), y todos caen a la vista de su zona.

**Ritmo.** Temporizador `proximoPie`, sorteado entre `0.5·pieMs` y `1.5·pieMs` (no corre si
`pieMs` es 0). Al saltar se elige al azar una de las situaciones posibles en esta zona.

### 5.1 Salir de un edificio y entrar en otro

Senda de una puerta (la del recinto del puesto, un hangar, una tienda) a otra o a un punto
cualquiera. El grupo aparece en la puerta y recorre la senda. Al llegar, `'entrar'` si acaba en otra
puerta y `'quedarse'` si no. Test: el grupo aparece, se mueve, y al acabar o no está o está quieto.

### 5.2 Bajar de un vehículo

Usa los `APARCAMIENTOS` de 4.4 (y hay que poner alguno en las zonas 2 a 4: allí no hay lanzadores,
pero sí desembarcos). Igual que en 4.4, se elige un camión o una patrulla en circulación, sale de la
red y aparca. Pero no despliega: al parar, salen 2-4 soldados por un lado y recorren una senda que
empieza en ese aparcamiento. El vehículo espera `DESEMBARCO_MS = 3000` y vuelve a la red por el mismo
acceso, marcha adelante: al aparcar se gira para quedar de cara a la salida, así que nunca va marcha
atrás. Se reincorpora con las mismas reglas de hueco que una aparición (1.5); si no hay sitio,
espera.

Reutiliza el código de 4.4 para salir de la red y aparcar: es el mismo, con otro final (`v.mision`
lleva `{ aparcamiento, luego: 'lanzar' | 'desembarcar' }`).

Test: el vehículo aparca, salen soldados, y el vehículo vuelve a circular.

### 5.3 Subir a un vehículo

Un grupo camina hasta un vehículo **aparcado de los estáticos** (los de la base, el puesto o el
cuartel) y, al llegar el último, desaparecen (`'subir'`). Desde ese momento el vehículo es uno que
circula. Para eso, a esos estáticos se les añade en `ESTATICOS` un quinto campo con el aparcamiento
por el que salen. Solo los que lo tengan pueden arrancar. Sin ese dato, que se quede quieto.

Test: tras subir el grupo, el vehículo deja de ser estático y acaba en un tramo de la red.

---

## Fase 6. Cierre

- Cabecera de `misil.mjs`: reescribe «Fase 2» y «Fase 3» describiendo el juego tal y como queda
  (seis zonas, final, tráfico, vaivén, fuego). Añade a «Decisiones» los porqués de este plan: el
  sentido único, que la estancia es solo de tiempo, la tabla `NIVELES`, que el helicóptero no
  retiene la zona, y que las vidas son compartidas.
- Cabecera de `misil-mapa.mjs`: «tres zonas» pasa a seis, en ruta lineal, y la red es de sentido único.
- `descripcion` del juego (`export default` de `misil.mjs`): añade los cazas, el helicóptero, la
  patrulla, que se acaba en la sexta zona y que desde la tercera te disparan.
- `estadisticas.suma`: añade `caza`, `patrulla`, `helicoptero`, `antiaereo`, `lanzador` y `rpg`
  con sus etiquetas.
- `node --test test/*.mjs` en verde, y una partida de verdad con el teclado (lo prueba el usuario).
- La versión de la app (tres ficheros) y la publicación **solo si el usuario lo pide**: ver
  `docs/PUBLICACION.md`.

Un commit por fase, en castellano, con el formato de los últimos
(`feat(misil): ...`).

---

## Trampas

- **`camara` es la base del índice del mapa de bits** (`pintarMapa`): si no es entera, se lee
  basura. Siempre `Math.round` y topes contra `MAPA_W - MUNDO_W` y `MAPA_H - MUNDO_H`.
- **`construirMapa` tiene caché por número de cajas**, no por contenido. Si cambias `ESTATICOS`,
  la caché ya no vale entre pruebas solo si cambia la cuenta. No te fíes de ella al depurar.
- **El orden de las llamadas a `aleatorio` cambia los resultados de las pruebas con semilla.** Si
  una prueba con semilla deja de pasar tras un cambio que no la afecta, quizá solo ha cambiado
  qué sale del azar. Revisa qué comprueba antes de cambiar la semilla.
- **Las pruebas que apuntan a un objetivo** (`apuntar` en el test) suponen una cámara quieta y
  objetivos en su sitio. Llevan `quieto: true`, y ojo con el tráfico nuevo, que puede pasar por
  encima del objetivo.
- **No escribas `rgba`, ni colores, ni nada de CSS**: aquí todo es un bit por píxel.
