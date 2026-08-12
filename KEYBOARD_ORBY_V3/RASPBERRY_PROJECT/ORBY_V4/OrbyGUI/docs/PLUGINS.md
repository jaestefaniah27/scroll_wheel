# Complementos de OrbyGUI

**Un complemento no es un programa: es un fichero.**

Esa es la única frase que hay que retener de este documento. Hasta la versión 1.0 un
complemento era un módulo de Node que OrbyGUI cargaba en su propio proceso y ejecutaba
sin caja de arena: podía leer tus ficheros, abrir sockets y tirar la app entera con una
excepción. Ahora un complemento declara **qué petición mandar** y quien la manda es
OrbyGUI. El complemento no ejecuta nada.

Lo que se gana: se puede leer entero antes de instalarlo, no puede colgar la app, y
escribir uno no exige saber programar. Lo que se pierde: no puedes hacer *cualquier
cosa*. Para lo que de verdad necesita código —hablar con OBS, leer sensores, vivir
dentro de Altium— existe el segundo tipo, `process`, donde el código corre en **su
propio proceso** y habla con OrbyGUI por un protocolo acotado.

| | `http` | `process` |
|---|---|---|
| Qué es | Solo el `plugin.json` | Un ejecutable al lado del manifiesto |
| Para qué | Cacharros con API REST en la red | OBS, sensores, audio, Altium, Chrome |
| Quién ejecuta | OrbyGUI | El propio programa, en otro proceso |
| Estado hoy | **Funciona** | **Sin implementar** (ver el final) |

Ejemplo de referencia, completo y en uso: [`plugins/lampdesk/plugin.json`](../plugins/lampdesk/plugin.json).

---

## 1. Dónde vive

```
%APPDATA%\OrbyGUI\plugins\<id>\plugin.json
```

Fuera de la carpeta de la app **a propósito**: así sobreviven a las actualizaciones, que
ahora se instalan solas y en silencio. El `<id>` es el nombre de la carpeta y tiene que
coincidir con el campo `id` del manifiesto.

Se instalan desde **Ajustes → Complementos**, eligiendo un `.zip` que contenga esa
carpeta. `npm run pack:plugins` genera esos `.zip` a partir de `plugins/` para repartir.

---

## 2. El manifiesto

### Cabecera

```jsonc
{
  "id": "lampdesk",              // minúsculas, dígitos y guiones; 2–39; sin empezar por guion
  "name": "Lámpara LampDesk",    // obligatorio
  "version": "2.0.0",            // libre; si falta, la interfaz enseña «—»
  "description": "…",
  "author": "…",
  "icon": "sun",                 // nombre de icono de src/icons.js; por defecto «plug»
  "apiVersion": 2,               // OBLIGATORIO y tiene que ser 2
  "kind": "http"                 // «http» o «process»; por defecto «http»
}
```

**`apiVersion` no es decorativo.** Un manifiesto con `apiVersion: 1` se rechaza al
cargar, con un mensaje que lo dice: los complementos de la API 1 eran programas de Node
y ya no hay quién los ejecute. Fallar al cargar es a propósito — mejor eso que reventar
a mitad de una pulsación.

La validación entera está en
[`manifiesto.rs`](../src-tauri/crates/orby-core/src/plugins/manifiesto.rs), con sus tests.

### `settings` — lo que configura el usuario

```jsonc
"settings": {
  "description": "Texto que sale arriba de la tarjeta en Ajustes.",
  "fields": [
    {
      "key": "host",                        // con qué nombre se lee: {{settings.host}}
      "label": "Dirección de la lámpara",
      "type": "text",                       // text | number | password | toggle | select
      "placeholder": "192.168.1.35:8080",
      "default": "192.168.1.35:8080",
      "hint": "Ayuda debajo del campo",
      "options": [ { "value": "a", "label": "Opción A" } ]   // solo para «select»
    }
  ]
}
```

Un `type` que no esté en esa lista rechaza el manifiesto entero. Los valores los guarda
OrbyGUI en su configuración local, no en el teclado.

### `request` — la base común

```jsonc
"request": {
  "base": "http://{{settings.host}}",
  "timeoutMs": 4000            // por defecto 5000
}
```

**El tiempo máximo importa más de lo que parece.** Sin él, un cacharro desenchufado deja
la ejecución de una secuencia colgada hasta que expire el TCP: más de un minuto con el
teclado aparentemente muerto. El valor por defecto son 5 s, y se sube cuando se sabe que
el aparato tarda: LampDesk pone 4 s porque cada escritura la hace avisar por HomeKit a
los iPhone emparejados, y esa ida y vuelta le bloquea el bucle principal (lo normal son
20–35 ms, pero el pico medido pasa de dos segundos).

### `actions` — lo que se asigna a una tecla o a un mando

```jsonc
{
  "op": "brightness_delta",              // identificador único dentro del complemento
  "label": "Brillo de la lámpara",
  "targets": ["turn"],                   // key | click | turn; por defecto ["key"]
  "step": 5,                             // cuánto vale una muesca del mando
  "hint": "Texto de ayuda al asignarla",
  "coalesce": { "ms": 120, "mode": "sum" },
  "request": { "path": "/set", "query": { "dbrightness": "{{value}}" } }
}
```

`targets` decide dónde se puede asignar: `key` (una tecla), `click` (pulsar el mando),
`turn` (girarlo). Una acción de giro sin `turn` no aparece en la lista del mando.

Para pedir un número al asignar la acción, en vez de `step`:

```jsonc
"value": { "min": 0, "max": 100, "step": 5, "default": 50 }
```

### `coalesce` — agrupar los giros

El firmware manda **un disparo por cada muesca** del encoder, así que un giro rápido son
veinte en un suspiro. Sin agrupar, eso son veinte peticiones en cola contra un cacharro
que las atiende de una en una: el efecto llega tarde y a tirones.

```jsonc
"coalesce": { "ms": 120, "mode": "sum" }
```

Durante `ms` se acumulan los incrementos y se manda **uno solo** con la suma. Mientras
hay una petición en vuelo se sigue acumulando sin mandar nada, porque encadenar
peticiones cada 60 ms contra algo que tarda 20 ms en resolver una dispara la mediana.

**Solo se agrupan incrementos.** Sumar dos incrementos da el mismo resultado que
aplicarlos por separado; fundir dos «encender/apagar» en uno los anularía entre sí. Por
eso el agrupado se declara **por acción** y no globalmente. Implementación y tests:
[`coalesce.rs`](../src-tauri/crates/orby-core/src/plugins/coalesce.rs).

Un apunte de diseño de LampDesk que conviene copiar: los incrementos se mandan **como
incrementos** (`dbrightness`) en vez de leer el estado, sumar y escribir el resultado.
Así resuelve la suma el aparato, y el mando no se pelea con la app del móvil por quién
tenía el valor bueno.

### `views` — leer y enseñar un valor

```jsonc
"views": [
  {
    "op": "brightness_view",
    "label": "Brillo actual",
    "icon": "sun",                       // por defecto «oled»
    "request": { "path": "/state" },
    "value": "{{response.brightness}}"
  }
]
```

Un complemento con al menos un visor sale con `hasRead: true` en su descriptor, y la
interfaz le ofrece los sitios donde se puede pintar un valor leído.

### `test` — el botón «Probar» de Ajustes

```jsonc
"test": {
  "request": { "path": "/state" },
  "retries": 1,
  "detail": "Responde: brillo {{response.brightness}} %"
}
```

`retries` son reintentos **además** del primer intento. Se permite aquí y **nunca en una
acción**: leer es inofensivo y repetirlo no cambia nada, mientras que reintentar un
incremento lo aplicaría dos veces. Un reintento en el test evita el «no responde» con el
aparato perfectamente vivo, que estaba ocupado atendiendo otra cosa.

### `legacy`

```jsonc
"legacy": { "stepType": "lamp", "configKey": "lamp" }
```

Solo para complementos que existían antes de que hubiera complementos, con su propio
tipo de paso en el editor de secuencias. En uno nuevo, no se pone.

---

## 3. Plantillas

Tres, y solo tres:

| Plantilla | Qué vale | Dónde |
|---|---|---|
| `{{settings.<clave>}}` | Lo que el usuario guardó en Ajustes | En cualquier sitio |
| `{{value}}` | El incremento del giro o el número elegido | En cualquier sitio |
| `{{response.<clave>}}` | Un campo de la respuesta JSON | Solo en `views` y en `test.detail` |

**Sustitución y nada más: ni condiciones, ni aritmética, ni bucles.** No es una
limitación pendiente de levantar, es la línea: en cuanto esto admita expresiones habremos
reinventado un lenguaje de programación sin querer, con su intérprete y sus fallos, y un
complemento habrá dejado de ser un fichero que se puede leer de un vistazo.

Dos detalles que ahorran una tarde de depuración:

- Una plantilla que no se reconoce **falla**, no interpola vacío. Una URL con un hueco en
  blanco la interpreta el aparato del otro lado como le parece, y el usuario solo ve «no
  funciona» sin ninguna pista.
- Los números se escriben sin `.0` sobrante: una API que espera un entero se atraganta
  con `brightness=50.0`, y ese fallo solo se ve con el aparato delante.

Implementación: [`plantilla.rs`](../src-tauri/crates/orby-core/src/plugins/plantilla.rs).

---

## 4. Plantilla para empezar uno nuevo

```jsonc
{
  "id": "mi-cacharro",
  "name": "Mi cacharro",
  "version": "1.0.0",
  "description": "Qué hace, en una línea.",
  "author": "Tu nombre",
  "icon": "plug",
  "apiVersion": 2,
  "kind": "http",

  "request": { "base": "http://{{settings.host}}", "timeoutMs": 5000 },

  "settings": {
    "description": "Dónde está el cacharro en la red.",
    "fields": [
      { "key": "host", "label": "Dirección", "type": "text", "default": "192.168.1.50" }
    ]
  },

  "actions": [
    {
      "op": "toggle",
      "label": "Encender / apagar",
      "targets": ["key", "click"],
      "request": { "path": "/toggle" }
    },
    {
      "op": "nivel_delta",
      "label": "Nivel",
      "targets": ["turn"],
      "step": 5,
      "coalesce": { "ms": 120, "mode": "sum" },
      "request": { "path": "/set", "query": { "dnivel": "{{value}}" } }
    }
  ],

  "views": [
    {
      "op": "nivel_view",
      "label": "Nivel actual",
      "request": { "path": "/state" },
      "value": "{{response.nivel}}"
    }
  ],

  "test": {
    "request": { "path": "/state" },
    "retries": 1,
    "detail": "Responde: nivel {{response.nivel}}"
  }
}
```

Los comentarios `//` no valen en JSON. El truco que usa LampDesk para documentarse por
dentro es meter claves que empiecen por `_comment`: se ignoran al parsear y se leen al
abrir el fichero.

---

## 5. El presupuesto de las OLED

Un complemento puede empujar imágenes a las diez pantallas del teclado. Eso **no desgasta
la Flash**: `OLED_CHUNK` escribe en la RAM del teclado y solo `SAVE_STATE` persiste, que
es la misma vía que usa `src/live-oled.js`. Un panel de temperaturas puede repintarse
todo el día.

Lo que sí es un límite duro es el **ancho de banda**. El CDC va a 115200 baudios —unos
11,5 KB/s— y por ahí pasan **también** las pulsaciones y los giros. Una pantalla son 360
bytes que viajan en cuatro `OLED_CHUNK`: unos 800 bytes por imagen. Salen ~14 pantallas
por segundo *si no circulara nada más*, y si circula, el teclado se vuelve pastoso al
teclear.

| Constante | Valor |
|---|---|
| Bytes por imagen | 800 |
| Imágenes por segundo y complemento | 4 |
| Presupuesto total de los complementos | 3 200 B/s (de 11 520 B/s del enlace) |

Cuatro por segundo es de sobra para un panel de temperaturas o para el parpadeo del REC
de OBS: por encima, el ojo ya no lo distingue en una pantalla de 72×40 y solo gasta cable.

**El arbitraje vive en la app, no en el complemento**, y esto no es negociable: un
programa de un tercero no puede tener la potestad de dejar el teclado lento. Aunque se
porte mal, de ahí no sale más de lo que cabe. De un mismo hueco solo se manda el último
fotograma; los intermedios se tiran. Las constantes se comprueban **en tiempo de
compilación**, así que subir el presupuesto «porque el dashboard va a tirones» no
compila: [`oled.rs`](../src-tauri/crates/orby-core/src/plugins/oled.rs).

Dos reglas más, heredadas de `live-oled.js`: solo se pinta en teclas de la página que el
teclado tiene puesta **ahora mismo**, y **nunca** se fuerza un cambio de página para
pintar de fondo.

---

## 6. Complementos de tipo `process`

> **Sin implementar.** El motor (`src-tauri/src/plugins/proceso.rs`) no existe todavía:
> la Tarea 11 del [PLAN_TAURI.md](PLAN_TAURI.md) está aplazada a propósito, porque no hay
> ningún complemento de proceso que escribir y diseñar el protocolo a ciegas es trabajo
> sin nadie que lo use. Se retomará contra las necesidades reales del primero que toque
> —OBS, monitor de hardware, audio, Chrome o Altium—. Lo que sigue es el diseño acordado,
> y está aquí para que ese complemento se pueda escribir sin volver a abrir OrbyGUI.

Un complemento `process` trae un ejecutable junto al manifiesto. OrbyGUI **no lo carga**:
lo lanza como proceso aparte y habla con él por **JSON, un objeto por línea**. Si se
muere, se muere él solo.

```jsonc
"kind": "process",
"process": {
  "exe": "mi-plugin.exe",     // relativo a la carpeta del complemento
  "mode": "stdio"             // «stdio» o «socket»
}
```

### Modo `stdio`

El habitual. OrbyGUI lanza el ejecutable y le habla por su entrada y su salida estándar.

**App → complemento:**

| Mensaje | Cuándo |
|---|---|
| `hello` | Al arrancar. Lleva la versión de la API y las capacidades del teclado |
| `settings` | Al arrancar y cada vez que el usuario los cambia |
| `run {op, value}` | Se ha pulsado una tecla o girado el mando |
| `read {op}` | Hay que refrescar un visor |
| `test` | El usuario ha pulsado «Probar» en Ajustes |
| `shutdown` | La app se cierra o se desactiva el complemento |

**Complemento → app:**

| Mensaje | Para qué |
|---|---|
| `ready {actions, views}` | Declara sus acciones y visores |
| `event {op, value}` | Algo ha cambiado por su cuenta |
| `setOled {slot, frame}` | Pinta una pantalla (sujeto al presupuesto de arriba) |
| `setLabel`, `setPage`, `setProfile` | Toca el teclado |
| `notify`, `log` | Avisa al usuario / deja rastro |

`ready` **refresca el descriptor en caliente** y avisa al frontend. Esa es la razón de ser
de este tipo: las escenas de OBS no se conocen hasta conectarse, así que no pueden salir
de un manifiesto escrito de antemano.

Un complemento que se muere se marca con su error en la lista y se reintenta con **espera
creciente**, nunca en bucle cerrado. Al cerrar la app se le manda `shutdown`, con matarile
por tiempo si no se va solo.

### Modo `socket`

Para lo que **vive dentro de otro programa** y por tanto OrbyGUI no puede lanzar: una DLL
del SDK de Altium corriendo dentro de Altium, o un Native Messaging Host que arranca
Chrome. Ahí se invierte quién llama: OrbyGUI escucha y el complemento se conecta cuando
existe. El protocolo por encima es exactamente el mismo.

En el caso de Chrome hay un paso extra que no es de OrbyGUI: registrar el host en el
registro de Windows, bajo
`HKCU\Software\Google\Chrome\NativeMessagingHosts\<nombre>`, apuntando a un manifiesto
propio de Chrome que declara qué extensión puede hablar con él. Es cosa del instalador
del complemento, no de la app.

---

## 7. Qué tipo le toca a cada uno

De la lista de [TODO.md](TODO.md):

| Complemento | Tipo | Por qué |
|---|---|---|
| Cacharros con API REST en la red | `http` | No hace falta ejecutar nada |
| OBS Studio | `process` + `stdio` | WebSocket y escenas que se descubren al conectar |
| Monitor de hardware (CPU/GPU/RAM) | `process` + `stdio` | Lee WMI y pinta OLED |
| Audio (Teams / Discord / Spotify) | `process` + `stdio` | API de audio del sistema |
| Altium Designer | `process` + `socket` | Vive dentro de Altium, como DLL del SDK |
| Chrome | `process` + `socket` | Vive dentro de Chrome, como Native Messaging Host |
