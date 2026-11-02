# Consejos en OrbyGUI

## Para qué

El teclado tiene muchas peculiaridades que nadie descubre solo: el doble toque de SUPER, el mando de menú, el zoom como pellizco de touchpad, las macros que viajan en el teclado... Hoy solo se sabe que existen leyendo la documentación. La idea, como los consejos de Apple: de vez en cuando la app enseña un aviso del tipo «¿Sabías que...?»; si se pulsa, lleva a un apartado **Consejos** donde están todos, con un vídeo corto cada uno y un acceso directo para activar o configurar eso mismo en el momento.

## Objetivos

- Botón de bombilla en la barra superior (junto a tema, perfiles y ajustes) que abre la vista Consejos.
- Avisos ocasionales dentro de la app, con el mismo aspecto que los `toast` actuales. Pulsarlos abre ese consejo.
- Cada consejo: título, texto corto, vídeo de unos segundos y, si corresponde, un botón de acceso directo que lleva a donde se activa o configura.
- Consejos sobre todas las peculiaridades del teclado y de la app (lista inicial más abajo).
- Los vídeos los graba el equipo tocando el teclado. Mientras no existan, el consejo sale con un hueco «Vídeo próximamente».

## Fuera de alcance

- Notificaciones del sistema de Windows (el plugin existe, pero no se usa aquí). Solo avisos dentro de la app.
- Consejos que cambien la configuración por sí solos. El acceso directo **lleva** al ajuste; no lo cambia.
- Estadísticas o telemetría de qué consejos se leen.
- Traducción: todo en castellano, como el resto de la app.

## Anti-objetivos (sería un fallo aunque «funcione»)

- Que un aviso interrumpa una escritura a Flash, una grabación o un cambio sin guardar.
- Que el mismo consejo salga dos veces, o que salgan seguidos varios.
- Que el instalador crezca por los vídeos, o que la app se vea más pesada o lenta.
- Que se sugiera algo que el teclado conectado no puede hacer (firmware viejo).

## Decisiones tomadas por el usuario

- El aviso es **como los avisos de la propia app**, no una notificación de Windows.
- **Todo consejo con algo que activar o configurar lleva acceso directo**, para hacerlo nada más leerlo.
- Vídeos de **baja resolución y tamaño pequeño en pantalla**, a propósito, para que no se note la calidad.
- Los vídeos **viven en git** y la app los **descarga una sola vez** (al instalar), no van dentro del instalador.

## Diseño

### Datos de los consejos
Un módulo `src/tips.js` con la lista. Cada consejo:

| Campo | Qué es |
|---|---|
| `id` | estable, también nombra el vídeo (`<id>.mp4`) |
| `categoria` | Teclas y menús · Rueda y mandos · Macros · Perfiles · Pantallas · App y teclado |
| `titulo`, `texto` | el texto del consejo (el aviso usa el título como pregunta: «¿Sabías que...?») |
| `requiere` | clave de `FEATURES` en `compat.js` (p. ej. `helpMode`); si el teclado conectado no la tiene, no se avisa y en la lista sale atenuado con «Necesita firmware X» |
| `accion` | acceso directo: `{ etiqueta, vista, destino }` (ver abajo), o ninguno si no hay nada que activar |

### Vista Consejos
Vista nueva `view-tips` (`src/views/tips.js`), registrada en `VIEWS`, `VIEW_TITLES` y con el botón `btn-tips` (bombilla) en la barra, conmutador como Ajustes y Perfiles. Tarjetas agrupadas por categoría, cada una con su vídeo pequeño (silenciado, en bucle, sin controles, ~320 px de ancho), el texto y el botón de acceso directo. Los ya vistos se marcan discretamente.

### Acceso directo
`accion.vista` es una vista de `main.js` y `accion.destino` lo que esa vista sabe abrir. Para Ajustes se generaliza el patrón que ya existe en `settings.openTarget` (`section: 'menukey'` hace `scrollIntoView` sobre una tarjeta): pasar a aceptar cualquier id de tarjeta. Para los consejos de Inicio o Perfiles, el mismo mecanismo (`openTarget`) que usa hoy la réplica. Hay consejos sin acceso directo posible (p. ej. «el doble toque de SUPER entra en el modo ayuda» es un gesto del teclado); su botón puede ser «Activar el modo ayuda» si hay un ajuste, o no existir.

### Cuándo salta un aviso
Reglas por defecto (decisión mía, ajustables):
- Como mucho **uno al día** y no antes de **2 minutos** de uso tras abrir la app.
- Solo con la ventana visible, el teclado conectado, **sin cambios sin guardar**, sin grabación en curso y sin otro `toast` en pantalla.
- Se elige el primer consejo no visto cuyo `requiere` cumple el teclado, en el orden de la lista.
- Se guardan en la configuración local del PC (`config_set`): ids vistos, fecha del último aviso, y `consejos: true/false`.
- Interruptor «Mostrar consejos» en Ajustes → General.
- El aviso dura más que un toast normal (~8 s), lleva la bombilla y se cierra con una X; pulsarlo abre la vista Consejos en esa tarjeta y lo da por visto.

### Vídeos
- **En git:** carpeta `OrbyGUI/tips-video/` en el repo, con un `manifest.json` (`id`, fichero, bytes, sha256). Vídeo mp4 H.264, ≤ 480 px de ancho, 5-10 s, sin audio, objetivo ≤ 300 KB cada uno.
- **Descarga única:** al primer arranque tras instalar (y cuando el manifiesto cambie) la app baja en segundo plano los que falten a `%APPDATA%\OrbyGUI\tips\`, comprueba el sha256 y los sirve en local. Un vídeo ya descargado y válido no se vuelve a bajar. Sin red, los consejos salen con el hueco de «Vídeo próximamente» y se reintenta en el siguiente arranque.
- **Lógica probable en `orby-core`** (qué falta, qué cambió, validar el manifiesto), con tests; `src-tauri/src/` solo hace la red y el disco, como el resto.
- **Contrato `window.orby`:** métodos nuevos para pedir la ruta/URL de un vídeo y lanzar la sincronización; hay que tocar las tres vías (`orby-tauri.js`, `electron/preload.js`, `orby-web.js`) y pasar `test/superficie-orby.test.mjs`. En la WebGUI no se descarga nada: el vídeo se ve directo desde la URL cruda de GitHub.
- Vídeo que falta (`<id>.mp4` aún no grabado, y no figura en el manifiesto): hueco «Vídeo próximamente».

## Lista inicial de consejos (se afina al escribirlos)

| Consejo | Acceso directo |
|---|---|
| Doble toque de SUPER = modo ayuda (cada tecla enseña su nombre) | Ajustes → Pantallas: cuánto dura |
| SUPER + tecla de menú: retroceder página; mantener: menú principal | — |
| Mando de menú: mantener, girar y soltar para elegir perfil | Ajustes → Controles (mando de menú) |
| El zoom de la rueda es un pellizco de touchpad | Inicio con la rueda en zoom |
| Rueda magnética: calibrarla | Ajustes → Controles |
| Páginas dentro de un perfil | Perfiles → páginas |
| Las macros viajan en el teclado a otro PC | Perfiles → macros |
| Escribir un texto largo con una tecla | editor de secuencias |
| Grabadora de ratón y teclado | grabadora |
| Cambio automático de perfil según la aplicación | Cambio automático |
| Biblioteca de perfiles (Altium, KiCad, VS Code, Office) | Perfiles → Biblioteca |
| Perfiles desactivados (guardar más de 9) | Perfiles |
| Iconos propios y catálogo | editor de icono |
| Elegir la intro de arranque (o una al azar) | Ajustes → Pantallas |
| Velocidad de las animaciones | Ajustes → Pantallas |
| Invertir el giro de mandos y rueda | Ajustes → Controles |
| Copias de seguridad | Ajustes → Copias |
| Complementos | Ajustes → Complementos |
| Actualizar el firmware sin pulsar BOOTSEL | Ajustes → Dispositivo |

## Para quien construya

- Seguir las convenciones de `CLAUDE.md`: todo en castellano, sin framework ni TypeScript, colores solo por tokens de `:root` (con valor en claro y oscuro), `ui.js` para avisos, comentarios que expliquen el por qué.
- Los textos de cada consejo y su orden los escribe quien construya a partir de la lista y de `compat.js`/`COMPATIBILIDAD.md`; deben ser breves y verificables contra el comportamiento real del firmware.
- Probar cada acceso directo de verdad en la app (con `npm run tauri:dev`), no solo leer el código.
- Los vídeos reales los graba el usuario: el código debe funcionar con **ningún vídeo** presente.
- Las tareas de grabar y editar los vídeos van al tablero del servidor (orvy.es/admin/tablero), no a este repo.

## Cosas que decido yo (revisables)

- Cadencia: 1 al día, 2 min tras abrir, en el orden de la lista.
- Descarga de vídeos al primer arranque en segundo plano (el instalador NSIS no descarga nada por sí solo).
- Hosting en `raw.githubusercontent.com` del repo `scroll_wheel`, como `compat.json`.
- Categorías y límite de ≤ 300 KB por vídeo.
