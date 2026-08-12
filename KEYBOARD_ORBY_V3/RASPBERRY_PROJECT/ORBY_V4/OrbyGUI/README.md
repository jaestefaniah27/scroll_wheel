# OrbyGUI — panel de control del Orby V4

Aplicación de escritorio (Tauri 2 + Vite: cáscara en Rust, interfaz en JS vanilla)
para configurar el teclado Orby V4 en caliente por USB CDC.

---

## 1. Cómo abrir la app

**La forma fácil: instalarla.** El `.exe` que sale de `npm run tauri:build` (o el de
la última release) la deja en el menú de inicio, y desde ahí se actualiza sola.

Para tocar el código, doble clic en `OrbyGUI.bat`: instala las dependencias si
faltan y arranca en modo desarrollo con recarga en caliente.

Desde la terminal:

| Comando               | Qué hace                                                    |
|-----------------------|-------------------------------------------------------------|
| `npm run tauri:dev`   | Vite + la cáscara de Rust, con recarga (equivalente al `.bat`) |
| `npm run tauri:build` | Instalador NSIS en `src-tauri/target/release/bundle/nsis/`   |
| `npm run build`       | Solo genera `dist/`                                          |
| `npm test`            | Los tests del renderer y del contrato `window.orby`          |

Requiere **Node.js 20.19 o superior** (Vite 7 no arranca por debajo) y **Rust**
(https://rustup.rs) para la cáscara. La primera compilación de Rust tarda varios
minutos; las siguientes son incrementales.

> **`tauri build` falla si la app está abierta**, con un `os error 32` que no dice
> cuál es el problema: Cargo no puede sobrescribir el `.exe`. Ciérrala antes desde
> el menú de la bandeja.

La app no descarga nada de internet: tipografía del sistema e iconos SVG
incrustados. Funciona sin conexión.

### Conexión

Al arrancar busca cada 3 s un puerto serie cuyo `vendorId` sea `cafe` o cuyo
`productId` sea `4005` / `4006`, y le manda `ACK`. Si el dispositivo contesta
`ORBY_V4:...`, se conecta y sincroniza tantos perfiles como diga el teclado
(`STATE:PROFILES:<cuántos>:<máximo>`).

### Copia local, huellas y modo lectura

La app guarda en el PC una copia completa de la configuración del teclado
(`deviceMirror` en el archivo de ajustes; lo escribe `src/mirror.js`). Sirve
para dos cosas:

- **Abrir la app sin el Orby enchufado** y ver perfiles, páginas, atajos e
  iconos. En **modo lectura**: sin el teclado delante no se deja cambiar nada,
  porque entonces habría dos versiones distintas de la misma configuración y al
  reconectar alguien tendría que perder sus cambios. Se avisa con un cartel fijo
  abajo a la izquierda, y cualquier intento de edición lo corta
  `requireDevice()` (`src/ui.js`) antes de tocar el modelo.

- **No descargarlo todo en cada conexión.** El teclado resume su configuración
  con `GET_HASH` (un CRC32 por perfil y uno global) y la app calcula la misma
  huella sobre su copia (`src/hash.js`). Solo se piden con `GET_PROFILE` los
  perfiles cuya huella no coincide. Antes se leían siempre los perfiles enteros
  y sus veinte iconos uno a uno.

La serialización sobre la que se calcula el CRC está escrita dos veces, en
`include/config_hash.h` (firmware) y en `src/hash.js` (app).
`tools/test/test_config_hash.cpp` + `.mjs` comprueban que las dos dan lo mismo;
si dejasen de coincidir, la app volvería a descargarlo todo sin que nadie se
enterase.

Nada más sincronizar, la app se trae **todos** los iconos de **todas** las
páginas de **todos** los perfiles con `GET_OLED_PG`, en segundo plano. Así
cambiar de perfil o de pestaña ya no dispara veinte idas y vueltas por el CDC ni
deja los huecos en blanco un segundo.

> El PID del firmware 2.0 es **0x4006**. Se cambió a propósito: Windows cachea el
> descriptor de informe HID por VID/PID, y sin cambiarlo seguiría usando el
> descriptor viejo, sin scroll de alta resolución.

---

## 2. Las vistas

### Dashboard
Réplica del hardware con la misma disposición que el teclado: las doce teclas a
la izquierda y, a su derecha, los mandos — los dos encoders arriba y la rueda
magnética, un disco grande, debajo. Las teclas se iluminan al pulsarlas, los
encoders parpadean y el disco de la rueda gira los mismos grados que la rueda
real. El panel de estado indica el perfil activo, si SUPER está pulsada, la
sensibilidad que se está aplicando y si Windows ha activado el **scroll de alta
resolución**.

### Perfiles y macros
El editor de verdad, y el único sitio donde se configura un perfil entero:
teclas, mandos giratorios y rueda de scroll. Ya no hay datos duplicados en la
app: todo se lee del firmware.

1. Elige el perfil arriba (el marcado *EN USO* es el activo en el teclado).
2. Cambia entre capa **NORMAL** y **SUPER** (la capa SUPER se activa con la tecla 12).
   El interruptor afecta a **todo**: teclas, mandos y rueda.
3. Pulsa cualquier tecla de la réplica para abrir el inspector. Cada tecla
   muestra su **icono real** —el mismo que está en la pantalla OLED— o su
   etiqueta de texto, más el atajo que ejecuta.
4. En el inspector puedes:
   - Ver la pantalla tal cual y saltar al editor con **Editar icono**, que abre
     el editor de iconos con ese perfil, esa tecla y esa capa ya seleccionados.
   - Editar la **etiqueta OLED** (máx. 7 caracteres), que es lo que se muestra
     cuando la tecla no tiene icono.
   - Marcar **modificadores** (Ctrl, Shift, Alt, Win…).
   - Elegir la **tecla** de la lista.
   - O pulsar **Capturar atajo** y teclear la combinación directamente en el PC.
   - O asignar una **acción multimedia** (play/pausa, siguiente, calculadora…).

Cada cambio viaja al teclado al instante y se ve en las pantallas OLED.

**Variaciones por aplicación.** Bajo las pestañas hay una barra *Según la app*.
Sirve para lo que no merece un perfil entero: si en tu editor «seleccionar todo»
es **Ctrl+E** en vez de Ctrl+A, no dupliques el perfil — crea una **variación**.

1. Pulsa **Nueva variación**: se crea ya apuntando a la aplicación que tengas
   delante.
2. Añade **todas las aplicaciones** que quieras que la disparen — el mismo
   retoque suele valer para varias. Escríbelas y pulsa *Añadir* (o Enter), usa
   *La app de delante*, y quítalas con la ✕ de cada etiqueta. Basta con que
   encaje una.
3. Cambia solo las teclas o mandos que difieran. Salen marcados en ámbar, y el
   inspector te recuerda qué hace el perfil base, con **Volver al valor base**.
4. Al ponerte delante de cualquiera de esas apps, el teclado aplica los cambios
   encima del perfil; al salir, vuelve solo.

Guarda **únicamente las diferencias**: si luego retocas el perfil base, la
variación hereda el cambio en todo lo que no haya redefinido. Puede cambiar
atajos de teclas, acciones de los mandos y etiquetas OLED (no la calibración de
la rueda ni los iconos).

> Es un ajuste del PC: se escribe en la RAM del teclado al vuelo y **nunca** en
> Flash. Al pulsar *Guardar en Flash* la app quita la variación, guarda el perfil
> base y la vuelve a poner, para que no se cuele en el perfil guardado. Necesita
> el detector de aplicaciones activado (interruptor de *Cambio automático*).

**Crear, duplicar y eliminar perfiles.** A la derecha de las pestañas hay tres
botones y un contador de huecos (`4 / 16`). *Nuevo* añade un perfil vacío,
*Duplicar* copia el actual **con sus iconos**, y *Eliminar* lo borra (siempre
tiene que quedar uno). El teclado admite hasta **16 perfiles**; al borrar, los
siguientes se desplazan y el menú físico del teclado se ajusta solo.

> En el menú del teclado (mantener los dos encoders, o la tecla 10) los perfiles
> se recorren con el encoder izquierdo, uno por pantalla, y las teclas 1-5 son
> atajos directos a los cinco primeros de la ventana. Con más de diez perfiles
> la lista se muestra por ventanas que siguen al cursor, y al abrir el menú este
> arranca sobre el perfil que esté puesto.

> **El tope son 16 por la RAM del teclado.** Cada perfil reserva 7,4 KB para sus
> 20 iconos (10 pantallas × 2 capas), así que el banco ocupa 116 KB de los 264 KB
> del RP2040. Para pasar de ahí habría que dejar de tener los bitmaps en RAM y
> leerlos directamente de la Flash mapeada. Si lo que necesitas son retoques por
> aplicación, casi siempre salen más a cuenta las **variaciones**.

**Mandos giratorios.** Debajo de las teclas hay un bloque con los dos encoders y
la rueda de scroll, en el mismo orden que en el teclado. Cada uno tiene sus
huecos configurables:

| Mando | Huecos |
|---|---|
| Encoder izquierdo | Giro horario, giro antihorario, pulsación |
| Rueda de scroll | Hacia abajo, hacia arriba |
| Encoder derecho | Giro horario, giro antihorario, pulsación |

Tipos de acción disponibles:

- **Multimedia / sistema** — volumen, silencio, brillo, pistas, navegador,
  calculadora, buscar, zoom…
- **Atajo de teclado** — cualquier combinación, también con captura en vivo.
- **Desplazar vertical / horizontal** y **Zoom (Ctrl + rueda)**.
- **Un complemento instalado**, si ofrece algo para ese hueco (ver *Complementos*).

Desplazar y zoom llevan el sentido en el signo del giro, así que una sola acción
cubre las dos direcciones y la app espeja el par automáticamente. Las
pulsaciones no los ofrecen: un clic no tiene dirección.

**Complementos.** Cada complemento instalado añade su propio tipo de acción con
lo que sepa hacer en ese hueco. Lo que se mueve por pasos (un brillo, un
volumen) es cosa del giro: cada muesca mueve un paso y el signo lo pone el hueco
—horario sube, antihorario baja—, así que basta con elegir lo mismo en los dos
sentidos. Si el mando está montado del revés, *Invertir giro* le da la vuelta al
par entero. En la pulsación del encoder solo salen las órdenes sueltas
(encender, apagar…), que es lo único que tiene sentido sin giro; para una tecla
están en su pestaña *Multimedia*.

> Solo **Desplazar vertical** aprovecha la alta resolución de la rueda
> magnética. El resto de acciones son discretas y trabajan por clics completos.

> Los encoders ejecutan estas acciones únicamente en modo normal. Dentro del
> menú del teclado siguen sirviendo para navegar.

**Los mandos también tienen capa SUPER.** Cada perfil guarda dos juegos de
acciones giratorias: uno para la capa normal y otro para cuando mantienes la
tecla 12. Al crear o duplicar un perfil los dos empiezan iguales, así que si no
tocas nada el comportamiento es el de siempre; en cuanto cambias uno, ese
encoder hace dos cosas distintas dentro del mismo perfil.

**Rueda de scroll.** Abajo del todo está la calibración de la rueda magnética,
que también pertenece al perfil **y a la capa**: cuántos clics equivale una
vuelta completa (preajustes Preciso 12, Suave 30, Estándar 60, Rápido 120), la
inversión de dirección, las cifras derivadas y un dial con el giro en vivo para
comprobarlo sin salir de la app. Un mismo perfil puede desplazar fino en normal
y a saltos largos con SUPER.

**Marcador en pantalla.** El dial sigue el **ángulo absoluto** del sensor, así
que se queda donde esté la rueda de verdad en lugar de volver al cero al
soltarla. Como el imán y el marcador de la tapa se montan en cualquier ángulo,
la tarjeta trae su calibración:

- **Círculo o raya** para el marcador.
- **Invertir giro**, si el dibujo gira al revés que la rueda.
- **Desfase** en grados, con barra y botones de ±1°.
- **«El marcador está arriba: alinear aquí»**: pon el circulito de la tapa
  mirando hacia arriba, pulsa, y el de la pantalla se coloca en el mismo sitio.

Es un ajuste de montaje, así que se guarda en el PC junto al resto de la
configuración local y vale para el dial y para la rueda del dashboard.

### Iconos OLED
Editor de mapas de bits 72×40 de 1 bit, a pantalla completa.

**Elegir destino.** El panel derecho es una réplica del teclado, no una lista de
números: cada tecla muestra el icono que tiene cargado ahora mismo (leído del
firmware con `GET_OLED`) o su etiqueta de texto, más el atajo que ejecuta. Así
sabes qué vas a pisar antes de tocarlo. Las teclas 10 (SUPER) y 12 (menú) salen
atenuadas porque no tienen pantalla. Al seleccionar una tecla se carga su icono
actual en el lienzo para poder editarlo encima.

**Dibujar.**
- Lápiz / borrador / relleno. Clic derecho borra.
- Marco, invertir, deshacer, vaciar.
- **Zoom de 3× a 28×**: botones `−` / `+`, ajustar a la ventana, o
  **Ctrl + rueda** sobre el lienzo (mantiene quieto el píxel bajo el cursor).
  Con la rueda sola se desplaza el lienzo cuando no cabe entero.

**Biblioteca de iconos.** Lo primero del panel derecho: 216 iconos vectoriales
ya dibujados (flechas, multimedia, edición, sistema, programación, comunicación
y varios), con buscador —que ignora acentos, así que *camara* encuentra
*Cámara*— y filtro por categoría. Pinchas en uno y entra como capa flotante,
igual que una imagen importada, para colocarlo a tu gusto antes de fijarlo. No
hace falta que nadie se dibuje sus propios PNG para empezar.

Están escritos a mano en `src/icon-library.js` como cadenas SVG sobre una
rejilla de 24×24, no en `node_modules` ni en ningún CDN: la app sigue
funcionando sin conexión. Como son trazos vectoriales limpios, entran con el
suavizado a 0 (a diferencia de una foto importada, que arranca en 1).

**Colocar imagen o texto.** Ni las imágenes ni el texto se pegan de golpe: se
convierten en una **capa flotante** con recuadro punteado que puedes ajustar
antes de fijarla.

- Arrastra dentro del recuadro para moverla; tira de la esquina inferior derecha
  para escalarla.
- **Cruceta** de botones para mover 1 px exacto, con centrar en medio. Las
  **flechas del teclado** hacen lo mismo (con Shift, 5 px).
- **Enter** fija la capa, **Esc** cancela.
- **Tamaño en píxeles**, de 2 a 120 px de alto, con botones de ±1 px. Se mide en
  píxeles de la pantalla y no en porcentaje del original a propósito: un
  porcentaje tiene un rango útil distinto según la fuente (un PNG de 800 px
  encaja al 5 %, un texto de 16 px necesita el 170 %), así que la barra quedaba
  casi toda inservible. El tope de 120 px son tres veces el alto del panel.
- **Umbral** de blanco/negro, **suavizado de bordes** (0–3) y difuminado.
- Al fijar puedes **sustituir** el lienzo o **combinar** con lo ya dibujado.

**Invertir** tiene tres modos, y ninguno toca nada fuera de la capa:

| Modo | Qué hace | Cuándo |
|---|---|---|
| No | Sin cambios | — |
| Colores | Voltea el color del contenido y respeta la transparencia | Iconos PNG negros sobre alfa, que sobre el fondo negro del OLED no se verían |
| Recuadro | Voltea el rectángulo entero de la capa: fondo encendido, contenido apagado | Texto o etiquetas en negativo |

Para invertir el lienzo completo está el botón *Invertir* de la barra de
herramientas, que es otra cosa.

Para el texto puedes elegir entre 11 fuentes del sistema (Segoe UI, Impact,
Consolas, Arial Black…), tamaño y negrita, y cambiarlas con la capa ya colocada.

> **Sobre los bordes feos.** El difuminado Floyd–Steinberg viene desactivado a
> propósito: en iconos de línea es justo lo que produce esos bordes mordidos.
> Actívalo solo para fotos. Para dibujo vectorial, el par *suavizado + umbral*
> da mucho mejor resultado; sube el suavizado a 1–2 y mueve el umbral hasta que
> el trazo tenga el grosor que quieres.

Las líneas violetas del lienzo marcan las páginas de 8 px en las que el SSD1306
direcciona su memoria.

> Pantallas 1–9 → teclas 1–9. Pantalla 10 → tecla 11.

### Cambio automático
El teclado cambia de perfil solo según la aplicación que tengas delante. Cada
aplicación es una **tarjeta**: programa y perfil, nada más.

1. Activa el interruptor de arriba a la derecha.
2. Abre el programa que quieras (Photoshop, Altium…) y vuelve a OrbyGUI.
   El panel *Estado* muestra la ventana que se detectó.
3. Pulsa **Añadir la app actual**: crea una tarjeta con ese ejecutable. También
   hay un carrusel de sugerencias rápidas y tarjetas en blanco.
4. Elige en la tarjeta qué perfil se usa. Listo.

En el pie de cada tarjeta puedes afinar contra qué se compara el texto (sin
distinguir mayúsculas): el nombre del ejecutable, el título de la ventana o
ambos. Si dos tarjetas encajan a la vez gana la de más arriba, y las flechas
↑ ↓ cambian esa prioridad.

La tarjeta **Perfil por defecto** decide a qué perfil se vuelve cuando no se
detecta ninguna de las apps añadidas; también puede dejarse en «no cambiar».

> **Dos cosas distintas.** Aquí eliges *qué perfil* se activa con cada app. Si lo
> que quieres es que un perfil concreto tenga un par de teclas diferentes en una
> app, eso son las **variaciones** de *Perfiles y macros*: guardan solo las
> diferencias y evitan duplicar el perfil. El mismo detector alimenta las dos.

Detalles de implementación:

- La ventana activa se consulta cada 400 ms llamando a `GetForegroundWindow` de
  Win32 directamente desde Rust (`src-tauri/src/foreground.rs`). **No hace falta
  ningún módulo nativo de Node** —lo que arrastran paquetes como `active-win`— ni
  el proceso de PowerShell permanente que hacía esto antes.
- Solo se manda `SET_PROFILE` cuando el perfil realmente cambia.
- Las reglas se guardan en tu PC (`%APPDATA%\OrbyGUI\orby-config.json`), no en el
  teclado: **no** necesitan «Guardar en Flash».
- Solo funciona en Windows; en otros sistemas la vista lo indica y no hace nada.

### Ajustes
Reposo automático, información del dispositivo, reconexión forzada, relectura de
la configuración y restauración de valores de fábrica.

**Complementos.** OrbyGUI sabe manejar el teclado; lo que hay al otro lado del
PC lo ponen los complementos. Un complemento es un programa que traduce «esta
tecla» a «esto de fuera»: una lámpara de la red, un servicio con API, lo que sea
que el ordenador pueda alcanzar. Sus acciones aparecen luego en la pestaña
*Multimedia* de una tecla y como un tipo más en un mando.

La instalación por defecto **no trae ninguno**. Se instalan desde esta tarjeta
(desde un `.zip` o, si lo estás desarrollando, desde su carpeta) y viven en
`%APPDATA%\OrbyGUI\plugins\<id>\`, así que actualizar la app no se los lleva.
Cada uno que pida ajustes propios añade su tarjeta debajo, con su botón de
comprobación.

> Un complemento se ejecuta dentro de OrbyGUI con tus mismos permisos: puede
> leer tus archivos, salir a internet y abrir programas. Instala solo los que
> vengan de un origen que conozcas; la app pide confirmación antes de copiarlo.

Sus acciones las ejecuta el PC, así que necesitan OrbyGUI abierto —basta con el
icono de la bandeja— y **no** necesitan «Guardar en Flash»: lo que guarda el
teclado es solo el identificador de la macro.

Para escribir uno, ver [`docs/PLUGINS.md`](docs/PLUGINS.md). El de la lámpara
del escritorio está en [`plugins/lampdesk/`](plugins/lampdesk/) y se empaqueta
con `npm run pack:plugins`.

**Copia de seguridad.** Guarda en un archivo JSON del PC todos los perfiles
completos: nombres, etiquetas, atajos, mandos giratorios de las dos capas, la
rueda de cada capa y los iconos OLED en hexadecimal. Restaurar los vuelca de
vuelta al teclado, creando los perfiles que hagan falta.

> Hazte una copia **antes de flashear**. El firmware migra la configuración
> entre versiones, pero un archivo en el PC es el único respaldo que sobrevive
> a cualquier cosa.

### Consola serie
Terminal bidireccional con filtro, pausa e historial (flecha arriba). Búfer
circular de 500 líneas: la versión anterior reconstruía el DOM en cada línea y
acababa congelándose con la telemetría de la rueda.

---

## 3. Flujo de trabajo importante

Todos los cambios se aplican **en la RAM del microcontrolador** y se pierden al
desenchufar. Para que sobrevivan hay que pulsar **Guardar en Flash** (botón de la
barra de título, que se pone ámbar cuando hay cambios pendientes).

Es deliberado: escribir en Flash desgasta la memoria, así que solo se hace
cuando tú lo pides.

---

## 4. Protocolo serie (firmware 3.0)

USB CDC, 115200 baudios, líneas terminadas en `\n`.

### PC → teclado

| Comando | Efecto |
|---|---|
| `ACK` | Handshake. Responde `ORBY_V4:FW=4.1:KEYS=12:OLEDS=10:ENCODERS=2:PROFILES=<n>:MAXPROFILES=16:MAXPAGES=4:MACROS=1:HASH=1:MODE=<modo>` |
| `GET_STATE` | Vuelca perfil, nº de perfiles, brillo, timeout, modo, SUPER y scroll. Termina en `STATE:END` |
| `GET_HASH` | Huella CRC32 de la configuración: `HASH:PROFILES:<n>`, un `HASH:P:<idx>:<hex>` por perfil y `HASH:ALL:<hex>`. Termina en `HASH:END` |
| `SET_PROFILE:<idx>` | Cambia el perfil activo |
| `ADD_PROFILE` | Crea un perfil vacío. Responde `PROFILE:ADDED:<idx>` y `PROFILES:OK:<n>:<activo>` |
| `DUP_PROFILE:<idx>` | Duplica un perfil con sus iconos |
| `DEL_PROFILE:<idx>` | Borra un perfil y desplaza los siguientes (nunca el último que quede) |
| `SET_BRIGHTNESS:<0-255>` | Contraste de las OLED |
| `SET_TIMEOUT:<0\|1\|5\|10>` | Minutos hasta el reposo (0 = desactivado) |
| `SET_PSCROLL:<perfil>:<capa 0-1>:<6-240>:<inv 0\|1>` | Rueda de un perfil y una capa |
| `SET_SCROLL:<6-240>` | Igual, pero sobre el perfil y la capa activos |
| `SET_SCROLL_INV:<0\|1>` | Invierte la dirección en el perfil y la capa activos |
| `GET_SCROLL` | Devuelve `SCROLL:OK:<clics>:<invertido>:<altares>` de lo que se aplica ahora |
| `GET_PROFILE:<idx>` | Vuelca un perfil completo. Termina en `PROF:<n>:END` |
| `GET_PROFILES` | Vuelca todos, precedidos del recuento |
| `GET_PROFILE_COUNT` | Devuelve `PROFILES:OK:<n>:<activo>` |
| `SET_NAME:<perfil>:<texto>` | Renombra el perfil (7 caracteres) |
| `SET_LABEL:<perfil>:<0-19>:<texto>` | Etiqueta OLED. Huecos 0-9 normal, 10-19 SUPER |
| `SET_KEYMAP:<perfil>:<0-23>:<mod>:<key>` | Acción. Huecos 0-11 normal, 12-23 SUPER |
| `SET_ROTARY:<perfil>:<0-15>:<tipo>:<mod>:<key>` | Acción de encoder o rueda (ver abajo) |
| `OLED_CHUNK:<perfil>:<0-19>:<offset>:<hex>` | Trozo del bitmap de 360 bytes |
| `GET_OLED:<perfil>:<0-19>` | Vuelca el bitmap guardado en líneas `OLEDDATA:…`, o `NONE` si el hueco usa etiqueta de texto. Lee la página que el teclado tenga puesta |
| `GET_OLED_PG:<perfil>:<página>:<0-19>` | Igual, pero de una página concreta y sin cambiar la puesta. Responde `OLEDDATA:<perfil>:P<página>:<hueco>:…` |
| `OLED_CLEAR:<perfil>:<hueco\|255>` | Borra un icono (255 = todos los del perfil) |
| `SAVE_STATE` | Escribe todo en Flash |
| `RESET_DEFAULTS` | Restaura fábrica en RAM (requiere `SAVE_STATE` para fijarlo) |

En `SET_KEYMAP`, `<mod>` es la máscara de modificadores HID
(1=Ctrl, 2=Shift, 4=Alt, 8=Win, y sus variantes derechas en 16/32/64/128).
El valor especial **254** marca una acción multimedia, y entonces `<key>` es un
índice de la tabla de consumo.

**Tabla de consumo** (`<key>` cuando la acción es multimedia): 1=explorador,
2=calculadora, 3=play/pausa, 4=stop, 5=siguiente, 6=anterior, 7=volumen +,
8=volumen −, 9=silencio, 10=brillo +, 11=brillo −, 12=navegador, 13=correo,
14=buscar, 15=zoom +, 16=zoom −.

**Huecos de `SET_ROTARY`**: 0-2 encoder izquierdo (horario, antihorario, clic),
3-5 encoder derecho, 6-7 rueda de scroll (abajo, arriba). Los huecos **8-15
repiten la misma lista para la capa SUPER**, igual que las teclas.

**Tipos de `SET_ROTARY`**: 0=nada, 1=multimedia (`<key>` = índice de la tabla de
consumo), 2=atajo de teclado, 3=desplazar vertical, 4=desplazar horizontal,
5=zoom. Los tipos 3-5 son bidireccionales: basta configurarlos en un sentido.

### Teclado → PC

| Línea | Significado |
|---|---|
| `KEY_EV:<1-12>:<0\|1>` | Tecla soltada / pulsada |
| `ENC:<1\|2>:<delta>` | Giro de encoder |
| `ENC_SW:<1\|2>:<0\|1>` | Pulsador del encoder |
| `WHEEL:<cuentas>:<ángulo>` | Rueda magnética, agregado a 20 Hz. 4096 cuentas = una vuelta. El segundo campo es el ángulo absoluto (0-4095) con el que la app dibuja la rueda en su posición real |
| `MODE:<NORMAL\|MENU>` | Cambio de modo |
| `PROFILES:OK:<n>:<activo>` | Recuento de perfiles tras crear, duplicar o borrar |
| `ERR:<causa>` | Comando rechazado (`PROFILE_FULL`, `PROFILE_LAST`, `PROFILE_RANGE`…) |

---

## 5. Problemas frecuentes

**El scroll sigue saltando de tres en tres líneas.**
Mira el indicador de alta resolución en la tarjeta *Rueda de scroll*, dentro de
*Perfiles y macros*. Si está en «No negociada», desconecta y reconecta el
teclado. Si sigue igual, comprueba que el firmware flasheado es el 3.0 (`ACK`
debe responder `FW=3.0`). Algunas
aplicaciones Win32 antiguas ignoran el desplazamiento fraccionario y seguirán
saltando: es una limitación del sistema, no del teclado.

**No encuentra el dispositivo.**
Ciérralo todo lo que pueda tener el puerto COM abierto (Arduino IDE, PuTTY, otra
instancia de OrbyGUI) y pulsa *Reconectar* en Ajustes.

**`npm install` falla al compilar serialport.**
Necesitas las herramientas de compilación nativas de Windows. Instala
[Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/) con la
carga de trabajo de escritorio C++.

**Los cambios desaparecen al desenchufar.**
Falta pulsar *Guardar en Flash*.

**La app avisa de «Firmware antiguo detectado».**
Tienes flasheado el 1.0, que no conoce `GET_STATE` ni `GET_PROFILE`. El
dashboard y la consola funcionan, pero el editor de perfiles, la calibración de
la rueda y los iconos OLED necesitan el 2.0 como mínimo. Flashea
`build/ORBY_V4.uf2`.

**La app avisa de «Firmware 2.x».**
Ese firmware tiene cuatro perfiles fijos, los mandos en una sola capa y una
rueda global. Todo lo demás funciona, pero crear o borrar perfiles, configurar
los encoders en la capa SUPER y calibrar la rueda por perfil necesitan el 3.0.
Flashea `build/ORBY_V4.uf2`; la configuración que tengas guardada se migra sola
(la capa SUPER de los mandos arranca copiando la normal).

**El cambio automático no detecta nada.**
Comprueba que el interruptor está activado y mira el panel *Estado*. Si sale un
error de PowerShell, prueba a ejecutar
`powershell -NoProfile -Command "Get-Process -Id $PID"` en una terminal: si falla,
tu política de ejecución o un antivirus está bloqueando PowerShell.
