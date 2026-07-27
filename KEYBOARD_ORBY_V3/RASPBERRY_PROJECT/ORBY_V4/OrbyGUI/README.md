# OrbyGUI — panel de control del Orby V4

Aplicación de escritorio (Electron + Vite) para configurar el teclado Orby V4 en
caliente por USB CDC.

---

## 1. Cómo abrir la app

**La forma fácil: doble clic en `OrbyGUI.bat`.**

Ese script instala las dependencias si faltan, compila la interfaz y abre la
app. Si quieres un acceso directo en el escritorio, clic derecho sobre el `.bat`
→ *Enviar a* → *Escritorio (crear acceso directo)*.

Desde la terminal:

| Comando         | Qué hace                                                          |
|-----------------|-------------------------------------------------------------------|
| `npm start`     | Compila y abre la app (equivalente al `.bat`)                      |
| `npm run dev`   | Vite + Electron con recarga en caliente, para tocar el código      |
| `npm run build` | Solo genera `dist/`                                                |
| `npm run dist`  | Instalador NSIS para Windows en `release/`                         |

Requiere **Node.js 20.19 o superior** (Vite 7 no arranca por debajo). La primera
vez, `npm install` también recompila `serialport` para la versión de Electron.

> `npm run dev` pasa `--dev` a Electron para que cargue desde el servidor de
> Vite. Sin ese flag siempre se sirve `dist/`.

La app no descarga nada de internet: tipografía del sistema e iconos SVG
incrustados. Funciona sin conexión.

### Conexión

Al arrancar busca cada 3 s un puerto serie cuyo `vendorId` sea `cafe` o cuyo
`productId` sea `4005` / `4006`, y le manda `ACK`. Si el dispositivo contesta
`ORBY_V4:...`, se conecta y descarga los 4 perfiles completos con `GET_PROFILE`.

> El PID del firmware 2.0 es **0x4006**. Se cambió a propósito: Windows cachea el
> descriptor de informe HID por VID/PID, y sin cambiarlo seguiría usando el
> descriptor viejo, sin scroll de alta resolución.

---

## 2. Las vistas

### Dashboard
Réplica del hardware. Las teclas se iluminan al pulsarlas, los encoders giran,
y la rueda muestra una barra de intensidad más el desplazamiento acumulado en
clics. El panel de estado indica si Windows ha activado el **scroll de alta
resolución**.

### Perfiles y macros
El editor de verdad. Ya no hay datos duplicados en la app: todo se lee del
firmware.

1. Elige el perfil arriba (el marcado *EN USO* es el activo en el teclado).
2. Cambia entre capa **NORMAL** y **SUPER** (la capa SUPER se activa con la tecla 10).
3. Pulsa cualquier tecla del panel para abrir el inspector.
4. En el inspector puedes:
   - Editar la **etiqueta OLED** (máx. 7 caracteres).
   - Marcar **modificadores** (Ctrl, Shift, Alt, Win…).
   - Elegir la **tecla** de la lista.
   - O pulsar **Capturar atajo** y teclear la combinación directamente en el PC.
   - O asignar una **acción multimedia** (play/pausa, siguiente, calculadora…).

Cada cambio viaja al teclado al instante y se ve en las pantallas OLED.

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

Los tres últimos llevan el sentido en el signo del giro, así que una sola acción
cubre las dos direcciones y la app espeja el par automáticamente. Las
pulsaciones no los ofrecen: un clic no tiene dirección.

> Solo **Desplazar vertical** aprovecha la alta resolución de la rueda
> magnética. El resto de acciones son discretas y trabajan por clics completos.

> Los encoders ejecutan estas acciones únicamente en modo normal. Dentro del
> menú del teclado siguen sirviendo para navegar.

### Rueda de scroll
Ajusta cuántos clics equivale una vuelta completa. Preajustes: Preciso (12),
Suave (30), Estándar (60), Rápido (120). También permite invertir la dirección.

El panel derecho muestra si Windows negoció la alta resolución, un dial con el
giro en vivo y una caja de prueba para notar la diferencia sin salir de la app.

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
El teclado cambia de perfil solo según la aplicación que tengas delante.

1. Activa el interruptor de arriba a la derecha.
2. Abre el programa que quieras (Photoshop, Altium…) y vuelve a OrbyGUI.
   El panel *Estado* muestra la ventana que se detectó.
3. Pulsa **Añadir la app actual**: crea una regla con ese ejecutable apuntando
   al perfil activo. También hay sugerencias rápidas y reglas en blanco.
4. Ajusta a qué perfil apunta cada regla.

Cada regla compara un texto (sin distinguir mayúsculas) contra el nombre del
ejecutable, el título de la ventana o ambos. Se evalúan **de arriba abajo y gana
la primera que encaja**, así que pon las más específicas primero. Los botones
↑ ↓ reordenan.

El desplegable *Cuando no encaje ninguna regla* permite volver a un perfil por
defecto, o dejarlo en «no cambiar».

Detalles de implementación:

- La ventana activa se consulta cada 400 ms desde un proceso de PowerShell que
  llama a `GetForegroundWindow` por P/Invoke. **No hace falta ningún módulo
  nativo**, que es lo que arrastran paquetes como `active-win` y hay que
  recompilar por cada versión de Electron.
- Solo se manda `SET_PROFILE` cuando el perfil realmente cambia.
- Las reglas se guardan en tu PC (`%APPDATA%\OrbyGUI\orby-config.json`), no en el
  teclado: **no** necesitan «Guardar en Flash».
- Solo funciona en Windows; en otros sistemas la vista lo indica y no hace nada.

### Ajustes
Brillo, reposo automático, información del dispositivo, reconexión forzada,
relectura de la configuración y restauración de valores de fábrica.

**Copia de seguridad.** Guarda en un archivo JSON del PC los cuatro perfiles
completos: nombres, etiquetas, atajos, mandos giratorios y los iconos OLED en
hexadecimal. Restaurar los vuelca de vuelta al teclado.

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

## 4. Protocolo serie (firmware 2.0)

USB CDC, 115200 baudios, líneas terminadas en `\n`.

### PC → teclado

| Comando | Efecto |
|---|---|
| `ACK` | Handshake. Responde `ORBY_V4:FW=2.0:KEYS=12:OLEDS=10:ENCODERS=2:MODE=<modo>` |
| `GET_STATE` | Vuelca perfil, brillo, timeout, modo y scroll. Termina en `STATE:END` |
| `SET_PROFILE:<0-3>` | Cambia el perfil activo |
| `SET_BRIGHTNESS:<0-255>` | Contraste de las OLED |
| `SET_TIMEOUT:<0\|1\|5\|10>` | Minutos hasta el reposo (0 = desactivado) |
| `SET_SCROLL:<6-240>` | Clics de rueda por vuelta completa |
| `SET_SCROLL_INV:<0\|1>` | Invierte la dirección del scroll |
| `GET_SCROLL` | Devuelve `SCROLL:OK:<clics>:<invertido>:<altares>` |
| `GET_PROFILE:<0-3>` | Vuelca un perfil completo. Termina en `PROF:<n>:END` |
| `GET_PROFILES` | Vuelca los cuatro |
| `SET_NAME:<perfil>:<texto>` | Renombra el perfil (7 caracteres) |
| `SET_LABEL:<perfil>:<0-19>:<texto>` | Etiqueta OLED. Huecos 0-9 normal, 10-19 SUPER |
| `SET_KEYMAP:<perfil>:<0-23>:<mod>:<key>` | Acción. Huecos 0-11 normal, 12-23 SUPER |
| `SET_ROTARY:<perfil>:<0-7>:<tipo>:<mod>:<key>` | Acción de encoder o rueda (ver abajo) |
| `OLED_CHUNK:<perfil>:<0-19>:<offset>:<hex>` | Trozo del bitmap de 360 bytes |
| `GET_OLED:<perfil>:<0-19>` | Vuelca el bitmap guardado en líneas `OLEDDATA:…`, o `NONE` si el hueco usa etiqueta de texto |
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
3-5 encoder derecho, 6-7 rueda de scroll (abajo, arriba).

**Tipos de `SET_ROTARY`**: 0=nada, 1=multimedia (`<key>` = índice de la tabla de
consumo), 2=atajo de teclado, 3=desplazar vertical, 4=desplazar horizontal,
5=zoom. Los tipos 3-5 son bidireccionales: basta configurarlos en un sentido.

### Teclado → PC

| Línea | Significado |
|---|---|
| `KEY_EV:<1-12>:<0\|1>` | Tecla soltada / pulsada |
| `ENC:<1\|2>:<delta>` | Giro de encoder |
| `ENC_SW:<1\|2>:<0\|1>` | Pulsador del encoder |
| `WHEEL:<cuentas>` | Rueda magnética, agregado a 20 Hz. 4096 cuentas = una vuelta |
| `MODE:<NORMAL\|MENU>` | Cambio de modo |
| `ERR:<causa>` | Comando rechazado |

---

## 5. Problemas frecuentes

**El scroll sigue saltando de tres en tres líneas.**
Mira el indicador de alta resolución en la vista *Rueda*. Si está en «No
negociada», desconecta y reconecta el teclado. Si sigue igual, comprueba que el
firmware flasheado es el 2.0 (`ACK` debe responder `FW=2.0`). Algunas
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
la rueda y los iconos OLED necesitan el 2.0. Flashea `build/ORBY_V4.uf2`.

**El cambio automático no detecta nada.**
Comprueba que el interruptor está activado y mira el panel *Estado*. Si sale un
error de PowerShell, prueba a ejecutar
`powershell -NoProfile -Command "Get-Process -Id $PID"` en una terminal: si falla,
tu política de ejecución o un antivirus está bloqueando PowerShell.
