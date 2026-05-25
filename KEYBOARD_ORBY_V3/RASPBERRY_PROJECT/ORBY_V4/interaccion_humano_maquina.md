# INTERACCIÓN HUMANO MAQUINA:

# TECLADO MACRO OLED

### 1. INTRODUCCIÓN Y DESCRIPCIÓN:

Para este proyecto vamos a usar un proyecto personal que teníamos en curso.
El primer diseño consistía en un teclado MACRO configurable con 12 teclas
dispuestas en una matriz de 4x3, dos encoders rotativos y una rueda de scroll.
Esta rueda actúa como si fuera la rueda de un ratón, pero trabaja con precisión
de píxeles, y lo más llamativo es que incluye pequeñas pantallas OLED de 0,
pulgadas en cada una de las teclas. Este prototipo físico se llegó a fabricar por
completo, pero todavía falta por programar el firmware.

Después de considerar las diferentes alternativas de hardware, hemos decidido basar todo el desarrollo en el **primer prototipo físico (ORBY_V4)** que fue fabricado por completo. Este consta de 12 teclas configurables (donde 10 de ellas integran pantallas OLED individuales de 72x40 píxeles), dos encoders rotativos y una rueda de scroll física de precisión basada en el sensor magnético AS5600.

A nivel de software y firmware, diseñaremos desde el inicio la lógica de interacción que nos permita resolver la interacción local y la configuración integrada sin necesidad de una pantalla central física, aprovechando la versatilidad de las pantallas individuales de las teclas para renderizar dinámicamente menús de configuración local (Menú Dinámico Distribuido). Este enfoque garantiza un realismo físico absoluto y una alta fidelidad en la interacción, permitiendo que el firmware sea cargado y probado de forma directa sobre el dispositivo real.



### 2. FUNCIONES SEGÚN USUARIO Y ESCENARIOS

Siguiendo la metodología de Diseño Centrado en el Usuario (UCD), a
continuación, se detalla la estructura de los tres niveles de usuario para el
dispositivo, incorporando los perfiles preconfigurados de serie para el nivel
intermedio.

**2.1. Usuario Básico**

- **Perfil del Usuario y Necesidades:** Carmen, 61 años. Trabaja como
    administrativa en una oficina gestionando documentación, leyendo
    informes y editando archivos de texto de forma intensiva. Carmen
    presenta fatiga visual crónica por el uso continuado de pantallas y rigidez
    articular incipiente en las manos. Su enfoque tecnológico es puramente
    utilitario: no tiene interés en aprender a usar programas de configuración
    ni menús complejos. Demanda un dispositivo directo y listo para usar
    ( _Plug & Play_ ) que funcione de inmediato al conectarlo para mejorar su
    ergonomía diaria.
- **Funcionalidades que solventan sus necesidades:**
    o **Perfil Predeterminado de Fábrica:** El sistema activa
       automáticamente un mapa de comandos estándar guardado en la
       memoria no volátil. Este perfil asigna funciones directas a las teclas
       físicas para los comandos más repetidos en ofimática: copiar,
       pegar, cortar y captura de pantalla.
    o **Controles de Sistema Directos:** Asignación inmediata de los
       encoders rotativos y de la rueda de scroll para actuar sobre las
       variables globales del ordenador: control de volumen de audio,
       ajuste de brillo de la pantalla del PC y desplazamiento vertical
       (scrollear). Las funciones de serie son suficientes para su flujo de
       trabajo y no requiere realizar ninguna acción de configuración.

**Escenario típico:** Carmen utiliza este dispositivo en su puesto de trabajo para
leer y editar documentos. Conecta el teclado y utiliza el perfil predeterminado
porque le agiliza el proceso de copiado y edición, cubriendo todas sus
necesidades para los programas de ofimática que maneja. No le preocupa la
funcionalidad personalizable del teclado, solo busca aumentar su productividad
diaria tal y como viene el dispositivo de serie.


**2.2. Usuario Intermedio**

- **Perfil del Usuario y Necesidades:** Álex, 28 años. Trabaja como
    diseñador técnico y editor de contenido multimedia, utilizando de forma
    alterna programas profesionales como Photoshop, Illustrator, Premiere o
    herramientas CAD como _Altium Designer_. Álex necesita adaptar el teclado
    macro a las herramientas de software que utiliza en su jornada laboral,
    pero busca hacerlo de la manera más directa, sencilla y rápida posible
    para ir "a tiro hecho". No quiere distraerse con opciones de
    personalización estéticas profundas; prefiere realizar modificaciones
    funcionales en caliente y disponer de entornos ya preparados para sus
    herramientas habituales.
- **Funcionalidades que solventan sus necesidades:**
    o **Perfiles Profesionales Predeterminados:** Inclusión en la
       memoria interna del hardware de perfiles preconfigurados de
       fábrica para los programas profesionales más populares
       (Photoshop, Premiere, Altium Designer, etc.), permitiendo al
       usuario disponer de mapas de comandos optimizados de serie sin
       tener que mapearlos desde cero.
    o **Acceso Físico al Menú Local:** Comando de seguridad en el
       firmware que despliega el menú interno al pulsar de forma
       simultánea los botones de los dos encoders rotativos durante unos
       segundos.
    o **Navegación Local Inteligente:** Interfaz gráfica distribuida sobre las pantallas OLED individuales de las teclas para desplazarse de forma autónoma entre los perfiles predeterminados y las opciones de personalización del hardware.
    o **Modificación de Teclas en Caliente:** Permite modificar o
       reasignar a cualquier tecla física una combinación de comandos
       propia, un atajo de una lista predefinida (ej: _Ctrl+C, Ctrl+V, Ctrl+Z_ ),
       una función del sistema (volumen, brillo) o la apertura de una
       aplicación profesional.
    o **Personalización Dinámica de Controles:** Opción para cambiar
       las funciones de los encoders o de la rueda de scroll, permitiendo
       al usuario alternar entre variar el volumen, el brillo, hacer zoom o
       realizar scroll tanto horizontal como verticalmente según el
       programa de diseño abierto.
    o **Biblioteca de Iconos Integrada:** Menú local para cambiar el icono
       de las pantallas OLED secundarias de las teclas
       seleccionando entre diferentes diseños gráficos predefinidos en el
       hardware para identificar los comandos técnicos.
    o **Ajustes de Hardware Locales:** Parámetros editables en las pantallas OLED individuales de las teclas (usando el menú dinámico distribuido) para cambiar el brillo general de los displays o modificar el tiempo de inactividad para entrar en modo de bajo consumo.
**Escenario típico:** Alex acaba de comprar el teclado y quiere utilizarlo para
mejorar su productividad diseñando una placa de circuito impreso en Altium
Designer. Para ello, accede al menú de configuración del teclado, navega hasta
la opción de perfiles predeterminados, selecciona el perfil de Altium y lo
selecciona para su uso. Ahora tiene los atajos mas comunes del programa
programados en las teclas del teclado, y todas las teclas muestran un icono de
la acción que realizan. Ya puede ponerse a trabajar y ahorrar tiempo presionando
el teclado Macro para atajar tiempo y no tener que buscar como era cierto atajo
concreto en caso de olvidarlo.

**2.3. Usuario Avanzado**

- **Perfil del Usuario y Necesidades:** Marcos, 23 años. Es un apasionado
    de los ordenadores, la tecnología y los videojuegos, y dispone de bastante
    tiempo libre para trastear con sus dispositivos. Marcos quiere exprimir al
    máximo el potencial de personalización del teclado macro y adaptarlo
    minuciosamente a sus sesiones de juego. Le encanta dedicar horas a
    merodear por internet en busca de trucos, investigar entre librerías online,
    y es fan de la personalización extrema para cada uno de sus videojuegos
    favoritos. Además, busca optimizar su rendimiento al jugar.
- **Funcionalidades que solventan sus necesidades:**
    o **Aplicación de Escritorio Dedicada:** Software específico para el
       ordenador que permite la creación, gestión y almacenamiento
       centralizado de múltiples perfiles independientes optimizados para
       diferentes videojuegos y aplicaciones.
    o **Acceso a Librerías Online y Comunidad:** Repositorio integrado
       en la aplicación de PC que permite a Marcos pasar tiempo
       investigando, descargando e intercambiando perfiles complejos
       creados por otros entusiastas.
    o **Editor de Macros con Temporización Configurable:**
       Herramienta avanzada para programar cadenas de comandos
       encadenados que admiten la inserción de retardos y tiempos
       específicos en milisegundos entre pulsaciones, ideal para ejecutar
       combos o acciones precisas en los juegos.


```
o Diseño Visual a Medida (Personalización OLED): Interfaz en la
aplicación de escritorio para dibujar o cargar imágenes
personalizadas y mapas de bits, adaptándolos a la resolución de
las pantallas OLED de 0,42 pulgadas de cada tecla para que
muestren logos o símbolos idénticos a las funciones del juego.
o Conmutación Automática por Contexto: La aplicación de
escritorio cambia el perfil seleccionado en el teclado en función de
la aplicación o servicio que se está utilizando en cada momento.
```
**Escenario típico:** Marcos pasa toda la tarde configurando su teclado macro para
su videojuego de estrategia favorito. Navega en la aplicación de escritorio en su
ordenador en busca de un perfil para su juego, y tras encontrar uno ya hecho por
otro jugador, decide modificarlo a su gusto, añadiendo imágenes hechas por él
a cada macro iguales a los botones del juego y modificando el timing al
milisegundo de cada macro para que sus estrategias se ejecuten a la perfección.
Al abrir el juego, la aplicación de escritorio lo detecta automáticamente y
establece el perfil de ese juego en el teclado. Más tarde Marcos cambia de juego,
y la aplicación vuelve a detectar el juego nuevo y selecciona un perfil que Marcos
ya había descargado con anterioridad para el juego nuevo.


## 3. ARQUITECTURA DEL FIRMWARE Y HARDWARE REAL (ORBY_V4)

Para garantizar la viabilidad y fidelidad física del proyecto con el prototipo realmente fabricado, se ha consolidado la **Opción A (Realismo Físico)**. El diseño se adapta con máxima precisión a los componentes y pines reales del circuito impreso actual:

*   **12 Teclas Físicas Directas:** Conectadas a los pines GP1 a GP12 de la Raspberry Pi Pico.
*   **10 Pantallas OLED SSD1306 (72x40 píxeles):** Multiplexadas a través de una línea SPI compartida. La selección de chip (CS) de cada pantalla se maneja mediante dos registros de desplazamiento `74HC595` conectados en cascada, optimizando los pines de la Pico.
    *   *Mapeo de pantallas:* Las teclas 1 a 9 tienen asociadas las pantallas OLED 1 a 9. La tecla 11 está asociada a la pantalla OLED 10. Las teclas 10 y 12 no tienen pantallas (se reservan para funciones especiales o modificadores globales).
*   **2 Encoders Rotativos Mecánicos (EC11):** Dotados de pulsador integrado (`ENC1_SW` en GP25 y `ENC2_SW` en GP28).
*   **1 Rueda de Scroll de Precisión:** Basada en el sensor magnético rotativo **AS5600** conectado por bus I2C (`I2C_SDA` en GP14 y `I2C_SCL` en GP15).
*   **Conexión USB CDC:** Puerto serie virtual emulado a través de la librería TinyUSB.

### 3.2. Esquema de Comunicación Dual (HID Nativo + Canal Serie CDC)

Para maximizar la versatilidad y garantizar un funcionamiento sin fricciones, el firmware implementa un canal de comunicación híbrido y bidireccional. Al conectarse al ordenador, la Raspberry Pi Pico se enumera como un **dispositivo USB compuesto (Composite Device)**, exponiendo simultáneamente:
1.  **Dispositivos USB HID Nativo:** Teclado estándar (Keyboard), Control del Consumo (Consumer Control) y Ratón (Mouse).
2.  **Puerto Serie Virtual (USB CDC):** Canal de comunicación bidireccional serie para la sincronización con la aplicación de PC (`OrbyGUI`).

#### A. Comunicación Dispositivo -> PC (Emisión de Acciones)
Cuando el usuario pulsa una tecla, gira o pulsa un encoder, o desliza la rueda magnética AS5600, el firmware determina cómo enviar la acción basándose en el mapeo configurado en el perfil activo:

1.  **Acciones Nativas (Modo Offline / Plug & Play):**
    *   Si la acción asignada se corresponde con comandos estándar del sistema operativo (por ejemplo: atajos de teclado comunes como copiar (`Ctrl+C`), pegar (`Ctrl+V`), rehacer (`Ctrl+Y`), pulsar teclas multimedia como volumen arriba/abajo, pausar música o scroll vertical de la rueda), el firmware las envía directamente a través del protocolo **USB HID Nativo**.
    *   La rueda magnética AS5600 se traduce directamente en comandos nativos de la rueda del ratón (`HID Mouse Scroll`).
    *   **Ventaja:** El teclado funciona de inmediato como un dispositivo de entrada estándar al conectarlo (Plug & Play) en cualquier ordenador o sistema operativo, incluso sin tener la aplicación `OrbyGUI` instalada o ejecutándose.
2.  **Acciones No Nativas (Modo Enlazado con App de PC):**
    *   Si la acción asignada en el perfil requiere interactuar con el sistema operativo host de una manera que no se puede representar con atajos de teclado estándar (por ejemplo: abrir una aplicación específica como Excel, ejecutar scripts de automatización, activar macros complejas en NodeJS, o interactuar con APIs externas), el teclado transmite el evento por el puerto virtual **USB CDC Serial** (ej: `KEY_EV:3:1` o `ENC:2:1`).
    *   La aplicación de escritorio `OrbyGUI` escucha en segundo plano el puerto serie, recibe la señal, busca la acción asociada a esa tecla en su base de datos de configuración y la ejecuta en el ordenador host.

#### B. Comunicación PC -> Dispositivo (Gestión y Configuración)
La aplicación `OrbyGUI` se comunica bidireccionalmente con el teclado por el canal serie USB CDC para realizar las siguientes tareas en tiempo real:
*   **Conmutación Automática por Contexto:** La app detecta qué programa profesional (Photoshop, Altium, Premiere, etc.) está en primer plano en el ordenador y envía una orden al teclado para cambiar automáticamente al perfil mapeado, refrescando los iconos OLED de las teclas en menos de 100 ms.
*   **Gestión y Guardado Permanente:** Envía mapas de macros y combinaciones de teclas al dispositivo, el cual los almacena directamente en la memoria **Flash no volátil** de la Pico. Esto garantiza que las macros nativas sigan funcionando de forma autónoma (Modo Offline).
*   **Renderizado dinámico de Iconos:** Envía mapas de bits en formato hexadecimal (72x40 píxeles, monocromo) correspondientes a los iconos de las macros configuradas para que el Core 1 las dibuje en las 10 pantallas OLED de las teclas.

---

## 4. MENÚ LOCAL DISTRIBUIDO (INTERACCIÓN SIN PANTALLA CENTRAL)

Para resolver de manera elegante la ausencia de una pantalla central física en este diseño consolidado de 12 teclas, el firmware implementa la navegación del menú local directamente sobre las 10 pantallas OLED individuales de las teclas, eliminando componentes innecesarios.

### 4.1. Entrada al Menú de Configuración
*   Para ingresar al menú local, el usuario realiza una pulsación prolongada simultánea (de 2 segundos) en los botones integrados de ambos encoders (`ENC1_SW` y `ENC2_SW`).
*   Al entrar, el firmware pasa a **Modo Menú** y cambia temporalmente el contenido de las pantallas OLED de las teclas **1 a 5** por texto invertido con las opciones globales:
    *   **Tecla 1 (OLED 1):** `PERF` (Menú de perfiles de macros locales)
    *   **Tecla 2 (OLED 2):** `BRIL` (Ajuste del brillo de las pantallas)
    *   **Tecla 3 (OLED 3):** `REPO` (Ajuste del tiempo de reposo en bajo consumo)
    *   **Tecla 4 (OLED 4):** `INFO` (Información del firmware y telemetría)
    *   **Tecla 5 (OLED 5):** `EXIT` (Salir del menú y volver a macros estándar)

### 4.2. Flujo de Navegación y Ajuste de Parámetros
1.  **Selección de Opción Principal:** El usuario puede pulsar directamente la tecla física correspondiente (1 a 5) o girar el `Encoder 1` para desplazarse entre las opciones (la pantalla de la opción seleccionada parpadea) y presionar el botón del encoder (`ENC1_SW`) para confirmar la selección.
2.  **Ajuste de Perfiles Locales (`PERF`):**
    *   Las pantallas de las teclas 1 a 4 muestran los perfiles cargados (ej: `OFIM` en OLED 1, `PHTS` en OLED 2, `ALTM` en OLED 3, `PREM` en OLED 4). La tecla 5 muestra el texto `BACK` (Atrás).
    *   Al pulsar las teclas 1-4, el firmware carga el perfil elegido de la memoria Flash, actualiza las pantallas con sus iconos correspondientes y vuelve al Modo Normal.
3.  **Ajuste de Brillo (`BRIL`):**
    *   La pantalla de la tecla 1 muestra una barra de progreso gráfica interactiva (0% a 100%). La pantalla de la tecla 5 muestra `SAVE` (Guardar).
    *   El usuario gira cualquiera de los dos encoders para regular el brillo general. Al pulsar la tecla 5 (o hacer clic en el encoder), el nivel de brillo se escribe permanentemente en la Flash y se regresa al menú principal.
4.  **Ajuste de Reposo (`REPO`):**
    *   Las teclas 1 a 4 muestran tiempos de apagado por inactividad: `1 MIN` (OLED 1), `5 MIN` (OLED 2), `10 MIN` (OLED 3) y `OFF` (OLED 4, nunca apagar). La tecla 5 muestra `BACK` (Atrás).
    *   Al pulsar la tecla correspondiente, se reconfigura el temporizador de bajo consumo del microcontrolador y se vuelve al menú principal.
5.  **Información y Diagnóstico (`INFO`):**
    *   Las pantallas 1 a 3 muestran de manera estática los datos del dispositivo: `FW v1.0` en OLED 1, `CON: USB` en OLED 2 y `KEYS: 12` en OLED 3. La tecla 5 muestra `BACK`.

---

## 5. PROTOCOLO DE COMUNICACIÓN SERIE (PC <-> TECLADO)

Las comunicaciones sobre el puerto virtual serie USB CDC utilizan tramas ASCII limpias terminadas en carácter de nueva línea (`\n`).

### 5.1. Transmisión desde el Teclado hacia el PC (Telemetría de Eventos)

| Evento | Trama de Envío | Descripción |
| :--- | :--- | :--- |
| **Pulsación de Tecla** | `KEY_EV:<id_tecla>:<estado>\n` | Emitido únicamente si la tecla tiene asignada una macro no nativa. `<id_tecla>` es de 1 a 12. `<estado>` es 1 (pulsada) o 0 (soltada). |
| **Giro de Encoder** | `ENC:<id_encoder>:<delta>\n` | Emitido si el giro del encoder ejecuta una macro no nativa. `<id_encoder>` es 1 o 2. `<delta>` es un entero positivo o negativo. |
| **Pulsación de Encoder** | `ENC_SW:<id_encoder>:<estado>\n` | Emitido al presionar el pulsador de un encoder si tiene asignada una acción no nativa. |
| **Modo del Dispositivo** | `MODE:<tipo>\n` | Notifica a la GUI cuando el usuario cambia localmente de modo. `<tipo>` puede ser `NORMAL` o `MENU`. |

### 5.2. Transmisión desde el PC hacia el Teclado (Configuración y Renderizado)

| Comando | Trama de Envío | Acción del Firmware | Respuesta del Teclado |
| :--- | :--- | :--- | :--- |
| **Handshake / ACK** | `ACK\n` | Solicita información del dispositivo para la detección automática en la GUI. | `ORBY_V4:FW=1.0:KEYS=12:OLEDS=10:ENCODERS=2:MODE=NORMAL\n` |
| **Cambiar Perfil Activo** | `SET_PROFILE:<id_perfil>\n` | Carga inmediatamente en memoria el mapa de teclas y los iconos del perfil solicitado. `<id_perfil>` es de 1 a 10. | `PROFILE:OK:%d\n` |
| **Ajustar Brillo OLED** | `SET_BRIGHTNESS:<valor>\n` | Modifica el nivel de brillo general de las pantallas. `<valor>` es de 0 a 255. | `BRIGHTNESS:OK:%d\n` |
| **Ajustar Tiempo de Reposo** | `SET_TIMEOUT:<minutos>\n` | Modifica el temporizador para apagar las pantallas en inactividad. `0` deshabilita el modo reposo. | `TIMEOUT:OK:%d\n` |
| **Cargar Imagen OLED** | `WRITE_OLED:<id_pantalla>:<bitmap_hex>\n` | Carga en la pantalla especificada (`1` a `10`) un mapa de bits dinámico en hexadecimal (360 bytes de buffer decodificado). | `OLED:OK:%d\n` |
| **Definir Macro de Tecla** | `SET_MACRO:<id_tecla>:<tipo_macro>:<secuencia>\n` | Asigna una macro a la tecla. `<tipo_macro>` puede ser `HID` (ej: combinaciones nativas de teclas) o `APP` (ej: abrir app). Se almacena en la Flash para funcionar en modo offline u online. | `MACRO:OK:%d\n` |
