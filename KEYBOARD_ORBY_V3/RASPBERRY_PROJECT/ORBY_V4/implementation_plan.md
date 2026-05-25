# Plan de Implementación: Diseño del Firmware y Especificaciones de Interacción (Orby Keyboard)

Este documento detalla el plan para establecer de manera definitiva el diseño del firmware del teclado Orby, partiendo del documento existente `interaccion_humano_maquina.md` (IHM). Antes de escribir una sola línea de código, acordaremos los aspectos arquitectónicos y de interacción, refinando y completando los diagramas de interacción y el comportamiento de los perfiles.

---

## Estado Actual y Análisis Técnico

Hemos analizado minuciosamente el código y los pines definidos en el repositorio actual:
1. **Hardware Físico Actual (ORBY_V4):**
   - **Teclas:** 12 teclas directas a GPIOs (`KEY_1` a `KEY_12`).
   - **Pantallas OLED:** 10 pantallas SSD1306 (72x40 px) multiplexadas por SPI mediante dos registros de desplazamiento 74HC595. Las teclas 10 y 12 no tienen pantalla asociada; la tecla 11 se asocia a la pantalla 10.
   - **Encoders:** 2 encoders mecánicos EC11 con pulsador integrado (`ENC1_A/B/SW` y `ENC2_A/B/SW`).
   - **Rueda de Scroll:** Sensor magnético de alta precisión AS5600 conectado por I2C (`I2C_SDA=14`, `I2C_SCL=15`).
   - **Comunicación:** Enlace USB CDC (puerto serie virtual) implementado con TinyUSB (`tusb`).
2. **Propuesta del Documento IHM (Versión 2):**
   - **Teclas y OLEDs:** Matriz de 4x4 (16 teclas con 16 pantallas individuales).
   - **Pantalla Central:** Una pantalla OLED de mayor tamaño en el centro, dedicada exclusivamente a menús y configuración.
   - **Diferencia Clave:** El hardware físico actual es de 12 teclas sin pantalla central, mientras que el diseño conceptual de la V2 es de 16 teclas con pantalla central.

---

## Preguntas Abiertas para el Usuario (Revisión Requerida)

Para refinar el firmware y el documento `interaccion_humano_maquina.md` a nuestro gusto, debemos resolver las siguientes cuestiones de diseño:

> [!IMPORTANT]
> **Pregunta 1: ¿A qué versión de hardware adaptamos el diseño del firmware?**
> - **Opción A (Realismo Físico - Adaptado a 12 teclas / 10 OLEDs):** Adaptamos el firmware y el documento para que coincidan exactamente con la PCB física existente. Para el menú de configuración local (ya que no hay pantalla central física), entraremos en **Modo Menú** pulsando ambos encoders, y **las pantallas de las teclas se convertirán dinámicamente en los botones de navegación del menú** (por ejemplo, mostrando flechas de dirección, "OK", "ATRÁS", "BRILLO", "PERFIL" directamente bajo los dedos del usuario). Esto es sumamente interactivo y de altísima fidelidad.
> - **Opción B (Dual / Emulación V2 - 16 teclas y Pantalla Central):** Mantenemos la descripción teórica de la V2 en el documento IHM. En el firmware C++, diseñamos la lógica abstrayendo la pantalla central y las 16 teclas, permitiendo simular la interfaz a través de logs serie o una ventana virtual en la app de escritorio `OrbyGUI` para demostrar los menús multinivel sin necesidad de la pantalla física.
>
> **Pregunta 2: ¿Cómo estructuramos los Diagramas JSD y STN en el documento?**
> Queremos que el documento IHM sea excelente. Proponemos completarlo directamente en las secciones vacías con:
> - **Diagrama JSD (Jackson System Development) en texto estructurado o Mermaid:** Detallando el ciclo de vida del dispositivo y los procesos concurrentes en el microcontrolador de doble núcleo (Core 0 para entradas y USB, Core 1 para refresco SPI de pantallas).
> - **Diagrama STN (State Transition Network) en Mermaid:** Modelando todos los estados de la interfaz de usuario: Modo Normal, Menú de Configuración, Selección de Perfil, Ajuste de Brillo local, Ajuste de Tiempo de Espera, y Modo de Bajo Consumo (Sleep).
>
> **Pregunta 3: ¿Qué protocolo de comunicación serie implementaremos?**
> Diseñaremos un protocolo robusto y ligero basado en comandos ASCII sobre USB CDC. Por ejemplo:
> - **Dispositivo a PC:**
>   - `KEY_EV:<num_tecla>:<estado>` (0=soltado, 1=pulsado)
>   - `ENC:<num_encoder>:<delta_giro>` (ej: `ENC:1:-1` o `ENC:2:1`)
>   - `WHEEL:<grados_o_delta>` (desplazamiento de la rueda magnética)
> - **PC a Dispositivo (Configuración):**
>   - `SET_PROFILE:<id_perfil>` (cambio automático de perfil)
>   - `SET_BRIGHTNESS:<valor_0_255>`
>   - `WRITE_OLED:<num_pantalla>:<data_hex_o_rle>` (para cambiar los iconos en caliente)
>   - `SAVE_MACRO:<tecla>:<secuencia_de_teclas_con_delays>`

---

## Propuesta de Cambios y Plan de Acción

Proponemos realizar la preparación en dos fases rigurosas:

### Fase 1: Consolidación y Adaptación del Documento de Diseño (IHM)
1. **Discusión y Decisión de Arquitectura:** El usuario responde a las preguntas abiertas sobre la compatibilidad de hardware (12 teclas vs 16 teclas) y la interacción del menú local.
2. **Actualización de `interaccion_humano_maquina.md`:**
   - Redactar los detalles de funcionamiento según la opción elegida.
   - Insertar **Diagramas Mermaid** detallados para **JSD** y **STN**.
   - Definir formalmente la **tabla del protocolo de comunicación serie**.
   - Validar que el flujo de Carmen (Básico), Álex (Intermedio) y Marcos (Avanzado) encaja a la perfección con la arquitectura acordada.

### Fase 2: Planificación del Desarrollo del Firmware (C++)
Una vez aprobado el documento IHM, crearemos un plan técnico para implementar en C++:
- Modularización de controladores (`KeyboardMatrix`, `EncoderManager`, `AS5600Wheel`, `MenuEngine`, `ProtocolParser`).
- Almacenamiento no volátil (Flash de la Pico) para perfiles y macros.
- Lógica de sincronización de doble núcleo sin bloqueos.

---

## Plan de Verificación de Diseño

- **Revisión del Documento:** El usuario lee el documento modificado `interaccion_humano_maquina.md` con los diagramas y el protocolo serial y da su aprobación.
- **Prueba de Flujos:** Simularemos mentalmente o con diagramas de secuencia casos de uso como:
  1. Carmen conecta el teclado y funciona de inmediato como teclado numérico/ofimática.
  2. Álex entra al menú local con ambos encoders y cambia de perfil usando la navegación local.
  3. Marcos abre un juego, la app `OrbyGUI` lo detecta y envía la configuración de macros y los iconos correspondientes por serial.
