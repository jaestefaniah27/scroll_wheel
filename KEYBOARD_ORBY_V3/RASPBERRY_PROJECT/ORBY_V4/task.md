# Tareas: Diseño y Desarrollo del Firmware Orby

- [x] Analizar el código base existente (`main.cpp`, `pinout.h`, `hardware_oled.h`, `main_diag.cpp`).
- [x] Crear el Plan de Implementación inicial y plantear las preguntas de diseño al usuario.
- [x] Obtener feedback del usuario para determinar el camino a seguir (Opción A vs Opción B).
- [x] Completar y refinar el documento `interaccion_humano_maquina.md`:
    - [x] Detallar la especificación del hardware elegido.
    - [x] Agregar el Diagrama JSD (Jackson System Development).
    - [x] Agregar el Diagrama STN (State Transition Network) de los menús.
    - [x] Definir el protocolo formal de comunicación serial PC-Teclado.
- [x] Desarrollar e implementar el firmware en C++ según el diseño acordado.
- [x] Desarrollar o adaptar la interfaz gráfica en `OrbyGUI` para integrarla con el firmware.
- [ ] Validar e implementar pruebas de verificación integradas.

## Firmware 2.0 — scroll de alta resolución (hecho)

- [x] Descriptor HID de ratón con Resolution Multiplier (usage 0x48) y rueda de 16 bits.
- [x] Callbacks GET/SET_FEATURE para negociar el multiplicador con Windows.
- [x] PID cambiado a 0x4006 para invalidar el descriptor cacheado por el host.
- [x] Intervalo del endpoint HID bajado de 5 ms a 1 ms.
- [x] AS5600: muestreo a 1 kHz, salida a 250 Hz, filtro EMA sobre la posición,
      banda muerta de arranque y comprobación del imán cada 500 ms en vez de por
      cada iteración del bucle.
- [x] Acumulador de scroll que conserva el resto (era la causa directa de los saltos).
- [x] Sensibilidad configurable de 6 a 240 clics por vuelta (por defecto 60).
- [x] Telemetría `WHEEL:` agregada a 20 Hz para no saturar el CDC.

## Firmware 2.0 — configuración editable (hecho)

- [x] Perfiles mutables en RAM con persistencia versionada en Flash.
- [x] Banco de bitmaps OLED personalizados (20 huecos por perfil).
- [x] Escritura en Flash por perfil, para no dejar el USB mudo durante el borrado.
- [x] Comandos `GET_STATE`, `GET_PROFILE`, `SET_NAME`, `SET_LABEL`, `SET_KEYMAP`,
      `SET_SCROLL`, `SET_SCROLL_INV`, `GET_SCROLL`, `OLED_CHUNK`, `OLED_CLEAR`,
      `RESET_DEFAULTS`.

## OrbyGUI 2.0 (hecho)

- [x] Sin recursos remotos: tipografía del sistema e iconos SVG en línea.
- [x] Empaquetado con electron-builder (`npm run dist`).
- [x] Consola con búfer circular, filtro, pausa e historial.
- [x] Botón «Guardar en Flash» con indicador de cambios pendientes.
- [x] Editor de perfiles real, leyendo del firmware (sin datos duplicados).
- [x] Captura de atajos pulsándolos en el teclado del PC.
- [x] Vista de calibración de la rueda con estado de la alta resolución.
- [x] Editor de iconos OLED con importación de imagen y difuminado.
- [x] Editor OLED v2: lienzo a altura completa con zoom 3×-28×, selector de
      destino como réplica del teclado (icono real + atajo de cada tecla),
      capa flotante movible y escalable para imagen y texto, suavizado de
      bordes previo al umbral y 11 fuentes del sistema.
- [x] `GET_OLED` en el firmware para leer los bitmaps guardados.

## Encoders y rueda configurables (hecho)

- [x] Modelo `RotaryAction` con 8 huecos por perfil: dos encoders (giro horario,
      antihorario y pulsación) más los dos sentidos de la rueda magnética.
- [x] Tipos: nada, multimedia, atajo de teclado, desplazamiento vertical,
      desplazamiento horizontal (AC Pan) y zoom (Ctrl + rueda).
- [x] Tabla de teclas de consumo ampliada a 16 entradas conservando los índices
      antiguos. Corregidas las de brillo: el código usaba 0x00B0/0x00B1, que en
      la página de consumo son Play y Pause, no brillo.
- [x] Cola de paneo horizontal junto a la vertical en el informe de ratón.
- [x] Comandos `SET_ROTARY` y volcado `PROF:<n>:ROT:...` en `GET_PROFILE`.
- [x] Editor en la vista Perfiles, con espejado automático del sentido contrario
      en las acciones de desplazamiento.

## Protección de la configuración (hecho)

- [x] Migración de ajustes entre versiones de formato en vez de descartarlos.
      Los bitmaps OLED viven en su propia región de Flash y sobreviven al
      flasheo, pero la máscara de huecos ocupados está en el sector de ajustes:
      invalidarla equivalía a perder los iconos.
- [x] Rutas de migración desde `0xDEB001CE` (v1) y `0xDEB00204` (v2).
- [x] Copia de seguridad completa a un archivo JSON del PC (perfiles, mandos e
      iconos en hexadecimal) con restauración.
- [x] Guía de uso en `OrbyGUI/README.md`.

## Auto-cambio de perfil por aplicación (hecho)

- [x] Detector de ventana activa sin módulos nativos: PowerShell de vida larga
      con P/Invoke a `GetForegroundWindow`, arrancado con `-EncodedCommand`.
- [x] Motor de reglas con prioridad por orden y comparación contra ejecutable,
      título de ventana o ambos.
- [x] Perfil de reserva cuando ninguna regla encaja.
- [x] Configuración local en `%APPDATA%\OrbyGUI\orby-config.json` (no ocupa Flash).
- [x] Vista con estado en vivo, sugerencias rápidas y botón «añadir la app actual».

## Arranque de la app (hecho)

- [x] `OrbyGUI.bat`: doble clic → instala si hace falta, compila y abre.
- [x] Eliminada la dependencia `wait-on` (no estaba instalada y rompía `npm run dev`);
      Electron reintenta la carga del servidor de Vite.
- [x] Modo desarrollo por flag `--dev` en vez de deducirlo de `app.isPackaged`.
- [x] Aviso claro al conectar un teclado con firmware anterior al 2.0.

## Pendiente

- [ ] Probar en hardware: flashear `build/ORBY_V4.uf2` y verificar el scroll suave.
- [ ] Pruebas de verificación integradas.
