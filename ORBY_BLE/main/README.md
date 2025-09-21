
# Refactor BLE — Igual forma que USB (sin tocar la pila BLE)

Este refactor **no modifica la lógica BLE** (callbacks, descriptors, advertising, etc.).
Solo extrae todo lo demás (botones, encoder, háptica, modos y salida de rueda) a módulos
como en `ORBY_USB/main`. Así puedes mantener el *main.ino* BLE tal cual para la parte
Bluetooth y usar exactamente la misma estructura modular.

## Estructura nueva (en `ORBY_BLE_refactored/main/`)
- `config.h` — Pines, constantes de UI y encoder.
- `events.h` — Tipos `ButtonEvent` y `WheelMode`.
- `buttons.{h,cpp}` — FSM de botones + feedback.
- `haptics.{h,cpp}` — Control del motor ERM/LRA (vibración).
- `encoder_as5600.{h,cpp}` — Muestreo en bloque (AS5600 @ I2C).
- `modes.{h,cpp}` — Cambio de modo + LEDs (sin teclado BLE).
- `wheel_output.{h,cpp}` — Aplica deltas según modo y llama a `bleSendReport(...)`.
- `hid_ble.h` — **Puente**: declara `bleSendReport(...)` que debes implementar
  en tu `main.ino` BLE para redirigir al `sendReport(...)` que ya tienes.

> Nota: se han **omitido** las llamadas a `Keyboard` y `Consumer` del USB para no
> tocar la pila BLE. Si más adelante añades un servicio BLE de teclado/consumer,
> puedes reactivar esas líneas fácilmente (están comentadas en el código).

## Cómo integrar en tu `ORBY_BLE/main/main.ino` sin romper BLE

1) **Incluye** los módulos en la cabecera de tu `main.ino` BLE, bajo tus includes BLE:

```cpp
#include "config.h"
#include "events.h"
#include "buttons.h"
#include "haptics.h"
#include "encoder_as5600.h"
#include "modes.h"
#include "wheel_output.h"
#include "hid_ble.h"  // declara bleSendReport(...)
```

2) **Implementa el puente** hacia tu `sendReport(...)` BLE existente **sin cambiarlo**:

```cpp
// En tu main.ino (o en un .cpp BLE), añade SOLO este wrapper:
extern "C" void bleSendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH) {
  // Llama a tu función BLE original (no la modifiques)
  sendReport(buttons, x, y, wheelV, wheelH);
}
```

> No renombres ni edites tu `sendReport(...)` original. El wrapper solo delega.

3) En `setup()`, **no cambies** tu init BLE. Añade, después de inicializar BLE:

```cpp
initConfigPins();
initHaptics();
initButtons();
initEncoder();
centerPosition();
setWheelMode(WheelMode::SCROLL);
setLedsForMode(getWheelMode());
```

4) En `loop()`, **sustituye** tu lógica de lectura directa por el bucle modular:

```cpp
// UI siempre viva
pollButtonsAndFeedback();
updateHaptics();

// Encoder solo entrega cuando hay bloque listo
int16_t deltaTicks;
if (sampleBlockAndGetDelta(deltaTicks)) {
  handleWheelDelta(deltaTicks);     // esto termina en bleSendReport(...)
}
```

> Deja intacto todo lo relativo a NimBLE (advertising, conexiones, callbacks, mapas
> de reporte, características, etc.). Solo reemplaza tu lectura de botones/encoder
> y generación de salidas por estas llamadas modulares.

## ¿Qué gana esto?
- Misma **forma** que el proyecto USB → mantenimiento y features comunes más fáciles.
- La **pila BLE** queda tal cual (probada), aislada en tu `main.ino`.
- Reutilizas `buttons`, `encoder`, `modes`, `haptics` y `wheel_output` en ambos targets.

## Notas
- `modes.cpp` y `wheel_output.cpp` traen las mismas APIs que en USB pero con las
  partes de `Keyboard`/`Consumer` **comentadas** para no alterar tu BLE actual.
  Si ya incluyes esos usos en tu descriptor BLE, descoméntalos y listo.
- Si tu pinout difiere, ajusta `config.h` (pines y divisores).
- El muestreo del AS5600 usa parámetros idénticos a USB; ciñe `Enc::SAMPLE_*` si es necesario.
