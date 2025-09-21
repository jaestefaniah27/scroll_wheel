/*
 * ESP32 BLE HID - Mouse con rueda 16-bit y Resolution Multiplier (0x48)
 * Prueba simple: alterna scroll arriba/abajo cada 1 s
 * Librería: NimBLE-Arduino
 */

#include <Arduino.h>
#include "config.h"
#include "buttons.h"
#include "haptics.h"
#include "encoder_as5600.h"
#include "modes.h"
#include "wheel_output.h"
#include "hid_ble.h"  // para bleSendReport()

// ---------- Demo: alternar scroll arriba/abajo ----------
void setup() {
  initConfigPins();

  initHaptics();
  initButtons();
  initEncoder();

  setupHIDble();

  setWheelMode(WheelMode::SCROLL);
  setLedsForMode(getWheelMode());

  // Breve test visual/háptico de arranque (opcional)
  for (int i = 0; i < 2; ++i) {
    digitalWrite(Pins::LED_R, HIGH);
    digitalWrite(Pins::LED_L, HIGH);
    startHaptic(100);
    delay(110);
    digitalWrite(Pins::LED_R, LOW);
    digitalWrite(Pins::LED_L, LOW);
    updateHaptics();
    delay(100);
  }
}

void loop() {
  // UI siempre viva
  pollButtonsAndFeedback();
  updateHaptics();

  // if (delta != 0) {
  //   sendReport(0, 0, 0, -delta, 0);
  // }

  // Encoder sólo cuando haya bloque listo
  int16_t deltaTicks;
  if (sampleBlockAndGetDelta(deltaTicks)) {
    handleWheelDelta(deltaTicks);
  }
  
}
