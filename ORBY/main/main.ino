
#include "config.h"
#include "events.h"
#include "buttons.h"
#include "haptics.h"
#include "encoder_as5600.h"
#include "hid_adapter.h"
#include "modes.h"
#include "wheel_output.h"

void setup() {
  initConfigPins();

  initHaptics();
  initButtons();
  initEncoder();
  hidSetup();

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

  // Encoder sólo cuando haya bloque listo
  int16_t deltaTicks;
  if (sampleBlockAndGetDelta(deltaTicks)) {
    handleWheelDelta(deltaTicks);
  }
}
