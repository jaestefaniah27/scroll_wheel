
#include "config.h"
#include "events.h"
#include "buttons.h"
#include "haptics.h"
#include "encoder_as5600.h"
#include "hid_adapter.h"
#include "modes.h"
#include "wheel_output.h"

void test()
{
  for (int i = 0; i < 2; ++i)
  {
    digitalWrite(Pins::LED_R, HIGH);
    digitalWrite(Pins::LED_L, HIGH);
    startHaptic(100);
    delay(100);
    updateHaptics();
    digitalWrite(Pins::LED_R, LOW);
    digitalWrite(Pins::LED_L, LOW);
    delay(100);
  }
}

void setup()
{
  initConfigPins();

  initHaptics();
  initButtons();
  initEncoder();
  hidSetup();

  setWheelMode(WheelMode::SCROLL);
  setLedsForMode(getWheelMode());

  // Breve test visual/háptico de arranque (opcional)
  test();
  orbyLoadFromEEPROM();
  Serial.begin(115200);
}

void loop()
{
  // UI siempre viva
  pollButtonsAndFeedback();
  updateHaptics();

  // Encoder sólo cuando haya bloque listo
  int16_t deltaTicks;
  if (sampleBlockAndGetDelta(deltaTicks))
  {
    handleWheelDelta(deltaTicks);
  }

  orbyProcessSerial(); // <- añade esta llamada
}
