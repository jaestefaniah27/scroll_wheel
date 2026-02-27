
#include "config.h"
#include "events.h"
#include "buttons.h"
#include "haptics.h"
#include "encoder_as5600.h"
#include "hid_adapter.h"
#include "config_runtime.h"
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
  hidUsbFeatureBegin();
  // 2) Carga defaults a RAM y aplícalos (si no lo hace ya tu constructor/EEPROM)
  sw_feature_config_t cfg;
  sw_storage_load_defaults(&cfg); // o: lee EEPROM si ya lo implementaste dentro
  sw_runtime_apply_config(&cfg);

  setWheelMode(WheelMode::SCROLL);
  setLedsForMode(getWheelMode());

  // Breve test visual/háptico de arranque (opcional)
  test();
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
}
