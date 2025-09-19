#include "haptics.h"
#include "config.h"

namespace {
struct Haptic {
  bool active = false;
  unsigned long offAt = 0;
  void start(uint16_t ms) {
    digitalWrite(Pins::VIB, HIGH);
    active = true;
    offAt = millis() + ms;
  }
  void update() {
    if (active && (long)(millis() - offAt) >= 0) {
      digitalWrite(Pins::VIB, LOW);
      active = false;
    }
  }
} haptic;
} // namespace

void initHaptics() {
  // Pin ya configurado en initConfigPins()
  digitalWrite(Pins::VIB, LOW);
}

void startHaptic(uint16_t ms) { haptic.start(ms); }
void updateHaptics() { haptic.update(); }
