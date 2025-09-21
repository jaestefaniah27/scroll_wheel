#include "buttons.h"
#include "config.h"
#include "events.h"
#include "haptics.h"
#include "modes.h"

namespace {

// Resultado interno del poll
enum class PollResult : uint8_t { NONE=0, SHORT=1, LONG=2, LONG_HIT=3 };

struct ButtonFSM {
  uint8_t pin = 0;
  bool activeLow = true;

  bool  stableLevel = true;
  bool  lastReadLevel = true;
  unsigned long lastChangeMs = 0;
  bool  pressed = false;
  unsigned long pressStartMs = 0;
  bool  longNotified = false;

  void begin(uint8_t _pin, bool _activeLow=true) {
    pin = _pin; activeLow = _activeLow;
    pinMode(pin, activeLow ? INPUT_PULLUP : INPUT);
    lastReadLevel = digitalRead(pin);
    stableLevel   = lastReadLevel;
    lastChangeMs  = millis();
    pressed       = false;
    pressStartMs  = 0;
    longNotified  = false;
  }

  inline bool logicalPressed(bool level) const {
    return activeLow ? (level == LOW) : (level == HIGH);
  }

  PollResult poll() {
    bool raw = digitalRead(pin);
    const unsigned long now = millis();

    if (raw != lastReadLevel) {
      lastReadLevel = raw;
      lastChangeMs  = now;
    }

    if ((now - lastChangeMs) >= Ui::DEBOUNCE_MS && stableLevel != lastReadLevel) {
      stableLevel = lastReadLevel;
      const bool isPressed = logicalPressed(stableLevel);

      if (isPressed && !pressed) {
        pressed = true;
        pressStartMs = now;
        longNotified = false;
      } else if (!isPressed && pressed) {
        pressed = false;
        const uint32_t held = now - pressStartMs;
        longNotified = false;
        return (held >= Ui::LONG_MS) ? PollResult::LONG : PollResult::SHORT;
      }
    }

    if (pressed && !longNotified) {
      const uint32_t held = now - pressStartMs;
      if (held >= Ui::LONG_MS) {
        longNotified = true;
        return PollResult::LONG_HIT;
      }
    }

    return PollResult::NONE;
  }
};

// Instancias internas
ButtonFSM btnR, btnL;

} // namespace

void initButtons() {
  btnR.begin(Pins::BTN_R, true);
  btnL.begin(Pins::BTN_L, true);
}

void pollButtonsAndFeedback() {
  // Derecha
  switch (btnR.poll()) {
    case PollResult::LONG_HIT: startHaptic(Ui::LONG_HIT_MS); break;
    case PollResult::SHORT:    changeMode(ButtonEvent::R_SHORT); break;
    case PollResult::LONG:     changeMode(ButtonEvent::R_LONG);  break;
    default: break;
  }
  // Izquierda
  switch (btnL.poll()) {
    case PollResult::LONG_HIT: startHaptic(Ui::LONG_HIT_MS); break;
    case PollResult::SHORT:    changeMode(ButtonEvent::L_SHORT); break;
    case PollResult::LONG:     changeMode(ButtonEvent::L_LONG);  break;
    default: break;
  }
}
