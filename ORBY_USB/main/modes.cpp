#include "modes.h"
#include "config.h"
#include "haptics.h"
#include <HID-Project.h>   // Keyboard, Mouse

namespace {
  WheelMode wheel_mode = WheelMode::SCROLL;
}

void setWheelMode(WheelMode m) { wheel_mode = m; }
WheelMode getWheelMode() { return wheel_mode; }

void setLedsForMode(WheelMode m) {
  switch (m) {
    case WheelMode::PAN:    digitalWrite(Pins::LED_R, HIGH); digitalWrite(Pins::LED_L, LOW);  break;
    case WheelMode::ZOOM:   digitalWrite(Pins::LED_R, HIGH); digitalWrite(Pins::LED_L, HIGH); break;
    case WheelMode::VOLUME: digitalWrite(Pins::LED_R, LOW);  digitalWrite(Pins::LED_L, HIGH); break;
    case WheelMode::SELECT: digitalWrite(Pins::LED_R, LOW);  digitalWrite(Pins::LED_L, HIGH); break;
    case WheelMode::SCROLL: default:
                             digitalWrite(Pins::LED_R, LOW);  digitalWrite(Pins::LED_L, LOW);  break;
  }
}

void changeMode(ButtonEvent ev) {
  const WheelMode prev = wheel_mode;

  switch (ev) {
    case ButtonEvent::R_SHORT: // PAN <-> SCROLL
      wheel_mode = (wheel_mode != WheelMode::PAN) ? WheelMode::PAN : WheelMode::SCROLL;
      break;

    case ButtonEvent::R_LONG:  // ZOOM <-> SCROLL  (Ctrl sticky mientras dure el modo)
      wheel_mode = (wheel_mode != WheelMode::ZOOM) ? WheelMode::ZOOM : WheelMode::SCROLL;
      if (wheel_mode == WheelMode::ZOOM) Keyboard.press(KEY_LEFT_CTRL);
      break;

    case ButtonEvent::L_SHORT: // SELECT <-> SCROLL
      wheel_mode = (wheel_mode != WheelMode::SELECT) ? WheelMode::SELECT : WheelMode::SCROLL;
      if (wheel_mode == WheelMode::SELECT) Mouse.press(MOUSE_LEFT);
      break;

    case ButtonEvent::L_LONG:  // VOLUME <-> SCROLL
      wheel_mode = (wheel_mode != WheelMode::VOLUME) ? WheelMode::VOLUME : WheelMode::SCROLL;
      break;

    default: return;
  }

  if (wheel_mode != prev) {
    setLedsForMode(wheel_mode);
    startHaptic(140);

    if (prev == WheelMode::SELECT && wheel_mode != WheelMode::SELECT) {
      Mouse.release(MOUSE_LEFT);
    }
    if (prev == WheelMode::ZOOM && wheel_mode != WheelMode::ZOOM) {
      Keyboard.release(KEY_LEFT_CTRL);
    }
  }
}
