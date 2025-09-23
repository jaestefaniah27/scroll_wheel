#pragma once
#include <Arduino.h>

enum class ButtonEvent : uint8_t {
  NONE = 0,
  R_SHORT, R_LONG,
  L_SHORT, L_LONG,
  RST_SHORT, RST_LONG
};

enum class WheelMode : uint8_t {
  SCROLL,
  VOLUME,
  PAN,
  ZOOM,
  SELECT
};
