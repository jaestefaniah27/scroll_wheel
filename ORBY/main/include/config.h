#pragma once
#include <Arduino.h>
#include "backend_select.h"

namespace Pins {
  #ifdef HID_BACKEND_USB
    // ---- Pines para el modo USB ----
    constexpr uint8_t BTN_R = 10;
    constexpr uint8_t LED_R = 16;
    constexpr uint8_t BTN_L = 9;
    constexpr uint8_t LED_L = 8;
    constexpr uint8_t VIB   = 5;
    constexpr uint8_t RST_CON = 4; // INT
    // I2C (si aplica)
    constexpr uint8_t I2C_SDA = 2;
    constexpr uint8_t I2C_SCL = 3;
  #endif
  #ifdef HID_BACKEND_BLE
    // ---- Pines para el modo BLE ----
    constexpr uint8_t BTN_R = 33;
    constexpr uint8_t LED_R = 17;
    constexpr uint8_t BTN_L = 32;
    constexpr uint8_t LED_L = 16;
    constexpr uint8_t VIB   = 5;
    constexpr uint8_t RST_CON = 0; // INT
    // I2C (si aplica)
    // constexpr uint8_t I2C_SDA = 21;
    // constexpr uint8_t I2C_SCL = 22;
  #endif

} // namespace Pins

// ===== AS5600 =====
namespace Enc {
  constexpr uint8_t  AS5600_ADDR   = 0x36;
  constexpr uint8_t  REG_STATUS    = 0x0B;
  constexpr unsigned long SAMPLE_US = 1000000UL / 600; // ~833us (1200 Hz)
  constexpr uint8_t  SAMPLES_PER_BLOCK = 10;            // 120 Hz de bloque
}

// ===== UI / Timings / Divisores =====
namespace Ui {
  constexpr uint16_t DEBOUNCE_MS   = 25;
  constexpr uint16_t LONG_MS       = 300;
  constexpr uint16_t LONG_HIT_MS   = 80;
  constexpr int      VOL_TICK_DIV  = 64;
  constexpr int      ZOOM_TICK_DIV = 5;
  constexpr int16_t  DELTA_MAX     = 32767;
}

// Inicialización básica de pines
inline void initConfigPins() {
  pinMode(Pins::LED_R, OUTPUT);
  pinMode(Pins::LED_L, OUTPUT);
  pinMode(Pins::VIB,   OUTPUT);
  digitalWrite(Pins::LED_R, LOW);
  digitalWrite(Pins::LED_L, LOW);
  digitalWrite(Pins::VIB,   LOW);
}
