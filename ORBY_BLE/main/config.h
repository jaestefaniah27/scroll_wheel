#pragma once
#include <Arduino.h>

// ===== Pines =====
namespace Pins {
  constexpr uint8_t BTN_R = 19;
  constexpr uint8_t LED_R = 16;
  constexpr uint8_t BTN_L = 18;
  constexpr uint8_t LED_L = 17;
  constexpr uint8_t VIB   = 5;
  // I2C: SDA = 2, SCL = 3 (según placa)
}

// ===== AS5600 =====
namespace Enc {
  constexpr uint8_t  AS5600_ADDR   = 0x36;
  constexpr uint8_t  REG_STATUS    = 0x0B;
  constexpr unsigned long SAMPLE_US = 1000000UL / 1200; // ~833us (1200 Hz)
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
