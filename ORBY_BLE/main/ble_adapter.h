#pragma once
#include <Arduino.h>

// Puente hacia el main BLE original. Debes implementar esta función
// en tu main.ino (o en un .cpp BLE) como un wrapper que llame a tu
// sendReport(...) existente del stack BLE.
extern "C" void bleSendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH);
void setupHIDble();