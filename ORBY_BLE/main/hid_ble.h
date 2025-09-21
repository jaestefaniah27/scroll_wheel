#pragma once
#include <Arduino.h>

#include <stdint.h>

// ================= Ratón =================
// Botones
#define MOUSE_LEFT    0x01
#define MOUSE_RIGHT   0x02
#define MOUSE_MIDDLE  0x04
#define MOUSE_BTN4    0x08
#define MOUSE_BTN5    0x10

// API ratón (Report ID 1)
// Envia un paquete de 7 bytes: botones, x, y, wheelV(16b), wheelH(16b)
void bleSendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH);


// ================= Consumer Control =================
// Usages HID Consumer Page (0x0C)
#define CC_MUTE       0x00E2
#define CC_VOL_UP     0x00E9
#define CC_VOL_DOWN   0x00EA

// API consumer control (Report ID 2)
void bleConsumerClick(uint16_t usage);


// ================= Inicialización HID BLE =================
void setupHIDble(void);