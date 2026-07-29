// Sustituto mínimo de TinyUSB para poder probar hid_out.h en el PC.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define KEYBOARD_MODIFIER_LEFTCTRL  0x01
#define KEYBOARD_MODIFIER_LEFTSHIFT 0x02

#ifdef __cplusplus
extern "C" {
#endif
bool tud_hid_ready(void);
bool tud_hid_keyboard_report(uint8_t report_id, uint8_t modifier, const uint8_t* keycode);
bool tud_hid_report(uint8_t report_id, const void* report, uint16_t len);
#ifdef __cplusplus
}
#endif
