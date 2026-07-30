#pragma once
// ==========================================================================
// Orby V4 - HID High-Resolution Scrolling
// --------------------------------------------------------------------------
// Definiciones compartidas entre el descriptor USB (C) y la lógica de la
// rueda de scroll (C++).
//
// Windows solo activa el scroll suave (pixel a pixel) si el descriptor HID
// declara un "Resolution Multiplier" (Usage 0x48) dentro de una Logical
// Collection junto al Wheel. El host escribe entonces un Feature Report
// pidiendo el multiplicador; a partir de ese momento cada unidad enviada en
// el campo Wheel vale 1/HID_HIRES_MULTIPLIER de un "detent" clásico.
// ==========================================================================

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REPORT_ID_KEYBOARD  1
#define REPORT_ID_MOUSE     2
#define REPORT_ID_CONSUMER  3

// Un "detent" clásico de rueda equivale a WHEEL_DELTA = 120 en Win32.
#define HID_HIRES_MULTIPLIER 120

// Resolución del encoder magnético AS5600: 4096 cuentas por vuelta.
#define AS5600_COUNTS_PER_REV 4096

// Estado del multiplicador negociado con el host.
//   0 -> el host no ha pedido alta resolución (modo detent clásico)
//   1 -> alta resolución activa (multiplicador físico = HID_HIRES_MULTIPLIER)
extern volatile uint8_t g_hires_multiplier;

// Lo mismo para el eje horizontal (AC Pan). El host los negocia por separado,
// así que puede activar uno y no el otro.
extern volatile uint8_t g_hires_pan;

// Informe de ratón extendido: las dos ruedas de 16 bits. El paneo era de 8 bits
// y viajaba en detents enteros, así que el scroll horizontal iba a tirones
// aunque el vertical fuese suave.
typedef struct __attribute__((packed)) {
    uint8_t buttons;
    int8_t  x;
    int8_t  y;
    int16_t wheel;   // vertical, en unidades de alta resolución
    int16_t pan;     // horizontal (AC Pan), en las mismas unidades
} orby_mouse_report_t;

#ifdef __cplusplus
}
#endif
