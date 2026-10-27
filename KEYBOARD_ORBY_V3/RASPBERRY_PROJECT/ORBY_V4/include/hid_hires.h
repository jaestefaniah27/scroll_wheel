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
// Suspender / apagar el PC (página Generic Desktop, System Control).
#define REPORT_ID_SYSTEM    4
// Puntero absoluto: colocar el cursor en un punto exacto de la pantalla.
#define REPORT_ID_ABS       5

// Valores del informe de System Control. El descriptor los declara como un
// array con mínimo lógico 1: el 0 es "nada pulsado", que es la suelta.
#define SYSCTL_POWER_DOWN 1
#define SYSCTL_SLEEP      2
#define SYSCTL_WAKE_UP    3

// Rango de las coordenadas del puntero absoluto. El host lo reparte por toda
// la superficie que le toque; la app convierte píxeles a este rango al subir
// la secuencia (ver src/native/ en OrbyGUI).
#define ABS_POINTER_MAX 32767

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

// Informe del puntero absoluto. Lleva botones propios para que una grabación
// pueda arrastrar (pulsar, mover, soltar) en el mismo dispositivo que coloca
// el cursor.
typedef struct __attribute__((packed)) {
    uint8_t  buttons;
    uint16_t x;      // 0..ABS_POINTER_MAX
    uint16_t y;
} orby_abs_report_t;

// LEDs que el host le pide al teclado (bit 0 Bloq Num, bit 1 Bloq Mayús). Es
// lo único que dice cómo va a interpretar Windows una tecla: con Bloq Mayús
// puesto, Shift+a sale "a", y los códigos Alt+teclado numérico solo funcionan
// con Bloq Num. Lo escribe tud_hid_set_report_cb.
extern volatile uint8_t g_host_leds;
#define HOST_LED_NUMLOCK  0x01
#define HOST_LED_CAPSLOCK 0x02

#ifdef __cplusplus
}
#endif
