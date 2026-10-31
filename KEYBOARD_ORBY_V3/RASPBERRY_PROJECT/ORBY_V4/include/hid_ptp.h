#pragma once
// ==========================================================================
// Orby V4 - Touchpad de precisión (Windows Precision Touchpad)
// --------------------------------------------------------------------------
// Segunda interfaz HID, independiente de la del teclado/ratón/multimedia. Sirve
// para una sola cosa: que el zoom de la rueda llegue al sistema como un
// pellizco de dos dedos de trackpad, sin Ctrl de por medio.
//
// Va en interfaz aparte para que, si el host no la acepta o el descriptor falla,
// el teclado, la rueda y el multimedia no se enteren.
//
// Los IDs de informe son de ESTA interfaz: no chocan con los de hid_hires.h.
// ==========================================================================

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PTP_ITF_INSTANCE        1   // instancia HID (la 0 es el teclado)

#define PTP_REPORTID_TOUCHPAD   1   // Input: un contacto por informe (híbrido)
#define PTP_REPORTID_MAX_COUNT  2   // Feature: contactos máximos + tipo de pad
#define PTP_REPORTID_PTPHQA     3   // Feature: blob de certificación (256 B)
#define PTP_REPORTID_FEATURE    4   // Feature: modo de entrada (ratón / PTP)
#define PTP_REPORTID_FUNCSWITCH 5   // Feature: superficie y botones activos
#define PTP_REPORTID_MOUSE      6   // Input: ratón de reserva (no se usa)

// Valor del Feature «Input Mode» cuando el host quiere el touchpad de verdad.
#define PTP_INPUT_MODE_PTP      3

// Superficie táctil que se anuncia. 4095 unidades lógicas en 4,00 x 2,75 pulg.
#define PTP_X_MAX               4095
#define PTP_Y_MAX               4095

// Un dedo dentro del informe.
typedef struct __attribute__((packed)) {
    uint8_t  flags;          // bit0 confianza, bit1 punta apoyada, bits2-3 id
    uint16_t x;
    uint16_t y;
} orby_ptp_finger_t;

// Informe del touchpad: los dos dedos juntos (modo paralelo). Un cuadro en dos
// informes dejaba al host ver un dedo suelto entre uno y otro, y un toque de un
// solo dedo es un clic izquierdo.
typedef struct __attribute__((packed)) {
    orby_ptp_finger_t finger[2];
    uint16_t scan_time;      // décimas de milisegundo
    uint8_t  contact_count;
    uint8_t  buttons;
} orby_ptp_report_t;

// El host ha puesto el touchpad en modo PTP (Feature «Input Mode» = 3). Sin
// eso —Linux, macOS, Windows que no lo negocia— el zoom cae al Ctrl + rueda de
// siempre.
extern volatile uint8_t g_ptp_active;

#ifdef __cplusplus
}
#endif
