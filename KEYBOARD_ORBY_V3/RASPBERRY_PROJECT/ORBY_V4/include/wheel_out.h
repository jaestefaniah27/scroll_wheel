#pragma once
#include <stdint.h>
#include "tusb.h"
#include "hid_hires.h"
#include "hid_out.h"

// ============================================================================
// SALIDA DE LA RUEDA — informe de ratón
// ============================================================================
// Vertical y horizontal se manejan igual: los dos acumulan en unidades de alta
// resolución (120 por detent clásico) y el resto NUNCA se descarta, que es lo
// que provocaba el scroll a saltos.
//
// El horizontal era el hermano pobre: viajaba en detents enteros por un campo
// de 8 bits, así que el paneo iba a tirones aunque el vertical fuese suave.
namespace WheelOut {

static int32_t pending_wheel = 0;  // unidades verticales pendientes
static int32_t pending_pan   = 0;  // unidades horizontales pendientes

// El zoom necesita que el host tenga el Ctrl puesto ANTES de recibir la rueda;
// si no, lo interpreta como scroll normal. Mientras la compuerta esté echada,
// lo pendiente se guarda en vez de salir.
static bool zoom_gate = false;

// Sentido por defecto del paneo horizontal. El AC Pan del HID cuenta positivo
// hacia la derecha, pero en la rueda de ORBY el sentido natural es el
// contrario, así que se invierte aquí. Es el único punto por el que pasa todo
// el movimiento horizontal —la rueda magnética y los encoders configurados como
// paneo—, de modo que con esto quedan los dos igual. A 0 se vuelve al sentido
// de la especificación.
//
// Ojo: esto es el sentido POR DEFECTO. El interruptor de invertir de cada perfil
// sigue funcionando encima, partiendo de aquí.
#define ORBY_PAN_INVERT 1

static inline void add_wheel(int32_t units) { pending_wheel += units; }

static inline void add_pan(int32_t units) {
#if ORBY_PAN_INVERT
    pending_pan -= units;
#else
    pending_pan += units;
#endif
}

static inline void zoom_arm()  { zoom_gate = true; }

// Se llama al soltar el Ctrl del zoom. Sin esto, unas unidades que quedaran a
// medias dejaban la compuerta echada esperando un Ctrl que ya no iba a volver,
// y la rueda se quedaba muda para siempre.
static inline void zoom_disarm() { zoom_gate = false; }

static inline int32_t clamp16(int32_t v) {
    if (v >  32767) return  32767;
    if (v < -32767) return -32767;
    return v;
}

// Vacía las colas sin bloquear: si el endpoint está ocupado, lo pendiente se
// conserva para la siguiente vuelta en vez de perderse.
static void flush() {
    if (pending_wheel == 0 && pending_pan == 0) return;
    if (!tud_hid_ready()) return;
    if (zoom_gate && !(HidOut::sent_modifiers() & KEYBOARD_MODIFIER_LEFTCTRL)) return;

    int32_t v = clamp16(pending_wheel);
    int32_t h = clamp16(pending_pan);

    orby_mouse_report_t rpt = { 0, 0, 0, (int16_t)v, (int16_t)h };
    if (tud_hid_report(REPORT_ID_MOUSE, &rpt, sizeof(rpt))) {
        pending_wheel -= v;
        pending_pan   -= h;
    }
}

} // namespace WheelOut
