// ============================================================================
// Contienda por el endpoint HID entre la rueda y el teclado
// ============================================================================
// Reproduce el bucle principal: el endpoint solo acepta un informe por
// intervalo de 1 ms, y el bucle gira cada 250 us. La pregunta es si la rueda
// sigue saliendo mientras se mantiene una tecla pulsada.
#include <stdio.h>
#include <string.h>
#include "hid_out.h"
#include "wheel_out.h"

// --- Endpoint de mentira: ocupado durante 1 ms tras cada informe ------------
static uint32_t now_us = 0;
static uint32_t busy_until_us = 0;
static int kb_reports = 0, mouse_reports = 0;
static int32_t wheel_recibido = 0, pan_recibido = 0;

extern "C" bool tud_hid_ready(void) { return now_us >= busy_until_us; }

extern "C" bool tud_hid_keyboard_report(uint8_t, uint8_t, const uint8_t*) {
    if (!tud_hid_ready()) return false;
    busy_until_us = now_us + 1000;
    kb_reports++;
    return true;
}

extern "C" bool tud_hid_report(uint8_t id, const void* report, uint16_t) {
    if (!tud_hid_ready()) return false;
    busy_until_us = now_us + 1000;
    if (id == REPORT_ID_MOUSE) {
        const orby_mouse_report_t* r = (const orby_mouse_report_t*)report;
        wheel_recibido += r->wheel;
        pan_recibido   += r->pan;
        mouse_reports++;
    } else {
        kb_reports++;
    }
    return true;
}

// Una vuelta del bucle principal, en el mismo orden que main.cpp.
static void loop_iteration() {
    WheelOut::flush();
    HidOut::pump();
    now_us += 250;
}

static int fails = 0, checks = 0;
static void check(bool c, const char* what) {
    checks++;
    if (c) { printf("  ok   %s\n", what); return; }
    fails++; printf("  FALLA %s\n", what);
}

static void reset() {
    now_us = 10000; busy_until_us = 0;
    kb_reports = mouse_reports = 0;
    wheel_recibido = pan_recibido = 0;
    HidOut::pending_release = 0;
    HidOut::init();
    WheelOut::pending_wheel = 0;
    WheelOut::pending_pan = 0;
    WheelOut::zoom_disarm();
}

#define KEY_C 0x06
#define CTRL  KEYBOARD_MODIFIER_LEFTCTRL

// El caso denunciado: la rueda con una tecla mantenida.
static void test_rueda_con_tecla_mantenida() {
    printf("\nRueda girando con una tecla mantenida\n");
    reset();

    // Tecla mapeada a Ctrl+C, mantenida todo el rato.
    HidOut::key_down(0, CTRL, KEY_C);

    // 200 ms de bucle; la rueda entrega 120 unidades (un detent) cada 4 ms,
    // que es la cadencia real del AS5600.
    int32_t enviado = 0;
    for (int i = 0; i < 800; i++) {
        if (i % 16 == 0) { WheelOut::add_wheel(120); enviado += 120; }
        loop_iteration();
    }

    printf("      informes de raton=%d, de teclado=%d\n", mouse_reports, kb_reports);
    printf("      unidades enviadas=%d, recibidas=%d\n", (int)enviado, (int)wheel_recibido);
    check(mouse_reports > 0, "la rueda SIGUE saliendo con la tecla mantenida");
    check(wheel_recibido == enviado, "no se pierde ni una unidad de scroll");
    check(kb_reports == 1, "la tecla mantenida solo genera un informe, no inunda el canal");
}

// Y con la tecla pulsándose y soltándose sin parar mientras se gira.
static void test_rueda_con_teclas_aporreando() {
    printf("\nRueda girando mientras se aporrea una tecla\n");
    reset();

    int32_t enviado = 0;
    for (int i = 0; i < 800; i++) {
        if (i % 16 == 0) { WheelOut::add_wheel(120); enviado += 120; }
        if (i % 40 == 0)  HidOut::key_down(0, CTRL, KEY_C);
        if (i % 40 == 20) HidOut::key_up(0);
        loop_iteration();
    }
    printf("      informes de raton=%d, de teclado=%d\n", mouse_reports, kb_reports);
    check(wheel_recibido == enviado, "tampoco se pierde scroll aporreando");
}

// La compuerta del zoom no puede quedarse echada cuando el zoom termina.
static void test_compuerta_zoom_no_se_atasca() {
    printf("\nLa compuerta del zoom no se atasca\n");
    reset();

    // Zoom: Ctrl pegajoso y unidades pendientes.
    HidOut::set_sticky_mod(CTRL);
    WheelOut::zoom_arm();
    WheelOut::add_wheel(120);
    for (int i = 0; i < 20; i++) loop_iteration();
    check(wheel_recibido == 120, "con el Ctrl puesto, el zoom sale");

    // Deja de girar: se suelta el Ctrl. Aqui es donde antes quedaba pendiente
    // media unidad y la compuerta se quedaba echada para siempre.
    WheelOut::add_wheel(60);       // unidades a medias
    HidOut::set_sticky_mod(0);
    WheelOut::zoom_disarm();
    for (int i = 0; i < 20; i++) loop_iteration();

    WheelOut::add_wheel(120);      // scroll normal, ya sin Ctrl
    for (int i = 0; i < 20; i++) loop_iteration();
    check(wheel_recibido == 300, "y despues el scroll normal sigue saliendo");
}

// El signo con el que sale el paneo. Va aparte porque es una decision de
// producto, no un detalle: si alguien toca ORBY_PAN_INVERT sin querer, esta
// prueba lo canta.
#define PAN_SIGN (ORBY_PAN_INVERT ? -1 : 1)

static void test_sentido_del_paneo() {
    printf("\nSentido del paneo horizontal\n");
    reset();

    WheelOut::add_pan(120);   // un detent en el sentido crudo de la rueda
    for (int i = 0; i < 10; i++) loop_iteration();
    check(pan_recibido == 120 * PAN_SIGN,
          ORBY_PAN_INVERT ? "va invertido respecto al AC Pan del HID"
                          : "va en el sentido del AC Pan del HID");
    check(ORBY_PAN_INVERT == 1, "y el sentido por defecto es el invertido");
}

// El paneo horizontal tiene que ser tan fino como el vertical.
static void test_paneo_alta_resolucion() {
    printf("\nPaneo horizontal de alta resolucion\n");
    reset();

    // Un tercio de detent: con el campo de 8 bits en detents esto era 0.
    WheelOut::add_pan(40);
    for (int i = 0; i < 10; i++) loop_iteration();
    check(pan_recibido == 40 * PAN_SIGN, "sale un tercio de detent sin redondear a cero");

    reset();
    WheelOut::add_pan(5000);   // mas de lo que cabia en int8
    for (int i = 0; i < 10; i++) loop_iteration();
    check(pan_recibido == 5000 * PAN_SIGN, "y un paneo grande no se recorta a 127");

    reset();
    WheelOut::add_pan(80000);  // por encima del tope del campo de 16 bits
    for (int i = 0; i < 40; i++) loop_iteration();
    check(pan_recibido == 80000 * PAN_SIGN, "lo que no cabe en un informe se manda en varios");

    // Y el vertical no se ha tocado: sigue en el sentido de siempre.
    reset();
    WheelOut::add_wheel(120);
    for (int i = 0; i < 10; i++) loop_iteration();
    check(wheel_recibido == 120, "el vertical sigue como estaba, sin invertir");
}

int main() {
    printf("=== Contienda por el endpoint y paneo ===\n");
    test_rueda_con_tecla_mantenida();
    test_rueda_con_teclas_aporreando();
    test_compuerta_zoom_no_se_atasca();
    test_sentido_del_paneo();
    test_paneo_alta_resolucion();
    printf("\n%d comprobaciones, %d fallos\n", checks, fails);
    return fails ? 1 : 0;
}
