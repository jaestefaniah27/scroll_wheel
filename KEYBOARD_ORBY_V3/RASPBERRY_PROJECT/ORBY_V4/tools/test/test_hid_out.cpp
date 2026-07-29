// ============================================================================
// Pruebas de la salida HID (include/hid_out.h)
// ============================================================================
// Compilar y ejecutar:  tools/test/run.sh
//
// Se prueba contra un TinyUSB de mentira que apunta cada informe que sale, así
// que lo que se comprueba es exactamente lo que vería el PC.
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include "hid_out.h"

// --- TinyUSB de mentira -----------------------------------------------------
struct Report {
    uint8_t id;
    uint8_t modifier;
    uint8_t keys[6];
    uint16_t consumer;
};
static std::vector<Report> sent;
static bool endpoint_ready = true;

extern "C" bool tud_hid_ready(void) { return endpoint_ready; }

extern "C" bool tud_hid_keyboard_report(uint8_t id, uint8_t modifier, const uint8_t* keycode) {
    if (!endpoint_ready) return false;
    Report r = { id, modifier, {0,0,0,0,0,0}, 0 };
    if (keycode) memcpy(r.keys, keycode, 6);
    sent.push_back(r);
    return true;
}

extern "C" bool tud_hid_report(uint8_t id, const void* report, uint16_t len) {
    if (!endpoint_ready) return false;
    Report r = { id, 0, {0,0,0,0,0,0}, 0 };
    if (len >= 2) memcpy(&r.consumer, report, 2);
    sent.push_back(r);
    return true;
}

// --- Utilidades -------------------------------------------------------------
static int fails = 0, checks = 0;

static void pumps(int n) { for (int i = 0; i < n; i++) HidOut::pump(); }

static std::string fmt(const Report& r) {
    char b[96];
    if (r.id == REPORT_ID_CONSUMER) { snprintf(b, sizeof(b), "CONS(0x%04X)", r.consumer); return b; }
    snprintf(b, sizeof(b), "KB(mod=0x%02X %02X %02X %02X %02X %02X %02X)",
             r.modifier, r.keys[0], r.keys[1], r.keys[2], r.keys[3], r.keys[4], r.keys[5]);
    return b;
}

static void dump() {
    for (size_t i = 0; i < sent.size(); i++) printf("      [%u] %s\n", (unsigned)i, fmt(sent[i]).c_str());
}

static void check(bool cond, const char* what) {
    checks++;
    if (cond) { printf("  ok   %s\n", what); return; }
    fails++;
    printf("  FALLA %s\n", what);
    dump();
}

static bool kb_is(size_t i, uint8_t mod, uint8_t k0 = 0, uint8_t k1 = 0, uint8_t k2 = 0) {
    if (i >= sent.size()) return false;
    const Report& r = sent[i];
    return r.id == REPORT_ID_KEYBOARD && r.modifier == mod &&
           r.keys[0] == k0 && r.keys[1] == k1 && r.keys[2] == k2;
}

static void reset() { sent.clear(); endpoint_ready = true; HidOut::init(); }

#define KEY_A 0x04
#define KEY_B 0x05
#define KEY_C 0x06
#define CTRL  KEYBOARD_MODIFIER_LEFTCTRL
#define SHIFT KEYBOARD_MODIFIER_LEFTSHIFT

// --- Pruebas ----------------------------------------------------------------

// Lo que antes estaba roto: soltar una tecla soltaba todas las demás.
static void test_rollover() {
    printf("\nRollover de varias teclas a la vez\n");
    reset();

    HidOut::key_down(0, 0, KEY_A); pumps(2);
    check(kb_is(0, 0, KEY_A), "al pulsar A sale A");

    HidOut::key_down(1, 0, KEY_B); pumps(2);
    check(kb_is(1, 0, KEY_A, KEY_B), "al pulsar B siguen A y B");

    HidOut::key_down(2, 0, KEY_C); pumps(2);
    check(kb_is(2, 0, KEY_A, KEY_B, KEY_C), "al pulsar C siguen las tres");

    HidOut::key_up(1); pumps(2);
    check(kb_is(3, 0, KEY_A, KEY_C), "al soltar B, A y C SIGUEN pulsadas");

    HidOut::key_up(0); HidOut::key_up(2); pumps(2);
    check(kb_is(4, 0, 0, 0, 0), "al soltar el resto no queda nada");
}

// Dos teclas que comparten modificador: soltar una no puede llevarse el Ctrl.
static void test_modificadores_compartidos() {
    printf("\nModificadores compartidos entre teclas\n");
    reset();

    HidOut::key_down(0, CTRL, KEY_C); pumps(2);   // Ctrl+C
    HidOut::key_down(1, CTRL, KEY_A); pumps(2);   // Ctrl+A
    check(kb_is(1, CTRL, KEY_C, KEY_A), "las dos con Ctrl");

    HidOut::key_up(0); pumps(2);
    check(kb_is(2, CTRL, KEY_A), "al soltar una, el Ctrl de la otra se queda");

    HidOut::key_up(1); pumps(2);
    check(kb_is(3, 0, 0), "al soltar la última se va el Ctrl");
}

// Cambiar de capa con la tecla hundida: se suelta lo que se envió, no lo que
// diga el perfil ahora. Antes esto dejaba teclas pegadas.
static void test_cambio_de_capa_con_tecla_hundida() {
    printf("\nCambio de capa con una tecla hundida\n");
    reset();

    HidOut::key_down(4, SHIFT, KEY_B); pumps(2);
    check(kb_is(0, SHIFT, KEY_B), "sale Shift+B");

    // Aquí el usuario pulsa SUPER y el perfil pasaría a mandar otra cosa.
    HidOut::key_up(4); pumps(2);
    check(kb_is(1, 0, 0), "al soltar se va justo lo que se había enviado");
}

// Dos pasos seguidos del mismo encoder tienen que verse como dos pulsaciones,
// no fundirse en una.
static void test_pulsaciones_puntuales() {
    printf("\nPulsaciones puntuales de los encoders\n");
    reset();

    HidOut::tap(0, KEY_A);
    HidOut::tap(0, KEY_A);
    pumps(6);
    check(sent.size() == 4, "dos pasos producen cuatro informes");
    check(kb_is(0, 0, KEY_A) && kb_is(1, 0, 0) && kb_is(2, 0, KEY_A) && kb_is(3, 0, 0),
          "pulsa, suelta, pulsa, suelta");
}

// Un giro de encoder mientras se mantiene una tecla no puede soltarla.
static void test_giro_con_tecla_hundida() {
    printf("\nGiro de encoder con una tecla hundida\n");
    reset();

    HidOut::key_down(0, 0, KEY_A); pumps(2);
    HidOut::tap(CTRL, KEY_B); pumps(3);
    check(kb_is(1, CTRL, KEY_A, KEY_B), "la puntual se monta encima de la hundida");
    check(kb_is(2, 0, KEY_A), "al terminar, A sigue hundida");
}

static void test_multimedia() {
    printf("\nTeclas multimedia\n");
    reset();

    HidOut::consumer_tap(0x00E9); // subir volumen
    pumps(4);
    check(sent.size() == 2, "una multimedia son dos informes");
    check(sent[0].id == REPORT_ID_CONSUMER && sent[0].consumer == 0x00E9, "sale el código");
    check(sent[1].id == REPORT_ID_CONSUMER && sent[1].consumer == 0, "y luego la suelta");
}

// Con el endpoint ocupado no se pierde nada: se reintenta al liberarse.
static void test_endpoint_ocupado() {
    printf("\nEndpoint ocupado\n");
    reset();

    endpoint_ready = false;
    HidOut::key_down(0, 0, KEY_A);
    HidOut::tap(0, KEY_B);
    pumps(20);
    check(sent.empty(), "con el canal ocupado no sale nada");

    endpoint_ready = true;
    pumps(6);
    // Se funden en lo mínimo imprescindible: la puntual sale montada sobre la
    // hundida y después queda solo la hundida. Dos informes, nada perdido.
    check(sent.size() == 2, "al liberarse sale lo pendiente sin informes de más");
    check(kb_is(0, 0, KEY_A, KEY_B), "primero A y B juntas");
    check(kb_is(1, 0, KEY_A), "y luego A sola, que sigue hundida");
}

// Al suspender el PC o tirar del cable no puede quedarse nada pegado.
static void test_release_all() {
    printf("\nSuspensión y desconexión\n");
    reset();

    HidOut::key_down(0, CTRL, KEY_A);
    HidOut::key_down(1, 0, KEY_B);
    pumps(3);
    size_t antes = sent.size();

    HidOut::release_all();
    pumps(5);
    check(sent.size() == antes, "no se intenta enviar nada por un bus suspendido");

    // Y al soltar de verdad la tecla, no revive nada.
    HidOut::key_up(0); HidOut::key_up(1);
    pumps(3);
    check(sent.size() == antes, "soltar después no reenvía nada");
    check(HidOut::sent_modifiers() == 0, "no queda ningún modificador puesto");
}

// El Ctrl del zoom: la rueda no debe salir hasta que el host lo tenga.
static void test_zoom_sticky() {
    printf("\nCtrl pegajoso del zoom\n");
    reset();

    HidOut::set_sticky_mod(CTRL);
    check(HidOut::sent_modifiers() == 0, "antes de bombear, el host aun NO tiene el Ctrl");

    pumps(2);
    check(HidOut::sent_modifiers() == CTRL, "tras bombear, el Ctrl esta puesto");
    check(kb_is(0, CTRL, 0), "y salio como modificador solo, sin tecla");

    HidOut::set_sticky_mod(0);
    pumps(2);
    check(HidOut::sent_modifiers() == 0, "al dejar de girar se suelta");
}

// Una tecla normal pulsada mientras el zoom tiene el Ctrl puesto.
static void test_zoom_con_tecla() {
    printf("\nTecla pulsada durante el zoom\n");
    reset();

    HidOut::set_sticky_mod(CTRL);
    pumps(2);
    HidOut::key_down(0, 0, KEY_A);
    pumps(2);
    check(kb_is(1, CTRL, KEY_A), "la tecla sale junto al Ctrl del zoom");

    HidOut::set_sticky_mod(0);
    pumps(2);
    check(kb_is(2, 0, KEY_A), "al soltarse el zoom, la tecla sigue hundida");
}

int main() {
    printf("=== Pruebas de la salida HID ===\n");
    test_rollover();
    test_modificadores_compartidos();
    test_cambio_de_capa_con_tecla_hundida();
    test_pulsaciones_puntuales();
    test_giro_con_tecla_hundida();
    test_multimedia();
    test_endpoint_ocupado();
    test_release_all();
    test_zoom_sticky();
    test_zoom_con_tecla();

    printf("\n%d comprobaciones, %d fallos\n", checks, fails);
    return fails ? 1 : 0;
}
