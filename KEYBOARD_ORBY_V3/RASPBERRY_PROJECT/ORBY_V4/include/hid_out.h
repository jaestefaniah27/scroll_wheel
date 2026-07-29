#pragma once
#include <stdint.h>
#include <string.h>
#include "tusb.h"
#include "hid_hires.h"

// ============================================================================
// SALIDA HID — estado de las teclas y cola de envío
// ============================================================================
// Antes cada pulsación construía un informe con una sola tecla de las seis que
// caben, y además se quedaba esperando al endpoint hasta 100 ms (200 en las
// multimedia) dentro del bucle principal, que es el mismo que muestrea la
// rueda a 1 kHz. Dos consecuencias: no había rollover —soltar una tecla
// soltaba todas— y pulsar algo mientras girabas abría un agujero en el
// muestreo del AS5600.
//
// Aquí no se espera a nadie:
//
//   - Las teclas físicas son ESTADO. Cada hueco apunta lo que envió, y el
//     informe se recompone de los huecos ocupados. Si el envío falla o se
//     pierde, en la vuelta siguiente se reintenta solo, porque siempre se
//     compara lo que hay con lo último que salió de verdad.
//   - Los giros de encoder son PULSACIONES PUNTUALES y van a una cola, porque
//     son eventos que no se pueden perder ni fundir con el estado.
//   - pump() suelta como mucho un informe por vuelta, cuando el canal está
//     libre.
namespace HidOut {

// Doce teclas físicas más un hueco para modificadores pegajosos: el Ctrl del
// zoom, que aguanta mientras se sigue girando.
#define HID_SLOTS       13
#define HID_SLOT_STICKY 12
#define HID_QUEUE       32

enum : uint8_t { EV_KEY = 0, EV_CONSUMER = 1 };

struct Event {
    uint8_t  type;
    uint8_t  modifier;
    uint8_t  keycode;
    uint16_t consumer;
};

// --- Estado sostenido -------------------------------------------------------
static uint8_t slot_mod[HID_SLOTS];
static uint8_t slot_key[HID_SLOTS];

static uint8_t held_mods, held_keys[6];   // lo que debería ver el host
static uint8_t sent_mods, sent_keys[6];   // lo último que salió de verdad

// --- Cola de pulsaciones puntuales ------------------------------------------
static Event   queue[HID_QUEUE];
static uint8_t q_head, q_tail;
static uint8_t pending_release;           // 0 nada, 1 teclado, 2 multimedia

static inline uint8_t q_next(uint8_t i) { return (uint8_t)((i + 1) % HID_QUEUE); }
static inline bool q_empty() { return q_head == q_tail; }

// Recompone el informe a partir de los huecos ocupados. Recalcular en vez de ir
// sumando y restando es lo que hace que soltar una tecla no arrastre a las
// demás, y que dos teclas con el mismo modificador no se pisen.
static void recompute() {
    held_mods = 0;
    memset(held_keys, 0, sizeof(held_keys));
    uint8_t n = 0;
    for (int i = 0; i < HID_SLOTS; i++) {
        held_mods |= slot_mod[i];
        uint8_t k = slot_key[i];
        if (!k || n >= 6) continue;
        bool dup = false;
        for (uint8_t j = 0; j < n; j++) if (held_keys[j] == k) { dup = true; break; }
        if (!dup) held_keys[n++] = k;
    }
}

static bool state_differs() {
    return sent_mods != held_mods || memcmp(sent_keys, held_keys, sizeof(held_keys)) != 0;
}

static bool send_state() {
    if (!tud_hid_keyboard_report(REPORT_ID_KEYBOARD, held_mods, held_keys)) return false;
    sent_mods = held_mods;
    memcpy(sent_keys, held_keys, sizeof(sent_keys));
    return true;
}

// --- API --------------------------------------------------------------------

static void init() {
    memset(slot_mod, 0, sizeof(slot_mod));
    memset(slot_key, 0, sizeof(slot_key));
    memset(held_keys, 0, sizeof(held_keys));
    memset(sent_keys, 0, sizeof(sent_keys));
    held_mods = sent_mods = 0;
    q_head = q_tail = pending_release = 0;
}

// Tecla física hundida. `slot` es el índice de la tecla (0-11): se guarda lo
// que se envió, no lo que diga el perfil, para que cambiar de capa con la tecla
// hundida no deje nada pegado.
static void key_down(uint8_t slot, uint8_t mod, uint8_t key) {
    if (slot >= HID_SLOTS) return;
    slot_mod[slot] = mod;
    slot_key[slot] = key;
    recompute();
}

static void key_up(uint8_t slot) {
    if (slot >= HID_SLOTS) return;
    if (!slot_mod[slot] && !slot_key[slot]) return;
    slot_mod[slot] = 0;
    slot_key[slot] = 0;
    recompute();
}

// Modificador que se mantiene solo mientras dure una acción continua.
static void set_sticky_mod(uint8_t mod) {
    if (slot_mod[HID_SLOT_STICKY] == mod) return;
    slot_mod[HID_SLOT_STICKY] = mod;
    recompute();
}

static void push(const Event& e) {
    uint8_t n = q_next(q_head);
    // Cola llena (haría falta un giro absurdo): se pierde un paso, que es
    // preferible a pisar uno pendiente. El estado sostenido no se ve afectado.
    if (n == q_tail) return;
    queue[q_head] = e;
    q_head = n;
}

// Pulsación y suelta de una tecla, para los giros de encoder.
static void tap(uint8_t mod, uint8_t key) {
    if (!mod && !key) return;
    Event e = { EV_KEY, mod, key, 0 };
    push(e);
}

static void consumer_tap(uint16_t code) {
    if (!code) return;
    Event e = { EV_CONSUMER, 0, 0, code };
    push(e);
}

// Modificadores que el host tiene puestos ahora mismo de verdad. El zoom lo
// consulta para no mandar la rueda antes de que el Ctrl haya llegado.
static inline uint8_t sent_modifiers() { return sent_mods; }

// Suelta todo sin mandar nada: para cuando el bus se suspende o se desenchufa
// el cable, que es cuando una tecla se queda pegada en el PC.
static void release_all() {
    memset(slot_mod, 0, sizeof(slot_mod));
    memset(slot_key, 0, sizeof(slot_key));
    recompute();
    memset(sent_keys, 0, sizeof(sent_keys));
    sent_mods = 0;
    q_head = q_tail = pending_release = 0;
}

// Se llama en cada vuelta del bucle. Suelta como mucho un informe: el endpoint
// solo acepta uno por intervalo y así el bucle nunca se queda esperando.
static void pump() {
    if (!tud_hid_ready()) return;

    // La suelta de una pulsación puntual va antes que nada, si no dos pasos
    // seguidos del mismo encoder se fundirían en uno solo para el host.
    if (pending_release == 1) {
        if (send_state()) pending_release = 0;
        return;
    }
    if (pending_release == 2) {
        uint16_t zero = 0;
        if (tud_hid_report(REPORT_ID_CONSUMER, &zero, 2)) pending_release = 0;
        return;
    }

    if (!q_empty()) {
        const Event& e = queue[q_tail];
        if (e.type == EV_KEY) {
            // La pulsación puntual se monta ENCIMA de lo que haya hundido, para
            // no soltar sin querer una tecla que el usuario tiene apretada.
            uint8_t keys[6];
            memcpy(keys, held_keys, sizeof(keys));
            for (int i = 0; i < 6; i++) {
                if (keys[i] == e.keycode) break;
                if (keys[i] == 0) { keys[i] = e.keycode; break; }
            }
            if (tud_hid_keyboard_report(REPORT_ID_KEYBOARD, (uint8_t)(held_mods | e.modifier), keys)) {
                sent_mods = (uint8_t)(held_mods | e.modifier);
                memcpy(sent_keys, keys, sizeof(sent_keys));
                q_tail = q_next(q_tail);
                pending_release = 1;
            }
        } else {
            if (tud_hid_report(REPORT_ID_CONSUMER, &queue[q_tail].consumer, 2)) {
                q_tail = q_next(q_tail);
                pending_release = 2;
            }
        }
        return;
    }

    if (state_differs()) send_state();
}

} // namespace HidOut
