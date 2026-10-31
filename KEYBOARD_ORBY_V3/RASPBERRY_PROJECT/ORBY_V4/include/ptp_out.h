#pragma once
#include <stdint.h>
#include <math.h>
#include "tusb.h"
#include "pico/time.h"
#include "hid_ptp.h"
#include "hid_hires.h"

// ============================================================================
// ZOOM COMO PELLIZCO — informes del touchpad de precisión
// ============================================================================
// El zoom de una rueda o mando sale como dos dedos simétricos que se abren o se
// cierran sobre el centro del pad. Windows lo recibe como el pellizco de un
// trackpad: sin Ctrl pulsado, así que no estorba a nada del teclado.
//
// - En escala logarítmica. El zoom que aplica el host es el cociente entre la
//   separación nueva y la vieja: sumando en ln(separación), cada clic
//   multiplica por lo mismo, esté donde esté el gesto.
//
// - En alta resolución. La entrada va en las mismas unidades que el scroll
//   (HID_HIRES_MULTIPLIER por clic): la rueda magnética manda su giro fino tal
//   cual y un encoder manda clics enteros.
//
// - Con un suavizado ajustable (ver Cfg), para que un clic de encoder no sea un
//   salto. Los dedos nunca se quedan sin informar mientras están apoyados: si
//   el dispositivo se calla, Windows los da por levantados y lo toma por un
//   toque, que es un clic.
//
// Si el giro pide más de lo que da de sí el pad, los dedos se levantan en el
// extremo, se vuelven a apoyar en el otro y siguen.
namespace PtpOut {

static constexpr int32_t CX = PTP_X_MAX / 2;
static constexpr int32_t CY = PTP_Y_MAX / 2;

// Mitad de la separación entre dedos, en unidades del pad (1 unidad ≈ 0,025
// mm). De 2 a 8,7 cm entre dedos, como un pellizco de verdad: con los dedos a
// menos de 1 cm o a milímetros del borde, Windows los trata distinto (contactos
// fundidos, rechazo de bordes) y el zoom arrancaba o paraba a destiempo. Caben
// ln(1750/400) / ln(1,12) ≈ 13 clics por gesto.
static constexpr float H_MIN = 400.0f;
static constexpr float H_MAX = 1750.0f;

static constexpr uint32_t FRAME_US = 4000;   // 250 cuadros/s
// Distancia (en ln) a la que se da por llegado a un extremo.
static constexpr float EDGE_EPS = 0.01f;

// Parámetros del gesto. Se pueden cambiar en caliente con ZOOMCFG (solo para
// ajustarlos midiendo; la app no lo usa).
struct Cfg {
    float    zoom_per_detent = 1.12f; // la separación se multiplica por esto
    uint32_t settle_first_us = 8000;  // quietos al apoyar
    uint32_t settle_again_us = 8000;  // quietos al repetir el gesto
    // Sin movimiento: levantar. Largo a propósito: cada gesto nuevo obliga a
    // Windows a reconocer otra vez el pellizco, y eso a veces tarda y a veces
    // no. Con los dedos apoyados, las pausas normales entre clics no lo rompen.
    uint32_t idle_us         = 1000000;
    // 0: exponencial de primer orden con constante tau_us (0 = sin suavizado).
    //    Arranca a toda velocidad y frena solo: Windows ve recorrido enseguida
    //    (reconoce antes el pellizco) y no queda cola larga al soltar.
    // 1: muelle críticamente amortiguado de pulsación omega. Más bonito sobre
    //    el papel, pero arranca desde velocidad cero: el pellizco tardaba en
    //    reconocerse y luego iba ~65 ms por detrás de la rueda.
    uint8_t  model           = 0;
    uint32_t tau_us          = 25000;
    float    omega           = 30.0f;
};
static Cfg cfg;

enum State : uint8_t { IDLE, DOWN, ACTIVE, LIFT };

static State    state      = IDLE;
static float    x          = 0;     // ln(separación) actual
static float    v          = 0;     // su velocidad (solo el muelle)
static float    goal       = 0;     // ln(separación) pedida, sin recortar
static uint32_t last_frame = 0;
static uint32_t last_input = 0;
static uint32_t down_since = 0;
static uint32_t settle_us  = 0;

static const float L_MIN = logf(H_MIN);
static const float L_MAX = logf(H_MAX);

static inline float l_step() {
    return logf(cfg.zoom_per_detent) / HID_HIRES_MULTIPLIER;
}

static inline float clampf(float a, float lo, float hi) {
    return a < lo ? lo : (a > hi ? hi : a);
}

static inline bool send(const orby_ptp_report_t& r) {
    return tud_hid_n_report(PTP_ITF_INSTANCE, PTP_REPORTID_TOUCHPAD, &r, sizeof(r));
}

static orby_ptp_finger_t finger(uint8_t id, bool tip, int32_t px) {
    orby_ptp_finger_t f;
    // Al levantar, confianza 0, como una palma: Windows descarta el final del
    // gesto en vez de poder tomarlo por un toque.
    f.flags = (uint8_t)((tip ? 0x03 : 0) | (id << 2));
    f.x     = (uint16_t)px;
    f.y     = (uint16_t)CY;
    return f;
}

// Un cuadro entero en un solo informe: el host nunca ve un dedo suelto.
static bool send_frame(bool tip, uint32_t now_us) {
    int32_t half = (int32_t)lroundf(expf(x));
    orby_ptp_report_t r;
    r.finger[0]     = finger(0, tip, CX - half);
    r.finger[1]     = finger(1, tip, CX + half);
    r.scan_time     = (uint16_t)(now_us / 100);
    r.contact_count = 2;
    r.buttons       = 0;
    return send(r);
}

static inline void touch_down(float at, uint32_t now, uint32_t settle) {
    x = at;
    down_since = now;
    settle_us = settle;
    state = DOWN;
}

// Movimiento de zoom en unidades de alta resolución (HID_HIRES_MULTIPLIER por
// clic). Positivo acerca (separa los dedos), igual que Ctrl + rueda arriba.
static inline void add_units(int32_t units) {
    if (units == 0) return;
    uint32_t now = time_us_32();
    last_input = now;
    if (state == IDLE) {
        // Se apoya en el extremo que deja más recorrido en el sentido del giro.
        float start = units > 0 ? L_MIN : L_MAX;
        goal = start;
        v = 0;
        touch_down(start, now, cfg.settle_first_us);
    }
    goal += units * l_step();
}

// Un paso de encoder (o de lo que llegue en clics enteros).
static inline void add(int32_t detents) {
    add_units(detents * HID_HIRES_MULTIPLIER);
}

static inline void follow(float dt) {
    float target = clampf(goal, L_MIN, L_MAX);
    if (cfg.model == 1) {
        // Euler semimplícito: estable de sobra con ω·dt pequeño.
        float a = cfg.omega * cfg.omega * (target - x) - 2.0f * cfg.omega * v;
        v += a * dt;
        x += v * dt;
    } else if (cfg.tau_us == 0) {
        x = target;
    } else {
        x += (target - x) * (1.0f - expf(-dt * 1e6f / (float)cfg.tau_us));
    }
    x = clampf(x, L_MIN, L_MAX);
}

// Llamar en cada vuelta del bucle principal.
static inline void tick() {
    // El host ha cerrado el modo (suspensión, reinicio de la interfaz): lo que
    // hubiera a medias ya no tiene a quién llegar.
    if (!g_ptp_active) { state = IDLE; return; }
    if (state == IDLE) return;
    if (!tud_hid_n_ready(PTP_ITF_INSTANCE)) return;

    uint32_t now = time_us_32();
    uint32_t elapsed = now - last_frame;
    if (elapsed < FRAME_US) return;
    // Tras un rato sin cuadros (el primero de un gesto) no se integra de golpe.
    float dt = (elapsed > 3 * FRAME_US ? FRAME_US : elapsed) * 1e-6f;

    switch (state) {
    case DOWN:
        // Quietos hasta que Windows haya visto los dedos apoyados; mientras,
        // el objetivo puede seguir creciendo.
        if (send_frame(true, now)) {
            last_frame = now;
            if ((uint32_t)(now - down_since) >= settle_us) state = ACTIVE;
        }
        break;

    case ACTIVE: {
        follow(dt);
        float target = clampf(goal, L_MIN, L_MAX);
        bool overflow = goal != target;
        bool at_edge  = fabsf(x - target) < EDGE_EPS;
        bool resting  = fabsf(x - target) < 1e-4f && fabsf(v) < 1e-3f;

        if (overflow && at_edge) {
            state = LIFT;
        } else if (resting && (uint32_t)(now - last_input) >= cfg.idle_us) {
            state = LIFT;
        }
        // También quietos hay que seguir informando: si el dispositivo se
        // calla, Windows da los dedos por levantados a los ~150 ms y un clic
        // suelto de rueda (apoyar, mover, «soltar») le parece un toque de dos
        // dedos: un clic. Medido con ZOOMTEST.
        if (state == ACTIVE && send_frame(true, now)) last_frame = now;
        break;
    }

    case LIFT:
        if (send_frame(false, now)) {
            last_frame = now;
            float target = clampf(goal, L_MIN, L_MAX);
            if (goal != target) {
                // Se sigue girando: volver a apoyar en el extremo contrario con
                // lo que faltaba, medido desde donde quedaron los dedos (se
                // levantan a EDGE_EPS del extremo y ese resto se perdía).
                float excess = goal - x;
                float start = excess > 0 ? L_MIN : L_MAX;
                goal = start + excess;
                touch_down(start, now, cfg.settle_again_us);
            } else {
                state = IDLE;
            }
        }
        break;

    default:
        break;
    }
}

} // namespace PtpOut
