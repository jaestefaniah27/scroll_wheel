#pragma once
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware_oled.h"

// ============================================================================
// INTRO DE ARRANQUE — reproductor
// ============================================================================
// Aquí no se calcula nada: la animación se diseña en tools/anim/index.html y se
// exporta ya horneada a píxeles en include/orby_intro_data.h. El firmware solo
// descomprime y vuelca. Lo que se ve en el editor es exactamente lo que sale en
// las pantallas, sin dos implementaciones que mantener en paralelo.
//
// Si no existe el archivo de datos, la intro no hace nada más que pintar el
// contenido normal, así que el firmware compila igual sin él.
//
// Formato del flujo, por fotograma:
//   uint16  máscara de pantallas que cambian (bit 0 = OLED 1)
//   por cada pantalla marcada, en orden:
//     uint16  longitud del bloque comprimido
//     bytes   pares (repeticiones, valor) hasta completar los 360 del
//             framebuffer
// Las pantallas que no cambian no ocupan nada y encima se ahorran el SPI.

#if defined(__has_include)
#  if __has_include("orby_intro_data.h")
#    include "orby_intro_data.h"
#    define ORBY_HAS_INTRO_DATA 1
#  endif
#endif

namespace OledIntro {

#ifdef ORBY_HAS_INTRO_DATA

// Descomprime un bloque en el framebuffer. Devuelve false si los datos están
// mal formados, para no seguir leyendo a ciegas.
static bool rle_decode(const uint8_t* src, uint16_t len, uint8_t* dst) {
    uint32_t o = 0;
    for (uint16_t i = 0; i + 1 < len; i += 2) {
        uint8_t n = src[i], v = src[i + 1];
        if (o + n > 360) return false;
        memset(dst + o, v, n);
        o += n;
    }
    return o == 360;
}

// should_skip : cualquier tecla corta la intro (puede ser NULL)
// paint_final : pinta una pantalla con su contenido normal (refresh_single_screen)
static void run(HardwareOled& oleds,
                bool (*should_skip)(),
                void (*paint_final)(HardwareOled&, uint8_t)) {

    const uint8_t* p   = intro_stream;
    const uint8_t* end = intro_stream + sizeof(intro_stream);
    uint8_t fb[360];

    // Ritmo con espera activa en vez de sleep_until: este core no tiene nada
    // más que hacer durante la intro y así no se toca el pool de alarmas, que
    // lo creó Core 0.
    const uint32_t frame_us = 1000000u / INTRO_FPS;
    absolute_time_t next = make_timeout_time_us(frame_us);

    for (uint16_t f = 0; f < INTRO_FRAMES; f++) {
        if (should_skip && should_skip()) break;
        if (p + 2 > end) break;

        uint16_t mask = (uint16_t)(p[0] | (p[1] << 8));
        p += 2;

        for (int s = 0; s < 10; s++) {
            if (!(mask & (1u << s))) continue;
            if (p + 2 > end) return;
            uint16_t len = (uint16_t)(p[0] | (p[1] << 8));
            p += 2;
            if (p + len > end) return;
            if (rle_decode(p, len, fb)) oleds.paint_screen((uint8_t)(s + 1), fb);
            p += len;
        }

        busy_wait_until(next);
        next = delayed_by_us(next, frame_us);
    }

    // Termine como termine, las diez acaban con su contenido de verdad.
    for (uint8_t i = 1; i <= 10; i++) paint_final(oleds, i);
}

#else // Sin animación horneada

static void run(HardwareOled& oleds,
                bool (*should_skip)(),
                void (*paint_final)(HardwareOled&, uint8_t)) {
    (void)should_skip;
    for (uint8_t i = 1; i <= 10; i++) paint_final(oleds, i);
}

#endif

} // namespace OledIntro
