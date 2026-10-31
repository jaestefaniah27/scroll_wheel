#pragma once
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware_oled.h"

// ============================================================================
// INTRO DE ARRANQUE — reproductor
// ============================================================================
// Aquí no se calcula nada: las intros se generan con tools/anim/intro_orvy.py y
// llegan ya horneadas a píxeles en include/orby_intro_data.h, varias a elegir
// (intro_clips). El firmware solo descomprime y vuelca. Lo que se ve en la vista
// previa es exactamente lo que sale en las pantallas, sin dos implementaciones
// que mantener en paralelo.
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

// Reproduce el flujo entero. Si llega al final devuelve true y deja las diez
// pantallas en negro (así acaba el flujo): los iconos los abre después Core 1
// con la transición de SUPER, que depende del perfil cargado y no se puede
// hornear. Si no la hay o no se reproduce entera, pinta el contenido normal
// sin animar y devuelve false.
//
// variant     : cuál de intro_clips; una que no exista es la 0.
// should_skip : no la reproduce (tras un reinicio por watchdog). Ya no la corta
//               ninguna tecla: se mira solo al empezar. Puede ser NULL.
// paint_final : pinta una pantalla con su contenido normal (refresh_single_screen)
static bool play(HardwareOled& oleds, uint8_t variant, bool (*should_skip)()) {
    if (should_skip && should_skip()) return false;

    const IntroClip& clip = intro_clips[variant < INTRO_COUNT ? variant : 0];
    const uint8_t* p   = clip.data;
    const uint8_t* end = clip.data + clip.size;
    uint8_t fb[360];

    // Ritmo con espera activa en vez de sleep_until: este core no tiene nada
    // más que hacer durante la intro y así no se toca el pool de alarmas, que
    // lo creó Core 0.
    const uint32_t frame_us = 1000000u / INTRO_FPS;
    absolute_time_t next = make_timeout_time_us(frame_us);

    for (uint16_t f = 0; f < clip.frames; f++) {
        if (p + 2 > end) return false;

        uint16_t mask = (uint16_t)(p[0] | (p[1] << 8));
        p += 2;

        for (int s = 0; s < 10; s++) {
            if (!(mask & (1u << s))) continue;
            if (p + 2 > end) return false;
            uint16_t len = (uint16_t)(p[0] | (p[1] << 8));
            p += 2;
            if (p + len > end) return false;
            if (!rle_decode(p, len, fb)) return false;
            oleds.paint_screen((uint8_t)(s + 1), fb);
            p += len;
        }

        busy_wait_until(next);
        next = delayed_by_us(next, frame_us);
    }
    return true;
}

static bool run(HardwareOled& oleds, uint8_t variant,
                bool (*should_skip)(),
                void (*paint_final)(HardwareOled&, uint8_t)) {
    if (play(oleds, variant, should_skip)) return true;
    // Datos rotos a medias o intro saltada: nada de dejar las pantallas a
    // medio dibujar.
    for (uint8_t i = 1; i <= 10; i++) paint_final(oleds, i);
    return false;
}

#else // Sin animación horneada

#define INTRO_COUNT 1

static bool run(HardwareOled& oleds, uint8_t variant,
                bool (*should_skip)(),
                void (*paint_final)(HardwareOled&, uint8_t)) {
    (void)variant;
    (void)should_skip;
    for (uint8_t i = 1; i <= 10; i++) paint_final(oleds, i);
    return false;
}

#endif

} // namespace OledIntro
