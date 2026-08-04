// ============================================================================
// La huella del teclado y la de la app tienen que coincidir
// ============================================================================
// El firmware resume su configuración con un CRC32 (include/config_hash.h) y la
// app calcula el mismo número sobre su copia local (OrbyGUI/src/hash.js). Si
// coinciden, al conectar no se descarga nada; si no, se descarga solo lo que
// haya cambiado.
//
// Dos implementaciones del mismo formato en dos lenguajes distintos se desvían
// solas en cuanto alguien añade un campo en un lado y se olvida del otro, y el
// síntoma es silencioso: la app deja de fiarse de su copia y vuelve a
// descargarlo todo, que es exactamente lo que se quería evitar.
//
// Este programa fabrica una configuración de mentira y escribe sus huellas.
// test_config_hash.mjs fabrica la MISMA con el generador de números de abajo,
// la pasa por hash.js y compara.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "config_hash.h"

// --- Estructuras equivalentes a las del firmware ---------------------------
// No se incluye main.cpp (arrastraría medio SDK de la Pico): basta con que los
// campos se llamen igual, que es lo que pide la plantilla crc32_profile_of.
struct KeyAction    { uint8_t modifier, keycode; };
struct RotaryAction { uint8_t type, modifier, keycode; };

struct Page {
    char         oled_labels[CFGHASH_LABELS][8];
    KeyAction    key_mappings[CFGHASH_KEYS];
    RotaryAction rotary[CFGHASH_ROTARY];
    uint8_t      scroll_detents[2];
    uint8_t      scroll_invert[2];
};

#define TEST_PAGES    4
#define TEST_PROFILES 3

struct TestProfile {
    char     name[8];
    uint8_t  page_count;
    Page     pages[TEST_PAGES];
    uint32_t mask[TEST_PAGES];
    uint8_t  icons[TEST_PAGES][CFGHASH_SLOTS][CFGHASH_FB_SIZE];
};

static TestProfile profiles[TEST_PROFILES];

// --- Generador reproducible -------------------------------------------------
// Un congruencial lineal de los de toda la vida: lo que importa es que JavaScript
// pueda dar exactamente la misma secuencia (ver rnd() en el .mjs).
static uint32_t rng_state = 12345;

static uint8_t rnd() {
    rng_state = (uint32_t)(rng_state * 1103515245u + 12345u);
    return (uint8_t)(rng_state >> 24);
}

// Etiquetas escogidas para tocar los casos raros: la vacía, una de un carácter y
// una de OCHO, que llena el char[8] sin dejar sitio al terminador.
static const char* const LABELS[] = {
    "", "A", "COPY", "abcdefgh", "z", "~!@#", "0", "        ",
};
#define LABEL_COUNT ((int)(sizeof(LABELS) / sizeof(LABELS[0])))

static const char* const NAMES[TEST_PROFILES] = { "", "OFIM", "12345678" };

// Copia un texto en un char[8] SIN limpiar lo que hubiera antes: así queda
// basura detrás del terminador, igual que en el teclado cuando se sustituye una
// etiqueta larga por una corta. La huella tiene que ignorarla.
static void put_text8(char dst[8], const char* src) {
    memset(dst, 0xAA, 8);
    size_t n = strlen(src);
    if (n > 8) n = 8;
    memcpy(dst, src, n);
    if (n < 8) dst[n] = 0;
}

static void build() {
    for (int p = 0; p < TEST_PROFILES; p++) {
        TestProfile& tp = profiles[p];
        put_text8(tp.name, NAMES[p]);
        tp.page_count = (uint8_t)(p + 1);

        for (int pg = 0; pg < tp.page_count; pg++) {
            Page& page = tp.pages[pg];

            for (int s = 0; s < CFGHASH_LABELS; s++) {
                put_text8(page.oled_labels[s], LABELS[(s + pg + p) % LABEL_COUNT]);
            }
            for (int s = 0; s < CFGHASH_KEYS; s++) {
                page.key_mappings[s].modifier = rnd();
                page.key_mappings[s].keycode  = rnd();
            }
            for (int s = 0; s < CFGHASH_ROTARY; s++) {
                page.rotary[s].type     = (uint8_t)(rnd() % 6);
                page.rotary[s].modifier = rnd();
                page.rotary[s].keycode  = rnd();
            }
            for (int l = 0; l < 2; l++) {
                page.scroll_detents[l] = (uint8_t)(3 + rnd() % 238);
                page.scroll_invert[l]  = (uint8_t)(rnd() & 1);
            }

            uint32_t mask = (uint32_t)rnd() | ((uint32_t)rnd() << 8) | ((uint32_t)rnd() << 16);
            tp.mask[pg] = mask & 0xFFFFF;

            for (int s = 0; s < CFGHASH_SLOTS; s++) {
                if (!(tp.mask[pg] & (1u << s))) continue;
                for (int i = 0; i < CFGHASH_FB_SIZE; i++) tp.icons[pg][s][i] = rnd();
            }
        }
    }
}

int main() {
    build();

    uint32_t per[TEST_PROFILES];
    for (int p = 0; p < TEST_PROFILES; p++) {
        const TestProfile& tp = profiles[p];
        per[p] = crc32_profile_of(
            tp.name, tp.page_count, tp.pages,
            [&tp](uint8_t pg) { return tp.mask[pg]; },
            [&tp](uint8_t pg, uint8_t slot) { return tp.icons[pg][slot]; },
            []() {});
        printf("P%d %08x\n", p, (unsigned)per[p]);
    }

    // El 5 es el tiempo de reposo por defecto del teclado.
    printf("ALL %08x\n", (unsigned)crc32_snapshot_of(TEST_PROFILES, 5, per));

    // Vector conocido del CRC-32 de siempre: si esto falla, la tabla está mal y
    // no hay nada más que mirar.
    const char* check = "123456789";
    uint32_t crc = crc32_feed(CFGHASH_INIT, (const uint8_t*)check, 9) ^ 0xFFFFFFFFu;
    if (crc != 0xCBF43926u) {
        printf("FALLO: CRC32(\"123456789\") = %08x, se esperaba cbf43926\n", (unsigned)crc);
        return 1;
    }
    return 0;
}
