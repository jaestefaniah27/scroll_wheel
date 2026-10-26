// Comprueba que include/oled_ui.h dibuja cada pantalla igual, píxel a píxel,
// que el modelo en Python (tools/oled_ui/layout.py). Los casos los escribe
// tools/oled_ui/build.py en ui_fixture.h.
//
//   g++ -std=c++17 -O2 -I../../include test_ui.cpp -o test_ui && ./test_ui
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oled_ui.h"
#include "ui_fixture.h"

using namespace OledUi;

static int fallos = 0;

static void comparar(const char* what, const uint8_t* got, const uint8_t* want) {
    if (memcmp(got, want, FB) == 0) return;
    fallos++;
    printf("FALLO: %s\n", what);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const bool g = (got[(y >> 3) * W + x] >> (y & 7)) & 1;
            const bool w = (want[(y >> 3) * W + x] >> (y & 7)) & 1;
            putchar(g == w ? (g ? '#' : '.') : (g ? '+' : '-'));
        }
        putchar('\n');
    }
}

int main() {
    uint8_t fb[FB];
    for (const UiCase& c : ui_cases) {
        const char* w = c.what;
        if (!strncmp(w, "icon:", 5)) {
            const char* key = w + 5;
            const char* slash = strchr(key, '/');
            int e;
            if (slash) {
                char prof[8] = {0};
                memcpy(prof, key, (size_t)(slash - key));
                e = lookup(prof, slash + 1);
            } else {
                e = lookup("", key);
            }
            if (e < 0) { printf("FALLO: %s no está en el catálogo\n", w); fallos++; continue; }
            screen_icon(fb, entries[e].icon, entries[e].caption);
        } else if (!strncmp(w, "text:", 5)) {
            screen_text(fb, w + 5);
        } else if (!strncmp(w, "toggle:", 7)) {
            screen_toggle(fb, w[7] == '1', w + 9);
        } else if (!strncmp(w, "dots:", 5)) {
            screen_dots(fb, w[5] - '0', w[7] - '0', w + 9);
        } else if (!strncmp(w, "pill:", 5)) {
            screen_system(fb, SYS_MENU_PROFILES);
            invert_pill(fb);
        } else {
            printf("FALLO: caso desconocido %s\n", w); fallos++; continue;
        }
        comparar(w, fb, c.fb);
    }

    // La búsqueda: el perfil gana a la genérica, y sin perfil sale la genérica.
    const int corte = lookup("PREM", "CUT"), tijeras = lookup("OFIM", "CUT");
    if (corte < 0 || tijeras < 0 || entries[corte].icon == entries[tijeras].icon) {
        printf("FALLO: PREM/CUT no gana a CUT\n"); fallos++;
    }
    if (lookup("OFIM", "undo") != lookup("", "UNDO")) { printf("FALLO: mayúsculas\n"); fallos++; }
    if (lookup("OFIM", "XYZW") != -1) { printf("FALLO: XYZW no debería existir\n"); fallos++; }
    if (lookup_profile("ALTM") < 0) { printf("FALLO: sin icono para ALTM\n"); fallos++; }

    // Todo lo del catálogo cabe en la pantalla.
    for (const auto& e : entries) {
        if (text_width(e.caption, false) > W - 2) { printf("FALLO: «%s» no cabe\n", e.caption); fallos++; }
    }
    for (const auto& e : system_entries) {
        if (text_width(e.caption, false) > W - 2) { printf("FALLO: «%s» no cabe\n", e.caption); fallos++; }
    }

    if (fallos) { printf("%d fallos\n", fallos); return 1; }
    printf("OK: %zu pantallas iguales a layout.py\n", sizeof(ui_cases) / sizeof(ui_cases[0]));
    return 0;
}
