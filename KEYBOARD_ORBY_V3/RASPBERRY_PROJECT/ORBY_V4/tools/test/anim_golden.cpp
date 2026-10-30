// Genera OrbyGUI/test/fixtures/oled-anim-golden.txt: el CRC32 de las diez
// pantallas de cada transición en 41 instantes, a dos velocidades, para
// unas imágenes de partida y de llegada pseudoaleatorias. PICK depende además
// del origen: el bucle general usa el de por defecto (4) y al final van las
// líneas `pick <origen> ...` de los otros dos extremos (0 y 9).
//
// La app repite en el dashboard las animaciones del teclado con una copia en
// JS de oled_anim.h (OrbyGUI/src/oled-anim.mjs). test/oled-anim.test.mjs
// calcula lo mismo en JS y lo compara con este fichero: si alguien toca una
// constante en un lado y no en el otro, salta. Regenerar tras cambiar
// oled_anim.h a propósito:
//
//   g++ -O1 -I../../include anim_golden.cpp -o anim_golden.exe
//   anim_golden.exe > ../../OrbyGUI/test/fixtures/oled-anim-golden.txt
#include <stdio.h>
#include "oled_anim.h"
using namespace OledAnim;

static uint8_t from[N][FB], to[N][FB], out[FB];
static uint32_t seed = 12345;
static uint8_t rnd() { seed = seed * 1103515245u + 12345u; return (uint8_t)(seed >> 16); }

static uint32_t crc32(uint32_t c, const uint8_t* p, int n) {
    c = ~c;
    while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1))); }
    return ~c;
}

int main() {
    init();
    for (int i = 0; i < N; i++) for (int b = 0; b < FB; b++) { from[i][b] = rnd(); to[i][b] = rnd(); }
    const float rates[2] = {1.0f, 1.4f};
    set_origin(4);
    for (int k = 1; k <= PICK; k++) for (int r = 0; r < 2; r++) {
        set_rate(group_of((Kind)k), rates[r]);
        for (int ti = 0; ti <= 40; ti++) {
            uint32_t c = 0;
            for (int i = 0; i < N; i++) { render((Kind)k, ti * 0.0125f, i, from, to, out); c = crc32(c, out, FB); }
            printf("%d %d %d %08x\n", k, r, ti, (unsigned)c);
        }
    }
    const int origenes[2] = {0, 9};
    for (int o = 0; o < 2; o++) for (int r = 0; r < 2; r++) {
        set_origin(origenes[o]);
        set_rate(group_of(PICK), rates[r]);
        for (int ti = 0; ti <= 40; ti++) {
            uint32_t c = 0;
            for (int i = 0; i < N; i++) { render(PICK, ti * 0.0125f, i, from, to, out); c = crc32(c, out, FB); }
            printf("pick %d %d %d %08x\n", origenes[o], r, ti, (unsigned)c);
        }
    }
    Press p; uint8_t pr[FB];
    press_step(p, 1.0f, 0.03f);
    render_press(to[0], p, pr);
    printf("press %08x\n", (unsigned)crc32(0, pr, FB));
}
