// Comprueba las transiciones de include/oled_anim.h en el PC y, si se le pasa
// una carpeta, vuelca cada una fotograma a fotograma para verla.
//
//   g++ -std=c++17 -O2 -I../../include test_anim.cpp -o test_anim
//   ./test_anim                 # solo las comprobaciones
//   ./test_anim frames/         # + frames/<transición>_NNN.pgm a 60 fps
//
// Los .pgm llevan las diez pantallas colocadas como en el teclado, así que un
//   ffmpeg -framerate 60 -i frames/page_%03d.pgm page.gif
// enseña exactamente lo que se va a ver en las teclas.
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "oled_anim.h"
#include "oled_ui.h"

using namespace OledAnim;

static int fallos = 0;
#define CHECK(c, ...) do { if (!(c)) { fallos++; printf("FALLO: " __VA_ARGS__); printf("\n"); } } while (0)

static uint8_t A[N][FB], B[N][FB], C[N][FB], M[N][FB], S[N][FB], P[N][FB];

// Lienzo del .pgm: pantallas a escala 3 con huecos proporcionados a los reales
// (paso de 19 mm sobre 8,6 x 4,8 mm de área activa, comprimidos a la mitad).
static const int ESC = 3, HX = 43, HY = 59;
static const int PW = 3 * W * ESC + 4 * HX * ESC / 2, PH = 4 * H * ESC + 5 * HY * ESC / 2;

static void volcar(const char* dir, const char* nombre, int f, uint8_t pant[N][FB]) {
    static unsigned char img[PH][PW];
    memset(img, 18, sizeof(img));
    for (int i = 0; i < N; i++) {
        const int ox = HX * ESC / 2 + COL[i] * (W * ESC + HX * ESC / 2);
        const int oy = HY * ESC / 2 + ROW[i] * (H * ESC + HY * ESC / 2);
        for (int y = 0; y < H * ESC; y++)
            for (int x = 0; x < W * ESC; x++)
                img[oy + y][ox + x] = get_px(pant[i], x / ESC, y / ESC) ? 235 : 0;
    }
    char ruta[512];
    snprintf(ruta, sizeof(ruta), "%s/%s_%03d.pgm", dir, nombre, f);
    FILE* fp = fopen(ruta, "wb");
    if (!fp) { perror(ruta); exit(1); }
    fprintf(fp, "P5 %d %d 255\n", PW, PH);
    fwrite(img, 1, sizeof(img), fp);
    fclose(fp);
}

static void probar(const char* nombre, Kind k, uint8_t from[N][FB], uint8_t to[N][FB], const char* dir) {
    uint8_t out[N][FB];
    const float d = duration(k);
    printf("%-11s %4.0f ms\n", nombre, d * 1000.0f);
    CHECK(d > 0.05f && d < 1.2f, "%s dura %.3f s", nombre, d);

    // Empieza exactamente donde estaba: si el primer fotograma no es lo que ya
    // había en las pantallas, la transición arranca con un salto.
    for (int i = 0; i < N; i++) {
        render(k, 0.0f, i, from, to, out[i]);
        CHECK(memcmp(out[i], from[i], FB) == 0, "%s: el fotograma 0 de la pantalla %d no es el de partida", nombre, i + 1);
    }
    // Y acaba exactamente en el destino.
    for (int i = 0; i < N; i++) {
        render(k, d, i, from, to, out[i]);
        CHECK(memcmp(out[i], to[i], FB) == 0, "%s: el último fotograma de la pantalla %d no es el destino", nombre, i + 1);
    }

    if (!dir) return;
    int f = 0;
    for (int r = 0; r < 12; r++, f++) volcar(dir, nombre, f, from);          // reposo antes
    for (float t = 0.0f; t < d + 0.0001f; t += 1.0f / 60.0f, f++) {
        for (int i = 0; i < N; i++) render(k, t, i, from, to, out[i]);
        volcar(dir, nombre, f, out);
    }
    for (int r = 0; r < 24; r++, f++) volcar(dir, nombre, f, to);            // y después
}

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : nullptr;
    init();

    // Las pantallas de verdad, compuestas igual que en el firmware.
    // A: OFIM, B: su segunda página, C: Photoshop, M: menú principal con el
    // cursor en Perfiles, S: el submenú de reposo, Ms: el menú con el cursor
    // una opción más abajo.
    const char* ofim[N] = {"COPY", "PAST", "CUT", "UNDO", "REDO", "ALL", "SAVE", "FIND", "CLSE", "SNAP"};
    const char* pag2[N] = {"F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", ""};
    const char* phts[N] = {"BRSH", "ERAS", "PENC", "UNDO", "REDO", "LASS", "CROP", "MOVE", "NEWL", "DUPL"};
    const uint8_t menu[6] = {OledUiData::SYS_MENU_PROFILES, OledUiData::SYS_MENU_SLEEP, OledUiData::SYS_MENU_WHEEL, OledUiData::SYS_MENU_RESET, OledUiData::SYS_MENU_INFO, OledUiData::SYS_MENU_EXIT};
    const uint8_t repo[5] = {OledUiData::SYS_SLEEP_1, OledUiData::SYS_SLEEP_5, OledUiData::SYS_SLEEP_10, OledUiData::SYS_SLEEP_OFF, OledUiData::SYS_BACK};
    // P: el menú de perfiles de partida, un perfil por pantalla.
    const char* perf[N] = {"OFIM", "PHTS", "KICD", "ALTM", "VSCD", "WORD", "XLSX", "BLND", "CHRM", "DISC"};
    uint8_t Ms[N][FB];
    for (int i = 0; i < N; i++) {
        OledUi::screen_key(P[i], "PERF", perf[i]);
        OledUi::screen_key(A[i], "OFIM", ofim[i]);
        OledUi::screen_key(B[i], "OFIM", pag2[i]);
        OledUi::screen_key(C[i], "PHTS", phts[i]);
        memset(M[i], 0, FB); memset(Ms[i], 0, FB); memset(S[i], 0, FB);
        if (i < 6) { OledUi::screen_system(M[i], menu[i]); OledUi::screen_system(Ms[i], menu[i]); }
        if (i < 5) OledUi::screen_system(S[i], repo[i]);
    }
    OledUi::screen_dots(B[9], 1, 2, "Página 2 de 2");
    OledUi::invert_pill(M[0]);
    OledUi::invert_pill(Ms[1]);
    OledUi::invert_pill(S[1]);

    probar("page",       PAGE_NEXT,  A, B, dir);
    probar("page_prev",  PAGE_PREV,  B, A, dir);
    probar("menu_open",  MENU_OPEN,  A, M, dir);
    probar("menu_close", MENU_CLOSE, M, A, dir);
    probar("push",       PUSH,       M, S, dir);
    probar("pop",        POP,        S, M, dir);
    probar("profile",    PROFILE,    A, C, dir);
    probar("crossfade",  CROSSFADE,  M, Ms, dir);
    // Elegir perfil: del menú de perfiles a la pantalla normal del elegido. El
    // origen cambia por dónde se cuela todo, así que se prueban una esquina, el
    // centro y la pantalla suelta de la última fila.
    const int origenes[3] = {0, 4, 9};
    for (int o = 0; o < 3; o++) {
        char nombre[16];
        snprintf(nombre, sizeof(nombre), "pick%d", origenes[o]);
        set_origin(origenes[o]);
        probar(nombre, PICK, P, C, dir);
    }
    set_origin(4);

    // Cuando se da por terminada, al muelle le queda menos de medio píxel: el
    // salto final a `to` no se ve. (Comparar píxeles no sirve: un desplazamiento
    // de un píxel ya cambia todos los bordes.)
    CHECK(fabsf(spring_left(NUDGE, duration(PAGE_NEXT))) * CANVAS <= 0.5f, "el carrusel corta con más de medio píxel por recorrer");
    CHECK(fabsf(spring_left(PULL, duration(PUSH))) * W <= 0.5f, "el submenú corta con más de medio píxel por recorrer");
    // Los muelles de Apple que van sin rebote no rebotan aquí tampoco.
    for (float t = 0.0f; t < 1.0f; t += 0.001f) {
        CHECK(spring(FOLDER, t) <= 1.0001f, "el muelle de carpeta se pasa en t=%.3f", t);
        CHECK(spring(NUDGE, t)  <= 1.0001f, "el muelle de nudge se pasa en t=%.3f", t);
        if (fallos) break;
    }

    // La onda sale del centro: la 5 primero, la 10 la última.
    CHECK(hop_delay(4) == 0.0f, "la pantalla 5 no es la primera de la onda");
    CHECK(hop_delay(9) > hop_delay(0), "la pantalla 10 no va por detrás de las esquinas");

    // Pulsación: se hunde, vuelve, y si se suelta a medias da la vuelta sin saltos.
    {
        Press p;
        int pasos = 0;
        while (press_step(p, 1.0f, 1.0f / 60.0f) && pasos < 120) pasos++;
        CHECK(p.x == 1.0f, "la pulsación no llega al fondo");
        CHECK(pasos < 40, "la pulsación tarda %d fotogramas en hundirse", pasos);
        for (int i = 0; i < 3; i++) press_step(p, 0.0f, 1.0f / 60.0f);
        const float medio = p.x;
        press_step(p, 1.0f, 1.0f / 60.0f);
        CHECK(fabsf(p.x - medio) < 0.2f, "al volver a pulsar a medias el muelle salta");
        pasos = 0;
        while (press_step(p, 0.0f, 1.0f / 60.0f) && pasos < 120) pasos++;
        CHECK(p.x == 0.0f, "la pulsación no vuelve a reposo");
        uint8_t out[FB];
        render_press(A[0], p, out);
        CHECK(memcmp(out, A[0], FB) == 0, "en reposo el icono no queda intacto");
        if (dir) {
            // Pulsar 250 ms y soltar, en la tecla 5.
            Press q; uint8_t pant[N][FB]; memcpy(pant, A, sizeof(pant));
            for (int f = 0; f < 60; f++) {
                press_step(q, (f >= 6 && f < 21) ? 1.0f : 0.0f, 1.0f / 60.0f);
                render_press(A[4], q, pant[4]);
                // En el teclado la tecla pulsada además va en negativo.
                if (f >= 6 && f < 21) for (int b = 0; b < FB; b++) pant[4][b] ^= 0xFF;
                volcar(dir, "press", f, pant);
            }
        }
    }

    if (fallos) { printf("%d comprobaciones fallidas\n", fallos); return 1; }
    printf("OK\n");
    return 0;
}
