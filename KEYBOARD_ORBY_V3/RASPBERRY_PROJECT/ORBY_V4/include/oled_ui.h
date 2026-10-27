#pragma once
#include <stdint.h>
#include <string.h>
#include "oled_ui_data.h"

// ============================================================================
// COMPOSICIÓN DE LAS PANTALLAS
// ============================================================================
// Coloca iconos y texto sobre un framebuffer de 72x40 en el formato del SSD1306
// (5 páginas de 8 filas, bit 0 arriba). Los dibujos y la fuente vienen
// generados en oled_ui_data.h; las medidas son las de tools/oled_ui/layout.py,
// y tools/test/test_ui.cpp comprueba que las dos ponen cada píxel en su sitio.
//
// Como oled_anim.h, no toca el SDK de la Pico: se compila y se prueba en el PC.

namespace OledUi {

using namespace OledUiData;

static constexpr int W = 72, H = 40, FB = 360;

// Icono arriba, nombre debajo. Ver layout.py: los dos sitios dicen lo mismo.
static constexpr int ICON_X = (W - 24) / 2;
static constexpr int ICON_Y = 2;
static constexpr int CAP_Y  = 28;

static inline void set_px(uint8_t* fb, int x, int y) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    fb[(y >> 3) * W + x] |= (uint8_t)(1u << (y & 7));
}
static inline void clr_px(uint8_t* fb, int x, int y) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    fb[(y >> 3) * W + x] &= (uint8_t)~(1u << (y & 7));
}

// Pinta una columna de bits (bit 0 = fila `y`) en la columna x.
static inline void put_col(uint8_t* fb, int x, int y, uint32_t bits, int h) {
    for (int r = 0; r < h; r++) if (bits & (1u << r)) set_px(fb, x, y + r);
}

static inline void draw_sprite(uint8_t* fb, uint16_t id, int x, int y) {
    if (id >= SPR_COUNT) return;
    const Sprite& s = sprites[id];
    for (int c = 0; c < s.w; c++) put_col(fb, x + c, y, sprite_cols[s.off + c], s.h);
}

// --- Texto ----------------------------------------------------------------------
// Las etiquetas llegan de la app en UTF-8 (puede haber una ñ o una tilde). Un
// byte que no forme una secuencia válida se toma tal cual, como Latin-1: así un
// texto mal codificado se ve raro, pero nunca se come los caracteres siguientes.
static inline uint32_t utf8_next(const char*& s) {
    const uint8_t c = (uint8_t)*s;
    if (c < 0x80) { s++; return c; }
    if ((c & 0xE0) == 0xC0 && (s[1] & 0xC0) == 0x80) {
        uint32_t cp = ((uint32_t)(c & 0x1F) << 6) | (uint32_t)(s[1] & 0x3F);
        s += 2; return cp;
    }
    if ((c & 0xF0) == 0xE0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80) {
        uint32_t cp = ((uint32_t)(c & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) | (uint32_t)(s[2] & 0x3F);
        s += 3; return cp;
    }
    s++; return c;
}

static inline int find_glyph(uint32_t cp) {
    for (int pass = 0; pass < 2; pass++) {
        int lo = 0, hi = FONT_GLYPHS - 1;
        while (lo <= hi) {
            const int mid = (lo + hi) / 2;
            if (font_small[mid].cp == cp) return mid;
            if (font_small[mid].cp < cp) lo = mid + 1; else hi = mid - 1;
        }
        bool folded = false;
        for (const auto& f : fold) if (f[0] == cp) { cp = f[1]; folded = true; break; }
        if (!folded) break;
    }
    return find_glyph('?');
}

static inline int text_width(const char* s, bool big) {
    int w = 0, n = 0;
    while (*s) {
        const int g = find_glyph(utf8_next(s));
        w += big ? font_big[g].w : font_small[g].w;
        n++;
    }
    if (n == 0) return 0;
    return w + (n - 1) * (big ? 2 : 1);
}

// Texto centrado en cx con la parte alta de las mayúsculas en `top`.
static inline void draw_text(uint8_t* fb, const char* s, int cx, int top, bool big) {
    int x = cx - text_width(s, big) / 2;
    while (*s) {
        const int g = find_glyph(utf8_next(s));
        if (big) {
            const Glyph& gl = font_big[g];
            for (int c = 0; c < gl.w; c++) put_col(fb, x + c, top, big_cols[gl.off + c], 18);
            x += gl.w + 2;
        } else {
            const Glyph& gl = font_small[g];
            for (int c = 0; c < gl.w; c++) put_col(fb, x + c, top, font_cols[gl.off + c], 9);
            x += gl.w + 1;
        }
    }
}

// --- Pantallas tipo ---------------------------------------------------------------
static inline void screen_icon(uint8_t* fb, uint16_t icon, const char* caption) {
    memset(fb, 0, FB);
    draw_sprite(fb, icon, ICON_X, ICON_Y);
    if (caption && *caption) draw_text(fb, caption, W / 2, CAP_Y, false);
}

static inline void screen_system(uint8_t* fb, uint8_t sys) {
    screen_icon(fb, system_entries[sys].icon, system_entries[sys].caption);
}

// Solo texto: en grande si cabe con 3 px de margen a cada lado, si no en
// pequeño. Se centra por el alto de las mayúsculas: si contaran los rabos, el
// texto parecería caído.
static inline void screen_text(uint8_t* fb, const char* s) {
    memset(fb, 0, FB);
    if (text_width(s, false) * 2 <= W - 6) draw_text(fb, s, W / 2, (H - 14) / 2, true);
    else                                   draw_text(fb, s, W / 2, (H - 7) / 2, false);
}

static inline void screen_toggle(uint8_t* fb, bool on, const char* caption) {
    memset(fb, 0, FB);
    draw_sprite(fb, on ? SPR_TOGGLE_ON : SPR_TOGGLE_OFF, (W - 34) / 2, 5);
    draw_text(fb, caption, W / 2, CAP_Y, false);
}

// Puntos de página, como bajo la rejilla de iconos del iPhone.
static constexpr int DOT = 7, DOT_STEP = 12, DOT_Y = 10;

static inline void screen_dots(uint8_t* fb, int cur, int count, const char* caption) {
    memset(fb, 0, FB);
    const int x = (W - (DOT + (count - 1) * DOT_STEP)) / 2;
    for (int i = 0; i < count; i++)
        draw_sprite(fb, i == cur ? SPR_DOT_ON : SPR_DOT_OFF, x + i * DOT_STEP, DOT_Y);
    draw_text(fb, caption, W / 2, CAP_Y, false);
}

// La opción elegida de un menú va en negativo con las esquinas redondeadas.
// Se hornea en el framebuffer en vez de invertir el panel por hardware: así la
// selección también se anima, y las esquinas solo se pueden hacer así.
static const uint8_t PILL_CORNER[] = {3, 1, 1};

static inline void invert_pill(uint8_t* fb) {
    for (int i = 0; i < FB; i++) fb[i] = (uint8_t)~fb[i];
    for (int k = 0; k < (int)sizeof(PILL_CORNER); k++)
        for (int i = 0; i < PILL_CORNER[k]; i++) {
            clr_px(fb, i, k);          clr_px(fb, W - 1 - i, k);
            clr_px(fb, i, H - 1 - k);  clr_px(fb, W - 1 - i, H - 1 - k);
        }
}

// --- Catálogo ---------------------------------------------------------------------
// Las etiquetas se comparan sin distinguir mayúsculas: «Undo» escrita a mano
// en la app también es «UNDO».
static inline bool same_label(const char* a, const char* b) {
    for (;; a++, b++) {
        char x = *a, y = *b;
        if (x >= 'a' && x <= 'z') x = (char)(x - 32);
        if (y >= 'a' && y <= 'z') y = (char)(y - 32);
        if (x != y) return false;
        if (!x) return true;
    }
}

// Busca qué icono le toca a una etiqueta en un perfil. Primero la entrada
// propia del perfil («PREM/CUT»), luego la genérica. -1 si no hay.
static inline int lookup(const char* profile, const char* label) {
    if (!label || !*label) return -1;
    char key[20];
    if (profile && *profile) {
        size_t n = strlen(profile);
        if (n + 1 + strlen(label) < sizeof(key)) {
            memcpy(key, profile, n);
            key[n] = '/';
            strcpy(key + n + 1, label);
            for (const auto& e : entries) if (same_label(e.key, key)) return (int)(&e - entries);
        }
    }
    for (const auto& e : entries) if (same_label(e.key, label)) return (int)(&e - entries);
    return -1;
}

// Icono de un perfil por su nombre, para el menú de perfiles.
static inline int lookup_profile(const char* name) {
    if (!name || !*name) return -1;
    char key[12];
    key[0] = '@';
    strncpy(key + 1, name, sizeof(key) - 2);
    key[sizeof(key) - 1] = '\0';
    for (const auto& e : entries) if (same_label(e.key, key)) return (int)(&e - entries);
    return -1;
}

// Pantalla de una tecla: con icono si su etiqueta está en el catálogo, con el
// texto tal cual si no, y a negro si no tiene etiqueta.
static inline void screen_key(uint8_t* fb, const char* profile, const char* label) {
    const int e = lookup(profile, label);
    if (e >= 0)              screen_icon(fb, entries[e].icon, entries[e].caption);
    else if (label && *label) screen_text(fb, label);
    else                      memset(fb, 0, FB);
}

} // namespace OledUi
