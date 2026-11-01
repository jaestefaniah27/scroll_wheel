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

// Solo texto: lo más grande que quepa, con las reglas del nombre del modo
// ayuda (screen_name, más abajo).
static inline void screen_name(uint8_t* fb, const char* s);
static inline void screen_text(uint8_t* fb, const char* s) { screen_name(fb, s); }

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

// --- Teclas: icono grande y modo ayuda -------------------------------------------------
// Sin el nombre debajo, el icono de una tecla sale a x1.5 y casi llena la
// pantalla: a 24x22 con el nombre en 7 px no se leía ni lo uno ni lo otro. El
// nombre lo enseña el modo ayuda (screen_name). Ver layout.py.
static constexpr int BIG_X = (W - BIG_W) / 2;
static constexpr int BIG_Y = (H - BIG_H) / 2;

static inline void screen_icon_big(uint8_t* fb, uint16_t icon) {
    memset(fb, 0, FB);
    if (icon >= BIG_COUNT) return;
    const uint8_t* p = big_icon_cols + (size_t)icon * BIG_W * 5;
    for (int c = 0; c < BIG_W; c++, p += 5) {
        const uint64_t bits = (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16)
                            | ((uint64_t)p[3] << 24) | ((uint64_t)p[4] << 32);
        for (int r = 0; r < BIG_H; r++) if ((bits >> r) & 1) set_px(fb, BIG_X + c, BIG_Y + r);
    }
}

// --- El texto, lo más grande que quepa ----------------------------------------------
// Para el nombre del modo ayuda y para cualquier tecla que solo lleve texto.
// La fuente grande cabe dos veces en alto; de lo que cabe se elige lo primero:
//   1. una línea grande        (normal y luego apretada)
//   2. dos líneas por espacio  (normal y luego apretada)
//   3. dos líneas con guion    (normal y luego apretada)
//   4. pequeña, en una línea y luego en dos
// «Apretada» es la grande con 1 px entre letras y sin margen: así caben en
// grande todos los nombres del catálogo. Ver layout.py, que es el modelo.
//
// Se trabaja ya descodificado: partir con UTF-8 a medias obligaría a vigilar
// en cada corte que no cae dentro de una tilde.
static constexpr int NAME_MAX = 40;
static constexpr int BIG_GAP[2]   = {2, 1};
static constexpr int BIG_AVAIL[2] = {W - 6, W};
static constexpr int SMALL_AVAIL  = W - 6;
static constexpr int NAME_BIG_TOP[2]   = {4, 22};   // los rabos de la 2.ª acaban en la 39
static constexpr int NAME_SMALL_TOP[2] = {10, 22};

// Ancho en pequeño (gap 1) o en grande con `gap` px entre letras.
static inline int cps_width(const uint32_t* c, int n, bool big, int gap) {
    if (n <= 0) return 0;
    int w = 0;
    for (int i = 0; i < n; i++) {
        const int g = find_glyph(c[i]);
        w += big ? font_big[g].w : font_small[g].w;
    }
    return w + (n - 1) * gap;
}

static inline void draw_cps(uint8_t* fb, const uint32_t* c, int n, int cx, int top, bool big, int gap) {
    int x = cx - cps_width(c, n, big, gap) / 2;
    for (int i = 0; i < n; i++) {
        const int g = find_glyph(c[i]);
        if (big) {
            const Glyph& gl = font_big[g];
            for (int k = 0; k < gl.w; k++) put_col(fb, x + k, top, big_cols[gl.off + k], 18);
            x += gl.w + gap;
        } else {
            const Glyph& gl = font_small[g];
            for (int k = 0; k < gl.w; k++) put_col(fb, x + k, top, font_cols[gl.off + k], 9);
            x += gl.w + gap;
        }
    }
}

static inline bool name_letter(uint32_t c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
        || (c >= 0xC0 && c <= 0x17F && c != 0xD7 && c != 0xF7);
}
static inline bool name_vowel(uint32_t c) {
    static const uint32_t v[] = {'a', 'e', 'i', 'o', 'u', 'A', 'E', 'I', 'O', 'U', 0xE1, 0xE9, 0xED,
                                 0xF3, 0xFA, 0xFC, 0xC1, 0xC9, 0xCD, 0xD3, 0xDA, 0xDC};
    for (uint32_t x : v) if (x == c) return true;
    return false;
}
static inline bool name_consonant(uint32_t c) { return name_letter(c) && !name_vowel(c); }
static inline uint32_t name_lower(uint32_t c) { return (c >= 'A' && c <= 'Z') ? c + 32 : c; }

// Parejas que no se separan: «com-pro-bar», no «compr-obar».
static inline bool name_glued(uint32_t a, uint32_t b) {
    a = name_lower(a); b = name_lower(b);
    if ((a == 'l' && b == 'l') || (a == 'r' && b == 'r') || (a == 'c' && b == 'h')) return true;
    const bool oclusiva = a == 'b' || a == 'c' || a == 'd' || a == 'f' || a == 'g' || a == 'k'
                       || a == 'p' || a == 't';
    return oclusiva && (b == 'l' || b == 'r');
}

// Dónde se puede partir con guion: antes de consonante (o pareja inseparable)
// seguida de vocal, con dos letras por lo menos a cada lado.
static inline bool name_hyphen_at(const uint32_t* c, int n, int i) {
    if (i < 2 || i > n - 2) return false;
    if (!name_letter(c[i - 2]) || !name_letter(c[i - 1]) || !name_letter(c[i]) || !name_letter(c[i + 1]))
        return false;
    const bool cv  = name_consonant(c[i]) && name_vowel(c[i + 1]) && !name_glued(c[i - 1], c[i]);
    const bool ccv = i + 2 < n && name_glued(c[i], c[i + 1]) && name_vowel(c[i + 2]);
    return cv || ccv;
}

struct NameSplit { int a_len; bool hyphen; int b_from; };

// La partición (por espacio o con guion, según `hyphen`) que cabe con la línea
// más ancha más estrecha; a igualdad, la primera.
static inline bool name_split(const uint32_t* c, int n, bool big, int gap, int avail, bool hyphen,
                              NameSplit* out) {
    static const uint32_t dash = '-';
    int best = -1;
    for (int i = 1; i < n; i++) {
        NameSplit sp;
        if (!hyphen) {
            if (c[i] != ' ' || i + 1 >= n) continue;
            sp = {i, false, i + 1};
        } else {
            if (!name_hyphen_at(c, n, i)) continue;
            sp = {i, true, i};
        }
        const int wa = cps_width(c, sp.a_len, big, gap)
                     + (sp.hyphen ? gap + cps_width(&dash, 1, big, gap) : 0);
        const int wb = cps_width(c + sp.b_from, n - sp.b_from, big, gap);
        if (wa > avail || wb > avail) continue;
        const int m = wa > wb ? wa : wb;
        if (best < 0 || m < best) { best = m; *out = sp; }
    }
    return best >= 0;
}

static inline void draw_split(uint8_t* fb, const uint32_t* c, int n, const NameSplit& sp,
                              bool big, int gap) {
    const int* tops = big ? NAME_BIG_TOP : NAME_SMALL_TOP;
    uint32_t a[NAME_MAX + 1];
    memcpy(a, c, sizeof(uint32_t) * (size_t)sp.a_len);
    int na = sp.a_len;
    if (sp.hyphen) a[na++] = '-';
    draw_cps(fb, a, na, W / 2, tops[0], big, gap);
    draw_cps(fb, c + sp.b_from, n - sp.b_from, W / 2, tops[1], big, gap);
}

static inline void screen_name(uint8_t* fb, const char* s) {
    memset(fb, 0, FB);
    uint32_t c[NAME_MAX];
    int n = 0;
    while (*s && n < NAME_MAX) c[n++] = utf8_next(s);
    if (n == 0) return;
    NameSplit sp;
    for (int t = 0; t < 2; t++) {
        if (cps_width(c, n, true, BIG_GAP[t]) <= BIG_AVAIL[t]) {
            draw_cps(fb, c, n, W / 2, (H - 14) / 2, true, BIG_GAP[t]);
            return;
        }
    }
    for (int hyphen = 0; hyphen < 2; hyphen++) {
        for (int t = 0; t < 2; t++) {
            if (name_split(c, n, true, BIG_GAP[t], BIG_AVAIL[t], hyphen, &sp)) {
                draw_split(fb, c, n, sp, true, BIG_GAP[t]);
                return;
            }
        }
    }
    if (cps_width(c, n, false, 1) <= SMALL_AVAIL) {
        draw_cps(fb, c, n, W / 2, (H - 7) / 2, false, 1);
        return;
    }
    for (int hyphen = 0; hyphen < 2; hyphen++) {
        if (name_split(c, n, false, 1, SMALL_AVAIL, hyphen, &sp)) {
            draw_split(fb, c, n, sp, false, 1);
            return;
        }
    }
    draw_cps(fb, c, n, W / 2, (H - 7) / 2, false, 1);
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

// Pantalla de una tecla: con el icono grande si su etiqueta está en el
// catálogo, con el texto tal cual si no, y a negro si no tiene etiqueta.
static inline void screen_key(uint8_t* fb, const char* profile, const char* label) {
    const int e = lookup(profile, label);
    if (e >= 0)              screen_icon_big(fb, entries[e].icon);
    else if (label && *label) screen_text(fb, label);
    else                      memset(fb, 0, FB);
}

// La misma tecla en modo ayuda: el nombre del catálogo, que dice qué hace, y
// si no está, la etiqueta. Devuelve false si no hay nada que nombrar.
static inline bool screen_key_name(uint8_t* fb, const char* profile, const char* label) {
    const int e = lookup(profile, label);
    if (e >= 0)               screen_name(fb, entries[e].caption);
    else if (label && *label) screen_name(fb, label);
    else                      return false;
    return true;
}

} // namespace OledUi
