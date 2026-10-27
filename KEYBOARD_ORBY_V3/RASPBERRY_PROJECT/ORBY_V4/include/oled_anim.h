#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

// ============================================================================
// TRANSICIONES DE LAS PANTALLAS — al estilo de la pantalla de inicio del iPhone
// ============================================================================
// No hay ni una curva inventada: los muelles y los fundidos salen de los
// `setDefaultValues` de SpringBoardHome de iOS 27.2 (iPhone18,3, compilación
// 24B119), leídos del binario de la caché dyld. De dónde sale cada número está
// apuntado junto a su constante.
//
// Este fichero no conoce el SDK de la Pico ni el bus SPI: recibe framebuffers y
// devuelve framebuffers. Así se puede compilar en el PC y mirar las
// animaciones fotograma a fotograma (tools/test/test_anim.cpp) sin teclado.
//
// Las pantallas son de 1 bit, así que la opacidad no existe: un fundido es un
// tramado Bayer 4x4 que va encendiendo píxeles de la imagen nueva y apagando
// los de la vieja. Con 72x40 se lee como un fundido de verdad, no como ruido.

namespace OledAnim {

static constexpr int W  = 72;
static constexpr int H  = 40;
static constexpr int FB = 360;   // 72 x 5 páginas de 8 filas
static constexpr int N  = 10;    // pantallas

// Dónde está cada pantalla en la rejilla de 3x4 teclas. La 10 va en la tecla
// del centro de la última fila; las otras dos de esa fila no llevan pantalla.
static const int8_t COL[N] = {0, 1, 2, 0, 1, 2, 0, 1, 2, 1};
static const int8_t ROW[N] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3};

static inline int screen_at(int col, int row) {
    for (int i = 0; i < N; i++) if (COL[i] == col && ROW[i] == row) return i;
    return -1;
}

// --- Muelles -----------------------------------------------------------------
// Masa, rigidez y amortiguamiento, exactamente como los recibe CASpringAnimation.
struct Spring { float mass, stiffness, damping; };

// SBHFolderOpenSettings / SBHFolderCloseSettings → centralAnimationSettings:
// mass 1, stiffness 341.51, damping 36.96. Es un muelle crítico (ζ = 1,00,
// response 0,34 s): llega sin rebotar y sin cola. Abre y cierra los menús, como
// una carpeta, y hace crecer los iconos al cambiar de perfil.
static constexpr Spring FOLDER = {1.0f, 341.51f, 36.96f};

// SBHIconEditingSettings: nudgeSpringDuration 0.6, nudgeSpringDampingRatio 1.1.
// Es el muelle con el que los iconos se apartan para dejar sitio al arrastrar
// uno, o sea, iconos que viajan de un hueco a otro: lo mismo que hace aquí el
// cambio de página. UIKit lo da como duración + ζ; pasado a masa 1 con la misma
// convención (envolvente por debajo de 0,001 a los 0,6 s) sale ω = 19,351.
static constexpr Spring NUDGE = {1.0f, 374.45f, 42.572f};

// SBHHomeScreenSettings, transición de tirar para buscar: dampingRatio 1.0,
// response 0.54. Algo más lento que la carpeta: se usa para entrar y salir de
// los submenús, que es navegar en profundidad, no abrir algo encima.
static constexpr Spring PULL = {1.0f, 135.39f, 23.271f};

// Desplazamiento que le queda al muelle por recorrer (1 → 0), partiendo del
// reposo. Las tres ramas porque el nudge está sobreamortiguado (ζ = 1,1).
static inline float spring_left(const Spring& s, float t) {
    if (t <= 0.0f) return 1.0f;
    const float w = sqrtf(s.stiffness / s.mass);
    const float z = s.damping / (2.0f * sqrtf(s.stiffness * s.mass));
    if (z < 0.999f) {
        const float wd = w * sqrtf(1.0f - z * z);
        return expf(-z * w * t) * (cosf(wd * t) + (z * w / wd) * sinf(wd * t));
    }
    if (z < 1.001f) return expf(-w * t) * (1.0f + w * t);
    const float r  = sqrtf(z * z - 1.0f);
    const float r1 = -w * (z - r), r2 = -w * (z + r);
    return (r2 * expf(r1 * t) - r1 * expf(r2 * t)) / (r2 - r1);
}

static inline float spring(const Spring& s, float t) { return 1.0f - spring_left(s, t); }

// Cuándo el muelle ya no mueve nada que se vea. Es lo que hace QuartzCore en
// -[CASpringAnimation durationForEpsilon:], solo que aquí el épsilon se elige
// en píxeles: medio píxel del recorrido total. Pasado eso, el panel enseña lo
// mismo fotograma tras fotograma y seguir animando solo gasta SPI.
static inline float settle(const Spring& s, float travel_px) {
    const float eps = 0.5f / (travel_px > 1.0f ? travel_px : 1.0f);
    float t = 0.0f;
    while (t < 2.0f && fabsf(spring_left(s, t)) > eps) t += 0.005f;
    return t;
}

// --- Fundidos con curva -------------------------------------------------------
// Los fundidos de SpringBoard no son muelles: duración, retardo y una de las
// curvas de UIView. `setCurve:` recibe la opción de UIView tal cual (0x10000 =
// EaseIn, 0x20000 = EaseOut), y las curvas son las de CAMediaTimingFunction.
enum Curve : uint8_t { EASE_IN, EASE_OUT, EASE_IN_OUT, LINEAR };

struct Fade { float delay, duration; Curve curve; };

static inline float cubic_bezier(float x1, float y1, float x2, float y2, float u) {
    // Bisección sobre x(s): 14 pasadas sobran para 72 píxeles.
    float lo = 0.0f, hi = 1.0f, s = u;
    for (int i = 0; i < 14; i++) {
        s = 0.5f * (lo + hi);
        const float m = 1.0f - s;
        const float x = 3.0f * m * m * s * x1 + 3.0f * m * s * s * x2 + s * s * s;
        if (x < u) lo = s; else hi = s;
    }
    const float m = 1.0f - s;
    return 3.0f * m * m * s * y1 + 3.0f * m * s * s * y2 + s * s * s;
}

static inline float fade(const Fade& f, float t) {
    float u = (t - f.delay) / f.duration;
    if (u <= 0.0f) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    switch (f.curve) {
        case EASE_IN:     return cubic_bezier(0.42f, 0.0f, 1.0f,  1.0f, u);
        case EASE_OUT:    return cubic_bezier(0.0f,  0.0f, 0.58f, 1.0f, u);
        case EASE_IN_OUT: return cubic_bezier(0.42f, 0.0f, 0.58f, 1.0f, u);
        default:          return u;
    }
}

static inline float fade_end(const Fade& f) { return f.delay + f.duration; }

// SBHFolderOpenSettings: outerFolderFade 0.4 s EaseIn (lo de fuera se va),
// innerFolderFade 0.3 s con 0.1 de retardo, EaseIn (el contenido aparece).
static constexpr Fade OPEN_OUTER  = {0.0f,  0.4f, EASE_IN};
static constexpr Fade OPEN_INNER  = {0.1f,  0.3f, EASE_IN};
// SBHFolderCloseSettings: innerFolderFade 0.2 s EaseOut (el contenido se va).
static constexpr Fade CLOSE_INNER = {0.0f,  0.2f, EASE_OUT};

// Abrir y cerrar un menú (y la capa SUPER) no usan la pareja de carpeta de
// arriba tal cual. Con ella lo que entra aparece con EaseIn y 0,1 s de retardo,
// y para cuando se ve, el muelle ya lo ha llevado casi a su tamaño: en 1 bit no
// se notaba que creciera, solo que aparecía debajo. Aquí lo que entra empieza a
// moverse justo cuando empieza a verse (mismo retardo en su muelle) y aparece
// con EaseOut, así que se le ve crecer; lo que sale se va en 0,25 s con EaseIn,
// que lo deja a la vista mientras se agranda. Lo nuevo espera a que lo viejo
// esté casi ido (0,2 s): solapados, en 1 bit los dos dibujos se mezclan en una
// trama que no se lee como ninguno de los dos.
static constexpr Fade FOLDER_OUT = {0.0f,  0.25f, EASE_IN};
static constexpr Fade FOLDER_IN  = {0.2f,  0.2f,  EASE_OUT};

// Fundido corto para lo que cambia sin cambiar de sitio: la capa SUPER, el
// cursor de un menú, un icono recién subido. Es el innerFolderFade de cerrar.
static constexpr Fade SWAP = CLOSE_INNER;

// --- Onda desde el centro -------------------------------------------------------
// SBHCenterAppSuspendSettings: distanceEffect = YES, firstHopIncrement = 2,
// hopIncrementAcceleration = -1. Al volver a la pantalla de inicio los iconos no
// llegan a la vez: salen por anillos desde el centro, y cada salto entre anillos
// es un incremento más corto que el anterior (2, luego 1). La unidad es el
// fotograma a 60 Hz. El centro aquí es la pantalla 5.
static inline int ring(int i) {
    const int dc = COL[i] - 1, dr = ROW[i] - 1;
    const int ac = dc < 0 ? -dc : dc, ar = dr < 0 ? -dr : dr;
    return ac > ar ? ac : ar;
}

static inline float hop_delay(int i) {
    float d = 0.0f, inc = 2.0f;
    for (int r = ring(i); r > 0; r--) {
        d += inc;
        inc -= 1.0f;
        if (inc < 1.0f) inc = 1.0f;   // la onda nunca se para ni se adelanta
    }
    return d / 60.0f;
}

// --- Tipos de transición -------------------------------------------------------
enum Kind : uint8_t {
    CUT,          // sin animación: reposo, o nada que se pueda animar
    CROSSFADE,    // mismo sitio, contenido distinto
    PAGE_NEXT,    // la página entra por la derecha y empuja a la anterior
    PAGE_PREV,
    MENU_OPEN,    // de la pantalla normal a un menú (o a la capa SUPER): se abre como una carpeta
    MENU_CLOSE,   // del menú (o de SUPER) a la pantalla normal: la carpeta se cierra
    PUSH,         // entrar en un submenú
    POP,          // volver al menú principal
    PROFILE       // otro perfil: los iconos nuevos crecen en onda desde el centro
};

// Escalas de la carpeta. Al abrir, el contenido del menú nace casi de un punto
// (3 %: dos píxeles de ancho) y la pantalla de fondo se acerca un 60 % mientras
// se va, como la rejilla de iconos cuando se abre una carpeta; al cerrar, al
// revés: el menú se encoge hasta un punto. Con el 60 % y el 25 % de antes, en
// 72x40 el cambio de tamaño se quedaba en dos o tres píxeles y no se leía como
// zoom.
static constexpr float FOLDER_MIN  = 0.03f;
static constexpr float BG_ZOOM     = 1.60f;
static constexpr float PROFILE_MIN = 0.60f;

// Hueco entre pantallas en el carrusel de páginas, a escala real: los 72 px
// del panel miden unos 10 mm (7,2 px/mm) y entre una pantalla y la vecina hay
// 9,75 mm, o sea 70 px. Con el hueco comprimido un icono parecía saltar de
// tecla en tecla más deprisa de lo que se mueve el resto del carrusel; a escala
// real va a la misma velocidad mientras cruza el espacio que no se ve.
static constexpr int CAROUSEL_GAP = 70;
static constexpr int PITCH        = W + CAROUSEL_GAP;
static constexpr int CANVAS       = 3 * PITCH;

static inline float g_settle[PROFILE + 1];

static inline void init() {
    g_settle[CUT]        = 0.0f;
    g_settle[CROSSFADE]  = fade_end(SWAP);
    g_settle[PAGE_NEXT]  = settle(NUDGE, (float)CANVAS);
    g_settle[PAGE_PREV]  = g_settle[PAGE_NEXT];
    // Lo que sale ya no se ve pasado su fundido; lo que entra manda, y al
    // cerrar entra desde el zoom grande, que es el recorrido más largo.
    const float viaje = fmaxf(1.0f - FOLDER_MIN, BG_ZOOM - 1.0f) * W / 2;
    g_settle[MENU_OPEN]  = fmaxf(fade_end(FOLDER_OUT),
                                 fmaxf(FOLDER_IN.delay + settle(FOLDER, viaje), fade_end(FOLDER_IN)));
    g_settle[MENU_CLOSE] = g_settle[MENU_OPEN];
    g_settle[PUSH]       = settle(PULL, (float)W);
    g_settle[POP]        = g_settle[PUSH];
    g_settle[PROFILE]    = fmaxf(fade_end(OPEN_OUTER),
                                 fmaxf(settle(FOLDER, (1.0f - PROFILE_MIN) * W / 2), fade_end(OPEN_INNER))
                                 + hop_delay(9));   // la 10 es la del anillo más lejano
}

// --- Velocidad configurable ---------------------------------------------------
// Un multiplicador por grupo de transición: 1.0 es el muelle "de fábrica" de
// SpringBoard tal cual está arriba. Por encima, la misma animación tarda menos
// tiempo real sin cambiar de forma: escalar el tiempo que recibe un muelle
// crítico/sobreamortiguado por `r` es exactamente el muelle que saldría de
// multiplicar su rigidez por r² y su amortiguamiento por r (mismo ζ, más ω), así
// que no hace falta tocar las constantes de arriba para acelerarlo.
//
// El valor neutro de esta librería es 1.0 en los cinco grupos a propósito: así
// test_anim.cpp sigue comprobando los muelles de Apple sin recortar. Quien fija
// la velocidad "de fábrica" del teclado (ANIM_BASE_PCT, distinta por grupo) es
// main.cpp, con OledAnim::set_rate.
enum Group : uint8_t { G_MENU, G_CAROUSEL, G_SUBMENU, G_PROFILE, G_FADE, G_COUNT };

static inline float g_rate[G_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

static inline Group group_of(Kind k) {
    switch (k) {
        case MENU_OPEN:
        case MENU_CLOSE: return G_MENU;
        case PAGE_NEXT:
        case PAGE_PREV:  return G_CAROUSEL;
        case PUSH:
        case POP:        return G_SUBMENU;
        case PROFILE:    return G_PROFILE;
        default:         return G_FADE;   // CROSSFADE; CUT no anima
    }
}

static inline float clamp_rate(float r) {
    if (r < 0.25f) return 0.25f;
    if (r > 4.0f)  return 4.0f;
    return r;
}

static inline void set_rate(Group g, float r) {
    if (g < G_COUNT) g_rate[g] = clamp_rate(r);
}

static inline float rate(Group g) { return (g < G_COUNT) ? g_rate[g] : 1.0f; }

static inline float rate_of(Kind k) { return g_rate[group_of(k)]; }

// Lo que dura una transición, onda incluida, al ritmo configurado ahora mismo.
// Pasado ese instante real render() devuelve el destino tal cual, bit a bit.
static inline float duration(Kind k) { return g_settle[k] / rate_of(k); }

// --- Dibujo -----------------------------------------------------------------
static inline bool get_px(const uint8_t* fb, int x, int y) {
    return (fb[(y >> 3) * W + x] >> (y & 7)) & 1;
}
static inline void put_px(uint8_t* fb, int x, int y, bool on) {
    const uint8_t bit = (uint8_t)(1u << (y & 7));
    if (on) fb[(y >> 3) * W + x] |= bit;
    else    fb[(y >> 3) * W + x] &= (uint8_t)~bit;
}

// Umbrales del Bayer 4x4. Un píxel de una capa con opacidad a se pinta si su
// umbral cae en la ventana [lo, hi) de esa capa. Dos capas con ventanas que no
// se pisan se reparten los píxeles: un fundido cruzado sin que nada se duplique.
static const uint8_t BAYER[4][4] = {
    { 0,  8,  2, 10},
    {12,  4, 14,  6},
    { 3, 11,  1,  9},
    {15,  7, 13,  5}
};

static inline uint8_t alpha16(float a) {
    if (a <= 0.0f) return 0;
    if (a >= 1.0f) return 16;
    return (uint8_t)(a * 16.0f + 0.5f);
}

struct Layer {
    const uint8_t* src;
    float   scale;     // respecto al centro de la pantalla
    float   dx, dy;    // desplazamiento en píxeles, después de escalar
    uint8_t lo, hi;    // ventana del Bayer en la que esta capa existe
    bool    opaque;    // su negro tapa lo de debajo (dentro de su rectángulo)
};

// Pinta una capa encima de lo que haya en `out`. El escalado es por vecino más
// cercano y separable: se calcula una vez qué columna y qué fila de origen le
// toca a cada píxel de destino, y el bucle interior ya no hace aritmética.
static inline void draw_layer(uint8_t* out, const Layer& L) {
    if (L.lo >= L.hi || !L.src) return;
    int8_t mx[W], my[H];
    const float inv = 1.0f / L.scale;
    for (int x = 0; x < W; x++) {
        const float sx = ((float)x + 0.5f - W * 0.5f - L.dx) * inv + W * 0.5f;
        const int i = (int)floorf(sx);
        mx[x] = (i >= 0 && i < W) ? (int8_t)i : (int8_t)-1;
    }
    for (int y = 0; y < H; y++) {
        const float sy = ((float)y + 0.5f - H * 0.5f - L.dy) * inv + H * 0.5f;
        const int i = (int)floorf(sy);
        my[y] = (i >= 0 && i < H) ? (int8_t)i : (int8_t)-1;
    }
    const bool full = (L.lo == 0 && L.hi >= 16);
    for (int y = 0; y < H; y++) {
        if (my[y] < 0) continue;
        const uint8_t* brow = BAYER[y & 3];
        for (int x = 0; x < W; x++) {
            if (mx[x] < 0) continue;
            if (!full) {
                const uint8_t b = brow[x & 3];
                if (b < L.lo || b >= L.hi) continue;
            }
            const bool on = get_px(L.src, mx[x], my[y]);
            if (on || L.opaque) put_px(out, x, y, on);
        }
    }
}

// Raya vertical: es la sombra del borde de la vista que entra al navegar. En
// las columnas de los extremos no se pinta: ahí ya no hay borde que señalar y
// quedaría una raya suelta pegada al marco al empezar o terminar.
static inline void vline(uint8_t* out, int x) {
    if (x < 1 || x >= W - 1) return;
    for (int p = 0; p < 5; p++) out[p * W + x] = 0xFF;
}

// Carrusel: las diez pantallas son ventanas sobre una tira continua donde la
// página vieja y la nueva van una detrás de otra. Desplazar la tira es lo que
// hace que un icono salga por un lado de una tecla y entre por el otro de la
// vecina. Solo se mueve en horizontal, así que se copian columnas de bytes
// enteras: es la transición más barata de todas.
static inline void draw_carousel(uint8_t* out, int i, float shift,
                          const uint8_t from[][FB], const uint8_t to[][FB]) {
    const int s = (int)lroundf(shift);
    for (int x = 0; x < W; x++) {
        int X = COL[i] * PITCH + x + s;
        const uint8_t (*page)[FB] = from;
        if (X >= CANVAS)  { X -= CANVAS; page = to; }
        else if (X < 0)   { X += CANVAS; page = to; }
        const int lx = X % PITCH;
        const int j  = screen_at(X / PITCH, ROW[i]);
        if (lx >= W || j < 0) {
            for (int p = 0; p < 5; p++) out[p * W + x] = 0;
        } else {
            for (int p = 0; p < 5; p++) out[p * W + x] = page[j][p * W + lx];
        }
    }
}

// Un fotograma de la pantalla i en el instante t (segundos desde el inicio).
static inline void render(Kind k, float t, int i,
                   const uint8_t from[][FB], const uint8_t to[][FB], uint8_t* out) {
    if (k == CUT || t >= duration(k)) { memcpy(out, to[i], FB); return; }

    memset(out, 0, FB);
    const uint8_t* a = from[i];
    const uint8_t* b = to[i];

    // Una pantalla que no cambia se queda quieta, salvo en el carrusel, donde
    // por ella pasan las vecinas. Sin esto, las pantallas vacías de un submenú
    // verían cruzar la sombra del borde sola, sin nada que la empuje.
    if (k != PAGE_NEXT && k != PAGE_PREV && memcmp(a, b, FB) == 0) {
        memcpy(out, b, FB);
        return;
    }

    // Tiempo virtual: a `rate_of(k)` veces el real, para que el muelle/fundido
    // de más abajo recorra la misma forma en menos tiempo real (ver set_rate).
    const float tv = t * rate_of(k);

    switch (k) {
    case CROSSFADE: {
        const uint8_t n = alpha16(fade(SWAP, tv));
        draw_layer(out, {a, 1.0f, 0, 0, n, 16, false});
        draw_layer(out, {b, 1.0f, 0, 0, 0,  n, false});
        return;
    }
    case PAGE_NEXT:
    case PAGE_PREV: {
        const float p = spring(NUDGE, tv) * (float)CANVAS;
        draw_carousel(out, i, k == PAGE_NEXT ? p : -p, from, to);
        return;
    }
    case MENU_OPEN:
    case MENU_CLOSE: {
        // Cerrar es abrir con las escalas cambiadas de lado: al abrir lo de
        // antes se agranda y lo nuevo crece desde pequeño; al cerrar lo de
        // antes se encoge y lo nuevo llega desde grande.
        const bool abre = (k == MENU_OPEN);
        const float sale_a  = abre ? BG_ZOOM : FOLDER_MIN;
        const float entra_de = abre ? FOLDER_MIN : BG_ZOOM;
        const float po = spring(FOLDER, tv);
        const float pi = spring(FOLDER, tv - FOLDER_IN.delay);
        const uint8_t no = alpha16(1.0f - fade(FOLDER_OUT, tv));
        const uint8_t ni = alpha16(fade(FOLDER_IN, tv));
        // Las ventanas se cuentan desde lados opuestos del Bayer: mientras las
        // dos opacidades no sumen más de 1 no comparten ni un píxel.
        draw_layer(out, {a, 1.0f + (sale_a - 1.0f) * po, 0, 0, (uint8_t)(16 - no), 16, false});
        draw_layer(out, {b, entra_de + (1.0f - entra_de) * pi, 0, 0, 0, ni, false});
        return;
    }
    case PUSH: {
        // La vista nueva entra entera por la derecha y tapa a la vieja, que se
        // retira un tercio y se apaga a medias: el empuje de UINavigationController.
        const float p = spring(PULL, tv);
        const float xn = W * (1.0f - p);
        draw_layer(out, {a, 1.0f, -W * 0.3f * p, 0, 0, alpha16(1.0f - 0.5f * p), false});
        draw_layer(out, {b, 1.0f, xn, 0, 0, 16, true});
        vline(out, (int)lroundf(xn) - 1);
        return;
    }
    case POP: {
        const float p = spring(PULL, tv);
        const float xo = W * p;
        draw_layer(out, {b, 1.0f, -W * 0.3f * (1.0f - p), 0, 0, alpha16(0.5f + 0.5f * p), false});
        draw_layer(out, {a, 1.0f, xo, 0, 0, 16, true});
        vline(out, (int)lroundf(xo) - 1);
        return;
    }
    case PROFILE: {
        // Los fundidos son la pareja de abrir carpeta (lo de fuera se va en
        // 0,4 s, lo de dentro llega con 0,1 de retardo), y no la de cerrar: con
        // 1 bit, irse en 0,2 s antes de que llegue nada deja las diez pantallas
        // a negro un instante, y se lee como un parpadeo, no como un cambio.
        const float tl = tv - hop_delay(i);
        const float p  = spring(FOLDER, tl);
        const float fo = fade(OPEN_OUTER, tv);
        const uint8_t no = alpha16(1.0f - fo);
        const uint8_t ni = alpha16(fade(OPEN_INNER, tl));
        draw_layer(out, {a, 1.0f - 0.1f * fo, 0, 0, (uint8_t)(16 - no), 16, false});
        if (tl > 0.0f)
            draw_layer(out, {b, PROFILE_MIN + (1.0f - PROFILE_MIN) * p, 0, 0, 0, ni, false});
        return;
    }
    default:
        memcpy(out, b, FB);
        return;
    }
}

// --- Pulsación ---------------------------------------------------------------
// Al pulsar, el icono se hunde un 12 % y al soltar vuelve, con el muelle de la
// carpeta. Va integrado paso a paso, y no con la fórmula cerrada, porque una
// pulsación se puede soltar a medio hundir y el muelle tiene que dar la vuelta
// desde donde esté y con la velocidad que lleve.
static constexpr float PRESS_DEPTH = 0.12f;

struct Press { float x = 0.0f, v = 0.0f; };

// Avanza dt segundos hacia `target` (0 suelta, 1 pulsada). Devuelve true si
// todavía se mueve.
static inline bool press_step(Press& p, float target, float dt) {
    const Spring& s = FOLDER;
    while (dt > 0.0f) {
        const float h = dt > 0.002f ? 0.002f : dt;   // por debajo de la escala del muelle
        const float acc = (-s.stiffness * (p.x - target) - s.damping * p.v) / s.mass;
        p.v += acc * h;
        p.x += p.v * h;
        dt -= h;
    }
    if (fabsf(p.x - target) < 0.002f && fabsf(p.v) < 0.02f) {
        p.x = target; p.v = 0.0f;
        return false;
    }
    return true;
}

static inline void render_press(const uint8_t* src, const Press& p, uint8_t* out) {
    if (p.x <= 0.0f) { memcpy(out, src, FB); return; }
    memset(out, 0, FB);
    draw_layer(out, {src, 1.0f - PRESS_DEPTH * p.x, 0, 0, 0, 16, false});
}

} // namespace OledAnim
