"""Intro de arranque con la marca de Orvy: genera include/orby_intro_data.h.

La intro del editor (index.html) escribía «ORBY» con una fuente del sistema; la
marca nueva es un dibujo concreto, el dial, y no se puede pedir a un lienzo con
parámetros. Aquí se rasteriza la misma geometría que los SVG de
OrbyGUI/assets/brand/ y se escribe el flujo en el formato que ya lee
include/oled_intro.h, así que el firmware no cambia.

Guion (30 fps):
  1. El dial de la pantalla central da una vuelta y se asienta con un rebote.
  2. Se encienden las otras nueve, de dentro afuera: primero el píxel, luego el
     anillo se dibuja detrás de él.
  3. Los diez dan una vuelta a la vez.
  4. Se apagan de fuera adentro y la central se queda con el logotipo.

Uso:  python tools/anim/intro_dial.py            (escribe el .h)
      python tools/anim/intro_dial.py --preview  (además, PNG por fotograma en
                                                  tools/anim/_preview/)
"""
import math
import pathlib
import sys

W, H = 72, 40
FPS = 30
# Misma rejilla que engine.js: la pantalla 10 cuelga sola bajo la columna central.
COL = [0, 1, 2, 0, 1, 2, 0, 1, 2, 1]
ROW = [0, 0, 0, 1, 1, 1, 2, 2, 2, 3]
CENTER = 4

ROOT = pathlib.Path(__file__).resolve().parents[2]


# --- Geometría (unidades de los SVG maestros) --------------------------------
def d_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def d_arc(px, py, cx, cy, r, a0, span):
    """Distancia a un arco que sale de a0 y recorre span grados en el sentido de
    las agujas del reloj en pantalla (y hacia abajo)."""
    if span <= 0:
        return 1e9
    a = math.degrees(math.atan2(py - cy, px - cx))
    if (a - a0) % 360 <= span:
        return abs(math.hypot(px - cx, py - cy) - r)
    ends = [(cx + r * math.cos(math.radians(g)), cy + r * math.sin(math.radians(g))) for g in (a0, a0 + span)]
    return min(math.hypot(px - x, py - y) for x, y in ends)


GAP_HALF = 36.3   # medio hueco del dial, en grados: el mismo de los SVG


def dial(rot=0.0, grow=1.0):
    """Isotipo girado rot grados. grow (0..1) es cuánto anillo hay dibujado,
    creciendo hacia atrás desde el píxel, que es lo que se enciende primero."""
    g = -45.0 + rot
    gr = math.radians(g)
    dx, dy = 50 + 30 * math.cos(gr), 50 + 30 * math.sin(gr)
    span = (360 - 2 * GAP_HALF) * grow
    start = g + 360 - GAP_HALF - span

    def f(x, y):
        if math.hypot(x - dx, y - dy) <= 6.75:
            return True
        return d_arc(x, y, 50, 50, 30, start, span) <= 6.75
    return f


def word(x, y):
    # A 64 px el píxel de la o se pegaba al anillo: hueco algo más abierto y
    # píxel algo menor que en el SVG, solo en esta versión de 1 bit.
    if math.hypot(x - 47.32, y - 32.68) <= 5.0:
        return True
    return min(
        d_arc(x, y, 30, 50, 24.5, 2, 266),
        d_seg(x, y, 79.5, 74.5, 79.5, 43.5), d_arc(x, y, 97.5, 43.5, 18, 180, 90),
        d_seg(x, y, 116.5, 25.5, 138.5, 74.5), d_seg(x, y, 138.5, 74.5, 160.5, 25.5),
        d_seg(x, y, 178, 25.5, 200, 74.5), d_seg(x, y, 222, 25.5, 184.96, 108)) <= 5.5


def raster(f, s, ox, oy, n=4):
    """Framebuffer de 360 bytes en el formato de página del SSD1306 (bit 0 arriba),
    con supermuestreo n×n y umbral a la mitad."""
    fb = bytearray(360)
    for j in range(H):
        for i in range(W):
            hit = 0
            for a in range(n):
                for b in range(n):
                    if f((i + (a + .5) / n - ox) / s, (j + (b + .5) / n - oy) / s):
                        hit += 1
            if hit * 2 >= n * n:
                fb[(j >> 3) * W + i] |= 1 << (j & 7)
    return bytes(fb)


S_ISO = 34 / 73.5                       # el dial ocupa 34 de los 40 px de alto
S_WORD = 64 / 228
BLANK = bytes(360)
_cache = {}


def dial_fb(rot, grow=1.0):
    key = (round(rot, 1), round(grow, 3))
    if key not in _cache:
        _cache[key] = raster(dial(rot, grow), S_ISO, 36 - 50 * S_ISO, 20 - 50 * S_ISO)
    return _cache[key]


WORD_FB = raster(word, S_WORD, (W - 64) / 2, (H - 94 * S_WORD) / 2 - 20 * S_WORD)


def spin_ease(p):
    # Arranca rápido y se asienta con un pequeño rebote, como un mando que se suelta.
    return 1 - (1 - p) ** 3 * math.cos(p * math.pi * 1.5)


# --- Guion --------------------------------------------------------------------
DIST = [math.hypot(COL[i] - COL[CENTER], ROW[i] - ROW[CENTER]) for i in range(10)]

T_SPIN1 = (6, 40)        # vuelta de la central
T_WAVE = 40              # empiezan a encenderse las demás
WAVE_STEP = 4            # fotogramas por unidad de distancia
GROW = 6                 # fotogramas que tarda en dibujarse un anillo
T_SPIN2 = (60, 82)       # vuelta de las diez
T_OUT = 84               # se apagan de fuera adentro
T_WORD = 90
T_END = 112


def screen_at(s, f):
    if s == CENTER:
        if f < T_SPIN1[0]:
            return BLANK
        if f < T_SPIN1[1]:
            p = (f - T_SPIN1[0]) / (T_SPIN1[1] - T_SPIN1[0])
            return dial_fb(-360 + 360 * spin_ease(p))
    else:
        on = T_WAVE + round(DIST[s] * WAVE_STEP)
        off = T_OUT + round((max(DIST) - DIST[s]) * 2)
        if f < on or f >= off:
            return BLANK
        if f < on + GROW:
            return dial_fb(0, (f - on) / GROW)
    if s == CENTER and f >= T_WORD:
        return WORD_FB
    if T_SPIN2[0] <= f < T_SPIN2[1]:
        p = (f - T_SPIN2[0]) / (T_SPIN2[1] - T_SPIN2[0])
        return dial_fb(-360 + 360 * spin_ease(p))
    return dial_fb(0)


def rle(fb):
    out, i = bytearray(), 0
    while i < len(fb):
        v, n = fb[i], 1
        while i + n < len(fb) and fb[i + n] == v and n < 255:
            n += 1
        out += bytes((n, v))
        i += n
    return out


def bake():
    stream, prev = bytearray(), [None] * 10
    frames = []
    for f in range(T_END):
        scr = [screen_at(s, f) for s in range(10)]
        frames.append(scr)
        mask, payload = 0, bytearray()
        for s in range(10):
            if scr[s] == prev[s]:
                continue
            mask |= 1 << s
            enc = rle(scr[s])
            payload += len(enc).to_bytes(2, 'little') + enc
            prev[s] = scr[s]
        stream += mask.to_bytes(2, 'little') + payload
    return stream, frames


def header(stream):
    lines = [
        '// ============================================================',
        '// Intro de arranque de Orvy — GENERADO, no editar a mano',
        '// Creado con tools/anim/intro_dial.py (la geometría es la de',
        '// OrbyGUI/assets/brand/): python tools/anim/intro_dial.py',
        f'// {T_END} fotogramas a {FPS} fps ({T_END / FPS:.2f} s), {len(stream)} bytes',
        '// ============================================================',
        '#pragma once',
        '#include <stdint.h>',
        '',
        f'#define INTRO_FPS    {FPS}',
        f'#define INTRO_FRAMES {T_END}',
        '',
        '// Por fotograma: uint16 con las pantallas que cambian (bit 0 = OLED 1) y,',
        '// por cada una, uint16 de longitud y su framebuffer comprimido en pares',
        '// (repeticiones, valor).',
        'static const uint8_t intro_stream[] = {',
    ]
    for i in range(0, len(stream), 16):
        lines.append('    ' + ','.join(f'0x{b:02x}' for b in stream[i:i + 16]) + ',')
    lines += ['};', '']
    return '\n'.join(lines)


def preview(frames):
    from PIL import Image
    out = ROOT / 'tools' / 'anim' / '_preview'
    out.mkdir(exist_ok=True)
    gx, gy, k = 24, 20, 3
    cw, ch = 3 * (W + gx), 4 * (H + gy)
    for f, scr in enumerate(frames):
        im = Image.new('RGB', (cw, ch), (10, 12, 14))
        px = im.load()
        for s in range(10):
            ox, oy = COL[s] * (W + gx) + gx // 2, ROW[s] * (H + gy) + gy // 2
            for y in range(H):
                for x in range(W):
                    on = (scr[s][(y >> 3) * W + x] >> (y & 7)) & 1
                    px[ox + x, oy + y] = (157, 233, 255) if on else (2, 3, 4)
        im.resize((cw * k, ch * k), Image.NEAREST).save(out / f'{f:03d}.png')
    print(f'vista previa en {out}')


if __name__ == '__main__':
    # Desde la 4.25 el .h lleva varias intros (intro_orvy.py) y el reproductor
    # ya no entiende el formato de una sola que escribe esto: hoy este fichero
    # se usa como biblioteca de geometría.
    if '--force' not in sys.argv:
        sys.exit('intro_dial.py escribe el formato anterior a la 4.25 y rompería la '
                 'compilación; usa intro_orvy.py (o --force si sabes lo que haces).')
    stream, frames = bake()
    dst = ROOT / 'include' / 'orby_intro_data.h'
    dst.write_text(header(stream), encoding='utf-8', newline='\n')
    print(f'{dst}: {T_END} fotogramas, {len(stream)} bytes')
    if '--preview' in sys.argv:
        preview(frames)
