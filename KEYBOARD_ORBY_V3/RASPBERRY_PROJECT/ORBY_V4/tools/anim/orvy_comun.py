"""Lo común a las tres intros «orvy»: geometría, sprites y los pasos 1 a 4.

Guion compartido (30 fps), con las teclas 2, 5, 8 y 11 (pantallas 2, 5, 8 y 10):
  1. El dial nace de un punto en la tecla 5 y crece con un rebote elástico.
  2. Sube a la tecla 2 cruzando el hueco real entre teclas.
  3. Entran de una en una, elásticas: la r por la derecha hasta la 5 (cruzando
     la 6), la v por la izquierda hasta la 8 (cruzando la 7) y la y por abajo
     hasta la 11.
  4. «orvy» se queda quieto para leerlo.
Lo que pasa después es de cada final (final_*.py), y todos acaban con las diez
pantallas en negro: los iconos los abre el firmware como con SUPER.

Todo se coloca en un plano común a escala real: entre una pantalla y la
siguiente hay el mismo hueco que en el teclado (OledAnim::PITCH), y lo que cae
en el hueco no se ve, igual que en el carrusel de páginas.
"""
import math

from intro_dial import (W, H, COL, ROW, ROOT, d_seg, d_arc, dial, rle)

FPS = 30
PITCH = 142            # 72 px de panel + 70 de hueco: el paso real entre teclas

# Pantallas (índice base 0) de la columna central, de arriba abajo.
S_TOP, S_MID, S_LOW, S_BOT = 1, 4, 7, 9


def screen_origin(s):
    return COL[s] * PITCH, ROW[s] * PITCH


def screen_center(s):
    x, y = screen_origin(s)
    return x + W / 2, y + H / 2


# --- Letras (unidades del logotipo de los SVG) --------------------------------
# Trazo algo más grueso que en el logotipo (5.5): a esta escala se quedaba en
# cuatro píxeles, más fino que el anillo del dial que tienen encima.
HW = 7.5


def letter_r(x, y):
    return min(d_seg(x, y, 79.5, 74.5, 79.5, 43.5), d_arc(x, y, 97.5, 43.5, 18, 180, 90)) <= HW


def letter_v(x, y):
    return min(d_seg(x, y, 116.5, 25.5, 138.5, 74.5), d_seg(x, y, 138.5, 74.5, 160.5, 25.5)) <= HW


def letter_y(x, y):
    return min(d_seg(x, y, 178, 25.5, 200, 74.5), d_seg(x, y, 222, 25.5, 184.96, 108)) <= HW


S_LET = 37 / 97        # la y, con su cola, ocupa casi los 40 px de alto
S_ISO = 34 / 73.5      # el dial, como en intro_dial.py


def sprite(f, s, ux, uy, half, n=4):
    """Píxeles encendidos de la figura f a escala s, relativos al punto (ux, uy)
    de la figura, que queda en el origen. half: semilado de la caja a barrer, en
    píxeles. Supermuestreo n×n con umbral a la mitad, como raster()."""
    pts = []
    for j in range(-half, half):
        for i in range(-half, half):
            hit = 0
            for a in range(n):
                for b in range(n):
                    if f(ux + (i + (a + .5) / n) / s, uy + (j + (b + .5) / n) / s):
                        hit += 1
            if hit * 2 >= n * n:
                pts.append((i, j))
    return pts


R_PTS = sprite(letter_r, S_LET, 97.5, 50, 24)
V_PTS = sprite(letter_v, S_LET, 138.5, 50, 24)
Y_PTS = sprite(letter_y, S_LET, 200, 66.75, 24)

_dial = {}


def dial_pts(z):
    """El dial a la escala z (1 = tamaño normal), centrado en el origen."""
    key = round(z, 3)
    if key not in _dial:
        _dial[key] = sprite(dial(), S_ISO * key, 50, 50, 26) if key > 0.01 else []
    return _dial[key]


# --- Curvas -------------------------------------------------------------------
def elastic(p, k=5.0):
    """Llega pasándose y vuelve con un par de oscilaciones que se apagan. En p=1
    vale exactamente 1 (el coseno pasa por cero), así que no hay salto al final."""
    if p <= 0:
        return 0.0
    if p >= 1:
        return 1.0
    return 1 - math.exp(-k * p) * math.cos(2.5 * math.pi * p)


def ease_in_out(p):
    p = max(0.0, min(1.0, p))
    return 4 * p ** 3 if p < .5 else 1 - (-2 * p + 2) ** 3 / 2


def phase(f, a, b):
    return (f - a) / (b - a)


# --- Guion --------------------------------------------------------------------
T_ZOOM = (4, 24)
T_UP = (30, 50)
T_R = (44, 68)
T_V = (58, 82)
T_Y = (72, 96)

ENTRY = PITCH + W / 2 + 26   # r y v salen justo fuera del borde de la pantalla vecina


def placed(f):
    """Lista de (puntos, x, y) con lo que hay en el plano: el dial y las letras que han entrado."""
    out = []
    cx5, cy5 = screen_center(S_MID)
    cx2, cy2 = screen_center(S_TOP)
    if f >= T_ZOOM[0]:
        z = elastic(phase(f, *T_ZOOM))
        y = cy5 + (cy2 - cy5) * elastic(phase(f, *T_UP), k=6.5)
        out.append((dial_pts(z), cx5, y))
    for pts, t, s, dx, dy in ((R_PTS, T_R, S_MID, ENTRY, 0),
                              (V_PTS, T_V, S_LOW, -ENTRY, 0),
                              (Y_PTS, T_Y, S_BOT, 0, PITCH)):
        if f < t[0]:
            continue
        p = 1 - elastic(phase(f, *t))
        cx, cy = screen_center(s)
        out.append((pts, round(cx + dx * p), round(cy + dy * p)))
    return out


def to_pixels(items):
    px = set()
    for pts, x, y in items:
        x, y = round(x), round(y)
        for i, j in pts:
            px.add((x + i, y + j))
    return px



def hash01(*k):
    h = 2166136261
    for v in k:
        h = ((h ^ (v & 0xffffffff)) * 16777619) & 0xffffffff
    h ^= h >> 15
    h = (h * 2246822519) & 0xffffffff
    h ^= h >> 13
    return h / 2 ** 32


def screens(px):
    """Del plano a los diez framebuffers, en el formato de página del SSD1306."""
    fbs = [bytearray(360) for _ in range(10)]
    for s in range(10):
        ox, oy = screen_origin(s)
        fb = fbs[s]
        for x, y in px:
            i, j = x - ox, y - oy
            if 0 <= i < W and 0 <= j < H:
                fb[(j >> 3) * W + i] |= 1 << (j & 7)
    return [bytes(b) for b in fbs]
