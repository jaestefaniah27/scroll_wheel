# Iconos de las pantallas de ORBY.
#
# Se dibujan como vectores sobre un lienzo de 24x22 px y se rasterizan a 1 bit
# con supermuestreo. Las reglas para que salgan nítidos a ese tamaño:
#
# - Trazo de 2 px en todo el juego, como un SF Symbol en semibold. Con 1 px se
#   pierden en el OLED; con 3 no caben los detalles.
# - Las líneas rectas de 2 px van sobre coordenadas enteras (el borde entre dos
#   píxeles), así cubren exactamente dos filas o columnas y no salen borrosas.
# - Los rellenos dicen más que los contornos cuando el icono es pequeño: un
#   candado o una papelera se reconocen por la silueta.
#
# Cada icono es una función que recibe el lienzo `c` y dibuja con sus métodos.
import math
from PIL import Image, ImageDraw

IW, IH = 24, 22      # lienzo de un icono
SS = 8               # supermuestreo
TH = 0.5             # umbral de cobertura para encender un píxel


class Canvas:
    def __init__(self, w=IW, h=IH):
        self.w, self.h = w, h
        self.im = Image.new('L', (w * SS, h * SS), 0)
        self.d = ImageDraw.Draw(self.im)

    # -- utilidades internas ---------------------------------------------------
    def _p(self, x, y):
        return (x * SS, y * SS)

    def _disc(self, x, y, r, v):
        r *= SS
        x, y = x * SS, y * SS
        self.d.ellipse((x - r, y - r, x + r, y + r), fill=v)

    def _seg(self, a, b, w, v):
        (x0, y0), (x1, y1) = a, b
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy) or 1.0
        ox, oy = -dy / n * w / 2, dx / n * w / 2
        self.d.polygon([self._p(x0 + ox, y0 + oy), self._p(x1 + ox, y1 + oy),
                        self._p(x1 - ox, y1 - oy), self._p(x0 - ox, y0 - oy)], fill=v)

    # -- trazos --------------------------------------------------------------
    def line(self, *pts, w=2, cap='round', v=255):
        """Polilínea. `cap='round'` redondea extremos y uniones; 'butt' los
        deja rectos, que es lo que quieren las líneas de un marco."""
        for a, b in zip(pts, pts[1:]):
            self._seg(a, b, w, v)
        joins = pts if cap == 'round' else pts[1:-1]
        for (x, y) in joins:
            self._disc(x, y, w / 2, v)

    def arc(self, cx, cy, r, a0, a1, w=2, v=255, cap='round'):
        """Arco en grados, 0 = derecha, sentido horario (y hacia abajo)."""
        n = max(8, int(abs(a1 - a0) / 6))
        pts = [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
                cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
        self.line(*pts, w=w, cap=cap, v=v)
        return pts

    def circle(self, cx, cy, r, w=2, v=255):
        R, ri = (r + w / 2) * SS, (r - w / 2) * SS
        x, y = cx * SS, cy * SS
        self.d.ellipse((x - R, y - R, x + R, y + R), fill=v)
        if ri > 0:
            self.d.ellipse((x - ri, y - ri, x + ri, y + ri), fill=0 if v else 255)

    def ring(self, cx, cy, r, w=2, v=255):
        """Como circle(), pero el hueco no borra lo que hubiera dentro."""
        self.arc(cx, cy, r, 0, 360, w=w, v=v, cap='butt')

    def dot(self, x, y, r, v=255):
        self._disc(x, y, r, v)

    def ellipse(self, cx, cy, rx, ry, w=2, v=255):
        pts = [(cx + rx * math.cos(t * math.pi / 32), cy + ry * math.sin(t * math.pi / 32)) for t in range(65)]
        self.line(*pts, w=w, cap='butt', v=v)

    # -- formas --------------------------------------------------------------
    def rect(self, x0, y0, x1, y1, r=0, w=2, v=255):
        """Contorno de rectángulo con el trazo hacia DENTRO: (x0,y0)-(x1,y1)
        es el borde exterior, así un rect(2,2,22,20) ocupa los píxeles 2..21."""
        self.fill(x0, y0, x1, y1, r=r, v=v)
        ri = max(0, r - w)
        self.fill(x0 + w, y0 + w, x1 - w, y1 - w, r=ri, v=0 if v else 255)

    def fill(self, x0, y0, x1, y1, r=0, v=255):
        if x1 <= x0 or y1 <= y0:
            return
        self.d.rounded_rectangle((x0 * SS, y0 * SS, x1 * SS - 1, y1 * SS - 1), radius=r * SS, fill=v)

    def poly(self, *pts, v=255):
        self.d.polygon([self._p(x, y) for x, y in pts], fill=v)

    def clear(self, x0, y0, x1, y1, r=0):
        self.fill(x0, y0, x1, y1, r=r, v=0)

    def chevron(self, x, y, d, s=4, w=2):
        """Punta de flecha abierta con el vértice en (x,y) apuntando a `d`
        ('l','r','u','d')."""
        if d == 'l': self.line((x + s, y - s), (x, y), (x + s, y + s), w=w)
        if d == 'r': self.line((x - s, y - s), (x, y), (x - s, y + s), w=w)
        if d == 'u': self.line((x - s, y + s), (x, y), (x + s, y + s), w=w)
        if d == 'd': self.line((x - s, y - s), (x, y), (x + s, y - s), w=w)

    def dashed_rect(self, x0, y0, x1, y1, dash=3, gap=2, w=2):
        # Las esquinas siempre llevan trazo: un marco discontinuo sin esquinas
        # deja de leerse como rectángulo y parece una nube de guiones.
        def run(a, b, fixed, horiz):
            p, on = a, True
            while p < b:
                q = min(b, p + (dash if on else gap))
                if on:
                    if horiz: self.fill(p, fixed, q, fixed + w)
                    else:     self.fill(fixed, p, fixed + w, q)
                p, on = q, not on
        run(x0, x1, y0, True); run(x0, x1, y1 - w, True)
        run(y0, y1, x0, False); run(y0, y1, x1 - w, False)

    # -- salida --------------------------------------------------------------
    def bitmap(self):
        small = self.im.resize((self.w, self.h), Image.BOX)
        return [[small.getpixel((x, y)) >= 255 * TH for x in range(self.w)] for y in range(self.h)]


# ============================================================================
# El juego de iconos. Coordenadas sobre 24x22.
# ============================================================================
ICONS = {}

def icon(name):
    def deco(f):
        ICONS[name] = f
        return f
    return deco


# --- Edición -----------------------------------------------------------------
@icon('copy')
def _(c):
    c.rect(2, 1, 15, 15, r=3)
    c.clear(7, 6, 22, 21, r=3)
    c.rect(8, 7, 22, 21, r=3)

@icon('paste')
def _(c):
    c.rect(3, 3, 21, 22, r=3)
    c.clear(7, 0, 17, 7, r=2)
    c.fill(8, 1, 16, 6, r=2)
    c.fill(8, 11, 16, 13)
    c.fill(8, 16, 14, 18)

@icon('cut')
def _(c):
    c.line((8.5, 13.5), (17, 1))
    c.line((15.5, 13.5), (7, 1))
    c.circle(6.5, 16.5, 3.5)
    c.circle(17.5, 16.5, 3.5)

def _uturn(c, mirror=False):
    m = (lambda x: 24 - x) if mirror else (lambda x: x)
    pts = c.arc(13, 13, 6, -90, 90, w=2, cap='butt') if not mirror else c.arc(11, 13, 6, 270, 90, w=2, cap='butt')
    c.line((m(4), 7), (m(13), 7), w=2)
    c.line((m(13), 19), (m(9), 19), w=2)
    c.chevron(m(3.5), 7, 'r' if mirror else 'l', s=4.5)

@icon('undo')
def _(c):
    _uturn(c)

@icon('redo')
def _(c):
    _uturn(c, mirror=True)

@icon('select_all')
def _(c):
    c.dashed_rect(1, 1, 23, 21, dash=3, gap=2)
    c.fill(6, 6, 18, 16, r=2)

@icon('save')
def _(c):
    c.rect(3, 1, 21, 21, r=3)
    c.fill(7, 1, 15, 8, r=1)
    c.clear(12, 3, 14, 6)
    c.rect(7, 12, 17, 21, r=1)

@icon('find')
def _(c):
    c.circle(10, 9, 6.5)
    c.line((15, 14), (20.5, 19.5), w=3)

@icon('close')
def _(c):
    c.rect(1, 1, 23, 21, r=5)
    c.line((8, 7), (16, 15))
    c.line((16, 7), (8, 15))

@icon('screenshot')
def _(c):
    for (x, y, sx, sy) in ((1, 1, 1, 1), (23, 1, -1, 1), (1, 21, 1, -1), (23, 21, -1, -1)):
        c.line((x + 6 * sx, y + 1 * sy), (x + 1 * sx, y + 1 * sy), (x + 1 * sx, y + 6 * sy), cap='round')
    c.dot(12, 11, 4.5)
    c.clear(10, 9, 12, 11)

# --- Aplicaciones y sistema ---------------------------------------------------
@icon('table')
def _(c):
    c.rect(1, 2, 23, 21, r=3)
    c.fill(1, 2, 23, 8, r=3)
    c.fill(1, 6, 23, 8)
    c.fill(1, 13, 23, 15)
    c.fill(8, 2, 10, 21)
    c.fill(15, 8, 17, 21)

@icon('document')
def _(c):
    c.poly((5, 1), (15, 1), (20, 6), (20, 21), (5, 21))
    c.poly((7, 3), (14, 3), (14, 7), (18, 7), (18, 19), (7, 19), v=0)
    c.fill(9, 9, 16, 11)
    c.fill(9, 13, 16, 15)
    c.fill(9, 16, 13, 18)

@icon('globe')
def _(c):
    c.circle(12, 11, 9.5)
    c.ellipse(12, 11, 4, 9.5, w=2)
    c.fill(3, 10, 21, 12)

@icon('calculator')
def _(c):
    c.rect(4, 0, 20, 22, r=3)
    c.fill(7, 3, 17, 8, r=1)
    for y in (11, 15, 19):
        for x in (8.5, 12, 15.5):
            c.fill(x - 1, y - 1, x + 1, y + 1)

@icon('activity')
def _(c):
    c.line((1, 12), (6, 12), (9, 5), (13, 18), (16, 8), (18, 12), (23, 12))

@icon('lock')
def _(c):
    c.arc(12, 8, 5, 180, 360, w=2.5, cap='butt')
    c.fill(5.75, 8, 8.25, 11)
    c.fill(15.75, 8, 18.25, 11)
    c.fill(4, 10, 20, 22, r=3)
    c.dot(12, 15, 2, v=0)
    c.clear(11, 15, 13, 19)

@icon('play_pause')
def _(c):
    c.poly((2, 4), (2, 18), (12, 11))
    c.fill(14, 4, 17, 18, r=1)
    c.fill(19, 4, 22, 18, r=1)

@icon('play')
def _(c):
    c.poly((6, 2), (6, 20), (20, 11))

@icon('stop')
def _(c):
    c.fill(4, 3, 20, 19, r=3)

@icon('next')
def _(c):
    c.poly((3, 4), (3, 18), (14, 11))
    c.fill(16, 4, 20, 18, r=1)

@icon('prev')
def _(c):
    c.poly((21, 4), (21, 18), (10, 11))
    c.fill(4, 4, 8, 18, r=1)

# --- Dibujo -------------------------------------------------------------------
@icon('brush')
def _(c):
    c.line((21.5, 1.5), (12.5, 10.5), w=3)
    c.poly((9, 11), (13, 15), (11, 19), (6, 21.5), (2, 21.5), (4, 18), (5, 14))

@icon('eraser')
def _(c):
    # Goma inclinada: la punta de borrar, rellena; el cuerpo, en hueco.
    c.poly((1, 14), (10, 5), (20, 15), (14, 21), (8, 21))
    c.poly((4.5, 14), (10, 8.5), (16.5, 15), (13, 18.5), (9, 18.5), v=0)
    c.poly((10, 5), (14, 1), (24, 11), (20, 15))
    c.fill(15, 20, 23, 22)

def _pencil(c, x0, y0, x1, y1, wid=6):
    dx, dy = x1 - x0, y1 - y0
    n = math.hypot(dx, dy); ux, uy = dx / n, dy / n; px, py = -uy * wid / 2, ux * wid / 2
    tip = 6
    bx, by = x0 + ux * tip, y0 + uy * tip
    c.poly((x0, y0), (bx + px, by + py), (x1 + px, y1 + py), (x1 - px, y1 - py), (bx - px, by - py))
    q = 2.0
    c.poly((x0 + ux * q + px * 0.35, y0 + uy * q + py * 0.35), (bx + px * 0.6, by + py * 0.6),
           (x1 - ux * 4 + px * 0.6, y1 - uy * 4 + py * 0.6), (x1 - ux * 4 - px * 0.6, y1 - uy * 4 - py * 0.6),
           (bx - px * 0.6, by - py * 0.6), (x0 + ux * q - px * 0.35, y0 + uy * q - py * 0.35), v=0)
    c.poly((x0, y0), (x0 + ux * 2.6 + px * 0.45, y0 + uy * 2.6 + py * 0.45),
           (x0 + ux * 2.6 - px * 0.45, y0 + uy * 2.6 - py * 0.45))

@icon('pencil')
def _(c):
    # Lápiz macizo con el corte de la madera antes de la mina.
    c.line((6, 16), (19, 3), w=6, cap='butt')
    c.poly((2, 20), (3.5, 14.5), (7.5, 18.5))
    c.line((8.2, 13.8), (11.8, 17.4), w=1.4, v=0)
    c.line((15.8, 6.2), (19.4, 9.8), w=1.4, v=0)

@icon('lasso')
def _(c):
    c.ellipse(13, 8, 9, 6)
    c.line((7, 12.5), (6, 16), (8, 18.5), (5, 21.5), w=2)
    c.dot(6.5, 16, 2.2)

@icon('crop')
def _(c):
    c.line((6, 0), (6, 16), (23, 16), cap='butt', w=2.5)
    c.line((1, 6), (17, 6), (17, 22), cap='butt', w=2.5)

def _move(c):
    c.line((12, 2), (12, 20))
    c.line((3, 11), (21, 11))
    for (x, y, d) in ((12, 1, 'u'), (12, 21, 'd'), (2, 11, 'l'), (22, 11, 'r')):
        c.chevron(x, y, d, s=3.5)

@icon('move')
def _(c):
    _move(c)

def _layers(c, ox=0, oy=0, k=1.0):
    P = lambda x, y: (ox + x * k, oy + y * k)
    c.poly(P(10, 1), P(19, 6), P(10, 11), P(1, 6))
    c.line(P(1, 10.5), P(10, 15.5), P(19, 10.5), w=2)
    c.line(P(1, 15), P(10, 20), P(19, 15), w=2)

@icon('layer_new')
def _(c):
    _layers(c)
    c.dot(19, 17, 5.5, v=0)
    c.line((19, 13.5), (19, 20.5), w=2, cap='butt')
    c.line((15.5, 17), (22.5, 17), w=2, cap='butt')

@icon('duplicate')
def _(c):
    c.rect(1, 1, 14, 14, r=3)
    c.clear(6, 6, 23, 22, r=3)
    c.fill(7, 7, 23, 22, r=3)
    c.fill(14, 10, 16, 19, v=0)
    c.fill(10.5, 13.5, 19.5, 15.5, v=0)

@icon('zoom')
def _(c):
    c.circle(10, 9, 7)
    c.line((15.5, 14.5), (21, 20), w=3)
    c.fill(9, 5.5, 11, 12.5)
    c.fill(6.5, 8, 13.5, 10)

@icon('hand')
def _(c):
    for (x, y0) in ((7.5, 6), (11, 3), (14.5, 4), (18, 7)):
        c.line((x, y0), (x, 13), w=3)
    c.poly((6, 12), (19.5, 12), (19.5, 15), (16, 21), (9, 21), (3, 14.5), (5, 12.5))
    c.line((2.5, 11.5), (6.5, 15.5), w=3)

@icon('text')
def _(c):
    c.fill(4, 2, 20, 5.5, r=1)
    c.fill(10.25, 2, 13.75, 20, r=1)
    c.fill(7, 17, 17, 20, r=1)

@icon('gradient')
def _(c):
    # El degradado de verdad: tramado Bayer de izquierda a derecha. Es el
    # único icono que no se puede dibujar con trazos, y en 1 bit es la forma
    # honesta de enseñar un gris.
    c.rect(1, 1, 23, 21, r=3)
    c._gradient = (3, 3, 21, 19)

@icon('pen')
def _(c):
    # Plumilla vectorial: punta rellena con su ranura y el agujero.
    c.poly((12, 1), (19, 9), (16, 16), (8, 16), (5, 9))
    c.fill(11.2, 2, 12.8, 9, v=0)
    c.dot(12, 10, 2, v=0)
    c.fill(7, 18, 17, 22, r=1)

@icon('shapes')
def _(c):
    c.rect(1, 8, 14, 21, r=2)
    c.dot(16, 7, 7.5, v=0)
    c.dot(16, 7, 6.5)

@icon('eyedropper')
def _(c):
    c.line((4, 19), (13, 10), w=2.5)
    c.dot(3, 20, 1.6)
    c.line((10, 7), (16, 13), w=3)
    c.poly((14, 6), (17.5, 2.5), (21.5, 6.5), (18, 10))
    c.dot(19, 4, 3.2)

@icon('marquee')
def _(c):
    c.dashed_rect(1, 1, 23, 21, dash=3, gap=2)

@icon('select')
def _(c):
    c.dashed_rect(1, 1, 19, 17, dash=3, gap=2)
    c.poly((9, 7), (9, 22), (12.5, 18.5), (15, 23), (17.5, 21.5), (15, 17.5), (20, 17), v=0)
    c.poly((10, 8.5), (10, 19.5), (12.8, 16.8), (15.4, 21.4), (16.4, 20.8), (13.8, 16.2), (18, 16))

@icon('slice')
def _(c):
    # Cuchillo: hoja rellena con filo recto, mango en hueco.
    c.poly((2, 21), (14, 9), (17, 12), (8, 19))
    c.line((15.5, 7.5), (21, 2), w=4.5)
    c.line((16.5, 6.5), (20, 3), w=1.5, v=0)

@icon('path_select')
def _(c):
    c.poly((6, 1), (6, 19), (10.5, 14.5), (13.5, 21), (16.5, 19.5), (13.5, 13.5), (19.5, 13))

@icon('pointer')
def _(c):
    c.poly((6, 1), (6, 19), (10.5, 14.5), (13.5, 21), (16.5, 19.5), (13.5, 13.5), (19.5, 13))
    c.poly((8, 5.5), (8, 14.5), (10.8, 11.8), (13.6, 17.8), (14.2, 17.4), (11.6, 11.6), (15, 11.2), v=0)

# --- Electrónica (Altium) -----------------------------------------------------
@icon('wire')
def _(c):
    c.line((3, 17), (11, 17), (11, 5), (21, 5), cap='butt')
    c.dot(3, 17, 2.8)
    c.dot(21, 5, 2.8)
    c.dot(11, 11, 0.01)

@icon('port')
def _(c):
    c.poly((1, 5), (16, 5), (23, 11), (16, 17), (1, 17))
    c.poly((3.5, 7.5), (15, 7.5), (19.5, 11), (15, 14.5), (3.5, 14.5), v=0)
    c.fill(6, 10, 13, 12)

@icon('pad')
def _(c):
    c.fill(3, 2, 21, 20, r=3)
    c.dot(12, 11, 3.5, v=0)

@icon('via')
def _(c):
    c.dot(12, 11, 9)
    c.dot(12, 11, 4, v=0)

@icon('track')
def _(c):
    c.line((4, 18), (9, 18), (17, 6), (21, 6), w=3)
    for (x, y) in ((3.5, 18), (20.5, 6)):
        c.dot(x, y, 3.2)
        c.dot(x, y, 1.2, v=0)

@icon('fit')
def _(c):
    for (x, y, sx, sy) in ((1, 1, 1, 1), (23, 1, -1, 1), (1, 21, 1, -1), (23, 21, -1, -1)):
        c.line((x + 5 * sx, y + 1 * sy), (x + 1 * sx, y + 1 * sy), (x + 1 * sx, y + 5 * sy))
    c.fill(7, 6, 17, 16, r=2)

@icon('cube')
def _(c):
    c.line((12, 1.5), (20.5, 6), (20.5, 16), (12, 20.5), (3.5, 16), (3.5, 6), (12, 1.5), w=2)
    c.line((3.5, 6), (12, 10.5), (20.5, 6), w=2)
    c.line((12, 10.5), (12, 20.5), w=2)
    c.poly((12, 1.5), (20.5, 6), (12, 10.5), (3.5, 6))

@icon('schematic')
def _(c):
    # Resistencia en zigzag entre dos terminales.
    c.line((0, 11), (4, 11), (6, 4), (9.5, 18), (13, 4), (16.5, 18), (18.5, 11), (24, 11), w=2)

@icon('pcb')
def _(c):
    c.rect(1, 1, 23, 21, r=3)
    c.line((6, 16), (10, 16), (15, 9), (18, 9), w=2)
    c.dot(6, 16, 2.4)
    c.dot(18, 9, 2.4)
    c.dot(7, 7, 1.6)
    c.dot(17, 16, 1.6)

@icon('library')
def _(c):
    c.fill(2, 3, 7, 21, r=1)
    c.fill(9, 6, 14, 21, r=1)
    c.poly((15.5, 6.5), (19.5, 5), (23, 19.5), (19, 21))
    c.fill(3.5, 6, 5.5, 8, v=0)
    c.fill(10.5, 9, 12.5, 11, v=0)

@icon('chip')
def _(c):
    c.rect(5, 4, 19, 18, r=2)
    c.fill(9, 8, 15, 14, r=1)
    for k in (8, 12, 16):
        c.fill(k - 1, 0, k + 1, 4)
        c.fill(k - 1, 18, k + 1, 22)
    for k in (7, 11, 15):
        c.fill(1, k - 1, 5, k + 1)
        c.fill(19, k - 1, 23, k + 1)

@icon('grid')
def _(c):
    for y in (3, 11, 19):
        for x in (3, 9, 15, 21):
            c.fill(x - 1.5, y - 1.5, x + 1.5, y + 1.5)

@icon('ruler')
def _(c):
    c.rect(0, 6, 24, 17, r=2)
    for i, x in enumerate((5, 9, 13, 17)):
        c.fill(x - 1, 6, x + 1, 13 if i % 2 == 0 else 11)

@icon('checklist')
def _(c):
    for y in (4, 11, 18):
        c.fill(1, y - 2, 6, y + 3, r=1)
        c.fill(9, y - 0.5, 23, y + 1.5)
    c.fill(2.5, 9.5, 4.5, 11.5, v=0)

@icon('bolt')
def _(c):
    c.poly((14, 0), (4, 13), (11, 13), (9, 22), (20, 8), (13, 8), (16, 0))

@icon('check_box')
def _(c):
    c.fill(1, 1, 23, 21, r=5)
    c.line((6.5, 11), (10.5, 15), (17.5, 7), w=2.5, v=0)

@icon('gear')
def _(c):
    for k in range(8):
        a = math.radians(k * 45 + 22.5)
        c.line((12 + 6 * math.cos(a), 11 + 6 * math.sin(a)), (12 + 9.6 * math.cos(a), 11 + 9.6 * math.sin(a)), w=3.6, cap='butt')
    c.dot(12, 11, 7.5)
    c.dot(12, 11, 3, v=0)

# --- Vídeo (Premiere) ---------------------------------------------------------
@icon('razor')
def _(c):
    # Cuchilla de afeitar: la hoja con su ranura central.
    c.fill(1, 4, 23, 18, r=2)
    c.fill(5, 9.5, 19, 12.5, r=1.5, v=0)
    c.fill(0, 10, 2, 12, v=0)
    c.fill(22, 10, 24, 12, v=0)

@icon('marker')
def _(c):
    c.poly((5, 1), (19, 1), (19, 21), (12, 15), (5, 21))

@icon('link')
def _(c):
    # Dos eslabones de cadena enganchados en diagonal.
    c.line((4.5, 17.5), (10.5, 11.5), w=8)
    c.line((4.5, 17.5), (10.5, 11.5), w=4, v=0)
    c.line((13.5, 10.5), (19.5, 4.5), w=8)
    c.line((13.5, 10.5), (19.5, 4.5), w=4, v=0)
    c.line((8.5, 13.5), (15.5, 8.5), w=2.5)

@icon('ripple')
def _(c):
    c.fill(1, 6, 8, 16, r=1)
    c.fill(16, 6, 23, 16, r=1)
    c.line((14, 11), (10, 11), w=2)
    c.chevron(9.5, 11, 'l', s=3)
    c.line((12, 1), (12, 4), w=2)
    c.line((12, 18), (12, 21), w=2)

@icon('export')
def _(c):
    c.line((8, 8), (4, 8), (4, 21), (20, 21), (20, 8), (16, 8), cap='round')
    c.line((12, 15), (12, 1.5))
    c.chevron(12, 1.5, 'u', s=4)

@icon('nest')
def _(c):
    c.rect(1, 1, 23, 21, r=3)
    c.rect(5, 5, 19, 17, r=2)
    c.fill(9, 9, 15, 13, r=1)

@icon('slip')
def _(c):
    c.rect(1, 4, 23, 18, r=2)
    c.line((7, 11), (17, 11))
    c.chevron(6, 11, 'l', s=3)
    c.chevron(18, 11, 'r', s=3)

@icon('slide')
def _(c):
    c.fill(7, 5, 17, 17, r=2)
    c.line((1.5, 11), (4.5, 11), w=2)
    c.line((19.5, 11), (22.5, 11), w=2)
    c.chevron(1, 11, 'l', s=3)
    c.chevron(23, 11, 'r', s=3)

@icon('trim')
def _(c):
    c.fill(1, 5, 10, 17, r=1)
    c.line((15, 3), (13, 3), (13, 19), (15, 19), cap='butt')
    c.line((18, 11), (23, 11), w=2)
    c.chevron(17.5, 11, 'l', s=3)

@icon('rate')
def _(c):
    # Clip estirado: dos topes y flechas hacia fuera.
    c.fill(7, 6, 17, 16, r=1)
    c.fill(1, 3, 3, 19)
    c.fill(21, 3, 23, 19)
    c.chevron(4, 11, 'l', s=2.5)
    c.chevron(20, 11, 'r', s=2.5)

@icon('roll')
def _(c):
    c.fill(1, 6, 9.5, 16, r=1)
    c.fill(14.5, 6, 23, 16, r=1)
    c.fill(11, 1, 13, 21)
    c.chevron(9, 1.5, 'l', s=2.5)
    c.chevron(15, 1.5, 'r', s=2.5)

@icon('speed')
def _(c):
    c.arc(12, 14, 9.5, 180, 360, w=2.5)
    c.line((12, 14), (17.5, 7), w=2.5)
    c.dot(12, 14, 2.8)
    c.fill(2, 19, 22, 21)

# --- Perfiles de fábrica ------------------------------------------------------
@icon('briefcase')
def _(c):
    c.rect(8, 1, 16, 7, r=2)
    c.fill(1, 5, 23, 21, r=3)
    c.fill(1, 11, 23, 13, v=0)
    c.fill(10, 9.5, 14, 14.5, r=1)

@icon('film')
def _(c):
    c.fill(3, 1, 21, 21, r=2)
    for y in (3, 7, 11, 15, 19):
        c.fill(4.5, y - 1, 6.5, y + 1, v=0)
        c.fill(17.5, y - 1, 19.5, y + 1, v=0)
    c.fill(8.5, 3, 15.5, 10, v=0)
    c.fill(8.5, 12, 15.5, 19, v=0)

# --- Menús ----------------------------------------------------------------------
@icon('profiles')
def _(c):
    # Tres tarjetas apiladas: un perfil es una baraja de teclas.
    c.fill(6, 1, 18, 3, r=1)
    c.fill(3, 5, 21, 7, r=1)
    c.rect(1, 9, 23, 22, r=3)
    for x in (6, 12, 18):
        c.fill(x - 1.5, 13, x + 1.5, 16)

@icon('moon')
def _(c):
    c.dot(11, 11, 9.5)
    c.dot(16.5, 6.5, 8, v=0)
    c.fill(18, 13, 20, 15)
    c.fill(21, 9, 23, 11)

@icon('moon_off')
def _(c):
    c.dot(11, 11, 9.5)
    c.dot(16.5, 6.5, 8, v=0)
    c.line((2, 2), (22, 20), w=5, v=0)
    c.line((2, 2), (22, 20), w=2)

@icon('wheel')
def _(c):
    c.circle(12, 11, 7.5)
    for k in range(12):
        a = math.radians(k * 30)
        c.line((12 + 9.6 * math.cos(a), 11 + 9.6 * math.sin(a)), (12 + 11 * math.cos(a), 11 + 11 * math.sin(a)), w=1.6, cap='butt')
    c.fill(11, 6, 13, 11, r=1)

@icon('reset')
def _(c):
    c.arc(12, 11, 8.5, -60, 230, w=2.5)
    pts = (12 + 8.5 * math.cos(math.radians(230)), 11 + 8.5 * math.sin(math.radians(230)))
    c.poly((pts[0] - 4.5, pts[1] - 2.5), (pts[0] + 3.5, pts[1] - 3), (pts[0] - 0.5, pts[1] + 4.5))

@icon('info')
def _(c):
    c.circle(12, 11, 9.5)
    c.dot(12, 6.5, 1.7)
    c.fill(11, 9.5, 13, 16.5)
    c.fill(9.5, 9.5, 12, 11)
    c.fill(9.5, 15, 14.5, 16.5)

@icon('exit')
def _(c):
    c.line((10, 1), (3, 1), (3, 21), (10, 21), cap='butt', w=2)
    c.fill(3, 1, 5, 21)
    c.line((9, 11), (21.5, 11))
    c.chevron(22, 11, 'r', s=4)

@icon('back')
def _(c):
    c.line((15, 2), (6, 11), (15, 20), w=3)

def _timer(c, frac):
    c.circle(12, 12, 9.5)
    c.fill(10, 0, 14, 2, r=0.5)
    if frac > 0:
        a1 = -90 + 360 * frac
        pts = [(12, 12)] + [(12 + 7 * math.cos(math.radians(a)), 12 + 7 * math.sin(math.radians(a)))
                            for a in [(-90 + (a1 + 90) * i / 24) for i in range(25)]]
        c.poly(*pts)

@icon('timer1')
def _(c): _timer(c, 1 / 10)

@icon('timer5')
def _(c): _timer(c, 5 / 10)

@icon('timer10')
def _(c): _timer(c, 1.0)

@icon('trash')
def _(c):
    c.fill(2, 3, 22, 5.5, r=1)
    c.fill(9, 0, 15, 3, r=1)
    c.fill(4, 7, 20, 22, r=2)
    for x in (8.5, 12, 15.5):
        c.fill(x - 0.75, 10, x + 0.75, 19, v=0)

@icon('xmark')
def _(c):
    c.line((5, 4), (19, 18), w=3)
    c.line((19, 4), (5, 18), w=3)

@icon('pc')
def _(c):
    c.rect(1, 1, 23, 16, r=2)
    c.fill(10, 16, 14, 19)
    c.fill(6, 19, 18, 21, r=1)

@icon('pc_off')
def _(c):
    c.rect(1, 1, 23, 16, r=2)
    c.fill(10, 16, 14, 19)
    c.fill(6, 19, 18, 21, r=1)
    c.line((7, 5), (17, 13), w=2)
    c.line((17, 5), (7, 13), w=2)

@icon('calendar')
def _(c):
    c.rect(1, 3, 23, 22, r=3)
    c.fill(1, 3, 23, 9, r=3); c.fill(1, 7, 23, 9)
    c.fill(6, 0, 8, 5); c.fill(16, 0, 18, 5)
    for y in (12.5, 17):
        for x in (6, 10, 14, 18):
            c.fill(x - 1, y - 1, x + 1, y + 1)

@icon('pages')
def _(c):
    c.rect(6, 1, 23, 18, r=2)
    c.clear(0, 4, 18, 22, r=2)
    c.rect(1, 4, 18, 22, r=2)


def render(name):
    cv = Canvas()
    ICONS[name](cv)
    bm = cv.bitmap()
    g = getattr(cv, '_gradient', None)
    if g:
        x0, y0, x1, y1 = g
        B = ((0, 8, 2, 10), (12, 4, 14, 6), (3, 11, 1, 9), (15, 7, 13, 5))
        for y in range(y0, y1):
            for x in range(x0, x1):
                lvl = (x - x0 + 0.5) / (x1 - x0) * 16
                bm[y][x] = B[y & 3][x & 3] < lvl
    return bm


# ============================================================================
# Piezas de interfaz con tamaño propio (no son iconos de 24x22).
# ============================================================================
SPRITES = {}

def sprite(name, w, h):
    def deco(f):
        SPRITES[name] = (w, h, f)
        return f
    return deco

# Interruptor de iOS: píldora con el pomo a un lado. Encendido va relleno con
# el pomo en hueco; apagado, en contorno con el pomo macizo. Así se distingue
# sin color, que es justo lo que no hay.
@sprite('toggle_on', 34, 18)
def _(c):
    c.fill(0, 0, 34, 18, r=9)
    c.dot(25, 9, 6, v=0)

@sprite('toggle_off', 34, 18)
def _(c):
    c.rect(0, 0, 34, 18, r=9)
    c.dot(9, 9, 5)

# Puntos de página: el de la página activa macizo, los demás en aro. A 7 px
# se dibujan a mano: rasterizados salen torcidos un píxel.
ASCII_SPRITES = {
    'dot_on':  ['..###..', '.#####.', '#######', '#######', '#######', '.#####.', '..###..'],
    'dot_off': ['..###..', '.#...#.', '#.....#', '#.....#', '#.....#', '.#...#.', '..###..'],
}


def render_sprite(name):
    if name in ASCII_SPRITES:
        return [[ch == '#' for ch in r] for r in ASCII_SPRITES[name]]
    w, h, f = SPRITES[name]
    cv = Canvas(w, h)
    f(cv)
    return cv.bitmap()
