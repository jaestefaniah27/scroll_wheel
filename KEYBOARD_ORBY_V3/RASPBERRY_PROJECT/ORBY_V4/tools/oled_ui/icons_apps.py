# Iconos de los perfiles de la biblioteca. Mismas reglas que icons.py: lienzo
# de 24x22, trazo de 2 px sobre coordenadas enteras, siluetas rellenas donde un
# contorno no se leería. Este fichero lo importa icons.py al final.
import math
from icons import icon, ICONS


# --- Piezas que se repiten ----------------------------------------------------------
def arrow(c, a, b, s=4, w=2):
    """Flecha de a a b con la punta abierta en b."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = x1 - x0, y1 - y0
    n = math.hypot(dx, dy) or 1
    ux, uy = dx / n, dy / n
    c.line(a, b, w=w)
    c.line((x1 - s * ux - s * uy, y1 - s * uy + s * ux), (x1, y1),
           (x1 - s * ux + s * uy, y1 - s * uy - s * ux), w=w)


def window(c, x0, y0, x1, y1, r=2):
    """Ventana: marco con la barra de título rellena."""
    c.rect(x0, y0, x1, y1, r=r)
    c.fill(x0, y0, x1, y0 + 5, r=r)
    c.fill(x0, y0 + 3, x1, y0 + 5)


def doc(c, x0=4, y0=0, x1=20, y1=22, fold=5):
    """Hoja con la esquina doblada, en contorno."""
    c.poly((x0, y0), (x1 - fold, y0), (x1, y0 + fold), (x1, y1), (x0, y1))
    c.poly((x0 + 2, y0 + 2), (x1 - fold - 1, y0 + 2), (x1 - fold - 1, y0 + fold + 1),
           (x1 - 2, y0 + fold + 1), (x1 - 2, y1 - 2), (x0 + 2, y1 - 2), v=0)


def text_lines(c, x0, x1, ys, short=None, align='l'):
    """Renglones de texto de 2 px. `short` = índices de los renglones cortos."""
    for i, y in enumerate(ys):
        a, b = x0, x1
        if short and i in short:
            k = (x1 - x0) * 0.4
            if align == 'l': b = x1 - k
            elif align == 'r': a = x0 + k
            elif align == 'c': a, b = x0 + k / 2, x1 - k / 2
        c.fill(a, y, b, y + 2)


def badge(c, x, y, kind, r=5):
    """Insignia redonda abajo a la derecha: '+', 'x', '-'. Abre hueco antes."""
    c.dot(x, y, r + 1.5, v=0)
    c.dot(x, y, r)
    k = r * 0.55
    if kind == '+':
        c.fill(x - 0.9, y - k, x + 0.9, y + k, v=0)
        c.fill(x - k, y - 0.9, x + k, y + 0.9, v=0)
    elif kind == '-':
        c.fill(x - k, y - 0.9, x + k, y + 0.9, v=0)
    elif kind == 'x':
        c.line((x - k * 0.8, y - k * 0.8), (x + k * 0.8, y + k * 0.8), w=1.6, v=0)
        c.line((x + k * 0.8, y - k * 0.8), (x - k * 0.8, y + k * 0.8), w=1.6, v=0)


def screens(c):
    """Dos pantallas superpuestas: los escritorios virtuales."""
    c.rect(6, 1, 23, 13, r=2)
    c.clear(0, 5, 18, 18, r=2)
    c.rect(1, 5, 18, 17, r=2)


# ============================================================================
# Perfiles
# ============================================================================
@icon('sine')
def _(c):
    pts = [(1 + x * 0.5, 11 - 8 * math.sin((x * 0.5) / 22 * 2 * math.pi)) for x in range(45)]
    c.line(*pts, w=2.5)

@icon('code')
def _(c):
    c.line((8, 4), (2, 11), (8, 18), w=2.5)
    c.line((16, 4), (22, 11), (16, 18), w=2.5)
    c.line((14, 2), (10, 20), w=2)

@icon('windows')
def _(c):
    for (x0, y0) in ((2, 2), (13, 2), (2, 12), (13, 12)):
        c.fill(x0, y0, x0 + 9, y0 + 8)

@icon('slides')
def _(c):
    c.fill(1, 1, 23, 3, r=1)
    c.rect(3, 3, 21, 16)
    c.fill(6, 10, 9, 13); c.fill(10.5, 7, 13.5, 13); c.fill(15, 5, 18, 13)
    c.line((12, 16), (12, 19), w=2, cap='butt')
    c.line((7, 22), (12, 18), (17, 22), w=2)

# ============================================================================
# Texto y Office
# ============================================================================
@icon('align_l')
def _(c): text_lines(c, 2, 22, (3, 8, 13, 18), short=(1, 3), align='l')

@icon('align_c')
def _(c): text_lines(c, 2, 22, (3, 8, 13, 18), short=(1, 3), align='c')

@icon('align_r')
def _(c): text_lines(c, 2, 22, (3, 8, 13, 18), short=(1, 3), align='r')

@icon('align_j')
def _(c): text_lines(c, 2, 22, (3, 8, 13, 18), short=(3,), align='l')

# En el Office en español, negrita es la N, cursiva la K y subrayado la S: el
# icono enseña la misma letra que el botón de la cinta.
@icon('bold')
def _(c): c.text('N', 7, 2, big=True)

@icon('italic')
def _(c): c.text('K', 6, 2, big=True, italic=True)

@icon('underline')
def _(c):
    c.text('S', 7, 0, big=True)
    c.fill(4, 19, 20, 21)

def _heading(c, n):
    c.text('T', 1, 2, big=True)
    c.text(str(n), 13, 2, big=True)

@icon('h1')
def _(c): _heading(c, 1)

@icon('h2')
def _(c): _heading(c, 2)

@icon('h3')
def _(c): _heading(c, 3)

@icon('sigma')
def _(c):
    c.line((19, 3), (5, 3), (12.5, 11), (5, 19), (19, 19), w=2.5, cap='butt')

@icon('dollar')
def _(c):
    c.text('$', 5, 2, big=True)

@icon('clear_format')
def _(c):
    c.text('T', 4, 2, big=True)
    c.line((15, 13), (22, 20), w=4, v=0)
    c.line((16, 14), (21.5, 19.5), w=2)
    c.line((21.5, 14), (16, 19.5), w=2)

@icon('format_painter')
def _(c):
    c.fill(3, 1, 21, 9, r=2)
    c.line((21, 5), (23, 5), (23, 12), (12, 12), (12, 15), w=2, cap='butt')
    c.fill(10, 14, 14, 22, r=1)

@icon('format_cells')
def _(c):
    c.rect(1, 2, 23, 21, r=2)
    c.fill(1, 8, 23, 10); c.fill(8, 2, 10, 21)
    c.fill(10, 10, 23, 21, r=1)

@icon('fill_down')
def _(c):
    c.rect(4, 1, 20, 7, r=1)
    c.fill(4, 9, 20, 22, r=1)
    c.fill(11, 10, 13, 18, v=0)
    c.line((8, 15), (12, 19), (16, 15), w=2, v=0)

@icon('fill_right')
def _(c):
    c.rect(1, 4, 8, 18, r=1)
    c.fill(10, 4, 23, 18, r=1)
    c.fill(11, 10, 19, 12, v=0)
    c.line((16, 7), (20, 11), (16, 15), w=2, v=0)

def _grid(c):
    c.rect(1, 1, 23, 21, r=2)
    for y in (7.5, 14.5): c.fill(1, y - 1, 23, y + 1)
    for x in (8.5, 15.5): c.fill(x - 1, 1, x + 1, 21)

@icon('row_select')
def _(c):
    # Tres filas: la elegida maciza, las otras en hueco.
    c.rect(1, 0, 23, 6, r=1)
    c.fill(1, 8, 23, 14, r=1)
    c.rect(1, 16, 23, 22, r=1)

@icon('col_select')
def _(c):
    c.rect(1, 0, 7, 22, r=1)
    c.fill(9, 0, 15, 22, r=1)
    c.rect(17, 0, 23, 22, r=1)

@icon('row_plus')
def _(c):
    c.rect(1, 1, 19, 17, r=2)
    c.fill(1, 8, 19, 10); c.fill(9, 1, 11, 17)
    badge(c, 18, 16, '+')

@icon('row_minus')
def _(c):
    c.rect(1, 1, 19, 17, r=2)
    c.fill(1, 8, 19, 10); c.fill(9, 1, 11, 17)
    badge(c, 18, 16, '-')

@icon('sheet_new')
def _(c):
    c.rect(1, 1, 19, 17, r=2)
    c.fill(1, 1, 19, 6, r=2); c.fill(1, 4, 19, 6)
    c.fill(9, 6, 11, 17)
    badge(c, 18, 16, '+')

@icon('chart')
def _(c):
    c.fill(3, 12, 7, 20); c.fill(10, 6, 14, 20); c.fill(17, 2, 21, 20)
    c.fill(1, 20, 23, 22)

@icon('filter')
def _(c):
    c.poly((1, 2), (23, 2), (14, 12), (14, 20), (10, 22), (10, 12))

@icon('note')
def _(c):
    c.poly((2, 2), (22, 2), (22, 15), (15, 22), (2, 22))
    c.poly((4, 4), (20, 4), (20, 13), (13, 13), (13, 20), (4, 20), v=0)
    c.fill(6, 7, 17, 9); c.fill(6, 11, 11, 13)

@icon('return')
def _(c):
    c.line((20, 2), (20, 13), (4, 13), w=2.5, cap='butt')
    c.line((9, 7.5), (3.5, 13), (9, 18.5), w=2.5)

@icon('page_break')
def _(c):
    c.line((4, 0), (4, 7), (20, 7), (20, 0), w=2, cap='butt')
    c.line((4, 22), (4, 15), (20, 15), (20, 22), w=2, cap='butt')
    for x in (1, 7, 13, 19):
        c.fill(x, 10, x + 3, 12)

@icon('spell')
def _(c):
    c.text('ABC', 1, 0)
    c.line((5, 15), (10, 20), (22, 9), w=2.5)

@icon('book')
def _(c):
    c.poly((1, 3), (10, 3), (12, 5), (12, 21), (10, 19), (1, 19))
    c.poly((23, 3), (14, 3), (12, 5), (12, 21), (14, 19), (23, 19))
    c.fill(12, 5, 12.8, 21, v=0)
    c.fill(4, 7, 9, 8.5, v=0); c.fill(4, 11, 9, 12.5, v=0)
    c.fill(15, 7, 20, 8.5, v=0); c.fill(15, 11, 20, 12.5, v=0)

@icon('doc_new')
def _(c):
    doc(c, 2, 0, 18, 21)
    badge(c, 18, 16, '+')

@icon('folder')
def _(c):
    c.poly((1, 3), (9, 3), (11, 6), (23, 6), (23, 20), (1, 20))
    c.fill(3, 9, 21, 10.5, v=0)

@icon('files')
def _(c):
    c.rect(7, 0, 23, 16, r=2)
    c.clear(0, 4, 18, 22, r=2)
    doc(c, 1, 5, 17, 22, fold=4)

@icon('printer')
def _(c):
    c.rect(6, 0, 18, 6)
    c.fill(1, 6, 23, 16, r=2)
    c.fill(18, 9, 20, 11, v=0)
    c.rect(6, 12, 18, 22)
    c.fill(9, 15, 15, 16.5); c.fill(9, 18, 15, 19.5)

@icon('goto')
def _(c):
    c.circle(16, 11, 6.5)
    c.dot(16, 11, 2.2)
    arrow(c, (1, 11), (12, 11), s=4)

@icon('to_end')
def _(c):
    arrow(c, (12, 1), (12, 16), s=5)
    c.fill(3, 19, 21, 22, r=1)

@icon('to_start')
def _(c):
    arrow(c, (12, 21), (12, 6), s=5)
    c.fill(3, 0, 21, 3, r=1)

@icon('slide_new')
def _(c):
    c.rect(1, 2, 19, 15, r=2)
    c.fill(4, 5, 13, 7)
    c.fill(4, 9, 10, 11)
    badge(c, 18, 15, '+')

@icon('play_start')
def _(c):
    c.fill(3, 4, 7, 18, r=1)
    c.poly((9, 3), (9, 19), (21, 11))

@icon('presenter')
def _(c):
    c.rect(8, 1, 23, 13, r=1)
    c.dot(4, 6, 3)
    c.poly((0, 22), (0, 13), (3, 11), (6, 11), (11, 9), (12, 11), (7, 14), (8, 22))

@icon('screen_black')
def _(c):
    c.fill(1, 1, 23, 16, r=2)
    c.fill(10, 16, 14, 19)
    c.fill(6, 19, 18, 21, r=1)

@icon('laser')
def _(c):
    c.dot(12, 11, 3.5)
    for k in range(8):
        a = math.radians(k * 45)
        c.line((12 + 6.5 * math.cos(a), 11 + 6.5 * math.sin(a)),
               (12 + 10 * math.cos(a), 11 + 10 * math.sin(a)), w=2)

# ============================================================================
# Navegador
# ============================================================================
@icon('eye')
def _(c):
    # Almendra con dos arcos y la pupila maciza.
    c.arc(12, 16.2, 12.2, 205, 335, w=2, cap='round')
    c.arc(12, 5.8, 12.2, 25, 155, w=2, cap='round')
    c.dot(12, 11, 4.2)

@icon('eye_off')
def _(c):
    ICONS['eye'](c)
    c.line((3, 1), (21, 21), w=5, v=0)
    c.line((3, 1), (21, 21), w=2)

@icon('star')
def _(c):
    pts = []
    for k in range(10):
        r = 10.5 if k % 2 == 0 else 4.4
        a = math.radians(-90 + k * 36)
        pts.append((12 + r * math.cos(a), 11.8 + r * math.sin(a)))
    c.poly(*pts)

@icon('clock')
def _(c):
    c.circle(12, 11, 9.5)
    c.line((12, 5), (12, 11), (16.5, 14), w=2)

@icon('url')
def _(c):
    c.rect(0, 5, 24, 17, r=6)
    c.circle(6, 11, 2.6, w=1.6)
    c.fill(11, 10, 20, 12)

@icon('window_new')
def _(c):
    window(c, 1, 1, 20, 17)
    badge(c, 18, 16, '+')

@icon('incognito')
def _(c):
    c.poly((5, 8), (7, 1), (12, 3), (17, 1), (19, 8))
    c.fill(0, 8, 24, 10.5, r=1)
    c.circle(6.5, 16, 3.5)
    c.circle(17.5, 16, 3.5)
    c.line((10, 15.5), (14, 15.5), w=2)

@icon('download')
def _(c):
    arrow(c, (12, 1), (12, 14), s=5)
    c.line((2, 13), (2, 20), (22, 20), (22, 13), w=2.5, cap='butt')

@icon('fullscreen')
def _(c):
    for (x, y, sx, sy) in ((1, 1, 1, 1), (23, 1, -1, 1), (1, 21, 1, -1), (23, 21, -1, -1)):
        c.line((x + 7 * sx, y + 1 * sy), (x + 1 * sx, y + 1 * sy), (x + 1 * sx, y + 7 * sy), w=2.5)

def _skip(c, mirror, txt):
    m = (lambda x: 24 - x) if mirror else (lambda x: x)
    if not mirror:
        c.arc(12, 12, 9, 200, 470, w=2)
        c.poly((m(1.5), 4), (m(8.5), 2), (m(6), 9))
    else:
        c.arc(12, 12, 9, -290, -20, w=2)
        c.poly((m(1.5), 4), (m(8.5), 2), (m(6), 9))
    c.text(txt, 7, 8)

@icon('back10')
def _(c): _skip(c, False, '10')

@icon('fwd10')
def _(c): _skip(c, True, '10')

@icon('cc')
def _(c):
    c.fill(0, 3, 24, 19, r=3)
    c.text('CC', 6, 7, v=False)

@icon('restart')
def _(c):
    c.fill(2, 4, 6, 18, r=1)
    c.poly((21, 4), (21, 18), (8, 11))

@icon('theater')
def _(c):
    c.rect(0, 4, 24, 18, r=2)
    c.fill(3, 7, 21, 15, r=1)

@icon('mini_player')
def _(c):
    c.rect(0, 2, 24, 20, r=2)
    c.fill(11, 10, 21, 17, r=1)

@icon('zoom100')
def _(c):
    c.text('1:1', 2, 2, big=True)


# ============================================================================
# Electrónica
# ============================================================================
def _tag(c, x0, y0, x1, y1, point=5):
    """Etiqueta con punta a la derecha, en contorno."""
    m = (y0 + y1) / 2
    c.poly((x0, y0), (x1 - point, y0), (x1, m), (x1 - point, y1), (x0, y1))
    c.poly((x0 + 2, y0 + 2), (x1 - point - 0.8, y0 + 2), (x1 - 2.6, m),
           (x1 - point - 0.8, y1 - 2), (x0 + 2, y1 - 2), v=0)

@icon('label_net')
def _(c):
    # Nombre de red sobre su cable, como se ve en el esquema.
    c.text('NET', 3, 3)
    c.line((0, 16), (24, 16), w=2, cap='butt')
    c.dot(4, 16, 2.4)

@icon('label_global')
def _(c):
    _tag(c, 1, 5, 23, 17, point=6)
    c.fill(5, 10, 14, 12)

@icon('label_hier')
def _(c):
    c.poly((1, 11), (7, 5), (23, 5), (23, 17), (7, 17))
    c.poly((3.8, 11), (8, 7), (21, 7), (21, 15), (8, 15), v=0)
    c.fill(10, 10, 18, 12)

@icon('power_sym')
def _(c):
    c.fill(3, 2, 21, 5, r=1)
    c.line((12, 5), (12, 21), w=2.5, cap='butt')

@icon('ground')
def _(c):
    c.line((12, 0), (12, 9), w=2.5, cap='butt')
    c.fill(2, 9, 22, 11.5)
    c.fill(6, 14, 18, 16.5)
    c.fill(10, 19, 14, 21.5)

@icon('bus')
def _(c):
    c.fill(0, 4, 24, 8)
    for x in (5, 12, 19):
        c.line((x, 8), (x - 3, 11), (x - 3, 21), w=2, cap='butt')

@icon('junction')
def _(c):
    c.line((0, 11), (24, 11), w=2, cap='butt')
    c.line((12, 11), (12, 22), w=2, cap='butt')
    c.dot(12, 11, 4)

@icon('no_connect')
def _(c):
    c.line((0, 11), (12, 11), w=2, cap='butt')
    c.line((7, 4), (19, 16), w=2.5)
    c.line((19, 4), (7, 16), w=2.5)

@icon('annotate')
def _(c):
    c.schematic = None
    c.line((0, 15), (4, 15), (6, 10), (9, 20), (12, 10), (15, 20), (17, 15), (24, 15), w=2)
    c.text('R1', 6, 0)

@icon('hammer')
def _(c):
    c.poly((3, 6), (9, 1), (19, 4), (21, 7), (13, 7), (11, 10), (6, 10))
    c.line((11, 9), (21, 20), w=3.5)

@icon('find_similar')
def _(c):
    c.circle(9, 9, 6.5)
    c.line((14, 14), (19.5, 19.5), w=3)
    c.fill(5.5, 6.5, 12.5, 8)
    c.fill(5.5, 10, 12.5, 11.5)
    c.dot(21, 4, 2.5)

@icon('sliders')
def _(c):
    for x, k in ((5, 14), (12, 6), (19, 11)):
        c.fill(x - 1, 1, x + 1, 21)
        c.fill(x - 3.5, k - 2.5, x + 3.5, k + 2.5, r=1)

@icon('cross_select')
def _(c):
    c.rect(0, 2, 10, 14, r=1)
    c.rect(14, 8, 24, 20, r=1)
    c.line((7, 17), (17, 17), w=2)
    c.line((14, 14), (17.5, 17), (14, 20), w=2)
    c.line((17, 5), (7, 5), w=2)
    c.line((10, 2), (6.5, 5), (10, 8), w=2)

def _mirror(c, vertical):
    T = (lambda x, y: (y + 1, x - 1)) if vertical else (lambda x, y: (x, y))
    c.poly(T(1, 4), T(10, 18), T(1, 18))
    c.poly(T(23, 4), T(14, 18), T(23, 18))
    c.poly(T(20.8, 9.8), T(16.6, 16.2), T(20.8, 16.2), v=0)
    for y in range(1, 22, 4):
        c.fill(*(T(11.25, y) + T(12.75, y + 2.4)) if not vertical else (y, 10.25, y + 2.4, 11.75))

@icon('mirror_h')
def _(c): _mirror(c, False)

@icon('mirror_v')
def _(c):
    c.poly((3, 1), (21, 9), (3, 9))
    c.poly((3, 21), (21, 13), (3, 13))
    c.poly((5.2, 14.9), (13, 14.9), (5.2, 18.3), v=0)
    for x in range(0, 24, 4):
        c.fill(x, 10.25, x + 2.4, 11.75)

@icon('clear_hl')
def _(c):
    c.dot(12, 11, 9)
    c.dot(12, 11, 6.5, v=0)
    c.line((5, 18), (19, 4), w=2.5)

@icon('to_pcb')
def _(c):
    # Del esquema (zigzag) a la placa: flecha hacia la placa.
    c.line((0, 4), (3, 4), (4.5, 1), (7, 7), (9.5, 1), (11, 4), (14, 4), w=1.6)
    c.rect(9, 11, 24, 22, r=2)
    c.dot(13, 18, 1.6); c.dot(20, 15, 1.6)
    arrow(c, (3, 9), (3, 17), s=3)
    c.line((3, 17), (7, 17), w=2)

@icon('to_sch')
def _(c):
    c.rect(0, 0, 15, 11, r=2)
    c.dot(4, 7, 1.6); c.dot(11, 4, 1.6)
    c.line((10, 18), (13, 18), (14.5, 15), (17, 21), (19.5, 15), (21, 18), (24, 18), w=1.6)
    arrow(c, (20, 3), (20, 11), s=3)

@icon('diff_pair')
def _(c):
    c.line((0, 6), (8, 6), (14, 2), (24, 2), w=2)
    c.line((0, 12), (8, 12), (14, 8), (24, 8), w=2)
    c.line((0, 18), (24, 18), w=1, cap='butt')
    c.fill(0, 17, 24, 19.5)

def _layers_stack(c, hi=None):
    """Tres placas inclinadas apiladas; la elegida, maciza."""
    for i, y in enumerate((1, 8, 15)):
        pts = ((5, y), (23, y), (19, y + 6), (1, y + 6))
        if i == hi:
            c.poly(*pts)
        else:
            c.poly(*pts)
            c.poly((7.2, y + 1.6), (20.2, y + 1.6), (17.9, y + 4.4), (4.8, y + 4.4), v=0)

@icon('layers')
def _(c): _layers_stack(c)

@icon('layer_one')
def _(c): _layers_stack(c, hi=1)

@icon('layer_up')
def _(c):
    _layers_stack(c, hi=0)

@icon('layer_down')
def _(c):
    _layers_stack(c, hi=2)

@icon('rotate')
def _(c):
    c.rect(7, 8, 19, 20, r=2)
    c.arc(9, 9, 7, 180, 290, w=2)
    c.poly((7.5, 0.5), (14.5, 2.5), (9.5, 7.5))

@icon('polygon')
def _(c):
    c.poly((4, 2), (20, 2), (23, 11), (17, 20), (6, 20), (1, 12))
    c.poly((6, 7), (12, 7), (12, 13), (6, 13), v=0)

@icon('bucket')
def _(c):
    c.poly((2, 9), (11, 0), (20, 9), (11, 18))
    c.poly((5.2, 9), (11, 3.2), (16.8, 9), v=0)
    c.poly((18, 12), (21.5, 17), (20, 20), (16.5, 20), (15, 17))

@icon('bucket_off')
def _(c):
    # Una barra a 45° se perdería: corre paralela a los lados del bote.
    ICONS['bucket'](c)
    badge(c, 18, 16, 'x')

@icon('route_mode')
def _(c):
    c.dot(12, 13, 4.5)
    c.line((0, 6), (6, 6), (9, 3), (15, 3), (18, 6), (24, 6), w=2)
    c.line((0, 20), (24, 20), w=2, cap='butt')

@icon('track_width')
def _(c):
    c.fill(0, 7, 24, 15)
    c.line((12, 0), (12, 5), w=2, cap='butt')
    c.line((12, 17), (12, 22), w=2, cap='butt')
    c.line((9, 2), (12, 5), (15, 2), w=2)
    c.line((9, 20), (12, 17), (15, 20), w=2)

@icon('via_size')
def _(c):
    c.dot(11, 11, 7)
    c.dot(11, 11, 2.5, v=0)
    arrow(c, (16, 6), (22, 0.5), s=3.5)
    arrow(c, (6, 16), (0.5, 21.5), s=3.5)

@icon('flip')
def _(c):
    # Placa por las dos caras: arriba maciza, abajo en hueco, y el giro.
    c.fill(1, 2, 17, 7, r=1)
    c.rect(1, 15, 17, 20, r=1)
    c.arc(17, 11, 6, -80, 80, w=2, cap='butt')
    c.poly((13, 14), (19, 14.5), (15, 20))

@icon('tuning')
def _(c):
    c.line((0, 18), (3, 18), (3, 4), (8, 4), (8, 18), (13, 18), (13, 4), (18, 4), (18, 18), (21, 18), (24, 18), w=2)

@icon('view2d')
def _(c):
    c.text('2D', 1, 2, big=True)

@icon('units')
def _(c):
    c.rect(0, 12, 24, 22, r=1)
    for x in (5, 9, 13, 17):
        c.fill(x - 1, 12, x + 1, 16)
    c.text('mm', 2, 0)

@icon('teardrop')
def _(c):
    c.poly((2, 11), (13, 4), (13, 18))
    c.dot(16, 11, 7)
    c.dot(16, 11, 2.5, v=0)

@icon('pdf')
def _(c):
    c.fill(2, 0, 22, 22, r=2)
    c.text('PDF', 3, 7, v=False)

def _tab(c, kind):
    c.line((0, 20), (3, 20), (3, 4), (16, 4), (16, 20), (24, 20), w=2, cap='butt')
    if kind == 'next':
        arrow(c, (6, 12), (14, 12), s=3.5)
    elif kind == 'prev':
        arrow(c, (14, 12), (6, 12), s=3.5)

@icon('tab_prev')
def _(c): _tab(c, 'prev')

@icon('tab_next')
def _(c): _tab(c, 'next')

@icon('footprint')
def _(c):
    for y in (2, 9, 16):
        c.fill(1, y, 7, y + 4, r=1)
        c.fill(17, y, 23, y + 4, r=1)
    c.rect(9, 0, 15, 22, r=1)

@icon('drag')
def _(c):
    c.fill(2, 9, 12, 19, r=2)
    arrow(c, (14, 9), (22, 1), s=4)

@icon('sel_conn')
def _(c):
    c.line((1, 18), (8, 18), (14, 8), (23, 8), w=3)
    c.dot(2, 18, 2.5); c.dot(22, 8, 2.5)
    c.dashed_rect(0, 0, 24, 22, dash=2, gap=2, w=1)

@icon('contrast')
def _(c):
    c.circle(12, 11, 9.5)
    c.poly(*[(12 + 9 * math.cos(math.radians(a)), 11 + 9 * math.sin(math.radians(a))) for a in range(-90, 91, 10)])

@icon('repeat')
def _(c):
    c.arc(12, 11, 8, 200, 340, w=2)
    c.arc(12, 11, 8, 20, 160, w=2)
    c.poly((17, 0.5), (22, 6), (15, 7))
    c.poly((7, 21.5), (2, 16), (9, 15))

@icon('capacitor')
def _(c):
    c.line((0, 11), (9, 11), w=2, cap='butt')
    c.line((15, 11), (24, 11), w=2, cap='butt')
    c.fill(8.5, 3, 11, 19)
    c.fill(13, 3, 15.5, 19)

@icon('inductor')
def _(c):
    # Cuatro espiras: jorobas de 6 px, que con menos se leen como una línea.
    pts = [(x * 0.25, 16 - (6 * abs(math.sin(math.pi * (x * 0.25 - 2) / 5)) if 2 <= x * 0.25 <= 22 else 0))
           for x in range(97)]
    c.line(*pts, w=2)

@icon('diode')
def _(c):
    c.line((0, 11), (24, 11), w=2, cap='butt')
    c.poly((6, 3), (6, 19), (16, 11))
    c.fill(16, 3, 18.5, 19)

@icon('spice')
def _(c):
    c.text('.op', 1, 0, big=True)

# ============================================================================
# CAD
# ============================================================================
@icon('line_tool')
def _(c):
    c.line((4, 18), (20, 4), w=2)
    c.rect(1, 15, 7, 21)
    c.rect(17, 1, 23, 7)

@icon('rect_tool')
def _(c):
    c.rect(3, 4, 21, 18)
    c.fill(0, 1, 6, 7); c.fill(18, 15, 24, 21)

@icon('circle_tool')
def _(c):
    c.circle(12, 11, 9.5)
    c.dot(12, 11, 2.2)

@icon('dimension')
def _(c):
    c.fill(1, 6, 3, 22); c.fill(21, 6, 23, 22)
    c.line((4, 14), (20, 14), w=2, cap='butt')
    c.poly((3, 14), (8, 10.5), (8, 17.5))
    c.poly((21, 14), (16, 10.5), (16, 17.5))
    c.text('10', 7, 1)

@icon('trim_sketch')
def _(c):
    c.line((0, 11), (24, 11), w=2, cap='butt')
    c.fill(15, 9, 24, 13, v=0)
    for x in (16, 20):
        c.fill(x, 10, x + 2, 12)
    c.line((12, 2), (12, 20), w=2, cap='butt')

@icon('offset')
def _(c):
    c.arc(22, 22, 12, 180, 270, w=2)
    c.arc(22, 22, 19, 180, 270, w=2)

@icon('construction')
def _(c):
    for k in range(5):
        x = 1 + k * 5
        c.line((x, 20 - k * 4.2), (x + 2.6, 20 - k * 4.2 - 2.2), w=2)
    c.dot(2, 20, 1.5); c.dot(22, 2, 1.5)

@icon('project_down')
def _(c):
    # Una arista (arriba) cae proyectada sobre el plano del boceto (abajo).
    c.line((4, 3), (20, 3), w=2.5)
    arrow(c, (12, 5), (12, 13), s=3)
    c.poly((5, 15), (23, 15), (19, 22), (1, 22))
    c.poly((7, 17), (20, 17), (18, 20), (4, 20), v=0)
    c.fill(5, 17.5, 19, 19.5)

@icon('extrude')
def _(c):
    c.poly((2, 17), (12, 13), (22, 17), (12, 21))
    c.line((2, 6), (12, 2), (22, 6), (12, 10), (2, 6), w=2)
    c.line((2, 6), (2, 17), w=2, cap='butt')
    c.line((22, 6), (22, 17), w=2, cap='butt')
    c.line((12, 10), (12, 21), w=2, cap='butt')

@icon('press_pull')
def _(c):
    c.fill(5, 9, 19, 21, r=1)
    arrow(c, (12, 16), (12, 1), s=4)
    c.fill(11, 9, 13, 16, v=0)
    c.fill(11, 10, 13, 15, v=0)

@icon('fillet')
def _(c):
    c.line((2, 1), (2, 12), w=2.5, cap='butt')
    c.arc(12, 11, 10, 90, 180, w=2.5, cap='butt')
    c.line((12, 21), (23, 21), w=2.5, cap='butt')
    c.dashed_rect(2, 1, 23, 22, dash=2, gap=2, w=1)

@icon('hole')
def _(c):
    c.ellipse(12, 6, 9, 4, w=2)
    c.line((3, 6), (3, 16), w=2, cap='butt')
    c.line((21, 6), (21, 16), w=2, cap='butt')
    c.arc(12, 16, 9, 0, 180, w=2, cap='butt')
    c.poly(*[(12 + 6 * math.cos(t * math.pi / 12), 6 + 2.2 * math.sin(t * math.pi / 12)) for t in range(24)])

@icon('joint')
def _(c):
    c.fill(1, 2, 10, 20, r=2)
    c.fill(14, 2, 23, 20, r=2)
    c.dot(12, 11, 4.5)
    c.dot(12, 11, 1.8, v=0)

@icon('command')
def _(c):
    c.rect(0, 2, 24, 20, r=3)
    c.line((5, 7), (9, 11), (5, 15), w=2)
    c.fill(11, 14, 19, 16)

@icon('palette')
def _(c):
    c.dot(12, 11, 10)
    c.dot(17, 15, 3.2, v=0)
    for (x, y) in ((7, 7), (12, 5), (17, 7), (6, 13)):
        c.dot(x, y, 1.7, v=0)


# ============================================================================
# Programación
# ============================================================================
@icon('arrow_left')
def _(c): arrow(c, (21, 11), (3, 11), s=7, w=2.5)

@icon('arrow_right')
def _(c): arrow(c, (3, 11), (21, 11), s=7, w=2.5)

@icon('braces')
def _(c):
    c.text('{}', 5, 2, big=True)

@icon('breakpoint')
def _(c):
    c.dot(12, 11, 8)
    c.fill(0, 10, 3, 12); c.fill(21, 10, 24, 12)

@icon('bug')
def _(c):
    c.dot(12, 13, 7)
    c.dot(12, 5, 3.5)
    c.fill(11.25, 7, 12.75, 21, v=0)
    for (y, dy) in ((9, -3), (13, 0), (17, 3)):
        c.line((5, y), (1, y + dy), w=2)
        c.line((19, y), (23, y + dy), w=2)
    c.line((10, 2), (8, 0), w=1.6); c.line((14, 2), (16, 0), w=1.6)

@icon('bulb')
def _(c):
    c.dot(12, 8.5, 7.5)
    c.poly((7, 12), (17, 12), (15, 16), (9, 16))
    c.fill(8.5, 17, 15.5, 19, r=1)
    c.fill(10, 20, 14, 22, r=1)
    c.arc(12, 8.5, 4.5, 200, 260, w=1.5, v=0)

@icon('comment')
def _(c):
    c.poly((1, 2), (23, 2), (23, 16), (11, 16), (5, 21), (6, 16), (1, 16))
    c.fill(5, 6, 19, 8, v=0); c.fill(5, 10, 14, 12, v=0)

@icon('del_line')
def _(c):
    c.fill(1, 3, 23, 5); c.fill(1, 17, 23, 19)
    c.fill(1, 10, 13, 12)
    badge(c, 18, 11, 'x', r=4.5)

@icon('dup_line')
def _(c):
    c.fill(1, 3, 16, 5)
    c.fill(1, 10, 16, 12)
    c.fill(1, 17, 10, 19)
    badge(c, 18, 15, '+', r=4.5)

@icon('line_up')
def _(c):
    c.fill(1, 15, 13, 18)
    c.fill(1, 8, 13, 11)
    arrow(c, (19, 20), (19, 3), s=4)

@icon('line_down')
def _(c):
    c.fill(1, 4, 13, 7)
    c.fill(1, 11, 13, 14)
    arrow(c, (19, 2), (19, 19), s=4)

@icon('multi_cursor')
def _(c):
    for x in (5, 12, 19):
        c.fill(x - 1, 3, x + 1, 19)
        c.fill(x - 3, 3, x + 3, 5)
        c.fill(x - 3, 17, x + 3, 19)

@icon('extensions')
def _(c):
    c.fill(1, 9, 9, 17, r=1); c.fill(10, 9, 18, 17, r=1)
    c.fill(1, 18, 9, 22, r=1); c.fill(10, 18, 18, 22, r=1)
    c.fill(14, 1, 22, 9, r=1)
    c.fill(1, 8, 18, 9, v=0)

@icon('file_search')
def _(c):
    doc(c, 1, 0, 16, 21)
    c.circle(16, 14, 4)
    c.line((19, 17), (22.5, 20.5), w=2.5)

@icon('find_all')
def _(c):
    c.circle(13, 9, 6.5)
    c.line((18, 14), (22.5, 18.5), w=3)
    c.fill(0, 4, 4, 6); c.fill(0, 9, 4, 11); c.fill(0, 14, 4, 16); c.fill(0, 19, 13, 21)

@icon('fold')
def _(c):
    c.line((5, 1), (12, 7), (19, 1), w=2.5)
    c.fill(1, 10, 23, 12)
    c.line((5, 21), (12, 15), (19, 21), w=2.5)

@icon('unfold')
def _(c):
    c.line((5, 7), (12, 1), (19, 7), w=2.5)
    c.fill(1, 10, 23, 12)
    c.line((5, 15), (12, 21), (19, 15), w=2.5)

@icon('format_code')
def _(c):
    c.fill(1, 2, 15, 4)
    c.fill(5, 7, 19, 9)
    c.fill(5, 12, 23, 14)
    c.fill(1, 17, 11, 19)
    c.line((1, 7), (1, 14), w=2, cap='butt')

@icon('git')
def _(c):
    c.dot(6, 4, 3); c.dot(6, 18, 3); c.dot(18, 8, 3)
    c.line((6, 4), (6, 18), w=2)
    c.line((18, 8), (18, 11), (6, 15), w=2)

@icon('goto_def')
def _(c):
    c.rect(9, 3, 24, 19, r=2)
    c.fill(13, 8, 20, 10); c.fill(13, 12, 18, 14)
    arrow(c, (0, 11), (10, 11), s=4)

@icon('refs')
def _(c):
    for y in (3, 11, 19):
        c.dot(3, y, 2.5)
        c.fill(8, y - 1, 23, y + 1)

@icon('rename')
def _(c):
    c.rect(0, 5, 24, 17, r=2)
    c.fill(4, 10, 11, 12)
    c.fill(15, 2, 17, 20)
    c.fill(13, 2, 19, 4); c.fill(13, 18, 19, 20)

@icon('replace')
def _(c):
    c.text('a', 0, 0)
    c.text('b', 17, 11)
    c.arc(12, 11, 7, 200, 330, w=2)
    c.poly((19, 4), (21, 10), (15, 9))

@icon('panel')
def _(c):
    c.rect(1, 1, 23, 21, r=2)
    c.fill(1, 12, 23, 21, r=2); c.fill(1, 12, 23, 15)
    c.line((5, 15), (8, 17.5), (5, 20), w=1.6, v=0)

@icon('sidebar')
def _(c):
    c.rect(1, 1, 23, 21, r=2)
    c.fill(1, 1, 9, 21, r=2); c.fill(5, 1, 9, 21)

@icon('warning')
def _(c):
    c.poly((12, 0), (24, 21), (0, 21))
    c.fill(11, 6, 13, 14, v=0)
    c.fill(11, 16, 13, 18, v=0)

@icon('tab_close')
def _(c):
    c.line((0, 20), (3, 20), (3, 4), (16, 4), (16, 20), (24, 20), w=2, cap='butt')
    c.line((6.5, 8.5), (12.5, 14.5), w=2)
    c.line((12.5, 8.5), (6.5, 14.5), w=2)

@icon('tab_new')
def _(c):
    c.line((0, 20), (3, 20), (3, 4), (16, 4), (16, 20), (24, 20), w=2, cap='butt')
    c.fill(8.5, 7.5, 10.5, 16.5); c.fill(5, 11, 14, 13)

@icon('tab_undo')
def _(c):
    c.line((0, 20), (3, 20), (3, 4), (16, 4), (16, 20), (24, 20), w=2, cap='butt')
    c.arc(10, 12, 3.5, -90, 120, w=2)
    c.poly((5, 8.5), (10, 6), (10, 11))

@icon('zen')
def _(c):
    c.circle(12, 11, 9.5)
    c.dot(12, 11, 3)

@icon('close_all')
def _(c):
    c.rect(6, 0, 24, 15, r=2)
    c.clear(0, 4, 19, 22, r=2)
    c.rect(0, 5, 18, 22, r=3)
    c.line((5, 10), (13, 17), w=2); c.line((13, 10), (5, 17), w=2)

@icon('save_all')
def _(c):
    c.rect(6, 0, 24, 16, r=2)
    c.clear(0, 4, 19, 22, r=2)
    ICONS['save'](_Shift(c, -2, 1, 0.82))

@icon('save_as')
def _(c):
    ICONS['save'](c)
    c.line((14, 21), (23, 12), w=6, v=0)
    c.line((15, 20), (22, 13), w=3)
    c.poly((13, 22), (13, 19), (16, 22))

@icon('step_over')
def _(c):
    c.arc(12, 13, 8, 200, 340, w=2)
    c.poly((20, 6), (22, 13), (15.5, 12))
    c.dot(12, 18, 3)

@icon('step_in')
def _(c):
    arrow(c, (12, 1), (12, 12), s=4)
    c.dot(12, 18, 3)

@icon('step_out')
def _(c):
    arrow(c, (12, 13), (12, 1), s=4)
    c.dot(12, 18, 3)

@icon('wrap')
def _(c):
    c.fill(1, 3, 23, 5)
    c.line((1, 11), (19, 11), w=2, cap='butt')
    c.arc(19, 15, 4, -90, 90, w=2)
    c.line((19, 19), (10, 19), w=2)
    c.line((13, 16), (10, 19), (13, 22), w=2)


class _Shift:
    """Dibuja un icono existente desplazado y encogido: para combinarlos."""
    def __init__(self, c, dx, dy, k):
        self.c, self.dx, self.dy, self.k = c, dx, dy, k

    def _p(self, x, y):
        return (self.dx + x * self.k, self.dy + y * self.k)

    def rect(self, x0, y0, x1, y1, r=0, w=2, v=255):
        (a, b), (e, f) = self._p(x0, y0), self._p(x1, y1)
        self.c.rect(a, b, e, f, r=r * self.k, w=w, v=v)

    def fill(self, x0, y0, x1, y1, r=0, v=255):
        (a, b), (e, f) = self._p(x0, y0), self._p(x1, y1)
        self.c.fill(a, b, e, f, r=r * self.k, v=v)

    def clear(self, x0, y0, x1, y1, r=0):
        self.fill(x0, y0, x1, y1, r=r, v=0)


# ============================================================================
# Windows
# ============================================================================
@icon('desktop')
def _(c):
    c.rect(1, 1, 23, 16, r=2)
    c.fill(10, 16, 14, 19)
    c.fill(6, 19, 18, 21, r=1)
    c.fill(4, 12, 20, 14)

@icon('task_view')
def _(c):
    c.rect(0, 2, 10, 10, r=1); c.rect(14, 2, 24, 10, r=1)
    c.rect(0, 13, 10, 21, r=1); c.rect(14, 13, 24, 21, r=1)

@icon('desk_left')
def _(c):
    screens(c)
    arrow(c, (14, 11), (5, 11), s=3)

@icon('desk_right')
def _(c):
    screens(c)
    arrow(c, (5, 11), (14, 11), s=3)

@icon('desk_new')
def _(c):
    screens(c)
    c.fill(8.5, 7.5, 10.5, 14.5); c.fill(6, 10, 13, 12)

@icon('desk_close')
def _(c):
    screens(c)
    c.line((6.5, 8), (12.5, 14), w=2); c.line((12.5, 8), (6.5, 14), w=2)

def _win_half(c, side):
    c.rect(0, 2, 24, 20, r=2)
    if side == 'l': c.fill(0, 2, 12, 20, r=2); c.fill(8, 2, 12, 20)
    elif side == 'r': c.fill(12, 2, 24, 20, r=2); c.fill(12, 2, 16, 20)

@icon('snap_left')
def _(c): _win_half(c, 'l')

@icon('snap_right')
def _(c): _win_half(c, 'r')

@icon('maximize')
def _(c):
    window(c, 0, 1, 24, 21)

@icon('minimize')
def _(c):
    c.rect(4, 0, 20, 12, r=2)
    arrow(c, (12, 4), (12, 13), s=3)
    c.fill(2, 18, 22, 21, r=1)

@icon('monitors')
def _(c):
    c.rect(0, 2, 11, 12, r=1)
    c.rect(13, 2, 24, 12, r=1)
    c.fill(4, 12, 7, 14); c.fill(17, 12, 20, 14)
    arrow(c, (4, 18), (20, 18), s=3)

@icon('clipboard')
def _(c):
    c.rect(3, 3, 21, 22, r=3)
    c.clear(7, 0, 17, 7, r=2)
    c.fill(8, 1, 16, 6, r=2)
    c.fill(7, 10, 17, 12); c.fill(7, 14, 17, 16); c.fill(7, 18, 13, 19.5)

@icon('smile')
def _(c):
    c.circle(12, 11, 9.5)
    c.dot(8.5, 8.5, 1.8); c.dot(15.5, 8.5, 1.8)
    c.arc(12, 11, 5.5, 25, 155, w=2)

@icon('speaker_off')
def _(c):
    c.poly((1, 8), (6, 8), (12, 2), (12, 20), (6, 14), (1, 14))
    c.line((15, 7), (22, 14), w=2.2)
    c.line((22, 7), (15, 14), w=2.2)

@icon('mic')
def _(c):
    c.fill(8, 0, 16, 14, r=4)
    c.arc(12, 9, 7, 0, 180, w=2, cap='butt')
    c.fill(11, 16, 13, 20)
    c.fill(7, 20, 17, 22, r=1)

@icon('mic_off')
def _(c):
    ICONS['mic'](c)
    c.line((3, 1), (21, 21), w=5, v=0)
    c.line((3, 1), (21, 21), w=2)

@icon('mail')
def _(c):
    c.rect(0, 3, 24, 19, r=2)
    c.line((2, 5), (12, 13), (22, 5), w=2)

@icon('record')
def _(c):
    c.circle(12, 11, 9.5)
    c.dot(12, 11, 5.5)

@icon('gamepad')
def _(c):
    c.fill(1, 5, 23, 17, r=6)
    c.fill(5.5, 9.5, 11.5, 11.5, v=0); c.fill(7.5, 7.5, 9.5, 13.5, v=0)
    c.dot(16, 9, 1.5, v=0); c.dot(19, 12, 1.5, v=0)

@icon('bell')
def _(c):
    c.poly((5, 16), (6, 8), (9, 4), (12, 3), (15, 4), (18, 8), (19, 16), (21, 18), (3, 18))
    c.fill(11, 0, 13, 3)
    c.dot(12, 20, 2.3)

@icon('widgets')
def _(c):
    c.fill(1, 1, 11, 11, r=2); c.fill(13, 1, 23, 11, r=5)
    c.fill(1, 13, 23, 21, r=2)

@icon('run_box')
def _(c):
    window(c, 0, 1, 24, 21)
    c.line((4, 10), (8, 13.5), (4, 17), w=2)
    c.fill(10, 16, 17, 18)


# ============================================================================
# Photoshop
# ============================================================================
@icon('balance')
def _(c):
    c.fill(11, 1, 13, 20)
    c.fill(6, 20, 18, 22, r=1)
    c.fill(2, 4, 22, 6)
    c.poly((0, 13), (4, 6), (8, 13)); c.poly((16, 13), (20, 6), (24, 13))
    c.fill(0, 13, 8, 15, r=1); c.fill(16, 13, 24, 15, r=1)

@icon('bandage')
def _(c):
    c.line((4, 18), (20, 4), w=10)
    c.fill(9, 8, 15, 14, v=0)
    for (x, y) in ((11, 10), (13, 10), (11, 12), (13, 12)):
        c.dot(x, y, 0.6)

@icon('camera')
def _(c):
    c.fill(1, 6, 23, 20, r=2)
    c.fill(7, 3, 15, 6, r=1)
    c.dot(12, 13, 5, v=0)
    c.dot(12, 13, 3)

@icon('canvas')
def _(c):
    c.rect(6, 6, 18, 16, r=1)
    for (x, y, sx, sy) in ((0, 0, 1, 1), (24, 0, -1, 1), (0, 22, 1, -1), (24, 22, -1, -1)):
        c.line((x + 5 * sx, y + 1 * sy), (x + 1 * sx, y + 1 * sy), (x + 1 * sx, y + 5 * sy), w=2)

@icon('clip_mask')
def _(c):
    c.rect(4, 0, 24, 9, r=2)
    c.fill(4, 13, 24, 22, r=2)
    c.line((1, 4), (1, 17), w=2, cap='butt')
    c.line((1, 17), (3, 17), w=2, cap='butt')
    c.poly((0, 14), (3, 17), (0, 20))

@icon('colors_default')
def _(c):
    c.fill(1, 1, 15, 15)
    c.rect(9, 7, 23, 21)
    c.fill(11, 9, 21, 19, v=0)

@icon('swap')
def _(c):
    # Flecha curva de dos puntas, como el botón de la paleta de colores.
    c.arc(4, 18, 13, -90, 0, w=2.5)
    c.poly((2, 5), (8, 1), (8, 9))
    c.poly((17, 16), (21, 22), (25, 16))
@icon('curves')
def _(c):
    c.rect(0, 0, 24, 22, r=2)
    pts = [(2 + t, 20 - 18 * (0.5 - 0.5 * math.cos(math.pi * t / 20))) for t in range(21)]
    c.line(*pts, w=2)

@icon('levels')
def _(c):
    for x, h in ((1, 5), (4, 9), (7, 14), (10, 18), (13, 12), (16, 8), (19, 11), (22, 4)):
        c.fill(x, 22 - h, x + 2.5, 22)

@icon('huesat')
def _(c):
    c.poly((12, 0), (19, 10), (19.5, 14), (17, 19), (12, 21), (7, 19), (4.5, 14), (5, 10))
    c.poly((9, 12), (9, 16), (12, 18), v=0)

@icon('image_size')
def _(c):
    c.rect(0, 0, 17, 15, r=1)
    c.poly((3, 12), (7, 6), (10, 10), (12, 8), (14, 12))
    arrow(c, (14, 13), (23, 22), s=4)

@icon('invert')
def _(c):
    c.rect(1, 1, 23, 21, r=3)
    c.poly((1, 21), (23, 1), (23, 21))
    c.fill(1, 1, 23, 21, r=3, v=0) if False else None
    c.dot(8, 8, 3)
    c.dot(16, 14, 3, v=0)

@icon('magic')
def _(c):
    c.line((2, 21), (14, 9), w=3.5)
    for (x, y, s) in ((18, 4, 4), (9, 3, 2.5), (21, 13, 2.5)):
        c.fill(x - 0.9, y - s, x + 0.9, y + s)
        c.fill(x - s, y - 0.9, x + s, y + 0.9)

@icon('merge')
def _(c):
    c.line((3, 1), (12, 10), (21, 1), w=2.5)
    arrow(c, (12, 10), (12, 21), s=4, w=2.5)

@icon('quick_mask')
def _(c):
    c.fill(1, 1, 23, 21, r=2)
    c.dot(12, 11, 6, v=0)
    c.dot(12, 11, 3.5)

@icon('select_inverse')
def _(c):
    c.fill(0, 0, 24, 22, r=2)
    c.fill(6, 5, 18, 17, v=0)
    c.dashed_rect(6, 5, 18, 17, dash=2, gap=2, w=1.5)

@icon('deselect')
def _(c):
    c.dashed_rect(1, 1, 19, 17, dash=3, gap=2)
    badge(c, 18, 16, 'x')

@icon('stamp')
def _(c):
    c.dot(12, 4.5, 4)
    c.fill(10, 7, 14, 13)
    c.fill(3, 12, 21, 17, r=2)
    c.fill(3, 19, 21, 21)

@icon('dodge')
def _(c):
    c.circle(10, 9, 7.5)
    c.line((15.5, 14.5), (22, 21), w=3)

@icon('transform')
def _(c):
    c.rect(3, 3, 21, 19, w=1.5)
    for (x, y) in ((3, 3), (21, 3), (3, 19), (21, 19), (12, 3), (12, 19), (3, 11), (21, 11)):
        c.fill(x - 2.5, y - 2.5, x + 2.5, y + 2.5)


# ============================================================================
# Premiere
# ============================================================================
def _clip(c, x0, x1, y0=6, y1=16):
    c.fill(x0, y0, x1, y1, r=1)

@icon('add_edit')
def _(c):
    _clip(c, 0, 24)
    c.fill(11, 2, 13, 20, v=0)
    c.fill(11.25, 0, 12.75, 22)

@icon('trim_prev')
def _(c):
    _clip(c, 0, 10)
    c.rect(14, 6, 24, 16, r=1)
    c.fill(11, 0, 13, 22)
    arrow(c, (22, 11), (16, 11), s=2.5, w=1.6)

@icon('trim_next')
def _(c):
    c.rect(0, 6, 10, 16, r=1)
    _clip(c, 14, 24)
    c.fill(11, 0, 13, 22)
    arrow(c, (2, 11), (8, 11), s=2.5, w=1.6)

@icon('mark_in')
def _(c):
    c.line((14, 2), (6, 2), (6, 20), (14, 20), w=3, cap='butt')
    c.fill(10, 9, 22, 13)

@icon('mark_out')
def _(c):
    c.line((10, 2), (18, 2), (18, 20), (10, 20), w=3, cap='butt')
    c.fill(2, 9, 14, 13)

@icon('mark_clip')
def _(c):
    c.line((6, 2), (1, 2), (1, 20), (6, 20), w=2.5, cap='butt')
    c.line((18, 2), (23, 2), (23, 20), (18, 20), w=2.5, cap='butt')
    _clip(c, 5, 19, 7, 15)

@icon('edit_prev')
def _(c):
    c.fill(3, 2, 6, 20)
    c.poly((20, 4), (20, 18), (8, 11))

@icon('edit_next')
def _(c):
    c.fill(18, 2, 21, 20)
    c.poly((4, 4), (4, 18), (16, 11))

@icon('match_frame')
def _(c):
    c.rect(0, 3, 11, 19, r=1)
    c.fill(13, 3, 24, 19, r=1)
    c.fill(3, 9, 8, 10.5); c.fill(3, 12, 8, 13.5)
    c.fill(16, 9, 21, 10.5, v=0); c.fill(16, 12, 21, 13.5, v=0)

@icon('magnet')
def _(c):
    c.arc(12, 10, 7.5, 0, 180, w=5, cap='butt')
    c.fill(2, 0, 7, 10); c.fill(17, 0, 22, 10)
    c.fill(2, 3, 7, 5, v=0); c.fill(17, 3, 22, 5, v=0)

@icon('render')
def _(c):
    c.rect(0, 2, 24, 20, r=2)
    c.fill(3, 13, 21, 17, r=1)
    c.fill(12, 14, 20, 16, v=0)
    c.poly((8, 5), (8, 11), (14, 8))

@icon('ripple_edit')
def _(c):
    _clip(c, 0, 9)
    c.line((14, 2), (12, 2), (12, 20), (14, 20), w=2, cap='butt')
    c.rect(15, 6, 24, 16, r=1)
    c.poly((11, 11), (6, 7.5), (6, 14.5), v=0)

@icon('track_select')
def _(c):
    arrow(c, (2, 4), (12, 4), s=3)
    arrow(c, (2, 11), (12, 11), s=3)
    arrow(c, (2, 18), (12, 18), s=3)
    c.fill(15, 1, 24, 7, r=1); c.fill(15, 8, 24, 14, r=1); c.fill(15, 15, 24, 21, r=1)

@icon('crossfade')
def _(c):
    c.poly((0, 20), (24, 2), (24, 20))
    c.poly((0, 2), (24, 20), (0, 20))
    c.poly((3, 18), (21, 18), (12, 11.3), v=0)
