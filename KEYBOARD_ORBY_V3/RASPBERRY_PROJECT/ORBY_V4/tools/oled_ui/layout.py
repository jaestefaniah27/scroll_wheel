# Composición de una pantalla de 72x40, en Python. Es el modelo del que copia
# include/oled_ui.h: las dos tienen que colocar todo en el mismo píxel, y
# tools/oled_ui/build.py lo comprueba comparando sus salidas.
import font, icons

W, H = 72, 40

# Icono arriba, nombre debajo. El icono empieza en la fila 2 y el nombre en
# la 28: deja 2 px de aire arriba y 3 abajo (la fila de los rabos queda libre
# para la g, la p y la j).
ICON_X = (W - icons.IW) // 2
ICON_Y = 2
CAP_Y = 28

def blank():
    return [[False] * W for _ in range(H)]

def blit(scr, bm, ox, oy):
    for y, row in enumerate(bm):
        for x, v in enumerate(row):
            if v and 0 <= ox + x < W and 0 <= oy + y < H:
                scr[oy + y][ox + x] = True

def text_rows(s, big=False, gap=None):
    """`gap` es el aire entre letras en grande: 2 de serie, 1 en la versión
    apretada. EPX no cruza la columna de aire, así que cada letra se puede
    pasar por EPX suelta y juntarlas después: sale lo mismo que en oled_ui.h."""
    if big and gap is not None:
        rows = [''] * 18
        for i, ch in enumerate(s):
            g = font.big_rows(ch)
            for k in range(18):
                rows[k] += g[k] + ('.' * gap if i < len(s) - 1 else '')
        return [[c == '#' for c in r] for r in rows]
    if big:
        return text_rows(s, True, 2 if gap is None else gap)
    rows = [''] * 9
    for i, ch in enumerate(s):
        g = font.rows9(ch)
        for k in range(9):
            rows[k] += g[k] + ('.' if i < len(s) - 1 else '')
    return [[c == '#' for c in r] for r in rows]

def draw_text(scr, s, cx, top, big=False, gap=None):
    rows = text_rows(s, big, gap)
    w = len(rows[0]) if rows else 0
    blit(scr, rows, cx - w // 2, top)
    return w

def screen_icon(bm, caption):
    scr = blank()
    blit(scr, bm, ICON_X, ICON_Y)
    if caption:
        draw_text(scr, caption, W // 2, CAP_Y)
    return scr

def screen_toggle(on, caption):
    scr = blank()
    blit(scr, icons.render_sprite('toggle_on' if on else 'toggle_off'), (W - 34) // 2, 5)
    draw_text(scr, caption, W // 2, CAP_Y)
    return scr

DOT, DOT_STEP, DOT_Y = 7, 12, 10

def screen_dots(cur, count, caption):
    """Puntos de página como los de la pantalla de inicio del iPhone."""
    scr = blank()
    x = (W - (DOT + (count - 1) * DOT_STEP)) // 2
    for i in range(count):
        blit(scr, icons.render_sprite('dot_on' if i == cur else 'dot_off'), x + i * DOT_STEP, DOT_Y)
    draw_text(scr, caption, W // 2, CAP_Y)
    return scr

# Esquina redondeada de la píldora de selección: cuántos píxeles se quedan a
# negro en cada una de las primeras filas, desde el borde.
PILL_CORNER = (3, 1, 1)

def invert_pill(scr):
    """La opción elegida va en negativo, pero con las esquinas redondeadas:
    una pantalla invertida a pelo parece un fallo; con esquinas, un botón."""
    for y in range(H):
        for x in range(W):
            scr[y][x] = not scr[y][x]
    for k, n in enumerate(PILL_CORNER):
        for i in range(n):
            for (x, y) in ((i, k), (W - 1 - i, k), (i, H - 1 - k), (W - 1 - i, H - 1 - k)):
                scr[y][x] = False
    return scr

def screen_text(s):
    """Solo texto: lo más grande que quepa, con las mismas reglas que el
    nombre del modo ayuda (screen_name)."""
    return screen_name(s)

# --- Teclas ---------------------------------------------------------------------
# Icono grande, sin nombre debajo: a 24x22 con el nombre en 7 px no se leía ni
# lo uno ni lo otro. El nombre lo enseña el modo ayuda (screen_name).
BIG_X = (W - icons.BIW) // 2
BIG_Y = (H - icons.BIH) // 2

def screen_icon_big(bm):
    scr = blank()
    blit(scr, bm, BIG_X, BIG_Y)
    return scr

# --- El texto, lo más grande que quepa --------------------------------------------
# Para el nombre del modo ayuda y para cualquier tecla que solo lleve texto. La
# fuente grande cabe dos veces en alto; de las variantes que caben se elige la
# primera de esta lista:
#   1. una línea grande        (normal y luego apretada)
#   2. dos líneas por espacio  (normal y luego apretada)
#   3. dos líneas con guion    (normal y luego apretada)
#   4. pequeña, en una línea y luego en dos
# «Apretada» es la grande con 1 px entre letras y sin margen: así caben en
# grande todos los nombres del catálogo, que con 2 px y margen se quedaban 38
# en pequeño.
BIG_FITS = ((2, W - 6), (1, W))   # (aire entre letras, ancho disponible)
SMALL_AVAIL = W - 6
# Centrado por la altura de mayúscula: si contaran los rabos, el texto parecería
# caído. Dos líneas grandes: cabeza en la 4 y en la 22; los rabos de la segunda
# (4 filas en grande) acaban justo en la 39.
BIG_TOP, SMALL_TOP = (H - 14) // 2, (H - 7) // 2
NAME_BIG_TOPS = (4, 22)
NAME_SMALL_TOPS = (10, 22)

VOWELS = set('aeiouáéíóúüAEIOUÁÉÍÓÚÜ')
# Parejas que no se separan al partir una palabra: «com-pro-bar», no «compr-obar».
GLUED = {a + b for a in 'bcdfgkpt' for b in 'lr'} | {'ll', 'rr', 'ch'}

def _letter(ch):
    # Las ASCII y las de Latin-1 y Latin Extended-A: lo mismo que mira
    # oled_ui.h, que no tiene isalpha() para tildes ni eñes.
    c = ord(ch)
    return ('a' <= ch.lower() <= 'z' and c < 128) or (0xC0 <= c <= 0x17F and c not in (0xD7, 0xF7))

def _consonant(ch):
    return _letter(ch) and ch not in VOWELS

def _glued(a, b):
    return (a + b).lower() in GLUED and a.isascii() and b.isascii()

def hyphen_points(s):
    """Dónde se puede partir una palabra con guion: antes de una sílaba que
    empieza por consonante (o por una pareja inseparable) seguida de vocal,
    dejando dos letras por lo menos a cada lado. No es la división silábica
    entera del castellano, pero basta para nombres de una o dos palabras."""
    out = []
    n = len(s)
    for i in range(2, n - 1):
        if not (_letter(s[i - 1]) and _letter(s[i - 2]) and _letter(s[i]) and _letter(s[i + 1])):
            continue
        cv = _consonant(s[i]) and s[i + 1] in VOWELS and not _glued(s[i - 1], s[i])
        ccv = i + 2 < n and _glued(s[i], s[i + 1]) and s[i + 2] in VOWELS
        if cv or ccv:
            out.append(i)
    return out

def width_at(t, big, gap):
    """Ancho en la fuente pequeña, o en la grande con `gap` px entre letras."""
    if not t:
        return 0
    if not big:
        return font.text_width(t)
    return 2 * sum(font.width(c) for c in t) + gap * (len(t) - 1)

def _splits(s, hyphen):
    if not hyphen:
        return [(s[:i], s[i + 1:]) for i, ch in enumerate(s) if ch == ' ' and 0 < i < len(s) - 1]
    return [(s[:i] + '-', s[i:]) for i in hyphen_points(s)]

def _best_split(s, big, gap, avail, hyphen):
    """La partición que cabe con la línea más ancha más estrecha; a igualdad,
    la primera."""
    best = None
    for a, b in _splits(s, hyphen):
        wa, wb = width_at(a, big, gap), width_at(b, big, gap)
        if wa <= avail and wb <= avail and (best is None or max(wa, wb) < best[0]):
            best = (max(wa, wb), a, b)
    return best and (best[1], best[2])

def name_layout(s):
    """Qué variante toca: (líneas, grande, aire). Ver la lista de arriba."""
    for gap, avail in BIG_FITS:
        if width_at(s, True, gap) <= avail:
            return [s], True, gap
    for hyphen in (False, True):
        for gap, avail in BIG_FITS:
            two = _best_split(s, True, gap, avail, hyphen)
            if two:
                return list(two), True, gap
    if width_at(s, False, 1) <= SMALL_AVAIL:
        return [s], False, 1
    for hyphen in (False, True):
        two = _best_split(s, False, 1, SMALL_AVAIL, hyphen)
        if two:
            return list(two), False, 1
    return [s], False, 1

def screen_name(s):
    scr = blank()
    if not s:
        return scr
    lines, big, gap = name_layout(s)
    if len(lines) == 1:
        draw_text(scr, s, W // 2, BIG_TOP if big else SMALL_TOP, big, gap)
    else:
        for t, top in zip(lines, NAME_BIG_TOPS if big else NAME_SMALL_TOPS):
            draw_text(scr, t, W // 2, top, big, gap)
    return scr
