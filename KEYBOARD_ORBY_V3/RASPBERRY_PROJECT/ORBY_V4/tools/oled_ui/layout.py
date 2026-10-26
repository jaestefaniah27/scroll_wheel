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

def text_rows(s, big=False):
    rows = [''] * 9
    for i, ch in enumerate(s):
        g = font.rows9(ch)
        for k in range(9):
            rows[k] += g[k] + ('.' if i < len(s) - 1 else '')
    if big:
        rows = font.epx(rows)
    return [[c == '#' for c in r] for r in rows]

def draw_text(scr, s, cx, top, big=False):
    rows = text_rows(s, big)
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
    """Solo texto: en grande si cabe, centrado en vertical por la altura de
    mayúscula (los rabos no cuentan, si no el texto parece caído)."""
    scr = blank()
    if font.text_width(s) * 2 <= W - 6:
        draw_text(scr, s, W // 2, (H - 14) // 2, big=True)
    else:
        draw_text(scr, s, W // 2, (H - 7) // 2)
    return scr
