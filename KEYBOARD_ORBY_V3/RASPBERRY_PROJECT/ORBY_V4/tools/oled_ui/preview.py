# Hoja de previsualización: cada perfil de fábrica, capa normal y SUPER, con
# las diez pantallas colocadas como en el teclado. Uso:
#   python3 preview.py salida/
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from PIL import Image
import font, icons, catalog
from layout import screen_icon, screen_text, W, H

PROFILES = {
    'OFIM': (["COPY", "PAST", "CUT", "UNDO", "REDO", "ALL", "SAVE", "FIND", "CLSE", "SNAP"],
             ["EXCL", "WORD", "BROW", "CALC", "TASK", "LOCK", "PLAY", "STOP", "NEXT", "PREV"]),
    'PHTS': (["BRSH", "ERAS", "PENC", "UNDO", "REDO", "LASS", "CROP", "MOVE", "NEWL", "DUPL"],
             ["ZOOM", "HAND", "TEXT", "GRAD", "PEN", "SHAP", "EYE", "SELC", "SLIC", "PATH"]),
    'ALTM': (["WIRE", "PORT", "PAD", "UNDO", "REDO", "VIA", "MOVE", "TRAC", "FIT", "3D"],
             ["SCH", "PCB", "LIB", "COMP", "GRID", "MEAS", "RULE", "ERC", "DRC", "GEN"]),
    'PREM': (["CUT", "SELC", "PLAY", "UNDO", "REDO", "MARK", "LINK", "RIPL", "EXPO", "NEST"],
             ["SLIP", "SLID", "PEN", "HAND", "TEXT", "ZOOM", "TRIM", "RATE", "ROLL", "SPED"]),
}
COL = [0, 1, 2, 0, 1, 2, 0, 1, 2, 1]
ROW = [0, 0, 0, 1, 1, 1, 2, 2, 2, 3]

def lookup(profile, label):
    return catalog.ENTRIES.get(profile + '/' + label) or catalog.ENTRIES.get(label)

def screen_for(profile, label):
    e = lookup(profile, label)
    if e:
        return screen_icon(icons.render(e[0]), e[1])
    return screen_text(label)

def board(screens, esc=3, gx=12, gy=12):
    BW = 3 * W * esc + 4 * gx
    BH = 4 * H * esc + 5 * gy
    im = Image.new('L', (BW, BH), 24)
    for i, s in enumerate(screens):
        ox = gx + COL[i] * (W * esc + gx)
        oy = gy + ROW[i] * (H * esc + gy)
        for y in range(H):
            for x in range(W):
                v = 235 if s[y][x] else 0
                for dy in range(esc):
                    for dx in range(esc):
                        im.putpixel((ox + x * esc + dx, oy + y * esc + dy), v)
    return im

def sheet(boards):
    w = sum(b.width for b in boards) + 10 * (len(boards) - 1)
    h = max(b.height for b in boards)
    im = Image.new('L', (w, h), 60)
    x = 0
    for b in boards:
        im.paste(b, (x, 0)); x += b.width + 10
    return im

if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for p, (norm, sup) in PROFILES.items():
        sheet([board([screen_for(p, l) for l in norm]),
               board([screen_for(p, l) for l in sup])]).save(os.path.join(out, f'perfil_{p}.png'))
    # Todos los iconos sueltos, a 6x, para revisar el dibujo.
    names = list(icons.ICONS)
    cols = 12
    esc = 6
    im = Image.new('L', (cols * (icons.IW + 4) * esc, ((len(names) + cols - 1) // cols) * (icons.IH + 4) * esc), 40)
    for k, n in enumerate(names):
        bm = icons.render(n)
        ox = (k % cols) * (icons.IW + 4) * esc + 2 * esc
        oy = (k // cols) * (icons.IH + 4) * esc + 2 * esc
        for y in range(icons.IH):
            for x in range(icons.IW):
                v = 235 if bm[y][x] else 0
                im.paste(v, (ox + x * esc, oy + y * esc, ox + (x + 1) * esc, oy + (y + 1) * esc))
    im.save(os.path.join(out, 'iconos.png'))
