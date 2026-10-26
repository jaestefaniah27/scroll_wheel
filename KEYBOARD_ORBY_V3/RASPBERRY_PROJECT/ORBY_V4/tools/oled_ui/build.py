# Genera include/oled_ui_data.h (fuente, iconos y catálogo para el firmware)
# y tools/test/ui_fixture.h (pantallas de referencia para test_ui.cpp).
#
#   python3 tools/oled_ui/build.py           # regenera
#   python3 tools/oled_ui/build.py --check   # falla si lo generado está viejo
#
# Hay que volver a lanzarlo después de tocar font.py, icons.py o catalog.py.
# Las dos salidas van al repositorio: compilar el firmware no necesita Python.
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import font, icons, catalog, layout

ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
OUT = os.path.join(ROOT, 'include', 'oled_ui_data.h')
FIX = os.path.join(ROOT, 'tools', 'test', 'ui_fixture.h')


def cols(bm):
    """Mapa de bits (filas de bool) → una palabra por columna, bit 0 arriba.
    Es el sentido del SSD1306, donde cada byte es una columna de 8 píxeles."""
    h, w = len(bm), len(bm[0]) if bm else 0
    return [sum(1 << y for y in range(h) if bm[y][x]) for x in range(w)]


def cid(name):
    return name.upper().replace('-', '_')


def c_str(s):
    return '"' + s.replace('\\', '\\\\').replace('"', '\\"') + '"'


def main():
    L = []
    w = L.append
    w('// GENERADO por tools/oled_ui/build.py a partir de font.py, icons.py y')
    w('// catalog.py. No se edita a mano: se cambia el dibujo y se regenera.')
    w('#pragma once')
    w('#include <stdint.h>')
    w('')
    w('namespace OledUiData {')
    w('')
    w('struct Glyph  { uint16_t cp; uint8_t w; uint16_t off; };')
    w('struct Sprite { uint8_t w, h; uint16_t off; };')
    w('struct Entry  { const char* key; uint8_t icon; const char* caption; };')
    w('')

    # --- fuente pequeña y grande ------------------------------------------------
    chars = sorted(font.G.keys(), key=ord)
    small, big, sg, bg = [], [], [], []
    for ch in chars:
        r9 = font.rows9(ch)
        bm = [[c == '#' for c in r] for r in r9]
        sg.append((ord(ch), len(r9[0]), len(small)))
        small += cols(bm)
        e = font.epx(r9)
        bg.append((ord(ch), len(e[0]), len(big)))
        big += cols([[c == '#' for c in r] for r in e])
    w('// Fuente pequeña: 9 filas (7 de mayúscula + 2 de rabo), una palabra por columna.')
    w('static const uint16_t font_cols[] = {')
    for i in range(0, len(small), 16):
        w('    ' + ', '.join('0x%03x' % v for v in small[i:i + 16]) + ',')
    w('};')
    w('static const Glyph font_small[] = {')
    for cp, gw, off in sg:
        w('    {0x%04x, %d, %d},' % (cp, gw, off))
    w('};')
    w('')
    w('// Fuente grande: la misma pasada por EPX (ver font.py), 18 filas.')
    w('static const uint32_t big_cols[] = {')
    for i in range(0, len(big), 12):
        w('    ' + ', '.join('0x%05x' % v for v in big[i:i + 12]) + ',')
    w('};')
    w('static const Glyph font_big[] = {')
    for cp, gw, off in bg:
        w('    {0x%04x, %d, %d},' % (cp, gw, off))
    w('};')
    w('static const uint16_t FONT_GLYPHS = %d;' % len(sg))
    w('')
    w('// Letras sin glifo propio y la que las sustituye.')
    w('static const uint16_t fold[][2] = {')
    for a, b in sorted(font.FOLD.items()):
        w('    {0x%04x, 0x%04x},' % (ord(a), ord(b)))
    w('};')
    w('')

    # --- iconos y piezas ----------------------------------------------------------
    names = list(icons.ICONS) + list(icons.SPRITES) + list(icons.ASCII_SPRITES)
    data, table = [], []
    for n in names:
        bm = icons.render(n) if n in icons.ICONS else icons.render_sprite(n)
        table.append((n, len(bm[0]), len(bm), len(data)))
        data += cols(bm)
    w('static const uint32_t sprite_cols[] = {')
    for i in range(0, len(data), 10):
        w('    ' + ', '.join('0x%06x' % v for v in data[i:i + 10]) + ',')
    w('};')
    w('static const Sprite sprites[] = {')
    for n, sw, sh, off in table:
        w('    {%d, %d, %d},  // %s' % (sw, sh, off, n))
    w('};')
    w('')
    w('enum : uint8_t {')
    for k, (n, *_r) in enumerate(table):
        w('    SPR_%s = %d,' % (cid(n), k))
    w('    SPR_COUNT')
    w('};')
    w('')

    # --- catálogo -------------------------------------------------------------------
    w('// Etiqueta de tecla → icono y nombre. «PERFIL/ETIQUETA» gana a la genérica;')
    w('// «@NOMBRE» es el icono de un perfil en el menú de perfiles.')
    w('static const Entry entries[] = {')
    for key, (ic, cap) in catalog.ENTRIES.items():
        w('    {%s, SPR_%s, %s},' % (c_str(key), cid(ic), c_str(cap)))
    w('};')
    w('')
    w('// Pantallas del propio firmware.')
    w('enum : uint8_t {')
    for k, key in enumerate(catalog.SYSTEM):
        w('    SYS_%s = %d,' % (cid(key), k))
    w('};')
    w('static const Entry system_entries[] = {')
    for key, (ic, cap) in catalog.SYSTEM.items():
        w('    {%s, SPR_%s, %s},' % (c_str(key), cid(ic), c_str(cap)))
    w('};')
    w('')
    w('} // namespace OledUiData')
    salidas = {OUT: '\n'.join(L) + '\n'}

    # --- pantallas de referencia ------------------------------------------------------
    cases = []
    for key in ('COPY', 'CALC', 'PREM/CUT', 'EYE', '3D', 'SNAP'):
        ic, cap = catalog.ENTRIES[key]
        cases.append(('icon:' + key, layout.screen_icon(icons.render(ic), cap)))
    for s in ('F1', 'WIRE', 'P2/2', 'ABCDEFG', 'ñandú', 'Año 2026'):
        cases.append(('text:' + s, layout.screen_text(s)))
    cases.append(('toggle:1:Normal', layout.screen_toggle(True, 'Normal')))
    cases.append(('toggle:0:SUPER', layout.screen_toggle(False, 'SUPER')))
    cases.append(('dots:1:3:Página 2 de 3', layout.screen_dots(1, 3, 'Página 2 de 3')))
    ic, cap = catalog.SYSTEM['menu_profiles']
    cases.append(('pill:menu_profiles', layout.invert_pill(layout.screen_icon(icons.render(ic), cap))))

    F = ['// GENERADO por tools/oled_ui/build.py: lo que layout.py dibuja para cada',
         '// caso. test_ui.cpp comprueba que oled_ui.h dibuja exactamente lo mismo.',
         '#pragma once', '#include <stdint.h>', '',
         'struct UiCase { const char* what; uint8_t fb[360]; };',
         'static const UiCase ui_cases[] = {']
    for name, scr in cases:
        fb = [0] * 360
        for y in range(40):
            for x in range(72):
                if scr[y][x]:
                    fb[(y >> 3) * 72 + x] |= 1 << (y & 7)
        F.append('    {%s, {' % c_str(name))
        for i in range(0, 360, 24):
            F.append('        ' + ', '.join('0x%02x' % v for v in fb[i:i + 24]) + ',')
        F.append('    }},')
    F.append('};')
    salidas[FIX] = '\n'.join(F) + '\n'

    if '--check' in sys.argv:
        # Para run.ps1: si alguien toca un icono y no regenera, el firmware
        # seguiría dibujando el viejo sin que nada avise.
        viejos = [p for p, txt in salidas.items()
                  if not os.path.exists(p) or open(p, encoding='utf-8').read() != txt]
        for p in viejos:
            print('Sin regenerar: %s (lanza tools/oled_ui/build.py)' % os.path.relpath(p, ROOT))
        sys.exit(1 if viejos else 0)
    for p, txt in salidas.items():
        open(p, 'w', encoding='utf-8').write(txt)

    print('%s: %d glifos, %d dibujos (%d bytes), %d etiquetas' % (
        os.path.relpath(OUT, ROOT), len(sg), len(table), 4 * len(data) + 2 * len(small) + 4 * len(big),
        len(catalog.ENTRIES)))
    print('%s: %d casos' % (os.path.relpath(FIX, ROOT), len(cases)))


if __name__ == '__main__':
    main()
