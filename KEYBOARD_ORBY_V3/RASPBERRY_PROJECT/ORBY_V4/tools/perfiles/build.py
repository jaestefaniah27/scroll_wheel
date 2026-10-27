# Genera OrbyGUI/src/profile-library.json a partir de biblioteca.py.
#
#   python3 tools/perfiles/build.py           # regenera
#   python3 tools/perfiles/build.py --check   # falla si está sin regenerar
#
# Además de compilar, comprueba lo que el teclado no perdonaría: etiquetas de
# más de 7 caracteres, teclas de menú o SUPER ocupadas, secuencias más largas de
# lo que cabe, y etiquetas sin icono en el catálogo de las pantallas (esas
# saldrían como texto, que es justo lo que no queremos en un perfil de fábrica).
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'oled_ui'))
import modelo, biblioteca  # noqa: F401  (biblioteca rellena modelo.PERFILES)
import catalog

ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
OUT = os.path.join(ROOT, 'OrbyGUI', 'src', 'profile-library.json')
DOC = os.path.join(HERE, 'PERFILES.md')

MAX_STEPS = 48          # MACRO_MAX_STEPS en main.cpp
MAX_PAGES = 4           # MAX_PAGES en main.cpp
SYSTEM_KEYS = (9, 11)   # menú y SUPER (base 0)


def icono_de(perfil, etiqueta):
    return (catalog.ENTRIES.get((perfil + '/' + etiqueta).upper())
            or catalog.ENTRIES.get(etiqueta.upper()))


def nombre(perfil, etiqueta):
    e = icono_de(perfil, etiqueta)
    return e[1] if e else etiqueta


def mando(t):
    if not t:
        return '—'
    return {'Scroll': 'desplazar', 'ScrollH': 'desplazar en horizontal',
            'Zoom': 'Ctrl + rueda'}.get(t, t.replace('Media:', ''))


def chuleta():
    """PERFILES.md: cada página tecla a tecla, para revisar los atajos sin leer
    biblioteca.py. Se genera; no se edita."""
    L = ['# Biblioteca de perfiles', '',
         'Generado por `tools/perfiles/build.py` a partir de `biblioteca.py`. No se edita a mano.', '',
         'Teclas en el orden del teclado: la rejilla 3×3 y la 11 (abajo, en el centro).',
         '«Seq:» es una secuencia de teclas, que el teclado reproduce él solo.', '']
    for p in modelo.PERFILES:
        L += ['## %s (`%s`)' % (p.programa, p.nombre), '', p.descripcion, '']
        if p.procesos:
            L += ['Se activa solo con: %s' % ', '.join('`%s`' % x for x in p.procesos), '']
        for pg in p.paginas:
            L += ['### %s' % pg['titulo'], '', '| Tecla | Normal | Atajo | SUPER | Atajo |', '|---|---|---|---|---|']
            for i in range(10):
                n, s_ = pg['normal'][i], pg['super_'][i]
                fila = ['%d' % (i + 1 if i < 9 else 11)]
                for t in (n, s_):
                    fila += [nombre(p.nombre, t[0]), '`%s`' % t[1]] if t else ['', '']
                L.append('| ' + ' | '.join(fila) + ' |')
            L += ['', 'Mando derecho: %s / %s, clic: %s. Rueda: %s. Con SUPER, mando derecho: %s / %s.'
                  % (mando(pg['der'][0]), mando(pg['der'][1]), mando(pg['der'][2]), mando(pg['rueda'][0]),
                     mando(pg['der_super'][0]), mando(pg['der_super'][1])), '']
    return '\n'.join(L) + '\n'


def main():
    errores, sin_icono = [], {}
    perfiles = []
    for p in modelo.PERFILES:
        d = modelo.compilar(p)
        if len(d['pages']) > MAX_PAGES:
            errores.append('%s: %d páginas, caben %d' % (p.nombre, len(d['pages']), MAX_PAGES))
        if not icono_de('', '@' + p.nombre):
            sin_icono.setdefault('@' + p.nombre.upper(), set()).add(p.nombre)
        for pg in d['pages']:
            for capa in (0, 12):
                for k in SYSTEM_KEYS:
                    a = pg['keys'][capa + k]
                    if a.get('modifier') or a.get('keycode') or a.get('sequence'):
                        errores.append('%s/%s: la tecla %d es de sistema' % (p.nombre, pg['title'], k + 1))
            for a in pg['keys']:
                if len(a.get('sequence', [])) * 2 > MAX_STEPS:
                    errores.append('%s/%s: secuencia demasiado larga' % (p.nombre, pg['title']))
            for lab in pg['labels']:
                if lab and not icono_de(p.nombre, lab):
                    sin_icono.setdefault(lab, set()).add(p.nombre)
        perfiles.append(d)

    texto = json.dumps({'version': 1, 'profiles': perfiles}, ensure_ascii=False, indent=1) + '\n'

    for e in errores:
        print('ERROR:', e)
    if sin_icono:
        print('Etiquetas sin icono (%d):' % len(sin_icono))
        for lab in sorted(sin_icono):
            print('  %-8s %s' % (lab, ', '.join(sorted(sin_icono[lab]))))

    salidas = {OUT: texto, DOC: chuleta()}
    if '--check' in sys.argv:
        for ruta, txt in salidas.items():
            viejo = open(ruta, encoding='utf-8').read() if os.path.exists(ruta) else None
            if viejo != txt:
                print('Sin regenerar: %s (lanza tools/perfiles/build.py)' % os.path.relpath(ruta, ROOT))
                errores.append('sin regenerar')
    else:
        for ruta, txt in salidas.items():
            open(ruta, 'w', encoding='utf-8').write(txt)
        n_pag = sum(len(p['pages']) for p in perfiles)
        n_seq = sum(1 for p in perfiles for pg in p['pages'] for a in pg['keys'] if 'sequence' in a)
        print('%s: %d perfiles, %d páginas, %d teclas con secuencia'
              % (os.path.relpath(OUT, ROOT), len(perfiles), n_pag, n_seq))
    sys.exit(1 if errores or sin_icono else 0)


if __name__ == '__main__':
    main()
