# Cómo se escribe un perfil de la biblioteca.
#
# Las teclas se dan en el orden en que están en el teclado: las nueve de la
# rejilla de 3x3 de arriba abajo y de izquierda a derecha, y luego la de abajo
# en el centro (la 11, entre la de menú y la de SUPER):
#
#     1  2  3
#     4  5  6
#     7  8  9
#     ·  11 ·      · = menú y SUPER, que no se tocan
#
# Cada tecla es (ETIQUETA, atajo). La etiqueta es lo que guarda el teclado (7
# caracteres como mucho) y la que elige icono y nombre en tools/oled_ui/catalog.py.
from teclas import action, rot

PERFILES = []

# Mandos por defecto: el izquierdo es el volumen en todos los perfiles, para
# que la mano lo encuentre siempre en el mismo sitio; el derecho y la rueda
# son los que cambia cada programa.
VOLUMEN = ('Media:VolUp', 'Media:VolDown', 'Media:Mute')
BRILLO = ('Media:BrightUp', 'Media:BrightDown', None)
SCROLL = ('Scroll', 'Scroll')


class Perfil:
    def __init__(self, nombre, programa, grupo, procesos=(), descripcion=''):
        if len(nombre.encode('utf-8')) > 7:
            raise ValueError('nombre de perfil de más de 7 bytes: %s' % nombre)
        self.nombre, self.programa, self.grupo = nombre, programa, grupo
        self.procesos = list(procesos)
        self.descripcion = descripcion
        self.paginas = []
        PERFILES.append(self)

    def pagina(self, titulo, normal, super_=None, der=BRILLO, rueda=SCROLL,
               izq=VOLUMEN, der_super=None, rueda_super=None, izq_super=None,
               detents=(60, 60)):
        self.paginas.append(dict(titulo=titulo, normal=normal, super_=super_ or [None] * 10,
                                 izq=izq, der=der, rueda=rueda,
                                 izq_super=izq_super or izq, der_super=der_super or der,
                                 rueda_super=rueda_super or rueda, detents=detents))
        return self


def _key_index(pos):
    """Posición en la lista (0-9) → índice de tecla del firmware (0-11)."""
    return pos if pos < 9 else 10


def compilar(perfil):
    """El perfil en el formato que entiende la app (ver src/profile-library.js)."""
    pages = []
    for pg in perfil.paginas:
        labels = [''] * 20
        keys = [{'modifier': 0, 'keycode': 0} for _ in range(24)]
        for capa, lista in ((0, pg['normal']), (1, pg['super_'])):
            if len(lista) != 10:
                raise ValueError('%s / %s: la capa %d tiene %d teclas, no 10'
                                 % (perfil.nombre, pg['titulo'], capa, len(lista)))
            for pos, t in enumerate(lista):
                if t is None:
                    continue
                etiqueta, atajo = t
                if len(etiqueta.encode('utf-8')) > 7 or ':' in etiqueta:
                    raise ValueError('%s: etiqueta no válida «%s»' % (perfil.nombre, etiqueta))
                labels[pos + 10 * capa] = etiqueta
                keys[_key_index(pos) + 12 * capa] = action(atajo)
        rotary = []
        for capa in ('', '_super'):
            izq, der, rueda = pg['izq' + capa], pg['der' + capa], pg['rueda' + capa]
            rotary += [rot(izq[0]), rot(izq[1]), rot(izq[2]),
                       rot(der[0]), rot(der[1]), rot(der[2]),
                       rot(rueda[0]), rot(rueda[1])]
        scroll = [{'detentsPerRev': d, 'invert': False} for d in pg['detents']]
        pages.append({'title': pg['titulo'], 'labels': labels, 'keys': keys,
                      'rotary': rotary, 'scroll': scroll})
    return {
        'id': perfil.nombre.lower(),
        'name': perfil.nombre,
        'app': perfil.programa,
        'group': perfil.grupo,
        'description': perfil.descripcion,
        'processes': perfil.procesos,
        'pages': pages,
    }
