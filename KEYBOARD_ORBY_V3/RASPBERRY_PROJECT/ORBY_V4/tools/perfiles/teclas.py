# Atajos escritos como se leen («Ctrl+Shift+S», «Seq:P,V») → lo que guarda el
# teclado (modificador + keycode HID, o una secuencia de pulsaciones).
#
# Los keycodes HID son posiciones físicas, no letras: el PC las traduce con su
# distribución de teclado. Por eso los signos van con nombre propio y pensados
# para el teclado ESPAÑOL, que es el que hay al otro lado: «Mas» es la tecla
# del + (la que en EE. UU. sería ]), «Menos» la del - (la / de EE. UU.).
# Letras, cifras, F1-F12, flechas y teclado numérico salen igual en cualquier
# distribución.

MOD = {'CTRL': 0x01, 'SHIFT': 0x02, 'ALT': 0x04, 'WIN': 0x08}

KEY = {}
for i, ch in enumerate('ABCDEFGHIJKLMNOPQRSTUVWXYZ'):
    KEY[ch] = 0x04 + i
for i, ch in enumerate('123456789'):
    KEY[ch] = 0x1E + i
KEY['0'] = 0x27
for i in range(12):
    KEY['F%d' % (i + 1)] = 0x3A + i
KEY.update({
    'ENTER': 0x28, 'ESC': 0x29, 'BACKSPACE': 0x2A, 'TAB': 0x2B, 'SPACE': 0x2C,
    'PRINTSCREEN': 0x46, 'INSERT': 0x49, 'HOME': 0x4A, 'PAGEUP': 0x4B,
    'DELETE': 0x4C, 'END': 0x4D, 'PAGEDOWN': 0x4E,
    'RIGHT': 0x4F, 'LEFT': 0x50, 'DOWN': 0x51, 'UP': 0x52,
    'KP/': 0x54, 'KP*': 0x55, 'KP-': 0x56, 'KP+': 0x57, 'KPENTER': 0x58,
    # Teclado español
    'MAS': 0x30, 'MENOS': 0x38, 'PUNTO': 0x37, 'COMA': 0x36,
})

# Tabla get_consumer_key_from_index() del firmware.
MEDIA = {
    'PLAY': 3, 'STOP': 4, 'NEXT': 5, 'PREV': 6, 'VOLUP': 7, 'VOLDOWN': 8,
    'MUTE': 9, 'BRIGHTUP': 10, 'BRIGHTDOWN': 11, 'BROWSER': 12, 'MAIL': 13,
    'CALC': 2, 'SEARCH': 14, 'ZOOMIN': 15, 'ZOOMOUT': 16,
}

CONSUMER = 0xFE
GOTO_PAGE = 0xFD
PAGE_STATE = 0xFC

# Tipos de acción giratoria (RotaryType en main.cpp)
R_NONE, R_CONSUMER, R_KEY, R_SCROLL_V, R_SCROLL_H, R_ZOOM = range(6)


def combo(text):
    """'Ctrl+Shift+S' → (modificador, keycode). Lanza ValueError si no se entiende."""
    parts = [p.strip() for p in text.split('+')]
    # «Ctrl+KP+» parte en ['Ctrl', 'KP', '']: se recompone
    if len(parts) >= 2 and parts[-1] == '' and parts[-2].upper() == 'KP':
        parts = parts[:-2] + ['KP+']
    mod, key = 0, None
    for p in parts:
        u = p.upper()
        if u in MOD:
            mod |= MOD[u]
        elif u in KEY:
            if key is not None:
                raise ValueError('dos teclas en «%s»' % text)
            key = KEY[u]
        else:
            raise ValueError('no sé qué tecla es «%s» en «%s»' % (p, text))
    if key is None:
        raise ValueError('falta la tecla en «%s»' % text)
    return mod, key


def action(text):
    """Una acción de tecla. Devuelve un dict para la biblioteca:
    {'modifier','keycode'} o {'sequence': [...]}."""
    if text.startswith('Seq:'):
        steps = [combo(s) for s in text[4:].split(',')]
        return {'sequence': [{'modifier': m, 'keycode': k} for m, k in steps]}
    if text.startswith('Media:'):
        return {'modifier': CONSUMER, 'keycode': MEDIA[text[6:].upper()]}
    if text.startswith('Pagina:'):
        return {'modifier': GOTO_PAGE, 'keycode': int(text[7:])}
    if text == 'EstadoPagina':
        return {'modifier': PAGE_STATE, 'keycode': 0}
    m, k = combo(text)
    return {'modifier': m, 'keycode': k}


def rot(text):
    """Una acción de giro o de clic de mando.
    'Scroll', 'ScrollH', 'Zoom' (Ctrl+rueda), 'Media:VolUp', o un atajo."""
    if text is None:
        return {'type': R_NONE, 'modifier': 0, 'keycode': 0}
    if text == 'Scroll':
        return {'type': R_SCROLL_V, 'modifier': 0, 'keycode': 0}
    if text == 'ScrollH':
        return {'type': R_SCROLL_H, 'modifier': 0, 'keycode': 0}
    if text == 'Zoom':
        return {'type': R_ZOOM, 'modifier': 0, 'keycode': 0}
    if text.startswith('Media:'):
        return {'type': R_CONSUMER, 'modifier': 0, 'keycode': MEDIA[text[6:].upper()]}
    m, k = combo(text)
    return {'type': R_KEY, 'modifier': m, 'keycode': k}
