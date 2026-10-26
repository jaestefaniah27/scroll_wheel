# Qué icono y qué nombre enseña cada etiqueta.
#
# El firmware busca por la etiqueta de la tecla (los 7 caracteres que guarda la
# página), no por el perfil ni por la posición: así un icono sigue a su acción
# si el usuario la mueve de sitio, y una etiqueta escrita a mano como «UNDO» en
# un perfil nuevo también sale con su icono. Si el usuario cambia la etiqueta,
# deja de coincidir y la tecla pasa a enseñar su texto, que es lo que pidió.
#
# Unas pocas etiquetas significan cosas distintas según el programa («SELC» es
# la marquesina en Photoshop y la flecha de selección en Premiere). Esas llevan
# una entrada «PERFIL/ETIQUETA» que gana a la genérica.

ENTRIES = {
    # Oficina
    'COPY': ('copy', 'Copiar'),
    'PAST': ('paste', 'Pegar'),
    'CUT':  ('cut', 'Cortar'),
    'UNDO': ('undo', 'Deshacer'),
    'REDO': ('redo', 'Rehacer'),
    'ALL':  ('select_all', 'Todo'),
    'SAVE': ('save', 'Guardar'),
    'FIND': ('find', 'Buscar'),
    'CLSE': ('close', 'Cerrar'),
    'SNAP': ('screenshot', 'Captura'),
    'EXCL': ('table', 'Excel'),
    'WORD': ('document', 'Word'),
    'BROW': ('globe', 'Navegador'),
    'CALC': ('calculator', 'Calculadora'),
    'TASK': ('activity', 'Tareas'),
    'LOCK': ('lock', 'Bloquear'),
    'PLAY': ('play_pause', 'Reproducir'),
    'STOP': ('stop', 'Detener'),
    'NEXT': ('next', 'Siguiente'),
    'PREV': ('prev', 'Anterior'),
    # Dibujo
    'BRSH': ('brush', 'Pincel'),
    'ERAS': ('eraser', 'Borrador'),
    'PENC': ('pencil', 'Lápiz'),
    'LASS': ('lasso', 'Lazo'),
    'CROP': ('crop', 'Recortar'),
    'MOVE': ('move', 'Mover'),
    'NEWL': ('layer_new', 'Capa nueva'),
    'DUPL': ('duplicate', 'Duplicar'),
    'ZOOM': ('zoom', 'Zoom'),
    'HAND': ('hand', 'Mano'),
    'TEXT': ('text', 'Texto'),
    'GRAD': ('gradient', 'Degradado'),
    'PEN':  ('pen', 'Pluma'),
    'SHAP': ('shapes', 'Forma'),
    'EYE':  ('eyedropper', 'Cuentagotas'),
    'SELC': ('select', 'Selección'),
    'SLIC': ('slice', 'Sector'),
    'PATH': ('path_select', 'Trazado'),
    # Electrónica
    'WIRE': ('wire', 'Cable'),
    'PORT': ('port', 'Puerto'),
    'PAD':  ('pad', 'Pad'),
    'VIA':  ('via', 'Vía'),
    'TRAC': ('track', 'Pista'),
    'FIT':  ('fit', 'Encuadrar'),
    '3D':   ('cube', 'Vista 3D'),
    'SCH':  ('schematic', 'Esquema'),
    'PCB':  ('pcb', 'PCB'),
    'LIB':  ('library', 'Librería'),
    'COMP': ('chip', 'Componente'),
    'GRID': ('grid', 'Rejilla'),
    'MEAS': ('ruler', 'Medir'),
    'RULE': ('checklist', 'Reglas'),
    'ERC':  ('bolt', 'ERC'),
    'DRC':  ('check_box', 'DRC'),
    'GEN':  ('gear', 'Opciones'),
    # Vídeo
    'MARK': ('marker', 'Marcador'),
    'LINK': ('link', 'Vincular'),
    'RIPL': ('ripple', 'Borrar rizo'),
    'EXPO': ('export', 'Exportar'),
    'NEST': ('nest', 'Anidar'),
    'SLIP': ('slip', 'Desplazar'),
    'SLID': ('slide', 'Deslizar'),
    'TRIM': ('trim', 'Recorte'),
    'RATE': ('rate', 'Estirar'),
    'ROLL': ('roll', 'Rotación'),
    'SPED': ('speed', 'Velocidad'),

    # Las que significan otra cosa según el programa
    'PREM/CUT':  ('razor', 'Cuchilla'),
    'PREM/SELC': ('pointer', 'Selección'),
    'PHTS/SELC': ('marquee', 'Marco'),

    # Nombres de los perfiles de fábrica (menú de perfiles)
    '@OFIM': ('briefcase', 'Oficina'),
    '@PHTS': ('brush', 'Photoshop'),
    '@ALTM': ('chip', 'Altium'),
    '@PREM': ('film', 'Premiere'),
}

# Pantallas del sistema: no salen de una etiqueta sino del propio firmware.
SYSTEM = {
    'menu_profiles': ('profiles', 'Perfiles'),
    'menu_sleep':    ('moon', 'Reposo'),
    'menu_wheel':    ('wheel', 'Rueda'),
    'menu_reset':    ('reset', 'Restablecer'),
    'menu_info':     ('info', 'Info'),
    'menu_exit':     ('exit', 'Salir'),
    'back':          ('back', 'Volver'),
    'sleep_1':       ('timer1', '1 min'),
    'sleep_5':       ('timer5', '5 min'),
    'sleep_10':      ('timer10', '10 min'),
    'sleep_off':     ('moon_off', 'Nunca'),
    'reset_yes':     ('trash', 'Sí, borrar'),
    'reset_no':      ('xmark', 'No'),
    'host_ok':       ('pc', 'PC conectado'),
    'host_none':     ('pc_off', 'Sin PC'),
    'build_date':    ('calendar', ''),
    'pages':         ('pages', ''),
}
