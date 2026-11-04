# Etiquetas de los juegos y de sus menús. La almohadilla delante las distingue de las que
# escribe un usuario en una tecla (nadie escribe «#TOPOS»), y ocho caracteres como máximo,
# que es lo que guarda el firmware por etiqueta.
#
# La app dibuja con ellas las pantallas del menú de juegos y de la pantalla final
# (OrbyGUI/src/games/). El firmware no las busca por etiqueta: el mando del selector de
# perfiles lo saca directo de su icono (SPR_GAMEPAD). Están en el catálogo porque es lo que
# se genera para las dos mitades, y así la app encuentra cada icono por su nombre.

ENTRIES = {
    '#JUEGOS': ('gamepad', 'Juegos'),
    '#TOPOS':  ('mole', 'Topos'),
    '#SIMON':  ('simon', 'Simon'),
    '#SINT':   ('radio', 'Sintoniza'),
    '#REFLEJO': ('reflejos', 'Reflejos'),
    '#MISIL':  ('missile', 'Air Strike'),
    '#OTRA':   ('reset', 'Otra vez'),
    '#MENU':   ('list', 'Menú'),
    '#SALIR':  ('exit', 'Salir'),
}
