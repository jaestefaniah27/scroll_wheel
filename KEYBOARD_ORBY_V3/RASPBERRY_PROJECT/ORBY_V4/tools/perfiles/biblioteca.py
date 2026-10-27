# La biblioteca de perfiles. Ver modelo.py para cómo se lee cada página.
#
# Criterios:
# - Una página por tarea: lo que se usa junto va junto (en Altium, el
#   esquemático en una página y el rutado en otra). Se cambia de página con un
#   toque a la tecla de menú.
# - En la capa normal, lo de cada momento; con SUPER, lo de cada rato.
# - Donde tiene sentido, la fila de abajo de la rejilla es Deshacer y Rehacer
#   en los extremos (7 y 9), y la tecla 11 es la orden que más se repite.
# - Mando izquierdo: volumen, siempre. Mando derecho: el zoom o lo que haga
#   las veces de «moverse» en ese programa. Rueda: desplazamiento.
# - Office en español (Ctrl+G guardar, Ctrl+N negrita, Ctrl+B buscar...) y
#   teclado español. Los atajos con signos se evitan salvo los que caen en la
#   misma tecla en español.
from modelo import Perfil, VOLUMEN, SCROLL

UNDO = ('UNDO', 'Ctrl+Z')
REDO_Y = ('REDO', 'Ctrl+Y')
REDO_SZ = ('REDO', 'Ctrl+Shift+Z')

# ============================================================================
# INGENIERÍA
# ============================================================================

# --- Altium Designer ----------------------------------------------------------
# Altium funciona con aceleradores de menú en secuencia: «P, V» es Colocar >
# Vía. El teclado los reproduce él solo (se instalan como secuencias), así que
# funcionan aunque OrbyGUI esté cerrada.
ZOOM_ALTIUM = ('PageUp', 'PageDown', 'Ctrl+PageDown')   # acercar, alejar, encuadrar

Perfil('Altium', 'Altium Designer', 'Ingeniería', procesos=['x2.exe'],
       descripcion='Esquemático, rutado de PCB y vista/fabricación, una página cada uno.') \
.pagina('Esquemático', [
    ('WIRE', 'Ctrl+W'),      ('NETLBL', 'Seq:P,N'),  ('COMP', 'Seq:P,P'),
    ('POWER', 'Seq:P,O'),    ('PORT', 'Seq:P,R'),    ('BUS', 'Seq:P,B'),
    UNDO,                    ('ROTATE', 'Space'),    REDO_Y,
    ('SAVE', 'Ctrl+S'),
], super_=[
    ('NOERC', 'Seq:P,V,N'),  ('ANNOT', 'Seq:T,A,A'), ('COMPILE', 'Seq:C,C'),
    ('SIMILAR', 'Shift+F'),  ('PROPS', 'F11'),       ('XSEL', 'Ctrl+Shift+X'),
    ('MIRRORX', 'X'),        ('CLEAR', 'Shift+C'),   ('MIRRORY', 'Y'),
    ('TOPCB', 'Seq:D,U'),
], der=ZOOM_ALTIUM, rueda_super=('ScrollH', 'ScrollH'),
   der_super=('Ctrl+Tab', 'Ctrl+Shift+Tab', None)) \
.pagina('PCB', [
    ('ROUTE', 'Ctrl+W'),     ('VIA', 'Seq:P,V'),     ('DIFFPR', 'Seq:P,I'),
    ('LAYPREV', 'KP-'),      ('ONELAY', 'Shift+S'),  ('LAYNEXT', 'KP+'),
    UNDO,                    ('ROTATE', 'Space'),    REDO_Y,
    ('SAVE', 'Ctrl+S'),
], super_=[
    ('POLYGON', 'Seq:P,G'),  ('FILL', 'Seq:P,F'),    ('COMP', 'Seq:P,C'),
    ('RTMODE', 'Shift+R'),   ('WIDTH', 'Shift+W'),   ('VIASIZE', 'Shift+V'),
    ('FLIPSD', 'L'),         ('TUNING', 'Seq:T,R'),  ('CLEAR', 'Shift+C'),
    ('MEAS', 'Ctrl+M'),
], der=ZOOM_ALTIUM, rueda_super=('ScrollH', 'ScrollH'),
   der_super=('KP+', 'KP-', 'Shift+S')) \
.pagina('Vista y salida', [
    ('VIEW2D', '2'),         ('3D', '3'),            ('FLIPBRD', 'Seq:V,B'),
    ('LAYERS', 'L'),         ('FIT', 'Ctrl+PageDown'), ('UNITS', 'Q'),
    ('REPOUR', 'Seq:T,G,A'), ('DRC', 'Seq:T,D'),     ('TEARDRP', 'Seq:T,E'),
    ('COMPILE', 'Seq:C,C'),
], super_=[
    ('SMRTPDF', 'Seq:F,M'),  ('BOM', 'Seq:R,I'),     ('TOSCH', 'Seq:D,U'),
    ('DOCPREV', 'Ctrl+Shift+Tab'), ('GRID', 'G'),    ('DOCNEXT', 'Ctrl+Tab'),
    ('SIMILAR', 'Shift+F'),  ('PROPS', 'F11'),       ('ALL', 'Ctrl+A'),
    ('SAVE', 'Ctrl+S'),
], der=ZOOM_ALTIUM, rueda_super=('ScrollH', 'ScrollH'))

# --- KiCad --------------------------------------------------------------------
# Atajos de KiCad 8/9. Eeschema y Pcbnew son programas aparte, así que la regla
# de cambio automático los reconoce a los dos.
ZOOM_KICAD = ('F1', 'F2', 'Home')

Perfil('KiCad', 'KiCad', 'Ingeniería', procesos=['eeschema.exe', 'pcbnew.exe', 'kicad.exe'],
       descripcion='Esquemático y PCB de KiCad 8/9.') \
.pagina('Esquemático', [
    ('WIRE', 'W'),           ('NETLBL', 'L'),        ('COMP', 'A'),
    ('POWER', 'P'),          ('NOCONN', 'Q'),        ('JUNCT', 'J'),
    UNDO,                    ('ROTATE', 'R'),        REDO_Y,
    ('SAVE', 'Ctrl+S'),
], super_=[
    ('GLOBLBL', 'Ctrl+L'),   ('HIERLBL', 'H'),       ('BUS', 'B'),
    ('MOVE', 'M'),           ('PROPS', 'E'),         ('DRAG', 'G'),
    ('MIRRORX', 'X'),        ('REPEAT', 'Insert'),   ('MIRRORY', 'Y'),
    ('TOPCB', 'F8'),
], der=ZOOM_KICAD, rueda_super=('ScrollH', 'ScrollH')) \
.pagina('PCB', [
    ('ROUTE', 'X'),          ('VIA', 'Ctrl+Shift+V'), ('DIFFPR', '6'),
    ('LAYPREV', 'KP-'),      ('FLIPSD', 'F'),        ('LAYNEXT', 'KP+'),
    UNDO,                    ('ROTATE', 'R'),        REDO_Y,
    ('SAVE', 'Ctrl+S'),
], super_=[
    ('FILLZN', 'B'),         ('UNFILL', 'Ctrl+B'),   ('FOOTPR', 'A'),
    ('WIDTH', 'W'),          ('DRAG', 'D'),          ('SELCONN', 'U'),
    ('CONTRST', 'Ctrl+H'),   ('MEAS', 'Ctrl+Shift+M'), ('3D', 'Alt+3'),
    ('FIT', 'Home'),
], der=ZOOM_KICAD, der_super=('KP+', 'KP-', 'F'), rueda_super=('ScrollH', 'ScrollH'))

# --- Fusion 360 ---------------------------------------------------------------
# En Fusion la rueda ya hace zoom: el mando derecho también, y su clic encuadra.
ZOOM_FUSION = ('Scroll', 'Scroll', 'F6')
COMUNES_FUSION = [
    ('TOOLBOX', 'S'),        ('MEAS', 'I'),          ('VISIBLE', 'V'),
    ('APPEAR', 'A'),         ('FIT', 'F6'),          ('COMPUTE', 'Ctrl+B'),
    UNDO,                    ('DELETE', 'Delete'),   REDO_Y,
    ('SAVE', 'Ctrl+S'),
]

Perfil('Fusion', 'Fusion 360', 'Ingeniería', procesos=['fusion360.exe'],
       descripcion='Boceto y modelado de sólidos.') \
.pagina('Boceto', [
    ('LINE', 'L'),           ('RECT', 'R'),          ('CIRCLE', 'C'),
    ('DIM', 'D'),            ('TRIM', 'T'),          ('OFFSET', 'O'),
    UNDO,                    ('CONSTR', 'X'),        REDO_Y,
    ('PROJECT', 'P'),
], super_=COMUNES_FUSION, der=ZOOM_FUSION) \
.pagina('Sólidos', [
    ('EXTRUDE', 'E'),        ('PRESSPL', 'Q'),       ('FILLET', 'F'),
    ('HOLE', 'H'),           ('MOVE', 'M'),          ('JOINT', 'J'),
    UNDO,                    ('MEAS', 'I'),          REDO_Y,
    ('FIT', 'F6'),
], super_=COMUNES_FUSION, der=ZOOM_FUSION)

# --- LTspice ------------------------------------------------------------------
Perfil('LTspice', 'LTspice', 'Ingeniería', procesos=['ltspice.exe', 'xviix64.exe'],
       descripcion='Dibujo del esquema y componentes básicos.') \
.pagina('Esquema', [
    ('WIRE', 'F3'),          ('COMP', 'F2'),         ('NETLBL', 'F4'),
    ('RES', 'R'),            ('CAP', 'C'),           ('IND', 'L'),
    ('UNDO', 'F9'),          ('ROTATE', 'Ctrl+R'),   ('REDO', 'Shift+F9'),
    ('GND', 'G'),
], super_=[
    ('DIODE', 'D'),          ('SPICE', 'S'),         ('TEXT', 'T'),
    ('MOVE', 'F7'),          ('DRAG', 'F8'),         ('DUPL', 'F6'),
    ('MIRROR', 'Ctrl+E'),    ('DELETE', 'F5'),       ('FIT', 'Space'),
    ('SAVE', 'Ctrl+S'),
], der=('Scroll', 'Scroll', 'Space'))

# ============================================================================
# PROGRAMACIÓN Y SISTEMA
# ============================================================================

# --- VS Code ------------------------------------------------------------------
TABS_CODE = ('Ctrl+PageDown', 'Ctrl+PageUp', None)

Perfil('VSCode', 'Visual Studio Code', 'Programación', procesos=['code.exe'],
       descripcion='Edición, paneles y depuración.') \
.pagina('Edición', [
    ('PALETTE', 'Ctrl+Shift+P'), ('GOFILE', 'Ctrl+P'), ('FINDALL', 'Ctrl+Shift+F'),
    ('MULTSEL', 'Ctrl+D'),   ('LINEUP', 'Alt+Up'),   ('DUPLINE', 'Shift+Alt+Down'),
    UNDO,                    ('LINEDN', 'Alt+Down'), REDO_Y,
    ('SAVE', 'Ctrl+S'),
], super_=[
    ('FORMAT', 'Shift+Alt+F'), ('RENAME', 'F2'),     ('REPLACE', 'Ctrl+H'),
    ('GODEF', 'F12'),        ('REFS', 'Shift+F12'),  ('NAVBACK', 'Alt+Left'),
    ('DELLINE', 'Ctrl+Shift+K'), ('QFIX', 'Ctrl+Punto'), ('SYMBOL', 'Ctrl+Shift+O'),
    ('COMMENT', 'Seq:Ctrl+K,Ctrl+C'),
], der=TABS_CODE, rueda_super=('ScrollH', 'ScrollH')) \
.pagina('Paneles', [
    ('EXPLR', 'Ctrl+Shift+E'), ('GIT', 'Ctrl+Shift+G'), ('EXTENS', 'Ctrl+Shift+X'),
    ('SIDEBAR', 'Ctrl+B'),   ('PANEL', 'Ctrl+J'),    ('PROBLEM', 'Ctrl+Shift+M'),
    ('TABPREV', 'Ctrl+PageUp'), ('TABCLOS', 'Ctrl+W'), ('TABNEXT', 'Ctrl+PageDown'),
    ('ZEN', 'Seq:Ctrl+K,Z'),
], super_=[
    ('NEWFILE', 'Ctrl+N'),   ('OPENFLD', 'Seq:Ctrl+K,Ctrl+O'), ('SETTING', 'Ctrl+Coma'),
    ('WRAP', 'Alt+Z'),       ('MDPREV', 'Ctrl+Shift+V'), ('CLOSEAL', 'Seq:Ctrl+K,W'),
    ('FOLDALL', 'Seq:Ctrl+K,Ctrl+0'), ('SAVEALL', 'Seq:Ctrl+K,S'), ('UNFOLD', 'Seq:Ctrl+K,Ctrl+J'),
    ('REOPEN', 'Ctrl+Shift+T'),
], der=TABS_CODE, rueda_super=('ScrollH', 'ScrollH')) \
.pagina('Depuración', [
    ('DEBUG', 'F5'),         ('RUN', 'Ctrl+F5'),     ('DBGSTOP', 'Shift+F5'),
    ('STEPOVR', 'F10'),      ('STEPIN', 'F11'),      ('STEPOUT', 'Shift+F11'),
    ('BREAKPT', 'F9'),       ('DBGRST', 'Ctrl+Shift+F5'), ('BUILD', 'Ctrl+Shift+B'),
    ('PANEL', 'Ctrl+J'),
], super_=[
    ('GODEF', 'F12'),        ('REFS', 'Shift+F12'),  ('SYMBOL', 'Ctrl+Shift+O'),
    ('NAVBACK', 'Alt+Left'), ('PROBLEM', 'Ctrl+Shift+M'), ('NAVFWD', 'Alt+Right'),
    UNDO,                    ('GIT', 'Ctrl+Shift+G'), REDO_Y,
    ('SAVE', 'Ctrl+S'),
], der=TABS_CODE, rueda_super=('ScrollH', 'ScrollH'))

# --- Windows ------------------------------------------------------------------
Perfil('Windows', 'Windows', 'Programación',
       descripcion='Ventanas y escritorios virtuales; música y utilidades del sistema.') \
.pagina('Ventanas', [
    ('DESK', 'Win+D'),       ('TASKV', 'Win+Tab'),   ('FILES', 'Win+E'),
    ('SNAPL', 'Win+Left'),   ('MAXIM', 'Win+Up'),    ('SNAPR', 'Win+Right'),
    ('MONITOR', 'Win+Shift+Left'), ('MINIM', 'Win+Down'), ('CLOSEW', 'Alt+F4'),
    ('SNAP', 'Win+Shift+S'),
], super_=[
    ('DESKL', 'Win+Ctrl+Left'), ('DESKN', 'Win+Ctrl+D'), ('DESKR', 'Win+Ctrl+Right'),
    ('CLIPB', 'Win+V'),      ('EMOJI', 'Win+Punto'), ('LOCK', 'Win+L'),
    ('SETTING', 'Win+I'),    ('TASK', 'Ctrl+Shift+Esc'), ('DESKX', 'Win+Ctrl+F4'),
    ('SEARCH', 'Win+S'),
]) \
.pagina('Multimedia', [
    ('PREV', 'Media:Prev'),  ('PLAY', 'Media:Play'), ('NEXT', 'Media:Next'),
    ('MUTE', 'Media:Mute'),  ('STOP', 'Media:Stop'), ('MICMUTE', 'Win+Alt+K'),
    ('BROW', 'Media:Browser'), ('MAIL', 'Media:Mail'), ('CALC', 'Media:Calc'),
    ('PROJ', 'Win+P'),
], super_=[
    ('REC', 'Win+Alt+R'),    ('GAMEBAR', 'Win+G'),   ('SHOTFUL', 'Win+PrintScreen'),
    ('DICTATE', 'Win+H'),    ('NOTIF', 'Win+N'),     ('QUICK', 'Win+A'),
    ('RUN', 'Win+R'),        ('WIDGET', 'Win+W'),    ('SETTING', 'Win+I'),
    ('SEARCH', 'Win+S'),
])

# --- Navegador ------------------------------------------------------------------
TABS_WEB = ('Ctrl+Tab', 'Ctrl+Shift+Tab', None)

Perfil('Web', 'Navegador (Chrome, Edge, Firefox)', 'Programación',
       procesos=['chrome.exe', 'msedge.exe', 'firefox.exe', 'brave.exe', 'opera.exe'],
       descripcion='Pestañas y navegación; y una página para YouTube.') \
.pagina('Pestañas', [
    ('TABPREV', 'Ctrl+Shift+Tab'), ('TABNEW', 'Ctrl+T'), ('TABNEXT', 'Ctrl+Tab'),
    ('NAVBACK', 'Alt+Left'), ('RELOAD', 'F5'),       ('NAVFWD', 'Alt+Right'),
    ('TABUNDO', 'Ctrl+Shift+T'), ('URL', 'Ctrl+L'),  ('TABCLOS', 'Ctrl+W'),
    ('FIND', 'Ctrl+F'),
], super_=[
    ('WINNEW', 'Ctrl+N'),    ('PRIVATE', 'Ctrl+Shift+N'), ('DOWNLD', 'Ctrl+J'),
    ('HISTORY', 'Ctrl+H'),   ('BOOKMRK', 'Ctrl+D'),  ('FULLSCR', 'F11'),
    ('ZOOMRST', 'Ctrl+0'),   ('DEVTOOL', 'F12'),     ('HRELOAD', 'Ctrl+Shift+R'),
    ('PRINT', 'Ctrl+P'),
], der=('Zoom', 'Zoom', 'Ctrl+0'), rueda_super=TABS_WEB[:2]) \
.pagina('YouTube', [
    ('YTBACK', 'J'),         ('PLAY', 'K'),          ('YTFWD', 'L'),
    ('YTPREV', 'Shift+P'),   ('FULLSCR', 'F'),       ('YTNEXT', 'Shift+N'),
    ('YTMUTE', 'M'),         ('CAPTION', 'C'),       ('YTSTART', '0'),
    ('THEATER', 'T'),
], super_=[
    ('TABPREV', 'Ctrl+Shift+Tab'), ('MINIPL', 'I'),  ('TABNEXT', 'Ctrl+Tab'),
    ('NAVBACK', 'Alt+Left'), ('RELOAD', 'F5'),       ('NAVFWD', 'Alt+Right'),
    ('HISTORY', 'Ctrl+H'),   ('URL', 'Ctrl+L'),      ('TABCLOS', 'Ctrl+W'),
    ('TABUNDO', 'Ctrl+Shift+T'),
], der=('Left', 'Right', 'K'))

# ============================================================================
# OFFICE (en español)
# ============================================================================

# --- Excel --------------------------------------------------------------------
HOJAS = ('Ctrl+PageDown', 'Ctrl+PageUp', None)

Perfil('Excel', 'Microsoft Excel', 'Office', procesos=['excel.exe'],
       descripcion='Edición de celdas; filas, columnas y hojas.') \
.pagina('Edición', [
    ('COPY', 'Ctrl+C'),      ('PAST', 'Ctrl+V'),     ('PASTESP', 'Ctrl+Alt+V'),
    ('EDITCEL', 'F2'),       ('FILLDN', 'Ctrl+J'),   ('ABSREF', 'F4'),
    UNDO,                    ('AUTOSUM', 'Alt+Shift+0'), REDO_Y,
    ('SAVE', 'Ctrl+G'),
], super_=[
    ('FMTCELL', 'Ctrl+1'),   ('BOLD', 'Ctrl+N'),     ('FILTER', 'Ctrl+Shift+L'),
    ('CELLNL', 'Alt+Enter'), ('NOTE', 'Shift+F2'),   ('FILLRT', 'Ctrl+D'),
    ('FIND', 'Ctrl+B'),      ('RECALC', 'F9'),       ('REPLACE', 'Ctrl+L'),
    ('CHART', 'Alt+F1'),
], der=HOJAS, rueda_super=('ScrollH', 'ScrollH')) \
.pagina('Filas y hojas', [
    ('SELROW', 'Shift+Space'), ('INSROW', 'Ctrl+KP+'), ('SELCOL', 'Ctrl+Space'),
    ('HIDEROW', 'Ctrl+9'),   ('DELROW', 'Ctrl+KP-'), ('GOTO', 'Ctrl+I'),
    ('SHEETPV', 'Ctrl+PageUp'), ('NEWSHT', 'Shift+F11'), ('SHEETNX', 'Ctrl+PageDown'),
    ('TOEND', 'Ctrl+End'),
], super_=[
    ('TOSTART', 'Ctrl+Home'), ('ALL', 'Ctrl+E'),     ('PRINT', 'Ctrl+P'),
    ('COPY', 'Ctrl+C'),      ('PAST', 'Ctrl+V'),     ('CUT', 'Ctrl+X'),
    UNDO,                    ('FIND', 'Ctrl+B'),     REDO_Y,
    ('SAVE', 'Ctrl+G'),
], der=HOJAS, rueda_super=('ScrollH', 'ScrollH'))

# --- Word ---------------------------------------------------------------------
Perfil('Word', 'Microsoft Word', 'Office', procesos=['winword.exe'],
       descripcion='Formato de texto; documento y revisión.') \
.pagina('Texto', [
    ('BOLD', 'Ctrl+N'),      ('ITALIC', 'Ctrl+K'),   ('UNDERLN', 'Ctrl+S'),
    ('ALIGNL', 'Ctrl+Q'),    ('ALIGNC', 'Ctrl+T'),   ('ALIGNR', 'Ctrl+D'),
    UNDO,                    ('JUSTIFY', 'Ctrl+J'),  REDO_Y,
    ('SAVE', 'Ctrl+G'),
], super_=[
    ('COPYFMT', 'Ctrl+Shift+C'), ('PASTFMT', 'Ctrl+Shift+V'), ('CLRFMT', 'Ctrl+Space'),
    ('HEAD1', 'Ctrl+Alt+1'), ('HEAD2', 'Ctrl+Alt+2'), ('HEAD3', 'Ctrl+Alt+3'),
    ('HLINK', 'Ctrl+Alt+K'), ('PGBREAK', 'Ctrl+Enter'), ('FIND', 'Ctrl+B'),
    ('REPLACE', 'Ctrl+L'),
]) \
.pagina('Documento', [
    ('NEWDOC', 'Ctrl+U'),    ('OPEN', 'Ctrl+A'),     ('PRINT', 'Ctrl+P'),
    ('SPELL', 'F7'),         ('THESAUR', 'Shift+F7'), ('GOTO', 'Ctrl+I'),
    ('TOSTART', 'Ctrl+Home'), ('ALL', 'Ctrl+E'),     ('TOEND', 'Ctrl+End'),
    ('CLOSEDC', 'Ctrl+F4'),
], super_=[
    ('COPY', 'Ctrl+C'),      ('PAST', 'Ctrl+V'),     ('CUT', 'Ctrl+X'),
    ('BOLD', 'Ctrl+N'),      ('ITALIC', 'Ctrl+K'),   ('UNDERLN', 'Ctrl+S'),
    UNDO,                    ('SAVE', 'Ctrl+G'),     REDO_Y,
    ('FIND', 'Ctrl+B'),
])

# --- PowerPoint -----------------------------------------------------------------
Perfil('PPoint', 'Microsoft PowerPoint', 'Office', procesos=['powerpnt.exe'],
       descripcion='Presentar con puntero y tinta; con SUPER, editar diapositivas.') \
.pagina('Presentar', [
    ('FROMBEG', 'F5'),       ('FROMCUR', 'Shift+F5'), ('PRESVW', 'Alt+F5'),
    ('PREVSLD', 'PageUp'),   ('BLACK', 'Punto'),     ('NEXTSLD', 'PageDown'),
    ('LASER', 'Ctrl+L'),     ('PEN', 'Ctrl+P'),      ('ERASINK', 'E'),
    ('ENDSHOW', 'Esc'),
], super_=[
    ('NEWSLD', 'Ctrl+M'),    ('BOLD', 'Ctrl+N'),     ('ITALIC', 'Ctrl+K'),
    ('ALIGNL', 'Ctrl+Q'),    ('ALIGNC', 'Ctrl+T'),   ('ALIGNR', 'Ctrl+D'),
    UNDO,                    ('COPYFMT', 'Ctrl+Shift+C'), REDO_Y,
    ('SAVE', 'Ctrl+G'),
], der=('PageDown', 'PageUp', None))

# ============================================================================
# CREATIVOS
# ============================================================================

# --- Photoshop ------------------------------------------------------------------
# Zoom con Ctrl + las teclas + y - del teclado español; con SUPER, el mando
# derecho recorre el historial.
ZOOM_PS = ('Ctrl+Mas', 'Ctrl+Menos', 'Ctrl+0')
HISTORIA_PS = ('Ctrl+Shift+Z', 'Ctrl+Z', None)

Perfil('PShop', 'Adobe Photoshop', 'Creativos', procesos=['photoshop.exe'],
       descripcion='Herramientas, capas y ajustes de imagen.') \
.pagina('Herramientas', [
    ('MOVE', 'V'),           ('MARQ', 'M'),          ('LASS', 'L'),
    ('QSELECT', 'W'),        ('CROP', 'C'),          ('EYE', 'I'),
    ('BRSH', 'B'),           ('ERAS', 'E'),          ('HEAL', 'J'),
    ('CLONE', 'S'),
], super_=[
    ('PEN', 'P'),            ('TEXT', 'T'),          ('SHAP', 'U'),
    ('GRAD', 'G'),           ('DODGE', 'O'),         ('PATH', 'A'),
    ('HAND', 'H'),           ('ZOOM', 'Z'),          ('ROTVIEW', 'R'),
    ('QMASK', 'Q'),
], der=ZOOM_PS, der_super=HISTORIA_PS, rueda_super=('ScrollH', 'ScrollH')) \
.pagina('Capas', [
    ('NEWL', 'Ctrl+Shift+N'), ('DUPL', 'Ctrl+J'),    ('MERGE', 'Ctrl+E'),
    ('GROUP', 'Ctrl+G'),     ('CLIPMSK', 'Ctrl+Alt+G'), ('STAMP', 'Ctrl+Alt+Shift+E'),
    UNDO,                    ('TRANSF', 'Ctrl+T'),   REDO_SZ,
    ('SAVE', 'Ctrl+S'),
], super_=[
    ('DESELEC', 'Ctrl+D'),   ('INVSEL', 'Ctrl+Shift+I'), ('ALL', 'Ctrl+A'),
    ('FILLDLG', 'Shift+F5'), ('SWAPCLR', 'X'),       ('DEFCLR', 'D'),
    ('FIT', 'Ctrl+0'),       ('ZOOM100', 'Ctrl+1'),  ('SCRMODE', 'F'),
    ('SAVEAS', 'Ctrl+Shift+S'),
], der=ZOOM_PS, der_super=HISTORIA_PS, rueda_super=('ScrollH', 'ScrollH')) \
.pagina('Ajustes', [
    ('LEVELS', 'Ctrl+L'),    ('CURVES', 'Ctrl+M'),   ('HUESAT', 'Ctrl+U'),
    ('COLORBL', 'Ctrl+B'),   ('INVERT', 'Ctrl+I'),   ('DESAT', 'Ctrl+Shift+U'),
    ('IMGSIZE', 'Ctrl+Alt+I'), ('CANVAS', 'Ctrl+Alt+C'), ('AUTOTN', 'Ctrl+Shift+L'),
    ('EXPORT', 'Ctrl+Alt+Shift+W'),
], super_=[
    ('AUTOCON', 'Ctrl+Alt+Shift+L'), ('AUTOCLR', 'Ctrl+Shift+B'), ('BW', 'Ctrl+Alt+Shift+B'),
    ('FIT', 'Ctrl+0'),       ('ZOOM100', 'Ctrl+1'),  ('TRANSF', 'Ctrl+T'),
    UNDO,                    ('SAVE', 'Ctrl+S'),     REDO_SZ,
    ('SAVEAS', 'Ctrl+Shift+S'),
], der=ZOOM_PS, der_super=HISTORIA_PS, rueda_super=('ScrollH', 'ScrollH'))

# --- Premiere Pro ---------------------------------------------------------------
# Mando derecho: fotograma a fotograma, clic para reproducir. Con SUPER, salta
# de corte en corte. Rueda con SUPER: zoom de la línea de tiempo.
JOG = ('Right', 'Left', 'Space')
CORTES = ('Down', 'Up', 'Space')
ZOOM_LINEA = ('Mas', 'Menos')

Perfil('Premier', 'Adobe Premiere Pro', 'Creativos', procesos=['adobe premiere pro.exe'],
       descripcion='Montaje en la línea de tiempo y herramientas.') \
.pagina('Montaje', [
    ('CUT', 'C'),            ('SELC', 'V'),          ('ADDEDIT', 'Ctrl+K'),
    ('TRIMPRV', 'Q'),        ('RIPL', 'Shift+Delete'), ('TRIMNXT', 'W'),
    UNDO,                    ('MARK', 'M'),          REDO_SZ,
    ('PLAY', 'Space'),
], super_=[
    ('MARKIN', 'I'),         ('MARKCLP', 'X'),       ('MARKOUT', 'O'),
    ('PREVEDT', 'Up'),       ('MATCHFR', 'F'),       ('NEXTEDT', 'Down'),
    ('LINK', 'Ctrl+L'),      ('SNAPTL', 'S'),        ('SPED', 'Ctrl+R'),
    ('RENDER', 'Enter'),
], der=JOG, der_super=CORTES, rueda_super=ZOOM_LINEA) \
.pagina('Herramientas', [
    ('RIPEDIT', 'B'),        ('ROLL', 'N'),          ('RATE', 'R'),
    ('SLIP', 'Y'),           ('SLID', 'U'),          ('TRKSEL', 'A'),
    ('PEN', 'P'),            ('HAND', 'H'),          ('TEXT', 'T'),
    ('ZOOM', 'Z'),
], super_=[
    ('VTRANS', 'Ctrl+D'),    ('ATRANS', 'Ctrl+Shift+D'), ('BTRANS', 'Shift+D'),
    ('EXPFRM', 'Ctrl+Shift+E'), ('EXPO', 'Ctrl+M'),  ('SAVE', 'Ctrl+S'),
    UNDO,                    ('PLAY', 'Space'),      REDO_SZ,
    ('RENDER', 'Enter'),
], der=JOG, der_super=CORTES, rueda_super=ZOOM_LINEA)
