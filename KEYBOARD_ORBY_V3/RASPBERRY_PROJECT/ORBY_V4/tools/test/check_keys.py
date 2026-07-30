#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Comprueba que el firmware y la app coinciden en que tecla es SUPER y cual abre
el menu. Si se desincronizan, la app pinta los papeles al reves y el
interruptor NORMAL/SUPER del dashboard deja de seguir a la tecla real, que es
un fallo silencioso y muy molesto de encontrar.
"""
import re, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')

fw = open(os.path.join(ROOT, 'main.cpp'), encoding='utf-8', errors='replace').read()
gui = open(os.path.join(ROOT, 'OrbyGUI', 'src', 'store.js'), encoding='utf-8', errors='replace').read()

def one(pat, text, what):
    m = re.search(pat, text)
    if not m:
        print('No encuentro %s' % what); sys.exit(1)
    return int(m.group(1))

# El firmware va en base 0, la app numera como el usuario (1 a 12).
fw_super = one(r'#define\s+KEY_IDX_SUPER\s+(\d+)', fw, 'KEY_IDX_SUPER') + 1
fw_menu  = one(r'#define\s+KEY_IDX_MENU\s+(\d+)',  fw, 'KEY_IDX_MENU')  + 1
gui_super = one(r'KEY_SUPER\s*=\s*(\d+)', gui, 'KEY_SUPER')
gui_menu  = one(r'KEY_MENU\s*=\s*(\d+)',  gui, 'KEY_MENU')

print('firmware: SUPER=tecla %d, menu=tecla %d' % (fw_super, fw_menu))
print('app     : SUPER=tecla %d, menu=tecla %d' % (gui_super, gui_menu))

errors = []
if fw_super != gui_super:
    errors.append('SUPER no coincide: firmware %d, app %d' % (fw_super, gui_super))
if fw_menu != gui_menu:
    errors.append('el menu no coincide: firmware %d, app %d' % (fw_menu, gui_menu))
if fw_super == fw_menu:
    errors.append('SUPER y el menu estan en la misma tecla (%d)' % fw_super)

# Las dos teclas de sistema no pueden tener pantalla: no hay donde enseñar nada.
m = re.search(r'KEY_TO_SCREEN\s*=\s*\[([^\]]*)\]', gui)
screens = [int(x.strip()) for x in m.group(1).split(',')]
for name, k in (('SUPER', fw_super), ('menu', fw_menu)):
    if screens[k - 1] != 0:
        errors.append('la tecla de %s (%d) tiene pantalla asignada' % (name, k))


# --- Acciones especiales de tecla ---------------------------------------------
# Van en el campo del modificador y firmware y app tienen que usar los mismos
# codigos: si divergen, la app graba una accion que el teclado no reconoce y la
# tecla simplemente no hace nada, sin ningun error visible.
hid = open(os.path.join(ROOT, 'OrbyGUI', 'src', 'hid-keys.js'), encoding='utf-8', errors='replace').read()

def hexdef(pat, text, what):
    m = re.search(pat, text)
    if not m:
        print('No encuentro %s' % what); sys.exit(1)
    return int(m.group(1), 16)

acts = [
    ('KEYACT_CONSUMER',   r'#define\s+KEYACT_CONSUMER\s+0x([0-9A-Fa-f]+)',
     'CONSUMER_MODIFIER',   r'CONSUMER_MODIFIER\s*=\s*0x([0-9A-Fa-f]+)'),
    ('KEYACT_GOTO_PAGE',  r'#define\s+KEYACT_GOTO_PAGE\s+0x([0-9A-Fa-f]+)',
     'GOTO_PAGE_MODIFIER',  r'GOTO_PAGE_MODIFIER\s*=\s*0x([0-9A-Fa-f]+)'),
    ('KEYACT_PAGE_STATE', r'#define\s+KEYACT_PAGE_STATE\s+0x([0-9A-Fa-f]+)',
     'PAGE_STATE_MODIFIER', r'PAGE_STATE_MODIFIER\s*=\s*0x([0-9A-Fa-f]+)'),
]
print()
for fw_name, fw_pat, gui_name, gui_pat in acts:
    a = hexdef(fw_pat, fw, fw_name)
    b = hexdef(gui_pat, hid, gui_name)
    print('%-18s firmware 0x%02X   app 0x%02X' % (fw_name, a, b))
    if a != b:
        errors.append('%s no coincide: firmware 0x%02X, app 0x%02X' % (fw_name, a, b))

# El tope de paginas del firmware es el que la app ofrece en el selector.
mp = re.search(r'#define\s+MAX_PAGES\s+(\d+)', fw)
if not mp:
    errors.append('no encuentro MAX_PAGES en el firmware')
else:
    print('MAX_PAGES          firmware %s (la app lo lee del ACK)' % mp.group(1))

if errors:
    print()
    for e in errors:
        print('  FALLA %s' % e)
    sys.exit(1)
print('\n  ok   firmware y app de acuerdo')
