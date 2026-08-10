#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Valida la disposicion de la Flash y la aritmetica del banco de iconos.

Es lo unico de la mudanza del banco que se puede comprobar sin el teclado
delante, y es donde un error sale caro: un tramo mal calculado se pisa los
iconos de otra pagina, o los ajustes, o se sale del modulo. El firmware ya
lleva static_assert para lo peor, pero aqui se comprueba tambien lo que un
assert no ve: que los 64 tramos son disjuntos y que la migracion no lee de
donde ya ha escrito.
"""
import re, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..', 'main.cpp')
src = open(SRC, encoding='utf-8', errors='replace').read()

def define(name, default=None):
    m = re.search(r'#define\s+%s\s+\(?([0-9]+)\s*\*?\s*([0-9]*)\)?' % name, src)
    if not m:
        if default is not None:
            return default
        print('No encuentro #define %s' % name); sys.exit(1)
    a = int(m.group(1))
    b = m.group(2)
    return a * int(b) if b else a

K = 1024
FLASH_SIZE      = 2048 * K          # modulo del Pico
SECTOR          = 4096
MAX_PROFILES    = define('MAX_PROFILES')
MAX_PAGES       = define('MAX_PAGES')
OLED_PAGE_STRIDE = define('OLED_PAGE_STRIDE')
OLED_OFFSET     = define('FLASH_OLED_OFFSET')
SETTINGS_OFFSET = define('FLASH_TARGET_OFFSET')
SETTINGS_SECT   = define('FLASH_SETTINGS_SECTORS')
SLICE           = 2 * SECTOR        # FLASH_OLED_SLICE

SETTINGS_BYTES = SETTINGS_SECT * SECTOR
BANK_SLICES    = MAX_PROFILES * MAX_PAGES

def slice_at(bank, page):
    return OLED_OFFSET + ((bank * MAX_PAGES) + page) * SLICE

print('perfiles=%d  paginas=%d  tramos=%d de %d B' % (MAX_PROFILES, MAX_PAGES, BANK_SLICES, SLICE))
print('banco   : %d - %d KB' % (OLED_OFFSET // K, (slice_at(MAX_PROFILES - 1, MAX_PAGES - 1) + SLICE) // K))
print('ajustes : %d - %d KB' % (SETTINGS_OFFSET // K, (SETTINGS_OFFSET + SETTINGS_BYTES) // K))

errors = []

# 1. Los tramos no se pisan entre si y estan alineados a sector.
seen = {}
for b in range(MAX_PROFILES):
    for pg in range(MAX_PAGES):
        a = slice_at(b, pg)
        if a % SECTOR:
            errors.append('el tramo (%d,%d) no esta alineado a sector' % (b, pg))
        if a in seen:
            errors.append('los tramos (%d,%d) y %s comparten direccion' % (b, pg, seen[a]))
        seen[a] = (b, pg)

# 2. El tramo cubre los 21 bitmaps (20 de tecla + el icono del perfil) y cabe
#    en su hueco.
if OLED_PAGE_STRIDE < 21 * 360:
    errors.append('el stride (%d) no cubre los 21 bitmaps de 360 B' % OLED_PAGE_STRIDE)
if OLED_PAGE_STRIDE > SLICE:
    errors.append('el stride (%d) no cabe en el tramo de %d B' % (OLED_PAGE_STRIDE, SLICE))
if OLED_PAGE_STRIDE % 256:
    errors.append('el stride no es multiplo de pagina: flash_range_program lo exige')

# 3. El banco no se come los ajustes ni se sale de la Flash.
bank_end = slice_at(MAX_PROFILES - 1, MAX_PAGES - 1) + SLICE
if bank_end > SETTINGS_OFFSET:
    errors.append('el banco (acaba en %d KB) invade los ajustes (%d KB)'
                  % (bank_end // K, SETTINGS_OFFSET // K))
if SETTINGS_OFFSET + SETTINGS_BYTES > FLASH_SIZE:
    errors.append('los ajustes se salen de los 2 MB')

# 4. Hueco para el programa. El binario ocupa ~120 KB; si el banco baja
#    demasiado, se lo come.
elf = os.path.join(HERE, '..', '..', 'build', 'ORBY_V4.elf')
if os.path.exists(elf):
    size = os.path.getsize(os.path.join(HERE, '..', '..', 'build', 'ORBY_V4.bin')) \
        if os.path.exists(os.path.join(HERE, '..', '..', 'build', 'ORBY_V4.bin')) else 0
    if size:
        print('binario : %d KB' % (size // K))
        if size > OLED_OFFSET:
            errors.append('el binario (%d KB) pisa el banco (%d KB)' % (size // K, OLED_OFFSET // K))
        elif size > OLED_OFFSET * 0.5:
            errors.append('el binario ocupa mas de la mitad del hueco hasta el banco')

# 5. La migracion: el banco viejo (detras de los ajustes) tiene que ser disjunto
#    del nuevo, o copiar de uno a otro se destruiria a si mismo.
for old_off, name in ((SETTINGS_OFFSET + 2 * SECTOR, '3.1'), (SETTINGS_OFFSET + SECTOR, '3.0')):
    old_end = old_off + MAX_PROFILES * SLICE
    if not (old_end <= OLED_OFFSET or old_off >= bank_end):
        errors.append('el banco viejo de la %s (%d-%d KB) se solapa con el nuevo'
                      % (name, old_off // K, old_end // K))
    # Y la region de ajustes nueva SI pisa el banco viejo: de ahi que haya que
    # copiar los iconos antes de reescribir los ajustes.
    if not (old_end <= SETTINGS_OFFSET or old_off >= SETTINGS_OFFSET + SETTINGS_BYTES):
        print('  nota: los ajustes nuevos pisan el banco viejo de la %s,' % name)
        print('        asi que la migracion tiene que copiar los iconos primero.')

# 6. Que la migracion lo haga en ese orden, comprobado en el codigo.
mig = re.search(r'migrate_oled_bank\(.*?\);\s*\n\s*//[^\n]*\n\s*//[^\n]*\n\s*save_settings\(\);', src, re.S)
if not mig:
    errors.append('no veo migrate_oled_bank() seguido de save_settings(): '
                  'si se graban los ajustes antes de copiar, se pierden los iconos')

if errors:
    print()
    for e in errors:
        print('  FALLA %s' % e)
    sys.exit(1)
print('\n  ok   disposicion de la Flash coherente (%d tramos disjuntos)' % BANK_SLICES)
