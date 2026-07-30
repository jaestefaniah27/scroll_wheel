#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Valida el descriptor HID del raton en src/usb_descriptors.c.

Un descriptor mal formado no da un error bonito: Windows rechaza el dispositivo
entero o lee el informe descuadrado. Aqui se comprueba lo que se puede
comprobar sin enchufar nada:

  - Que las Collection y End Collection cuadran.
  - Que el tamano del informe de entrada coincide con orby_mouse_report_t.
  - Que el Feature report (los multiplicadores) suma un byte exacto.
  - Que hay DOS multiplicadores de resolucion, uno por eje, cada uno dentro de
    su propia Logical Collection, que es lo que Windows exige para dar alta
    resolucion en ese eje.
"""
import re, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..', 'src', 'usb_descriptors.c')
HDR = os.path.join(HERE, '..', '..', 'include', 'hid_hires.h')

# --- Sacar los bytes del macro ---------------------------------------------
text = open(SRC, encoding='utf-8', errors='replace').read()
m = re.search(r'#define ORBY_HID_REPORT_DESC_MOUSE_HIRES(.*?)\n\n', text, re.S)
if not m:
    print('No encuentro el macro del descriptor'); sys.exit(1)

body = m.group(1)
# Fuera los comentarios primero, y en bloque: alguno ocupa varias lineas y
# lleva dentro cosas como «Usage 0x48» que se colarian como bytes.
body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
tokens = []
for line in body.splitlines():
    line = line.replace('\\', '')
    for tok in line.split(','):
        tok = tok.strip()
        if not tok:
            continue
        if tok == 'REPORT_ID_MOUSE':
            tokens.append(2)
        elif re.fullmatch(r'0x[0-9a-fA-F]+', tok):
            tokens.append(int(tok, 16))
        else:
            print('Token que no entiendo: %r' % tok); sys.exit(1)

# --- Recorrer los items -----------------------------------------------------
i = 0
depth = 0
report_size = 0
report_count = 0
input_bits = 0
feature_bits = 0
collections = []          # pila de tipos de collection
res_mult_in_logical = 0   # multiplicadores dentro de una Logical Collection
usage_pending = []
errors = []

while i < len(tokens):
    b = tokens[i]
    size = b & 0x03
    if size == 3:
        size = 4
    typ = (b >> 2) & 0x03
    tag = (b >> 4) & 0x0F
    data = 0
    for k in range(size):
        data |= tokens[i + 1 + k] << (8 * k)
    i += 1 + size

    if typ == 1:  # Global
        if tag == 0x7:   report_size = data
        elif tag == 0x9: report_count = data
    elif typ == 2:  # Local
        if tag == 0x0:   usage_pending.append(data)
    elif typ == 0:  # Main
        if tag == 0xA:   # Collection
            depth += 1
            collections.append(data)
        elif tag == 0xC: # End Collection
            depth -= 1
            if depth < 0:
                errors.append('End Collection de mas')
            collections.pop() if collections else None
        elif tag == 0x8: # Input
            input_bits += report_size * report_count
        elif tag == 0xB: # Feature
            feature_bits += report_size * report_count
            # Resolution Multiplier = Usage 0x48 dentro de Collection Logical (0x02)
            if 0x48 in usage_pending and collections and collections[-1] == 0x02:
                res_mult_in_logical += 1
        usage_pending = []

# --- Tamano de orby_mouse_report_t ------------------------------------------
hdr = open(HDR, encoding='utf-8', errors='replace').read()
struct = re.search(r'typedef struct.*?\{(.*?)\}\s*orby_mouse_report_t', hdr, re.S).group(1)
widths = {'uint8_t': 8, 'int8_t': 8, 'int16_t': 16, 'uint16_t': 16}
struct_bits = 0
for line in struct.splitlines():
    line = re.sub(r'//.*', '', line).strip()
    if not line:
        continue
    t = line.split()[0]
    if t in widths:
        struct_bits += widths[t]

# --- Veredicto --------------------------------------------------------------
print('Descriptor del raton: %d bytes' % len(tokens))
print('  informe de entrada : %d bits (%d bytes)' % (input_bits, input_bits // 8))
print('  orby_mouse_report_t: %d bits (%d bytes)' % (struct_bits, struct_bits // 8))
print('  feature report     : %d bits' % feature_bits)
print('  multiplicadores    : %d' % res_mult_in_logical)

if depth != 0:
    errors.append('Collections descuadradas (quedan %d abiertas)' % depth)
# El Report ID ocupa un byte que no se declara en el descriptor pero si viaja
# en el informe; orby_mouse_report_t no lo incluye porque lo pone TinyUSB.
if input_bits != struct_bits:
    errors.append('El informe declarado (%d bits) no coincide con la struct (%d bits)'
                  % (input_bits, struct_bits))
if feature_bits % 8 != 0:
    errors.append('El feature report no suma bytes enteros (%d bits)' % feature_bits)
if feature_bits != 8:
    errors.append('El feature report deberia ocupar 8 bits, ocupa %d' % feature_bits)
if res_mult_in_logical != 2:
    errors.append('Deberia haber 2 multiplicadores (vertical y horizontal), hay %d'
                  % res_mult_in_logical)

if errors:
    print()
    for e in errors:
        print('  FALLA %s' % e)
    sys.exit(1)

print('\n  ok   descriptor coherente')
