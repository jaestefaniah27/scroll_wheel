"""Intros de arranque «orvy»: genera include/orby_intro_data.h con las tres.

Comparten los pasos 1 a 4 (orvy_comun.py: el dial nace en la tecla 5, sube a la
2 y entran r, v e y) y cambian en cómo se va todo:
  0. ventilador (final_ventilador.py): la O gira y su torbellino se traga las
     letras en espiral; luego se desintegra ella.
  1. viento (final_viento.py): cada píxel sale hacia fuera en su remolino.
  2. lija (final_lija.py): la O se descontrola, crece hasta cruzar el teclado
     destrozando las letras y se gasta hasta desaparecer.
El orden es el número que guarda el teclado (SET_INTRO), contando desde 1: el 0
es «una al azar cada vez», la de fábrica. Añadir una va al final, para no cambiar
la que ya tenga elegida nadie.

Las tres acaban con las diez pantallas en negro: lo que sigue (los iconos
abriéndose como con SUPER) lo dibuja el firmware, porque depende del perfil.

Uso:  python tools/anim/intro_orvy.py            (escribe el .h)
      python tools/anim/intro_orvy.py --preview  (además, un GIF por intro en
                                                  tools/anim/_preview/)
"""
import importlib
import sys

from intro_dial import ROOT, COL, ROW, W, H, rle
from orvy_comun import FPS, screens

# A cuántos fotogramas por segundo se hornea. El guion va en fotogramas de 30
# (FPS) y se muestrea aquí más fino. Los paneles barren a unos 170 Hz (oscilador
# por defecto, 40 filas) y el SPI a 10 MHz repinta las diez en 3 ms, así que 120
# es lo más alto que todavía se ve. El precio es Flash: cada vez que se dobla,
# las intros ocupan casi el doble (a 120, unos 470 KB las tres).
OUT_FPS = 120

FINALES = ['ventilador', 'viento', 'lija']


def frames_of(fin):
    return round(fin.T_END * OUT_FPS / FPS)


def bake(fin):
    stream, prev, frames = bytearray(), [None] * 10, []
    blank = bytes(360)
    n = frames_of(fin)
    for k in range(n):
        f = k * FPS / OUT_FPS
        scr = screens(fin.pixels_at(f)) if f < fin.T_END - 2 else [blank] * 10
        frames.append(scr)
        mask, payload = 0, bytearray()
        for s in range(10):
            if scr[s] == prev[s]:
                continue
            mask |= 1 << s
            enc = rle(scr[s])
            payload += len(enc).to_bytes(2, 'little') + enc
            prev[s] = scr[s]
        stream += mask.to_bytes(2, 'little') + payload
    return stream, frames


def header(clips):
    total = sum(len(s) for _, s, _ in clips)
    lines = [
        '// ============================================================',
        '// Intros de arranque de Orvy — GENERADO, no editar a mano',
        '// Creado con tools/anim/intro_orvy.py (la geometría es la de',
        '// OrbyGUI/assets/brand/): python tools/anim/intro_orvy.py',
    ]
    for i, (name, s, n) in enumerate(clips):
        lines.append(f'//   {i} {name}: {n} fotogramas ({n / OUT_FPS:.2f} s), {len(s)} bytes')
    lines += [
        f'// {total} bytes en total',
        '// ============================================================',
        '#pragma once',
        '#include <stdint.h>',
        '',
        f'#define INTRO_FPS   {OUT_FPS}',
        f'#define INTRO_COUNT {len(clips)}',
        '',
        '// Por fotograma: uint16 con las pantallas que cambian (bit 0 = OLED 1) y,',
        '// por cada una, uint16 de longitud y su framebuffer comprimido en pares',
        '// (repeticiones, valor). El último deja las diez en negro: el firmware',
        '// abre encima los iconos de verdad.',
    ]
    for i, (name, s, _) in enumerate(clips):
        lines.append(f'static const uint8_t intro_stream_{i}[] = {{  // {name}')
        for k in range(0, len(s), 16):
            lines.append('    ' + ','.join(f'0x{b:02x}' for b in s[k:k + 16]) + ',')
        lines += ['};', '']
    lines += [
        'struct IntroClip { const uint8_t* data; uint32_t size; uint16_t frames; };',
        '',
        'static const IntroClip intro_clips[INTRO_COUNT] = {',
    ]
    for i, (name, _, n) in enumerate(clips):
        lines.append(f'    {{ intro_stream_{i}, sizeof(intro_stream_{i}), {n} }},  // {name}')
    lines += ['};', '']
    return '\n'.join(lines)


def gif(name, frames):
    from PIL import Image
    out = ROOT / 'tools' / 'anim' / '_preview'
    out.mkdir(exist_ok=True)
    gx, gy, k = 24, 20, 3
    cw, ch = 3 * (W + gx), 4 * (H + gy)
    ims = []
    for scr in frames:
        im = Image.new('RGB', (cw, ch), (10, 12, 14))
        px = im.load()
        for s in range(10):
            ox, oy = COL[s] * (W + gx) + gx // 2, ROW[s] * (H + gy) + gy // 2
            for y in range(H):
                for x in range(W):
                    on = (scr[s][(y >> 3) * W + x] >> (y & 7)) & 1
                    px[ox + x, oy + y] = (157, 233, 255) if on else (2, 3, 4)
        ims.append(im.resize((cw * k, ch * k), Image.NEAREST))
    dst = out / f'intro_{name}.gif'
    ims[0].save(dst, save_all=True, append_images=ims[1:], duration=round(1000 / OUT_FPS), loop=0)
    print(f'vista previa en {dst}')


if __name__ == '__main__':
    clips = []
    for name in FINALES:
        fin = importlib.import_module(f'final_{name}')
        stream, frames = bake(fin)
        clips.append((name, stream, frames_of(fin)))
        print(f'{name}: {frames_of(fin)} fotogramas, {len(stream)} bytes')
        if '--preview' in sys.argv:
            gif(name, frames)
    dst = ROOT / 'include' / 'orby_intro_data.h'
    dst.write_text(header(clips), encoding='utf-8', newline='\n')
    print(f'{dst}: {sum(len(s) for _, s, _ in clips)} bytes')
