# Iconos de los juegos y de sus menús. Mismas reglas que icons.py: lienzo de 24x22,
# siluetas rellenas con los detalles en hueco (como el mando de icons_apps.py, que
# también se usa aquí). Este fichero lo importa icons.py al final.
#
# Para los juegos que ya traen su icono hecho —el mando de
# «Juegos», la flecha circular de «Otra vez» y la puerta de «Salir»— no se dibuja nada:
# catalog_juegos.py los apunta desde su etiqueta.
import math
from icons import icon


@icon('mole')
def _(c):
    # Un topo de perfil asomando del agujero. De frente parecía un osito (con orejas) o una
    # foca (sin ellas): lo que lo hace topo es el hocico largo en punta con la nariz al
    # final, el ojillo y la manaza de cavar con sus garras.
    c.dot(9, 9, 6.5)
    c.fill(2.5, 9, 15.5, 17)
    c.poly((13, 4), (20.5, 9.5), (20.5, 12), (13, 14.5))           # el hocico
    c.dot(21.8, 10.8, 1.7)                                         # la nariz
    c.dot(12.5, 7, 1, v=0)                                         # el ojillo
    c.clear(0, 17, 24, 22)
    c.ellipse(10, 19.2, 9.5, 2.2, w=1.6)                           # la boca del agujero
    c.dot(16.5, 16.2, 2.6)                                         # la mano, con garras
    for dy in (-1.8, 0, 1.8):
        c.line((18, 16.2 + dy), (21.5, 15 + dy), w=1.1)


@icon('simon')
def _(c):
    # El Simon de verdad: un disco partido en cuatro cuartos con el centro hueco. Uno
    # encendido (macizo) y tres apagados (en contorno), como a mitad de una secuencia.
    cx, cy, R, r = 12, 11, 10.5, 3.5
    c.dot(cx, cy, R)
    c.dot(cx, cy, R - 2, v=0)
    c.poly((cx, cy), *[(cx + R * math.cos(math.radians(a)), cy + R * math.sin(math.radians(a)))
                       for a in range(180, 271, 6)])
    c.dot(cx, cy, r + 2)
    c.dot(cx, cy, r, v=0)
    c.clear(11, 0, 13, 22)                # las separaciones entre cuartos
    c.clear(0, 10, 24, 12)


@icon('radio')
def _(c):
    # Una radio antigua: el altavoz con su rejilla a la izquierda, el dial con la aguja a la
    # derecha y la antena. «Sintoniza» es girar ese dial hasta la emisora.
    c.line((6, 5), (18, 0.5), w=1.5)                 # la antena
    c.fill(0, 5, 24, 21, r=3)
    c.dot(7, 13, 4.6, v=0)                           # el altavoz
    c.dot(7, 13, 3.2)
    c.dot(7, 13, 1.8, v=0)
    c.fill(13, 8.5, 21.5, 17.5, r=1, v=0)            # la ventana del dial
    for x in (14.5, 16.5, 18.5, 20.5):
        c.fill(x - 0.5, 15.5, x + 0.5, 17.5)         # sus marcas
    c.fill(17, 9.5, 18.5, 17.5)                      # la aguja


@icon('reflejos')
def _(c):
    # Un cronómetro con un rayo dentro: medir lo rápido que respondes.
    c.fill(9.5, 0, 14.5, 2, r=0.5)                   # el pulsador de arriba
    c.fill(11, 1.5, 13, 4)
    c.line((18.5, 4.5), (20.5, 6.5), w=2)            # y el de al lado
    c.circle(12, 13, 8.2, w=2)
    c.poly((13.5, 6.5), (8.5, 14), (11.6, 14), (10.5, 19.5), (15.5, 11.8), (12.4, 11.8))


@icon('list')
def _(c):
    # Tres renglones con su viñeta: un menú.
    for y in (3, 10, 17):
        c.dot(3.8, y + 1.5, 1.9)
        c.fill(9, y, 23, y + 3, r=1)


@icon('missile')
def _(c):
    # Un misil de lado, de punta hacia la derecha. El de antes iba en vertical y ocupaba el
    # lienzo entero, y en la pantalla del menú se veía solo la mitad: tumbado y más pequeño
    # cabe con aire alrededor. Aletas despegadas un píxel del cuerpo (pegadas, todo se leía
    # como un bloque) y una franja hueca que lo separa de una bala.
    c.fill(5, 9, 17, 13)                                           # el cuerpo
    c.poly((17, 8.6), (17, 13.4), (21.5, 11))                      # la ojiva
    c.poly((4, 7.6), (10, 7.6), (3, 4))                            # las aletas
    c.poly((4, 14.4), (10, 14.4), (3, 18))
    c.clear(13, 9, 14, 13)                                         # la franja


@icon('cards')
def _(c):
    # Dos naipes abiertos en abanico: el de detrás asoma por la izquierda y el de delante,
    # macizo, lleva una pica en hueco. Un naipe solo se confundía con un documento.
    c.poly((1, 5), (9.5, 1.5), (13.5, 13), (5, 16.5))              # el de detrás
    c.poly((3, 6), (9, 3.5), (12, 12), (6, 14.5), v=0)
    c.fill(9, 3, 23, 22, r=2)                                      # el de delante
    c.clear(7.5, 1.5, 9, 22)                                       # aire entre los dos
    c.dot(13.8, 12.4, 2.6, v=0)                                    # la pica
    c.dot(18.2, 12.4, 2.6, v=0)
    c.poly((11.4, 11.4), (16, 6), (20.6, 11.4), v=0)
    c.poly((16, 13), (14, 18), (18, 18), v=0)
