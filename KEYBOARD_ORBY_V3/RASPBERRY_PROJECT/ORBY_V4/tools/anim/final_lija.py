"""Final «lija»: la O gira cada vez más rápido hasta descontrolarse y crece
hasta cruzar el teclado entero. Lo que toca de las letras salta en esquirlas, y
ella misma se va gastando como una lija hasta que no queda nada. Ver
orvy_comun.py para lo que va antes."""
import math
import random

from intro_dial import dial
from orvy_comun import (W, H, PITCH, S_ISO, S_TOP, screen_origin, screen_center,
                        placed, to_pixels, phase, hash01)

# El dial se pinta directamente en el plano común, porque crece hasta cruzar
# todo el teclado y no cabe en un sprite. Pero es siempre la misma figura,
# dial(), escalada entera: puntas redondas y trazo que engorda con ella. Con un
# anillo de trazo fijo y cortado a escuadra en el hueco, parecía que se estiraba.
R_RING = 30 * S_ISO          # radio del anillo a tamaño normal, en píxeles
HW_RING = 6.75 * S_ISO       # medio trazo a tamaño normal
# Crece hasta que el borde de dentro ha pasado entera la tecla 11.
R_MAX = (3 * PITCH + H + 40) * 30 / (30 - 6.75)

T_SPIN = 104           # «orvy» ya leído: el dial empieza a girar
T_GROW = 140           # se descontrola y empieza a crecer
T_GONE = 196           # el anillo ha salido del teclado
T_END = T_GONE + 10    # lo que tardan en apagarse las últimas esquirlas


def spin_track():
    """Ángulo, temblor del centro, radio y medio trazo de cada fotograma desde
    T_SPIN. Se calcula de una vez porque el ángulo se integra: lo que acelera
    es la velocidad, no el ángulo."""
    rnd = random.Random(5)
    rot, out = 0.0, []
    for f in range(T_SPIN, T_END):
        if f < T_GROW:
            p = phase(f, T_SPIN, T_GROW)
            # Acelera hasta rozar los 45° por fotograma: a 30 fps el ojo deja de
            # seguir el hueco y el giro se vuelve errático, que es el descontrol.
            w = 3 + 42 * p ** 2.2
            w *= 1 + rnd.uniform(-0.25, 0.25) * p
            shake = 2.2 * max(0.0, p - 0.45) / 0.55
            r, hw = R_RING, HW_RING
        else:
            p = min(1.0, phase(f, T_GROW, T_GONE))
            w = 45 * (1 + rnd.uniform(-0.3, 0.3))
            shake = 2.2 + 1.5 * min(1.0, p * 3)
            # Arranca despacio y enseguida barre a ritmo casi constante. Con un
            # crecimiento exponencial pasaba medio segundo invisible en el hueco
            # entre teclas y luego cruzaba las tres letras de golpe.
            r = R_RING + (R_MAX - R_RING) * p ** 1.3
            hw = HW_RING * r / R_RING
        rot += w
        out.append((rot, rnd.uniform(-shake, shake), rnd.uniform(-shake, shake), r, hw))
    return out


TRACK = spin_track()
LETTERS = sorted(to_pixels(placed(T_SPIN)[1:]))   # [0] es el dial


def reach_frame(d):
    """Primer fotograma en que el borde exterior del anillo llega a distancia d."""
    for k, (_, _, _, r, hw) in enumerate(TRACK):
        if r + hw >= d:
            return T_SPIN + k
    return T_GONE


_C = screen_center(S_TOP)
# Hasta el primer choque con una letra el dial gira entero; desde ahí se gasta.
T_HIT = reach_frame(min(math.hypot(x - _C[0], y - _C[1]) for x, y in LETTERS))


def wear(f):
    """Cuánto material ha perdido el anillo (0 entero, 1 nada). Sube desde el
    primer choque y llega a todo justo cuando sale del teclado."""
    if f < T_HIT:
        return 0.0
    return min(1.0, 0.12 + 0.88 * phase(f, T_HIT, T_GONE - 2) ** 1.4)


def ring_pixels(f, s):
    """Píxeles del dial en la pantalla s, en coordenadas del plano."""
    rot, jx, jy, r, hw = TRACK[f - T_SPIN]
    cx, cy = _C[0] + jx, _C[1] + jy
    ox, oy = screen_origin(s)
    # La pantalla entera dentro del agujero, o entera fuera del anillo: nada.
    near = math.hypot(max(ox - cx, 0, cx - ox - W), max(oy - cy, 0, cy - oy - H))
    far = max(math.hypot(x - cx, y - cy) for x in (ox, ox + W) for y in (oy, oy + H))
    if near > r + hw + 1 or far < r - hw - 1:
        return []
    shape = dial(rot)
    k = 1 / (S_ISO * r / R_RING)          # píxeles del plano -> unidades del SVG
    e = wear(f)
    cr, sr = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    # A tamaño normal, el mismo supermuestreo que el sprite: así el primer
    # fotograma girando coincide píxel a píxel con el último quieto.
    n = 4 if r < 20 else 2
    out = []
    for j in range(H):
        for i in range(W):
            x, y = ox + i, oy + j
            d0 = math.hypot(x + .5 - cx, y + .5 - cy)
            if abs(d0 - r) > hw + 1:
                continue
            hit = 0
            for a in range(n):
                for b in range(n):
                    if shape(50 + (x + (a + .5) / n - cx) * k, 50 + (y + (b + .5) / n - cy) * k):
                        hit += 1
            if hit * 2 < n * n:
                continue
            if e > 0:
                # Lija: el grano está pegado al dial (coordenadas giradas con él),
                # así que gira con el dial y, como no depende del tiempo, un
                # agujero solo crece. Mezcla motas gruesas y finas para que se
                # desprenda a trozos y no como ruido de televisor; por ángulo y
                # radio salían rayas.
                u = (x + .5 - cx) * cr + (y + .5 - cy) * sr
                v = -(x + .5 - cx) * sr + (y + .5 - cy) * cr
                g = 0.55 * hash01(int(u // 6), int(v // 6)) + 0.45 * hash01(int(u // 2) + 7919, int(v // 2))
                if g < e:
                    continue
            out.append((x, y))
    return out


class Chip:
    """Una esquirla: un píxel arrancado (de una letra o del propio anillo) que
    sale despedido en el sentido del giro y algo hacia fuera, y se apaga."""

    def __init__(self, x, y, f0, rnd, speed):
        a = math.atan2(y - _C[1], x - _C[0])
        v = speed * rnd.uniform(0.5, 1.2)
        self.x0, self.y0, self.f0 = x, y, f0
        # El giro va en el sentido de las agujas del reloj en pantalla (y hacia
        # abajo): la tangente es (-sen, cos).
        self.vx = -math.sin(a) * v + math.cos(a) * v * 0.4
        self.vy = math.cos(a) * v + math.sin(a) * v * 0.4
        self.life = rnd.randint(4, 12)

    def at(self, f):
        t = f - self.f0
        if t < 0 or t >= self.life:
            return None
        return round(self.x0 + self.vx * t), round(self.y0 + self.vy * t)


def destruction():
    """Cuándo muere cada píxel de letra y las esquirlas que se sueltan."""
    rnd = random.Random(11)
    gone, chips = {}, []
    for x, y in LETTERS:
        f = reach_frame(math.hypot(x - _C[0], y - _C[1])) + rnd.randint(0, 2)
        gone[(x, y)] = f
        if rnd.random() < 0.6:
            chips.append(Chip(x, y, f, rnd, 4 + 6 * phase(f, T_GROW, T_GONE)))
    # El anillo también suelta material por donde pasa, más cuanto más gastado.
    for f in range(T_HIT, T_GONE):
        _, _, _, r, hw = TRACK[f - T_SPIN]
        for _ in range(int(40 * wear(f))):
            a = rnd.uniform(0, 2 * math.pi)
            rr = r + rnd.uniform(-hw, hw)      # de todo el grosor, no solo del centro
            chips.append(Chip(round(_C[0] + rr * math.cos(a)), round(_C[1] + rr * math.sin(a)),
                              f, rnd, 5 + r / 30))
    return gone, chips


GONE, CHIPS = destruction()


def pixels_at(f):
    if f < T_SPIN:
        return to_pixels(placed(f))
    px = {p for p in LETTERS if GONE[p] > f}
    if f < T_GONE:
        for s in range(10):
            px.update(ring_pixels(f, s))
    for c in CHIPS:
        p = c.at(f)
        if p:
            px.add(p)
    return px
