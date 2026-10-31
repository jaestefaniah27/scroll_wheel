"""Final «viento»: cada píxel de «orvy» se suelta y sale hacia fuera en su
propio remolino, cruzando a las pantallas vecinas si llega, y se apaga. Ver
orvy_comun.py para lo que va antes."""
import math
import random

from orvy_comun import placed, to_pixels

T_WIND = 108           # empieza el viento
WIND_LEN = 54          # lo que tarda el último píxel en irse
T_END = T_WIND + WIND_LEN + 4


class Grain:
    """Un píxel suelto por el viento. Sale de su sitio con un retraso propio,
    se aleja del centro de su letra (y de la columna, para que vaya hacia fuera
    del teclado) acelerando, y a la vez gira alrededor de un centro que se va
    con él: eso es el remolino. Cada uno con su giro, su sentido y su vida."""

    def __init__(self, x, y, rnd, home):
        self.x0, self.y0 = x, y
        hx, hy = home
        # Fuera respecto a su letra, con una ráfaga común hacia la derecha y algo
        # hacia arriba para que parezca viento y no una bomba.
        dx, dy = x - hx + rnd.uniform(-6, 6), y - hy + rnd.uniform(-6, 6)
        n = math.hypot(dx, dy) or 1
        self.ux = dx / n * 0.7 + 0.55
        self.uy = dy / n * 0.7 - 0.25
        # Los píxeles de barlovento (izquierda) se van antes: el viento llega de
        # ese lado y va deshaciendo la letra.
        self.delay = (x - hx + 36) / 72 * 14 + rnd.uniform(0, 12)
        self.acc = rnd.uniform(0.10, 0.22)
        self.v0 = rnd.uniform(0.3, 1.2)
        self.w = rnd.choice((-1, 1)) * rnd.uniform(0.12, 0.32)
        self.r = rnd.uniform(3, 11)
        self.ph = rnd.uniform(0, 2 * math.pi)
        self.life = rnd.uniform(22, 36)

    def at(self, t):
        """Posición t fotogramas después de que empiece el viento, o None si ya
        se ha apagado. Antes de soltarse sigue en su sitio."""
        t -= self.delay
        if t <= 0:
            return self.x0, self.y0
        if t >= self.life:
            return None
        # En el último tercio de la vida parpadea cada vez más hasta apagarse:
        # en una pantalla de 1 bit no hay otro modo de desvanecerse.
        fade = (t - self.life * 2 / 3) / (self.life / 3)
        if fade > 0 and math.sin(t * 2.7 + self.ph * 5) < fade * 2 - 1:
            return False
        d = self.v0 * t + self.acc * t * t
        r = self.r * min(1.0, t / 10)
        a = self.ph + self.w * t
        x = self.x0 + self.ux * d + r * (math.cos(a) - math.cos(self.ph))
        y = self.y0 + self.uy * d + r * (math.sin(a) - math.sin(self.ph))
        return round(x), round(y)


def make_grains():
    rnd = random.Random(27)
    items = placed(T_WIND)
    homes = [(round(x), round(y)) for _, x, y in items]
    grains, seen = [], set()
    for (pts, _, _), home in zip(items, homes):
        for i, j in pts:
            p = (home[0] + i, home[1] + j)
            if p not in seen:
                seen.add(p)
                grains.append(Grain(p[0], p[1], rnd, home))
    return grains


GRAINS = make_grains()


def pixels_at(f):
    if f < T_WIND:
        return to_pixels(placed(f))
    px = set()
    for g in GRAINS:
        p = g.at(f - T_WIND)
        if p:
            px.add(p)
    return px
