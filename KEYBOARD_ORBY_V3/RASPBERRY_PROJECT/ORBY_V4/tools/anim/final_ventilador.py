"""Final «ventilador»: la O gira, su torbellino se traga las letras en espiral y
al final se desintegra ella. Ver orvy_comun.py para lo que va antes."""
import math
import random

from intro_dial import dial
from orvy_comun import (W, H, S_ISO, S_TOP, screen_origin, screen_center, placed,
                        to_pixels, phase, hash01)

# La O se queda en la tecla 2 y se pone a girar cada vez más rápido. Su giro
# levanta un torbellino que arranca las letras píxel a píxel, de la más cercana
# a la más lejana, y se las traga en espiral. Cuando no queda ninguna, la propia
# O se desintegra y suelta lo que le queda en espiral hacia fuera.
#
# El dial se pinta directamente en el plano con su figura de siempre, dial(),
# porque las partículas cruzan pantallas y el giro necesita su ángulo exacto.
R_RING = 30 * S_ISO          # radio del anillo, en píxeles
HW_RING = 6.75 * S_ISO       # medio trazo

T_SPIN = 104           # «orvy» ya leído: el ventilador arranca
T_FULL = 128           # a toda velocidad; desde aquí ya tira de las letras
SUCK = [(T_FULL - 8, 14), (T_FULL, 14), (T_FULL + 8, 14)]   # r, v, y: inicio y duración del arranque
T_END = 240            # se recalcula abajo: lo que tarde en desaparecer todo


_C = screen_center(S_TOP)


class Swirl:
    """Un píxel arrancado por el torbellino. Gira alrededor de la O en el mismo
    sentido que ella y cae en espiral, cada vez más deprisa al acercarse (como
    el agua por un desagüe), hasta que la O se lo traga."""

    def __init__(self, x, y, f0, rnd, out=False):
        self.f0 = f0
        self.rho = math.hypot(x - _C[0], y - _C[1])
        self.th = math.atan2(y - _C[1], x - _C[0])
        self.out = out
        self.k = rnd.uniform(0.75, 1.3)          # cada uno a su ritmo
        self.wob = rnd.uniform(0, 2 * math.pi)
        self.seed = rnd.randrange(1 << 16)
        self.path = []
        rho, th, v = self.rho, self.th, 0.0
        for t in range(200):
            if out:
                # La O al romperse: sale despedida girando y se abre.
                v = min(v + 0.5 * self.k, 9)
                rho += v
                th += 0.32 * self.k * R_RING / max(rho, R_RING)
                if rho > R_RING + 70 or t > 26:
                    break
            else:
                # Hacia dentro acelerando; el giro crece al acercarse (con tope
                # para que a 30 fps se siga leyendo como espiral y no salte).
                v = min(v + 0.85 * self.k, 16)
                rho -= v * (0.6 + 0.4 * math.sin(t * 0.3 + self.wob) ** 2)
                th += min(0.55, 9.0 * self.k / max(rho, 1))
                if rho <= R_RING + HW_RING:
                    break
            self.path.append((round(_C[0] + rho * math.cos(th)), round(_C[1] + rho * math.sin(th))))

    @property
    def end(self):
        return self.f0 + len(self.path)

    def at(self, f):
        t = f - self.f0
        if t < 0 or t >= len(self.path):
            return None
        # Los de la O se apagan parpadeando al final: en 1 bit no hay fundido.
        if self.out and t > len(self.path) * 0.6 and hash01(self.seed, t) < 0.5:
            return None
        return self.path[t]


LETTERS = [to_pixels([it]) for it in placed(T_SPIN)[1:]]   # [0] es el dial: r, v, y


def suction():
    """Cuándo se suelta cada píxel de letra. Dentro de cada letra se van antes
    los más cercanos a la O: el torbellino la va deshaciendo desde ese lado."""
    rnd = random.Random(11)
    gone, swirls = {}, []
    for pts, (t0, dur) in zip(LETTERS, SUCK):
        ds = {p: math.hypot(p[0] - _C[0], p[1] - _C[1]) for p in pts}
        lo, hi = min(ds.values()), max(ds.values())
        for p in sorted(pts):
            f = t0 + round(dur * (0.65 * (ds[p] - lo) / (hi - lo or 1) + 0.35 * rnd.random()))
            gone[p] = f
            swirls.append(Swirl(p[0], p[1], f, rnd))
    return gone, swirls


GONE, SWIRLS = suction()
T_BREAK = max(s.end for s in SWIRLS) + 2       # ya no queda nada de las letras
T_DUST = T_BREAK + 16                           # la O ya no está
T_END = T_DUST + 24


def spin_track():
    """Ángulo y temblor de cada fotograma desde T_SPIN. Se integra porque lo que
    acelera es la velocidad, no el ángulo."""
    rnd = random.Random(5)
    rot, out = 0.0, []
    for f in range(T_SPIN, T_END):
        p = min(1.0, phase(f, T_SPIN, T_FULL))
        # Hasta 40° por fotograma: a 30 fps el hueco ya no se sigue y parece
        # un ventilador. Un poco de irregularidad para que no sea un patrón fijo.
        w = 3 + 37 * p ** 2 * (1 + rnd.uniform(-0.15, 0.15) * p)
        shake = 1.0 * p
        rot += w
        out.append((rot, rnd.uniform(-shake, shake), rnd.uniform(-shake, shake)))
    return out


TRACK = spin_track()


def wear(f):
    """Cuánto de la O se ha desintegrado (0 nada, 1 toda)."""
    return max(0.0, min(1.0, phase(f, T_BREAK, T_DUST)))


def dial_pixels(f):
    """Píxeles de la O girando, menos lo que ya se ha desprendido. El grano va
    pegado a ella (coordenadas giradas con el dial) y no depende del tiempo, así
    que un agujero solo crece."""
    rot, jx, jy = TRACK[f - T_SPIN]
    cx, cy = _C[0] + jx, _C[1] + jy
    ox, oy = screen_origin(S_TOP)
    shape = dial(rot)
    k = 1 / S_ISO
    e = wear(f)
    cr, sr = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    out = []
    for j in range(H):
        for i in range(W):
            x, y = ox + i, oy + j
            if abs(math.hypot(x + .5 - cx, y + .5 - cy) - R_RING) > HW_RING + 1:
                continue
            # El mismo supermuestreo que el sprite: el primer fotograma girando
            # coincide con el último quieto.
            hit = sum(shape(50 + (x + (a + .5) / 4 - cx) * k, 50 + (y + (b + .5) / 4 - cy) * k)
                      for a in range(4) for b in range(4))
            if hit * 2 < 16:
                continue
            if e > 0:
                u = (x + .5 - cx) * cr + (y + .5 - cy) * sr
                v = -(x + .5 - cx) * sr + (y + .5 - cy) * cr
                if 0.5 * hash01(int(u // 4), int(v // 4)) + 0.5 * hash01(int(u // 1.5) + 7919, int(v // 1.5)) < e:
                    continue
            out.append((x, y))
    return out


def breakup():
    """Lo que suelta la O al desintegrarse: más cuanto más deprisa se deshace."""
    rnd = random.Random(23)
    out = []
    for f in range(T_BREAK, T_DUST):
        for _ in range(9):
            a = rnd.uniform(0, 2 * math.pi)
            rr = R_RING + rnd.uniform(-HW_RING, HW_RING)
            out.append(Swirl(_C[0] + rr * math.cos(a), _C[1] + rr * math.sin(a), f, rnd, out=True))
    return out


DUST = breakup()


def pixels_at(f):
    if f < T_SPIN:
        return to_pixels(placed(f))
    px = {p for pts in LETTERS for p in pts if GONE[p] > f}
    if f < T_DUST:
        px.update(dial_pixels(f))
    for s in SWIRLS:
        p = s.at(f)
        if p:
            px.add(p)
    for s in DUST:
        p = s.at(f)
        if p:
            px.add(p)
    return px
