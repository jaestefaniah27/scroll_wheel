"""Prueba a mano del modo juego (GAME:1, firmware con GAMES=1).

    python tools/test/game_spike.py bench     # fps pintados: 10 pantallas, 1 pantalla, y el viejo hexadecimal
    python tools/test/game_spike.py mole      # topos: pulsa la tecla de la pantalla encendida

Hace falta cerrar OrbyGUI antes (Salir en la bandeja): el puerto COM es de uno solo.
Sale del juego con Ctrl+C; si se corta a las bravas, el teclado vuelve solo a los 3 s.
"""
import random
import statistics
import sys
import time

import serial
from serial.tools import list_ports

VID_ORBY = 0xCAFE
FB_BYTES = 360          # 72x40, 5 páginas de 72 columnas
PANTALLAS = 10
EN_VUELO = 3            # fotogramas sin acusar que se permiten a la vez


class Canal:
    """El puerto, con la lectura por líneas y el control de fotogramas en vuelo."""

    def __init__(self):
        for p in list_ports.comports():
            if p.vid == VID_ORBY:
                self.ser = serial.Serial(p.device, 115200, timeout=0, write_timeout=2)
                print(f"Conectado a {p.device}")
                break
        else:
            sys.exit("No hay ningún teclado Orby enchufado (VID CAFE).")
        self.resto = b""
        self.seq = 0
        self.acusado = -1
        self.teclas = []          # (instante, tecla, estado) sin atender

    def lineas(self):
        """Todas las líneas completas que hayan llegado; atiende acuses y teclas."""
        datos = self.ser.read(self.ser.in_waiting or 1)
        if not datos:
            return []
        self.resto += datos
        *completas, self.resto = self.resto.split(b"\n")
        salida = []
        for crudo in completas:
            l = crudo.decode(errors="replace").strip()
            if l.startswith("GAME:SHOWN:"):
                # GAME:SHOWN:<n>[:<µs>]: el número es el tercer campo. Con rsplit se cogía el µs
                # que el firmware añadió después, y cualquier medida salía disparatada.
                self.acusado = max(self.acusado, int(l.split(":")[2]))
            elif l.startswith("KEY_EV:"):
                _, t, e = l.split(":")
                self.teclas.append((time.perf_counter(), int(t), e == "1"))
            salida.append(l)
        return salida

    def esperar(self, prefijo, timeout=2.0):
        fin = time.time() + timeout
        while time.time() < fin:
            for l in self.lineas():
                if l.startswith(prefijo):
                    return l
        return None

    def enviar(self, texto):
        self.ser.write((texto + "\n").encode())

    def frame(self, pantallas):
        """Manda {n: 360 bytes} de las pantallas que cambian (1-10), sin esperar a que se pinte."""
        mascara = sum(1 << (n - 1) for n in pantallas)
        carga = b"".join(pantallas[n] for n in sorted(pantallas))
        # Control de flujo: nunca más de EN_VUELO fotogramas sin acuse.
        while self.seq - self.acusado > EN_VUELO:
            self.lineas()
        self.ser.write(f"GAME_BIN:{mascara}:{self.seq}\n".encode() + carga)
        self.seq += 1
        return self.seq - 1

    def vaciar(self):
        """Espera a que se pinte todo lo enviado."""
        fin = time.time() + 2
        while self.acusado < self.seq - 1 and time.time() < fin:
            self.lineas()


def caja():
    """Un cuadrado 24x24 en el centro: lo que hace de topo."""
    fb = bytearray(FB_BYTES)
    for pagina in range(1, 4):                 # filas 8..31
        for col in range(24, 48):
            fb[pagina * 72 + col] = 0xFF
    return bytes(fb)


def pantalla_de_tecla(n):
    """KEY_EV 1-9 son las pantallas 1-9 y la 11 es la 10; la 10 y la 12 no tienen."""
    if 1 <= n <= 9:
        return n
    return 10 if n == 11 else None


def entrar(c):
    # Orvy deja el espejo encendido en el teclado; se apaga y se vacía lo pendiente.
    c.enviar("MIRROR:0")
    c.esperar("MIRROR:OK:0")
    c.ser.reset_input_buffer()
    c.enviar("GAME:1")
    if not c.esperar("GAME:OK:1"):
        sys.exit("El teclado no ha entrado en modo juego (¿firmware sin GAMES=1 o con un menú abierto?).")
    time.sleep(0.8)   # la animación de entrada: los fotogramas la cortarían


def salir(c):
    c.enviar("GAME:0")
    c.esperar("GAME:OK:0")


def medir(c, nombre, n, construir):
    """n fotogramas generados por construir(i) -> {pantalla: bytes}; da fps pintados."""
    c.vaciar()
    base = c.seq
    t0 = time.perf_counter()
    for i in range(n):
        c.frame(construir(i))
    c.vaciar()
    dt = time.perf_counter() - t0
    pintados = c.acusado - base + 1
    print(f"{nombre:<34} {pintados}/{n} en {dt:.2f} s -> {pintados / dt:6.1f} fps")


def bench(c, n=300):
    llena, vacia = bytes([0xFF] * FB_BYTES), bytes(FB_BYTES)
    todas = range(1, PANTALLAS + 1)
    medir(c, "10 pantallas cambiando (binario)", n,
          lambda i: {p: (llena if i % 2 == 0 else vacia) for p in todas})
    medir(c, "1 pantalla cambiando (binario)", n,
          lambda i: {1 + i % PANTALLAS: (llena if (i // PANTALLAS) % 2 == 0 else vacia)})
    medir(c, "5 pantallas cambiando (binario)", n,
          lambda i: {p: (llena if i % 2 == 0 else vacia) for p in range(1, 6)})


def mole(c, segundos=30):
    topo = random.randint(1, PANTALLAS)
    aciertos = fallos = 0
    reaccion = []
    vacia = bytes(FB_BYTES)
    c.frame({p: (caja() if p == topo else vacia) for p in range(1, PANTALLAS + 1)})
    desde = time.perf_counter()
    fin = time.time() + segundos
    ultimo = time.time()
    while time.time() < fin:
        c.lineas()
        # Mantiene vivo el modo aunque el jugador no toque nada.
        if time.time() - ultimo > 1.0:
            c.enviar("GAME_SHOW")
            ultimo = time.time()
        while c.teclas:
            instante, tecla, pulsada = c.teclas.pop(0)
            p = pantalla_de_tecla(tecla)
            if not pulsada or p is None:
                continue
            if p == topo:
                aciertos += 1
                reaccion.append((instante - desde) * 1000)
                nuevo = random.choice([x for x in range(1, PANTALLAS + 1) if x != topo])
                # Solo cambian dos pantallas: la que se apaga y la que se enciende.
                c.frame({topo: vacia, nuevo: caja()})
                topo = nuevo
                desde = time.perf_counter()
                ultimo = time.time()
            else:
                fallos += 1
    print(f"aciertos {aciertos}, fallos {fallos}", end="")
    if reaccion:
        print(f", reacción mediana {statistics.median(reaccion):.0f} ms")
    else:
        print()


def main():
    modo = sys.argv[1] if len(sys.argv) > 1 else "bench"
    c = Canal()
    entrar(c)
    try:
        {"bench": bench, "mole": mole}[modo](c)
    except KeyboardInterrupt:
        pass
    finally:
        salir(c)


if __name__ == "__main__":
    main()
