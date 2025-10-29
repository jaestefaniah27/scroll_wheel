#!/usr/bin/env python3
# orby_hid_test.py
# Prueba de interfaz HID vendor (Usage Page 0xFF00) para ORBY.
# Requiere: pip install hidapi

import argparse, sys
import hid
from dataclasses import dataclass

REPORT_ID_CONFIG = 0x10
REPORT_ID_SAVE   = 0x11
REPORT_ID_RESET  = 0x12

# 16 bytes de payload (sin contar el Report ID):
# [0]=version, [1]=btn1, [2]=btn2, [3]=active_mode,
# [4..5]=sens0, [6..7]=sens1, [8..9]=sens2, [10..11]=sens3, [12]=flags, [13..15]=reserved
PAYLOAD_LEN = 16

@dataclass
class OrbyConfig:
    version: int = 1
    btn1: int = 0
    btn2: int = 1
    active_mode: int = 0
    sens: tuple = (120, 60, 30, 10)
    flags: int = 0
    reserved: bytes = b"\x00\x00\x00"  # 3 bytes

    @classmethod
    def from_bytes(cls, b: bytes):
        if len(b) != PAYLOAD_LEN:
            raise ValueError(f"Tamaño inesperado: {len(b)} (esperado {PAYLOAD_LEN})")
        version = b[0]
        btn1 = b[1]
        btn2 = b[2]
        active_mode = b[3]
        s0 = b[4] | (b[5] << 8)
        s1 = b[6] | (b[7] << 8)
        s2 = b[8] | (b[9] << 8)
        s3 = b[10] | (b[11] << 8)
        flags = b[12]
        reserved = bytes(b[13:16])
        return cls(version, btn1, btn2, active_mode, (s0, s1, s2, s3), flags, reserved)

    def to_bytes(self) -> bytes:
        out = bytearray(PAYLOAD_LEN)
        out[0] = self.version
        out[1] = self.btn1
        out[2] = self.btn2
        out[3] = self.active_mode
        out[4]  = self.sens[0] & 0xFF
        out[5]  = (self.sens[0] >> 8) & 0xFF
        out[6]  = self.sens[1] & 0xFF
        out[7]  = (self.sens[1] >> 8) & 0xFF
        out[8]  = self.sens[2] & 0xFF
        out[9]  = (self.sens[2] >> 8) & 0xFF
        out[10] = self.sens[3] & 0xFF
        out[11] = (self.sens[3] >> 8) & 0xFF
        out[12] = self.flags
        out[13:16] = self.reserved
        return bytes(out)

def find_orby(vendor_id=None, product_id=None):
    """Devuelve el primer dispositivo HID con Usage Page 0xFF00 (o el VID/PID si se dan)."""
    devs = hid.enumerate()
    candidates = []
    for d in devs:
        # En Windows, hid.enumerate expone usage_page
        if vendor_id and product_id:
            if d["vendor_id"] == vendor_id and d["product_id"] == product_id:
                candidates.append(d)
        else:
            # Filtra por Usage Page vendor 0xFF00 (puede que en Linux/Mac no aparezca)
            if d.get("usage_page") == 0xFF00:
                candidates.append(d)
    if not candidates:
        # Como fallback, lista todo para ayudar a localizar VID/PID
        print("No se encontró interfaz 0xFF00. Dispositivos detectados:")
        for d in devs:
            up = d.get("usage_page")
            print(f"- VID:PID {d['vendor_id']:04X}:{d['product_id']:04X} "
                  f"iface={d.get('interface_number')} usage_page={up} path={d.get('path')}")
        return None
    # Si hay varias, coge la primera
    return candidates[0]

def open_hid(dinfo):
    h = hid.device()
    # En Windows basta con abrir por path
    if dinfo.get("path"):
        h.open_path(dinfo["path"])
    else:
        h.open(dinfo["vendor_id"], dinfo["product_id"])
    h.set_nonblocking(0)
    return h

def read_config(h) -> OrbyConfig:
    # get_feature_report(report_id, length) incluye report_id al frente
    data = h.get_feature_report(REPORT_ID_CONFIG, PAYLOAD_LEN + 1)
    # data[0] = report_id, data[1:]=payload
    if not data or data[0] != REPORT_ID_CONFIG or len(data) < PAYLOAD_LEN + 1:
        raise RuntimeError("Lectura de CONFIG fallida o tamaño inesperado.")
    return OrbyConfig.from_bytes(bytes(data[1:1+PAYLOAD_LEN]))

def write_config(h, cfg: OrbyConfig):
    buf = bytearray(1 + PAYLOAD_LEN)
    buf[0] = REPORT_ID_CONFIG
    buf[1:] = cfg.to_bytes()
    # send_feature_report(buffer) debe incluir el Report ID
    wrote = h.send_feature_report(bytes(buf))
    if wrote != len(buf):
        raise RuntimeError(f"SET CONFIG escribió {wrote} bytes (esperado {len(buf)})")

def send_simple(h, report_id: int, dummy: int = 1):
    buf = bytes([report_id, dummy & 0xFF])
    wrote = h.send_feature_report(buf)
    if wrote != len(buf):
        raise RuntimeError(f"SET (0x{report_id:02X}) escribió {wrote} bytes (esperado {len(buf)})")

def pretty(cfg: OrbyConfig):
    print("== ORBY CONFIG ==")
    print(f"version      : {cfg.version}")
    print(f"btn1_action  : {cfg.btn1}")
    print(f"btn2_action  : {cfg.btn2}")
    print(f"active_mode  : {cfg.active_mode}")
    print(f"sensitivity  : {cfg.sens[0]}, {cfg.sens[1]}, {cfg.sens[2]}, {cfg.sens[3]}")
    print(f"flags        : 0x{cfg.flags:02X}")
    print(f"reserved     : {cfg.reserved.hex()}")

def main():
    ap = argparse.ArgumentParser(description="Tester de HID Feature Reports de ORBY")
    ap.add_argument("--vid", type=lambda x: int(x,16), help="VID en hex (opcional)")
    ap.add_argument("--pid", type=lambda x: int(x,16), help="PID en hex (opcional)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="Listar dispositivos HID y resaltar Usage Page 0xFF00")
    sub.add_parser("read", help="Leer configuración (GET_FEATURE 0x10)")

    sp_set = sub.add_parser("set", help="Establecer configuración (SET_FEATURE 0x10)")
    sp_set.add_argument("--btn1", type=int)
    sp_set.add_argument("--btn2", type=int)
    sp_set.add_argument("--mode", type=int, help="active_mode 0..3")
    sp_set.add_argument("--sens", type=int, nargs=4, metavar=("S0","S1","S2","S3"),
                        help="sensibilidades por modo (uint16)")
    sp_set.add_argument("--flags", type=lambda x: int(x,0), help="bitfield (p. ej. 0x01)")

    sub.add_parser("save", help="Guardar en EEPROM (SET_FEATURE 0x11)")
    sub.add_parser("reset", help="Reset de fábrica (SET_FEATURE 0x12)")

    args = ap.parse_args()

    if args.cmd == "list":
        for d in hid.enumerate():
            up = d.get("usage_page")
            mark = " <== 0xFF00" if up == 0xFF00 else ""
            print(f"VID:PID {d['vendor_id']:04X}:{d['product_id']:04X} "
                  f"iface={d.get('interface_number')} usage_page={up} path={d.get('path')}{mark}")
        return 0

    dinfo = find_orby(args.vid, args.pid)
    if not dinfo:
        print("\nNo se encontró el interfaz 0xFF00 automáticamente. "
              "Prueba 'list' para localizar VID/PID y vuelve a ejecutar con --vid --pid.")
        return 1

    with open_hid(dinfo) as h:
        if args.cmd == "read":
            cfg = read_config(h)
            pretty(cfg)
            return 0

        if args.cmd == "set":
            cfg = read_config(h)  # parte de lo actual y solo modifica lo que pases
            if args.btn1 is not None: cfg.btn1 = args.btn1
            if args.btn2 is not None: cfg.btn2 = args.btn2
            if args.mode is not None:
                if not (0 <= args.mode <= 3): raise SystemExit("active_mode debe ser 0..3")
                cfg.active_mode = args.mode
            if args.sens is not None:
                sens = tuple(int(max(0, min(65535, s))) for s in args.sens)
                if len(sens) != 4: raise SystemExit("--sens necesita 4 valores")
                cfg.sens = sens
            if args.flags is not None:
                cfg.flags = args.flags & 0xFF

            write_config(h, cfg)
            print("SET OK. Nueva configuración en RAM:")
            pretty(cfg)
            return 0

        if args.cmd == "save":
            send_simple(h, REPORT_ID_SAVE, 1)
            print("SAVE enviado (EEPROM).")
            return 0

        if args.cmd == "reset":
            send_simple(h, REPORT_ID_RESET, 1)
            print("RESET enviado (valores de fábrica en RAM).")
            return 0

    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("Error:", e)
        raise SystemExit(2)
