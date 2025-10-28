import sys
import struct
import hid

# ==== Ajusta estos si los sabes; si no, deja None y el script listará y te pedirá elegir ====
VID = None  # p.ej. 0x2341
PID = None  # p.ej. 0x8036

RID_CFG = 0x05
RID_CMD = 0x06

# Estructura C (32 bytes) que definimos:
# typedef struct {
#   u8 version; u8 active_mode; u16 sens_global_cpi;
#   u16 sens_mode[4]; u8 btn_map[8]; u8 flags; u8 pad[...]
# } sw_feature_config_t;
#
# Empaquetamos/desempaquetamos sólo el "payload" de 32 bytes (sin el Report ID).
# Little-endian:
FMT_CFG = "<BBH4H8B B"   # = 1+1 +2 + (4*2) + 8 + 1 = 21 bytes útiles
CFG_PAYLOAD_LEN = 32     # TAMAÑO EXACTO del bloque de config (sin Report ID)
PAD_LEN = CFG_PAYLOAD_LEN - struct.calcsize(FMT_CFG)  # debería dar 11

def decode_cfg(payload: bytes):
    base = struct.unpack(FMT_CFG, payload[:struct.calcsize(FMT_CFG)])
    version, active_mode, sens_global = base[0], base[1], base[2]
    sens_mode = list(base[3:7])          # 4 valores
    btn_map   = list(base[7:15])         # 8 valores
    flags     = base[15]
    return {
        "version": version,
        "active_mode": active_mode,
        "sens_global_cpi": sens_global,
        "sens_mode": sens_mode,
        "btn_map": btn_map,
        "flags": flags,
    }

def encode_cfg(cfg: dict) -> bytes:
    # Normaliza tamaños
    sens_mode = (cfg.get("sens_mode") or [0,0,0,0])[:4]
    sens_mode += [0]*(4-len(sens_mode))
    btn_map   = (cfg.get("btn_map") or list(range(8)))[:8]
    btn_map  += [0]*(8-len(btn_map))
    tup = (
        int(cfg.get("version", 1)) & 0xFF,
        int(cfg.get("active_mode", 0)) & 0xFF,
        int(cfg.get("sens_global_cpi", 800)) & 0xFFFF,
        int(sens_mode[0]) & 0xFFFF,
        int(sens_mode[1]) & 0xFFFF,
        int(sens_mode[2]) & 0xFFFF,
        int(sens_mode[3]) & 0xFFFF,
        int(btn_map[0]) & 0xFF, int(btn_map[1]) & 0xFF,
        int(btn_map[2]) & 0xFF, int(btn_map[3]) & 0xFF,
        int(btn_map[4]) & 0xFF, int(btn_map[5]) & 0xFF,
        int(btn_map[6]) & 0xFF, int(btn_map[7]) & 0xFF,
        int(cfg.get("flags", 0)) & 0xFF,
    )
    packed = struct.pack(FMT_CFG, *tup)
    if len(packed) > CFG_PAYLOAD_LEN:
        raise ValueError("Config supera los 32 bytes.")
    return packed + bytes(PAD_LEN)

def pick_device():
    devs = list(hid.enumerate())
    if VID is not None and PID is not None:
        for d in devs:
            if d["vendor_id"] == VID and d["product_id"] == PID:
                return d
        print("No se encontró el dispositivo con VID/PID especificados.")
    # Lista y elige por índice
    print("Dispositivos HID encontrados:")
    for i, d in enumerate(devs):
        print(f"[{i}] VID=0x{d['vendor_id']:04X} PID=0x{d['product_id']:04X} "
              f"prod={d.get('product_string')} mfg={d.get('manufacturer_string')}")
    idx = int(input("Elige índice del dispositivo (interfaz Feature de tu placa): ").strip())
    return devs[idx]

def open_device(d):
    h = hid.device()
    h.open_path(d["path"])
    # (Opcional) h.set_nonblocking(True)
    return h

def get_feature_cfg(h):
    # En hidapi, GET_FEATURE incluye el Report ID y el tamaño total a leer
    # Tamaño total = 1 (RID) + 32 (payload)
    buf = h.get_feature_report(RID_CFG, 1 + CFG_PAYLOAD_LEN)
    if not buf or buf[0] != RID_CFG:
        raise RuntimeError("GET_FEATURE(RID=0x05) inválido.")
    payload = bytes(buf[1:1+CFG_PAYLOAD_LEN])
    return decode_cfg(payload)

def set_feature_cfg(h, cfg_dict):
    payload = encode_cfg(cfg_dict)
    out = bytes([RID_CFG]) + payload
    # En hidapi, send_feature_report requiere que el primer byte sea el Report ID
    n = h.send_feature_report(out)
    if n < 0:
        raise RuntimeError("SET_FEATURE(RID=0x05) falló.")
    return n

def send_cmd(h, op, arg0=0, arg1=0):
    # Comando: 4 bytes de payload (op, arg0, arg1, (padding opcional si lo definiste))
    # Nosotros definimos struct con report_id + 3 bytes útiles (op,arg0,arg1).
    # Para simplicidad, enviamos 1 (RID) + 3 (payload) == 4.
    out = bytes([RID_CMD, op & 0xFF, arg0 & 0xFF, arg1 & 0xFF])
    n = h.send_feature_report(out)
    if n < 0:
        raise RuntimeError("SET_FEATURE(RID=0x06) falló.")
    return n

def main():
    d = pick_device()
    print(f"Usando VID=0x{d['vendor_id']:04X} PID=0x{d['product_id']:04X} path={d['path']}")
    h = open_device(d)
    try:
        # 1) Lee configuración actual
        cfg = get_feature_cfg(h)
        print("CFG actual:", cfg)

        # 2) Cambia algo (ej: +100 CPI global)
        new_cfg = dict(cfg)
        new_cfg["sens_global_cpi"] = max(1, min(4000, cfg["sens_global_cpi"] + 100))
        new_cfg["active_mode"] = 1 if cfg["active_mode"] != 1 else 0

        set_feature_cfg(h, new_cfg)
        cfg2 = get_feature_cfg(h)
        print("CFG tras SET_FEATURE:", cfg2)

        # 3) Guarda en EEPROM (op=0)
        send_cmd(h, op=0)
        print("SAVE enviado (op=0). Desconecta y reconecta para verificar persistencia.")

        # 4) (Opcional) Reset a defaults (op=1)
        # send_cmd(h, op=1)
        # print("RESET_DEFAULTS enviado (op=1).")

    finally:
        h.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
