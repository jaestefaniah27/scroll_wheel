# test_hid_config_win.py (Windows-only, pywinusb)
import struct
import pywinusb.hid as hid


RID_CFG = 0x05
RID_CMD = 0x06
CFG_PAYLOAD_LEN = 32
FMT_CFG = "<BBH4H8B B"  # 21 bytes útiles; el resto padding
PAD_LEN = CFG_PAYLOAD_LEN - struct.calcsize(FMT_CFG)

def encode_cfg(cfg: dict) -> bytes:
    sens_mode = (cfg.get("sens_mode") or [0,0,0,0])[:4] + [0]*max(0,4-len(cfg.get("sens_mode") or []))
    btn_map   = (cfg.get("btn_map")   or list(range(8)))[:8] + [0]*max(0,8-len(cfg.get("btn_map") or []))
    tup = (
        int(cfg.get("version", 1)) & 0xFF,
        int(cfg.get("active_mode", 0)) & 0xFF,
        int(cfg.get("sens_global_cpi", 800)) & 0xFFFF,
        int(sens_mode[0]) & 0xFFFF, int(sens_mode[1]) & 0xFFFF,
        int(sens_mode[2]) & 0xFFFF, int(sens_mode[3]) & 0xFFFF,
        int(btn_map[0]) & 0xFF, int(btn_map[1]) & 0xFF, int(btn_map[2]) & 0xFF, int(btn_map[3]) & 0xFF,
        int(btn_map[4]) & 0xFF, int(btn_map[5]) & 0xFF, 
        int(btn_map[6]) & 0xFF, int(btn_map[7]) & 0xFF,
        int(cfg.get("flags", 0)) & 0xFF,
    )
    packed = struct.pack(FMT_CFG, *tup)
    return packed + bytes(PAD_LEN)

def decode_cfg(payload: bytes):
    base = struct.unpack(FMT_CFG, payload[:struct.calcsize(FMT_CFG)])
    version, active_mode, sens_global = base[0], base[1], base[2]
    sens_mode = list(base[3:7])
    btn_map   = list(base[7:15])
    flags     = base[15]
    return {
        "version": version,
        "active_mode": active_mode,
        "sens_global_cpi": sens_global,
        "sens_mode": sens_mode,
        "btn_map": btn_map,
        "flags": flags,
    }

def pick_device():
    devs = hid.HidDeviceFilter().get_devices()
    print("Dispositivos HID encontrados:")
    for i, d in enumerate(devs):
        try:
            d.open()
            prod = d.product_name
            mfg  = d.vendor_name
            inter = d.device_path.split("&MI_")[-1][:2] if "&MI_" in d.device_path else "--"
            d.close()
        except:
            prod, mfg, inter = "", "", "--"
        print(f"[{i}] VID=0x{d.vendor_id:04X} PID=0x{d.product_id:04X} MI={inter}  prod='{prod}' mfg='{mfg}'")
    idx = int(input("Elige índice del dispositivo (interfaz Feature de tu placa): ").strip())
    return devs[idx]

def find_feature_reports(dev):
    # Abre y busca los Feature reports
    dev.open()
    feats = dev.find_feature_reports()
    # Mapea por Report ID
    rep_by_id = {r.report_id: r for r in feats}
    return rep_by_id

def get_cfg(rep_cfg):
    # Para leer: .get() devuelve lista de enteros (incluye report_id en [0])
    data = rep_cfg.get()
    if not data or data[0] != RID_CFG or len(data) < 1 + CFG_PAYLOAD_LEN:
        raise RuntimeError(f"GET_FEATURE inválido: len={len(data) if data else 0}, head={data[0] if data else None}")
    payload = bytes(bytearray(data[1:1+CFG_PAYLOAD_LEN]))
    return decode_cfg(payload)

def set_cfg(rep_cfg, cfg_dict):
    # Para escribir: set_raw_data(lista) y luego send()
    payload = encode_cfg(cfg_dict)
    # El buffer debe medir report_length; y empezar por report_id
    buf = [0] * rep_cfg.report_length
    buf[0] = RID_CFG
    # Copia payload
    for i, b in enumerate(payload, start=1):
        if i >= len(buf): break
        buf[i] = b
    rep_cfg.set_raw_data(buf)
    rep_cfg.send()

def send_cmd(rep_cmd, op, arg0=0, arg1=0):
    buf = [0]*rep_cmd.report_length
    buf[0] = RID_CMD
    buf[1] = op & 0xFF
    buf[2] = arg0 & 0xFF
    buf[3] = arg1 & 0xFF
    rep_cmd.set_raw_data(buf)
    rep_cmd.send()

def main():
    dev = pick_device()
    rep = find_feature_reports(dev)
    if RID_CFG not in rep:
        print("No veo el Feature report RID 0x05 en esta interfaz. ¿Escogiste la MI_xx correcta?")
        dev.close(); return
    rep_cfg = rep[RID_CFG]
    rep_cmd = rep.get(RID_CMD)

    # 1) Leer
    cfg = get_cfg(rep_cfg)
    print("CFG actual:", cfg)

    # 2) Cambiar y escribir
    new_cfg = dict(cfg)
    new_cfg["sens_global_cpi"] = min(4000, cfg["sens_global_cpi"] + 100)
    new_cfg["active_mode"] = 1 if cfg["active_mode"] != 1 else 0
    set_cfg(rep_cfg, new_cfg)

    # 3) Leer de nuevo
    cfg2 = get_cfg(rep_cfg)
    print("CFG tras SET_FEATURE:", cfg2)

    # 4) Guardar en EEPROM
    if rep_cmd:
        send_cmd(rep_cmd, op=0)
        print("SAVE enviado (op=0). Desconecta y reconecta para verificar persistencia.")
    else:
        print("Aviso: no encontré RID_CMD (0x06), pero la lectura/escritura del RID_CFG funciona.")

    dev.close()

if __name__ == "__main__":
    main()
