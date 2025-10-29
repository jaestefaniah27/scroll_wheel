import sys, struct, time
import serial, serial.tools.list_ports

CMD_GET=0x10; CMD_SET=0x11; CMD_SAVE=0x12; CMD_RESET=0x13

def checksum(data: bytes) -> int:
    return sum(data) & 0xFF

def find_arduino_port(prefer_vid=0x2341, prefer_pid=0x8037):
    ports = list(serial.tools.list_ports.comports())
    # 1) busca VID/PID típico Arduino
    for p in ports:
        if p.vid == prefer_vid and p.pid == prefer_pid:
            return p.device
    # 2) busca “Arduino” en descripción
    for p in ports:
        desc = (p.description or "").lower()
        if "arduino" in desc or "leonardo" in desc or "micro" in desc:
            return p.device
    # 3) último recurso: el primero
    return ports[0].device if ports else None

def send_cmd(ser, cmd, payload=b""):
    pkt = b"ORBY" + bytes([cmd, len(payload)]) + payload
    pkt += bytes([checksum(pkt[4:])])
    ser.write(pkt)

    # lee respuesta
    def readn(n, timeout=2.0):
        buf=b""
        t0=time.time()
        while len(buf)<n and (time.time()-t0)<timeout:
            c=ser.read(n-len(buf))
            if c: buf+=c
        return buf

    hdr = readn(6)
    if len(hdr)!=6 or hdr[:4]!=b"ORBY":
        raise RuntimeError("Respuesta inválida (cabecera)")
    rc_cmd, rc_len = hdr[4], hdr[5]
    payload = readn(rc_len)
    chk = readn(1)
    if len(payload)!=rc_len or len(chk)!=1:
        raise RuntimeError("Respuesta incompleta")
    if ((sum(hdr[4:6])+sum(payload)) & 0xFF) != chk[0]:
        raise RuntimeError("Checksum respuesta")
    if rc_cmd == 0xFF:
        err = payload[0] if payload else 0
        raise RuntimeError(f"Error del dispositivo: 0x{err:02X}")
    return rc_cmd & 0x7F, payload

def pretty_cfg(b):
    v, b1, b2, m = b[0], b[1], b[2], b[3]
    s0 = b[4] | (b[5]<<8); s1 = b[6] | (b[7]<<8)
    s2 = b[8] | (b[9]<<8); s3 = b[10] | (b[11]<<8)
    flags = b[12]
    print("== ORBY CONFIG ==")
    print("version     :", v)
    print("btn1_action :", b1)
    print("btn2_action :", b2)
    print("active_mode :", m)
    print("sensitivity :", s0, s1, s2, s3)
    print("flags       : 0x%02X" % flags)

def main():
    port = None
    if len(sys.argv)>=2 and sys.argv[1].lower().startswith("com"):
        port = sys.argv[1]
    if port is None:
        port = find_arduino_port()
    if port is None:
        print("No encuentro el puerto del Arduino.")
        return 2
    ser = serial.Serial(port, 115200, timeout=0.2)
    try:
        if len(sys.argv)==1 or sys.argv[1]=="read":
            _, payload = send_cmd(ser, CMD_GET, b"")
            if len(payload)!=16:
                raise RuntimeError("Longitud inesperada")
            pretty_cfg(payload)
            return 0
        elif sys.argv[1]=="set":
            # ejemplo: set 120 60 30 10  (solo sensibilidades)
            if len(sys.argv)<6:
                print("Uso: set s0 s1 s2 s3")
                return 2
            s = [int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])]
            # lee actual, modifica solo sensibilidad
            _, cur = send_cmd(ser, CMD_GET, b"")
            buf = bytearray(cur)
            buf[4]=s[0]&0xFF; buf[5]=s[0]>>8
            buf[6]=s[1]&0xFF; buf[7]=s[1]>>8
            buf[8]=s[2]&0xFF; buf[9]=s[2]>>8
            buf[10]=s[3]&0xFF; buf[11]=s[3]>>8
            send_cmd(ser, CMD_SET, bytes(buf))
            print("SET OK")
            return 0
        elif sys.argv[1]=="save":
            send_cmd(ser, CMD_SAVE, b"")
            print("SAVE OK (EEPROM)")
            return 0
        elif sys.argv[1]=="reset":
            send_cmd(ser, CMD_RESET, b"")
            print("RESET a fábrica (RAM)")
            return 0
        else:
            print("Comandos: read | set s0 s1 s2 s3 | save | reset | COMx")
            return 2
    finally:
        ser.close()

if __name__ == "__main__":
    import serial.tools.list_ports
    sys.exit(main())
