import time
import board
import busio

# Configuración del bus I2C (Ajusta los pines SDA y SCL según tu ruteo físico)
i2c = busio.I2C(scl=board.GP15, sda=board.GP14, frequency=400000)
AS5600_ADDR = 0x36

# 1. Validación cruda del bus (Si esto falla, hay un problema físico)
while not i2c.try_lock():
    pass
dispositivos = i2c.scan()
i2c.unlock()

if AS5600_ADDR not in dispositivos:
    print(f"ERROR: AS5600 (0x{AS5600_ADDR:02X}) no detectado en el bus I2C.")
    print("Comprueba soldaduras del U2 o si tienes cruzadas las pistas SDA/SCL.")
    while True:
        time.sleep(1)

print("AS5600 detectado correctamente. Arrancando lecturas...")

def leer_registros(registro, longitud):
    while not i2c.try_lock():
        pass
    # Se escribe el puntero del registro y se lee la respuesta
    i2c.writeto(AS5600_ADDR, bytes([registro]))
    resultado = bytearray(longitud)
    i2c.readfrom_into(AS5600_ADDR, resultado)
    i2c.unlock()
    return resultado

while True:
    # Leer 1 byte del registro de estado
    status = leer_registros(0x0B, 1)[0]
    
    # Máscaras de bits según el datasheet del AS5600
    md = (status & (1 << 5)) != 0  # Magnet Detected
    ml = (status & (1 << 4)) != 0  # Magnet too Weak
    mh = (status & (1 << 3)) != 0  # Magnet too Strong

    if md:
        # Leer los 2 bytes de RAW ANGLE
        raw_bytes = leer_registros(0x0C, 2)
        # Juntar los 12 bits
        angulo_raw = (raw_bytes[0] << 8) | raw_bytes[1]
        
        # Conversión a grados (0-360)
        grados = (angulo_raw / 4096.0) * 360.0
        
        # Diagnóstico del campo magnético (vital para el diseño mecánico de la rueda)
        calidad_campo = "ÓPTIMO"
        if ml: calidad_campo = "MUY DÉBIL (Acerca el imán)"
        if mh: calidad_campo = "MUY FUERTE (Aleja el imán)"
        
        print(f"Imán: {calidad_campo} | Ángulo: {grados:05.1f}º  (Raw: {angulo_raw})")
        
    time.sleep(0.1) # Tasa de refresco de 10Hz para leer la consola sin marearse