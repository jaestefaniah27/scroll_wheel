import time
import board
import busio
import digitalio
import adafruit_ssd1306

# --- CONFIGURACIÓN DEL SHIFT REGISTER ---
shift_data = digitalio.DigitalInOut(board.GP17)
shift_clk = digitalio.DigitalInOut(board.GP13)
shift_latch = digitalio.DigitalInOut(board.GP16)

for pin in (shift_data, shift_clk, shift_latch):
    pin.direction = digitalio.Direction.OUTPUT
    pin.value = False

# El diccionario maestro extraído de tu esquemático
MAPA_CS = {
    1: 5,   # CS1 -> U3 Q5
    2: 6,   # CS2 -> U3 Q6
    3: 7,   # CS3 -> U3 Q7
    4: 2,   # CS4 -> U3 Q2
    5: 3,   # CS5 -> U3 Q3
    6: 4,   # CS6 -> U3 Q4
    7: 0,   # CS7 -> U3 Q0
    8: 1,   # CS8 -> U3 Q1
    9: 14,  # CS9 -> U4 Q6
    10: 13  # CS10 -> U4 Q5
}

def aplicar_cs(pantalla_activa):
    """
    pantalla_activa: número de OLED (1 al 10) o None para aislar todas.
    """
    bit_objetivo = MAPA_CS.get(pantalla_activa, -1)
    
    shift_latch.value = False
    
    # Empujamos 16 bits (del 15 al 0). 
    # El primero en salir (15) acaba en la cola (U4 Q7).
    for i in range(15, -1, -1):
        if i == bit_objetivo:
            shift_data.value = False  # LOW = Selecciona la pantalla
        else:
            shift_data.value = True   # HIGH = Aísla la pantalla
            
        shift_clk.value = True
        shift_clk.value = False
        
    shift_latch.value = True

# --- BROADCAST DE INICIALIZACIÓN ---
print("Despertando todas las pantallas (CS a GND)...")
shift_latch.value = False
for _ in range(16):
    shift_data.value = False
    shift_clk.value = True
    shift_clk.value = False
shift_latch.value = True

spi = busio.SPI(clock=board.GP18, MOSI=board.GP19)
dc = digitalio.DigitalInOut(board.GP20)
rst = digitalio.DigitalInOut(board.GP21)
cs_dummy = digitalio.DigitalInOut(board.GP22) 

WIDTH = 72
HEIGHT = 40

# Inicialización simultánea
display = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, dc, rst, cs_dummy)

print("Aislando pantallas del bus SPI...")
aplicar_cs(None)
time.sleep(0.5)

# --- TEST DE MULTIPLEXADO SEGURO ---
# Como solo tienes 3 conectadas, probamos esas
pantallas_conectadas = [1, 2, 3]

while True:
    for num_oled in pantallas_conectadas:
        print(f"Dibujando en OLED {num_oled}...")
        
        # 1. Bajamos el CS específico (el resto se quedan en HIGH)
        aplicar_cs(num_oled)
        
        # 2. Renderizamos en memoria
        display.fill(0)
        display.rect(0, 0, WIDTH, HEIGHT, 1)
        display.text(f"PANTALLA {num_oled}", 5, 16, 1)
        
        # 3. Disparamos la imagen
        display.show()
        
        # 4. Levantamos el CS inmediatamente
        aplicar_cs(None)
        
        time.sleep(1)