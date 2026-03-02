import time
import board
import digitalio

# --- NUEVA CONFIGURACIÓN DE PINES (FÍSICA) ---
shift_data = digitalio.DigitalInOut(board.GP17)  # Antes GP16
shift_clk = digitalio.DigitalInOut(board.GP13)   # Sigue igual
shift_latch = digitalio.DigitalInOut(board.GP16) # Antes GP17

for pin in (shift_data, shift_clk, shift_latch):
    pin.direction = digitalio.Direction.OUTPUT
    pin.value = False

def test_camara_lenta():
    print("\n--- LLENANDO LA MEMORIA OCULTA CON UNOS ---")
    shift_latch.value = False 
    
    # Metemos 16 unos lógicos con tiempos exagerados
    for _ in range(16):
        shift_data.value = True
        time.sleep(0.05)
        shift_clk.value = True   # Flanco que mete el dato al registro
        time.sleep(0.05)
        shift_clk.value = False
        time.sleep(0.05)
        
    print(">>> MARTILLAZO AL LATCH (Pin 12) <<<")
    # Este es el flanco de subida crítico que registra las salidas
    shift_latch.value = True   
    time.sleep(0.5) # Lo mantenemos arriba medio segundo para que lo midas
    shift_latch.value = False
    
    print("Mide las patas de salida de U3 ahora. Tienes 5 segundos.")
    time.sleep(5)

while True:
    test_camara_lenta()