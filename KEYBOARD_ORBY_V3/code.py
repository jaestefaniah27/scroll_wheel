import time
import board
import digitalio
import rotaryio

# 1. Configuración de TODAS las teclas (GP1 a GP12)
pines_teclas = [
    board.GP1, board.GP2, board.GP3, board.GP4,
    board.GP5, board.GP6, board.GP7, board.GP8,
    board.GP9, board.GP10, board.GP11, board.GP12
]
teclas = []
estado_anterior = [] # Para guardar si la tecla estaba pulsada en el ciclo anterior

for pin in pines_teclas:
    tecla = digitalio.DigitalInOut(pin)
    tecla.direction = digitalio.Direction.INPUT
    tecla.pull = digitalio.Pull.UP # Fundamental: resistencia pull-up interna
    teclas.append(tecla)
    estado_anterior.append(True) # Inician en True (no pulsadas)

# 2. Configuración de Encoders
# Encoder 1
encoder1 = rotaryio.IncrementalEncoder(board.GP26, board.GP27)
btn_e1 = digitalio.DigitalInOut(board.GP28)
btn_e1.direction = digitalio.Direction.INPUT
btn_e1.pull = digitalio.Pull.UP
pos_e1_ant = encoder1.position
btn_e1_ant = True

# Encoder 2
encoder2 = rotaryio.IncrementalEncoder(board.GP23, board.GP24)
btn_e2 = digitalio.DigitalInOut(board.GP25)
btn_e2.direction = digitalio.Direction.INPUT
btn_e2.pull = digitalio.Pull.UP
pos_e2_ant = encoder2.position
btn_e2_ant = True

print("Iniciando test completo: 12 Teclas y 2 Encoders...")

while True:
    # --- TEST DE TECLAS ---
    for i, tecla in enumerate(teclas):
        estado_actual = tecla.value
        
        # Si ahora es False (pulsado) y antes era True (suelto) -> Flanco de bajada
        if estado_actual == False and estado_anterior[i] == True:
            print(f"-> Tecla KEY_{i+1} presionada (Pin GP{i+1})")
            time.sleep(0.02) # Mini-debounce para evitar rebotes mecánicos
            
        estado_anterior[i] = estado_actual

    # --- TEST ENCODER 1 ---
    estado_btn_e1 = btn_e1.value
    if estado_btn_e1 == False and btn_e1_ant == True:
        print("-> Botón SW_E1 presionado")
        time.sleep(0.02)
    btn_e1_ant = estado_btn_e1
        
    pos_e1_act = encoder1.position
    if pos_e1_act != pos_e1_ant:
        print(f"E1 movido | Posición: {pos_e1_act}")
        pos_e1_ant = pos_e1_act

    # --- TEST ENCODER 2 ---
    estado_btn_e2 = btn_e2.value
    if estado_btn_e2 == False and btn_e2_ant == True:
        print("-> Botón SW_E2 presionado")
        time.sleep(0.02)
    btn_e2_ant = estado_btn_e2
        
    pos_e2_act = encoder2.position
    if pos_e2_act != pos_e2_ant:
        print(f"E2 movido | Posición: {pos_e2_act}")
        pos_e2_ant = pos_e2_act

    # Pequeña pausa para no saturar el procesador
    time.sleep(0.005)