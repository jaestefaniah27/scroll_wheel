#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pinout.h"
#include "hardware_encoder.h"
#include "hardware_oled.h"

// --- TAREA DEL CORE 1 (Motor Gráfico Dedicado) ---
void core1_motor_grafico() {
    // Instanciamos el controlador OLED usando el PIO1 
    // (Dejamos el PIO0 libre entero para los encoders)
    HardwareOled pantallas(pio1);
    
    printf("[CORE 1] Inicializando hardware gráfico...\n");
    pantallas.init_all_screens();

    // Generamos un framebuffer de 360 bytes con ruido
    uint8_t ruido[360];
    for(int i = 0; i < 360; i++) ruido[i] = 0x55; // Patrón a rayas

    // Bucle de renderizado
    while (true) {
        // Pintamos las pantallas 1, 2 y 3
        pantallas.paint_screen(1, ruido);
        pantallas.paint_screen(2, ruido);
        pantallas.paint_screen(3, ruido);
        
        sleep_ms(33); // Capamos a ~30 FPS para no saturar
    }
}

// --- TAREA DEL CORE 0 (Lógica e Inputs) ---
int main() {
    stdio_init_all();
    sleep_ms(2000); 

    // Lanzamos el motor gráfico al Core 1
    multicore_launch_core1(core1_motor_grafico);

    // Instanciamos los encoders en el Core 0
    HardwareEncoder rueda_izq(pio0, Pins::ENC1_A, 1);
    HardwareEncoder rueda_der(pio0, Pins::ENC2_A, -1);

    while (true) {
        int delta_izq = rueda_izq.get_delta();
        if (delta_izq != 0) printf("Rueda IZQ: %+d\n", delta_izq);

        int delta_der = rueda_der.get_delta();
        if (delta_der != 0) printf("Rueda DER: %+d\n", delta_der);

        sleep_ms(1);
    }
    return 0;
}