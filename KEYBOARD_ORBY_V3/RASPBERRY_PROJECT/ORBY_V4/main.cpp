#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pinout.h"
#include "hardware_encoder.h"
#include "hardware_oled.h"

// Memoria compartida
static volatile bool g_teclas[12] = {false};

const uint8_t f_num[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}
};

// CORE 1: DEDICADO SOLO A REFRESCAR PANTALLAS
void core1_entry() {
    HardwareOled oleds;
    oleds.init_all_screens();
    
    // Pre-generamos los buffers para evitar cálculos en el bucle
    uint8_t buffers[3][360];
    for (int s = 0; s < 3; s++) {
        memset(buffers[s], 0, 360);
        int offset = 144 + 33; 
        for(int i=0; i<5; i++) buffers[s][offset+i] = f_num[s+1][i];
    }

    while (true) {
        for (uint8_t s = 1; s <= 3; s++) {
            // Si la tecla está pulsada, invertimos el buffer al vuelo 
            // solo durante el envío para no perder tiempo
            if (s <= 12 && g_teclas[s-1]) {
                uint8_t temp_inv[360];
                for(int i=0; i<360; i++) temp_inv[i] = ~buffers[s-1][i];
                oleds.paint_screen(s, temp_inv);
            } else {
                oleds.paint_screen(s, buffers[s-1]);
            }
        }
        // El tiempo muerto es ahora prácticamente cero.
    }
}

// CORE 0: INTERRUPCIONES Y INPUTS
int main() {
    stdio_init_all();
    sleep_ms(2000);

    // Lanzamos los gráficos al Core 1
    multicore_launch_core1(core1_entry);

    // Inicializamos encoders en Core 0
    HardwareEncoder enc1(pio0, Pins::ENC1_A, 1);
    HardwareEncoder enc2(pio0, Pins::ENC2_A, -1);

    // Pines de teclas
    const uint8_t t_pins[] = {Pins::KEY_1, Pins::KEY_2, Pins::KEY_3, Pins::KEY_4, 
                              Pins::KEY_5, Pins::KEY_6, Pins::KEY_7, Pins::KEY_8, 
                              Pins::KEY_9, Pins::KEY_10, Pins::KEY_11, Pins::KEY_12};
    for(uint8_t p : t_pins) {
        gpio_init(p); gpio_set_dir(p, GPIO_IN); gpio_pull_up(p);
    }

    while (true) {
        // Leer teclas para que el Core 1 reaccione
        for (int i = 0; i < 12; i++) {
            g_teclas[i] = !gpio_get(t_pins[i]);
        }

        // Leer encoders
        int d1 = enc1.get_delta();
        if(d1 != 0) printf("E1: %+d | Abs: %d\n", d1, enc1.get_absolute());
        
        int d2 = enc2.get_delta();
        if(d2 != 0) printf("E2: %+d | Abs: %d\n", d2, enc2.get_absolute());

        sleep_ms(1);
    }
}