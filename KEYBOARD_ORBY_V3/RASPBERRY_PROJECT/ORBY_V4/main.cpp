#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "pinout.h"
#include "hardware_encoder.h"
#include "hardware_oled.h"

// Estado de las 12 teclas (0 = suelta, 1 = pulsada)
static volatile bool g_teclas[12] = {false};
// Números del 1 al 9 en formato 5x8 (columnas de 1 byte)
const uint8_t f_num[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};
void core1_motor_grafico() {
    HardwareOled pantallas(pio1);
    pantallas.init_all_screens();
    
    uint8_t fb[360];

    while (true) {
        for (uint8_t s = 1; s <= 10; s++) {
            // Rellenamos con un patrón de puntos (0x01) para ver el área real
            memset(fb, 0x01, 360); 

            // Dibujamos el número de pantalla
            int centro = (2 * 72) + 33; 
            uint8_t n = s % 10;
            for(int col = 0; col < 5; col++) {
                fb[centro + col] = f_num[n][col];
            }

            // Inversión si se pulsa la tecla
            if (s <= 12 && g_teclas[s-1]) {
                for(int i = 0; i < 360; i++) fb[i] = ~fb[i];
            }

            pantallas.paint_screen(s, fb);
        }
        sleep_ms(20);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("=== MODO TEST: IDENTIFICACION PANTALLAS ===\n");

    // Pines de las teclas (Configuración rápida)
    const uint8_t pins_t[] = {
        Pins::KEY_1, Pins::KEY_2, Pins::KEY_3, Pins::KEY_4,
        Pins::KEY_5, Pins::KEY_6, Pins::KEY_7, Pins::KEY_8,
        Pins::KEY_9, Pins::KEY_10, Pins::KEY_11, Pins::KEY_12
    };

    for (uint8_t p : pins_t) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);
    }

    multicore_launch_core1(core1_motor_grafico);

    while (true) {
        // Polling de teclas
        for (int i = 0; i < 12; i++) {
            g_teclas[i] = !gpio_get(pins_t[i]);
        }
        sleep_ms(5);
    }
}