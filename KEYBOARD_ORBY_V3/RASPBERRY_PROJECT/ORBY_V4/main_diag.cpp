#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pinout.h"
#include "hardware_oled.h"
#include "oled_digits.h"

// ============================================================
// FIRMWARE DE DIAGNOSTICO VISUAL - ORBY V4
// No necesita monitor serial. Todo se muestra en las pantallas.
// ============================================================

// Funcion auxiliar: seleccionar un bit CS arbitrario (0-15)
void select_cs_bit(int bit) {
    uint16_t mask = 0xFFFF & ~(1 << bit);
    gpio_put(Pins::SHIFT_LATCH, 0);
    for (int j = 15; j >= 0; j--) {
        gpio_put(Pins::SHIFT_DATA, (mask >> j) & 1);
        gpio_put(Pins::SHIFT_CLK, 1);
        __asm volatile ("nop\n");
        gpio_put(Pins::SHIFT_CLK, 0);
    }
    gpio_put(Pins::SHIFT_LATCH, 1);
    sleep_us(10);
}

// Funcion auxiliar: aislar todas (todos los CS a HIGH)
void isolate_all() {
    gpio_put(Pins::SHIFT_LATCH, 0);
    for (int j = 0; j < 16; j++) {
        gpio_put(Pins::SHIFT_DATA, 1);
        gpio_put(Pins::SHIFT_CLK, 1);
        __asm volatile ("nop\n");
        gpio_put(Pins::SHIFT_CLK, 0);
    }
    gpio_put(Pins::SHIFT_LATCH, 1);
}

// Funcion auxiliar: broadcast (todos los CS a LOW)
void select_all() {
    gpio_put(Pins::SHIFT_LATCH, 0);
    for (int j = 0; j < 16; j++) {
        gpio_put(Pins::SHIFT_DATA, 0);
        gpio_put(Pins::SHIFT_CLK, 1);
        __asm volatile ("nop\n");
        gpio_put(Pins::SHIFT_CLK, 0);
    }
    gpio_put(Pins::SHIFT_LATCH, 1);
    sleep_us(10);
}

int main() {
    sleep_ms(1000);

    HardwareOled oleds;
    oleds.init_all_screens();

    uint8_t framebuffer[360];

    // ===========================================================
    // FASE 1: Pintar numeros en todas las pantallas (verificacion)
    // ===========================================================
    for (int i = 1; i <= 10; i++) {
        OledDigits::render_number_to_framebuffer(i, framebuffer);
        oleds.paint_screen(i, framebuffer);
    }
    sleep_ms(3000);

    // ===========================================================
    // FASE 2: BRUTE FORCE VISUAL
    //
    // Para cada bit (0-15):
    //   1. Muestra el numero del bit en la PANTALLA 1
    //   2. Enciende todos los pixeles de ese bit (0xA5)
    //   3. Espera 3 segundos
    //   4. Apaga y pasa al siguiente
    //
    // MIRA LA PANTALLA 1 para saber que bit se esta probando.
    // MIRA LA PANTALLA 9 para ver cuando se enciende.
    // El numero que veas en la pantalla 1 cuando la 9 brille
    // es el CS_BIT correcto para la pantalla 9.
    // ===========================================================

    while (true) {
        for (int bit = 0; bit < 16; bit++) {
            // Paso 1: Mostrar el numero del bit actual en pantalla 1
            OledDigits::render_number_to_framebuffer(bit, framebuffer);
            oleds.paint_screen(1, framebuffer);
            sleep_ms(200);

            // Paso 2: Seleccionar el bit bajo prueba y encender toda la pantalla
            select_cs_bit(bit);
            gpio_put(Pins::OLED_DC, 0);
            uint8_t cmd_on = 0xA5;
            spi_write_blocking(spi0, &cmd_on, 1);

            // Paso 3: Esperar 3 segundos para que el usuario mire
            sleep_ms(3000);

            // Paso 4: Apagar la pantalla bajo prueba (volver a mostrar RAM)
            uint8_t cmd_normal = 0xA4;
            spi_write_blocking(spi0, &cmd_normal, 1);

            // Aislar todo
            isolate_all();
            sleep_ms(500);
        }

        // Pausa antes de repetir el ciclo
        // Mostrar "99" en pantalla 1 como indicador de fin de ciclo
        OledDigits::render_number_to_framebuffer(99, framebuffer);
        oleds.paint_screen(1, framebuffer);
        sleep_ms(5000);
    }

    return 0;
}
