#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "encoder.pio.h" // El compilador inyectará aquí el C de arriba
#include "pinout.h"

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("=== TEST PIO ENCODERS V2 (ESTABLE) ===\n");

    PIO pio = pio0; 
    pio_clear_instruction_memory(pio); // Limpieza de seguridad
    pio_add_program(pio, &encoder_program);

    uint sm1 = pio_claim_unused_sm(pio, true);
    uint sm2 = pio_claim_unused_sm(pio, true);

    // Inyectamos la lógica de ensamblador en los dos motores
    encoder_program_init(pio, sm1, Pins::ENC1_A); 
    encoder_program_init(pio, sm2, Pins::ENC2_A); 

    int pos_e1 = 0;
    int pos_e2 = 0;
    int clics_e1_ant = 0; // Memoria física del encoder 1
    int clics_e2_ant = 0; // Memoria física del encoder 2
    
    while (true) {
        // 1. La CPU coge los datos en crudo de la memoria FIFO (resolución eléctrica)
        pos_e1 = encoder_get_count(pio, sm1, pos_e1);
        pos_e2 = encoder_get_count(pio, sm2, pos_e2);

        // 2. Dividimos entre 2 para sacar los "clics" reales físicos
        int clics_e1_act = pos_e1 / 2;
        int clics_e2_act = pos_e2 / 2;

        // 3. Calculamos el DELTA seguro contra desbordamientos (Wrap-Around).
        // Al forzar el casting a unsigned int, la resta calcula la distancia circular real
        // en la memoria de 32 bits, evitando cualquier corrupción aunque el número dé la vuelta.
        int delta_e1 = (int)((unsigned int)clics_e1_act - (unsigned int)clics_e1_ant);
        int delta_e2 = (int)((unsigned int)clics_e2_act - (unsigned int)clics_e2_ant);

        // --- LÓGICA DE EVENTOS ENCODER 1 ---
        if (delta_e1 != 0) {
            if (delta_e1 > 0) {
                printf("E1: Horario (+%d pasos)\n", delta_e1);
            } else {
                printf("E1: Antihorario (%d pasos)\n", delta_e1);
            }
            // Actualizamos la memoria interna solo cuando procesamos el evento
            clics_e1_ant = clics_e1_act; 
        }

        // --- LÓGICA DE EVENTOS ENCODER 2 ---
        if (delta_e2 != 0) {
            if (delta_e2 > 0) {
                printf("E2: Horario (+%d pasos)\n", delta_e2);
            } else {
                printf("E2: Antihorario (%d pasos)\n", delta_e2);
            }
            // Actualizamos la memoria interna
            clics_e2_ant = clics_e2_act; 
        }

        sleep_ms(1); // Relax para la CPU
    }
    return 0;
}