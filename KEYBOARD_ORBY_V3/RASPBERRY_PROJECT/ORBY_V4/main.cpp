#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pinout.h"
#include "hardware_encoder.h" // Nuestra nueva abstracción
#include "hardware_oled.h"
#include "oled_digits.h"

// ==========================================
// CORE 1: MANEJA EXCLUSIVAMENTE LAS PANTALLAS
// ==========================================
void core1_entry() {
    // Inicialización del subsistema OLED (Bit-Banging GPIO)
    HardwareOled oleds;
    oleds.init_all_screens();

    // Dibujar el número de pantalla en cada OLED (1 al 10)
    uint8_t framebuffer[360];
    for (int i = 1; i <= 10; i++) {
        OledDigits::render_number_to_framebuffer(i, framebuffer);
        oleds.paint_screen(i, framebuffer);
    }

    while (true) {
        // Bloquea hasta que el Core 0 mande un trabajo por la FIFO
        uint32_t msg = multicore_fifo_pop_blocking(); 
        
        // Decodificamos el mensaje: 
        // - msg >> 1 = índice de la pantalla (0 a 9)
        // - msg & 1  = estado pulsado (1) o suelto (0)
        uint8_t oled_idx = (msg >> 1);
        bool is_pressed = (msg & 1);
        
        oleds.invert_screen(oled_idx + 1, is_pressed);
    }
}

// ==========================================
// CORE 0: MANEJA ENCODERS, TECLAS Y LIBRERÍA USB (FUTURO)
// ==========================================
int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("=== SISTEMA DUAL-CORE INICIADO ===\n");

    // Despertar al Core 1 y asignarle su tarea
    multicore_launch_core1(core1_entry);

    // Instanciación del hardware. Listo para usar.
    // Encoder 1 en sentido normal (1), Encoder 2 invertido (-1)
    HardwareEncoder rueda_izq(pio0, Pins::ENC1_A, 1);
    HardwareEncoder rueda_der(pio0, Pins::ENC2_A, -1);

    // Inicializar pines de teclas (1 a 10 para las OLEDs)
    const uint8_t key_pins[10] = {
        Pins::KEY_1, Pins::KEY_2, Pins::KEY_3, Pins::KEY_4, 
        Pins::KEY_5, Pins::KEY_6, Pins::KEY_7, Pins::KEY_8, 
        Pins::KEY_9, Pins::KEY_10
    };
    bool last_key_state[10] = {false};

    for (int i = 0; i < 10; i++) {
        gpio_init(key_pins[i]);
        gpio_set_dir(key_pins[i], GPIO_IN);
        gpio_pull_up(key_pins[i]);
    }

    while (true) {
        
        // Solo tenemos que pedirle el delta al objeto
        int delta_izq = rueda_izq.get_delta();
        if (delta_izq != 0) {
            printf("Rueda IZQ: Movimiento %+d | Absoluto: %d\n", delta_izq, rueda_izq.get_absolute());
            // Aquí llamarías a: USB_Send_Volume(delta_izq);
        }

        int delta_der = rueda_der.get_delta();
        if (delta_der != 0) {
            printf("Rueda DER: Movimiento %+d | Absoluto: %d\n", delta_der, rueda_der.get_absolute());
            // Aquí llamarías a: USB_Send_Zoom(delta_der);
        }

        // Leer teclas
        for (int i = 0; i < 10; i++) {
            bool is_pressed = !gpio_get(key_pins[i]); // LOW = Pulsado
            if (is_pressed != last_key_state[i]) {
                last_key_state[i] = is_pressed;
                
                // Embalamos el evento (indice de pantalla + estado)
                uint32_t msg = (i << 1) | (is_pressed ? 1 : 0);
                
                // Lo enviamos a la cola FIFO del core 1 (no bloquea si hay espacio)
                multicore_fifo_push_blocking(msg);
            }
        }

        sleep_ms(1); // Relax para la CPU principal
    }
    
    return 0;
}