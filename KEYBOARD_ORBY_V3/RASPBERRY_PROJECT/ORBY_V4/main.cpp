#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "bsp/board.h"
#include "tusb.h"
#include "class/hid/hid_device.h"
#include "class/cdc/cdc_device.h"
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
// CALLBACKS CDC (SERIAL)
// ==========================================
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    char buf[64];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
    
    if (count > 0) {
        // Hacemos eco de vuelta al host para comprobar que la comunicación es bidireccional
        tud_cdc_n_write(itf, "RECIBIDO: ", 10);
        tud_cdc_n_write(itf, buf, count);
        tud_cdc_n_write_flush(itf);

        // Si recibimos "TEST", hacemos parpadear la primera pantalla
        if (count >= 4 && strncmp(buf, "TEST", 4) == 0) {
            uint32_t msg = (0 << 1) | 1; // Invertir pantalla 1 (Indice 0)
            multicore_fifo_push_blocking(msg);
        }
    }
}

// ==========================================
// CORE 0: MANEJA ENCODERS, TECLAS Y LIBRERÍA USB 
// ==========================================
int main() {
    board_init();
    tusb_init();

    // Reemplazamos stdio_init_all() ya que TinyUSB y CDC gestionarán el puerto serial si queremos
    // stdio_init_all();
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

    int pending_vol_clicks = 0;
    bool vol_release_pending = false;

    while (true) {
        tud_task(); // Manejo en tiempo real del stack USB (TinyUSB)
        
        // --- 1. ENCODER IZQUIERDO (VOLUMEN) ---
        pending_vol_clicks += rueda_izq.get_delta();
        
        if (tud_hid_ready()) {
            if (vol_release_pending) {
                // El reporte anterior fue un 'Press', ahora mandamos el 'Release'
                uint16_t volume_code = 0;
                tud_hid_report(3, &volume_code, 2);
                vol_release_pending = false;
            } else if (pending_vol_clicks != 0) {
                // Hay clics pendientes y el endpoint está libre
                uint16_t volume_code = (pending_vol_clicks > 0) ? HID_USAGE_CONSUMER_VOLUME_INCREMENT : HID_USAGE_CONSUMER_VOLUME_DECREMENT;
                tud_hid_report(3, &volume_code, 2); 
                vol_release_pending = true;
                
                // Descontamos el clic procesado
                if (pending_vol_clicks > 0) pending_vol_clicks--;
                else pending_vol_clicks++;
            }
        }

        // --- 2. ENCODER DERECHO (P. EJ. ZOOM, scroll) ---
        int delta_der = rueda_der.get_delta();
        if (delta_der != 0 && tud_hid_ready()) {
            // Placeholder: de momento solo imprimimos, más adelante podríamos mapearlo a scroll de ratón
            // printf("Rueda DER: Movimiento %+d | Absoluto: %d\n", delta_der, rueda_der.get_absolute());
        }

        // --- 3. TECLAS A MACROS ---
        for (int i = 0; i < 10; i++) {
            bool is_pressed = !gpio_get(key_pins[i]); // LOW = Pulsado
            if (is_pressed != last_key_state[i]) {
                
                // Esperamos cortésmente si el canal HID está ocupado mandando otro reporte consecutivo
                while (!tud_hid_ready()) {
                    tud_task(); 
                    sleep_us(100);
                }

                last_key_state[i] = is_pressed;
                
                // Embalamos el evento para cambiar el color de la OLED (Core 1)
                uint32_t msg = (i << 1) | (is_pressed ? 1 : 0);
                multicore_fifo_push_blocking(msg);

                // Lógica de Atajos (Core 0 -> USB)
                if (is_pressed) {
                    // Ejemplo: Si es la primera tecla (i == 0), enviamos L-CTRL + C
                    if (i == 0) {
                        uint8_t keycode[6] = {HID_KEY_C, 0, 0, 0, 0, 0};
                        // Report ID 1 corresponde al teclado. Enviamos Keyboard Modifier (L-CTRL) + Letra C
                        tud_hid_keyboard_report(1, KEYBOARD_MODIFIER_LEFTCTRL, keycode);
                    }
                } else {
                    // Cuando soltamos cualquier tecla, liberamos el teclado (mandamos todo a 0)
                    tud_hid_keyboard_report(1, 0, NULL);
                }
            }
        }

        sleep_ms(1); // Relax para la CPU principal
    }
    
    return 0;
}