#pragma once
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pinout.h"

class HardwareOled {
private:
    // Secuencia de 11 bytes: El "Mínimo Producto Viable" para encender un SSD1306
    const uint8_t init_seq[11] = {
        0xAE,               // Display OFF
        0xA8, 0x27,         // Multiplex Ratio (40px)
        0x8D, 0x14,         // Charge Pump ENABLE (Sin esto no hay luz)
        0x20, 0x00,         // Modo Horizontal
        0x81, 0xFF,         // Contraste al máximo
        0xA6,               // Normal Display
        0xAF                // Display ON
    };

    void apply_cs(uint16_t mask) {
        // Bit-banging manual exacto al que te funcionó
        gpio_put(Pins::SHIFT_LATCH, 0); // Latch Clock (Pin 12)
        for (int i = 15; i >= 0; i--) {
            gpio_put(Pins::SHIFT_CLK, 0); // Shift Clock (Pin 11)
            gpio_put(Pins::SHIFT_DATA, (mask >> i) & 1); // Data (Pin 14)
            sleep_us(1); // Tiempo de guarda para t_su (Setup Time)
            gpio_put(Pins::SHIFT_CLK, 1);
            sleep_us(1);
        }
        gpio_put(Pins::SHIFT_LATCH, 1);
        sleep_us(1); // Tiempo de guarda para t_w (Pulse Width)
    }

public:
    HardwareOled() {
        gpio_init(Pins::SHIFT_DATA);  gpio_set_dir(Pins::SHIFT_DATA, GPIO_OUT);
        gpio_init(Pins::SHIFT_CLK);   gpio_set_dir(Pins::SHIFT_CLK, GPIO_OUT);
        gpio_init(Pins::SHIFT_LATCH); gpio_set_dir(Pins::SHIFT_LATCH, GPIO_OUT);
        gpio_init(Pins::OLED_DC);     gpio_set_dir(Pins::OLED_DC, GPIO_OUT);
        gpio_init(Pins::OLED_RST);    gpio_set_dir(Pins::OLED_RST, GPIO_OUT);

        // SPI a 4MHz (Muy lento pero indestructible para debug)
        spi_init(spi0, 4000 * 1000); 
        gpio_set_function(Pins::SPI_SCK, GPIO_FUNC_SPI);
        gpio_set_function(Pins::SPI_MOSI, GPIO_FUNC_SPI);
    }

    void init_all_screens() {
        // Reset físico agresivo
        gpio_put(Pins::OLED_RST, 0); sleep_ms(100); 
        gpio_put(Pins::OLED_RST, 1); sleep_ms(100);
        
        apply_cs(0x0000); // Seleccionar todas
        gpio_put(Pins::OLED_DC, 0);
        spi_write_blocking(spi0, init_seq, 11);
        apply_cs(0xFFFF); 
    }

    void paint_screen(uint8_t screen_num, const uint8_t* fb) {
        // 1. Calculamos la máscara para esta pantalla
        uint16_t mask = 0xFFFF;
        mask &= ~(1 << OledMap::CS_BITS[screen_num - 1]);
        
        // 2. Seleccionamos la pantalla (Bajamos su CS)
        apply_cs(mask);

        // 3. Comandos de posicionamiento (Viewport 72x40)
        uint8_t setup[] = {0x21, 28, 28+71, 0x22, 0, 4};
        gpio_put(Pins::OLED_DC, 0);
        spi_write_blocking(spi0, setup, 6);

        // 4. Datos del Framebuffer
        gpio_put(Pins::OLED_DC, 1);
        spi_write_blocking(spi0, fb, 360);

        // --- EL CAMBIO CLAVE ---
        // NO ponemos apply_cs(0xFFFF) inmediatamente.
        // Añadimos un pequeño delay de "exposición" para que el 
        // driver interno de la OLED estabilice el brillo.
        sleep_us(500); 
        
        apply_cs(0xFFFF); 
    }
};