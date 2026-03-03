#pragma once
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pinout.h"
#include "hc595.pio.h" 

class HardwareOled {
private:
    PIO pio_hw;
    uint sm;

    // Secuencia de inicialización del SSD1306 (72x40)
    const uint8_t init_seq[28] = {
        0xAE,           // Display OFF
        0xD5, 0x80,     // Clock Divide Ratio
        0xA8, 0x27,     // Multiplex Ratio (40px - 1)
        0xD3, 0x00,     // Display Offset
        0x40,           // Start Line 0
        0x8D, 0x14,     // Charge Pump ON
        0x20, 0x00,     // <--- MODO HORIZONTAL (Vital para que el FB cuadre)
        0xA1,           // Segment Remap (Giro 180 si es necesario)
        0xC8,           // COM Output Scan Direction
        0xDA, 0x12,     // COM Pins Hardware Config
        0x81, 0xAF,     // Contrast
        0xD9, 0xF1,     // Pre-charge Period
        0xDB, 0x40,     // VCOMH Deselect Level
        0xA4,           // Entire Display ON (Resume)
        0xA6,           // Normal Display
        0xAF            // Display ON
    };

    void apply_cs(uint8_t screen_num) {
        uint16_t mask = 0xFFFF; // Todo HIGH (deseleccionado)
        if (screen_num >= 1 && screen_num <= 10) {
            uint8_t target_bit = OledMap::CS_BITS[screen_num - 1];
            mask &= ~(1 << target_bit); // LOW solo en la pantalla elegida
        }
        pio_sm_put_blocking(pio_hw, sm, mask);
    }

    void send_command(uint8_t cmd) {
        gpio_put(Pins::OLED_DC, 0); // Modo comando
        spi_write_blocking(spi0, &cmd, 1);
    }

public:
    HardwareOled(PIO pio_inst) : pio_hw(pio_inst) {
        // Inicializar pines de control fijos
        gpio_init(Pins::OLED_DC);  gpio_set_dir(Pins::OLED_DC, GPIO_OUT);
        gpio_init(Pins::OLED_RST); gpio_set_dir(Pins::OLED_RST, GPIO_OUT);

        // Inicializar Hardware SPI a 10 MHz
        spi_init(spi0, 10000 * 1000); 
        gpio_set_function(Pins::SPI_SCK, GPIO_FUNC_SPI);
        gpio_set_function(Pins::SPI_MOSI, GPIO_FUNC_SPI);

        // Inicializar el PIO para el multiplexor (Shift Register)
        pio_add_program(pio_hw, &hc595_program);
        sm = pio_claim_unused_sm(pio_hw, true);
        hc595_program_init(pio_hw, sm, Pins::SHIFT_DATA, Pins::SHIFT_CLK, Pins::SHIFT_LATCH);
    }

    void init_all_screens() {
        // Reset físico global
        gpio_put(Pins::OLED_RST, 1); sleep_ms(1);
        gpio_put(Pins::OLED_RST, 0); sleep_ms(10);
        gpio_put(Pins::OLED_RST, 1); sleep_ms(10);

        // Abrir todas las pantallas (CS a LOW) para inicialización masiva
        pio_sm_put_blocking(pio_hw, sm, 0x0000);

        for (uint8_t cmd : init_seq) {
            send_command(cmd);
        }

        // Aislar todas
        apply_cs(0); 
    }

    // --- EL MÉTODO QUE FALTABA ---
    void clear_all() {
        uint8_t vacio[360] = {0};
        for(int i = 1; i <= 10; i++) {
            paint_screen(i, vacio);
        }
    }

    void paint_screen(uint8_t screen_num, const uint8_t* framebuffer, size_t size = 360) {
        apply_cs(screen_num);

        // 1. Definir el límite de columnas (0 a 71)
        // Esto obliga a la pantalla a hacer el "wrap-around" en el pixel 71
        send_command(0x21); 
        send_command(0x1C); // Inicio en 28 (Offset típico en pantallas de 72px centradas)
        send_command(0x1C + 71); // Fin
        
        // 2. Definir el límite de páginas (0 a 4)
        send_command(0x22);
        send_command(0x00); // Inicio
        send_command(0x04); // Fin (Página 4 es la última para 40px)
        
        // 3. Enviar los datos
        gpio_put(Pins::OLED_DC, 1); 
        spi_write_blocking(spi0, framebuffer, size);

        apply_cs(0);
    }
};