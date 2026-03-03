#pragma once
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pinout.h"
#include "hc595.pio.h" 

class HardwareOled {
private:
    PIO pio_hw;
    uint sm;

    // Secuencia mágica de inicialización del SSD1306 (72x40)
    const uint8_t init_seq[25] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x27, 0xD3, 0x00, 0x40, 
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };

    void apply_cs(uint8_t screen_num) {
        uint16_t mask = 0xFFFF; // Todo HIGH (aislado)
        if (screen_num >= 1 && screen_num <= 10) {
            uint8_t target_bit = OledMap::CS_BITS[screen_num - 1];
            mask &= ~(1 << target_bit); // Ponemos a LOW solo el bit de esa pantalla
        }
        // Disparamos la máscara al coprocesador PIO
        pio_sm_put_blocking(pio_hw, sm, mask);
    }

    void send_command(uint8_t cmd) {
        gpio_put(Pins::OLED_DC, 0); // Modo comando
        spi_write_blocking(spi0, &cmd, 1);
    }

public:
    HardwareOled(PIO pio_inst) : pio_hw(pio_inst) {
        // 1. Inicializar pines de control OLED fijos
        gpio_init(Pins::OLED_DC);  gpio_set_dir(Pins::OLED_DC, GPIO_OUT);
        gpio_init(Pins::OLED_RST); gpio_set_dir(Pins::OLED_RST, GPIO_OUT);

        // 2. Inicializar Hardware SPI a 10 MHz
        spi_init(spi0, 10000 * 1000); 
        gpio_set_function(Pins::SPI_SCK, GPIO_FUNC_SPI);
        gpio_set_function(Pins::SPI_MOSI, GPIO_FUNC_SPI);

        // 3. Inicializar el PIO para el multiplexor
        pio_add_program(pio_hw, &hc595_program);
        sm = pio_claim_unused_sm(pio_hw, true);
        hc595_program_init(pio_hw, sm, Pins::SHIFT_DATA, Pins::SHIFT_CLK, Pins::SHIFT_LATCH);
    }

    void init_all_screens() {
        // Reset físico global
        gpio_put(Pins::OLED_RST, 1); sleep_ms(1);
        gpio_put(Pins::OLED_RST, 0); sleep_ms(10);
        gpio_put(Pins::OLED_RST, 1); sleep_ms(10);

        // Abrir TODAS las pantallas a la vez (Máscara 0x0000 = todos los CS a LOW)
        pio_sm_put_blocking(pio_hw, sm, 0x0000);

        // Mandar la secuencia de arranque 
        for (uint8_t cmd : init_seq) {
            send_command(cmd);
        }

        // Aislar todas (Máscara 0xFFFF)
        apply_cs(0); 
    }

    void paint_screen(uint8_t screen_num, const uint8_t* framebuffer, size_t size = 360) {
        apply_cs(screen_num); // El PIO enruta el hardware instantáneamente

        // Posicionar el cursor de la pantalla en (0,0)
        send_command(0x21); send_command(0); send_command(71);
        send_command(0x22); send_command(0); send_command(4);
        
        // Modo datos
        gpio_put(Pins::OLED_DC, 1); 
        
        // El SPI dispara la imagen a máxima velocidad
        spi_write_blocking(spi0, framebuffer, size);

        apply_cs(0); // Volver a cerrar el canal
    }
};