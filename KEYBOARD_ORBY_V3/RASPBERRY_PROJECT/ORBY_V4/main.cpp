#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "bsp/board.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include "pinout.h"
#include "hardware_encoder.h"
#include "hardware_oled.h"
#include "oled_digits.h"

// ==========================================
// CORE 1: MANEJA EXCLUSIVAMENTE LAS PANTALLAS
// ==========================================
void core1_entry() {
    HardwareOled oleds;
    oleds.init_all_screens();

    uint8_t framebuffer[360];
    for (int i = 1; i <= 10; i++) {
        OledDigits::render_number_to_framebuffer(i, framebuffer);
        oleds.paint_screen(i, framebuffer);
    }

    while (true) {
        uint32_t msg = multicore_fifo_pop_blocking();
        uint8_t oled_idx = (msg >> 1);
        bool is_pressed  = (msg & 1);
        oleds.invert_screen(oled_idx + 1, is_pressed);
    }
}

// ==========================================
// CALLBACKS CDC (SERIAL) — Core 0
// ==========================================
#define CMD_BUF_SIZE 256
static char cmd_buf[CMD_BUF_SIZE];
static int  cmd_pos = 0;

void process_command(const char* cmd) {
    // ACK — handshake info del firmware
    if (strncmp(cmd, "ACK", 3) == 0) {
        const char* resp = "ORBY_V4:FW=1.0:KEYS=10:ENCODERS=2:MODE=SERIAL\n";
        tud_cdc_n_write(0, resp, strlen(resp));
        tud_cdc_n_write_flush(0);
        return;
    }
}

void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    char tmp[64];
    uint32_t count = tud_cdc_n_read(itf, tmp, sizeof(tmp));
    for (uint32_t i = 0; i < count; i++) {
        char c = tmp[i];
        if (c == '\n' || c == '\r') {
            if (cmd_pos > 0) {
                cmd_buf[cmd_pos] = 0;
                process_command(cmd_buf);
                cmd_pos = 0;
            }
        } else if (cmd_pos < CMD_BUF_SIZE - 1) {
            cmd_buf[cmd_pos++] = c;
        }
    }
}

// ==========================================
// CORE 0: ENCODERS, TECLAS Y USB CDC
// ==========================================
int main() {
    board_init();
    tusb_init();
    sleep_ms(2000);

    multicore_launch_core1(core1_entry);

    HardwareEncoder rueda_izq(pio0, Pins::ENC1_A, 1);
    HardwareEncoder rueda_der(pio0, Pins::ENC2_A, -1);

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
        tud_task();

        // --- ENCODER IZQUIERDO ---
        int delta_izq = rueda_izq.get_delta();
        if (delta_izq != 0) {
            char tel[32];
            int len = snprintf(tel, sizeof(tel), "ENC:1:%d\n", delta_izq);
            tud_cdc_n_write(0, tel, len);
            tud_cdc_n_write_flush(0);
        }

        // --- ENCODER DERECHO ---
        int delta_der = rueda_der.get_delta();
        if (delta_der != 0) {
            char tel[32];
            int len = snprintf(tel, sizeof(tel), "ENC:2:%d\n", delta_der);
            tud_cdc_n_write(0, tel, len);
            tud_cdc_n_write_flush(0);
        }

        // --- TECLAS ---
        for (int i = 0; i < 10; i++) {
            bool is_pressed = !gpio_get(key_pins[i]);
            if (is_pressed != last_key_state[i]) {
                last_key_state[i] = is_pressed;

                // OLED Core 1
                uint32_t msg = (i << 1) | (is_pressed ? 1 : 0);
                multicore_fifo_push_blocking(msg);

                // Telemetría serial PC
                char tel[24];
                int len = snprintf(tel, sizeof(tel), "KEY_EV:%d:%d\n", i+1, is_pressed ? 1 : 0);
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);
            }
        }
        sleep_ms(1);
    }
    return 0;
}