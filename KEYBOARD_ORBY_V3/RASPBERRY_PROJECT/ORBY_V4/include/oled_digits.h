#pragma once
#include <stdint.h>
#include <string.h>

namespace OledDigits {

    // Standard 5x8 font for digits 0-9
    static const uint8_t font_digits[10][5] = {
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

    inline void render_number_to_framebuffer(uint8_t number, uint8_t* framebuffer) {
        memset(framebuffer, 0, 360); // Clear framebuffer to 0
        
        int d1 = number / 10;
        int d2 = number % 10;
        
        // Target page 2 (middle of the height), columns 30-40 (middle of the width)
        int page = 2;
        int col_offset = (d1 > 0) ? 28 : 31; // Center appropriately
        
        if (d1 > 0) {
            for (int i = 0; i < 5; i++) {
                framebuffer[page * 72 + col_offset + i] = font_digits[d1][i];
            }
            col_offset += 7;
        }
        
        for (int i = 0; i < 5; i++) {
            framebuffer[page * 72 + col_offset + i] = font_digits[d2][i];
        }
    }

} // namespace OledDigits
