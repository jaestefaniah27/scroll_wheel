#pragma once
#include <stdint.h>

namespace Pins {

    // --- MATRIZ DE TECLAS (1 a 12) ---
    constexpr uint8_t KEY_1  = 1;
    constexpr uint8_t KEY_2  = 2;
    constexpr uint8_t KEY_3  = 3;
    constexpr uint8_t KEY_4  = 4;
    constexpr uint8_t KEY_5  = 5;
    constexpr uint8_t KEY_6  = 6;
    constexpr uint8_t KEY_7  = 7;
    constexpr uint8_t KEY_8  = 8;
    constexpr uint8_t KEY_9  = 9;
    constexpr uint8_t KEY_10 = 10;
    constexpr uint8_t KEY_11 = 11;
    constexpr uint8_t KEY_12 = 12;

    // --- ENCODERS MECÁNICOS (EC11) ---
    // Encoder 1
    constexpr uint8_t ENC1_A  = 26;
    constexpr uint8_t ENC1_B  = 27;
    constexpr uint8_t ENC1_SW = 28;
    
    // Encoder 2
    constexpr uint8_t ENC2_A  = 23;
    constexpr uint8_t ENC2_B  = 24;
    constexpr uint8_t ENC2_SW = 25;

    // --- I2C (SENSOR MAGNÉTICO AS5600) ---
    constexpr uint8_t I2C_SDA = 14; 
    constexpr uint8_t I2C_SCL = 15;

    // --- SUBSISTEMA OLED (SPI + 74HC595) ---
    // Shift Registers (CS Multiplexing)
    constexpr uint8_t SHIFT_DATA  = 17;
    constexpr uint8_t SHIFT_CLK   = 13;
    constexpr uint8_t SHIFT_LATCH = 16;
    
    // Bus SPI dedicado
    constexpr uint8_t SPI_SCK  = 18;
    constexpr uint8_t SPI_MOSI = 19;
    
    // Control OLED
    constexpr uint8_t OLED_DC  = 20;
    constexpr uint8_t OLED_RST = 21;
    
} // namespace Pins

// Mapa de correspondencia: Número de OLED (1-10) -> Bit en el 74HC595
// Permite aislar la abstracción matemática del ruteo físico de la PCB
namespace OledMap {
    constexpr uint8_t CS_BITS[10] = {
        5,  // OLED 1  -> U3 Q5
        6,  // OLED 2  -> U3 Q6
        7,  // OLED 3  -> U3 Q7
        2,  // OLED 4  -> U3 Q2
        3,  // OLED 5  -> U3 Q3
        4,  // OLED 6  -> U3 Q4
        0,  // OLED 7  -> U3 Q0
        1,  // OLED 8  -> U3 Q1
        14, // OLED 9  -> U4 Q6
        13  // OLED 10 -> U4 Q5
    };
} // namespace OledMap