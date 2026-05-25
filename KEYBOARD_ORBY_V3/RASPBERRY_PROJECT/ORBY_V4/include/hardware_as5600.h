#pragma once
#include <stdint.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pinout.h"
#include "pico/time.h"

class HardwareAs5600 {
private:
    static constexpr uint8_t AS5600_ADDR = 0x36;
    static constexpr uint8_t REG_STATUS  = 0x0B;
    static constexpr uint8_t REG_ANGLE   = 0x0E;

    uint16_t lastRawAngle = 0;
    int64_t  accumPos     = 0;
    int64_t  sumAngle     = 0;
    uint8_t  sampleCount  = 0;
    float    prevMean     = 0.0f;
    uint32_t lastSampleTime = 0;

    // Constantes idénticas al firmware original de Arduino:
    static constexpr uint32_t SAMPLE_US = 1000000UL / 600; // ~1666 us (muestreo a 600 Hz)
    static constexpr uint8_t  SAMPLES_PER_BLOCK = 10;       // 10 muestras por bloque (~60 Hz de salida)
    static constexpr float    DELTA_MAX = 32767.0f;

    uint8_t readStatusRegister() {
        uint8_t reg = REG_STATUS;
        int written = i2c_write_blocking(i2c1, AS5600_ADDR, &reg, 1, true);
        if (written < 0) return 0xFF;
        uint8_t val = 0;
        int read = i2c_read_blocking(i2c1, AS5600_ADDR, &val, 1, false);
        if (read < 0) return 0xFF;
        return val;
    }

    uint16_t readAngle() {
        uint8_t reg = REG_ANGLE;
        int written = i2c_write_blocking(i2c1, AS5600_ADDR, &reg, 1, true);
        if (written < 0) return lastRawAngle;
        uint8_t buf[2] = {0};
        int read = i2c_read_blocking(i2c1, AS5600_ADDR, buf, 2, false);
        if (read < 0) return lastRawAngle;
        return (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    }

    int16_t readDeltaAngle() {
        uint16_t raw = readAngle();
        int16_t diff = (int16_t)lastRawAngle - (int16_t)raw;
        if      (diff >  2048) diff -= 4096;
        else if (diff < -2048) diff += 4096;
        lastRawAngle = raw;
        return diff;
    }

    int64_t readAccumulatedAngle() {
        accumPos += (int64_t)readDeltaAngle();
        return accumPos;
    }

public:
    void init() {
        // Inicializar bus I2C1 a 400 kHz (GP14 SDA, GP15 SCL)
        i2c_init(i2c1, 400 * 1000);
        gpio_set_function(Pins::I2C_SDA, GPIO_FUNC_I2C);
        gpio_set_function(Pins::I2C_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(Pins::I2C_SDA);
        gpio_pull_up(Pins::I2C_SCL);

        sleep_ms(10); // Tiempo de arranque del chip AS5600
        lastRawAngle = readAngle();
        accumPos = 0;
        sumAngle = 0;
        sampleCount = 0;
        prevMean = 0.0f;
        lastSampleTime = to_us_since_boot(get_absolute_time());
    }

    bool magnetPresent() {
        uint8_t status = readStatusRegister();
        // Verificar que hay comunicación y que el bit MD (Magnet Detected, 0x20) está activo
        return (status != 0xFF) && (status & 0x20);
    }

    bool sampleBlockAndGetDelta(int16_t& delta_out) {
        uint32_t now = to_us_since_boot(get_absolute_time());

        // Si no se detecta imán, o no ha pasado el tiempo del slot, salimos de inmediato
        if (!magnetPresent()) {
            return false;
        }

        if ((now - lastSampleTime) < SAMPLE_US) {
            return false;
        }
        lastSampleTime += SAMPLE_US;

        int64_t pos = readAccumulatedAngle();
        sumAngle += pos;
        sampleCount++;

        if (sampleCount < SAMPLES_PER_BLOCK) {
            return false;
        }

        float mean = (float)sumAngle / SAMPLES_PER_BLOCK;
        float deltaF = mean - prevMean;
        prevMean = mean;

        sumAngle = 0;
        sampleCount = 0;

        if (deltaF >  DELTA_MAX)  deltaF =  DELTA_MAX;
        if (deltaF < -DELTA_MAX)  deltaF = -DELTA_MAX;

        delta_out = (int16_t)deltaF;
        return true;
    }
};
