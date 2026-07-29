#pragma once
#include <stdint.h>
#include <stdlib.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pinout.h"
#include "pico/time.h"
#include "hid_hires.h"

// ==========================================================================
// Rueda de scroll magnética AS5600 (12 bits, 4096 cuentas por vuelta)
// --------------------------------------------------------------------------
// El muestreo antiguo promediaba bloques de 10 muestras y solo entregaba un
// delta cada ~16 ms (60 Hz), además de consultar el registro de estado del
// imán en cada iteración del bucle principal (dos transacciones I2C extra).
// Eso limitaba la fluidez y añadía latencia de grupo.
//
// Diseño actual:
//   - Muestreo del ángulo a 1 kHz, posición desenrollada en cuentas absolutas.
//   - Filtro EMA ligero sobre la posición (no sobre el delta): suaviza el
//     ruido de ±1-2 LSB sin perder desplazamiento acumulado ni derivar.
//   - Salida cada 4 ms (250 Hz) con banda muerta de arranque, de modo que la
//     rueda quieta no inunda el USB con micro-oscilaciones.
//   - Presencia del imán consultada una vez cada 500 ms.
// ==========================================================================
class HardwareAs5600 {
private:
    static constexpr uint8_t AS5600_ADDR = 0x36;
    static constexpr uint8_t REG_STATUS  = 0x0B;
    static constexpr uint8_t REG_ANGLE   = 0x0E;

    // Cadencias
    static constexpr uint32_t SAMPLE_US       = 1000;    // 1 kHz de muestreo
    static constexpr uint32_t OUTPUT_US       = 4000;    // 250 Hz de salida
    static constexpr uint32_t MAGNET_CHECK_US = 500000;  // 0.5 s

    // Filtrado y banda muerta
    static constexpr int EMA_SHIFT        = 2;   // alpha = 1/4
    static constexpr int32_t START_DEADBAND = 3; // cuentas para "arrancar"
    static constexpr uint32_t IDLE_US       = 120000; // 120 ms sin movimiento
    static constexpr int32_t DELTA_CLAMP    = 2048;   // media vuelta por bloque

    uint16_t lastRawAngle = 0;
    int32_t  rawPos       = 0;   // posición desenrollada, en cuentas
    int32_t  filtPosQ8    = 0;   // EMA de rawPos en punto fijo Q8
    int32_t  lastEmitPos  = 0;   // posición filtrada en la última emisión
    bool     moving       = false;

    uint32_t lastSampleUs = 0;
    uint32_t lastOutputUs = 0;
    uint32_t lastMagnetUs = 0;
    uint32_t lastMoveUs   = 0;
    bool     magnetOk     = false;

    // Lecturas I2C con timeout: una lectura bloqueante clásica puede colgar el
    // bucle principal indefinidamente si el bus se queda enganchado.
    bool readRegister(uint8_t reg, uint8_t* dst, size_t len) {
        if (i2c_write_timeout_us(i2c1, AS5600_ADDR, &reg, 1, true, 2000) < 0) return false;
        if (i2c_read_timeout_us(i2c1, AS5600_ADDR, dst, len, false, 2000) < 0) return false;
        return true;
    }

    bool readAngle(uint16_t& out) {
        uint8_t buf[2] = {0};
        if (!readRegister(REG_ANGLE, buf, 2)) return false;
        out = (uint16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF);
        return true;
    }

    void refreshMagnetStatus() {
        uint8_t status = 0;
        // Bit MD (0x20) = imán detectado.
        magnetOk = readRegister(REG_STATUS, &status, 1) && (status & 0x20);
    }

public:
    void init() {
        // Bus I2C1 a 400 kHz (GP14 SDA, GP15 SCL)
        i2c_init(i2c1, 400 * 1000);
        gpio_set_function(Pins::I2C_SDA, GPIO_FUNC_I2C);
        gpio_set_function(Pins::I2C_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(Pins::I2C_SDA);
        gpio_pull_up(Pins::I2C_SCL);

        sleep_ms(10); // arranque del chip

        uint16_t a = 0;
        readAngle(a);
        lastRawAngle = a;
        rawPos       = 0;
        filtPosQ8    = 0;
        lastEmitPos  = 0;
        moving       = false;

        uint32_t now  = to_us_since_boot(get_absolute_time());
        lastSampleUs  = now;
        lastOutputUs  = now;
        lastMagnetUs  = now;
        lastMoveUs    = now;

        refreshMagnetStatus();
    }

    bool magnetPresent() const { return magnetOk; }

    // Ángulo absoluto dentro de la vuelta (0-4095). Es la posición real del
    // imán, así que la app puede dibujar la rueda justo donde está en la mesa
    // en lugar de acumular incrementos, que se descuadran al reconectar.
    uint16_t rawAngle() const { return lastRawAngle; }

    // Debe llamarse en cada iteración del bucle principal. Devuelve true y
    // rellena delta_counts (cuentas del AS5600) cuando toca emitir un bloque.
    bool poll(int32_t& delta_counts) {
        uint32_t now = to_us_since_boot(get_absolute_time());

        if ((uint32_t)(now - lastMagnetUs) >= MAGNET_CHECK_US) {
            lastMagnetUs = now;
            refreshMagnetStatus();
        }
        if (!magnetOk) return false;

        // --- Muestreo ---
        if ((uint32_t)(now - lastSampleUs) >= SAMPLE_US) {
            lastSampleUs = now;

            uint16_t raw;
            if (readAngle(raw)) {
                int32_t diff = (int32_t)lastRawAngle - (int32_t)raw;
                if      (diff >  2048) diff -= 4096;
                else if (diff < -2048) diff += 4096;
                lastRawAngle = raw;
                rawPos += diff;

                // EMA sobre la posición: filtra ruido sin perder recorrido.
                filtPosQ8 += (((rawPos << 8) - filtPosQ8) >> EMA_SHIFT);
            }
        }

        // --- Emisión ---
        if ((uint32_t)(now - lastOutputUs) < OUTPUT_US) return false;
        lastOutputUs = now;

        int32_t filtPos = filtPosQ8 >> 8;
        int32_t d = filtPos - lastEmitPos;

        if (!moving) {
            // Banda muerta de arranque: no consumimos el resto, así que un
            // giro lento sigue acumulando hasta superar el umbral.
            if (d > -START_DEADBAND && d < START_DEADBAND) return false;
            moving = true;
        } else if (d == 0) {
            if ((uint32_t)(now - lastMoveUs) >= IDLE_US) moving = false;
            return false;
        }

        lastMoveUs  = now;
        lastEmitPos = filtPos;

        if (d >  DELTA_CLAMP) d =  DELTA_CLAMP;
        if (d < -DELTA_CLAMP) d = -DELTA_CLAMP;

        delta_counts = d;
        return true;
    }
};
