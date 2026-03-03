#pragma once
#include "hardware/pio.h"
#include "encoder.pio.h" // El C generado por CMake con nuestro ensamblador

class HardwareEncoder {
private:
    PIO pio_hw;
    uint sm;
    int dir_multiplier;
    
    int pos_raw;   // Fotografía eléctrica de la FIFO
    int clics_ant; // Memoria del estado lógico anterior

    // Variables estáticas compartidas por todos los objetos de esta clase
    // para garantizar que el ensamblador solo se carga una vez por bloque PIO.
    inline static bool program_loaded_pio0 = false;
    inline static bool program_loaded_pio1 = false;

public:
    // Constructor: Configura y arranca el hardware automáticamente
    HardwareEncoder(PIO pio_inst, uint pin_a, int direccion = 1) : 
        pio_hw(pio_inst), dir_multiplier(direccion), pos_raw(0), clics_ant(0) {
        
        // 1. Cargar el programa en memoria RAM del PIO de forma segura
        if (pio_hw == pio0 && !program_loaded_pio0) {
            pio_add_program(pio0, &encoder_program);
            program_loaded_pio0 = true;
        } else if (pio_hw == pio1 && !program_loaded_pio1) {
            pio_add_program(pio1, &encoder_program);
            program_loaded_pio1 = true;
        }

        // 2. Buscar un núcleo de procesamiento (State Machine) libre
        sm = pio_claim_unused_sm(pio_hw, true);

        // 3. Conectar los pines físicos y arrancar el reloj a 1MHz
        encoder_program_init(pio_hw, sm, pin_a);
    }

    // Método principal: Devuelve los clics relativos y actualiza la memoria
    int get_delta() {
        // Extraemos de la FIFO usando la macro estática inyectada por el SDK
        pos_raw = encoder_get_count(pio_hw, sm, pos_raw);

        // Traducimos a resolución física (división) y aplicamos la inversión de giro
        int clics_act = (pos_raw / 2) * dir_multiplier;

        // Calculamos la distancia circular en 32 bits para evitar fallos por Wrap-Around
        int delta = (int)((unsigned int)clics_act - (unsigned int)clics_ant);

        // Si hubo movimiento, actualizamos el punto de referencia
        if (delta != 0) {
            clics_ant = clics_act;
        }

        return delta;
    }

    // Método auxiliar por si necesitas mapear el encoder a un valor fijo (ej. 0 a 100%)
    int get_absolute() {
        return clics_ant;
    }
};