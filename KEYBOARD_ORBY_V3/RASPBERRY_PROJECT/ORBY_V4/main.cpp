#include <stdio.h>
#include "pico/stdlib.h"
#include "pinout.h"
#include "hardware_encoder.h" // Nuestra nueva abstracción

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("=== SISTEMA DUAL-CORE INICIADO ===\n");

    // Instanciación del hardware. Listo para usar.
    // Encoder 1 en sentido normal (1), Encoder 2 invertido (-1)
    HardwareEncoder rueda_izq(pio0, Pins::ENC1_A, 1);
    HardwareEncoder rueda_der(pio0, Pins::ENC2_A, -1);

    while (true) {
        
        // Solo tenemos que pedirle el delta al objeto
        int delta_izq = rueda_izq.get_delta();
        if (delta_izq != 0) {
            printf("Rueda IZQ: Movimiento %+d | Absoluto: %d\n", delta_izq, rueda_izq.get_absolute());
            // Aquí llamarías a: USB_Send_Volume(delta_izq);
        }

        int delta_der = rueda_der.get_delta();
        if (delta_der != 0) {
            printf("Rueda DER: Movimiento %+d | Absoluto: %d\n", delta_der, rueda_der.get_absolute());
            // Aquí llamarías a: USB_Send_Zoom(delta_der);
        }

        sleep_ms(1); // Relax para la CPU principal
    }
    
    return 0;
}