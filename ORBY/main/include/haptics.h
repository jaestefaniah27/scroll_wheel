#pragma once
#include <Arduino.h>

// Arranca un pulso háptico no bloqueante
void startHaptic(uint16_t ms);

// Actualiza estado del motor háptico (llamar en cada loop)
void updateHaptics();

// Inicializa pines del háptico
void initHaptics();


