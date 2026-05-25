#pragma once
#include <Arduino.h>

// Inicializa I2C y variables del encoder
void initEncoder();

// Centra la referencia de posición
void centerPosition();

// Devuelve true si hay un delta listo (entrega constrain a ±Ui::DELTA_MAX)
bool sampleBlockAndGetDelta(int16_t& delta_out);

// (Opcional) Presencia de imán
bool magnetPresent();
