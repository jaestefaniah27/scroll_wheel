#pragma once
#include <Arduino.h>

// Inicializa botones (pines + estados)
void initButtons();

// Lee botones, emite cambio de modo y feedback de LONG_HIT.
// No bloquea. Llama en cada loop.
void pollButtonsAndFeedback();
