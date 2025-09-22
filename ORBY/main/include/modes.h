#pragma once
#include "events.h"

// Cambiar modo explícitamente y consultar el actual
void setWheelMode(WheelMode m);
WheelMode getWheelMode();

// Cambia el modo en función del evento de botón (R/L, corto/largo)
void changeMode(ButtonEvent ev);

// Control de LEDs según modo
void setLedsForMode(WheelMode m);
