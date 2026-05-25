#pragma once
#include <stdint.h>

namespace OledBitmaps {

    // Obtener el puntero al buffer del mapa de bits (72x40 píxeles = 360 bytes) por su índice (0 a 9)
    const uint8_t* get_bitmap_by_index(int idx);

} // namespace OledBitmaps
