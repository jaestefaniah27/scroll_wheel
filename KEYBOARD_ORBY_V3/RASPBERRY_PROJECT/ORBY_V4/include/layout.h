#pragma once
#include <stdint.h>

// Tipos comunes a las tablas de distribución (layout_es.h, layout_us.h). Ver
// docs/PLAN_TEXTO_SIN_APP.md, Tarea 2: qué usage HID hay que pulsar para que
// salga cada carácter, con la distribución que tenga configurada el teclado.

enum TextLayout : uint8_t {
    TEXT_LAYOUT_ES = 0,
    TEXT_LAYOUT_US = 1,
};

// Una pulsación: modificador HID (KEYBOARD_MODIFIER_*, 0 = ninguno) + usage de
// la página 0x07. No es un KeyAction del firmware (ese es para atajos del
// usuario); este es interno, solo para el traductor de texto.
struct LayoutKey {
    uint8_t mod;
    uint8_t usage;
};

// El resultado de buscar un carácter en la tabla.
//   ok    — false si esta distribución no sabe escribirlo: se salta y se
//           cuenta (ver TEXT_END y text_player), nunca se cuelga ni inventa
//           otro carácter.
//   dead  — true si hace falta pulsar y soltar `dead_key` (una tecla muerta:
//           acento, diéresis...) antes de `key`. Son dos pulsaciones HID
//           reales y separadas, no una combinación.
struct LayoutLookup {
    bool      ok;
    bool      dead;
    LayoutKey dead_key;
    LayoutKey key;
};
