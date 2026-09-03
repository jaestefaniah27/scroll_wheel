#pragma once
#include "layout.h"
#include "layout_us.h"
#include "class/hid/hid.h"

// Distribución español (España), ISO. Las letras, los dígitos sin mayúscula y
// el espacio están en la misma posición física que en US (misma tecla, mismo
// carácter): se reutiliza para ellos la tabla ASCII de TinyUSB (ver
// layout_us.h) y aquí solo se corrige lo que de verdad cambia.
//
// Confianza de esta tabla, de más a menos:
//   1. Letras, dígitos, espacio: iguales que en US. Certeza total.
//   2. áéíóú/ÁÉÍÓÚ (tecla muerta ´), ü/Ü (tecla muerta ¨), ñ/Ñ, ¿¡: lo más
//      característico del teclado español y lo más repasado.
//   3. Fila de símbolos con Mayús (! " · $ % & / ( ) =), y ; : < > _ - ' ? +:
//      posición bien conocida del teclado ISO español.
//   4. Símbolos con AltGr (@ # ~ € \ | [ ] {): la posición física es la
//      habitual, pero SIN TECLADO DELANTE no se puede jurar carácter a
//      carácter. Es justo lo que existe para comprobar la Tarea 8 del plan
//      (docs/PLAN_TEXTO_SIN_APP.md): un botón que hace escribir al teclado un
//      texto de prueba con todo esto y compara lo que sale.
//   5. ^ ` } y *: fuera de la tabla a propósito. Los tres primeros son tecla
//      muerta o de dudosa combinación en este teclado; incluir una posición
//      equivocada sería peor que decir "no mapeable" (se salta y se cuenta,
//      nunca escribe otra cosa).
//
// Las teclas muertas son DOS pulsaciones HID reales y separadas (pulsar y
// soltar la muerta, luego pulsar y soltar la base), nunca una combinación:
// así es como funciona de verdad un teclado con tecla muerta.
static const LayoutKey ES_DEAD_ACUTE    = { 0,                        HID_KEY_APOSTROPHE }; // ´
static const LayoutKey ES_DEAD_DIERESIS = { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_APOSTROPHE }; // ¨

struct EsDirectEntry { uint32_t cp; LayoutKey key; };
struct EsDeadEntry    { uint32_t cp; LayoutKey dead; LayoutKey key; };

// Todo lo que no está en la fila de letras/dígitos/espacio, y sale con una
// sola pulsación (sin tecla muerta). Un carácter de aquí SIEMPRE gana a lo
// que diría la tabla US heredada: por eso se mira esta lista antes que nada.
static const EsDirectEntry ES_DIRECT[] = {
    // --- Fila de símbolos con Mayús: 1234567890 -> !"·$%&/()= ---------------
    { '!',    { 0, HID_KEY_1 } },                                    // = US, se deja por claridad
    { '"',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_2 } },
    { 0x00B7, { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_3 } },           // ·
    { '$',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_4 } },
    { '%',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_5 } },
    { '&',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_6 } },
    { '/',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_7 } },
    { '(',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_8 } },
    { ')',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_9 } },
    { '=',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_0 } },

    // --- La tecla ' ? (donde US tiene el guión) ----------------------------
    { '\'',   { 0,                          HID_KEY_MINUS } },
    { '?',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_MINUS } },

    // --- La tecla ¡ ¿ (donde US tiene el igual) -----------------------------
    { 0x00A1, { 0,                          HID_KEY_EQUAL } },        // ¡
    { 0x00BF, { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_EQUAL } },       // ¿

    // --- La tecla + * (donde US tiene ] ) -----------------------------------
    { '+',    { 0,                          HID_KEY_BRACKET_RIGHT } },
    { '*',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_BRACKET_RIGHT } },

    // --- ñ Ñ (donde US tiene ; ) --------------------------------------------
    { 0x00F1, { 0,                          HID_KEY_SEMICOLON } },    // ñ
    { 0x00D1, { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_SEMICOLON } },   // Ñ

    // --- , ; (donde US solo tiene la coma) ----------------------------------
    { ';',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_COMMA } },
    // --- . : (donde US solo tiene el punto) ---------------------------------
    { ':',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_PERIOD } },
    // --- - _ (donde US tiene / ? ) ------------------------------------------
    { '-',    { 0,                          HID_KEY_SLASH } },
    { '_',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_SLASH } },
    // --- < > (tecla extra ISO, sin equivalente en US) -----------------------
    { '<',    { 0,                          HID_KEY_EUROPE_2 } },
    { '>',    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_EUROPE_2 } },

    // --- º ª (donde US tiene la tilde grave `) -------------------------------
    { 0x00BA, { 0,                          HID_KEY_GRAVE } },        // º
    { 0x00AA, { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_GRAVE } },       // ª

    // --- AltGr: verificar con la Tarea 8 antes de darlos por buenos ---------
    { '@',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_2 } },
    { '#',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_3 } },
    { '~',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_4 } },
    { '|',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_1 } },
    { '\\',   { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_GRAVE } },
    { '[',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_BRACKET_LEFT } },
    { ']',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_BRACKET_RIGHT } },
    { '{',    { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_APOSTROPHE } },
    { 0x20AC, { KEYBOARD_MODIFIER_RIGHTALT, HID_KEY_E } },            // €
};

// Vocales con tecla muerta delante: acento agudo para á é í ó ú (y sus
// mayúsculas), diéresis solo para ü/Ü (la única que se usa en castellano,
// "güe"/"güi").
static const EsDeadEntry ES_DEAD[] = {
    { 0x00E1, ES_DEAD_ACUTE,    { 0, HID_KEY_A } },                          // á
    { 0x00E9, ES_DEAD_ACUTE,    { 0, HID_KEY_E } },                          // é
    { 0x00ED, ES_DEAD_ACUTE,    { 0, HID_KEY_I } },                          // í
    { 0x00F3, ES_DEAD_ACUTE,    { 0, HID_KEY_O } },                          // ó
    { 0x00FA, ES_DEAD_ACUTE,    { 0, HID_KEY_U } },                          // ú
    { 0x00C1, ES_DEAD_ACUTE,    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_A } }, // Á
    { 0x00C9, ES_DEAD_ACUTE,    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_E } }, // É
    { 0x00CD, ES_DEAD_ACUTE,    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_I } }, // Í
    { 0x00D3, ES_DEAD_ACUTE,    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_O } }, // Ó
    { 0x00DA, ES_DEAD_ACUTE,    { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_U } }, // Ú
    { 0x00FC, ES_DEAD_DIERESIS, { 0, HID_KEY_U } },                          // ü
    { 0x00DC, ES_DEAD_DIERESIS, { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_U } }, // Ü
};

// ^ ` } están aquí a propósito, para que NO caigan en el `return
// layout_lookup_us(cp)` de más abajo: en US son ASCII normal, pero en este
// teclado son tecla muerta o de posición incierta (ver el punto 5 de la
// cabecera del fichero). Sin esta lista, layout_lookup_es() heredaría de la
// tabla US una posición que en este teclado escribe otra cosa.
static const uint32_t ES_EXCLUDE[] = { '^', '`', '}' };

static inline LayoutLookup layout_lookup_es(uint32_t cp) {
    for (unsigned i = 0; i < sizeof(ES_DEAD) / sizeof(ES_DEAD[0]); i++) {
        if (ES_DEAD[i].cp == cp) {
            return { true, true, ES_DEAD[i].dead, ES_DEAD[i].key };
        }
    }
    for (unsigned i = 0; i < sizeof(ES_DIRECT) / sizeof(ES_DIRECT[0]); i++) {
        if (ES_DIRECT[i].cp == cp) {
            return { true, false, { 0, 0 }, ES_DIRECT[i].key };
        }
    }
    for (unsigned i = 0; i < sizeof(ES_EXCLUDE) / sizeof(ES_EXCLUDE[0]); i++) {
        if (ES_EXCLUDE[i] == cp) return { false, false, { 0, 0 }, { 0, 0 } };
    }
    // Nada específico de ES ni excluido: si es ASCII imprimible, la posición
    // física coincide con la de US (letras, dígitos sin mayúscula, espacio...).
    return layout_lookup_us(cp);
}
