#pragma once
#include "layout.h"
#include "class/hid/hid.h"

// Distribución US. Los usages HID de la página 0x07 son, por definición, la
// posición física de la tecla en un teclado QWERTY estadounidense: por eso la
// tabla ASCII→tecla que ya trae TinyUSB (pensada para que un dispositivo
// mande texto por HID) es exactamente esta distribución, carácter a
// carácter. Se reutiliza tal cual en vez de escribirla a mano: es la fuente
// de verdad de la propia pila USB, y no se puede desincronizar de ella.
//
// Sin teclas muertas: el ASCII imprimible no las necesita, y cualquier
// carácter fuera de 0x20-0x7E (una tilde, una eñe...) no se puede escribir
// con esta distribución. Se cuenta como no mapeable, igual que en ES.
static inline LayoutLookup layout_lookup_us(uint32_t cp) {
    static const uint8_t TABLE[128][2] = { HID_ASCII_TO_KEYCODE };
    LayoutLookup r = { false, false, { 0, 0 }, { 0, 0 } };
    if (cp >= 128 || TABLE[cp][1] == 0) return r;
    r.ok = true;
    r.key.mod   = TABLE[cp][0] ? KEYBOARD_MODIFIER_LEFTSHIFT : 0;
    r.key.usage = TABLE[cp][1];
    return r;
}
