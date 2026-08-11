#pragma once

#include <stdint.h>

// Versión del firmware, en un único sitio.
//
// Antes vivía escrita a mano en tres: el handshake de main.cpp (`FW=4.1`), el
// `bcdDevice` de src/usb_descriptors.c y la tabla de compatibilidad de la app.
// Los tres se desincronizaron alguna vez, y la app decide qué comandos puede
// mandar mirando justo ese número: si miente, manda comandos que el teclado no
// conoce y cada uno se queda esperando una respuesta que nunca llega.
//
// Al subir la versión hay que tocar también, en el mismo commit:
//   - docs/COMPATIBILIDAD.md          (qué versión trae qué)
//   - OrbyGUI/src/compat.js           (hasta dónde llega esta app)

#define ORBY_FW_MAJOR 4
#define ORBY_FW_MINOR 6

#define ORBY_STR_(x) #x
#define ORBY_STR(x)  ORBY_STR_(x)

// "4.1" — lo que anuncia el handshake y lo que la app compara.
#define ORBY_FW_VERSION ORBY_STR(ORBY_FW_MAJOR) "." ORBY_STR(ORBY_FW_MINOR)

// La misma versión en BCD para el descriptor USB (4.1 -> 0x0410). Windows la
// enseña en las propiedades del dispositivo, así que tiene que coincidir con la
// del handshake o el aparato dice dos cosas distintas según a quién se pregunte.
#define ORBY_USB_BCD_DEVICE ((uint16_t)((ORBY_FW_MAJOR << 8) | (ORBY_FW_MINOR << 4)))
