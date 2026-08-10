#ifndef CONFIG_HASH_H
#define CONFIG_HASH_H

// ============================================================================
// HUELLA DE LA CONFIGURACIÓN (CRC32)
// ============================================================================
// La app de escritorio guarda en el PC una copia completa de lo que hay en el
// teclado. Sin una forma barata de saber si las dos siguen coincidiendo, la
// única opción era releerlo todo por el CDC en cada conexión: un GET_PROFILE
// por perfil más un GET_OLED por icono, que con 16 perfiles de 4 páginas son
// cientos de idas y vueltas.
//
// Con GET_HASH basta una: si el CRC global coincide, la copia del PC vale tal
// cual; si no, los CRC por perfil dicen cuáles hay que releer.
//
// El CRC NO se calcula sobre la estructura en memoria: el relleno de los struct
// y la basura que queda detrás del terminador de los char[8] no son datos, y la
// app no puede verlos. Se calcula sobre una serialización canónica, definida
// aquí, que la app repite byte a byte en OrbyGUI/src/hash.js.
//
// CUALQUIER CAMBIO EN EL ORDEN O EL FORMATO DE LOS CAMPOS HAY QUE HACERLO
// TAMBIÉN EN hash.js. Si no, las huellas dejan de cuadrar y la app vuelve a
// descargarlo todo en cada conexión: molesto, pero no rompe nada, porque ante
// la duda se descarga, que es lo que hacía antes de existir esto.
//
// Vive en su propia cabecera para que tools/test/test_config_hash.cpp pueda
// compilarla en el PC y comprobar que da lo mismo que la versión de JavaScript.

#include <stdint.h>
#include <stddef.h>

// Tamaños de la serialización. Son los del firmware; el hueco de iconos mide
// OLED_FB_SIZE y hay OLED_SLOTS por página.
#define CFGHASH_LABELS   20
#define CFGHASH_KEYS     24
#define CFGHASH_ROTARY   16
#define CFGHASH_SLOTS    20
#define CFGHASH_FB_SIZE  360

// Hueco 20, solo en la página 0: el icono del perfil, aparte de los CFGHASH_SLOTS
// de las teclas. No entra en el `& 0xFFFFF` de la máscara por página de abajo.
#define CFGHASH_PROFILE_SLOT 20

#define CFGHASH_INIT 0xFFFFFFFFu

// Tabla de 16 entradas (medio byte por vuelta): la de 256 costaría 1 KB de
// Flash para ahorrar unos milisegundos en algo que se pide una vez al conectar.
inline constexpr uint32_t CFGHASH_NIBBLE[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
    0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
    0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
};

inline uint32_t crc32_feed(uint32_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        crc = (crc >> 4) ^ CFGHASH_NIBBLE[crc & 0x0F];
        crc = (crc >> 4) ^ CFGHASH_NIBBLE[crc & 0x0F];
    }
    return crc;
}

// Un char[8] del firmware entra siempre como 8 bytes: el texto hasta el
// terminador y ceros hasta el final. Lo que hay detrás del terminador es basura
// de lo que hubiera antes, y la app solo recibe el texto (ver el %.*s de
// dump_profile), así que incluirlo haría que las huellas no cuadrasen nunca.
inline uint32_t crc32_text8(uint32_t crc, const char* s) {
    uint8_t buf[8];
    size_t n = 0;
    while (n < 8 && s[n]) { buf[n] = (uint8_t)s[n]; n++; }
    while (n < 8) buf[n++] = 0;
    return crc32_feed(crc, buf, 8);
}

inline uint32_t crc32_u32le(uint32_t crc, uint32_t v) {
    uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    return crc32_feed(crc, b, 4);
}

// Huella de un perfil entero: nombre, número de páginas y, por cada página, sus
// etiquetas, teclas, mandos, rueda, máscara de iconos y los bitmaps de los
// huecos ocupados.
//
// Se escribe como plantilla para no arrastrar aquí la definición de Page ni las
// variables globales del firmware: vale cualquier tipo con los mismos nombres
// de campo, que es justo lo que necesita la prueba del PC para no repetir el
// orden de la serialización y arriesgarse a que se desvíe del de verdad.
//
//   mask_of(pg)        -> máscara de iconos de esa página
//   icon_of(pg, slot)  -> puntero a los CFGHASH_FB_SIZE bytes del bitmap
//   tick()             -> se llama al acabar cada página (el firmware refresca
//                         ahí el perro guardián; un perfil lleno son 28 KB)
template <typename PageT, typename MaskFn, typename IconFn, typename TickFn>
inline uint32_t crc32_profile_of(const char* name, uint8_t page_count,
                                 const PageT* pages, MaskFn mask_of,
                                 IconFn icon_of, TickFn tick) {
    uint32_t crc = CFGHASH_INIT;
    crc = crc32_text8(crc, name);
    crc = crc32_feed(crc, &page_count, 1);

    for (uint8_t pg = 0; pg < page_count; pg++) {
        const PageT& page = pages[pg];

        for (int s = 0; s < CFGHASH_LABELS; s++) crc = crc32_text8(crc, page.oled_labels[s]);
        for (int s = 0; s < CFGHASH_KEYS; s++) {
            uint8_t k[2] = { page.key_mappings[s].modifier, page.key_mappings[s].keycode };
            crc = crc32_feed(crc, k, 2);
        }
        for (int s = 0; s < CFGHASH_ROTARY; s++) {
            uint8_t r[3] = { page.rotary[s].type, page.rotary[s].modifier, page.rotary[s].keycode };
            crc = crc32_feed(crc, r, 3);
        }
        for (int l = 0; l < 2; l++) {
            uint8_t sc[2] = { page.scroll_detents[l], (uint8_t)(page.scroll_invert[l] ? 1 : 0) };
            crc = crc32_feed(crc, sc, 2);
        }

        // Solo los 20 bits que existen: los de arriba no los toca nadie, pero la
        // app monta su máscara desde cero y ahí siempre valen 0.
        const uint32_t mask = mask_of(pg) & ((1u << CFGHASH_SLOTS) - 1);
        crc = crc32_u32le(crc, mask);

        for (uint8_t s = 0; s < CFGHASH_SLOTS; s++) {
            if (!(mask & (1u << s))) continue;
            crc = crc32_feed(crc, icon_of(pg, s), CFGHASH_FB_SIZE);
        }

        tick();
    }

    // El icono del perfil es el hueco 20, pero solo existe en la página 0 y no
    // depende de cuántas páginas tenga el perfil: se resume aparte, después del
    // bucle de páginas, para que cambiarlo mueva la huella igual que cualquier
    // otro bitmap.
    const uint8_t has_icon = (mask_of(0) & (1u << CFGHASH_PROFILE_SLOT)) ? 1 : 0;
    crc = crc32_feed(crc, &has_icon, 1);
    if (has_icon) crc = crc32_feed(crc, icon_of(0, CFGHASH_PROFILE_SLOT), CFGHASH_FB_SIZE);

    return crc ^ 0xFFFFFFFFu;
}

// Huella global: número de perfiles, tiempo de reposo y las huellas de cada
// perfil encadenadas. Encadenar las que ya se han calculado evita recorrer los
// datos dos veces.
//
// NO entran el perfil ni la página activos: cambian cada vez que se toca el
// teclado y no son configuración, así que incluirlos haría que la huella se
// moviese sola y la app resincronizase sin motivo.
inline uint32_t crc32_snapshot_of(uint8_t profile_count, uint8_t timeout,
                                  const uint32_t* per_profile) {
    uint32_t crc = CFGHASH_INIT;
    uint8_t head[2] = { profile_count, timeout };
    crc = crc32_feed(crc, head, 2);
    for (uint8_t i = 0; i < profile_count; i++) crc = crc32_u32le(crc, per_profile[i]);
    return crc ^ 0xFFFFFFFFu;
}

#endif // CONFIG_HASH_H
