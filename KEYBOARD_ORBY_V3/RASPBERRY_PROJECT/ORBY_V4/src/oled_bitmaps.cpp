#include "oled_bitmaps.h"
#include <string.h>

namespace OledBitmaps {

    // Auxiliar para inicializar una tarjeta base con bordes redondeados y contorno
    // Ancho: 72 (columnas 0 a 71), Alto: 40 (páginas 0 a 4)
    static void init_card_base(uint8_t* bmp) {
        memset(bmp, 0, 360);
        
        // Bordes verticales (Columna 4 y Columna 67)
        for (int p = 0; p < 5; p++) {
            bmp[p * 72 + 4]  = 0xFF;
            bmp[p * 72 + 67] = 0xFF;
        }
        
        // Bordes horizontales (Página 0 arriba y Página 4 abajo)
        // Página 0: pixel superior (0x01)
        // Página 4: pixel inferior (0x80)
        for (int c = 5; c < 67; c++) {
            bmp[0 * 72 + c] |= 0x01;
            bmp[4 * 72 + c] |= 0x80;
        }

        // Esquinas redondeadas
        bmp[0 * 72 + 4] = 0xFC; // Quita los dos píxeles superiores
        bmp[0 * 72 + 5] = 0x03;
        bmp[0 * 72 + 66] = 0x03;
        bmp[0 * 72 + 67] = 0xFC;

        bmp[4 * 72 + 4] = 0x3F; // Quita los dos píxeles inferiores
        bmp[4 * 72 + 5] = 0xC0;
        bmp[4 * 72 + 66] = 0xC0;
        bmp[4 * 72 + 67] = 0x3F;
    }

    // ========================================================================
    // GENERACIÓN DE LOS ICONOS HILADOS EN EL CENTRO
    // ========================================================================

    // 1. COPY: Dos hojas superpuestas (desplazadas en diagonal)
    static const uint8_t* get_bmp_copy() {
        static uint8_t bmp[360];
        init_card_base(bmp);

        // Hoja trasera (arriba-izquierda): Cols 24-40, Alto: píxeles 8-24 (Págs 1 y 2)
        // Contorno de la hoja trasera
        for (int c = 24; c <= 40; c++) {
            bmp[1 * 72 + c] |= 0x01; // Borde superior hoja trasera
            bmp[2 * 72 + c] |= 0x01; // Borde inferior hoja trasera (parcial)
        }
        for (int p = 1; p <= 2; p++) {
            bmp[p * 72 + 24] |= 0xFF; // Borde izquierdo
            bmp[p * 72 + 40] |= 0xFF; // Borde derecho
        }

        // Hoja delantera (abajo-derecha): Cols 31-47, Alto: píxeles 14-30 (Págs 1, 2, 3)
        // Limpiamos el interior para que tape la trasera
        for (int p = 1; p <= 3; p++) {
            for (int c = 31; c <= 47; c++) {
                // Borrar fondo donde va la hoja de delante
                uint8_t mask = 0;
                if (p == 1) mask = 0x3F; // Conservar píxeles superiores de la hoja trasera
                if (p == 2) mask = 0x00;
                if (p == 3) mask = 0x00;
                
                // Conservar bordes de tarjeta
                if (c == 4 || c == 67) continue; 
                
                bmp[p * 72 + c] &= mask;
            }
        }

        // Volvemos a dibujar los contornos de la tarjeta que pudieron borrarse
        for (int c = 31; c <= 47; c++) {
            bmp[0 * 72 + c] |= 0x01;
            bmp[4 * 72 + c] |= 0x80;
        }

        // Dibujar contornos de la hoja delantera (Cols 31-47, Págs 1 a 3)
        // Línea superior (Pág 1, bit 6 = 0x40)
        for (int c = 31; c <= 47; c++) {
            bmp[1 * 72 + c] |= 0x40;
            bmp[3 * 72 + c] |= 0x40; // Línea inferior
        }
        for (int p = 1; p <= 3; p++) {
            uint8_t lmask = (p == 1) ? 0xC0 : 0xFF;
            bmp[p * 72 + 31] |= lmask; // Borde izquierdo
            bmp[p * 72 + 47] |= lmask; // Borde derecho
        }

        // Detalles del "texto" dentro de la hoja delantera (líneas horizontales en pág 2)
        for (int c = 35; c <= 43; c++) {
            bmp[2 * 72 + c] |= 0x18; // Dos líneas de texto simuladas
        }

        return bmp;
    }

    // 2. PASTE: Portapapeles con clip
    static const uint8_t* get_bmp_paste() {
        static uint8_t bmp[360];
        init_card_base(bmp);

        // Tabla del portapapeles: Cols 23-48, Págs 1-3
        for (int c = 23; c <= 48; c++) {
            bmp[1 * 72 + c] |= 0x04; // Parte superior tabla
            bmp[3 * 72 + c] |= 0x20; // Parte inferior tabla
        }
        for (int p = 1; p <= 3; p++) {
            bmp[p * 72 + 23] |= 0xFC; // Lateral izquierdo
            bmp[p * 72 + 48] |= 0xFC; // Lateral derecho
        }

        // Papel blanco (dentro): Cols 27-44, Págs 1-3
        // Borrar el fondo de madera de la tabla
        for (int p = 1; p <= 3; p++) {
            for (int c = 27; c <= 44; c++) {
                uint8_t mask = (p == 1) ? 0x0F : 0x00;
                bmp[p * 72 + c] &= mask;
            }
        }

        // Bordes del papel
        for (int c = 27; c <= 44; c++) {
            bmp[1 * 72 + c] |= 0x10; // Borde superior del papel
            bmp[3 * 72 + c] |= 0x08; // Borde inferior del papel
        }
        for (int p = 1; p <= 3; p++) {
            uint8_t pmask = (p == 1) ? 0xF0 : ((p == 3) ? 0x0F : 0xFF);
            bmp[p * 72 + 27] |= pmask;
            bmp[p * 72 + 44] |= pmask;
        }

        // Clip metálico arriba: Cols 31-40, Pág 0 y 1
        for (int c = 31; c <= 40; c++) {
            bmp[0 * 72 + c] |= 0xE0; // Clip arriba
            bmp[1 * 72 + c] |= 0x03; 
        }
        bmp[0 * 72 + 30] |= 0xC0;
        bmp[0 * 72 + 41] |= 0xC0;
        bmp[1 * 72 + 30] |= 0x03;
        bmp[1 * 72 + 41] |= 0x03;

        // Líneas de texto simuladas en el papel (Pág 2)
        for (int c = 31; c <= 40; c++) {
            bmp[2 * 72 + c] |= 0x24; // Texto simulado
        }

        return bmp;
    }

    // 3. UNDO: Flecha curvada hacia la izquierda
    static const uint8_t* get_bmp_undo() {
        static uint8_t bmp[360];
        init_card_base(bmp);

        // Cuerpo curvo (Arco de circunferencia en la mitad derecha):
        // Pág 1, 2, 3, Cols 30-52
        for (int c = 34; c <= 50; c++) {
            bmp[1 * 72 + c] |= 0x1E; // Arco superior
            bmp[3 * 72 + c] |= 0x78; // Arco inferior
        }
        for (int p = 1; p <= 3; p++) {
            bmp[p * 72 + 50] |= 0xFF; // Lateral derecho de la curva
        }

        // Borrar centro para hacer el canal del arco vacío
        for (int c = 38; c <= 46; c++) {
            bmp[2 * 72 + c] &= 0x03; 
        }

        // Cabeza de la flecha apuntando a la izquierda en Pág 1 y 2 (Cols 24 a 32)
        bmp[1 * 72 + 24] |= 0x80;
        bmp[1 * 72 + 25] |= 0xC0;
        bmp[1 * 72 + 26] |= 0xE0;
        bmp[1 * 72 + 27] |= 0xF0;
        bmp[1 * 72 + 28] |= 0xF8;
        bmp[1 * 72 + 29] |= 0xFC;
        bmp[1 * 72 + 30] |= 0xFE;
        bmp[1 * 72 + 31] |= 0xFF;
        
        bmp[2 * 72 + 24] |= 0x01;
        bmp[2 * 72 + 25] |= 0x03;
        bmp[2 * 72 + 26] |= 0x07;
        bmp[2 * 72 + 27] |= 0x0F;
        bmp[2 * 72 + 28] |= 0x1F;
        bmp[2 * 72 + 29] |= 0x3F;
        bmp[2 * 72 + 30] |= 0x7F;
        bmp[2 * 72 + 31] |= 0xFF;

        return bmp;
    }

    // 4. REDO: Flecha curvada hacia la derecha
    static const uint8_t* get_bmp_redo() {
        static uint8_t bmp[360];
        init_card_base(bmp);

        // Cuerpo curvo (Arco de circunferencia en la mitad izquierda):
        // Pág 1, 2, 3, Cols 20-42
        for (int c = 22; c <= 38; c++) {
            bmp[1 * 72 + c] |= 0x1E; // Arco superior
            bmp[3 * 72 + c] |= 0x78; // Arco inferior
        }
        for (int p = 1; p <= 3; p++) {
            bmp[p * 72 + 22] |= 0xFF; // Lateral izquierdo de la curva
        }

        // Cabeza de la flecha apuntando a la derecha en Pág 1 y 2 (Cols 40 a 48)
        bmp[1 * 72 + 48] |= 0x80;
        bmp[1 * 72 + 47] |= 0xC0;
        bmp[1 * 72 + 46] |= 0xE0;
        bmp[1 * 72 + 45] |= 0xF0;
        bmp[1 * 72 + 44] |= 0xF8;
        bmp[1 * 72 + 43] |= 0xFC;
        bmp[1 * 72 + 42] |= 0xFE;
        bmp[1 * 72 + 41] |= 0xFF;
        
        bmp[2 * 72 + 48] |= 0x01;
        bmp[2 * 72 + 47] |= 0x03;
        bmp[2 * 72 + 46] |= 0x07;
        bmp[2 * 72 + 45] |= 0x0F;
        bmp[2 * 72 + 44] |= 0x1F;
        bmp[2 * 72 + 43] |= 0x3F;
        bmp[2 * 72 + 42] |= 0x7F;
        bmp[2 * 72 + 41] |= 0xFF;

        return bmp;
    }

    // Helper para dibujar la estructura 3D de una tecla física (keycap)
    static void draw_keycap_frame(uint8_t* bmp) {
        // Marco exterior de la tecla: Cols 18 a 53, Págs 0 a 4
        // Borde superior
        for (int c = 18; c <= 53; c++) {
            bmp[0 * 72 + c] |= 0x0F; // Borde superior grueso
            bmp[4 * 72 + c] |= 0xF0; // Borde inferior grueso
        }
        for (int p = 0; p < 5; p++) {
            bmp[p * 72 + 18] |= 0xFF; // Lateral izquierdo
            bmp[p * 72 + 53] |= 0xFF; // Lateral derecho
        }

        // Bisel 3D (Borde interior de la tecla)
        for (int c = 22; c <= 49; c++) {
            bmp[0 * 72 + c] |= 0x30; 
            bmp[4 * 72 + c] |= 0x0C;
        }
        for (int p = 1; p <= 3; p++) {
            bmp[p * 72 + 22] |= 0x03;
            bmp[p * 72 + 49] |= 0xC0;
        }
    }

    // Renders de fuentes bold en el centro del keycap
    // A (Cols 32-39, Pág 2)
    static const uint8_t font_bold_a[8] = {0xE0, 0x1C, 0x13, 0x13, 0x1C, 0xE0, 0x00, 0x00};
    static const uint8_t font_bold_b[8] = {0xFF, 0x93, 0x93, 0x93, 0x6C, 0x00, 0x00, 0x00};
    static const uint8_t font_bold_c[8] = {0x3C, 0x42, 0x81, 0x81, 0x81, 0x42, 0x00, 0x00};
    static const uint8_t font_bold_d[8] = {0xFF, 0x81, 0x81, 0x81, 0x42, 0x3C, 0x00, 0x00};
    static const uint8_t font_bold_e[8] = {0xFF, 0x93, 0x93, 0x93, 0x81, 0x00, 0x00, 0x00};
    static const uint8_t font_bold_g[8] = {0x3C, 0x42, 0x81, 0x91, 0x91, 0xF2, 0x00, 0x00};

    static const uint8_t* get_bmp_key(const uint8_t* char_font) {
        static uint8_t bmp[360];
        memset(bmp, 0, 360);
        draw_keycap_frame(bmp);

        // Dibujar el caracter bold en la página 2 (centro)
        // Desplazamiento horizontal de columna para centrar el caracter de 8 píxeles:
        // Columna de inicio = 32
        for (int i = 0; i < 8; i++) {
            bmp[2 * 72 + 32 + i] = char_font[i];
        }

        return bmp;
    }

    // ========================================================================
    // CONSTRUCTOR DEL PUENTE DE CONEXIÓN CON LOS ARRAYS DE MEMORIA
    // ========================================================================
    
    // Función de inicialización general que copia las imágenes calculadas dinámicamente
    // a los arrays constantes expuestos.
    // Esto se realiza de manera transparente al primer pintado.
    static void fetch_bitmap(uint8_t* dest, const uint8_t* (*generator)()) {
        const uint8_t* src = generator();
        memcpy(dest, src, 360);
    }

    // Wrapper para que main.cpp lea los arrays directos
    class BitmapRegistry {
    public:
        uint8_t copy[360];
        uint8_t paste[360];
        uint8_t undo[360];
        uint8_t redo[360];
        uint8_t key_a[360];
        uint8_t key_b[360];
        uint8_t key_c[360];
        uint8_t key_d[360];
        uint8_t key_e[360];
        uint8_t key_g[360];

        BitmapRegistry() {
            fetch_bitmap(copy,  get_bmp_copy);
            fetch_bitmap(paste, get_bmp_paste);
            fetch_bitmap(undo,  get_bmp_undo);
            fetch_bitmap(redo,  get_bmp_redo);
            
            fetch_bitmap(key_a, []() { return get_bmp_key(font_bold_a); });
            fetch_bitmap(key_b, []() { return get_bmp_key(font_bold_b); });
            fetch_bitmap(key_c, []() { return get_bmp_key(font_bold_c); });
            fetch_bitmap(key_d, []() { return get_bmp_key(font_bold_d); });
            fetch_bitmap(key_e, []() { return get_bmp_key(font_bold_e); });
            fetch_bitmap(key_g, []() { return get_bmp_key(font_bold_g); });
        }
    };

    static const BitmapRegistry registry;

}

// Para evitar colisiones y optimizar la inicialización, exponemos los arrays directamente
// copiando el búfer estático del registro en tiempo de compilación/ejecución.
namespace OledBitmaps {
    // Definimos las variables reales apuntando a la memoria estática del registro
    const uint8_t* get_bitmap_by_index(int idx) {
        switch (idx) {
            case 0:  return registry.copy;
            case 1:  return registry.paste;
            case 2:  return registry.undo;
            case 3:  return registry.redo;
            case 4:  return registry.key_a;
            case 5:  return registry.key_b;
            case 6:  return registry.key_c;
            case 7:  return registry.key_d;
            case 8:  return registry.key_e;
            case 9:  return registry.key_g;
            default: return registry.copy;
        }
    }
}
