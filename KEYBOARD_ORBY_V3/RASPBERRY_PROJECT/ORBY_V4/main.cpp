#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "bsp/board.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include "pinout.h"
#include "hardware_encoder.h"
#include "hardware_oled.h"
#include "oled_text.h"
#include "oled_bitmaps.h"
#include "oled_intro.h"
#include "hid_hires.h"
#include "hid_out.h"
#include "wheel_out.h"
#include "hardware_as5600.h"

// Las combinaciones con modificador (Ctrl+C y compañía) se envían como
// pulsación breve en vez de mantenerse mientras la tecla esté hundida.
//
// El motivo: un macropad manda atajos, y mantener Ctrl+C dos segundos no copia
// dos veces, solo deja el Ctrl puesto en el host. Con el Ctrl puesto, la rueda
// deja de hacer scroll y pasa a hacer zoom; con Shift, pasa a scroll
// horizontal. Se pierde el autorrepetido de combinaciones como Ctrl+Z: si lo
// prefieres al revés, pon esto a 0.
//
// Las teclas sueltas (sin modificador) y los modificadores a pelo (Ctrl sin
// tecla, para usarlos con el ratón) SÍ se mantienen.
#define ORBY_COMBO_AS_TAP 1

// Acciones especiales de tecla. Van en el campo `modifier` del mapeo, que para
// un atajo normal nunca llega tan alto, así que no hace falta cambiar la
// estructura ni migrar nada.
#define KEYACT_CONSUMER   0xFE  // keycode = índice de la tabla multimedia
#define KEYACT_GOTO_PAGE  0xFD  // keycode = número de página (1..MAX_PAGES)
#define KEYACT_PAGE_STATE 0xFC  // enseña la página actual y abre el gestor
#define KEYACT_MACRO      0xFB  // keycode = id de la macro; la ejecuta el PC, no el teclado

// Las dos teclas de sistema: no llevan pantalla y no envían atajos. Son índices
// base 0 dentro de key_pins, así que la tecla N está en N-1.
//   - SUPER, mantenida, cambia de capa todo el perfil (teclas, mandos y rueda).
//   - MENÚ, mantenida un segundo, abre el menú de perfiles en las pantallas.
// Antes estaban al revés (SUPER en la 10 y el menú en la 12); cambiarlas de
// sitio es cambiar estas dos líneas.
#define KEY_IDX_SUPER 11   // Tecla 12
#define KEY_IDX_MENU   9   // Tecla 10

// Mapeos nativos de teclado para las 12 teclas
struct KeyAction {
    uint8_t modifier;
    uint8_t keycode;
};

// ==========================================
// ACCIONES DE LOS MANDOS GIRATORIOS
// ==========================================
// Cubren los dos encoders EC11 y la rueda magnética. Antes estaban cableadas
// en el bucle principal (volumen y brillo fijos), así que no había forma de
// cambiarlas por perfil.
enum RotaryType : uint8_t {
    ROT_NONE = 0,
    ROT_CONSUMER,   // keycode = índice de la tabla de teclas de consumo
    ROT_KEY,        // modifier + keycode HID
    ROT_SCROLL_V,   // rueda vertical (bidireccional por sí misma)
    ROT_SCROLL_H,   // paneo horizontal AC Pan
    ROT_ZOOM,       // Ctrl + rueda vertical
};

struct RotaryAction {
    uint8_t type;
    uint8_t modifier;
    uint8_t keycode;
};

// Índices de la tabla get_consumer_key_from_index(). Los del 1 al 6 se
// conservan tal cual para no invalidar los perfiles ya existentes; del 7 en
// adelante son los que necesitan los encoders configurables.
#define CONS_VOL_UP        7
#define CONS_VOL_DOWN      8
#define CONS_MUTE          9
#define CONS_BRIGHT_UP    10
#define CONS_BRIGHT_DOWN  11

// Huecos de acciones giratorias dentro de cada perfil. La lista se repite dos
// veces: 0-7 en capa normal y 8-15 con SUPER mantenida, igual que las teclas.
enum RotarySlot : uint8_t {
    ROT_ENC1_CW = 0, ROT_ENC1_CCW, ROT_ENC1_CLICK,
    ROT_ENC2_CW,     ROT_ENC2_CCW, ROT_ENC2_CLICK,
    ROT_WHEEL_CW,    ROT_WHEEL_CCW,
    ROT_LAYER_STRIDE
};
#define ROT_SLOT_COUNT (ROT_LAYER_STRIDE * 2)

// Índice del hueco giratorio según la capa activa.
static inline uint8_t rot_slot(uint8_t base, bool super) {
    return (uint8_t)(base + (super ? ROT_LAYER_STRIDE : 0));
}

// La sensibilidad de la rueda también es un ajuste del perfil, y con dos
// valores: uno por capa. Así un mismo perfil puede desplazar fino en normal y
// a saltos largos con SUPER.
#define SCROLL_DETENTS_MIN     3
#define SCROLL_DETENTS_MAX   240
#define SCROLL_DETENTS_DEFAULT 60

// ==========================================
// PÁGINAS
// ==========================================
// Una página es un juego completo de «lo que hace el teclado»: teclas,
// etiquetas, mandos, rueda e iconos. Un perfil tiene de una a MAX_PAGES y se
// alterna entre ellas con la pulsación corta del botón de menú, con una tecla de
// salto directo o desde el gestor de páginas.
//
// Cada página cuesta 260 bytes de RAM, así que multiplicarlas es barato. Lo que
// NO cabía eran los iconos (7424 bytes por página): por eso el banco de iconos
// se ha mudado a la Flash y se pinta directamente de ahí. Ver FLASH_OLED_OFFSET.
#define MAX_PAGES 4

struct Page {
    char oled_labels[20][8];    // 0-9 Normal, 10-19 SUPER
    KeyAction key_mappings[24]; // 0-11 Normal, 12-23 SUPER
    RotaryAction rotary[ROT_SLOT_COUNT];
    uint8_t scroll_detents[2];  // [0] normal, [1] SUPER
    uint8_t scroll_invert[2];
};

struct Profile {
    char    name[8];
    uint8_t page_count;         // 1..MAX_PAGES
    // Tramo del banco de iconos de la Flash que le pertenece. Es fijo mientras
    // el perfil viva: así borrar un perfil solo reordena metadatos, en vez de
    // obligar a arrastrar medio megabyte de iconos por la Flash.
    uint8_t oled_bank;
    uint8_t padding[2];
    Page    pages[MAX_PAGES];
};

// Cuántos perfiles caben. Los cuatro primeros son los de fábrica; el resto se
// crean desde la app con ADD_PROFILE o DUP_PROFILE.
#define MAX_PROFILES      16
#define DEFAULT_PROFILES  4

// Los perfiles de fábrica se declaran sin páginas —solo nombre, etiquetas y
// teclas— y se copian a la página 0 al arrancar. Así los literales de abajo, que
// son 150 líneas de llaves anidadas, no hay que tocarlos al añadir páginas.
struct DefaultProfile {
    char name[8];
    char oled_labels[20][8];
    KeyAction key_mappings[24];
};

// Perfiles de fábrica. Se copian a `profiles` (RAM) al arrancar; la app de
// escritorio puede editar la copia en RAM y persistirla con SAVE_STATE.
const DefaultProfile default_profiles[DEFAULT_PROFILES] = {
    // OFIM (Ofimática y Productividad General)
    {
        "OFIM",
        {
            // Normal Labels (0-9)
            "COPY", "PAST", "CUT", "UNDO", "REDO", "ALL", "SAVE", "FIND", "CLSE", "SNAP",
            // SUPER Labels (10-19)
            "EXCL", "WORD", "BROW", "CALC", "TASK", "LOCK", "PLAY", "STOP", "NEXT", "PREV"
        },
        {
            // Normal Key Mappings (0-11)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_C }, // Tecla 1 -> Ctrl+C
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_V }, // Tecla 2 -> Ctrl+V
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_X }, // Tecla 3 -> Ctrl+X
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Z }, // Tecla 4 -> Ctrl+Z
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Y }, // Tecla 5 -> Ctrl+Y
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_A }, // Tecla 6 -> Ctrl+A
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_S }, // Tecla 7 -> Ctrl+S
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_F }, // Tecla 8 -> Ctrl+F
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_W }, // Tecla 9 -> Ctrl+W
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { KEYBOARD_MODIFIER_LEFTGUI | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_S }, // Tecla 11 -> Win+Shift+S (OLED 10)
            { 0,                          0         }, // Tecla 12 -> NADA (SUPER)
            
            // SUPER Key Mappings (12-23)
            { KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_E }, // Tecla 1 -> Excel (Ctrl+Alt+Shift+E)
            { KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_W }, // Tecla 2 -> Word (Ctrl+Alt+Shift+W)
            { 0xFE,                       1         }, // Tecla 3 -> Browser (Consumer idx 1)
            { 0xFE,                       2         }, // Tecla 4 -> Calculator (Consumer idx 2)
            { KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_ESCAPE }, // Tecla 5 -> Task Manager (Ctrl+Shift+Esc)
            { KEYBOARD_MODIFIER_LEFTGUI,  HID_KEY_L }, // Tecla 6 -> Lock PC (Win+L)
            { 0xFE,                       3         }, // Tecla 7 -> Play/Pause (Consumer idx 3)
            { 0xFE,                       4         }, // Tecla 8 -> Stop (Consumer idx 4)
            { 0xFE,                       5         }, // Tecla 9 -> Next (Consumer idx 5)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { 0xFE,                       6         }, // Tecla 11 -> Prev (Consumer idx 6)
            { 0,                          0         }  // Tecla 12 -> NADA (SUPER)
        }
    },
    // PHTS (Adobe Photoshop)
    {
        "PHTS",
        {
            // Normal Labels (0-9)
            "BRSH", "ERAS", "PENC", "UNDO", "REDO", "LASS", "CROP", "MOVE", "NEWL", "DUPL",
            // SUPER Labels (10-19)
            "ZOOM", "HAND", "TEXT", "GRAD", "PEN", "SHAP", "EYE", "SELC", "SLIC", "PATH"
        },
        {
            // Normal Key Mappings (0-11)
            { 0,                          HID_KEY_B }, // Brush (B)
            { 0,                          HID_KEY_E }, // Eraser (E)
            { 0,                          HID_KEY_N }, // Pencil (N)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Z }, // Undo (Ctrl+Z)
            { KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_Z }, // Redo (Ctrl+Shift+Z)
            { 0,                          HID_KEY_L }, // Lasso (L)
            { 0,                          HID_KEY_C }, // Crop (C)
            { 0,                          HID_KEY_V }, // Move (V)
            { KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_N }, // New Layer (Ctrl+Shift+N)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_J }, // Duplicate Layer (Ctrl+J)
            { 0,                          0         }, // Tecla 12 -> NADA (SUPER)
            
            // SUPER Key Mappings (12-23)
            { 0,                          HID_KEY_Z }, // Zoom (Z)
            { 0,                          HID_KEY_H }, // Hand (H)
            { 0,                          HID_KEY_T }, // Text (T)
            { 0,                          HID_KEY_G }, // Gradient (G)
            { 0,                          HID_KEY_P }, // Pen (P)
            { 0,                          HID_KEY_U }, // Shape (U)
            { 0,                          HID_KEY_I }, // Eyedropper (I)
            { 0,                          HID_KEY_M }, // Marquee Selection (M)
            { 0,                          HID_KEY_K }, // Slice (K)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { 0,                          HID_KEY_A }, // Path Selection (A)
            { 0,                          0         }  // Tecla 12 -> NADA (SUPER)
        }
    },
    // ALTM (Altium Designer)
    {
        "ALTM",
        {
            // Normal Labels (0-9)
            "WIRE", "PORT", "PAD", "UNDO", "REDO", "VIA", "MOVE", "TRAC", "FIT", "3D",
            // SUPER Labels (10-19)
            "SCH", "PCB", "LIB", "COMP", "GRID", "MEAS", "RULE", "ERC", "DRC", "GEN"
        },
        {
            // Normal Key Mappings (0-11)
            { 0,                          HID_KEY_W }, // Place Wire (W)
            { 0,                          HID_KEY_R }, // Place Port (R)
            { 0,                          HID_KEY_P }, // Place Pad (P)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Z }, // Undo (Ctrl+Z)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Y }, // Redo (Ctrl+Y)
            { 0,                          HID_KEY_V }, // Place Via (V)
            { 0,                          HID_KEY_M }, // Move (M)
            { 0,                          HID_KEY_T }, // Track (T)
            { 0,                          HID_KEY_F }, // View Fit (F)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { 0,                          HID_KEY_3 }, // 3D View (3)
            { 0,                          0         }, // Tecla 12 -> NADA (SUPER)
            
            // SUPER Key Mappings (12-23)
            { 0,                          HID_KEY_D }, // SCH Doc options (D)
            { KEYBOARD_MODIFIER_LEFTALT,  HID_KEY_D }, // PCB Options (Alt+D)
            { 0,                          HID_KEY_L }, // Library options (L)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Z }, // Undo (Ctrl+Z)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Y }, // Redo (Ctrl+Y)
            { 0,                          HID_KEY_G }, // Grid Toggle (G)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_M }, // Measure (Ctrl+M)
            { KEYBOARD_MODIFIER_LEFTALT,  HID_KEY_R }, // Design Rules (Alt+R)
            { KEYBOARD_MODIFIER_LEFTALT,  HID_KEY_D }, // Run DRC (Alt+D)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { KEYBOARD_MODIFIER_LEFTALT,  HID_KEY_O }, // Options (Alt+O)
            { 0,                          0         }  // Tecla 12 -> NADA (SUPER)
        }
    },
    // PREM (Adobe Premiere Pro)
    {
        "PREM",
        {
            // Normal Labels (0-9)
            "CUT", "SELC", "PLAY", "UNDO", "REDO", "MARK", "LINK", "RIPL", "EXPO", "NEST",
            // SUPER Labels (10-19)
            "SLIP", "SLID", "PEN", "HAND", "TEXT", "ZOOM", "TRIM", "RATE", "ROLL", "SPED"
        },
        {
            // Normal Key Mappings (0-11)
            { 0,                          HID_KEY_C }, // Razor Tool (C)
            { 0,                          HID_KEY_V }, // Selection Tool (V)
            { 0,                          HID_KEY_SPACE }, // Play/Pause (Space)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Z }, // Undo (Ctrl+Z)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_Y }, // Redo (Ctrl+Y)
            { 0,                          HID_KEY_M }, // Marker (M)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_L }, // Link (Ctrl+L)
            { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_DELETE }, // Ripple Delete (Shift+Delete)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_M }, // Export (Ctrl+M)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { 0,                          HID_KEY_N }, // Nest (N)
            { 0,                          0         }, // Tecla 12 -> NADA (SUPER)
            
            // SUPER Key Mappings (12-23)
            { 0,                          HID_KEY_Y }, // Slip Tool (Y)
            { 0,                          HID_KEY_U }, // Slide Tool (U)
            { 0,                          HID_KEY_P }, // Pen Tool (P)
            { 0,                          HID_KEY_H }, // Hand Tool (H)
            { 0,                          HID_KEY_T }, // Type Tool (T)
            { 0,                          HID_KEY_Z }, // Zoom Tool (Z)
            { KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_T }, // Trim (Shift+T)
            { 0,                          HID_KEY_R }, // Rate Stretch (R)
            { KEYBOARD_MODIFIER_LEFTALT,  HID_KEY_R }, // Roll Tool (Alt+R)
            { 0,                          0         }, // Tecla 10 -> NADA (menu)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_R }, // Speed (Ctrl+R)
            { 0,                          0         }  // Tecla 12 -> NADA (SUPER)
        }
    }
};

// Copia viva y editable de los perfiles. `profile_count` es cuántos existen de
// verdad; el resto del array son huecos libres.
Profile profiles[MAX_PROFILES];
volatile uint8_t profile_count = DEFAULT_PROFILES;

// ==========================================
// BITMAPS OLED PERSONALIZADOS
// ==========================================
// 20 huecos por página: 10 pantallas × 2 capas (Normal / SUPER). Con páginas,
// la unidad es (perfil, página): 16 × 4 = 64 tramos de 8 KB, medio megabyte.
//
// Eso NO cabe en RAM —serían 464 KB de los 264 KB del chip—, así que el banco
// vive solo en la Flash y se pinta directamente de ahí: el RP2040 la mapea en el
// espacio de direcciones (XIP), así que al SPI se le pasa un puntero a Flash y no
// hace falta ni copiar ni descomprimir nada.
//
// En RAM queda únicamente un búfer de preparación con el tramo que se está
// editando, para que un icono recién subido se vea antes de grabarlo.
#define OLED_FB_SIZE         360
#define OLED_SLOTS           20
#define OLED_PAGE_STRIDE     7424   // 20*360 = 7200, redondeado a página de 256

// Un bit por hueco ocupado, por perfil y página. Vive en los ajustes: perderla
// equivale a perder los iconos.
static uint32_t custom_oled_mask[MAX_PROFILES][MAX_PAGES];

// Búfer de preparación: el tramo (perfil, página) que se está editando ahora.
// staging_dirty dice si tiene cambios sin grabar.
static uint8_t oled_staging[OLED_PAGE_STRIDE] __attribute__((aligned(4)));
static uint8_t staging_profile = 0xFF;   // 0xFF = vacío
static uint8_t staging_page    = 0xFF;
static bool    staging_dirty   = false;

// ==========================================
// ESTADO COMPARTIDO (SHARED STATE)
// ==========================================
enum AppMode {
    MODE_NORMAL,
    MODE_MENU_MAIN,
    MODE_MENU_PERF,
    MODE_MENU_REPO,
    MODE_MENU_INFO,
    MODE_MENU_PAGES   // gestor de páginas: un número por pantalla
};

// Qué tecla (índice base 0) hay debajo de cada pantalla. Las pantallas 1 a 9 van
// con las teclas 1 a 9, y la pantalla 10 con la tecla 11.
static inline uint8_t key_index_for_screen(uint8_t screen_num) {
    return (screen_num <= 9) ? (uint8_t)(screen_num - 1) : (uint8_t)10;
}

volatile AppMode current_mode = MODE_NORMAL;
volatile uint8_t active_profile_idx = 0;   // 0=OFIM, 1=PHTS, 2=ALTM, 3=PREM
volatile uint8_t active_page = 0;          // Página en uso del perfil activo
// Ya no es ajustable: las OLED no responden al registro de contraste, así que
// el menú BRIL se quitó. Se deja el valor fijo por compatibilidad con lo ya
// grabado en la Flash de teclados en uso.
volatile uint8_t current_brightness = 207;   // 0xCF = 207 por defecto
volatile uint8_t reposo_timeout_min = 5;    // 5 minutos por defecto (0=OFF)
volatile uint8_t selected_menu_idx = 0;     // Opción de menú activa (0 a 4)
volatile bool super_active = false;         // Estado de la tecla modificadora SUPER
volatile bool system_refresh_req = false;   // Flag para actualizar pantallas
volatile uint16_t active_inversions = 0;    // Bitmask para inversiones en tiempo real

// --- Encendido de las pantallas (reposo) ---
// Core 0 detecta la inactividad, pero quien manda al SSD1306 es Core 1: las
// pantallas y el bus SPI tienen un único dueño. Construir un segundo
// HardwareOled desde Core 0 dejaba OLED_RST a nivel bajo (gpio_init() pone la
// salida a cero), es decir, las diez pantallas en reset permanente.
volatile bool displays_on = true;
volatile bool display_power_req = false;

// La intro de arranque la reproduce Core 1; Core 0 solo avisa de que hay una
// tecla pulsada para poder cortarla.
volatile bool intro_running = false;
volatile bool intro_skip_req = false;
static bool intro_should_skip() { return intro_skip_req; }

// Latido de Core 1. Core 0 solo alimenta el watchdog mientras esto siga
// cambiando: si Core 1 se queda colgado con el bus SPI a medias, el teclado se
// reinicia en vez de quedarse mudo hasta que alguien lo desenchufe.
volatile uint32_t core1_heartbeat = 0;

// ==========================================
// CONFIGURACIÓN DE LA RUEDA DE SCROLL
// ==========================================
// Cuántos "detents" (clics de rueda de ratón clásica) equivalen a una vuelta
// completa de la rueda magnética. 60 = valor por defecto. Desde la versión 3.0
// ya no es un ajuste global: vive en el perfil, con un valor por capa, así que
// se consulta siempre a través de estos accesos.
// La página activa del perfil activo. Todo lo que el teclado hace «ahora mismo»
// sale de aquí.
static inline Page& cur_page() {
    return profiles[active_profile_idx].pages[active_page];
}

static inline uint8_t cur_scroll_detents() {
    uint8_t v = cur_page().scroll_detents[super_active ? 1 : 0];
    return (v < SCROLL_DETENTS_MIN || v > SCROLL_DETENTS_MAX) ? SCROLL_DETENTS_DEFAULT : v;
}

static inline bool cur_scroll_invert() {
    return cur_page().scroll_invert[super_active ? 1 : 0] != 0;
}

// ==========================================
// DISPOSICIÓN DE LA FLASH
// ==========================================
//   0 –  115 KB   el programa
//        ...libre...
//   1024 – 1536 KB  banco de iconos: 64 tramos (16 perfiles × 4 páginas) de 8 KB
//   1536 – 1560 KB  ajustes: perfiles con sus páginas y las máscaras de iconos
//        ...libre...
//
// El banco acaba justo donde empiezan los ajustes. Está debajo y no encima
// porque encima no cabía: 1536 + 512 se sale de los 2 MB del módulo.
//
// Con las páginas, la unidad del banco pasa de «un perfil» a «una página de un
// perfil», y el tramo que le toca a cada perfil se guarda en `Profile.oled_bank`
// en vez de ser su índice. Así borrar un perfil solo reordena metadatos: mover
// medio megabyte de iconos por la Flash llevaría segundos y gastaría ciclos de
// borrado a cambio de nada.
#define FLASH_OLED_OFFSET    (1024 * 1024)
#define FLASH_OLED_SLICE     (2 * FLASH_SECTOR_SIZE)  // 8 KB por (perfil, página)
#define FLASH_OLED_AT(bank, pg) \
    (FLASH_OLED_OFFSET + (((bank) * MAX_PAGES) + (pg)) * FLASH_OLED_SLICE)

#define FLASH_TARGET_OFFSET  (1536 * 1024)
#define FLASH_SETTINGS_SECTORS 6
#define FLASH_SETTINGS_BYTES (FLASH_SETTINGS_SECTORS * FLASH_SECTOR_SIZE)

// El banco no puede solaparse con los ajustes ni salirse de la Flash.
static_assert(FLASH_OLED_AT(MAX_PROFILES - 1, MAX_PAGES - 1) + FLASH_OLED_SLICE
                  <= FLASH_TARGET_OFFSET,
              "El banco de iconos se come la región de ajustes");
static_assert(FLASH_TARGET_OFFSET + FLASH_SETTINGS_BYTES <= PICO_FLASH_SIZE_BYTES,
              "Los ajustes se salen de la Flash del módulo");

// Disposiciones anteriores, solo para rescatar los iconos al actualizar. Hasta
// la versión 3.1 el banco iba detrás de los ajustes, con un tramo por perfil.
#define FLASH_OLED_OFFSET_V5 (FLASH_TARGET_OFFSET + 2 * FLASH_SECTOR_SIZE)
#define FLASH_OLED_AT_V5(p)  (FLASH_OLED_OFFSET_V5 + (p) * FLASH_OLED_SLICE)
#define FLASH_OLED_OFFSET_V3 (FLASH_TARGET_OFFSET + FLASH_SECTOR_SIZE)
#define FLASH_OLED_AT_V3(p)  (FLASH_OLED_OFFSET_V3 + (p) * FLASH_OLED_SLICE)

// Versiones del bloque de ajustes. Cada cambio de estructura necesita un magic
// nuevo, pero NO se descarta lo guardado: se migra campo a campo. Los bitmaps
// OLED viven en su propia región y sobreviven a todo esto, pero la máscara de
// huecos ocupados está aquí, así que perderla equivale a perder los iconos.
#define SETTINGS_MAGIC_V1 0xDEB001CE  // original: solo ajustes básicos
#define SETTINGS_MAGIC_V2 0xDEB00204  // + perfiles editables, scroll y bitmaps
#define SETTINGS_MAGIC_V3 0xDEB00205  // + acciones de encoders y rueda
#define SETTINGS_MAGIC_V4 0xDEB00300  // + nº de perfiles variable (tope 8), capa
                                      //   SUPER en los mandos y scroll por perfil
#define SETTINGS_MAGIC_V5 0xDEB00310  // + tope de 16 perfiles: los ajustes pasan
                                      //   a dos sectores y la región de bitmaps
                                      //   se desplaza uno
#define SETTINGS_MAGIC    0xDEB00400  // + páginas por perfil: el banco de iconos
                                      //   se muda a 1024 KB con un tramo por
                                      //   (perfil, página) y sale de la RAM

// Diseños antiguos, conservados solo para poder leer lo ya grabado.
struct ProfileV2 {
    char name[8];
    char oled_labels[20][8];
    KeyAction key_mappings[24];
};

struct ProfileV3 {
    char name[8];
    char oled_labels[20][8];
    KeyAction key_mappings[24];
    RotaryAction rotary[ROT_LAYER_STRIDE];  // solo capa normal
};

struct SettingsV1 {
    uint32_t magic;
    uint8_t  active_profile_idx;
    uint8_t  current_brightness;
    uint8_t  reposo_timeout_min;
    uint8_t  padding;
};

struct SettingsV2 {
    uint32_t  magic;
    uint8_t   active_profile_idx;
    uint8_t   current_brightness;
    uint8_t   reposo_timeout_min;
    uint8_t   scroll_detents_per_rev;
    uint8_t   scroll_invert;
    uint8_t   padding[3];
    uint32_t  custom_oled_mask[4];
    ProfileV2 profiles[4];
};

struct SettingsV3 {
    uint32_t  magic;
    uint8_t   active_profile_idx;
    uint8_t   current_brightness;
    uint8_t   reposo_timeout_min;
    uint8_t   scroll_detents_per_rev;
    uint8_t   scroll_invert;
    uint8_t   padding[3];
    uint32_t  custom_oled_mask[4];
    ProfileV3 profiles[4];
};

// Perfil de las versiones 3.0 y 3.1: el de antes de las páginas. Se conserva
// para poder leer lo que ya está grabado en la Flash.
struct ProfileV5 {
    char name[8];
    char oled_labels[20][8];
    KeyAction key_mappings[24];
    RotaryAction rotary[ROT_SLOT_COUNT];
    uint8_t scroll_detents[2];
    uint8_t scroll_invert[2];
};

// Formato 3.0: mismo perfil, pero con tope de 8.
#define SETTINGS_V4_PROFILES 8
struct SettingsV4 {
    uint32_t  magic;
    uint8_t   active_profile_idx;
    uint8_t   current_brightness;
    uint8_t   reposo_timeout_min;
    uint8_t   profile_count;
    uint8_t   padding[4];
    uint32_t  custom_oled_mask[SETTINGS_V4_PROFILES];
    ProfileV5 profiles[SETTINGS_V4_PROFILES];
};

// Formato 3.1: tope de 16 perfiles, todavía sin páginas.
struct SettingsV5 {
    uint32_t  magic;
    uint8_t   active_profile_idx;
    uint8_t   current_brightness;
    uint8_t   reposo_timeout_min;
    uint8_t   profile_count;
    uint8_t   padding[4];
    uint32_t  custom_oled_mask[MAX_PROFILES];
    ProfileV5 profiles[MAX_PROFILES];
};

struct Settings {
    uint32_t magic;
    uint8_t  active_profile_idx;
    uint8_t  current_brightness;
    uint8_t  reposo_timeout_min;
    uint8_t  profile_count;
    uint8_t  active_page;
    uint8_t  padding[3];
    uint32_t custom_oled_mask[MAX_PROFILES][MAX_PAGES];
    Profile  profiles[MAX_PROFILES];
};

// El blob de ajustes se programa redondeado a página de 256 bytes.
#define SETTINGS_BLOB_SIZE (((sizeof(Settings) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

static uint8_t settings_blob[SETTINGS_BLOB_SIZE];

// Guardas de disposición: si los perfiles crecen y dejan de caber, hay que
// ampliar la región reservada en lugar de pisar el banco de iconos.
static_assert(SETTINGS_BLOB_SIZE <= FLASH_SETTINGS_BYTES,
              "Los ajustes ya no caben en la región reservada de Flash");
static_assert(OLED_SLOTS * OLED_FB_SIZE <= OLED_PAGE_STRIDE,
              "El stride por página no cubre sus 20 bitmaps");
static_assert(OLED_PAGE_STRIDE % FLASH_PAGE_SIZE == 0,
              "flash_range_program exige múltiplos de página");

// ==========================================
// SECUENCIAS (MACROS) QUE EJECUTA EL PROPIO TECLADO
// ==========================================
// Antes toda secuencia la ejecutaba el PC al recibir "MACRO:<id>" por CDC (ver
// trigger_macro más abajo), así que dejaban de funcionar en cuanto la app no
// estaba abierta. Los pasos que no dependen de saber nada del host —esperar,
// pulsar una tecla, mover el ratón un delta relativo, hacer clic— se guardan y
// reproducen aquí. Solo la posición ABSOLUTA del cursor (exige conocer la
// resolución/escala de la pantalla del PC, que el teclado no puede saber) sigue
// yendo por CDC: una secuencia que la use se queda en el camino viejo (ver
// trigger_macro: sin copia jugable en el teclado, avisa por CDC como antes).
#define MACRO_MAX_COUNT  64
#define MACRO_MAX_STEPS  24

enum MacroStepType : uint8_t {
    MSTEP_NONE      = 0,
    MSTEP_DELAY     = 1,  // a = milisegundos de espera
    MSTEP_KEY       = 2,  // a = modificador HID, b = keycode HID (pulsación breve)
    MSTEP_MOVE      = 3,  // a = dx, b = dy: desplazamiento relativo del ratón (-127..127)
    MSTEP_CLICK     = 4,  // a = botón: 0 izquierdo, 1 central, 2 derecho
    // a = dx, b = dy: desplazamiento desde la esquina inferior derecha hasta el
    // punto que se quería (normalmente números negativos o cero: la app calcula
    // x - ancho_pantalla, y - alto_pantalla al capturar la posición). El ratón
    // HID solo sabe mover en relativo, así que primero se manda un
    // desplazamiento positivo grande (más que cualquier escritorio real) para
    // que el sistema operativo lo recorte solo contra esa esquina —eso SIEMPRE
    // pasa, haga lo que haga la aceleración del puntero, porque el recorte es
    // lo último que se aplica— y desde esa esquina conocida se viaja en
    // relativo hasta el punto final.
    //
    // TODO(homing-absoluto): el reproductor manda exactamente lo que se le
    // pide —comprobado byte a byte con MACRO_TEST— pero el resultado en
    // pantalla depende del multiplicador de "velocidad del puntero" de
    // Windows (no de la aceleración/"mejorar precisión", eso no influye), que
    // no se puede leer ni anular desde aquí. Sin una calibración (mandar un
    // desplazamiento conocido, medir cuánto se movió de verdad y guardar el
    // factor en la app) la posición absoluta no es fiable, así que la app NO
    // sube pasos de este tipo por ahora (aparcado a petición del usuario). El
    // reproductor y el protocolo (SET_MACRO_STEP tipo 5, MACRO_TEST para
    // depurar) se dejan tal cual para retomarlo con la calibración.
    MSTEP_HOME_MOVE = 5,
};

struct MacroStep {
    uint8_t type;
    int16_t a;
    int16_t b;
};

struct FlashMacro {
    uint8_t   step_count;
    MacroStep steps[MACRO_MAX_STEPS];
};

// Copia viva y editable, igual que `profiles`: lo que se reproduce sale de
// aquí, y la Flash es solo persistencia entre arranques.
static FlashMacro macros[MACRO_MAX_COUNT];

#define FLASH_MACROS_OFFSET  (FLASH_TARGET_OFFSET + FLASH_SETTINGS_BYTES)
#define FLASH_MACROS_SECTORS 4
#define FLASH_MACROS_BYTES   (FLASH_MACROS_SECTORS * FLASH_SECTOR_SIZE)
#define MACROS_MAGIC 0xDEB0AC10

struct MacrosBlob {
    uint32_t   magic;
    FlashMacro items[MACRO_MAX_COUNT];
};
#define MACROS_BLOB_SIZE (((sizeof(MacrosBlob) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

static uint8_t macros_blob[MACROS_BLOB_SIZE];

static_assert(FLASH_MACROS_OFFSET + FLASH_MACROS_BYTES <= PICO_FLASH_SIZE_BYTES,
              "Las secuencias se salen de la Flash del módulo");
static_assert(MACROS_BLOB_SIZE <= FLASH_MACROS_BYTES,
              "Las secuencias ya no caben en su región reservada de Flash");

static void save_macros() {
    MacrosBlob* b = (MacrosBlob*)macros_blob;
    memset(macros_blob, 0, sizeof(macros_blob));
    b->magic = MACROS_MAGIC;
    memcpy(b->items, macros, sizeof(macros));

    tud_task();
    watchdog_update();
    const bool lock = multicore_lockout_victim_is_initialized(1);
    uint32_t ints = save_and_disable_interrupts();
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(FLASH_MACROS_OFFSET, FLASH_MACROS_BYTES);
    flash_range_program(FLASH_MACROS_OFFSET, macros_blob, MACROS_BLOB_SIZE);
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(ints);
}

static void load_macros() {
    const uint32_t magic = *(const uint32_t*)(XIP_BASE + FLASH_MACROS_OFFSET);
    if (magic == MACROS_MAGIC) {
        const MacrosBlob* b = (const MacrosBlob*)(XIP_BASE + FLASH_MACROS_OFFSET);
        memcpy(macros, b->items, sizeof(macros));
    } else {
        memset(macros, 0, sizeof(macros));
    }
}

// ==========================================
// ACCESO A LOS ICONOS
// ==========================================
// Un icono se lee de la Flash mapeada, salvo que su página sea la que se está
// editando: entonces sale del búfer de preparación, para que un icono recién
// subido se vea en la pantalla antes de grabarlo.
static inline bool staging_holds(uint8_t profile, uint8_t page) {
    return staging_profile == profile && staging_page == page;
}

static inline uint32_t oled_bank_of(uint8_t profile) {
    return profiles[profile].oled_bank;
}

static const uint8_t* oled_slot_ptr(uint8_t profile, uint8_t page, uint8_t slot) {
    if (staging_holds(profile, page)) return &oled_staging[slot * OLED_FB_SIZE];
    return (const uint8_t*)(XIP_BASE + FLASH_OLED_AT(oled_bank_of(profile), page)
                            + slot * OLED_FB_SIZE);
}

static inline bool oled_slot_used(uint8_t profile, uint8_t page, uint8_t slot) {
    return (custom_oled_mask[profile][page] & (1u << slot)) != 0;
}

// Graba el búfer de preparación en su tramo de Flash. Se llama al terminar de
// editar una página y antes de empezar con otra.
static void flush_oled_staging();

// Deja en el búfer la página indicada, leyéndola de la Flash, después de grabar
// la que hubiera antes.
static void load_oled_staging(uint8_t profile, uint8_t page) {
    if (staging_holds(profile, page)) return;
    flush_oled_staging();
    memcpy(oled_staging,
           (const uint8_t*)(XIP_BASE + FLASH_OLED_AT(oled_bank_of(profile), page)),
           OLED_PAGE_STRIDE);
    staging_profile = profile;
    staging_page    = page;
    staging_dirty   = false;
}

// Los perfiles de fábrica solo declaran teclas y etiquetas; las acciones
// giratorias se rellenan aquí para no repetirlas en los cuatro literales.
static void apply_default_rotaries(Page& p) {
    p.rotary[ROT_ENC1_CW]    = { ROT_CONSUMER, 0, CONS_VOL_UP };
    p.rotary[ROT_ENC1_CCW]   = { ROT_CONSUMER, 0, CONS_VOL_DOWN };
    p.rotary[ROT_ENC1_CLICK] = { ROT_CONSUMER, 0, CONS_MUTE };
    p.rotary[ROT_ENC2_CW]    = { ROT_CONSUMER, 0, CONS_BRIGHT_UP };
    p.rotary[ROT_ENC2_CCW]   = { ROT_CONSUMER, 0, CONS_BRIGHT_DOWN };
    p.rotary[ROT_ENC2_CLICK] = { ROT_NONE, 0, 0 };
    p.rotary[ROT_WHEEL_CW]   = { ROT_SCROLL_V, 0, 0 };
    p.rotary[ROT_WHEEL_CCW]  = { ROT_SCROLL_V, 0, 0 };
    // La capa SUPER arranca igual que la normal: quien no la toque no nota el
    // cambio, y quien la edite obtiene dos juegos de mandos en un solo perfil.
    for (uint8_t s = 0; s < ROT_LAYER_STRIDE; s++) {
        p.rotary[ROT_LAYER_STRIDE + s] = p.rotary[s];
    }
}

static void apply_default_scroll(Page& p) {
    p.scroll_detents[0] = SCROLL_DETENTS_DEFAULT;
    p.scroll_detents[1] = SCROLL_DETENTS_DEFAULT;
    p.scroll_invert[0]  = 0;
    p.scroll_invert[1]  = 0;
}

// Página en blanco: sin atajos y con etiquetas numeradas, para que las pantallas
// no salgan vacías.
static void make_blank_page(Page& pg) {
    memset(&pg, 0, sizeof(pg));
    for (int s = 0; s < 20; s++) {
        snprintf(pg.oled_labels[s], sizeof(pg.oled_labels[s]), "T%d", (s % 10) + 1);
    }
    apply_default_rotaries(pg);
    apply_default_scroll(pg);
}

// Perfil recién creado desde la app: una sola página, en blanco.
static void make_blank_profile(Profile& p, uint8_t number, uint8_t bank) {
    memset(&p, 0, sizeof(p));
    snprintf(p.name, sizeof(p.name), "PERF%u", (unsigned)number);
    p.page_count = 1;
    p.oled_bank  = bank;
    make_blank_page(p.pages[0]);
}

void load_defaults() {
    memset(profiles, 0, sizeof(profiles));
    profile_count = DEFAULT_PROFILES;
    for (int i = 0; i < DEFAULT_PROFILES; i++) {
        Profile& p = profiles[i];
        memcpy(p.name, default_profiles[i].name, sizeof(p.name));
        p.page_count = 1;
        p.oled_bank  = (uint8_t)i;
        // Los literales de fábrica solo traen etiquetas y teclas, y van a la
        // página 0; el resto de páginas no existe todavía.
        memcpy(p.pages[0].oled_labels, default_profiles[i].oled_labels,
               sizeof(p.pages[0].oled_labels));
        memcpy(p.pages[0].key_mappings, default_profiles[i].key_mappings,
               sizeof(p.pages[0].key_mappings));
        apply_default_rotaries(p.pages[0]);
        apply_default_scroll(p.pages[0]);
    }
    // Los perfiles que aún no existen se quedan con su tramo de banco reservado,
    // para que al crearlos no haya que buscar hueco.
    for (int i = DEFAULT_PROFILES; i < MAX_PROFILES; i++) profiles[i].oled_bank = (uint8_t)i;

    memset(custom_oled_mask, 0, sizeof(custom_oled_mask));
    staging_profile = staging_page = 0xFF;
    staging_dirty = false;
    active_profile_idx = 0;
    active_page = 0;
    current_brightness = 207;
    reposo_timeout_min = 5;
}

// Graba un tramo del banco. Es la única función que escribe iconos en la Flash.
static void write_oled_slice(uint8_t bank, uint8_t page, const uint8_t* src) {
    tud_task();
    watchdog_update(); // borrar y programar 8 KB ronda los 120 ms

    // La migración escribe iconos desde load_settings(), que corre ANTES de
    // lanzar Core 1: pedirle entonces que se aparte bloquearía para siempre
    // esperando a un core que todavía no existe.
    const bool lock = multicore_lockout_victim_is_initialized(1);

    uint32_t ints = save_and_disable_interrupts();
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(FLASH_OLED_AT(bank, page), FLASH_OLED_SLICE);
    flash_range_program(FLASH_OLED_AT(bank, page), src, OLED_PAGE_STRIDE);
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(ints);
}

static void flush_oled_staging() {
    if (!staging_dirty || staging_profile >= MAX_PROFILES) return;
    write_oled_slice(oled_bank_of(staging_profile), staging_page, oled_staging);
    staging_dirty = false;
}

void save_settings() {
    // Primero los iconos pendientes: si se cortara la corriente a medias, es
    // mejor tener los iconos sin la máscara que la máscara sin los iconos.
    flush_oled_staging();

    Settings* s = (Settings*)settings_blob;
    memset(settings_blob, 0, sizeof(settings_blob));

    s->magic                  = SETTINGS_MAGIC;
    s->active_profile_idx     = active_profile_idx;
    s->current_brightness     = current_brightness;
    s->reposo_timeout_min     = reposo_timeout_min;
    s->profile_count          = profile_count;
    s->active_page            = active_page;
    memcpy(s->custom_oled_mask, custom_oled_mask, sizeof(custom_oled_mask));
    memcpy(s->profiles, profiles, sizeof(profiles));

    tud_task();
    watchdog_update();
    const bool lock = multicore_lockout_victim_is_initialized(1);
    uint32_t ints = save_and_disable_interrupts();
    if (lock) multicore_lockout_start_blocking();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SETTINGS_BYTES);
    flash_range_program(FLASH_TARGET_OFFSET, settings_blob, SETTINGS_BLOB_SIZE);
    if (lock) multicore_lockout_end_blocking();
    restore_interrupts(ints);
}

// Migración del banco de iconos: los formatos anteriores lo tenían detrás de los
// ajustes con un tramo por perfil, y ahora está en 1024 KB con un tramo por
// (perfil, página). Los iconos viejos pasan a ser la página 0 de cada perfil.
//
// Las dos zonas son disjuntas (origen 1544-1672 KB, destino 1024-1536 KB), así
// que se puede copiar en orden y sin solapamientos. Eso sí: hay que copiar ANTES
// de reescribir los ajustes, porque la región de ajustes nueva sí pisa la del
// banco viejo.
static void migrate_oled_bank(uint32_t old_slice_at(uint8_t), uint8_t count) {
    for (uint8_t p = 0; p < count && p < MAX_PROFILES; p++) {
        if (!custom_oled_mask[p][0]) continue;
        memcpy(oled_staging, (const uint8_t*)(XIP_BASE + old_slice_at(p)), OLED_PAGE_STRIDE);
        write_oled_slice(profiles[p].oled_bank, 0, oled_staging);
    }
    staging_profile = staging_page = 0xFF;
    staging_dirty = false;
}

static uint32_t old_slice_v5(uint8_t p) { return FLASH_OLED_AT_V5(p); }
static uint32_t old_slice_v3(uint8_t p) { return FLASH_OLED_AT_V3(p); }

// Convierte un perfil sin páginas en uno de una sola página.
static void adopt_v5_profile(Profile& dst, const ProfileV5& src, uint8_t bank) {
    memset(&dst, 0, sizeof(dst));
    memcpy(dst.name, src.name, sizeof(dst.name));
    dst.page_count = 1;
    dst.oled_bank  = bank;
    memcpy(dst.pages[0].oled_labels,  src.oled_labels,  sizeof(dst.pages[0].oled_labels));
    memcpy(dst.pages[0].key_mappings, src.key_mappings, sizeof(dst.pages[0].key_mappings));
    memcpy(dst.pages[0].rotary,       src.rotary,       sizeof(dst.pages[0].rotary));
    dst.pages[0].scroll_detents[0] = src.scroll_detents[0];
    dst.pages[0].scroll_detents[1] = src.scroll_detents[1];
    dst.pages[0].scroll_invert[0]  = src.scroll_invert[0];
    dst.pages[0].scroll_invert[1]  = src.scroll_invert[1];
}

static void sanitize_loaded() {
    if (profile_count < 1) profile_count = 1;
    if (profile_count > MAX_PROFILES) profile_count = MAX_PROFILES;
    if (active_profile_idx >= profile_count) active_profile_idx = 0;

    // Los tramos del banco tienen que ser válidos y no repetirse: dos perfiles
    // compartiendo tramo se pisarían los iconos.
    bool taken[MAX_PROFILES] = { false };
    for (uint8_t i = 0; i < MAX_PROFILES; i++) {
        uint8_t b = profiles[i].oled_bank;
        if (b >= MAX_PROFILES || taken[b]) {
            for (uint8_t c = 0; c < MAX_PROFILES; c++) {
                if (!taken[c]) { b = c; break; }
            }
            profiles[i].oled_bank = b;
        }
        taken[b] = true;
    }

    for (uint8_t i = 0; i < MAX_PROFILES; i++) {
        if (profiles[i].page_count < 1) profiles[i].page_count = 1;
        if (profiles[i].page_count > MAX_PAGES) profiles[i].page_count = MAX_PAGES;

        for (uint8_t pg = 0; pg < MAX_PAGES; pg++) {
            for (uint8_t l = 0; l < 2; l++) {
                uint8_t d = profiles[i].pages[pg].scroll_detents[l];
                if (d < SCROLL_DETENTS_MIN || d > SCROLL_DETENTS_MAX) {
                    profiles[i].pages[pg].scroll_detents[l] = SCROLL_DETENTS_DEFAULT;
                }
                profiles[i].pages[pg].scroll_invert[l] =
                    profiles[i].pages[pg].scroll_invert[l] ? 1 : 0;
            }
        }
    }

    if (active_page >= profiles[active_profile_idx].page_count) active_page = 0;
}

void load_settings() {
    const uint32_t magic = *(const uint32_t*)(XIP_BASE + FLASH_TARGET_OFFSET);

    if (magic == SETTINGS_MAGIC) {
        const Settings* s = (const Settings*)(XIP_BASE + FLASH_TARGET_OFFSET);
        active_profile_idx     = s->active_profile_idx;
        current_brightness     = s->current_brightness;
        reposo_timeout_min     = s->reposo_timeout_min;
        profile_count          = s->profile_count;
        active_page            = s->active_page;
        memcpy(profiles, s->profiles, sizeof(profiles));
        memcpy(custom_oled_mask, s->custom_oled_mask, sizeof(custom_oled_mask));
        sanitize_loaded();
        return;
    }

    // --- Formatos sin páginas (3.0 y 3.1) ---
    // Lo guardado se convierte en la página 0 de cada perfil, y sus iconos se
    // copian del banco viejo al nuevo ANTES de reescribir los ajustes: la región
    // de ajustes nueva se solapa con la del banco antiguo.
    if (magic == SETTINGS_MAGIC_V5 || magic == SETTINGS_MAGIC_V4) {
        const bool v5 = (magic == SETTINGS_MAGIC_V5);
        const SettingsV5* s5 = (const SettingsV5*)(XIP_BASE + FLASH_TARGET_OFFSET);
        const SettingsV4* s4 = (const SettingsV4*)(XIP_BASE + FLASH_TARGET_OFFSET);

        active_profile_idx = v5 ? s5->active_profile_idx : s4->active_profile_idx;
        current_brightness = v5 ? s5->current_brightness : s4->current_brightness;
        reposo_timeout_min = v5 ? s5->reposo_timeout_min : s4->reposo_timeout_min;
        profile_count      = v5 ? s5->profile_count      : s4->profile_count;
        active_page        = 0;

        const uint8_t count = v5 ? MAX_PROFILES : SETTINGS_V4_PROFILES;
        memset(profiles, 0, sizeof(profiles));
        memset(custom_oled_mask, 0, sizeof(custom_oled_mask));
        for (uint8_t i = 0; i < MAX_PROFILES; i++) {
            if (i < count) {
                adopt_v5_profile(profiles[i], v5 ? s5->profiles[i] : s4->profiles[i], i);
                custom_oled_mask[i][0] = v5 ? s5->custom_oled_mask[i] : s4->custom_oled_mask[i];
            } else {
                profiles[i].oled_bank = i;
            }
        }
        sanitize_loaded();
        // El banco viejo de la 3.1 iba dos sectores detrás de los ajustes; el de
        // la 3.0, uno.
        migrate_oled_bank(v5 ? old_slice_v5 : old_slice_v3, count);
        // Se graba ya en el formato nuevo: si no, la migración se repetiría en
        // cada arranque y reescribiría los iconos una y otra vez.
        save_settings();
        return;
    }

    if (magic == SETTINGS_MAGIC_V3) {
        // Formato anterior a los perfiles variables: cuatro perfiles con los
        // mandos en una sola capa y un scroll global. Se conserva todo y se
        // duplica la capa normal en SUPER, que es el comportamiento previo.
        const SettingsV3* s = (const SettingsV3*)(XIP_BASE + FLASH_TARGET_OFFSET);
        active_profile_idx = s->active_profile_idx;
        current_brightness = s->current_brightness;
        reposo_timeout_min = s->reposo_timeout_min;
        profile_count      = 4;
        memset(profiles, 0, sizeof(profiles));
        memset(custom_oled_mask, 0, sizeof(custom_oled_mask));
        memcpy(custom_oled_mask, s->custom_oled_mask, sizeof(s->custom_oled_mask));

        for (int i = 0; i < MAX_PROFILES; i++) profiles[i].oled_bank = (uint8_t)i;
        for (int i = 0; i < 4; i++) {
            Page& pg = profiles[i].pages[0];
            profiles[i].page_count = 1;
            memcpy(profiles[i].name, s->profiles[i].name, sizeof(profiles[i].name));
            memcpy(pg.oled_labels,  s->profiles[i].oled_labels,  sizeof(pg.oled_labels));
            memcpy(pg.key_mappings, s->profiles[i].key_mappings, sizeof(pg.key_mappings));
            for (uint8_t r = 0; r < ROT_LAYER_STRIDE; r++) {
                pg.rotary[r] = s->profiles[i].rotary[r];
                pg.rotary[ROT_LAYER_STRIDE + r] = s->profiles[i].rotary[r];
            }
            pg.scroll_detents[0] = s->scroll_detents_per_rev;
            pg.scroll_detents[1] = s->scroll_detents_per_rev;
            pg.scroll_invert[0]  = s->scroll_invert;
            pg.scroll_invert[1]  = s->scroll_invert;
        }
        sanitize_loaded();
        migrate_oled_bank(old_slice_v3, 4);
        save_settings();
        return;
    }

    if (magic == SETTINGS_MAGIC_V2) {
        // Formato anterior a las acciones giratorias. Se conserva todo lo que
        // costó trabajo —perfiles, etiquetas, atajos y la máscara de iconos— y
        // solo se rellenan de fábrica los mandos, que antes no existían.
        const SettingsV2* s = (const SettingsV2*)(XIP_BASE + FLASH_TARGET_OFFSET);
        active_profile_idx = s->active_profile_idx;
        current_brightness = s->current_brightness;
        reposo_timeout_min = s->reposo_timeout_min;
        profile_count      = 4;
        memset(profiles, 0, sizeof(profiles));
        memset(custom_oled_mask, 0, sizeof(custom_oled_mask));
        for (int i = 0; i < 4; i++) custom_oled_mask[i][0] = s->custom_oled_mask[i];

        for (int i = 0; i < MAX_PROFILES; i++) profiles[i].oled_bank = (uint8_t)i;
        for (int i = 0; i < 4; i++) {
            Page& pg = profiles[i].pages[0];
            profiles[i].page_count = 1;
            memcpy(profiles[i].name, s->profiles[i].name, sizeof(profiles[i].name));
            memcpy(pg.oled_labels,  s->profiles[i].oled_labels,  sizeof(pg.oled_labels));
            memcpy(pg.key_mappings, s->profiles[i].key_mappings, sizeof(pg.key_mappings));
            apply_default_rotaries(pg);
            apply_default_scroll(pg);
            pg.scroll_detents[0] = s->scroll_detents_per_rev;
            pg.scroll_detents[1] = s->scroll_detents_per_rev;
            pg.scroll_invert[0]  = s->scroll_invert;
            pg.scroll_invert[1]  = s->scroll_invert;
        }
        sanitize_loaded();
        migrate_oled_bank(old_slice_v3, 4);
        save_settings();
        return;
    }

    if (magic == SETTINGS_MAGIC_V1) {
        // El formato original solo guardaba tres ajustes; el resto es de fábrica.
        const SettingsV1* s = (const SettingsV1*)(XIP_BASE + FLASH_TARGET_OFFSET);
        load_defaults();
        active_profile_idx = s->active_profile_idx;
        current_brightness = s->current_brightness;
        reposo_timeout_min = s->reposo_timeout_min;
        sanitize_loaded();
        return; // no había bitmaps guardados que rescatar
    }

    load_defaults();
}

// ==========================================
// GESTIÓN DE PERFILES (CREAR / DUPLICAR / BORRAR)
// ==========================================
// Cada perfil se queda con su tramo del banco de iconos (`oled_bank`) mientras
// viva, así que reordenar la lista solo mueve metadatos: los iconos se quedan
// donde están y se siguen encontrando por el tramo, no por el índice. Antes, con
// el banco indexado por posición, borrar un perfil obligaba a arrastrar el tramo
// de todos los siguientes; ahora eso serían medio megabyte de Flash por nada.

// Copia los iconos de un perfil a otro, página a página. Es lo único que sí
// obliga a escribir en la Flash, y solo pasa al duplicar un perfil a mano.
static void oled_copy_profile(uint8_t dst, uint8_t src) {
    flush_oled_staging();
    for (uint8_t pg = 0; pg < MAX_PAGES; pg++) {
        custom_oled_mask[dst][pg] = custom_oled_mask[src][pg];
        if (!custom_oled_mask[src][pg]) continue;
        memcpy(oled_staging,
               (const uint8_t*)(XIP_BASE + FLASH_OLED_AT(profiles[src].oled_bank, pg)),
               OLED_PAGE_STRIDE);
        write_oled_slice(profiles[dst].oled_bank, pg, oled_staging);
    }
    staging_profile = staging_page = 0xFF;
    staging_dirty = false;
}

// Devuelve el índice del perfil nuevo, o -1 si ya no caben más.
static int profile_add(int copy_from) {
    if (profile_count >= MAX_PROFILES) return -1;

    uint8_t idx = profile_count;
    // El tramo de banco que ya tenía reservado este hueco se conserva: los
    // huecos libres arrastran su tramo desde load_defaults / sanitize_loaded.
    uint8_t bank = profiles[idx].oled_bank;

    if (copy_from >= 0 && copy_from < (int)profile_count) {
        profiles[idx] = profiles[copy_from];
        profiles[idx].oled_bank = bank;   // el suyo, no el del original
        profile_count = idx + 1;
        oled_copy_profile(idx, (uint8_t)copy_from);
    } else {
        make_blank_profile(profiles[idx], (uint8_t)(idx + 1), bank);
        for (uint8_t pg = 0; pg < MAX_PAGES; pg++) custom_oled_mask[idx][pg] = 0;
        profile_count = idx + 1;
    }
    return idx;
}

static bool profile_remove(uint8_t idx) {
    if (profile_count <= 1 || idx >= profile_count) return false;

    // Si el búfer de edición apuntaba al perfil que se va, se descarta: grabarlo
    // después escribiría en un tramo que ya no le pertenece a nadie.
    if (staging_profile == idx) {
        staging_profile = staging_page = 0xFF;
        staging_dirty = false;
    }

    // Los metadatos se desplazan arrastrando cada uno su tramo y su máscara.
    Profile  gone      = profiles[idx];
    uint32_t gone_mask[MAX_PAGES];
    memcpy(gone_mask, custom_oled_mask[idx], sizeof(gone_mask));

    for (uint8_t i = idx; i + 1 < profile_count; i++) {
        profiles[i] = profiles[i + 1];
        memcpy(custom_oled_mask[i], custom_oled_mask[i + 1], sizeof(gone_mask));
    }

    // El hueco que queda libre hereda el tramo del perfil borrado, para que
    // ningún tramo se quede huérfano y todos sigan siendo distintos.
    uint8_t last = (uint8_t)(profile_count - 1);
    memset(&profiles[last], 0, sizeof(profiles[last]));
    profiles[last].oled_bank = gone.oled_bank;
    memset(custom_oled_mask[last], 0, sizeof(gone_mask));
    profile_count = last;
    (void)gone_mask;

    if (active_profile_idx >= profile_count) active_profile_idx = (uint8_t)(profile_count - 1);
    else if (active_profile_idx > idx)       active_profile_idx--;
    if (active_page >= profiles[active_profile_idx].page_count) active_page = 0;
    return true;
}

// ==========================================
// PÁGINAS: CAMBIO Y GESTIÓN
// ==========================================
static void set_active_page(uint8_t pg) {
    uint8_t count = profiles[active_profile_idx].page_count;
    if (count < 1) count = 1;
    if (pg >= count) pg = 0;
    if (pg == active_page) return;
    active_page = pg;
    system_refresh_req = true;
}

static void next_page() {
    uint8_t count = profiles[active_profile_idx].page_count;
    if (count <= 1) return;
    set_active_page((uint8_t)((active_page + 1) % count));
}

// Añade una página al perfil, copiando la actual para no partir de un lienzo en
// blanco. Devuelve false si ya está en el tope.
static bool page_add(uint8_t profile, bool copy_current) {
    if (profile >= MAX_PROFILES) return false;
    Profile& p = profiles[profile];
    if (p.page_count >= MAX_PAGES) return false;
    uint8_t idx = p.page_count;
    if (copy_current && active_page < idx) p.pages[idx] = p.pages[active_page];
    else                                   make_blank_page(p.pages[idx]);
    custom_oled_mask[profile][idx] = 0;
    p.page_count = (uint8_t)(idx + 1);
    return true;
}

static bool page_remove(uint8_t profile, uint8_t pg) {
    if (profile >= MAX_PROFILES) return false;
    Profile& p = profiles[profile];
    if (p.page_count <= 1 || pg >= p.page_count) return false;

    flush_oled_staging();
    if (staging_profile == profile) {
        staging_profile = staging_page = 0xFF;
        staging_dirty = false;
    }
    // Aquí sí hay que mover los iconos: el número de página ES el índice de su
    // tramo, así que desplazar las páginas sin desplazarlos los descuadraría.
    // En orden ascendente nunca se lee un tramo ya sobrescrito.
    for (uint8_t i = pg; i + 1 < p.page_count; i++) {
        p.pages[i] = p.pages[i + 1];
        custom_oled_mask[profile][i] = custom_oled_mask[profile][i + 1];
        if (custom_oled_mask[profile][i]) {
            memcpy(oled_staging,
                   (const uint8_t*)(XIP_BASE + FLASH_OLED_AT(p.oled_bank, i + 1)),
                   OLED_PAGE_STRIDE);
            write_oled_slice(p.oled_bank, i, oled_staging);
        }
    }
    staging_profile = staging_page = 0xFF;
    staging_dirty = false;
    uint8_t last = (uint8_t)(p.page_count - 1);
    memset(&p.pages[last], 0, sizeof(p.pages[last]));
    custom_oled_mask[profile][last] = 0;
    p.page_count = last;

    if (profile == active_profile_idx && active_page >= p.page_count) active_page = 0;
    return true;
}

// Menú físico de perfiles. Las opciones son un perfil por entrada más BACK al
// final, pero solo hay diez pantallas: con más perfiles la lista se muestra por
// ventanas de diez que siguen al cursor.
#define PERF_MENU_SCREENS 10

static inline uint8_t perf_menu_count() {
    return (uint8_t)(profile_count + 1);  // + BACK
}

// Primera opción visible en las pantallas.
volatile uint8_t perf_menu_first = 0;

// Recoloca la ventana para que la opción elegida quede siempre a la vista.
static void perf_menu_follow() {
    if (selected_menu_idx < perf_menu_first) {
        perf_menu_first = selected_menu_idx;
    } else if (selected_menu_idx >= perf_menu_first + PERF_MENU_SCREENS) {
        perf_menu_first = (uint8_t)(selected_menu_idx - PERF_MENU_SCREENS + 1);
    }
}

// Al abrir el menú el cursor arranca en el perfil que está puesto: con muchos
// perfiles, empezar siempre en el primero obligaba a recorrer la lista entera.
static void perf_menu_open() {
    selected_menu_idx = (active_profile_idx < profile_count) ? active_profile_idx : 0;
    perf_menu_first = 0;
    perf_menu_follow();
}

// ==========================================
// ACCIONES USB HID NATIVAS
// ==========================================

uint16_t get_consumer_key_from_index(uint8_t idx) {
    switch (idx) {
        case 1:  return 0x0194; // AL Local Machine Browser
        case 2:  return 0x011A; // AL Calculator
        case 3:  return 0x00CD; // Play/Pause
        case 4:  return 0x00B7; // Stop
        case 5:  return 0x00B5; // Next Track
        case 6:  return 0x00B6; // Prev Track
        case 7:  return 0x00E9; // Volume Increment
        case 8:  return 0x00EA; // Volume Decrement
        case 9:  return 0x00E2; // Mute
        // Ojo: el código anterior usaba 0x00B0/0x00B1 para el brillo, que en la
        // página de consumo son Play y Pause. Los correctos son 0x006F/0x0070.
        case 10: return 0x006F; // Display Brightness Increment
        case 11: return 0x0070; // Display Brightness Decrement
        case 12: return 0x0196; // AL Internet Browser
        case 13: return 0x018A; // AL Email Reader
        case 14: return 0x0221; // AC Search
        case 15: return 0x022D; // AC Zoom In
        case 16: return 0x022E; // AC Zoom Out
        default: return 0;
    }
}

// ==========================================
// REPRODUCTOR DE SECUENCIAS (no bloqueante)
// ==========================================
// Igual que zoom_hold_until: un "plazo" que se comprueba una vez por vuelta del
// bucle, nunca una espera bloqueante. Como mucho se manda UN informe HID por
// vuelta (el endpoint no admite más), así que cada paso que toca al ratón se
// reintenta solo, sin congelar el resto del teclado (rueda, teclas, OLED).

// Declarada más abajo (sección CDC); hace falta antes para la telemetría de
// MACRO_TEST.
static void cdc_printf(const char* fmt, ...);

#define MACRO_CLICK_HOLD_MS 20

// Cuánto se empuja de golpe hacia la esquina inferior derecha antes de dar por
// hecho que el recorte del sistema operativo ya ha actuado. De sobra para
// cualquier pantalla (unos 4000/127 ≈ 32 informes, así que no alarga la
// espera nada que se note).
#define MACRO_HOME_BURST_PX 4000

enum MacroPhase : uint8_t {
    MPH_IDLE          = 0, // nada pendiente: toca leer el siguiente paso
    MPH_MOVE_SENT     = 1, // tras mandar un MSTEP_MOVE de un solo informe
    MPH_CLICK_PRESSED = 2, // tras mandar la pulsación de un clic
    MPH_CLICK_HOLD    = 3, // esperando el hold antes de soltarlo
    MPH_CLICK_RELEASED = 4, // tras mandar la suelta
    MPH_CHUNK_HOME    = 5, // empujón hacia la esquina, a trozos de ±127
    MPH_CHUNK_TARGET  = 6, // desde la esquina hasta (x, y), a trozos de ±127
};

struct MacroPlayer {
    bool     active;
    uint8_t  id;
    uint8_t  step;
    uint8_t  phase;
    uint32_t deadline;
    bool     report_pending;
    orby_mouse_report_t pending_report;
    // Lo que le queda por recorrer al movimiento a trozos en curso (home o
    // hacia el objetivo): con int32 no hay riesgo de desbordar sumando pasos
    // de hasta ±20000.
    int32_t  chunk_remaining_x;
    int32_t  chunk_remaining_y;
    // Solo con MACRO_TEST: manda por CDC cada trozo que se manda de verdad,
    // para poder ver qué está calculando el reproductor aunque la app esté
    // conectada (que si no, siempre gana el camino del PC y esto no se ve).
    bool     debug;
};
static MacroPlayer macro_player = { false, 0, 0, 0, 0, false, { 0, 0, 0, 0, 0 }, 0, 0, false };

static void macro_queue_report(uint8_t buttons, int8_t dx, int8_t dy) {
    macro_player.pending_report = { buttons, dx, dy, 0, 0 };
    macro_player.report_pending = true;
}

// Manda el informe de ratón pendiente sin bloquear: si el endpoint está ocupado
// (lo comparte con teclado y rueda), se reintenta solo en la próxima vuelta.
static bool macro_try_send_report() {
    if (!tud_hid_ready()) return false;
    if (!tud_hid_report(REPORT_ID_MOUSE, &macro_player.pending_report, sizeof(macro_player.pending_report))) return false;
    macro_player.report_pending = false;
    return true;
}

static void macro_player_start(uint8_t id) {
    macro_player.active = true;
    macro_player.id = id;
    macro_player.step = 0;
    macro_player.phase = MPH_IDLE;
    macro_player.report_pending = false;
    macro_player.deadline = to_ms_since_boot(get_absolute_time());
    macro_player.debug = false;
}

// Encola como mucho un trozo de ±127 del movimiento a trozos en curso y lo
// descuenta de lo que queda YA, no cuando se confirme el envío: el envío se
// reintenta solo (vía el bloque de report_pending al principio de la vuelta)
// mandando ese mismo trozo tal cual hasta que salga, así que si se restara al
// confirmarse, un reintento no restaría nada y el movimiento no terminaría
// nunca (justo lo que pasaba antes: se quedaba mandando el mismo trozo hacia
// la esquina para siempre en cuanto el endpoint tardaba una vuelta en
// quedar libre, que con teclado o rueda de por medio es lo normal).
static void macro_queue_chunk() {
    int32_t rx = macro_player.chunk_remaining_x;
    int32_t ry = macro_player.chunk_remaining_y;
    int16_t dx = rx > 127 ? 127 : (rx < -127 ? -127 : (int16_t)rx);
    int16_t dy = ry > 127 ? 127 : (ry < -127 ? -127 : (int16_t)ry);
    macro_player.chunk_remaining_x -= dx;
    macro_player.chunk_remaining_y -= dy;
    if (macro_player.debug) {
        cdc_printf("MACRODBG:CHUNK:ph=%d:dx=%d:dy=%d:restaX=%ld:restaY=%ld\n",
                   (int)macro_player.phase, (int)dx, (int)dy,
                   (long)macro_player.chunk_remaining_x, (long)macro_player.chunk_remaining_y);
    }
    macro_queue_report(0, (int8_t)dx, (int8_t)dy);
}

// Se llama una vez por vuelta del bucle principal. Actúa como mucho un paso —o
// un trozo de un movimiento largo, o media pulsación de clic— cada vez.
static void macro_player_tick(uint32_t now) {
    if (!macro_player.active) return;
    MacroPlayer& mp = macro_player;

    if (mp.report_pending) {
        if (!macro_try_send_report()) return;
        switch (mp.phase) {
            case MPH_MOVE_SENT: mp.step++; mp.deadline = now; mp.phase = MPH_IDLE; break;
            case MPH_CLICK_PRESSED: mp.deadline = now + MACRO_CLICK_HOLD_MS; mp.phase = MPH_CLICK_HOLD; return;
            case MPH_CLICK_RELEASED: mp.step++; mp.deadline = now; mp.phase = MPH_IDLE; break;
            case MPH_CHUNK_HOME: case MPH_CHUNK_TARGET: break; // se decide más abajo
            default: break;
        }
    }

    if ((int32_t)(now - mp.deadline) < 0) return;

    if (mp.phase == MPH_CLICK_HOLD) {
        macro_queue_report(0, 0, 0);
        mp.phase = MPH_CLICK_RELEASED;
        if (macro_try_send_report()) { mp.step++; mp.deadline = now; mp.phase = MPH_IDLE; }
        return;
    }

    if (mp.phase == MPH_CHUNK_HOME || mp.phase == MPH_CHUNK_TARGET) {
        if (mp.chunk_remaining_x == 0 && mp.chunk_remaining_y == 0) {
            if (mp.phase == MPH_CHUNK_HOME) {
                // Ya en la esquina inferior derecha: ahora, en relativo, el
                // tramo final (a, b) hasta el punto que se quería.
                const MacroStep& s = macros[mp.id].steps[mp.step];
                mp.chunk_remaining_x = s.a;
                mp.chunk_remaining_y = s.b;
                mp.phase = MPH_CHUNK_TARGET;
                if (mp.debug) {
                    cdc_printf("MACRODBG:HOME_DONE:step=%d:objX=%d:objY=%d\n",
                               (int)mp.step, (int)s.a, (int)s.b);
                }
                // sigue abajo: manda el primer trozo hacia el objetivo ya en
                // esta misma vuelta, sin esperar a la próxima.
            } else {
                if (mp.debug) cdc_printf("MACRODBG:TARGET_DONE:step=%d\n", (int)mp.step);
                mp.step++; mp.deadline = now; mp.phase = MPH_IDLE;
                return;
            }
        }
        macro_queue_chunk();
        macro_try_send_report(); // si falla (endpoint ocupado), se reintenta solo arriba
        return;
    }

    if (mp.id >= MACRO_MAX_COUNT || mp.step >= macros[mp.id].step_count) {
        mp.active = false;
        return;
    }

    const MacroStep& s = macros[mp.id].steps[mp.step];
    switch (s.type) {
        case MSTEP_DELAY:
            mp.deadline = now + (uint32_t)(s.a > 0 ? s.a : 0);
            mp.step++;
            break;

        case MSTEP_KEY:
            HidOut::tap((uint8_t)s.a, (uint8_t)s.b);
            mp.step++;
            mp.deadline = now;
            break;

        case MSTEP_MOVE: {
            int16_t dx = s.a < -127 ? -127 : (s.a > 127 ? 127 : s.a);
            int16_t dy = s.b < -127 ? -127 : (s.b > 127 ? 127 : s.b);
            macro_queue_report(0, (int8_t)dx, (int8_t)dy);
            mp.phase = MPH_MOVE_SENT;
            if (macro_try_send_report()) { mp.step++; mp.deadline = now; mp.phase = MPH_IDLE; }
            break;
        }

        case MSTEP_CLICK: {
            uint8_t bit = (s.a == 2) ? 0x02 : (s.a == 1) ? 0x04 : 0x01; // izq/centro/der
            macro_queue_report(bit, 0, 0);
            mp.phase = MPH_CLICK_PRESSED;
            if (macro_try_send_report()) { mp.deadline = now + MACRO_CLICK_HOLD_MS; mp.phase = MPH_CLICK_HOLD; }
            break;
        }

        case MSTEP_HOME_MOVE:
            mp.chunk_remaining_x = MACRO_HOME_BURST_PX;
            mp.chunk_remaining_y = MACRO_HOME_BURST_PX;
            mp.phase = MPH_CHUNK_HOME;
            // El paso avanza cuando el trozo hacia el objetivo termine, no aquí.
            break;

        default:
            mp.step++;
            break;
    }

    if (mp.phase == MPH_IDLE && mp.step >= macros[mp.id].step_count) mp.active = false;
}

// Dispara una macro. Reproducirla en el propio teclado —con el "homing" de
// MSTEP_HOME_MOVE si le hace falta— es solo un mecanismo de supervivencia
// para cuando no hay PC (o su app no está abierta): con la app conectada, se
// prefiere avisarla por CDC y que la ejecute ella, que sabe la posición real
// del ratón sin tener que empujarlo a una esquina primero. tud_cdc_n_connected
// refleja el DTR que la app pone al abrir el puerto (ver electron/serial.js),
// así que es un buen "¿hay alguien escuchando?" sin depender de que responda
// a tiempo ni de nada más lento que consultar un bit.
static void trigger_macro(uint8_t id) {
    const bool app_connected = tud_cdc_n_connected(0);
    const bool device_has_it = (id < MACRO_MAX_COUNT && macros[id].step_count > 0);

    if (!app_connected && device_has_it) {
        // Si ya hay una en marcha se ignora el reintento en vez de encolarla o
        // mandarla también al PC, que la ejecutaría dos veces por dos caminos.
        if (!macro_player.active) macro_player_start(id);
        return;
    }

    char tel[24];
    int len = snprintf(tel, sizeof(tel), "MACRO:%d\n", (int)id);
    tud_cdc_n_write(0, tel, len);
    tud_cdc_n_write_flush(0);
}

// El envío de informes vive en hid_out.h: estado de teclas con rollover de seis
// y cola para las pulsaciones puntuales, sin esperas bloqueantes.

// ==========================================
// RUEDA DE SCROLL -> HID
// ==========================================
// El cálculo se hace siempre en unidades de alta resolución y el resto NUNCA
// se descarta: es exactamente lo que provocaba el scroll a saltos, porque
// `wheel_delta / 8` tiraba la parte fraccionaria en cada informe y además se
// forzaba un mínimo de ±1 detent (= 3 líneas en Windows) ante cualquier
// micro-movimiento.
static int32_t scroll_frac    = 0;  // resto de la conversión cuentas -> unidades
static int32_t scroll_detent_frac = 0; // resto al degradar a detents clásicos

// La cola de salida y el envío del informe viven en wheel_out.h.

// Ctrl pegajoso del zoom. Antes se pulsaba y se soltaba en cada detent, con la
// rueda enviada en medio; a poco que se girase rápido, el host recibía la rueda
// sin tener claro si el Ctrl estaba puesto. Ahora se mantiene mientras se siga
// girando y se suelta sola un rato después del último paso.
#define ZOOM_HOLD_MS 250
static uint32_t zoom_hold_until = 0;

// ==========================================
// EJECUCIÓN DE LAS ACCIONES GIRATORIAS
// ==========================================

// Los tipos de desplazamiento llevan el sentido en el signo, así que no
// necesitan una acción distinta por dirección.
static inline bool rotary_is_scroll(uint8_t type) {
    return type == ROT_SCROLL_V || type == ROT_SCROLL_H || type == ROT_ZOOM;
}

// Envía desplazamiento medido en detents clásicos (un clic de rueda de ratón).
static void emit_scroll(uint8_t type, int32_t detents) {
    if (detents == 0) return;

    if (type == ROT_SCROLL_H) {
        WheelOut::add_pan(detents * (g_hires_pan ? HID_HIRES_MULTIPLIER : 1));
        return;
    }

    if (type == ROT_ZOOM) {
        // Ctrl mantenido mientras dura el desplazamiento: es lo que interpretan
        // navegadores y editores como acercar/alejar. Lo suelta el bucle
        // principal cuando pasa ZOOM_HOLD_MS sin más pasos.
        zoom_hold_until = to_ms_since_boot(get_absolute_time()) + ZOOM_HOLD_MS;
        WheelOut::zoom_arm();
        HidOut::set_sticky_mod(KEYBOARD_MODIFIER_LEFTCTRL);
        WheelOut::add_wheel(detents * (g_hires_multiplier ? HID_HIRES_MULTIPLIER : 1));
        return;
    }

    WheelOut::add_wheel(detents * (g_hires_multiplier ? HID_HIRES_MULTIPLIER : 1));
}

// Una sola repetición de una acción discreta (tecla o multimedia).
static void emit_discrete(const RotaryAction& a) {
    if (a.type == ROT_CONSUMER) {
        HidOut::consumer_tap(get_consumer_key_from_index(a.keycode));
    } else if (a.type == ROT_KEY) {
        // Una macro es un ROT_KEY con el modificador reservado KEYACT_MACRO: no
        // hace falta un RotaryType nuevo, con reutilizar el campo basta, igual
        // que en KeyAction.
        if (a.modifier == KEYACT_MACRO) trigger_macro(a.keycode);
        else HidOut::tap(a.modifier, a.keycode);
    }
}

// `steps` positivo = sentido horario. Elige la acción según el sentido salvo
// que sea de desplazamiento, en cuyo caso una sola cubre ambos.
void apply_rotary(const RotaryAction& cw, const RotaryAction& ccw, int32_t steps) {
    if (steps == 0) return;

    if (rotary_is_scroll(cw.type)) {
        emit_scroll(cw.type, steps);
        return;
    }

    const RotaryAction& a = (steps > 0) ? cw : ccw;
    if (a.type == ROT_NONE) return;
    if (rotary_is_scroll(a.type)) { emit_scroll(a.type, steps); return; }

    int32_t n = (steps > 0) ? steps : -steps;
    if (n > 16) n = 16; // salvaguarda ante un giro brusco
    for (int32_t i = 0; i < n; i++) emit_discrete(a);
}

void wheel_accumulate(int32_t delta_counts) {
    const bool super = super_active;
    const Page& p = cur_page();
    const RotaryAction& cw  = p.rotary[rot_slot(ROT_WHEEL_CW,  super)];
    const RotaryAction& ccw = p.rotary[rot_slot(ROT_WHEEL_CCW, super)];

    if (cur_scroll_invert()) delta_counts = -delta_counts;

    // unidades_hires = cuentas * detents_por_vuelta * 120 / 4096
    scroll_frac += delta_counts * (int32_t)cur_scroll_detents() * HID_HIRES_MULTIPLIER;
    int32_t units = scroll_frac / AS5600_COUNTS_PER_REV;
    scroll_frac  -= units * AS5600_COUNTS_PER_REV;
    if (units == 0) return;

    // Los dos desplazamientos aprovechan la alta resolución; el resto de
    // acciones son discretas y necesitan detents completos.
    if (cw.type == ROT_SCROLL_V && g_hires_multiplier != 0) {
        WheelOut::add_wheel(units);
        return;
    }
    if (cw.type == ROT_SCROLL_H && g_hires_pan != 0) {
        WheelOut::add_pan(units);
        return;
    }

    scroll_detent_frac += units;
    int32_t detents = scroll_detent_frac / HID_HIRES_MULTIPLIER;
    scroll_detent_frac -= detents * HID_HIRES_MULTIPLIER;
    apply_rotary(cw, ccw, detents);
}

// ==========================================
// CORE 1: MANEJA EXCLUSIVAMENTE LAS PANTALLAS
// ==========================================
// Dibuja un marco rectangular premium en el framebuffer (72x40)
void draw_premium_frame(uint8_t* fb) {
    // Línea superior (bit 0 de la página 0) e inferior (bit 7 de la página 4)
    for (int x = 0; x < 72; x++) {
        fb[0 * 72 + x] |= 0x01;
        fb[4 * 72 + x] |= 0x80;
    }
    // Líneas laterales izquierda (x=0) y derecha (x=71)
    for (int p = 0; p < 5; p++) {
        fb[p * 72] = 0xFF;
        fb[p * 72 + 71] = 0xFF;
    }
}

// Dibuja el contenido de una pantalla según el modo y perfil actual
void refresh_single_screen(HardwareOled& oleds, uint8_t screen_num) {
    uint8_t fb[360];
    memset(fb, 0, sizeof(fb));

    if (current_mode == MODE_NORMAL) {
        uint8_t slot = (uint8_t)((screen_num - 1) + (super_active ? 10 : 0));

        // Una tecla configurada como estado de páginas enseña en qué página
        // estás, así que su pantalla no la manda el icono sino el estado.
        uint8_t ki = key_index_for_screen(screen_num);
        const KeyAction& act = cur_page().key_mappings[ki + (super_active ? 12 : 0)];
        if (act.modifier == KEYACT_PAGE_STATE) {
            char txt[8];
            snprintf(txt, sizeof(txt), "P%u/%u",
                     (unsigned)(active_page + 1),
                     (unsigned)profiles[active_profile_idx].page_count);
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer(txt, fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
            return;
        }

        if (slot < OLED_SLOTS && oled_slot_used(active_profile_idx, active_page, slot)) {
            // Icono personalizado subido desde la app con OLED_CHUNK
            oleds.paint_screen(screen_num, oled_slot_ptr(active_profile_idx, active_page, slot));
        } else if (active_profile_idx == 0 && active_page == 0 && !super_active) {
            // Perfil por defecto (OFIM) -> Carga mapas de bits pre-horneados
            const uint8_t* bmp = OledBitmaps::get_bitmap_by_index(screen_num - 1);
            oleds.paint_screen(screen_num, bmp);
        } else {
            // Perfiles de fábrica de texto personalizado con contorno premium (o OFIM en SUPER)
            draw_premium_frame(fb);
            int label_idx = (screen_num - 1) + (super_active ? 10 : 0);
            const char* label = cur_page().oled_labels[label_idx];
            OledText::render_string_to_framebuffer(label, fb);
            draw_premium_frame(fb); // Volver a aplicar el marco tras renderizar el texto
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_MAIN) {
        if (screen_num >= 1 && screen_num <= 4) {
            draw_premium_frame(fb);
            const char* menu_opts[4] = {"PERF", "REPO", "INFO", "EXIT"};
            OledText::render_string_to_framebuffer(menu_opts[screen_num - 1], fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else {
            // Pantallas 5-10 vacías
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_PERF) {
        // Cada pantalla enseña una opción de la ventana visible: los perfiles y,
        // tras el último, BACK. Con más de diez perfiles la ventana se desplaza.
        uint8_t option = (uint8_t)(perf_menu_first + screen_num - 1);

        if (option < profile_count) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer(profiles[option].name, fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (option == profile_count) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("BACK", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else {
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_REPO) {
        if (screen_num >= 1 && screen_num <= 4) {
            draw_premium_frame(fb);
            const char* repo_opts[4] = {"1 MIN", "5 MIN", "10MIN", "OFF"};
            OledText::render_string_to_framebuffer(repo_opts[screen_num - 1], fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 5) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("BACK", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else {
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_INFO) {
        if (screen_num == 1) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("FW10", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 2) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("CONN", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 3) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("ORBY", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 5) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("BACK", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else {
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_PAGES) {
        // Gestor de páginas: una pantalla por página con su número. La página en
        // la que estás se marca invirtiendo la pantalla (lo hace el bucle de
        // Core 1, aquí solo va el número).
        uint8_t count = profiles[active_profile_idx].page_count;
        if (screen_num <= count) {
            char txt[8];
            snprintf(txt, sizeof(txt), "%u", (unsigned)screen_num);
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer(txt, fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 10) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("BACK", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else {
            oleds.paint_screen(screen_num, fb);
        }
    }
}

void core1_entry() {
    multicore_lockout_victim_init(); // Permitir que Core 0 pause este core durante escrituras Flash
    HardwareOled oleds;
    oleds.init_all_screens();

    // Intro de arranque. Si no hay animación horneada se limita a pintar el
    // contenido normal, que es lo que había aquí antes.
    intro_running = true;
    OledIntro::run(oleds, intro_should_skip, refresh_single_screen, current_brightness);
    intro_running = false;

    uint16_t current_inversions = 0;

    while (true) {
        core1_heartbeat++; // Core 0 no alimenta el watchdog si esto se para

        // El encendido y apagado del reposo lo pide Core 0 pero lo ejecuta este
        // core, que es el único dueño del bus SPI y de los pines del OLED.
        if (display_power_req) {
            display_power_req = false;
            oleds.set_all_displays_on(displays_on);
        }

        if (system_refresh_req) {
            system_refresh_req = false;

            // Aplicar brillo y estado en caliente
            oleds.set_brightness(current_brightness);

            // Restablecer la inversión de todas las pantallas a NORMAL (false)
            // Esto soluciona que queden pantallas invertidas al entrar o salir de menús
            for (int i = 1; i <= 10; i++) {
                oleds.invert_screen(i, false);
            }

            // Refrescar todas las pantallas de acuerdo al estado compartido
            for (int i = 1; i <= 10; i++) {
                refresh_single_screen(oleds, i);
            }

            // Si estamos en un menú con cursor, aplicar inversiones
            if (current_mode == MODE_MENU_PERF) {
                // El cursor se marca sobre la ventana visible, que con muchos
                // perfiles no empieza necesariamente en el primero.
                for (int i = 1; i <= PERF_MENU_SCREENS; i++) {
                    oleds.invert_screen(i, (selected_menu_idx == perf_menu_first + i - 1));
                }
            } else if (current_mode == MODE_MENU_MAIN || current_mode == MODE_MENU_REPO) {
                for (int i = 1; i <= 5; i++) {
                    oleds.invert_screen(i, (selected_menu_idx == (i - 1)));
                }
            } else if (current_mode == MODE_MENU_INFO) {
                // Destacar siempre "BACK" en la tecla 5
                for (int i = 1; i <= 5; i++) {
                    oleds.invert_screen(i, (i == 5));
                }
            } else if (current_mode == MODE_MENU_PAGES) {
                // La página en la que estás, en negativo: así se ve de un golpe
                // cuál es sin tener que leer los números.
                uint8_t count = profiles[active_profile_idx].page_count;
                for (int i = 1; i <= count; i++) {
                    oleds.invert_screen(i, (i - 1) == (int)active_page);
                }
            }
            current_inversions = 0; // Reiniciar estado local tras refresco
        } else if (current_mode == MODE_NORMAL) {
            uint16_t new_inversions = active_inversions;
            if (new_inversions != current_inversions) {
                for (int i = 1; i <= 10; i++) {
                    bool should_invert = (new_inversions & (1 << (i - 1))) != 0;
                    bool is_inverted = (current_inversions & (1 << (i - 1))) != 0;
                    if (should_invert != is_inverted) {
                        oleds.invert_screen(i, should_invert);
                    }
                }
                current_inversions = new_inversions;
            }
        }
        sleep_ms(10); // Polling ligero para no acaparar CPU si no hay mensajes
    }
}

// ==========================================
// CALLBACKS CDC (SERIAL) — Core 0
// ==========================================
#define CMD_BUF_SIZE 512
static char cmd_buf[CMD_BUF_SIZE];
static int  cmd_pos = 0;

void push_system_refresh() {
    system_refresh_req = true;
}

// Volcados como GET_PROFILE superan de largo el FIFO de TX del CDC, así que
// hay que ir drenándolo mientras se escribe o se pierden líneas.
static void cdc_write_all(const char* data, uint32_t len) {
    uint32_t sent = 0;
    int guard = 0;
    while (sent < len && guard < 4000) {
        uint32_t avail = tud_cdc_n_write_available(0);
        if (avail == 0) {
            tud_cdc_n_write_flush(0);
            tud_task();
            sleep_us(200);
            guard++;
            continue;
        }
        uint32_t chunk = (len - sent < avail) ? (len - sent) : avail;
        sent += tud_cdc_n_write(0, data + sent, chunk);
    }
    tud_cdc_n_write_flush(0);
}

static void cdc_printf(const char* fmt, ...) {
    // 256 bytes: los volcados de bitmap con GET_OLED llevan 180 caracteres hexadecimales.
    char line[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (len > 0) cdc_write_all(line, (uint32_t)(len < (int)sizeof(line) ? len : (int)sizeof(line) - 1));
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Extrae el siguiente campo entero separado por ':'. Devuelve NULL si falta.
static const char* next_field(const char* p, int* out) {
    if (!p) return NULL;
    char* end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) return NULL;
    *out = (int)v;
    return (*end == ':') ? end + 1 : end;
}

// Refleja la rueda tal y como está funcionando ahora mismo: perfil activo y
// capa que se esté pulsando.
static void send_scroll_state() {
    // El cuarto campo (alta resolución del paneo horizontal) va al final para no
    // romper a quien solo lea los tres primeros.
    cdc_printf("SCROLL:OK:%d:%d:%d:%d\n",
               (int)cur_scroll_detents(),
               (int)(cur_scroll_invert() ? 1 : 0),
               (int)(g_hires_multiplier != 0 ? 1 : 0),
               (int)(g_hires_pan != 0 ? 1 : 0));
}

static void send_profile_count() {
    cdc_printf("PROFILES:OK:%d:%d\n", (int)profile_count, (int)active_profile_idx);
}

// Página sobre la que actúan los comandos que no la especifican. La app antigua
// no sabe de páginas, así que edita la que está puesta si habla del perfil
// activo, y la primera de cualquier otro: es lo que ella cree que está editando.
static inline uint8_t cmd_page(uint8_t profile) {
    uint8_t pg = (profile == active_profile_idx) ? (uint8_t)active_page : 0;
    if (pg >= profiles[profile].page_count) pg = 0;
    return pg;
}

static inline Page& page_of(uint8_t profile, uint8_t pg) {
    return profiles[profile].pages[pg];
}

// Volcado de una página. Las líneas antiguas (PROF:<n>:LBL:...) se mantienen tal
// cual para la página que la app cree estar editando, y las nuevas llevan el
// número de página delante (PROF:<n>:P<pg>:LBL:...). Así una app que no conozca
// las páginas sigue funcionando: ignora las líneas que no entiende.
static void dump_profile_page(uint8_t idx, uint8_t pg, bool legacy) {
    const Page& p = page_of(idx, pg);
    char head[24];
    if (legacy) snprintf(head, sizeof(head), "PROF:%d", idx);
    else        snprintf(head, sizeof(head), "PROF:%d:P%d", idx, pg);

    for (int s = 0; s < 20; s++) {
        cdc_printf("%s:LBL:%d:%.*s\n", head, s, (int)sizeof(p.oled_labels[s]), p.oled_labels[s]);
    }
    for (int s = 0; s < 24; s++) {
        cdc_printf("%s:KEY:%d:%d:%d\n", head, s,
                   p.key_mappings[s].modifier, p.key_mappings[s].keycode);
    }
    for (int s = 0; s < ROT_SLOT_COUNT; s++) {
        cdc_printf("%s:ROT:%d:%d:%d:%d\n", head, s,
                   p.rotary[s].type, p.rotary[s].modifier, p.rotary[s].keycode);
    }
    for (int l = 0; l < 2; l++) {
        cdc_printf("%s:SCR:%d:%d:%d\n", head, l, p.scroll_detents[l], p.scroll_invert[l]);
    }
    cdc_printf("%s:OLEDMASK:%lu\n", head, (unsigned long)custom_oled_mask[idx][pg]);
}

static void dump_profile(uint8_t idx) {
    const Profile& p = profiles[idx];
    cdc_printf("PROF:%d:NAME:%.*s\n", idx, (int)sizeof(p.name), p.name);
    cdc_printf("PROF:%d:PAGES:%d:%d\n", idx, p.page_count, MAX_PAGES);

    // Primero la página «sin numerar», que es la que lee la app antigua.
    dump_profile_page(idx, cmd_page(idx), true);
    for (uint8_t pg = 0; pg < p.page_count; pg++) dump_profile_page(idx, pg, false);

    cdc_printf("PROF:%d:END\n", idx);
}

void process_command(const char* cmd) {
    // ---------- Descubrimiento y estado ----------
    if (strncmp(cmd, "ACK", 3) == 0) {
        // MAXPAGES lo añade el firmware 4.0; una app anterior lo ignora y una
        // app nueva puede usarlo para saber si este teclado tiene páginas.
        // MACROS=1 dice si este firmware sabe tocar secuencias él solo (los
        // comandos SET_MACRO_STEP/GET_MACRO/...); sin él, la app no debe
        // intentar subirlas o cada intento acabaría esperando una respuesta
        // que nunca llega.
        cdc_printf("ORBY_V4:FW=4.0:KEYS=12:OLEDS=10:ENCODERS=2:PROFILES=%d:MAXPROFILES=%d:MAXPAGES=%d:MACROS=1:MODE=%s\n",
                   (int)profile_count, MAX_PROFILES, MAX_PAGES,
                   (current_mode == MODE_NORMAL) ? "NORMAL" : "MENU");
        return;
    }

    if (strcmp(cmd, "GET_STATE") == 0) {
        cdc_printf("STATE:PROFILE:%d\n", (int)active_profile_idx);
        cdc_printf("STATE:PROFILES:%d:%d\n", (int)profile_count, MAX_PROFILES);
        cdc_printf("STATE:TIMEOUT:%d\n", (int)reposo_timeout_min);
        cdc_printf("STATE:MODE:%s\n", (current_mode == MODE_NORMAL) ? "NORMAL" : "MENU");
        cdc_printf("STATE:SUPER:%d\n", super_active ? 1 : 0);
        cdc_printf("STATE:PAGE:%d:%d:%d\n", (int)active_page,
                   (int)profiles[active_profile_idx].page_count, MAX_PAGES);
        send_scroll_state();
        cdc_printf("STATE:END\n");
        return;
    }

    // ---------- Ajustes básicos ----------
    if (strncmp(cmd, "SET_PROFILE:", 12) == 0) {
        int profile_idx = atoi(cmd + 12);
        if (profile_idx >= 0 && profile_idx < (int)profile_count) {
            active_profile_idx = profile_idx;
            active_page = 0; // el perfil nuevo puede tener menos páginas
            push_system_refresh();
            cdc_printf("PROFILE:OK:%d\n", (int)active_profile_idx);
        } else {
            cdc_printf("ERR:PROFILE_RANGE\n");
        }
        return;
    }

    if (strncmp(cmd, "SET_TIMEOUT:", 12) == 0) {
        int val = atoi(cmd + 12);
        if (val == 1 || val == 5 || val == 10 || val == 0) {
            reposo_timeout_min = val;
            cdc_printf("TIMEOUT:OK:%d\n", (int)reposo_timeout_min);
        } else {
            cdc_printf("ERR:TIMEOUT_VALUE\n");
        }
        return;
    }

    // ---------- Calibración de la rueda de scroll ----------
    // La sensibilidad es un ajuste del perfil y de la capa, así que las formas
    // cortas actúan sobre lo que está activo en ese momento.
    if (strncmp(cmd, "SET_SCROLL_INV:", 15) == 0) {
        cur_page().scroll_invert[super_active ? 1 : 0] = (atoi(cmd + 15) != 0) ? 1 : 0;
        send_scroll_state();
        return;
    }

    if (strncmp(cmd, "SET_SCROLL:", 11) == 0) {
        int val = atoi(cmd + 11);
        if (val >= SCROLL_DETENTS_MIN && val <= SCROLL_DETENTS_MAX) {
            cur_page().scroll_detents[super_active ? 1 : 0] = (uint8_t)val;
            send_scroll_state();
        } else {
            cdc_printf("ERR:SCROLL_RANGE:%d:%d\n", SCROLL_DETENTS_MIN, SCROLL_DETENTS_MAX);
        }
        return;
    }

    if (strcmp(cmd, "GET_SCROLL") == 0) {
        send_scroll_state();
        return;
    }

    // ---------- Páginas ----------
    // Los comandos de edición que no llevan número de página actúan sobre la que
    // esté puesta (ver cmd_page), así que para editar otra basta con cambiarla
    // antes con SET_PAGE.
    if (strncmp(cmd, "SET_PAGE:", 9) == 0) {
        int pg = atoi(cmd + 9);
        if (pg < 0 || pg >= (int)profiles[active_profile_idx].page_count) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        set_active_page((uint8_t)pg);
        cdc_printf("PAGE:OK:%d:%d\n", (int)active_page,
                   (int)profiles[active_profile_idx].page_count);
        return;
    }

    // ADD_PAGE:<perfil>[:<copiar 0-1>]  — por defecto copia la página actual
    if (strncmp(cmd, "ADD_PAGE:", 9) == 0) {
        int idx = 0, copy = 1;
        const char* p = next_field(cmd + 9, &idx);
        if (p) next_field(p, &copy);
        if (idx < 0 || idx >= (int)profile_count) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        if (!page_add((uint8_t)idx, copy != 0)) {
            cdc_printf("ERR:PAGE_LIMIT:%d\n", MAX_PAGES);
            return;
        }
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("PAGE:ADDED:%d:%d\n", idx, (int)profiles[idx].page_count);
        return;
    }

    // DEL_PAGE:<perfil>:<página>
    if (strncmp(cmd, "DEL_PAGE:", 9) == 0) {
        int idx = 0, pg = 0;
        const char* p = next_field(cmd + 9, &idx);
        p = next_field(p, &pg);
        if (!p || idx < 0 || idx >= (int)profile_count) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        if (!page_remove((uint8_t)idx, (uint8_t)pg)) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("PAGE:DELETED:%d:%d:%d\n", idx, pg, (int)profiles[idx].page_count);
        return;
    }

    // SET_PSCROLL:<perfil>:<capa 0-1>:<detents>:<invertir 0-1>
    if (strncmp(cmd, "SET_PSCROLL:", 12) == 0) {
        int idx = 0, layer = 0, det = 0, inv = 0;
        const char* p = next_field(cmd + 12, &idx);
        p = next_field(p, &layer);
        p = next_field(p, &det);
        p = next_field(p, &inv);
        if (!p || idx < 0 || idx >= (int)profile_count || layer < 0 || layer > 1 ||
            det < SCROLL_DETENTS_MIN || det > SCROLL_DETENTS_MAX) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        page_of((uint8_t)idx, cmd_page((uint8_t)idx)).scroll_detents[layer] = (uint8_t)det;
        page_of((uint8_t)idx, cmd_page((uint8_t)idx)).scroll_invert[layer]  = inv ? 1 : 0;
        cdc_printf("PSCROLL:OK:%d:%d:%d:%d\n", idx, layer, det, inv ? 1 : 0);
        return;
    }

    // ---------- Lectura y edición de perfiles ----------
    if (strncmp(cmd, "GET_PROFILE:", 12) == 0) {
        int idx = atoi(cmd + 12);
        if (idx >= 0 && idx < (int)profile_count) dump_profile((uint8_t)idx);
        else cdc_printf("ERR:PROFILE_RANGE\n");
        return;
    }

    if (strcmp(cmd, "GET_PROFILES") == 0) {
        send_profile_count();
        for (int i = 0; i < (int)profile_count; i++) dump_profile((uint8_t)i);
        return;
    }

    if (strcmp(cmd, "GET_PROFILE_COUNT") == 0) {
        send_profile_count();
        return;
    }

    // ---------- Crear, duplicar y borrar perfiles ----------
    if (strcmp(cmd, "ADD_PROFILE") == 0 || strncmp(cmd, "DUP_PROFILE:", 12) == 0) {
        const bool duplicating = (cmd[0] == 'D');
        int from = duplicating ? atoi(cmd + 12) : -1;
        if (duplicating && (from < 0 || from >= (int)profile_count)) {
            cdc_printf("ERR:PROFILE_RANGE\n");
            return;
        }

        int idx = profile_add(from);
        if (idx < 0) { cdc_printf("ERR:PROFILE_FULL:%d\n", MAX_PROFILES); return; }
        cdc_printf("PROFILE:ADDED:%d\n", idx);
        send_profile_count();
        return;
    }

    if (strncmp(cmd, "DEL_PROFILE:", 12) == 0) {
        int idx = atoi(cmd + 12);
        if (idx < 0 || idx >= (int)profile_count) { cdc_printf("ERR:PROFILE_RANGE\n"); return; }
        if (!profile_remove((uint8_t)idx)) { cdc_printf("ERR:PROFILE_LAST\n"); return; }
        push_system_refresh();
        cdc_printf("PROFILE:DELETED:%d\n", idx);
        send_profile_count();
        return;
    }

    // SET_NAME:<perfil>:<texto>
    if (strncmp(cmd, "SET_NAME:", 9) == 0) {
        int idx = 0;
        const char* p = next_field(cmd + 9, &idx);
        if (!p || idx < 0 || idx >= (int)profile_count) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        memset(profiles[idx].name, 0, sizeof(profiles[idx].name));
        strncpy(profiles[idx].name, p, sizeof(profiles[idx].name) - 1);
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("NAME:OK:%d\n", idx);
        return;
    }

    // SET_LABEL:<perfil>:<hueco 0-19>:<texto>
    if (strncmp(cmd, "SET_LABEL:", 10) == 0) {
        int idx = 0, slot = 0;
        const char* p = next_field(cmd + 10, &idx);
        p = next_field(p, &slot);
        if (!p || idx < 0 || idx >= (int)profile_count || slot < 0 || slot > 19) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        Page& pg = page_of((uint8_t)idx, cmd_page((uint8_t)idx));
        memset(pg.oled_labels[slot], 0, sizeof(pg.oled_labels[slot]));
        strncpy(pg.oled_labels[slot], p, sizeof(pg.oled_labels[slot]) - 1);
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("LABEL:OK:%d:%d\n", idx, slot);
        return;
    }

    // SET_KEYMAP:<perfil>:<hueco 0-23>:<modificador>:<keycode>
    // modificador 0xFE (254) = tecla de consumo multimedia, keycode = índice.
    if (strncmp(cmd, "SET_KEYMAP:", 11) == 0) {
        int idx = 0, slot = 0, mod = 0, key = 0;
        const char* p = next_field(cmd + 11, &idx);
        p = next_field(p, &slot);
        p = next_field(p, &mod);
        p = next_field(p, &key);
        if (!p || idx < 0 || idx >= (int)profile_count || slot < 0 || slot > 23 ||
            mod < 0 || mod > 255 || key < 0 || key > 255) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        Page& pg = page_of((uint8_t)idx, cmd_page((uint8_t)idx));
        pg.key_mappings[slot].modifier = (uint8_t)mod;
        pg.key_mappings[slot].keycode  = (uint8_t)key;
        cdc_printf("KEYMAP:OK:%d:%d\n", idx, slot);
        return;
    }

    // SET_ROTARY:<perfil>:<hueco 0-7>:<tipo>:<modificador>:<keycode>
    // Huecos: 0-2 encoder izquierdo (horario, antihorario, clic),
    //         3-5 encoder derecho, 6-7 rueda de scroll.
    if (strncmp(cmd, "SET_ROTARY:", 11) == 0) {
        int idx = 0, slot = 0, type = 0, mod = 0, key = 0;
        const char* p = next_field(cmd + 11, &idx);
        p = next_field(p, &slot);
        p = next_field(p, &type);
        p = next_field(p, &mod);
        p = next_field(p, &key);
        if (!p || idx < 0 || idx >= (int)profile_count || slot < 0 || slot >= ROT_SLOT_COUNT ||
            type < 0 || type > ROT_ZOOM || mod < 0 || mod > 255 || key < 0 || key > 255) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        page_of((uint8_t)idx, cmd_page((uint8_t)idx)).rotary[slot] = { (uint8_t)type, (uint8_t)mod, (uint8_t)key };
        cdc_printf("ROTARY:OK:%d:%d\n", idx, slot);
        return;
    }

    // ---------- Secuencias (macros) que ejecuta el propio teclado ----------
    // SET_MACRO_STEP:<id 0-63>:<paso 0-23>:<tipo>:<a>:<b>
    // Escribe un paso y, si extiende la secuencia, sube el recuento a la vez;
    // para acortarla hace falta MACRO_TRUNC.
    if (strncmp(cmd, "SET_MACRO_STEP:", 15) == 0) {
        int id = 0, step = 0, type = 0, a = 0, b = 0;
        const char* p = next_field(cmd + 15, &id);
        p = next_field(p, &step);
        p = next_field(p, &type);
        p = next_field(p, &a);
        p = next_field(p, &b);
        if (!p || id < 0 || id >= MACRO_MAX_COUNT || step < 0 || step >= MACRO_MAX_STEPS ||
            type < 0 || type > 255 || a < -32768 || a > 32767 || b < -32768 || b > 32767) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        macros[id].steps[step] = { (uint8_t)type, (int16_t)a, (int16_t)b };
        if (step >= macros[id].step_count) macros[id].step_count = (uint8_t)(step + 1);
        cdc_printf("MACRO:OK:%d:%d\n", id, step);
        return;
    }

    // MACRO_TRUNC:<id>:<pasos>  — fija el recuento exacto (recorta lo que sobre)
    if (strncmp(cmd, "MACRO_TRUNC:", 12) == 0) {
        int id = 0, count = 0;
        const char* p = next_field(cmd + 12, &id);
        p = next_field(p, &count);
        if (!p || id < 0 || id >= MACRO_MAX_COUNT || count < 0 || count > MACRO_MAX_STEPS) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        macros[id].step_count = (uint8_t)count;
        cdc_printf("MACRO:TRUNC:OK:%d:%d\n", id, count);
        return;
    }

    // MACRO_CLEAR:<id>  — vacía la copia del teclado (la macro vuelve a
    // ejecutarla el PC hasta que se le suba una jugable de nuevo)
    if (strncmp(cmd, "MACRO_CLEAR:", 12) == 0) {
        int id = atoi(cmd + 12);
        if (id < 0 || id >= MACRO_MAX_COUNT) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        memset(&macros[id], 0, sizeof(macros[id]));
        cdc_printf("MACRO:CLEARED:%d\n", id);
        return;
    }

    // GET_MACRO:<id>  — vuelca los pasos guardados en el teclado para esa macro
    if (strncmp(cmd, "GET_MACRO:", 10) == 0) {
        int id = atoi(cmd + 10);
        if (id < 0 || id >= MACRO_MAX_COUNT) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        const FlashMacro& m = macros[id];
        for (uint8_t i = 0; i < m.step_count; i++) {
            cdc_printf("MACRO:%d:STEP:%d:%d:%d:%d\n", id, (int)i,
                       (int)m.steps[i].type, (int)m.steps[i].a, (int)m.steps[i].b);
        }
        cdc_printf("MACRO:%d:END\n", id);
        return;
    }

    // MACRO_TEST:<id>  — SOLO PARA DEPURAR. Fuerza la reproducción en el
    // propio teclado aunque la app esté conectada (que si no, siempre gana
    // el camino del PC y no habría forma de ver esta telemetría), y manda
    // por CDC cada trozo de movimiento que calcula de verdad.
    if (strncmp(cmd, "MACRO_TEST:", 11) == 0) {
        int id = atoi(cmd + 11);
        if (id < 0 || id >= MACRO_MAX_COUNT) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        if (macro_player.active) { cdc_printf("ERR:MACRO_BUSY\n"); return; }
        macro_player_start((uint8_t)id);
        macro_player.debug = true;
        cdc_printf("MACRO:TEST:STARTED:%d\n", id);
        return;
    }

    // ---------- Iconos OLED personalizados ----------
    // OLED_CHUNK:<perfil>:<hueco 0-19>:<offset>:<hex>
    // El framebuffer es de 360 bytes (72x40, 5 páginas). La app lo trocea.
    if (strncmp(cmd, "OLED_CHUNK:", 11) == 0) {
        int idx = 0, slot = 0, offset = 0;
        const char* p = next_field(cmd + 11, &idx);
        p = next_field(p, &slot);
        p = next_field(p, &offset);
        if (!p || idx < 0 || idx >= (int)profile_count || slot < 0 || slot > 19 ||
            offset < 0 || offset >= OLED_FB_SIZE) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }

        // Los iconos viven en la Flash, así que el trozo entra en el búfer de
        // preparación de esa página. Se graba al guardar o al cambiar de página.
        uint8_t page = cmd_page((uint8_t)idx);
        load_oled_staging((uint8_t)idx, page);
        uint8_t* dst = &oled_staging[slot * OLED_FB_SIZE];
        int written = 0;
        while (p[0] && p[1] && (offset + written) < OLED_FB_SIZE) {
            int hi = hex_nibble(p[0]);
            int lo = hex_nibble(p[1]);
            if (hi < 0 || lo < 0) break;
            dst[offset + written] = (uint8_t)((hi << 4) | lo);
            written++;
            p += 2;
        }

        custom_oled_mask[idx][page] |= (1u << slot);
        staging_dirty = true;
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("OLED:OK:%d:%d:%d:%d\n", idx, slot, offset, written);
        return;
    }

    // GET_OLED:<perfil>:<hueco 0-19>
    // Devuelve el bitmap guardado para que la app pueda dibujar la previsualización
    // real de cada tecla en lugar de fiarse de una caché local.
    if (strncmp(cmd, "GET_OLED:", 9) == 0) {
        int idx = 0, slot = 0;
        const char* p = next_field(cmd + 9, &idx);
        p = next_field(p, &slot);
        if (!p || idx < 0 || idx >= (int)profile_count || slot < 0 || slot > 19) { cdc_printf("ERR:BAD_ARGS\n"); return; }

        uint8_t page = cmd_page((uint8_t)idx);
        if (!oled_slot_used((uint8_t)idx, page, (uint8_t)slot)) {
            cdc_printf("OLEDDATA:%d:%d:NONE\n", idx, slot);
            return;
        }

        static const char HEXCHARS[] = "0123456789abcdef";
        const uint8_t* src = oled_slot_ptr((uint8_t)idx, page, (uint8_t)slot);
        char hex[181];

        for (int off = 0; off < OLED_FB_SIZE; off += 90) {
            int n = (OLED_FB_SIZE - off < 90) ? (OLED_FB_SIZE - off) : 90;
            for (int i = 0; i < n; i++) {
                hex[i * 2]     = HEXCHARS[src[off + i] >> 4];
                hex[i * 2 + 1] = HEXCHARS[src[off + i] & 0x0F];
            }
            hex[n * 2] = 0;
            cdc_printf("OLEDDATA:%d:%d:%d:%s\n", idx, slot, off, hex);
        }
        cdc_printf("OLEDDATA:%d:%d:END\n", idx, slot);
        return;
    }

    // OLED_CLEAR:<perfil>:<hueco>  (hueco 255 = borrar todos los del perfil)
    if (strncmp(cmd, "OLED_CLEAR:", 11) == 0) {
        int idx = 0, slot = 0;
        const char* p = next_field(cmd + 11, &idx);
        p = next_field(p, &slot);
        if (idx < 0 || idx >= (int)profile_count) { cdc_printf("ERR:BAD_ARGS\n"); return; }

        uint8_t page = cmd_page((uint8_t)idx);
        load_oled_staging((uint8_t)idx, page);
        if (slot == 255) {
            custom_oled_mask[idx][page] = 0;
            memset(oled_staging, 0, OLED_SLOTS * OLED_FB_SIZE);
        } else if (slot >= 0 && slot <= 19) {
            custom_oled_mask[idx][page] &= ~(1u << slot);
            memset(&oled_staging[slot * OLED_FB_SIZE], 0, OLED_FB_SIZE);
        } else {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        staging_dirty = true;
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("OLED:CLEARED:%d:%d\n", idx, slot);
        return;
    }

    // ---------- Persistencia ----------
    if (strcmp(cmd, "SAVE_STATE") == 0) {
        save_settings();
        save_macros();
        cdc_printf("SAVE:OK\n");
        return;
    }

    if (strcmp(cmd, "RESET_DEFAULTS") == 0) {
        load_defaults();
        push_system_refresh();
        cdc_printf("RESET:OK\n");
        return;
    }

    cdc_printf("ERR:UNKNOWN_CMD\n");
}

// Función callback CDC eliminada; se realiza polling manual en main()

// ==========================================
// CICLO DE VIDA DEL USB
// ==========================================
// Si el PC se suspende o se tira del cable con una tecla hundida, el host se
// queda con esa tecla pulsada para siempre. Soltar todo en estos avisos es lo
// que evita el clásico «se me ha quedado el Ctrl pegado».
extern "C" void tud_umount_cb(void) {
    HidOut::release_all();
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    HidOut::release_all();
}

extern "C" void tud_resume_cb(void) {
    // A propósito no se reenvía nada: si al despertar el PC siguiera hundida
    // una tecla, reenviarla escribiría un carácter que nadie ha pedido. Vuelve
    // a contar en cuanto se suelte y se pulse otra vez.
}

// ==========================================
// CORE 0: ENCODERS, TECLAS Y USB CDC / HID
// ==========================================
int main() {
    board_init();
    tusb_init();
    HidOut::init();

    // Margen para que se asienten las alimentaciones antes de arrancar el resto.
    // Antes eran dos segundos de sleep_ms() a secas, sin llamar ni una vez a
    // tud_task(): el dispositivo estaba en el bus pero no contestaba a la
    // enumeración, lo que la retrasaba y de vez en cuando la hacía fallar.
    // Ahora se atiende al USB mientras se espera, y basta con mucho menos.
    #define BOOT_SETTLE_MS 300
    absolute_time_t settle = make_timeout_time_ms(BOOT_SETTLE_MS);
    while (!time_reached(settle)) {
        tud_task();
        sleep_us(500);
    }

    // Cargar configuración de Flash
    load_settings();
    load_macros();

    // Si venimos de un reinicio por watchdog, algo se colgó: nada de intro, a
    // dejar el teclado operativo cuanto antes.
    if (watchdog_caused_reboot()) intro_skip_req = true;

    multicore_launch_core1(core1_entry);

    // Ocho segundos, que es el tope del RP2040. Lo alimenta el bucle de abajo
    // solo si Core 1 sigue dando señales de vida: así un cuelgue de cualquiera
    // de los dos cores acaba en un reinicio en vez de en un teclado muerto que
    // hay que desenchufar.
    watchdog_enable(8000, 1);

    HardwareEncoder rueda_izq(pio0, Pins::ENC1_A, 1);
    HardwareEncoder rueda_der(pio0, Pins::ENC2_A, -1);

    HardwareAs5600 wheel;
    wheel.init();

    const uint8_t key_pins[12] = {
        Pins::KEY_1, Pins::KEY_2, Pins::KEY_3, Pins::KEY_4,
        Pins::KEY_5, Pins::KEY_6, Pins::KEY_7, Pins::KEY_8,
        Pins::KEY_9, Pins::KEY_10, Pins::KEY_11, Pins::KEY_12
    };
    bool last_key_state[12] = {false};

    // Antirrebote de las doce teclas. Los pulsadores de los encoders ya tenían
    // el suyo, pero las teclas se leían en crudo: un pulsador mecánico rebota
    // entre 1 y 5 ms y el bucle gira cada 250 us, así que un solo toque podía
    // colar hasta veinte flancos y repetir el atajo.
    //
    // Se acepta el primer flanco al instante (no añade ni un microsegundo de
    // latencia) y se ignora lo que llegue durante los 5 ms siguientes.
    #define KEY_DEBOUNCE_US 5000
    bool     key_stable[12]  = {false};
    uint32_t key_edge_us[12] = {0};

    const uint8_t key_to_oled[12] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,  // Teclas 1-9 → OLED 1-9
        0,   // Tecla 10 → sin pantalla
        10,  // Tecla 11 → OLED 10
        0    // Tecla 12 → sin pantalla
    };

    for (int i = 0; i < 12; i++) {
        gpio_init(key_pins[i]);
        gpio_set_dir(key_pins[i], GPIO_IN);
        gpio_pull_up(key_pins[i]);
    }

    gpio_init(Pins::ENC1_SW);
    gpio_set_dir(Pins::ENC1_SW, GPIO_IN);
    gpio_pull_up(Pins::ENC1_SW);

    gpio_init(Pins::ENC2_SW);
    gpio_set_dir(Pins::ENC2_SW, GPIO_IN);
    gpio_pull_up(Pins::ENC2_SW);

    int enc1_sw_debounce_cnt = 0;
    bool stable_enc1_sw_state = false;
    int enc2_sw_debounce_cnt = 0;
    bool stable_enc2_sw_state = false;

    // Control de inactividad / reposo
    uint32_t last_activity_time = to_ms_since_boot(get_absolute_time());
    bool is_sleeping = false;

    // Telemetría de la rueda, agregada a 20 Hz
    int32_t  wheel_tel_accum = 0;
    uint32_t last_wheel_tel  = last_activity_time;

    // Aquí había una segunda instancia de HardwareOled para despertar las
    // pantallas. Se ha quitado: su constructor reinicializaba los pines y el
    // SPI que Core 1 está usando, y dejaba OLED_RST a nivel bajo. El reposo se
    // pide ahora con display_power_req.

    uint32_t last_hb = 0, last_hb_ms = to_ms_since_boot(get_absolute_time());

    while (true) {
        tud_task();

        // Alimentar el watchdog, pero solo si Core 1 sigue vivo. Durante la
        // intro no se le exige latido porque está dentro del reproductor.
        {
            uint32_t ms = to_ms_since_boot(get_absolute_time());
            if (core1_heartbeat != last_hb) { last_hb = core1_heartbeat; last_hb_ms = ms; }
            if (intro_running || (uint32_t)(ms - last_hb_ms) < 3000) watchdog_update();
        }

        // Quien tenga prisa corta la intro con cualquier tecla o pulsador.
        if (intro_running && !intro_skip_req) {
            for (int i = 0; i < 12; i++) {
                if (!gpio_get(key_pins[i])) { intro_skip_req = true; break; }
            }
            if (!gpio_get(Pins::ENC1_SW) || !gpio_get(Pins::ENC2_SW)) intro_skip_req = true;
        }

        // --- PROCESAR COMANDOS CDC (POLLING) ---
        if (tud_cdc_available()) {
            char tmp[256];
            uint32_t count = tud_cdc_read(tmp, sizeof(tmp));
            for (uint32_t i = 0; i < count; i++) {
                char c = tmp[i];
                if (c == '\n' || c == '\r') {
                    if (cmd_pos > 0) {
                        cmd_buf[cmd_pos] = 0;
                        process_command(cmd_buf);
                        cmd_pos = 0;
                    }
                } else if (cmd_pos < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_pos++] = c;
                }
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        bool activity_detected = false;

        // --- ENCODER IZQUIERDO (VOLUMEN) ---
        int delta_izq = rueda_izq.get_delta();
        if (delta_izq != 0) {
            activity_detected = true;
            
            if (current_mode == MODE_NORMAL) {
                // Telemetría serie
                char tel[32];
                int len = snprintf(tel, sizeof(tel), "ENC:1:%d\n", delta_izq);
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);

                // Acción configurada en el perfil activo y la capa en curso
                const Page& p = cur_page();
                apply_rotary(p.rotary[rot_slot(ROT_ENC1_CW,  super_active)],
                             p.rotary[rot_slot(ROT_ENC1_CCW, super_active)], delta_izq);
            } else if (current_mode == MODE_MENU_MAIN || current_mode == MODE_MENU_PERF || current_mode == MODE_MENU_REPO) {
                // Desplazamiento de menú
                int opts = (current_mode == MODE_MENU_PERF) ? perf_menu_count()
                         : (current_mode == MODE_MENU_MAIN) ? 4 : 5;
                int next = ((int)selected_menu_idx + delta_izq) % opts;
                if (next < 0) next += opts;
                selected_menu_idx = (uint8_t)next;
                if (current_mode == MODE_MENU_PERF) perf_menu_follow();
                push_system_refresh();
            }
        }

        // --- ENCODER DERECHO (BRILLO) ---
        int delta_der = rueda_der.get_delta();
        if (delta_der != 0) {
            activity_detected = true;

            if (current_mode == MODE_NORMAL) {
                // Telemetría serie
                char tel[32];
                int len = snprintf(tel, sizeof(tel), "ENC:2:%d\n", delta_der);
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);

                // Acción configurada en el perfil activo y la capa en curso
                const Page& p = cur_page();
                apply_rotary(p.rotary[rot_slot(ROT_ENC2_CW,  super_active)],
                             p.rotary[rot_slot(ROT_ENC2_CCW, super_active)], delta_der);
            }
        }

        // --- BOTONES DE LOS ENCODERS (DEBOUNCED) ---
        bool raw_enc1_sw = !gpio_get(Pins::ENC1_SW);
        if (raw_enc1_sw != stable_enc1_sw_state) {
            enc1_sw_debounce_cnt++;
            if (enc1_sw_debounce_cnt >= 15) {
                stable_enc1_sw_state = raw_enc1_sw;
                enc1_sw_debounce_cnt = 0;
                activity_detected = true;
                
                // Telemetría serie
                char tel[32];
                int len = snprintf(tel, sizeof(tel), "ENC_SW:1:%d\n", stable_enc1_sw_state ? 1 : 0);
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);

                if (stable_enc1_sw_state) {
                    if (current_mode == MODE_NORMAL) {
                        emit_discrete(cur_page().rotary[rot_slot(ROT_ENC1_CLICK, super_active)]);
                    } else if (current_mode == MODE_MENU_MAIN) {
                        // Confirmar opción
                        if (selected_menu_idx == 0) current_mode = MODE_MENU_PERF;
                        else if (selected_menu_idx == 1) current_mode = MODE_MENU_REPO;
                        else if (selected_menu_idx == 2) current_mode = MODE_MENU_INFO;
                        else if (selected_menu_idx == 3) {
                            current_mode = MODE_NORMAL;
                            char tel2[24];
                            int len2 = snprintf(tel2, sizeof(tel2), "MODE:NORMAL\n");
                            tud_cdc_n_write(0, tel2, len2);
                            tud_cdc_n_write_flush(0);
                        }
                        selected_menu_idx = 0;
                        if (current_mode == MODE_MENU_PERF) perf_menu_open();
                        push_system_refresh();
                    } else if (current_mode == MODE_MENU_PERF) {
                    if (selected_menu_idx < profile_count) {
                        active_profile_idx = selected_menu_idx;
                        active_page = 0;
                        // Guardado inmediato: es un cambio discreto (no como el
                        // encoder de la rueda), así que no hay riesgo de desgastar
                        // la Flash a base de escrituras seguidas.
                        save_settings();
                        current_mode = MODE_NORMAL;
                        char tel2[24];
                        int len2 = snprintf(tel2, sizeof(tel2), "MODE:NORMAL\n");
                        tud_cdc_n_write(0, tel2, len2);
                        tud_cdc_n_write_flush(0);
                        } else {
                            current_mode = MODE_MENU_MAIN;
                        }
                        selected_menu_idx = 0;
                        push_system_refresh();
                    } else if (current_mode == MODE_MENU_REPO) {
                        if (selected_menu_idx >= 0 && selected_menu_idx <= 3) {
                            const uint8_t repo_vals[4] = {1, 5, 10, 0};
                            reposo_timeout_min = repo_vals[selected_menu_idx];
                            save_settings();
                        }
                        current_mode = MODE_MENU_MAIN;
                        selected_menu_idx = 1;
                        push_system_refresh();
                    } else if (current_mode == MODE_MENU_INFO) {
                        current_mode = MODE_MENU_MAIN;
                        selected_menu_idx = 2;
                        push_system_refresh();
                    }
                }
            }
        } else {
            enc1_sw_debounce_cnt = 0;
        }

        bool raw_enc2_sw = !gpio_get(Pins::ENC2_SW);
        if (raw_enc2_sw != stable_enc2_sw_state) {
            enc2_sw_debounce_cnt++;
            if (enc2_sw_debounce_cnt >= 15) {
                stable_enc2_sw_state = raw_enc2_sw;
                enc2_sw_debounce_cnt = 0;
                activity_detected = true;

                // Telemetría serie
                char tel[32];
                int len = snprintf(tel, sizeof(tel), "ENC_SW:2:%d\n", stable_enc2_sw_state ? 1 : 0);
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);

                if (stable_enc2_sw_state && current_mode == MODE_NORMAL) {
                    emit_discrete(cur_page().rotary[rot_slot(ROT_ENC2_CLICK, super_active)]);
                }
            }
        } else {
            enc2_sw_debounce_cnt = 0;
        }

        // --- DETECTAR ENTRADA AL MENÚ (HOLD 2 SEGUNDOS EN AMBOS ENCODERS) ---
        static uint32_t hold_start_time = 0;
        if (stable_enc1_sw_state && stable_enc2_sw_state) {
            if (hold_start_time == 0) {
                hold_start_time = now;
            } else if (now - hold_start_time >= 2000) {
                activity_detected = true;
                if (current_mode == MODE_NORMAL) {
                    current_mode = MODE_MENU_MAIN;
                    selected_menu_idx = 0;
                    push_system_refresh();
                    
                    // Notificar por CDC
                    char tel[24];
                    int len = snprintf(tel, sizeof(tel), "MODE:MENU\n");
                    tud_cdc_n_write(0, tel, len);
                    tud_cdc_n_write_flush(0);
                } else {
                    current_mode = MODE_NORMAL;
                    push_system_refresh();
                    
                    char tel[24];
                    int len = snprintf(tel, sizeof(tel), "MODE:NORMAL\n");
                    tud_cdc_n_write(0, tel, len);
                    tud_cdc_n_write_flush(0);
                }
                hold_start_time = 0;
                // Esperar liberación completa de botones para evitar dobles clics
                while (!gpio_get(Pins::ENC1_SW) || !gpio_get(Pins::ENC2_SW)) {
                    tud_task();
                    watchdog_update(); // el usuario puede tenerlo apretado un buen rato
                    sleep_ms(10);
                }
            }
        } else {
            hold_start_time = 0;
        }

        // --- RUEDA DE SCROLL AS5600 ---
        int32_t wheel_delta = 0;
        if (wheel.poll(wheel_delta) && wheel_delta != 0) {
            activity_detected = true;

            if (current_mode == MODE_NORMAL) {
                wheel_accumulate(wheel_delta);
            }
            // Telemetría agregada: la rueda emite a 250 Hz y volcar una línea
            // por bloque saturaba el CDC e introducía jitter en el scroll.
            wheel_tel_accum += wheel_delta;
        }
        // La rueda tiene preferencia sobre el endpoint: es lo que más se nota
        // si se retrasa. Después va lo que haya pendiente de teclado.
        WheelOut::flush();
        HidOut::pump();
        macro_player_tick(now);

        // Soltar el Ctrl del zoom cuando se deja de girar. Hay que abrir también
        // la compuerta: si quedaran unidades a medias, se quedaría esperando un
        // Ctrl que ya no va a volver y la rueda enmudecería para siempre.
        if (zoom_hold_until != 0 && (int32_t)(now - zoom_hold_until) >= 0) {
            zoom_hold_until = 0;
            HidOut::set_sticky_mod(0);
            WheelOut::zoom_disarm();
        }

        if (wheel_tel_accum != 0 && (uint32_t)(now - last_wheel_tel) >= 50) {
            last_wheel_tel = now;
            // Además del incremento va el ángulo absoluto (0-4095): es lo que
            // permite a la app dibujar la rueda en su posición real. El campo
            // se añade al final para no romper a quien solo lea el incremento.
            char tel[40];
            int len = snprintf(tel, sizeof(tel), "WHEEL:%ld:%u\n",
                               (long)wheel_tel_accum, (unsigned)wheel.rawAngle());
            wheel_tel_accum = 0;
            tud_cdc_n_write(0, tel, len);
            tud_cdc_n_write_flush(0);
        }

        // --- LECTURA ANTIRREBOTE DE LAS DOCE TECLAS ---
        // Todo lo que viene detrás (SUPER, el menú y los atajos) lee de
        // aquí, nunca del pin en crudo.
        {
            uint32_t now_us = time_us_32();
            for (int i = 0; i < 12; i++) {
                bool raw = !gpio_get(key_pins[i]);
                if (raw != key_stable[i] &&
                    (uint32_t)(now_us - key_edge_us[i]) >= KEY_DEBOUNCE_US) {
                    key_stable[i]  = raw;
                    key_edge_us[i] = now_us;
                }
            }
        }

        // --- TECLA DE MENÚ: CORTA CAMBIA DE PÁGINA, LARGA ABRE LOS PERFILES ---
        bool menu_key_pressed = key_stable[KEY_IDX_MENU];
        static uint32_t menu_hold_start = 0;
        static bool     menu_long_fired = false;
        if (menu_key_pressed && current_mode == MODE_NORMAL) {
            if (menu_hold_start == 0) {
                menu_hold_start = now;
                menu_long_fired = false;
            } else if (!menu_long_fired && now - menu_hold_start >= 1000) {
                menu_long_fired = true;
                activity_detected = true;
                current_mode = MODE_MENU_PERF;
                perf_menu_open();
                push_system_refresh();
                
                // Notificar por CDC
                char tel[24];
                int len = snprintf(tel, sizeof(tel), "MODE:MENU\n");
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);
                
                // Esperar a que se libere para no disparar atajos accidentales
                while (!gpio_get(key_pins[KEY_IDX_MENU])) {
                    tud_task();
                    watchdog_update(); // el usuario puede tenerla apretada un buen rato
                    sleep_ms(10);
                }
                menu_hold_start = 0;
            }
        } else {
            // Al soltar: si no llegó al segundo, era una pulsación corta y toca
            // pasar de página. El menú largo ya se ha encargado de lo suyo.
            if (menu_hold_start != 0 && !menu_long_fired && current_mode == MODE_NORMAL) {
                activity_detected = true;
                next_page();
            }
            menu_hold_start = 0;
            menu_long_fired = false;
        }

        // --- TECLAS ---
        // Primero, actualizar el estado de la tecla SUPER
        bool super_pressed = key_stable[KEY_IDX_SUPER];
        if (super_pressed != super_active) {
            super_active = super_pressed;
            if (current_mode == MODE_NORMAL) {
                push_system_refresh();
            }

            // Telemetría serial
            char tel[24];
            int len = snprintf(tel, sizeof(tel), "KEY_EV:%d:%d\n",
                               KEY_IDX_SUPER + 1, super_active ? 1 : 0);
            tud_cdc_n_write(0, tel, len);
            tud_cdc_n_write_flush(0);
        }

        for (int i = 0; i < 12; i++) {
            // La tecla SUPER se maneja por separado, arriba
            if (i == KEY_IDX_SUPER) continue;

            // La tecla de menú, en modo normal, solo responde al hold largo
            if (i == KEY_IDX_MENU && current_mode == MODE_NORMAL) continue;

            bool is_pressed = key_stable[i];
            if (is_pressed != last_key_state[i]) {
                last_key_state[i] = is_pressed;
                activity_detected = true;

                if (current_mode == MODE_NORMAL) {
                    // OLED Core 1 (solo si la tecla tiene pantalla asociada)
                    uint8_t oled = key_to_oled[i];
                    if (oled > 0) {
                        if (is_pressed) {
                            active_inversions |= (1 << (oled - 1));
                        } else {
                            active_inversions &= ~(1 << (oled - 1));
                        }
                    }

                    // Telemetría serial PC
                    char tel[24];
                    int len = snprintf(tel, sizeof(tel), "KEY_EV:%d:%d\n", i+1, is_pressed ? 1 : 0);
                    tud_cdc_n_write(0, tel, len);
                    tud_cdc_n_write_flush(0);

                    // Envío de atajo nativo HID según el perfil activo y SUPER.
                    // Al soltar no se vuelve a mirar el perfil: HidOut recuerda
                    // lo que envió cada tecla, así que cambiar de capa con una
                    // tecla hundida ya no puede dejarla pegada.
                    if (is_pressed) {
                        uint8_t mapping_idx = i + (super_active ? 12 : 0);
                        uint8_t mod = cur_page().key_mappings[mapping_idx].modifier;
                        uint8_t key = cur_page().key_mappings[mapping_idx].keycode;
                        if (mod == KEYACT_CONSUMER) {
                            // Acción multimedia, traducida de su índice de tabla
                            HidOut::consumer_tap(get_consumer_key_from_index(key));
                        } else if (mod == KEYACT_GOTO_PAGE) {
                            // Salto directo. El número va en base 1, que es como
                            // lo ve el usuario en la app y en las pantallas.
                            if (key >= 1 && key <= MAX_PAGES) set_active_page((uint8_t)(key - 1));
                        } else if (mod == KEYACT_PAGE_STATE) {
                            // Abre el gestor de páginas
                            current_mode = MODE_MENU_PAGES;
                            push_system_refresh();
                        } else if (mod == KEYACT_MACRO) {
                            // La secuencia la ejecuta el PC, no el teclado.
                            trigger_macro(key);
                        } else if (mod != 0 && key != 0 && ORBY_COMBO_AS_TAP) {
                            // Combinación tipo Ctrl+C: pulsación breve, no
                            // mantenida. Si se mantuviera, el Ctrl se queda
                            // puesto en el host y este convierte la rueda en
                            // zoom (y el Shift, en scroll horizontal): era la
                            // razón de que la rueda «no hiciera scroll» con una
                            // tecla apretada.
                            HidOut::tap(mod, key);
                        } else if (mod != 0 || key != 0) {
                            // Tecla suelta o modificador a pelo: se mantiene, que
                            // es lo que hace falta para el autorrepetido y para
                            // usarla como modificador del ratón.
                            HidOut::key_down((uint8_t)i, mod, key);
                        }
                    } else {
                        HidOut::key_up((uint8_t)i);
                    }
                } else if (is_pressed && current_mode == MODE_MENU_PAGES) {
                    // Gestor de páginas: cada tecla con número salta a su página.
                    // La de la pantalla 10 sale sin cambiar nada.
                    uint8_t count = profiles[active_profile_idx].page_count;
                    if (i < count) {
                        set_active_page((uint8_t)i);
                        current_mode = MODE_NORMAL;
                        push_system_refresh();
                    } else if (i == key_index_for_screen(10)) {
                        current_mode = MODE_NORMAL;
                        push_system_refresh();
                    }
                } else if (is_pressed) {
                    // En cualquier menú, las teclas 1 a 5 actúan como atajos de selección
                    if (i >= 0 && i <= 4) {
                        selected_menu_idx = i;
                        
                        if (current_mode == MODE_MENU_MAIN) {
                            if (selected_menu_idx == 0) current_mode = MODE_MENU_PERF;
                            else if (selected_menu_idx == 1) current_mode = MODE_MENU_REPO;
                            else if (selected_menu_idx == 2) current_mode = MODE_MENU_INFO;
                            else if (selected_menu_idx == 3) {
                                current_mode = MODE_NORMAL;
                                char tel[24];
                                int len = snprintf(tel, sizeof(tel), "MODE:NORMAL\n");
                                tud_cdc_n_write(0, tel, len);
                                tud_cdc_n_write_flush(0);
                            }
                            selected_menu_idx = 0;
                            if (current_mode == MODE_MENU_PERF) perf_menu_open();
                            push_system_refresh();
                        } else if (current_mode == MODE_MENU_PERF) {
                            if (selected_menu_idx < profile_count) {
                                active_profile_idx = selected_menu_idx;
                                active_page = 0;
                                save_settings();
                                current_mode = MODE_NORMAL;
                                char tel[24];
                                int len = snprintf(tel, sizeof(tel), "MODE:NORMAL\n");
                                tud_cdc_n_write(0, tel, len);
                                tud_cdc_n_write_flush(0);
                            } else {
                                current_mode = MODE_MENU_MAIN;
                            }
                            selected_menu_idx = 0;
                            push_system_refresh();
                        } else if (current_mode == MODE_MENU_REPO) {
                            if (selected_menu_idx >= 0 && selected_menu_idx <= 3) {
                                const uint8_t repo_vals[4] = {1, 5, 10, 0};
                                reposo_timeout_min = repo_vals[selected_menu_idx];
                                save_settings();
                            }
                            current_mode = MODE_MENU_MAIN;
                            selected_menu_idx = 1;
                            push_system_refresh();
                        } else if (current_mode == MODE_MENU_INFO) {
                            if (selected_menu_idx == 4) {
                                current_mode = MODE_MENU_MAIN;
                                selected_menu_idx = 2;
                                push_system_refresh();
                            }
                        }
                    }
                }
            }
        }

        // --- MANEJO DE INACTIVIDAD / REPOSO AUTOMÁTICO ---
        if (activity_detected) {
            last_activity_time = now;
            if (is_sleeping) {
                is_sleeping = false;
                displays_on = true;
                display_power_req = true;
                push_system_refresh();
            }
        } else if (!is_sleeping && reposo_timeout_min > 0) {
            uint32_t idle_duration = now - last_activity_time;
            if (idle_duration >= (uint32_t)(reposo_timeout_min * 60 * 1000)) {
                is_sleeping = true;
                displays_on = false;
                display_power_req = true;
            }
        }

        // 250 us: el AS5600 se muestrea a 1 kHz y el endpoint HID sondea cada
        // 1 ms, así que un ciclo de 1 ms dejaba el scroll con granularidad
        // visible y retrasaba también la respuesta de las teclas.
        sleep_us(250);
    }
    return 0;
}
