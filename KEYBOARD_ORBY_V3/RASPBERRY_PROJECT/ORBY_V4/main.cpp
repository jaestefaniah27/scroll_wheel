#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "bsp/board.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include "pinout.h"
#include "hardware_encoder.h"
#include "hardware_oled.h"
#include "oled_text.h"
#include "oled_bitmaps.h"
#include "hid_hires.h"
#include "hardware_as5600.h"

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

// Huecos de acciones giratorias dentro de cada perfil.
enum RotarySlot : uint8_t {
    ROT_ENC1_CW = 0, ROT_ENC1_CCW, ROT_ENC1_CLICK,
    ROT_ENC2_CW,     ROT_ENC2_CCW, ROT_ENC2_CLICK,
    ROT_WHEEL_CW,    ROT_WHEEL_CCW,
    ROT_SLOT_COUNT
};

struct Profile {
    char name[8];
    char oled_labels[20][8];    // 0-9 Normal, 10-19 SUPER
    KeyAction key_mappings[24]; // 0-11 Normal, 12-23 SUPER
    RotaryAction rotary[ROT_SLOT_COUNT];
};

// Perfiles de fábrica. Se copian a `profiles` (RAM) al arrancar; la app de
// escritorio puede editar la copia en RAM y persistirla con SAVE_STATE.
const Profile default_profiles[4] = {
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { KEYBOARD_MODIFIER_LEFTGUI | KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_S }, // Tecla 11 -> Win+Shift+S (OLED 10)
            { 0,                          0         }, // Tecla 12 -> NADA (Hold Menu)
            
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { 0xFE,                       6         }, // Tecla 11 -> Prev (Consumer idx 6)
            { 0,                          0         }  // Tecla 12 -> NADA
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_J }, // Duplicate Layer (Ctrl+J)
            { 0,                          0         }, // Tecla 12 -> NADA
            
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { 0,                          HID_KEY_A }, // Path Selection (A)
            { 0,                          0         }  // Tecla 12 -> NADA
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { 0,                          HID_KEY_3 }, // 3D View (3)
            { 0,                          0         }, // Tecla 12 -> NADA
            
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { KEYBOARD_MODIFIER_LEFTALT,  HID_KEY_O }, // Options (Alt+O)
            { 0,                          0         }  // Tecla 12 -> NADA
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { 0,                          HID_KEY_N }, // Nest (N)
            { 0,                          0         }, // Tecla 12 -> NADA
            
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
            { 0,                          0         }, // Tecla 10 -> NADA (SUPER)
            { KEYBOARD_MODIFIER_LEFTCTRL, HID_KEY_R }, // Speed (Ctrl+R)
            { 0,                          0         }  // Tecla 12 -> NADA
        }
    }
};

// Copia viva y editable de los perfiles.
Profile profiles[4];

// ==========================================
// BITMAPS OLED PERSONALIZADOS
// ==========================================
// 20 huecos por perfil: 10 pantallas × 2 capas (Normal / SUPER).
//
// Cada perfil ocupa su propia región alineada a páginas de Flash para poder
// reescribir solo el que ha cambiado. Borrar los 32 KB de golpe mantendría las
// interrupciones desactivadas cerca de un segundo y el host llegaría a perder
// el dispositivo USB.
#define OLED_FB_SIZE         360
#define OLED_SLOTS           20
#define OLED_PROFILE_STRIDE  7424   // 20*360 = 7200, redondeado a página de 256
#define OLED_BANK_BYTES      (4 * OLED_PROFILE_STRIDE)

static uint8_t  custom_oled_bank[OLED_BANK_BYTES];
static uint32_t custom_oled_mask[4];  // un bit por hueco ocupado
static uint8_t  custom_oled_dirty = 0; // un bit por perfil pendiente de escribir

static inline uint8_t* oled_slot_ptr(uint8_t profile, uint8_t slot) {
    return &custom_oled_bank[profile * OLED_PROFILE_STRIDE + slot * OLED_FB_SIZE];
}

static inline bool oled_slot_used(uint8_t profile, uint8_t slot) {
    return (custom_oled_mask[profile] & (1u << slot)) != 0;
}

// ==========================================
// ESTADO COMPARTIDO (SHARED STATE)
// ==========================================
enum AppMode {
    MODE_NORMAL,
    MODE_MENU_MAIN,
    MODE_MENU_PERF,
    MODE_MENU_BRIL,
    MODE_MENU_REPO,
    MODE_MENU_INFO
};

volatile AppMode current_mode = MODE_NORMAL;
volatile uint8_t active_profile_idx = 0;   // 0=OFIM, 1=PHTS, 2=ALTM, 3=PREM
volatile uint8_t current_brightness = 207;   // 0xCF = 207 por defecto
volatile uint8_t reposo_timeout_min = 5;    // 5 minutos por defecto (0=OFF)
volatile uint8_t selected_menu_idx = 0;     // Opción de menú activa (0 a 4)
volatile bool super_active = false;         // Estado de la tecla modificadora SUPER
volatile bool system_refresh_req = false;   // Flag para actualizar pantallas
volatile uint16_t active_inversions = 0;    // Bitmask para inversiones en tiempo real

// ==========================================
// CONFIGURACIÓN DE LA RUEDA DE SCROLL
// ==========================================
// Cuántos "detents" (clics de rueda de ratón clásica) equivalen a una vuelta
// completa de la rueda magnética. 60 = valor por defecto. Ajustable en caliente
// desde la app con SET_SCROLL:<n>.
#define SCROLL_DETENTS_MIN     6
#define SCROLL_DETENTS_MAX   240
#define SCROLL_DETENTS_DEFAULT 60

volatile uint8_t scroll_detents_per_rev = SCROLL_DETENTS_DEFAULT;
volatile uint8_t scroll_invert = 0;

// Persistencia en Flash. Dos regiones a partir de 1.5 MB:
//   - 1 sector (4 KB) para ajustes + los 4 perfiles editables
//   - 8 sectores (32 KB) para el banco de bitmaps OLED personalizados
#define FLASH_TARGET_OFFSET  (1536 * 1024)
#define FLASH_OLED_OFFSET    (FLASH_TARGET_OFFSET + FLASH_SECTOR_SIZE)
#define FLASH_OLED_SLICE     (2 * FLASH_SECTOR_SIZE)  // 8 KB por perfil
#define FLASH_OLED_AT(p)     (FLASH_OLED_OFFSET + (p) * FLASH_OLED_SLICE)

// Versiones del bloque de ajustes. Cada cambio de estructura necesita un magic
// nuevo, pero NO se descarta lo guardado: se migra campo a campo. Los bitmaps
// OLED viven en su propia región y sobreviven a todo esto, pero la máscara de
// huecos ocupados está aquí, así que perderla equivale a perder los iconos.
#define SETTINGS_MAGIC_V1 0xDEB001CE  // original: solo ajustes básicos
#define SETTINGS_MAGIC_V2 0xDEB00204  // + perfiles editables, scroll y bitmaps
#define SETTINGS_MAGIC    0xDEB00205  // + acciones de encoders y rueda

// Diseños antiguos, conservados solo para poder leer lo ya grabado.
struct ProfileV2 {
    char name[8];
    char oled_labels[20][8];
    KeyAction key_mappings[24];
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

struct Settings {
    uint32_t magic;
    uint8_t  active_profile_idx;
    uint8_t  current_brightness;
    uint8_t  reposo_timeout_min;
    uint8_t  scroll_detents_per_rev;
    uint8_t  scroll_invert;
    uint8_t  padding[3];
    uint32_t custom_oled_mask[4];
    Profile  profiles[4];
};

// El blob de ajustes se programa redondeado a página de 256 bytes.
#define SETTINGS_BLOB_SIZE (((sizeof(Settings) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

static uint8_t settings_blob[SETTINGS_BLOB_SIZE];

// Guardas de disposición: si los perfiles crecen y dejan de caber, hay que
// ampliar el sector reservado en lugar de pisar el banco de bitmaps.
static_assert(SETTINGS_BLOB_SIZE <= FLASH_SECTOR_SIZE,
              "Los ajustes ya no caben en un sector de Flash");
static_assert(OLED_SLOTS * OLED_FB_SIZE <= OLED_PROFILE_STRIDE,
              "El stride por perfil no cubre sus 20 bitmaps");
static_assert(OLED_PROFILE_STRIDE % FLASH_PAGE_SIZE == 0,
              "flash_range_program exige múltiplos de página");

// Los perfiles de fábrica solo declaran teclas y etiquetas; las acciones
// giratorias se rellenan aquí para no repetirlas en los cuatro literales.
static void apply_default_rotaries(Profile& p) {
    p.rotary[ROT_ENC1_CW]    = { ROT_CONSUMER, 0, CONS_VOL_UP };
    p.rotary[ROT_ENC1_CCW]   = { ROT_CONSUMER, 0, CONS_VOL_DOWN };
    p.rotary[ROT_ENC1_CLICK] = { ROT_CONSUMER, 0, CONS_MUTE };
    p.rotary[ROT_ENC2_CW]    = { ROT_CONSUMER, 0, CONS_BRIGHT_UP };
    p.rotary[ROT_ENC2_CCW]   = { ROT_CONSUMER, 0, CONS_BRIGHT_DOWN };
    p.rotary[ROT_ENC2_CLICK] = { ROT_NONE, 0, 0 };
    p.rotary[ROT_WHEEL_CW]   = { ROT_SCROLL_V, 0, 0 };
    p.rotary[ROT_WHEEL_CCW]  = { ROT_SCROLL_V, 0, 0 };
}

void load_defaults() {
    memcpy(profiles, default_profiles, sizeof(profiles));
    for (int i = 0; i < 4; i++) apply_default_rotaries(profiles[i]);
    memset(custom_oled_bank, 0, sizeof(custom_oled_bank));
    memset(custom_oled_mask, 0, sizeof(custom_oled_mask));
    custom_oled_dirty = 0x0F; // hay que borrar en Flash lo que hubiera antes
    active_profile_idx = 0;
    current_brightness = 207;
    reposo_timeout_min = 5;
    scroll_detents_per_rev = SCROLL_DETENTS_DEFAULT;
    scroll_invert = 0;
}

void save_settings() {
    Settings* s = (Settings*)settings_blob;
    memset(settings_blob, 0, sizeof(settings_blob));

    s->magic                  = SETTINGS_MAGIC;
    s->active_profile_idx     = active_profile_idx;
    s->current_brightness     = current_brightness;
    s->reposo_timeout_min     = reposo_timeout_min;
    s->scroll_detents_per_rev = scroll_detents_per_rev;
    s->scroll_invert          = scroll_invert;
    memcpy(s->custom_oled_mask, custom_oled_mask, sizeof(custom_oled_mask));
    memcpy(s->profiles, profiles, sizeof(profiles));

    // Ajustes y perfiles: un solo sector, ~60 ms con interrupciones cerradas.
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, settings_blob, SETTINGS_BLOB_SIZE);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    // Bitmaps: solo los perfiles tocados, y liberando el USB entre uno y otro
    // para que el host no pierda el dispositivo durante el borrado.
    for (uint8_t p = 0; p < 4; p++) {
        if (!(custom_oled_dirty & (1u << p))) continue;

        tud_task();
        ints = save_and_disable_interrupts();
        multicore_lockout_start_blocking();
        flash_range_erase(FLASH_OLED_AT(p), FLASH_OLED_SLICE);
        flash_range_program(FLASH_OLED_AT(p), &custom_oled_bank[p * OLED_PROFILE_STRIDE], OLED_PROFILE_STRIDE);
        multicore_lockout_end_blocking();
        restore_interrupts(ints);
    }
    custom_oled_dirty = 0;
}

// Trae a RAM los bitmaps de los perfiles marcados en la máscara.
static void load_oled_bank() {
    memset(custom_oled_bank, 0, sizeof(custom_oled_bank));
    for (uint8_t p = 0; p < 4; p++) {
        if (!custom_oled_mask[p]) continue;
        memcpy(&custom_oled_bank[p * OLED_PROFILE_STRIDE],
               (const uint8_t*)(XIP_BASE + FLASH_OLED_AT(p)),
               OLED_PROFILE_STRIDE);
    }
    custom_oled_dirty = 0;
}

static void sanitize_loaded() {
    if (active_profile_idx > 3) active_profile_idx = 0;
    if (scroll_detents_per_rev < SCROLL_DETENTS_MIN || scroll_detents_per_rev > SCROLL_DETENTS_MAX) {
        scroll_detents_per_rev = SCROLL_DETENTS_DEFAULT;
    }
    scroll_invert = scroll_invert ? 1 : 0;
}

void load_settings() {
    const uint32_t magic = *(const uint32_t*)(XIP_BASE + FLASH_TARGET_OFFSET);

    if (magic == SETTINGS_MAGIC) {
        const Settings* s = (const Settings*)(XIP_BASE + FLASH_TARGET_OFFSET);
        active_profile_idx     = s->active_profile_idx;
        current_brightness     = s->current_brightness;
        reposo_timeout_min     = s->reposo_timeout_min;
        scroll_detents_per_rev = s->scroll_detents_per_rev;
        scroll_invert          = s->scroll_invert;
        memcpy(profiles, s->profiles, sizeof(profiles));
        memcpy(custom_oled_mask, s->custom_oled_mask, sizeof(custom_oled_mask));
        sanitize_loaded();
        load_oled_bank();
        return;
    }

    if (magic == SETTINGS_MAGIC_V2) {
        // Formato anterior a las acciones giratorias. Se conserva todo lo que
        // costó trabajo —perfiles, etiquetas, atajos y la máscara de iconos— y
        // solo se rellenan de fábrica los mandos, que antes no existían.
        const SettingsV2* s = (const SettingsV2*)(XIP_BASE + FLASH_TARGET_OFFSET);
        active_profile_idx     = s->active_profile_idx;
        current_brightness     = s->current_brightness;
        reposo_timeout_min     = s->reposo_timeout_min;
        scroll_detents_per_rev = s->scroll_detents_per_rev;
        scroll_invert          = s->scroll_invert;
        memcpy(custom_oled_mask, s->custom_oled_mask, sizeof(custom_oled_mask));

        for (int i = 0; i < 4; i++) {
            memcpy(profiles[i].name,         s->profiles[i].name,         sizeof(profiles[i].name));
            memcpy(profiles[i].oled_labels,  s->profiles[i].oled_labels,  sizeof(profiles[i].oled_labels));
            memcpy(profiles[i].key_mappings, s->profiles[i].key_mappings, sizeof(profiles[i].key_mappings));
            apply_default_rotaries(profiles[i]);
        }
        sanitize_loaded();
        load_oled_bank();
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
        custom_oled_dirty = 0; // no había bitmaps guardados que borrar
        return;
    }

    load_defaults();
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

void send_keyboard_report(uint8_t modifier, uint8_t keycode) {
    // Esperar a que el canal HID esté listo para transmitir
    int retry = 0;
    while (!tud_hid_ready() && retry < 100) {
        tud_task();
        sleep_ms(1);
        retry++;
    }

    if (keycode == 0 && modifier == 0) {
        tud_hid_keyboard_report(1, 0, NULL);
    } else {
        uint8_t keycodes[6] = { keycode, 0, 0, 0, 0, 0 };
        tud_hid_keyboard_report(1, modifier, keycodes);
    }
}

void send_consumer_key(uint16_t keycode) {
    // Esperar a que el canal HID esté listo para enviar la pulsación
    int retry = 0;
    while (!tud_hid_ready() && retry < 100) {
        tud_task();
        sleep_ms(1);
        retry++;
    }
    
    tud_hid_report(3, &keycode, 2);
    
    // Esperar a que se complete la transmisión antes de enviar la liberación
    retry = 0;
    while (!tud_hid_ready() && retry < 100) {
        tud_task();
        sleep_ms(1);
        retry++;
    }
    
    uint16_t release = 0;
    tud_hid_report(3, &release, 2);
}

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
static int32_t pending_wheel  = 0;  // unidades verticales pendientes
static int32_t pending_pan    = 0;  // detents horizontales pendientes

// Vacía las colas de scroll sin bloquear: si el endpoint HID está ocupado, lo
// pendiente se conserva para el siguiente ciclo en vez de perderse.
void wheel_flush() {
    if ((pending_wheel == 0 && pending_pan == 0) || !tud_hid_ready()) return;

    int32_t v = pending_wheel;
    if (v >  32767) v =  32767;
    if (v < -32767) v = -32767;

    int32_t h = pending_pan;
    if (h >  127) h =  127;
    if (h < -127) h = -127;

    orby_mouse_report_t rpt = { 0, 0, 0, (int16_t)v, (int8_t)h };
    if (tud_hid_report(REPORT_ID_MOUSE, &rpt, sizeof(rpt))) {
        pending_wheel -= v;
        pending_pan   -= h;
    }
}

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
        pending_pan += detents;
        return;
    }

    if (type == ROT_ZOOM) {
        // Ctrl mantenido mientras dura el desplazamiento: es lo que interpretan
        // navegadores y editores como acercar/alejar.
        send_keyboard_report(KEYBOARD_MODIFIER_LEFTCTRL, 0);
        pending_wheel += detents * (g_hires_multiplier ? HID_HIRES_MULTIPLIER : 1);
        wheel_flush();
        send_keyboard_report(0, 0);
        return;
    }

    pending_wheel += detents * (g_hires_multiplier ? HID_HIRES_MULTIPLIER : 1);
}

// Una sola repetición de una acción discreta (tecla o multimedia).
static void emit_discrete(const RotaryAction& a) {
    if (a.type == ROT_CONSUMER) {
        uint16_t code = get_consumer_key_from_index(a.keycode);
        if (code) send_consumer_key(code);
    } else if (a.type == ROT_KEY) {
        if (a.modifier || a.keycode) {
            send_keyboard_report(a.modifier, a.keycode);
            send_keyboard_report(0, 0);
        }
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
    const RotaryAction& cw  = profiles[active_profile_idx].rotary[ROT_WHEEL_CW];
    const RotaryAction& ccw = profiles[active_profile_idx].rotary[ROT_WHEEL_CCW];

    if (scroll_invert) delta_counts = -delta_counts;

    // unidades_hires = cuentas * detents_por_vuelta * 120 / 4096
    scroll_frac += delta_counts * (int32_t)scroll_detents_per_rev * HID_HIRES_MULTIPLIER;
    int32_t units = scroll_frac / AS5600_COUNTS_PER_REV;
    scroll_frac  -= units * AS5600_COUNTS_PER_REV;
    if (units == 0) return;

    // Solo el desplazamiento vertical aprovecha la alta resolución; el resto de
    // acciones son discretas y necesitan detents completos.
    if (cw.type == ROT_SCROLL_V && g_hires_multiplier != 0) {
        pending_wheel += units;
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

// Dibuja una barra de progreso interactiva para el brillo
void draw_brightness_progress(uint8_t* fb, uint8_t val) {
    int page = 2; // Página central (vertical)
    // Marco exterior para la barra
    fb[page * 72 + 4] = 0xFF;
    fb[page * 72 + 67] = 0xFF;
    for (int x = 5; x < 67; x++) {
        fb[page * 72 + x] = 0x81; // Solo bits superior e inferior
    }
    // Relleno progresivo
    int fill_width = (val * 60) / 255;
    for (int x = 0; x < fill_width; x++) {
        fb[page * 72 + 6 + x] = 0xFF; // Relleno total de la página
    }
}

// Dibuja el contenido de una pantalla según el modo y perfil actual
void refresh_single_screen(HardwareOled& oleds, uint8_t screen_num) {
    uint8_t fb[360];
    memset(fb, 0, sizeof(fb));

    if (current_mode == MODE_NORMAL) {
        uint8_t slot = (uint8_t)((screen_num - 1) + (super_active ? 10 : 0));

        if (slot < OLED_SLOTS && oled_slot_used(active_profile_idx, slot)) {
            // Icono personalizado subido desde la app con OLED_CHUNK
            oleds.paint_screen(screen_num, oled_slot_ptr(active_profile_idx, slot));
        } else if (active_profile_idx == 0 && !super_active) {
            // Perfil por defecto (OFIM) -> Carga mapas de bits pre-horneados
            const uint8_t* bmp = OledBitmaps::get_bitmap_by_index(screen_num - 1);
            oleds.paint_screen(screen_num, bmp);
        } else {
            // Perfiles de fábrica de texto personalizado con contorno premium (o OFIM en SUPER)
            draw_premium_frame(fb);
            int label_idx = (screen_num - 1) + (super_active ? 10 : 0);
            const char* label = profiles[active_profile_idx].oled_labels[label_idx];
            OledText::render_string_to_framebuffer(label, fb);
            draw_premium_frame(fb); // Volver a aplicar el marco tras renderizar el texto
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_MAIN) {
        if (screen_num >= 1 && screen_num <= 5) {
            draw_premium_frame(fb);
            const char* menu_opts[5] = {"PERF", "BRIL", "REPO", "INFO", "EXIT"};
            OledText::render_string_to_framebuffer(menu_opts[screen_num - 1], fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else {
            // Pantallas 6-10 vacías
            oleds.paint_screen(screen_num, fb);
        }
    }
    else if (current_mode == MODE_MENU_PERF) {
        if (screen_num >= 1 && screen_num <= 4) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer(profiles[screen_num - 1].name, fb);
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
    else if (current_mode == MODE_MENU_BRIL) {
        if (screen_num == 1) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("BRIL", fb);
            draw_premium_frame(fb);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 2) {
            draw_brightness_progress(fb, current_brightness);
            oleds.paint_screen(screen_num, fb);
        } else if (screen_num == 5) {
            draw_premium_frame(fb);
            OledText::render_string_to_framebuffer("SAVE", fb);
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
}

void core1_entry() {
    multicore_lockout_victim_init(); // Permitir que Core 0 pause este core durante escrituras Flash
    HardwareOled oleds;
    oleds.init_all_screens();

    // Aplicar brillo inicial
    oleds.set_brightness(current_brightness);

    // Pintar los mapas de bits iniciales en modo normal
    for (int i = 1; i <= 10; i++) {
        refresh_single_screen(oleds, i);
    }

    uint16_t current_inversions = 0;

    while (true) {
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
            if (current_mode == MODE_MENU_MAIN || current_mode == MODE_MENU_PERF || current_mode == MODE_MENU_REPO) {
                for (int i = 1; i <= 5; i++) {
                    oleds.invert_screen(i, (selected_menu_idx == (i - 1)));
                }
            } else if (current_mode == MODE_MENU_INFO) {
                // Destacar siempre "BACK" en la tecla 5
                for (int i = 1; i <= 5; i++) {
                    oleds.invert_screen(i, (i == 5));
                }
            } else if (current_mode == MODE_MENU_BRIL) {
                // Destacar SAVE en la tecla 5
                oleds.invert_screen(1, false);
                oleds.invert_screen(2, false);
                oleds.invert_screen(5, true);
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

static void send_scroll_state() {
    cdc_printf("SCROLL:OK:%d:%d:%d\n",
               (int)scroll_detents_per_rev,
               (int)scroll_invert,
               (int)(g_hires_multiplier != 0 ? 1 : 0));
}

static void dump_profile(uint8_t idx) {
    const Profile& p = profiles[idx];
    cdc_printf("PROF:%d:NAME:%.*s\n", idx, (int)sizeof(p.name), p.name);
    for (int s = 0; s < 20; s++) {
        cdc_printf("PROF:%d:LBL:%d:%.*s\n", idx, s, (int)sizeof(p.oled_labels[s]), p.oled_labels[s]);
    }
    for (int s = 0; s < 24; s++) {
        cdc_printf("PROF:%d:KEY:%d:%d:%d\n", idx, s,
                   p.key_mappings[s].modifier, p.key_mappings[s].keycode);
    }
    for (int s = 0; s < ROT_SLOT_COUNT; s++) {
        cdc_printf("PROF:%d:ROT:%d:%d:%d:%d\n", idx, s,
                   p.rotary[s].type, p.rotary[s].modifier, p.rotary[s].keycode);
    }
    cdc_printf("PROF:%d:OLEDMASK:%lu\n", idx, (unsigned long)custom_oled_mask[idx]);
    cdc_printf("PROF:%d:END\n", idx);
}

void process_command(const char* cmd) {
    // ---------- Descubrimiento y estado ----------
    if (strncmp(cmd, "ACK", 3) == 0) {
        cdc_printf("ORBY_V4:FW=2.0:KEYS=12:OLEDS=10:ENCODERS=2:MODE=%s\n",
                   (current_mode == MODE_NORMAL) ? "NORMAL" : "MENU");
        return;
    }

    if (strcmp(cmd, "GET_STATE") == 0) {
        cdc_printf("STATE:PROFILE:%d\n", (int)active_profile_idx);
        cdc_printf("STATE:BRIGHTNESS:%d\n", (int)current_brightness);
        cdc_printf("STATE:TIMEOUT:%d\n", (int)reposo_timeout_min);
        cdc_printf("STATE:MODE:%s\n", (current_mode == MODE_NORMAL) ? "NORMAL" : "MENU");
        send_scroll_state();
        cdc_printf("STATE:END\n");
        return;
    }

    // ---------- Ajustes básicos ----------
    if (strncmp(cmd, "SET_PROFILE:", 12) == 0) {
        int profile_idx = atoi(cmd + 12);
        if (profile_idx >= 0 && profile_idx <= 3) {
            active_profile_idx = profile_idx;
            push_system_refresh();
            cdc_printf("PROFILE:OK:%d\n", (int)active_profile_idx);
        } else {
            cdc_printf("ERR:PROFILE_RANGE\n");
        }
        return;
    }

    if (strncmp(cmd, "SET_BRIGHTNESS:", 15) == 0) {
        int val = atoi(cmd + 15);
        if (val >= 0 && val <= 255) {
            current_brightness = val;
            push_system_refresh();
            cdc_printf("BRIGHTNESS:OK:%d\n", (int)current_brightness);
        } else {
            cdc_printf("ERR:BRIGHTNESS_RANGE\n");
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
    if (strncmp(cmd, "SET_SCROLL_INV:", 15) == 0) {
        scroll_invert = (atoi(cmd + 15) != 0) ? 1 : 0;
        send_scroll_state();
        return;
    }

    if (strncmp(cmd, "SET_SCROLL:", 11) == 0) {
        int val = atoi(cmd + 11);
        if (val >= SCROLL_DETENTS_MIN && val <= SCROLL_DETENTS_MAX) {
            scroll_detents_per_rev = (uint8_t)val;
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

    // ---------- Lectura y edición de perfiles ----------
    if (strncmp(cmd, "GET_PROFILE:", 12) == 0) {
        int idx = atoi(cmd + 12);
        if (idx >= 0 && idx <= 3) dump_profile((uint8_t)idx);
        else cdc_printf("ERR:PROFILE_RANGE\n");
        return;
    }

    if (strcmp(cmd, "GET_PROFILES") == 0) {
        for (int i = 0; i < 4; i++) dump_profile((uint8_t)i);
        return;
    }

    // SET_NAME:<perfil>:<texto>
    if (strncmp(cmd, "SET_NAME:", 9) == 0) {
        int idx = 0;
        const char* p = next_field(cmd + 9, &idx);
        if (!p || idx < 0 || idx > 3) { cdc_printf("ERR:BAD_ARGS\n"); return; }
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
        if (!p || idx < 0 || idx > 3 || slot < 0 || slot > 19) { cdc_printf("ERR:BAD_ARGS\n"); return; }
        memset(profiles[idx].oled_labels[slot], 0, sizeof(profiles[idx].oled_labels[slot]));
        strncpy(profiles[idx].oled_labels[slot], p, sizeof(profiles[idx].oled_labels[slot]) - 1);
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
        if (!p || idx < 0 || idx > 3 || slot < 0 || slot > 23 ||
            mod < 0 || mod > 255 || key < 0 || key > 255) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        profiles[idx].key_mappings[slot].modifier = (uint8_t)mod;
        profiles[idx].key_mappings[slot].keycode  = (uint8_t)key;
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
        if (!p || idx < 0 || idx > 3 || slot < 0 || slot >= ROT_SLOT_COUNT ||
            type < 0 || type > ROT_ZOOM || mod < 0 || mod > 255 || key < 0 || key > 255) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        profiles[idx].rotary[slot] = { (uint8_t)type, (uint8_t)mod, (uint8_t)key };
        cdc_printf("ROTARY:OK:%d:%d\n", idx, slot);
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
        if (!p || idx < 0 || idx > 3 || slot < 0 || slot > 19 ||
            offset < 0 || offset >= OLED_FB_SIZE) {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }

        uint8_t* dst = oled_slot_ptr((uint8_t)idx, (uint8_t)slot);
        int written = 0;
        while (p[0] && p[1] && (offset + written) < OLED_FB_SIZE) {
            int hi = hex_nibble(p[0]);
            int lo = hex_nibble(p[1]);
            if (hi < 0 || lo < 0) break;
            dst[offset + written] = (uint8_t)((hi << 4) | lo);
            written++;
            p += 2;
        }

        custom_oled_mask[idx] |= (1u << slot);
        custom_oled_dirty |= (1u << idx);
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
        if (!p || idx < 0 || idx > 3 || slot < 0 || slot > 19) { cdc_printf("ERR:BAD_ARGS\n"); return; }

        if (!oled_slot_used((uint8_t)idx, (uint8_t)slot)) {
            cdc_printf("OLEDDATA:%d:%d:NONE\n", idx, slot);
            return;
        }

        static const char HEXCHARS[] = "0123456789abcdef";
        const uint8_t* src = oled_slot_ptr((uint8_t)idx, (uint8_t)slot);
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
        if (idx < 0 || idx > 3) { cdc_printf("ERR:BAD_ARGS\n"); return; }

        if (slot == 255) {
            custom_oled_mask[idx] = 0;
            memset(oled_slot_ptr((uint8_t)idx, 0), 0, OLED_SLOTS * OLED_FB_SIZE);
        } else if (slot >= 0 && slot <= 19) {
            custom_oled_mask[idx] &= ~(1u << slot);
            memset(oled_slot_ptr((uint8_t)idx, (uint8_t)slot), 0, OLED_FB_SIZE);
        } else {
            cdc_printf("ERR:BAD_ARGS\n");
            return;
        }
        custom_oled_dirty |= (1u << idx);
        if ((int)active_profile_idx == idx) push_system_refresh();
        cdc_printf("OLED:CLEARED:%d:%d\n", idx, slot);
        return;
    }

    // ---------- Persistencia ----------
    if (strcmp(cmd, "SAVE_STATE") == 0) {
        save_settings();
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
// CORE 0: ENCODERS, TECLAS Y USB CDC / HID
// ==========================================
int main() {
    board_init();
    tusb_init();
    sleep_ms(2000);

    // Cargar configuración de Flash
    load_settings();

    multicore_launch_core1(core1_entry);

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

    // Instancia temporal para despertar pantallas
    HardwareOled main_oled_ctrl;

    while (true) {
        tud_task();

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

                // Acción configurada en el perfil activo
                const Profile& p = profiles[active_profile_idx];
                apply_rotary(p.rotary[ROT_ENC1_CW], p.rotary[ROT_ENC1_CCW], delta_izq);
            } else if (current_mode == MODE_MENU_MAIN || current_mode == MODE_MENU_PERF || current_mode == MODE_MENU_REPO) {
                // Desplazamiento de menú
                selected_menu_idx = (selected_menu_idx + delta_izq + 5) % 5;
                push_system_refresh();
            } else if (current_mode == MODE_MENU_BRIL) {
                // Ajuste de brillo en caliente
                int new_bril = current_brightness + delta_izq * 8;
                if (new_bril < 0) new_bril = 0;
                if (new_bril > 255) new_bril = 255;
                current_brightness = new_bril;
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

                // Acción configurada en el perfil activo
                const Profile& p = profiles[active_profile_idx];
                apply_rotary(p.rotary[ROT_ENC2_CW], p.rotary[ROT_ENC2_CCW], delta_der);
            } else if (current_mode == MODE_MENU_BRIL) {
                // También ajustar brillo con encoder derecho
                int new_bril = current_brightness + delta_der * 8;
                if (new_bril < 0) new_bril = 0;
                if (new_bril > 255) new_bril = 255;
                current_brightness = new_bril;
                push_system_refresh();
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
                        emit_discrete(profiles[active_profile_idx].rotary[ROT_ENC1_CLICK]);
                    } else if (current_mode == MODE_MENU_MAIN) {
                        // Confirmar opción
                        if (selected_menu_idx == 0) current_mode = MODE_MENU_PERF;
                        else if (selected_menu_idx == 1) current_mode = MODE_MENU_BRIL;
                        else if (selected_menu_idx == 2) current_mode = MODE_MENU_REPO;
                        else if (selected_menu_idx == 3) current_mode = MODE_MENU_INFO;
                        else if (selected_menu_idx == 4) {
                            current_mode = MODE_NORMAL;
                            char tel2[24];
                            int len2 = snprintf(tel2, sizeof(tel2), "MODE:NORMAL\n");
                            tud_cdc_n_write(0, tel2, len2);
                            tud_cdc_n_write_flush(0);
                        }
                        selected_menu_idx = 0;
                        push_system_refresh();
                    } else if (current_mode == MODE_MENU_PERF) {
                    if (selected_menu_idx >= 0 && selected_menu_idx <= 3) {
                        active_profile_idx = selected_menu_idx;
                        // save_settings() eliminado
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
                    } else if (current_mode == MODE_MENU_BRIL) {
                        // save_settings() eliminado (brillo se aplica temporalmente)
                        current_mode = MODE_MENU_MAIN;
                        selected_menu_idx = 1;
                        push_system_refresh();
                    } else if (current_mode == MODE_MENU_REPO) {
                        if (selected_menu_idx >= 0 && selected_menu_idx <= 3) {
                            const uint8_t repo_vals[4] = {1, 5, 10, 0};
                            reposo_timeout_min = repo_vals[selected_menu_idx];
                            // save_settings() eliminado
                        }
                        current_mode = MODE_MENU_MAIN;
                        selected_menu_idx = 2;
                        push_system_refresh();
                    } else if (current_mode == MODE_MENU_INFO) {
                        current_mode = MODE_MENU_MAIN;
                        selected_menu_idx = 3;
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
                    emit_discrete(profiles[active_profile_idx].rotary[ROT_ENC2_CLICK]);
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
        wheel_flush();

        if (wheel_tel_accum != 0 && (uint32_t)(now - last_wheel_tel) >= 50) {
            last_wheel_tel = now;
            char tel[32];
            int len = snprintf(tel, sizeof(tel), "WHEEL:%ld\n", (long)wheel_tel_accum);
            wheel_tel_accum = 0;
            tud_cdc_n_write(0, tel, len);
            tud_cdc_n_write_flush(0);
        }

        // --- DETECTAR ACCESO RÁPIDO A PERFILES (HOLD 1 SEGUNDO EN TECLA 12) ---
        bool key12_pressed = !gpio_get(Pins::KEY_12);
        static uint32_t key12_hold_start = 0;
        if (key12_pressed && current_mode == MODE_NORMAL) {
            if (key12_hold_start == 0) {
                key12_hold_start = now;
            } else if (now - key12_hold_start >= 1000) {
                activity_detected = true;
                current_mode = MODE_MENU_PERF;
                selected_menu_idx = 0;
                push_system_refresh();
                
                // Notificar por CDC
                char tel[24];
                int len = snprintf(tel, sizeof(tel), "MODE:MENU\n");
                tud_cdc_n_write(0, tel, len);
                tud_cdc_n_write_flush(0);
                
                key12_hold_start = 0;
                // Esperar a que se libere Key 12 para no disparar atajos accidentales
                while (!gpio_get(Pins::KEY_12)) {
                    tud_task();
                    sleep_ms(10);
                }
            }
        } else {
            key12_hold_start = 0;
        }

        // --- TECLAS ---
        // Primero, actualizar el estado de la tecla SUPER (Tecla 10, index 9)
        bool super_pressed = !gpio_get(key_pins[9]); // Tecla 10
        if (super_pressed != super_active) {
            super_active = super_pressed;
            if (current_mode == MODE_NORMAL) {
                push_system_refresh();
            }
            
            // Telemetría serial
            char tel[24];
            int len = snprintf(tel, sizeof(tel), "KEY_EV:10:%d\n", super_active ? 1 : 0);
            tud_cdc_n_write(0, tel, len);
            tud_cdc_n_write_flush(0);
        }

        for (int i = 0; i < 12; i++) {
            // Tecla 10 (index 9) se maneja por separado como SUPER
            if (i == 9) continue;
            
            // Tecla 12 (index 11) en Modo Normal solo responde al hold largo (ya manejado arriba)
            if (i == 11 && current_mode == MODE_NORMAL) continue;

            bool is_pressed = !gpio_get(key_pins[i]);
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

                    // Envío de atajo nativo HID según el perfil activo y SUPER
                    if (is_pressed) {
                        uint8_t mapping_idx = i + (super_active ? 12 : 0);
                        uint8_t mod = profiles[active_profile_idx].key_mappings[mapping_idx].modifier;
                        uint8_t key = profiles[active_profile_idx].key_mappings[mapping_idx].keycode;
                        if (mod == 0xFE) {
                            // Ejecutar acción multimedia nativa traducida de su índice de tabla
                            uint16_t cons_key = get_consumer_key_from_index(key);
                            if (cons_key != 0) {
                                send_consumer_key(cons_key);
                            }
                        } else if (mod != 0 || key != 0) {
                            send_keyboard_report(mod, key);
                        }
                    } else {
                        uint8_t mapping_idx = i + (super_active ? 12 : 0);
                        uint8_t mod = profiles[active_profile_idx].key_mappings[mapping_idx].modifier;
                        if (mod != 0xFE) {
                            send_keyboard_report(0, 0);
                        }
                    }
                } else if (is_pressed) {
                    // En cualquier menú, las teclas 1 a 5 actúan como atajos de selección
                    if (i >= 0 && i <= 4) {
                        selected_menu_idx = i;
                        
                        if (current_mode == MODE_MENU_MAIN) {
                            if (selected_menu_idx == 0) current_mode = MODE_MENU_PERF;
                            else if (selected_menu_idx == 1) current_mode = MODE_MENU_BRIL;
                            else if (selected_menu_idx == 2) current_mode = MODE_MENU_REPO;
                            else if (selected_menu_idx == 3) current_mode = MODE_MENU_INFO;
                            else if (selected_menu_idx == 4) {
                                current_mode = MODE_NORMAL;
                                char tel[24];
                                int len = snprintf(tel, sizeof(tel), "MODE:NORMAL\n");
                                tud_cdc_n_write(0, tel, len);
                                tud_cdc_n_write_flush(0);
                            }
                            selected_menu_idx = 0;
                            push_system_refresh();
                        } else if (current_mode == MODE_MENU_PERF) {
                            if (selected_menu_idx >= 0 && selected_menu_idx <= 3) {
                                active_profile_idx = selected_menu_idx;
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
                        } else if (current_mode == MODE_MENU_BRIL) {
                            if (selected_menu_idx == 4) {
                                save_settings();
                                current_mode = MODE_MENU_MAIN;
                                selected_menu_idx = 1;
                                push_system_refresh();
                            }
                        } else if (current_mode == MODE_MENU_REPO) {
                            if (selected_menu_idx >= 0 && selected_menu_idx <= 3) {
                                const uint8_t repo_vals[4] = {1, 5, 10, 0};
                                reposo_timeout_min = repo_vals[selected_menu_idx];
                                save_settings();
                            }
                            current_mode = MODE_MENU_MAIN;
                            selected_menu_idx = 2;
                            push_system_refresh();
                        } else if (current_mode == MODE_MENU_INFO) {
                            if (selected_menu_idx == 4) {
                                current_mode = MODE_MENU_MAIN;
                                selected_menu_idx = 3;
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
                main_oled_ctrl.set_all_displays_on(true);
                push_system_refresh();
            }
        } else if (!is_sleeping && reposo_timeout_min > 0) {
            uint32_t idle_duration = now - last_activity_time;
            if (idle_duration >= (uint32_t)(reposo_timeout_min * 60 * 1000)) {
                is_sleeping = true;
                main_oled_ctrl.set_all_displays_on(false);
            }
        }

        // 250 us: el AS5600 se muestrea a 1 kHz y el endpoint HID sondea cada
        // 1 ms, así que un ciclo de 1 ms dejaba el scroll con granularidad
        // visible y retrasaba también la respuesta de las teclas.
        sleep_us(250);
    }
    return 0;
}