#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
#include "hardware_as5600.h"

// Mapeos nativos de teclado para las 12 teclas
struct KeyAction {
    uint8_t modifier;
    uint8_t keycode;
};

struct Profile {
    char name[8];
    char oled_labels[20][8]; // 0-9 Normal, 10-19 SUPER
    KeyAction key_mappings[24]; // 0-11 Normal, 12-23 SUPER
};

const Profile profiles[4] = {
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

// Persistencia en Flash (Sector de 4KB seguro al final de la memoria de 2MB)
#define FLASH_TARGET_OFFSET (1536 * 1024) // 1.5 MB offset

struct Settings {
    uint32_t magic;
    uint8_t active_profile_idx;
    uint8_t current_brightness;
    uint8_t reposo_timeout_min;
    uint8_t padding;
};

void save_settings() {
    Settings s;
    s.magic = 0xDEB001CE;
    s.active_profile_idx = active_profile_idx;
    s.current_brightness = current_brightness;
    s.reposo_timeout_min = reposo_timeout_min;
    s.padding = 0;

    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, &s, sizeof(s));

    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
}

void load_settings() {
    const Settings* s = (const Settings*)(XIP_BASE + FLASH_TARGET_OFFSET);
    if (s->magic == 0xDEB001CE) {
        active_profile_idx = s->active_profile_idx;
        current_brightness = s->current_brightness;
        reposo_timeout_min = s->reposo_timeout_min;
    } else {
        active_profile_idx = 0;
        current_brightness = 207;
        reposo_timeout_min = 5;
    }
}

// ==========================================
// ACCIONES USB HID NATIVAS
// ==========================================

uint16_t get_consumer_key_from_index(uint8_t idx) {
    switch (idx) {
        case 1: return 0x0194; // AL Internet Browser
        case 2: return 0x011A; // AL Calculator
        case 3: return 0x00CD; // Play/Pause
        case 4: return 0x00B7; // Stop
        case 5: return 0x00B5; // Next Track
        case 6: return 0x00B6; // Prev Track
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
        if (active_profile_idx == 0 && !super_active) {
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
#define CMD_BUF_SIZE 256
static char cmd_buf[CMD_BUF_SIZE];
static int  cmd_pos = 0;

void push_system_refresh() {
    system_refresh_req = true;
}

void process_command(const char* cmd) {
    if (strncmp(cmd, "ACK", 3) == 0) {
        char resp[128];
        snprintf(resp, sizeof(resp), "ORBY_V4:FW=1.0:KEYS=12:OLEDS=10:ENCODERS=2:MODE=%s\n", 
                 (current_mode == MODE_NORMAL) ? "NORMAL" : "MENU");
        tud_cdc_n_write(0, resp, strlen(resp));
        tud_cdc_n_write_flush(0);
        return;
    }
    
    if (strncmp(cmd, "SET_PROFILE:", 12) == 0) {
        int profile_idx = atoi(cmd + 12);
        if (profile_idx >= 0 && profile_idx <= 3) {
            active_profile_idx = profile_idx;
            push_system_refresh();
            // save_settings() eliminado para evitar desgaste de Flash
            char resp[32];
            snprintf(resp, sizeof(resp), "PROFILE:OK:%d\n", active_profile_idx);
            tud_cdc_n_write(0, resp, strlen(resp));
            tud_cdc_n_write_flush(0);
        }
        return;
    }
    
    if (strncmp(cmd, "SAVE_STATE", 10) == 0) {
        save_settings();
        tud_cdc_n_write(0, "SAVE:OK\n", 8);
        tud_cdc_n_write_flush(0);
        return;
    }
    
    if (strncmp(cmd, "SET_BRIGHTNESS:", 15) == 0) {
        int val = atoi(cmd + 15);
        if (val >= 0 && val <= 255) {
            current_brightness = val;
            push_system_refresh();
            // save_settings() eliminado
            char resp[32];
            snprintf(resp, sizeof(resp), "BRIGHTNESS:OK:%d\n", current_brightness);
            tud_cdc_n_write(0, resp, strlen(resp));
            tud_cdc_n_write_flush(0);
        }
        return;
    }
    
    if (strncmp(cmd, "SET_TIMEOUT:", 12) == 0) {
        int val = atoi(cmd + 12);
        if (val == 1 || val == 5 || val == 10 || val == 0) {
            reposo_timeout_min = val;
            // save_settings() eliminado
            char resp[32];
            snprintf(resp, sizeof(resp), "TIMEOUT:OK:%d\n", reposo_timeout_min);
            tud_cdc_n_write(0, resp, strlen(resp));
            tud_cdc_n_write_flush(0);
        }
        return;
    }
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

    // Instancia temporal para despertar pantallas
    HardwareOled main_oled_ctrl;

    while (true) {
        tud_task();

        // --- PROCESAR COMANDOS CDC (POLLING) ---
        if (tud_cdc_available()) {
            char tmp[64];
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

                // Ajuste nativo de volumen
                if (delta_izq > 0) {
                    for (int i = 0; i < delta_izq; i++) {
                        send_consumer_key(0x00E9); // Volume Increment
                    }
                } else {
                    for (int i = 0; i < -delta_izq; i++) {
                        send_consumer_key(0x00EA); // Volume Decrement
                    }
                }
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

                // Ajuste nativo de brillo
                if (delta_der > 0) {
                    for (int i = 0; i < delta_der; i++) {
                        send_consumer_key(0x00B0); // Brightness Increment
                    }
                } else {
                    for (int i = 0; i < -delta_der; i++) {
                        send_consumer_key(0x00B1); // Brightness Decrement
                    }
                }
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
                        send_consumer_key(0x00E2); // Mute
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
        int16_t wheel_delta = 0;
        if (wheel.sampleBlockAndGetDelta(wheel_delta)) {
            if (wheel_delta != 0) {
                activity_detected = true;
                
                if (current_mode == MODE_NORMAL) {
                    // Telemetría serie
                    char tel[32];
                    int len = snprintf(tel, sizeof(tel), "WHEEL:%d\n", wheel_delta);
                    tud_cdc_n_write(0, tel, len);
                    tud_cdc_n_write_flush(0);

                    // Scroll vertical nativo
                    int8_t scroll_val = (int8_t)(wheel_delta / 8);
                    if (scroll_val == 0 && wheel_delta != 0) {
                        scroll_val = (wheel_delta > 0) ? 1 : -1;
                    }
                    tud_hid_mouse_report(2, 0, 0, 0, scroll_val, 0);
                }
            }
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

        sleep_ms(1);
    }
    return 0;
}