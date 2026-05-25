#pragma once
#include <stdint.h>
#include "backend_select.h"

// ========= Ratón =========
#define MOUSE_LEFT 0x01
#define MOUSE_RIGHT 0x02
#define MOUSE_MIDDLE 0x04
#define MOUSE_BTN4 0x08
#define MOUSE_BTN5 0x10

// ========= Consumer Control (Usage Page 0x0C) =========
#define CC_MUTE 0x00E2
#define CC_VOL_UP 0x00E9
#define CC_VOL_DOWN 0x00EA

// ========= Teclado: modificadores (byte 0) =========
#define KEY_MOD_LCTRL 0x01
#define KEY_MOD_LSHIFT 0x02
#define KEY_MOD_LALT 0x04
#define KEY_MOD_LGUI 0x08
#define KEY_MOD_RCTRL 0x10
#define KEY_MOD_RSHIFT 0x20
#define KEY_MOD_RALT 0x40
#define KEY_MOD_RGUI 0x80

// ========= Teclado: keycodes básicos (Usage Page 0x07) =========
#define HID_KEY_NONE 0x00
#define HID_KEY_A 0x04
#define HID_KEY_B 0x05
#define HID_KEY_C 0x06
#define HID_KEY_D 0x07
#define HID_KEY_E 0x08
#define HID_KEY_F 0x09
#define HID_KEY_G 0x0A
#define HID_KEY_H 0x0B
#define HID_KEY_I 0x0C
#define HID_KEY_J 0x0D
#define HID_KEY_K 0x0E
#define HID_KEY_L 0x0F
#define HID_KEY_M 0x10
#define HID_KEY_N 0x11
#define HID_KEY_O 0x12
#define HID_KEY_P 0x13
#define HID_KEY_Q 0x14
#define HID_KEY_R 0x15
#define HID_KEY_S 0x16
#define HID_KEY_T 0x17
#define HID_KEY_U 0x18
#define HID_KEY_V 0x19
#define HID_KEY_W 0x1A
#define HID_KEY_X 0x1B
#define HID_KEY_Y 0x1C
#define HID_KEY_Z 0x1D
#define HID_KEY_1 0x1E
#define HID_KEY_2 0x1F
#define HID_KEY_3 0x20
#define HID_KEY_4 0x21
#define HID_KEY_5 0x22
#define HID_KEY_6 0x23
#define HID_KEY_7 0x24
#define HID_KEY_8 0x25
#define HID_KEY_9 0x26
#define HID_KEY_0 0x27
#define HID_KEY_ENTER 0x28
#define HID_KEY_ESC 0x29
#define HID_KEY_BSPACE 0x2A
#define HID_KEY_TAB 0x2B
#define HID_KEY_SPACE 0x2C

// ========= API común (idéntica para BLE/USB) =========
void hidSetup(void);

// ==== Configuración vía HID Feature (USB) ====
#ifndef SW_FEATURE_CFG_H
#define SW_FEATURE_CFG_H

#define SW_MAX_MODES 4
#define SW_MAX_BTNS 8

// Report IDs para la interfaz "Feature-only"
enum
{
    SW_RID_CFG = 0x05,
    SW_RID_CMD = 0x06
};

#pragma pack(push, 1)
typedef struct
{
    uint8_t version;                  // =1
    uint8_t active_mode;              // 0..SW_MAX_MODES-1
    uint16_t sens_global_cpi;         // unidades propias
    uint16_t sens_mode[SW_MAX_MODES]; // sensibilidad por modo
    uint8_t btn_map[SW_MAX_BTNS];     // LUT: físico->lógico
    uint8_t flags;                    // bits: invertir eje, smoothing, etc.
    uint8_t _pad[32 - (1 + 1 + 2 + 2 * SW_MAX_MODES + SW_MAX_BTNS + 1)];
} sw_feature_config_t;

typedef struct
{
    uint8_t report_id; // = SW_RID_CMD (0x06)
    uint8_t op;        // 0=SAVE, 1=RESET_DEFAULTS, 2=REQUEST_STATE
    uint8_t arg0;
    uint8_t arg1;
} sw_feature_cmd_t;
#pragma pack(pop)

// Hooks que implementas en tu lógica (no USB):
#ifdef __cplusplus
extern "C"
{
#endif
    void sw_runtime_apply_config(const sw_feature_config_t *cfg); // aplicar en caliente
    void sw_storage_save_config(const sw_feature_config_t *cfg);  // persistir EEPROM/flash
    void sw_storage_load_defaults(sw_feature_config_t *cfg);      // valores de fábrica
#ifdef __cplusplus
}
#endif

// Utilidades expuestas por el backend USB Feature:
#ifdef __cplusplus
extern "C"
{
#endif
    // Copia una vista de la config vigente (RAM) para tu app/lógica
    void hidUsbGetConfig(sw_feature_config_t *out_cfg);
    // Fuerza reset a defaults y aplica (útil si lo pides desde el propio firmware)
    void hidUsbResetToDefaults(void);
#ifdef __cplusplus
}
#endif

#endif // SW_FEATURE_CFG_H

// Ratón: envía 7 bytes (buttons, x, y, wheelV 16b, wheelH 16b)
void hidMouseSend(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH);

// Consumer Control (usages 16-bit: 0xE9/0xEA/0xE2 ...)
void hidConsumerPress(uint16_t usage);
void hidConsumerRelease(void);
void hidConsumerClick(uint16_t usage);

// Teclado (6KRO: [mods,res,k0..k5])
void hidKeyboardPress(uint8_t modifiers, uint8_t keycode);
void hidKeyboardRelease(uint8_t keycode);
void hidKeyboardReleaseAll(void);
void hidKeyboardWrite(uint8_t modifiers, uint8_t keycode);

void hidResetConection(void);
void hidResetConfig(void);
#ifdef __cplusplus
extern "C"
{
#endif
    void hidUsbFeatureBegin(void); // inicializa la interfaz HID "Feature-only"
#ifdef __cplusplus
}
#endif