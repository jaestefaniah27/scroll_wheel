#include "hid_adapter.h"
#include <string.h> // <-- para memset(), memcpy(), etc.
#include <EEPROM.h> // si usas EEPROM (AVR)

static sw_feature_config_t g_runtime; // copia opcional si te sirve

void sw_runtime_apply_config(const sw_feature_config_t *cfg)
{
    // Aplica en caliente (sensibilidades, modo activo, LUT botones, flags...)
    // Por ejemplo, copiar a variables globales ya existentes:
    g_runtime = *cfg;
    // TODO: vincular con tus variables reales
}

void sw_storage_save_config(const sw_feature_config_t *cfg)
{
    // Persistencia simple en EEPROM (tamaño 32 bytes)
    // Ajusta dirección base si ya guardas otras cosas
    const int base = 0x00;
    EEPROM.put(base, *cfg);
}

void sw_storage_load_defaults(sw_feature_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = 1;
    cfg->sens_global_cpi = 800;
    for (int i = 0; i < SW_MAX_MODES; i++)
        cfg->sens_mode[i] = 800;
    for (int b = 0; b < SW_MAX_BTNS; b++)
        cfg->btn_map[b] = b; // identidad
    cfg->flags = 0;

    // (Opcional) Si quieres leer EEPROM en vez de defaults:
    // sw_feature_config_t tmp; EEPROM.get(0x00, tmp);
    // if (tmp.version == 1) *cfg = tmp;
}
