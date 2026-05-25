#pragma once
#include <stdint.h>
#include "hid_adapter.h" // define sw_feature_config_t y los Report IDs

#ifdef __cplusplus
extern "C"
{
#endif

    // Implementadas en config_runtime.cpp
    void sw_runtime_apply_config(const sw_feature_config_t *cfg);
    void sw_storage_save_config(const sw_feature_config_t *cfg);
    void sw_storage_load_defaults(sw_feature_config_t *cfg);

    // (Opcional, útil para depurar/GUI interna)
    // Devuelve una copia de la config en RAM si en tu .cpp mantienes g_runtime
    void sw_get_runtime_config(sw_feature_config_t *out_cfg);

#ifdef __cplusplus
}
#endif
