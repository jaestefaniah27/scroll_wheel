// Ajustes que solo viven en el PC: el firmware no sabe nada de aplicaciones de
// Windows. Port de electron/config.js, con su mismo esquema y su misma fusión.
//
// El fichero es el mismo (`orby-config.json` en la carpeta de datos del usuario), así
// que una instalación de Electron y una de Tauri se leen la una a la otra. Eso es lo
// que permite la prueba de paridad: copia de seguridad hecha con una, restaurada con
// la otra.

use serde_json::{json, Map, Value};

/// Claves que se sustituyen enteras en vez de fusionarse.
///
/// El espejo describe un estado completo: fusionándolo, un icono borrado o un perfil
/// que ya no está seguirían apareciendo en el archivo para siempre.
const REPLACE_WHOLE: &[&str] = &["deviceMirror"];

pub fn defaults() -> Value {
    json!({
        "autoProfile": { "enabled": false, "rules": [], "fallback": null },
        "profileVariants": [],
        "macros": [],
        "macrosOnDevice": null,
        "plugins": {},
        "wheelDial": { "invert": true, "offsetDeg": 62, "marker": "dot" },
        "deviceMirror": null,
    })
}

/// Fusiona `patch` sobre `base` sin tocar ninguno de los dos.
///
/// Solo los objetos se fusionan en profundidad. Los arrays se sustituyen enteros: un
/// array fusionado por índice dejaría vivos los elementos que el usuario acaba de
/// borrar.
pub fn merge(base: &Value, patch: &Value) -> Value {
    let vacio = Map::new();
    let base_obj = base.as_object().unwrap_or(&vacio);
    let mut out = base_obj.clone();

    let Some(patch_obj) = patch.as_object() else {
        return Value::Object(out);
    };

    for (key, value) in patch_obj {
        let fusionable =
            value.is_object() && !REPLACE_WHOLE.contains(&key.as_str());

        out.insert(
            key.clone(),
            if fusionable {
                let hueco = Value::Object(Map::new());
                merge(base_obj.get(key).unwrap_or(&hueco), value)
            } else {
                value.clone()
            },
        );
    }

    Value::Object(out)
}

#[cfg(test)]
mod tests {
    use super::{defaults, merge};
    use serde_json::json;

    #[test]
    fn los_valores_por_defecto_tienen_la_forma_esperada() {
        let d = defaults();
        assert!(d["macros"].is_array());
        assert!(d["macrosOnDevice"].is_null());
        assert!(d["deviceMirror"].is_null());
        assert_eq!(d["wheelDial"]["offsetDeg"], 62);
    }

    #[test]
    fn fusiona_en_profundidad_sin_mutar_la_base() {
        let base = json!({ "wheelDial": { "invert": true, "offsetDeg": 62 } });
        let out = merge(&base, &json!({ "wheelDial": { "offsetDeg": 10 } }));

        assert_eq!(out["wheelDial"]["offsetDeg"], 10);
        // Lo que el parche no menciona sobrevive.
        assert_eq!(out["wheelDial"]["invert"], true);
        // Y la base no se ha tocado.
        assert_eq!(base["wheelDial"]["offsetDeg"], 62);
    }

    #[test]
    fn el_espejo_se_sustituye_entero() {
        // Fusionarlo dejaría el icono del hueco 1 vivo para siempre, aunque el
        // usuario lo haya borrado.
        let base = json!({ "deviceMirror": { "icons": { "0:0:1": "ff" } } });
        let out = merge(&base, &json!({ "deviceMirror": { "icons": {} } }));

        assert_eq!(out["deviceMirror"], json!({ "icons": {} }));
    }

    #[test]
    fn los_arrays_se_sustituyen_enteros() {
        let base = json!({ "macros": [{ "id": 1 }, { "id": 2 }] });
        let out = merge(&base, &json!({ "macros": [{ "id": 9 }] }));

        assert_eq!(out["macros"], json!([{ "id": 9 }]));
    }

    #[test]
    fn un_valor_nulo_borra_en_vez_de_fusionar() {
        // `null` no es un objeto, así que sustituye. Es como se apaga el espejo.
        let base = json!({ "deviceMirror": { "savedAt": 1 } });
        let out = merge(&base, &json!({ "deviceMirror": null }));

        assert!(out["deviceMirror"].is_null());
    }
}
