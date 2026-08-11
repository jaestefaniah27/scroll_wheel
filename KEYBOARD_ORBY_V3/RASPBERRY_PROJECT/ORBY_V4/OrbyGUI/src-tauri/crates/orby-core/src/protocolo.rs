// Lo que se puede deducir de una línea suelta del CDC, sin estado.
//
// La correlación entre petición y respuesta NO está aquí: vive en src/device.js, con
// su cola de `pending`, y ahí se queda. Este módulo solo cubre lo que el proceso de la
// app necesita mirar por su cuenta mientras reenvía todas las líneas al renderer.

use std::collections::BTreeMap;

/// Lo que dice el teclado al saludar:
/// `ORBY_V4:FW=4.5:KEYS=12:OLEDS=10:ENCODERS=2:MODE=NORMAL`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    pub device: String,
    pub campos: BTreeMap<String, String>,
    pub raw: String,
}

impl DeviceInfo {
    pub fn get(&self, clave: &str) -> Option<&str> {
        self.campos.get(clave).map(String::as_str)
    }
}

/// Reconoce el saludo. Devuelve `None` si la línea no lo es.
///
/// Las claves se guardan en minúscula porque así las dejaba `_parseDeviceInfo` en
/// electron/serial.js y así las lee compat.js: `FW=` acaba en `fw`.
pub fn parse_device_info(linea: &str) -> Option<DeviceInfo> {
    let linea = linea.trim();
    if !linea.starts_with("ORBY") {
        return None;
    }

    let mut partes = linea.split(':');
    let device = partes.next().unwrap_or("ORBY_V4").to_string();

    let campos = linea
        .split(':')
        .filter_map(|p| p.split_once('='))
        .map(|(k, v)| {
            // El original parte por '=' y se queda con el segundo trozo, así que un
            // valor con '=' dentro se corta ahí. Se replica para no cambiar en
            // silencio lo que ve la app.
            let v = v.split('=').next().unwrap_or("");
            (k.to_lowercase(), v.to_string())
        })
        .collect();

    Some(DeviceInfo { device, campos, raw: linea.to_string() })
}

/// Un `MACRO:<id>` y nada más: es el teclado pidiendo que el PC ejecute la secuencia.
///
/// Tiene que ser estricto porque `MACRO:OK:…`, `MACRO:<id>:STEP:…` y `MACRO:<id>:END`
/// comparten prefijo y son respuestas a peticiones de la app, no disparos.
pub fn macro_trigger(linea: &str) -> Option<u32> {
    let resto = linea.trim().strip_prefix("MACRO:")?;
    if resto.is_empty() || !resto.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    resto.parse().ok()
}

/// ¿Este comando cambia algo en el teclado?
///
/// Solo estos se dejan por escrito en `orby.log`: la telemetría llenaría el fichero y
/// taparía justo lo que se va a buscar ahí, que es por qué dejó de detectarse.
pub fn cambia_algo(cmd: &str) -> bool {
    const PREFIJOS: &[&str] = &[
        "SET_", "DEL_", "ADD_", "SAVE_STATE", "RESET_DEFAULTS", "OLED_CLEAR",
        "MACRO_CLEAR", "MACRO_TRUNC", "BOOTSEL",
    ];
    PREFIJOS.iter().any(|p| cmd.starts_with(p))
}

#[cfg(test)]
mod tests {
    use super::{cambia_algo, macro_trigger, parse_device_info};

    #[test]
    fn lee_el_saludo_con_las_claves_en_minuscula() {
        let info =
            parse_device_info("ORBY_V4:FW=4.5:KEYS=12:OLEDS=10:MACROS=1:MODE=NORMAL")
                .expect("debería reconocer el saludo");

        assert_eq!(info.device, "ORBY_V4");
        assert_eq!(info.get("fw"), Some("4.5"));
        assert_eq!(info.get("keys"), Some("12"));
        assert_eq!(info.get("mode"), Some("NORMAL"));
        // Sin '=' no es un campo, es el nombre del cacharro.
        assert_eq!(info.get("orby_v4"), None);
    }

    #[test]
    fn la_telemetria_no_es_un_saludo() {
        assert!(parse_device_info("KEY_EV:3:1").is_none());
        assert!(parse_device_info("EV:CTX:0:1:3").is_none());
    }

    #[test]
    fn solo_dispara_con_macro_y_un_numero_pelado() {
        assert_eq!(macro_trigger("MACRO:7"), Some(7));
        assert_eq!(macro_trigger("MACRO:12"), Some(12));

        // Estas son respuestas a peticiones de la app, no disparos. Confundirlas
        // ejecutaría una secuencia cada vez que se lee una.
        assert_eq!(macro_trigger("MACRO:OK:3:0"), None);
        assert_eq!(macro_trigger("MACRO:7:STEP:0:1:2:3:4:5"), None);
        assert_eq!(macro_trigger("MACRO:7:END"), None);
        assert_eq!(macro_trigger("MACRO:TEST:STARTED:7"), None);
        assert_eq!(macro_trigger("MACRO:"), None);
    }

    #[test]
    fn distingue_los_comandos_que_tocan_el_teclado() {
        assert!(cambia_algo("SET_KEYMAP:0:3:0:4"));
        assert!(cambia_algo("SAVE_STATE"));
        assert!(cambia_algo("BOOTSEL"));
        assert!(cambia_algo("MACRO_CLEAR:3"));

        assert!(!cambia_algo("GET_PROFILE:0"));
        assert!(!cambia_algo("ACK"));
        assert!(!cambia_algo("GET_STATE"));
    }
}
