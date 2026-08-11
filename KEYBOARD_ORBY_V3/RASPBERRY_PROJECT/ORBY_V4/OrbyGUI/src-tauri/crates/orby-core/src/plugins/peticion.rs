// Montar la petición HTTP de una acción a partir del manifiesto.
//
// Aquí no se manda nada: se decide QUÉ mandar. Separarlo es lo que permite comprobar sin
// red la parte que más fácil es equivocar —la interpolación, la codificación de la
// cadena de consulta, de dónde sale el tiempo máximo— y dejar para la cáscara lo único
// que necesita un socket.

use super::manifiesto::Manifiesto;
use super::plantilla::{render, Contexto, ErrorPlantilla};
use serde_json::Value;

/// Si el manifiesto no dice otra cosa. Sin tiempo máximo, un cacharro desenchufado deja
/// la ejecución de secuencias colgada hasta que agote el TCP: más de un minuto.
pub const TIMEOUT_POR_DEFECTO_MS: u64 = 5_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Peticion {
    pub metodo: String,
    pub url: String,
    pub timeout_ms: u64,
    /// Cuántas veces reintentar **además** de la primera. Las acciones nunca reintentan.
    pub reintentos: u32,
}

#[derive(Debug, PartialEq)]
pub enum ErrorPeticion {
    /// La acción o el visor no existen en el manifiesto.
    NoExiste(String),
    /// La acción existe pero no declara qué mandar.
    SinPeticion(String),
    Plantilla(ErrorPlantilla),
}

impl std::fmt::Display for ErrorPeticion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoExiste(op) => write!(f, "la acción «{op}» no existe en este complemento"),
            Self::SinPeticion(op) => write!(f, "la acción «{op}» no declara ninguna petición"),
            Self::Plantilla(e) => write!(f, "{e}"),
        }
    }
}

impl From<ErrorPlantilla> for ErrorPeticion {
    fn from(e: ErrorPlantilla) -> Self {
        Self::Plantilla(e)
    }
}

/// Codifica un valor para que quepa en una cadena de consulta.
///
/// Se hace a mano y no con una dependencia porque son cuatro reglas y así no entra un
/// crate entero por esto. Se deja sin tocar lo que no reserva ningún significado.
fn codifica(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char)
            }
            b' ' => out.push_str("%20"),
            otro => out.push_str(&format!("%{otro:02X}")),
        }
    }
    out
}

fn busca<'a>(lista: Option<&'a Value>, op: &str) -> Option<&'a Value> {
    lista?.as_array()?.iter().find(|a| a.get("op").and_then(Value::as_str) == Some(op))
}

/// Monta la petición de una acción (`run`) o de un visor (`read`).
///
/// `valor` es el incremento del giro o el número elegido al asignar la acción; `ajustes`
/// son los que el usuario haya guardado.
pub fn construye(
    m: &Manifiesto,
    op: &str,
    valor: Option<f64>,
    ajustes: &Value,
) -> Result<Peticion, ErrorPeticion> {
    let entrada = busca(m.crudo.get("actions"), op)
        .or_else(|| busca(m.crudo.get("views"), op))
        .ok_or_else(|| ErrorPeticion::NoExiste(op.to_string()))?;

    let req = entrada
        .get("request")
        .ok_or_else(|| ErrorPeticion::SinPeticion(op.to_string()))?;

    construye_desde(m, req, valor, ajustes, 0)
}

/// La petición de la comprobación de los ajustes (`test`), que es la única que reintenta.
pub fn construye_test(m: &Manifiesto, ajustes: &Value) -> Result<Peticion, ErrorPeticion> {
    let test = m
        .crudo
        .get("test")
        .ok_or_else(|| ErrorPeticion::NoExiste("test".into()))?;
    let req = test
        .get("request")
        .ok_or_else(|| ErrorPeticion::SinPeticion("test".into()))?;

    // Leer es inofensivo, así que se puede reintentar: si el cacharro estaba ocupado, el
    // primer intento se queda sin respuesta y la tarjeta de ajustes diría «no responde»
    // con el aparato perfectamente vivo. Las acciones NO usan esta vía.
    let reintentos = test.get("retries").and_then(Value::as_u64).unwrap_or(0) as u32;

    construye_desde(m, req, None, ajustes, reintentos)
}

fn construye_desde(
    m: &Manifiesto,
    req: &Value,
    valor: Option<f64>,
    ajustes: &Value,
    reintentos: u32,
) -> Result<Peticion, ErrorPeticion> {
    let global = m.crudo.get("request");
    let ctx = Contexto { settings: Some(ajustes), value: valor, response: None };

    let base = global
        .and_then(|g| g.get("base"))
        .and_then(Value::as_str)
        .unwrap_or("");
    let path = req.get("path").and_then(Value::as_str).unwrap_or("");

    let mut url = format!("{}{}", render(base, &ctx)?, render(path, &ctx)?);

    if let Some(Value::Object(q)) = req.get("query") {
        let mut partes = Vec::with_capacity(q.len());
        for (clave, plantilla) in q {
            // Las claves del mapa de serde_json vienen ordenadas, así que la cadena sale
            // estable: sin eso, dos ejecuciones darían URLs distintas y las pruebas
            // saldrían intermitentes.
            let bruto = match plantilla {
                Value::String(s) => render(s, &ctx)?,
                otro => otro.to_string(),
            };
            partes.push(format!("{}={}", codifica(clave), codifica(&bruto)));
        }
        if !partes.is_empty() {
            url.push('?');
            url.push_str(&partes.join("&"));
        }
    }

    let timeout_ms = req
        .get("timeoutMs")
        .or_else(|| global.and_then(|g| g.get("timeoutMs")))
        .and_then(Value::as_u64)
        .unwrap_or(TIMEOUT_POR_DEFECTO_MS);

    let metodo = req
        .get("method")
        .and_then(Value::as_str)
        .unwrap_or("GET")
        .to_uppercase();

    Ok(Peticion { metodo, url, timeout_ms, reintentos })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugins::Manifiesto;
    use serde_json::json;

    const LAMPDESK: &str = include_str!("../../tests/fixtures/lampdesk-api2.plugin.json");

    fn lampdesk() -> Manifiesto {
        Manifiesto::desde_json(LAMPDESK).unwrap()
    }

    fn ajustes() -> Value {
        json!({ "host": "192.168.1.35:8080" })
    }

    #[test]
    fn monta_la_url_de_encender() {
        let p = construye(&lampdesk(), "on", None, &ajustes()).unwrap();
        assert_eq!(p.url, "http://192.168.1.35:8080/set?on=1");
        assert_eq!(p.metodo, "GET");
    }

    #[test]
    fn interpola_el_incremento_del_giro() {
        let p = construye(&lampdesk(), "brightness_delta", Some(10.0), &ajustes()).unwrap();
        assert_eq!(p.url, "http://192.168.1.35:8080/set?dbrightness=10");
    }

    #[test]
    fn un_giro_hacia_atras_manda_un_negativo() {
        let p = construye(&lampdesk(), "brightness_delta", Some(-15.0), &ajustes()).unwrap();
        assert_eq!(p.url, "http://192.168.1.35:8080/set?dbrightness=-15");
    }

    #[test]
    fn el_valor_exacto_no_arrastra_decimales() {
        // `brightness=50.0` se lo come mal más de una API que espera un entero.
        let p = construye(&lampdesk(), "brightness_set", Some(50.0), &ajustes()).unwrap();
        assert_eq!(p.url, "http://192.168.1.35:8080/set?brightness=50");
    }

    #[test]
    fn el_tiempo_maximo_sale_del_manifiesto() {
        let p = construye(&lampdesk(), "on", None, &ajustes()).unwrap();
        assert_eq!(p.timeout_ms, 4000);
    }

    #[test]
    fn las_acciones_no_reintentan_nunca() {
        // Un incremento repetido se aplicaría dos veces.
        for op in ["on", "off", "toggle", "brightness_delta", "brightness_set"] {
            let p = construye(&lampdesk(), op, Some(5.0), &ajustes()).unwrap();
            assert_eq!(p.reintentos, 0, "«{op}» no debe reintentar");
        }
    }

    #[test]
    fn la_comprobacion_de_ajustes_si_reintenta() {
        let p = construye_test(&lampdesk(), &ajustes()).unwrap();
        assert_eq!(p.url, "http://192.168.1.35:8080/state");
        assert_eq!(p.reintentos, 1);
    }

    #[test]
    fn los_visores_tambien_montan_su_peticion() {
        let p = construye(&lampdesk(), "brightness_view", None, &ajustes()).unwrap();
        assert_eq!(p.url, "http://192.168.1.35:8080/state");
    }

    #[test]
    fn una_accion_que_no_existe_da_un_error_que_se_entiende() {
        let e = construye(&lampdesk(), "inventada", None, &ajustes()).unwrap_err();
        assert_eq!(e, ErrorPeticion::NoExiste("inventada".into()));
        assert!(e.to_string().contains("inventada"));
    }

    #[test]
    fn la_direccion_se_codifica() {
        let m = Manifiesto::desde_json(
            &json!({
                "id": "prueba", "name": "P", "apiVersion": 2,
                "request": { "base": "http://x" },
                "actions": [{
                    "op": "a", "label": "A",
                    "request": { "path": "/set", "query": { "texto": "{{settings.t}}" } }
                }]
            })
            .to_string(),
        )
        .unwrap();

        let p = construye(&m, "a", None, &json!({ "t": "hola mundo & cía" })).unwrap();
        assert_eq!(p.url, "http://x/set?texto=hola%20mundo%20%26%20c%C3%ADa");
    }

    #[test]
    fn una_plantilla_desconocida_impide_mandar_la_peticion() {
        // Preferimos no mandar nada a mandar una petición con un hueco en blanco que el
        // cacharro del otro lado interpretaría como le pareciera.
        let m = Manifiesto::desde_json(
            &json!({
                "id": "prueba", "name": "P", "apiVersion": 2,
                "request": { "base": "http://x" },
                "actions": [{
                    "op": "a", "label": "A",
                    "request": { "path": "/set", "query": { "v": "{{ajustes.host}}" } }
                }]
            })
            .to_string(),
        )
        .unwrap();

        assert!(matches!(
            construye(&m, "a", None, &json!({})),
            Err(ErrorPeticion::Plantilla(_))
        ));
    }

    #[test]
    fn sin_tiempo_maximo_declarado_se_pone_uno() {
        let m = Manifiesto::desde_json(
            &json!({
                "id": "prueba", "name": "P", "apiVersion": 2,
                "request": { "base": "http://x" },
                "actions": [{ "op": "a", "label": "A", "request": { "path": "/y" } }]
            })
            .to_string(),
        )
        .unwrap();

        let p = construye(&m, "a", None, &json!({})).unwrap();
        assert_eq!(p.timeout_ms, TIMEOUT_POR_DEFECTO_MS);
    }

    #[test]
    fn se_puede_pedir_otro_metodo() {
        let m = Manifiesto::desde_json(
            &json!({
                "id": "prueba", "name": "P", "apiVersion": 2,
                "request": { "base": "http://x" },
                "actions": [{
                    "op": "a", "label": "A",
                    "request": { "path": "/y", "method": "post" }
                }]
            })
            .to_string(),
        )
        .unwrap();

        assert_eq!(construye(&m, "a", None, &json!({})).unwrap().metodo, "POST");
    }

    #[test]
    fn las_siete_acciones_de_lampdesk_montan_su_url() {
        // El complemento entero, de punta a punta, sin red.
        let esperado = [
            ("toggle", "http://192.168.1.35:8080/set?on=toggle"),
            ("on", "http://192.168.1.35:8080/set?on=1"),
            ("off", "http://192.168.1.35:8080/set?on=0"),
            ("brightness_delta", "http://192.168.1.35:8080/set?dbrightness=5"),
            ("color_delta", "http://192.168.1.35:8080/set?dcolor=5"),
            ("brightness_set", "http://192.168.1.35:8080/set?brightness=5"),
            ("color_set", "http://192.168.1.35:8080/set?color=5"),
        ];

        let m = lampdesk();
        for (op, url) in esperado {
            let p = construye(&m, op, Some(5.0), &ajustes()).unwrap();
            assert_eq!(p.url, url, "la URL de «{op}» no es la esperada");
        }
    }
}
