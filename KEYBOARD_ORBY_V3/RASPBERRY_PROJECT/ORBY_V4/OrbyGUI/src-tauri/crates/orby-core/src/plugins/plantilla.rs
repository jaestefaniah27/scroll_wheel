// Sustitución de plantillas en las peticiones de un complemento declarativo.
//
// Solo sustitución: `{{settings.host}}`, `{{value}}`, `{{response.brightness}}`. Nada de
// expresiones ni de lógica. Un complemento es un fichero de configuración, y en cuanto
// esto admita condiciones o aritmética habremos reinventado un lenguaje de programación
// sin querer, con su intérprete y sus fallos.

use serde_json::Value;
use std::fmt;

#[derive(Debug, PartialEq, Eq)]
pub enum ErrorPlantilla {
    /// Una plantilla que no reconocemos. Se falla a propósito en vez de interpolar
    /// vacío: una petición con un hueco en blanco la interpreta el cacharro del otro
    /// lado como le parece, y el usuario ve "no funciona" sin ninguna pista.
    Desconocida(String),
    /// `{{response.x}}` usada donde no hay respuesta que mirar (por ejemplo en `run`).
    SinRespuesta(String),
}

impl fmt::Display for ErrorPlantilla {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Desconocida(t) => write!(f, "plantilla desconocida: {{{{{t}}}}}"),
            Self::SinRespuesta(t) => {
                write!(f, "{{{{{t}}}}} no se puede usar aquí: no hay respuesta")
            }
        }
    }
}

/// Lo que hay disponible para rellenar huecos.
#[derive(Default)]
pub struct Contexto<'a> {
    pub settings: Option<&'a Value>,
    pub value: Option<f64>,
    pub response: Option<&'a Value>,
}

/// Convierte un valor JSON en el texto que iría en una URL.
///
/// Los números se escriben sin `.0` sobrante: la API de un cacharro que espera un entero
/// se atraganta con `brightness=50.0`, y ese fallo es de los que solo se ven con el
/// aparato delante.
fn texto_de(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        Value::Number(n) => n.to_string(),
        Value::Bool(b) => b.to_string(),
        Value::Null => String::new(),
        otro => otro.to_string(),
    }
}

fn numero_a_texto(n: f64) -> String {
    if n.fract() == 0.0 && n.is_finite() {
        format!("{}", n as i64)
    } else {
        n.to_string()
    }
}

fn resuelve(nombre: &str, ctx: &Contexto) -> Result<String, ErrorPlantilla> {
    if nombre == "value" {
        // Sin valor es 0, no un error: una acción sin incremento (encender, apagar) se
        // declara igual que una con él.
        return Ok(numero_a_texto(ctx.value.unwrap_or(0.0)));
    }

    if let Some(campo) = nombre.strip_prefix("settings.") {
        let v = ctx.settings.and_then(|s| s.get(campo));
        return Ok(v.map(texto_de).unwrap_or_default());
    }

    if let Some(campo) = nombre.strip_prefix("response.") {
        let Some(resp) = ctx.response else {
            return Err(ErrorPlantilla::SinRespuesta(nombre.to_string()));
        };
        // Un campo ausente en la respuesta da cadena vacía, no error: el cacharro puede
        // no mandarlo y eso no es culpa del complemento.
        return Ok(resp.get(campo).map(texto_de).unwrap_or_default());
    }

    Err(ErrorPlantilla::Desconocida(nombre.to_string()))
}

/// Sustituye todos los `{{...}}` de `entrada`.
pub fn render(entrada: &str, ctx: &Contexto) -> Result<String, ErrorPlantilla> {
    let mut salida = String::with_capacity(entrada.len());
    let mut resto = entrada;

    while let Some(inicio) = resto.find("{{") {
        salida.push_str(&resto[..inicio]);
        let tras_abrir = &resto[inicio + 2..];

        let Some(fin) = tras_abrir.find("}}") else {
            // Un `{{` sin cerrar es una errata del manifiesto. Se deja tal cual para que
            // se vea en el mensaje de error en vez de comerse el resto de la cadena.
            salida.push_str(&resto[inicio..]);
            return Ok(salida);
        };

        let nombre = tras_abrir[..fin].trim();
        salida.push_str(&resuelve(nombre, ctx)?);
        resto = &tras_abrir[fin + 2..];
    }

    salida.push_str(resto);
    Ok(salida)
}

#[cfg(test)]
mod tests {
    use super::{render, Contexto, ErrorPlantilla};
    use serde_json::json;

    fn ctx_con_valor(v: f64) -> Contexto<'static> {
        Contexto { value: Some(v), ..Default::default() }
    }

    #[test]
    fn sustituye_un_ajuste() {
        let s = json!({ "host": "192.168.1.35:8080" });
        let ctx = Contexto { settings: Some(&s), ..Default::default() };
        assert_eq!(render("http://{{settings.host}}", &ctx).unwrap(), "http://192.168.1.35:8080");
    }

    #[test]
    fn el_valor_cero_y_los_negativos() {
        // Un giro hacia atrás es -5, y el 0 tiene que sobrevivir: si se colara un
        // "falsy" como en JS, apagar por valor exacto dejaría de funcionar.
        assert_eq!(render("{{value}}", &ctx_con_valor(0.0)).unwrap(), "0");
        assert_eq!(render("{{value}}", &ctx_con_valor(-5.0)).unwrap(), "-5");
        assert_eq!(render("{{value}}", &ctx_con_valor(50.0)).unwrap(), "50");
    }

    #[test]
    fn los_enteros_no_arrastran_decimales() {
        // `brightness=50.0` se lo come mal más de una API que espera un entero.
        assert_eq!(render("{{value}}", &ctx_con_valor(50.0)).unwrap(), "50");
    }

    #[test]
    fn sin_valor_es_cero() {
        assert_eq!(render("{{value}}", &Contexto::default()).unwrap(), "0");
    }

    #[test]
    fn un_ajuste_ausente_queda_vacio() {
        let s = json!({});
        let ctx = Contexto { settings: Some(&s), ..Default::default() };
        assert_eq!(render("x={{settings.host}}", &ctx).unwrap(), "x=");
    }

    #[test]
    fn lee_un_campo_de_la_respuesta() {
        let r = json!({ "brightness": 42, "on": true });
        let ctx = Contexto { response: Some(&r), ..Default::default() };
        assert_eq!(render("{{response.brightness}}", &ctx).unwrap(), "42");
        assert_eq!(render("{{response.on}}", &ctx).unwrap(), "true");
    }

    #[test]
    fn un_campo_ausente_de_la_respuesta_queda_vacio() {
        let r = json!({});
        let ctx = Contexto { response: Some(&r), ..Default::default() };
        assert_eq!(render("{{response.brillo}}", &ctx).unwrap(), "");
    }

    #[test]
    fn la_respuesta_no_se_puede_usar_donde_no_la_hay() {
        let err = render("{{response.x}}", &Contexto::default()).unwrap_err();
        assert_eq!(err, ErrorPlantilla::SinRespuesta("response.x".into()));
    }

    #[test]
    fn una_plantilla_desconocida_falla_en_vez_de_quedar_vacia() {
        // Interpolar vacío mandaría una petición mal formada y el fallo aparecería muy
        // lejos de su causa.
        let err = render("{{valor}}", &Contexto::default()).unwrap_err();
        assert_eq!(err, ErrorPlantilla::Desconocida("valor".into()));

        let err = render("{{ajustes.host}}", &Contexto::default()).unwrap_err();
        assert_eq!(err, ErrorPlantilla::Desconocida("ajustes.host".into()));
    }

    #[test]
    fn varias_plantillas_en_la_misma_cadena() {
        let s = json!({ "host": "lampara" });
        let ctx = Contexto { settings: Some(&s), value: Some(7.0), ..Default::default() };
        assert_eq!(
            render("http://{{settings.host}}/set?d={{value}}&t={{value}}", &ctx).unwrap(),
            "http://lampara/set?d=7&t=7"
        );
    }

    #[test]
    fn el_texto_sin_plantillas_pasa_tal_cual() {
        assert_eq!(render("/state", &Contexto::default()).unwrap(), "/state");
        assert_eq!(render("", &Contexto::default()).unwrap(), "");
    }

    #[test]
    fn se_admiten_espacios_dentro_de_las_llaves() {
        assert_eq!(render("{{ value }}", &ctx_con_valor(3.0)).unwrap(), "3");
    }
}
