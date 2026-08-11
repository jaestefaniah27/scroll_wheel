// El manifiesto de un complemento y el descriptor que se le manda al renderer.
//
// Un complemento ya no es un programa: es este fichero. Lo que antes hacía `main.js` en
// Node y sin caja de arena, ahora se declara y lo ejecuta Rust.
//
// El descriptor que sale de aquí tiene que ser BIT A BIT el mismo que producía
// electron/plugins.js, porque src/plugins.js y la tarjeta de Ajustes ya lo consumen y no
// se tocan. Los valores por defecto de más abajo (icono `plug`, versión `—`, destino
// `key`, icono de visor `oled`…) no son gustos: son los que ya espera la interfaz.

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

/// La versión de API que sabe hablar esta app.
pub const API_VERSION: u32 = 2;

const TIPOS_DE_CAMPO: &[&str] = &["text", "number", "password", "toggle", "select"];

#[derive(Debug, PartialEq, Eq)]
pub enum ErrorManifiesto {
    IdInvalido(String),
    SinNombre,
    ApiIncompatible { encontrada: u32 },
    TipoDesconocido(String),
    Json(String),
}

impl std::fmt::Display for ErrorManifiesto {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IdInvalido(id) => write!(
                f,
                "el identificador «{id}» no vale: solo minúsculas, dígitos y guiones, \
                 entre 2 y 39 caracteres, y sin empezar por guion"
            ),
            Self::SinNombre => write!(f, "al complemento le falta el campo «name»"),
            // Mensaje claro a propósito: los complementos de la API 1 eran programas de
            // Node y ya no hay quién los ejecute. Mejor decirlo al cargar que reventar a
            // mitad de una pulsación.
            Self::ApiIncompatible { encontrada } => write!(
                f,
                "este complemento usa la versión {encontrada} de la API y esta versión de \
                 OrbyGUI habla la {API_VERSION}. Hace falta una versión actualizada."
            ),
            Self::TipoDesconocido(t) => {
                write!(f, "el tipo de complemento «{t}» no existe: usa «http» o «process»")
            }
            Self::Json(e) => write!(f, "el plugin.json no se puede leer: {e}"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Tipo {
    /// Sin código: el manifiesto declara qué petición mandar.
    Http,
    /// Un programa externo que habla el protocolo por líneas.
    Process,
}

#[derive(Debug, Clone)]
pub struct Manifiesto {
    pub id: String,
    pub name: String,
    pub tipo: Tipo,
    pub version: String,
    pub description: String,
    pub author: String,
    pub icon: String,
    pub actions: Vec<Accion>,
    pub views: Vec<Visor>,
    pub settings: Option<Ajustes>,
    pub tiene_test: bool,
    /// Lo crudo, para que el motor de peticiones lea sus `request` sin volver a parsear.
    pub crudo: Value,
}

#[derive(Debug, Clone)]
pub struct Accion {
    pub op: String,
    pub label: String,
    pub targets: Vec<String>,
    pub step: f64,
    pub hint: String,
    pub value: Option<Rango>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rango {
    pub min: f64,
    pub max: f64,
    pub step: f64,
    pub default: f64,
}

#[derive(Debug, Clone)]
pub struct Visor {
    pub op: String,
    pub label: String,
    pub icon: String,
}

#[derive(Debug, Clone)]
pub struct Ajustes {
    pub description: String,
    pub fields: Vec<Campo>,
}

#[derive(Debug, Clone)]
pub struct Campo {
    pub key: String,
    pub label: String,
    pub tipo: String,
    pub placeholder: String,
    pub hint: String,
    pub options: Vec<(String, String)>,
}

/// Mismo criterio que `ID_RE` en electron/plugins.js: el id es el nombre de la carpeta y
/// la clave de sus ajustes, así que cambiarlo pierde lo que el usuario tenga asignado.
pub fn id_valido(id: &str) -> bool {
    let n = id.chars().count();
    if !(2..=39).contains(&n) {
        return false;
    }
    let mut cs = id.chars();
    let primero = cs.next().unwrap();
    if !(primero.is_ascii_lowercase() || primero.is_ascii_digit()) {
        return false;
    }
    cs.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
}

fn texto(v: Option<&Value>, por_defecto: &str) -> String {
    match v {
        Some(Value::String(s)) if !s.is_empty() => s.clone(),
        Some(Value::Number(n)) => n.to_string(),
        _ => por_defecto.to_string(),
    }
}

fn numero(v: Option<&Value>) -> Option<f64> {
    v.and_then(Value::as_f64).filter(|n| n.is_finite())
}

/// Un número para el descriptor, entero si no tiene decimales.
///
/// JSON no distingue, pero serde sí: emitiría `step: 0.0` donde Electron emitía `step: 0`.
/// Se iguala para que la comparación con el descriptor de Electron sea exacta y no
/// «equivalente», y para que lo que llega al renderer sea idéntico byte a byte.
fn num(n: f64) -> Value {
    if n.fract() == 0.0 && n.abs() < 9e15 {
        Value::from(n as i64)
    } else {
        Value::from(n)
    }
}

/// Rango de un valor exacto («Fijar brillo»). Sin `min` y `max` no hay nada que elegir,
/// y la acción se trata como las de un solo clic.
fn rango(v: Option<&Value>) -> Option<Rango> {
    let v = v?;
    let min = numero(v.get("min"))?;
    let max = numero(v.get("max"))?;
    Some(Rango {
        min,
        max,
        step: numero(v.get("step")).filter(|s| *s > 0.0).unwrap_or(1.0),
        default: numero(v.get("default")).unwrap_or(min),
    })
}

fn acciones(v: Option<&Value>) -> Vec<Accion> {
    let Some(Value::Array(items)) = v else { return Vec::new() };
    items
        .iter()
        // Sin `op` o sin `label` no se puede ni ofrecer en el editor: se descarta en vez
        // de enseñar una entrada en blanco.
        .filter(|a| a.get("op").is_some() && a.get("label").is_some())
        .map(|a| {
            let targets = match a.get("targets") {
                Some(Value::Array(t)) if !t.is_empty() => {
                    t.iter().map(|x| texto(Some(x), "")).collect()
                }
                _ => vec!["key".to_string()],
            };
            Accion {
                op: texto(a.get("op"), ""),
                label: texto(a.get("label"), ""),
                targets,
                step: numero(a.get("step")).unwrap_or(0.0),
                hint: texto(a.get("hint"), ""),
                value: rango(a.get("value")),
            }
        })
        .collect()
}

fn visores(v: Option<&Value>) -> Vec<Visor> {
    let Some(Value::Array(items)) = v else { return Vec::new() };
    items
        .iter()
        .filter(|x| x.get("op").is_some() && x.get("label").is_some())
        .map(|x| Visor {
            op: texto(x.get("op"), ""),
            label: texto(x.get("label"), ""),
            icon: texto(x.get("icon"), "oled"),
        })
        .collect()
}

fn ajustes(v: Option<&Value>) -> Result<Option<Ajustes>, ErrorManifiesto> {
    let Some(s) = v else { return Ok(None) };

    let mut fields = Vec::new();
    if let Some(Value::Array(items)) = s.get("fields") {
        for f in items {
            let Some(key) = f.get("key").and_then(Value::as_str) else { continue };
            let tipo = texto(f.get("type"), "text");
            if !TIPOS_DE_CAMPO.contains(&tipo.as_str()) {
                return Err(ErrorManifiesto::TipoDesconocido(tipo));
            }
            let options = match f.get("options") {
                Some(Value::Array(os)) => os
                    .iter()
                    .map(|o| {
                        let valor = texto(o.get("value"), "");
                        let etiqueta = texto(o.get("label"), &valor);
                        (valor, etiqueta)
                    })
                    .collect(),
                _ => Vec::new(),
            };
            fields.push(Campo {
                key: key.to_string(),
                label: texto(f.get("label"), key),
                tipo,
                placeholder: texto(f.get("placeholder"), ""),
                hint: texto(f.get("hint"), ""),
                options,
            });
        }
    }

    Ok(Some(Ajustes { description: texto(s.get("description"), ""), fields }))
}

impl Manifiesto {
    pub fn desde_json(texto_json: &str) -> Result<Self, ErrorManifiesto> {
        let v: Value = serde_json::from_str(texto_json)
            .map_err(|e| ErrorManifiesto::Json(e.to_string()))?;
        Self::desde_valor(v)
    }

    pub fn desde_valor(v: Value) -> Result<Self, ErrorManifiesto> {
        let id = v.get("id").and_then(Value::as_str).unwrap_or("").to_string();
        if !id_valido(&id) {
            return Err(ErrorManifiesto::IdInvalido(id));
        }

        let Some(name) = v.get("name").and_then(Value::as_str).filter(|s| !s.is_empty())
        else {
            return Err(ErrorManifiesto::SinNombre);
        };

        // Se comprueba antes que nada de lo demás: un complemento de la API 1 trae
        // `main.js` y campos que aquí no significan lo mismo, y validarlos daría un
        // mensaje que despista.
        let api = v.get("apiVersion").and_then(Value::as_u64).unwrap_or(0) as u32;
        if api != API_VERSION {
            return Err(ErrorManifiesto::ApiIncompatible { encontrada: api });
        }

        let tipo = match v.get("kind").and_then(Value::as_str).unwrap_or("http") {
            "http" => Tipo::Http,
            "process" => Tipo::Process,
            otro => return Err(ErrorManifiesto::TipoDesconocido(otro.to_string())),
        };

        Ok(Manifiesto {
            id,
            name: name.to_string(),
            tipo,
            version: texto(v.get("version"), "—"),
            description: texto(v.get("description"), ""),
            author: texto(v.get("author"), ""),
            icon: texto(v.get("icon"), "plug"),
            actions: acciones(v.get("actions")),
            views: visores(v.get("views")),
            settings: ajustes(v.get("settings"))?,
            tiene_test: v.get("test").is_some(),
            crudo: v,
        })
    }

    /// Lo que ve el renderer. Misma forma exacta que producía electron/plugins.js.
    pub fn descriptor(&self, enabled: bool, values: &Value) -> Value {
        let acciones: Vec<Value> = self
            .actions
            .iter()
            .map(|a| {
                let mut m = Map::new();
                m.insert("op".into(), a.op.clone().into());
                m.insert("label".into(), a.label.clone().into());
                m.insert("targets".into(), a.targets.clone().into());
                m.insert("step".into(), num(a.step));
                m.insert("hint".into(), a.hint.clone().into());
                m.insert(
                    "value".into(),
                    match a.value {
                        Some(r) => serde_json::json!({
                            "min": num(r.min), "max": num(r.max),
                            "step": num(r.step), "default": num(r.default)
                        }),
                        None => Value::Null,
                    },
                );
                Value::Object(m)
            })
            .collect();

        let visores: Vec<Value> = self
            .views
            .iter()
            .map(|v| serde_json::json!({ "op": v.op, "label": v.label, "icon": v.icon }))
            .collect();

        let ajustes = match &self.settings {
            None => Value::Null,
            Some(s) => {
                let campos: Vec<Value> = s
                    .fields
                    .iter()
                    .map(|f| {
                        let opciones: Vec<Value> = f
                            .options
                            .iter()
                            .map(|(v, l)| serde_json::json!({ "value": v, "label": l }))
                            .collect();
                        serde_json::json!({
                            "key": f.key, "label": f.label, "type": f.tipo,
                            "placeholder": f.placeholder, "hint": f.hint, "options": opciones
                        })
                    })
                    .collect();
                serde_json::json!({
                    "description": s.description,
                    "hasTest": self.tiene_test,
                    "fields": campos
                })
            }
        };

        serde_json::json!({
            "id": self.id,
            "name": self.name,
            "version": self.version,
            "description": self.description,
            "author": self.author,
            "icon": self.icon,
            "enabled": enabled,
            "error": Value::Null,
            "actions": acciones,
            "views": visores,
            // En la API 1 era «exporta una función read()». Ahora un complemento sabe
            // leer si declara visores: es lo mismo dicho con datos.
            "hasRead": !self.views.is_empty(),
            "settings": ajustes,
            "values": values.clone(),
        })
    }

    /// Descriptor de un complemento que no cargó. La interfaz lo enseña en Ajustes con
    /// su error, pero no deja asignarlo a ninguna tecla.
    pub fn descriptor_roto(id: &str, error: &str) -> Value {
        serde_json::json!({
            "id": id, "name": id, "version": "—", "description": "", "author": "",
            "icon": "plug", "enabled": false, "error": error,
            "actions": [], "views": [], "hasRead": false,
            "settings": Value::Null, "values": {},
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{id_valido, ErrorManifiesto, Manifiesto, Tipo};
    use serde_json::json;

    fn base() -> serde_json::Value {
        json!({ "id": "prueba", "name": "Prueba", "apiVersion": 2 })
    }

    #[test]
    fn acepta_un_manifiesto_minimo() {
        let m = Manifiesto::desde_valor(base()).unwrap();
        assert_eq!(m.id, "prueba");
        // Sin `kind` se asume `http`: es el caso fácil y el que no necesita programa.
        assert_eq!(m.tipo, Tipo::Http);
        // Los valores por defecto son los que ya espera la interfaz.
        assert_eq!(m.version, "—");
        assert_eq!(m.icon, "plug");
    }

    #[test]
    fn rechaza_identificadores_que_no_valen() {
        // El id es el nombre de la carpeta y la clave de sus ajustes.
        for malo in ["Mayusculas", "con espacio", "-empieza-por-guion", "a", "", "acentué"] {
            let mut v = base();
            v["id"] = json!(malo);
            assert!(
                matches!(Manifiesto::desde_valor(v), Err(ErrorManifiesto::IdInvalido(_))),
                "«{malo}» no debería valer como id"
            );
        }
        assert!(id_valido("lampdesk"));
        assert!(id_valido("orby-obs-2"));
    }

    #[test]
    fn rechaza_la_api_1_con_un_mensaje_que_se_entiende() {
        // Los complementos de la API 1 eran programas de Node y ya no hay quién los
        // ejecute. Tiene que decirlo al cargar, no reventar a mitad de una pulsación.
        let mut v = base();
        v["apiVersion"] = json!(1);

        let err = Manifiesto::desde_valor(v).unwrap_err();
        assert_eq!(err, ErrorManifiesto::ApiIncompatible { encontrada: 1 });

        let texto = err.to_string();
        assert!(texto.contains("versión 1"), "el mensaje debería decir qué versión trae: {texto}");
        assert!(texto.contains("actualizada"), "y qué hacer al respecto: {texto}");
    }

    #[test]
    fn rechaza_un_manifiesto_sin_nombre() {
        let mut v = base();
        v.as_object_mut().unwrap().remove("name");
        assert_eq!(Manifiesto::desde_valor(v).unwrap_err(), ErrorManifiesto::SinNombre);
    }

    #[test]
    fn rechaza_un_tipo_que_no_existe() {
        let mut v = base();
        v["kind"] = json!("websocket");
        assert!(matches!(
            Manifiesto::desde_valor(v),
            Err(ErrorManifiesto::TipoDesconocido(_))
        ));
    }

    #[test]
    fn una_accion_sin_op_o_sin_label_se_descarta() {
        // Enseñar una entrada en blanco en el editor es peor que no enseñar nada.
        let mut v = base();
        v["actions"] = json!([
            { "op": "buena", "label": "Buena" },
            { "op": "sin_label" },
            { "label": "Sin op" },
        ]);

        let m = Manifiesto::desde_valor(v).unwrap();
        assert_eq!(m.actions.len(), 1);
        assert_eq!(m.actions[0].op, "buena");
    }

    #[test]
    fn una_accion_sin_destino_va_a_la_tecla() {
        let mut v = base();
        v["actions"] = json!([{ "op": "x", "label": "X" }]);

        let m = Manifiesto::desde_valor(v).unwrap();
        assert_eq!(m.actions[0].targets, vec!["key"]);
    }

    #[test]
    fn un_rango_sin_minimo_y_maximo_no_es_rango() {
        // Sin ellos no hay nada que elegir, y la acción se trata como las de un clic.
        let mut v = base();
        v["actions"] = json!([
            { "op": "a", "label": "A", "value": { "min": 0 } },
            { "op": "b", "label": "B", "value": { "min": 0, "max": 100 } },
        ]);

        let m = Manifiesto::desde_valor(v).unwrap();
        assert!(m.actions[0].value.is_none());

        let r = m.actions[1].value.unwrap();
        assert_eq!(r.step, 1.0, "sin `step` declarado, de uno en uno");
        assert_eq!(r.default, 0.0, "sin `default`, el mínimo");
    }

    #[test]
    fn un_visor_sin_icono_lleva_el_generico() {
        let mut v = base();
        v["views"] = json!([{ "op": "v", "label": "V" }]);

        let m = Manifiesto::desde_valor(v).unwrap();
        assert_eq!(m.views[0].icon, "oled");
    }

    #[test]
    fn sabe_leer_solo_si_declara_visores() {
        let mut sin = base();
        sin["views"] = json!([]);
        let d = Manifiesto::desde_valor(sin).unwrap().descriptor(true, &json!({}));
        assert_eq!(d["hasRead"], false);

        let mut con = base();
        con["views"] = json!([{ "op": "v", "label": "V" }]);
        let d = Manifiesto::desde_valor(con).unwrap().descriptor(true, &json!({}));
        assert_eq!(d["hasRead"], true);
    }

    #[test]
    fn rechaza_un_tipo_de_campo_desconocido() {
        let mut v = base();
        v["settings"] = json!({ "fields": [{ "key": "x", "type": "contraseña" }] });
        assert!(matches!(
            Manifiesto::desde_valor(v),
            Err(ErrorManifiesto::TipoDesconocido(_))
        ));
    }

    #[test]
    fn un_complemento_roto_se_ve_pero_no_se_puede_asignar() {
        // En Ajustes sale con su error; en el editor de teclas no aparece.
        let d = Manifiesto::descriptor_roto("lampdesk", "el plugin.json no se puede leer");

        assert_eq!(d["id"], "lampdesk");
        assert_eq!(d["enabled"], false);
        assert_eq!(d["error"], "el plugin.json no se puede leer");
        assert_eq!(d["actions"].as_array().unwrap().len(), 0);
        assert_eq!(d["views"].as_array().unwrap().len(), 0);
    }

    #[test]
    fn un_json_ilegible_no_revienta() {
        assert!(matches!(
            Manifiesto::desde_json("{ esto no es json"),
            Err(ErrorManifiesto::Json(_))
        ));
    }
}
