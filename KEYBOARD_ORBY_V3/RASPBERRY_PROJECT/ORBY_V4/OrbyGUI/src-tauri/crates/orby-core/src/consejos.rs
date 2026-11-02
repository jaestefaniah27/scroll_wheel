// Vídeos de los consejos: qué dice el manifiesto y cuáles faltan por bajar.
//
// Separado de la descarga para poder comprobar sin red lo que decide. El manifiesto
// viene de internet, así que se lee a la defensiva: una entrada rara se descarta sola,
// sin tirar las demás ni el arranque.

use serde_json::Value;
use sha2::{Digest, Sha256};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntradaVideo {
    pub id: String,
    pub fichero: String,
    pub bytes: u64,
    pub sha256: String,
}

pub fn sha256_hex(datos: &[u8]) -> String {
    Sha256::digest(datos).iter().map(|b| format!("{b:02x}")).collect()
}

fn es_sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| b.is_ascii_hexdigit())
}

/// `fichero` acaba en una ruta del disco del usuario: nada de carpetas ni de `..`, o un
/// manifiesto manipulado escribiría fuera de la carpeta de vídeos.
fn fichero_seguro(s: &str) -> bool {
    !s.is_empty() && !s.contains(['/', '\\']) && !s.contains("..")
}

pub fn leer_manifiesto(json: &str) -> Vec<EntradaVideo> {
    let Ok(valor) = serde_json::from_str::<Value>(json) else { return Vec::new() };
    let Some(lista) = valor.get("videos").and_then(Value::as_array) else { return Vec::new() };
    lista
        .iter()
        .filter_map(|e| {
            let id = e.get("id")?.as_str()?.to_string();
            let fichero = e.get("fichero")?.as_str()?.to_string();
            let sha256 = e.get("sha256")?.as_str()?.to_ascii_lowercase();
            let bytes = e.get("bytes").and_then(Value::as_u64).unwrap_or(0);
            (!id.is_empty() && fichero_seguro(&fichero) && es_sha256(&sha256))
                .then_some(EntradaVideo { id, fichero, bytes, sha256 })
        })
        .collect()
}

/// `presentes` son pares `(fichero, sha256 calculado)` de lo que hay en disco. Un fichero
/// con otro hash cuenta como ausente: es una descarga cortada o un vídeo que cambió.
pub fn faltan(manifiesto: &[EntradaVideo], presentes: &[(String, String)]) -> Vec<EntradaVideo> {
    manifiesto
        .iter()
        .filter(|e| !presentes.iter().any(|(f, h)| f == &e.fichero && h.eq_ignore_ascii_case(&e.sha256)))
        .cloned()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const H1: &str = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
    const H2: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

    fn entrada(id: &str, fichero: &str, sha: &str) -> EntradaVideo {
        EntradaVideo { id: id.into(), fichero: fichero.into(), bytes: 1, sha256: sha.into() }
    }

    #[test]
    fn manifiesto_valido_se_lee() {
        let json = format!(r#"{{"schema":1,"videos":[{{"id":"a","fichero":"a.mp4","bytes":10,"sha256":"{H1}"}}]}}"#);
        assert_eq!(leer_manifiesto(&json), vec![EntradaVideo { id: "a".into(), fichero: "a.mp4".into(), bytes: 10, sha256: H1.into() }]);
    }

    #[test]
    fn lee_el_formato_que_escribe_tools_consejos_manifiesto_py() {
        let json = format!("{{
  \"schema\": 1,
  \"videos\": [
    {{
      \"id\": \"prueba-a\",
      \"fichero\": \"prueba-a.mp4\",
      \"bytes\": 2000,
      \"sha256\": \"{H2}\"
    }}
  ]
}}
");
        assert_eq!(leer_manifiesto(&json).len(), 1);
        assert_eq!(leer_manifiesto("{
  \"schema\": 1,
  \"videos\": []
}
").len(), 0);
    }

    #[test]
    fn json_invalido_da_vacio() {
        assert!(leer_manifiesto("no es json").is_empty());
        assert!(leer_manifiesto("{}").is_empty());
    }

    #[test]
    fn entrada_con_ruta_peligrosa_se_descarta() {
        for malo in ["../x.mp4", "a/b.mp4", "a\\b.mp4", ""] {
            let json = format!(r#"{{"videos":[{{"id":"a","fichero":"{}","sha256":"{H1}"}}]}}"#, malo.replace('\\', "\\\\"));
            assert!(leer_manifiesto(&json).is_empty(), "{malo}");
        }
    }

    #[test]
    fn sha_mal_formado_se_descarta_y_las_buenas_se_conservan() {
        let json = format!(r#"{{"videos":[{{"id":"a","fichero":"a.mp4","sha256":"zz"}},{{"id":"b","fichero":"b.mp4","sha256":"{H1}"}}]}}"#);
        let r = leer_manifiesto(&json);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].id, "b");
    }

    #[test]
    fn faltan_devuelve_los_que_no_estan() {
        let m = [entrada("a", "a.mp4", H1), entrada("b", "b.mp4", H2)];
        let f = faltan(&m, &[("a.mp4".into(), H1.into())]);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].id, "b");
    }

    #[test]
    fn faltan_devuelve_los_de_hash_distinto() {
        let m = [entrada("a", "a.mp4", H1)];
        assert_eq!(faltan(&m, &[("a.mp4".into(), H2.into())]).len(), 1);
    }

    #[test]
    fn faltan_vacio_si_todo_coincide() {
        let m = [entrada("a", "a.mp4", H1)];
        assert!(faltan(&m, &[("a.mp4".into(), H1.to_ascii_uppercase())]).is_empty());
    }

    #[test]
    fn sha256_de_cadena_vacia_es_el_conocido() {
        assert_eq!(sha256_hex(b""), H1);
        assert_eq!(sha256_hex(b"abc"), H2);
    }
}
