// Qué versión de firmware se le puede ofrecer al usuario, a partir de lo que devuelve la
// API de releases de GitHub.
//
// Está separado de la descarga para poder comprobar sin red la parte que decide, que es
// donde están los fallos con consecuencias: ofrecer un firmware que la app no sabe
// manejar, o no ofrecer ninguno porque se filtró de más.

use crate::fw::compare_fw;
use serde_json::Value;
use std::cmp::Ordering;

/// Las releases de firmware van etiquetadas así para no mezclarse con las de la app.
pub const PREFIJO_ETIQUETA: &str = "fw-v";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Candidato {
    pub version: String,
    pub nombre_asset: String,
    pub url: String,
    pub tamano: u64,
}

/// Devuelve las versiones ofrecibles, de la más nueva a la más vieja.
///
/// `max_fw` es hasta dónde sabe hablar esta versión de la app (`FW_RECOMMENDED` en
/// src/compat.js): publicar un firmware más nuevo que la app no debe ofrecerse aquí,
/// porque la app no sabría entenderse con él.
pub fn candidatos(releases: &Value, max_fw: Option<&str>) -> Vec<Candidato> {
    let Some(lista) = releases.as_array() else { return Vec::new() };

    let mut out: Vec<Candidato> = lista
        .iter()
        // Los borradores no los puede descargar nadie.
        .filter(|r| r.get("draft") != Some(&Value::Bool(true)))
        // Las prerelease SÍ entran: es como se publican las de firmware, para que
        // `releases/latest` siga apuntando a una release de la APP. Filtrarlas aquí
        // dejaría al usuario sin ninguna actualización de firmware.
        .filter_map(|r| {
            let etiqueta = r.get("tag_name")?.as_str()?;
            let version = etiqueta.strip_prefix(PREFIJO_ETIQUETA)?.to_string();

            // El .uf2, no el .sha256 que lo acompaña.
            let asset = r
                .get("assets")?
                .as_array()?
                .iter()
                .find(|a| {
                    a.get("name")
                        .and_then(Value::as_str)
                        .is_some_and(|n| n.ends_with(".uf2"))
                })?;

            Some(Candidato {
                version,
                nombre_asset: asset.get("name")?.as_str()?.to_string(),
                url: asset.get("browser_download_url")?.as_str()?.to_string(),
                tamano: asset.get("size").and_then(Value::as_u64).unwrap_or(0),
            })
        })
        .filter(|c| match max_fw {
            Some(max) => compare_fw(&c.version, max) != Ordering::Greater,
            None => true,
        })
        .collect();

    out.sort_by(|a, b| compare_fw(&b.version, &a.version));
    out
}

/// ¿Hay algo más nuevo que lo que lleva puesto el teclado?
pub fn hay_actualizacion(ultima: Option<&Candidato>, actual: Option<&str>) -> bool {
    match (ultima, actual) {
        (Some(u), Some(a)) => compare_fw(&u.version, a) == Ordering::Greater,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn release(etiqueta: &str, prerelease: bool, draft: bool, assets: Value) -> Value {
        json!({ "tag_name": etiqueta, "prerelease": prerelease, "draft": draft, "assets": assets })
    }

    fn uf2(nombre: &str) -> Value {
        json!([{ "name": nombre, "browser_download_url": format!("https://x/{nombre}"), "size": 400_000 }])
    }

    #[test]
    fn las_prerelease_cuentan_como_versiones_ofrecibles() {
        // Es COMO se publican las de firmware, para que `releases/latest` siga apuntando
        // a una release de la app y no rompa el actualizador de todo el parque
        // instalado. Filtrarlas dejaría al usuario sin ninguna actualización.
        let releases = json!([release("fw-v4.5", true, false, uf2("orby-4.5.uf2"))]);

        let c = candidatos(&releases, None);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].version, "4.5");
    }

    #[test]
    fn los_borradores_no() {
        let releases = json!([release("fw-v4.6", true, true, uf2("orby-4.6.uf2"))]);
        assert!(candidatos(&releases, None).is_empty());
    }

    #[test]
    fn las_releases_de_la_app_no_se_confunden_con_las_de_firmware() {
        let releases = json!([
            release("v0.5.1", false, false, uf2("OrbyGUI-Setup.uf2")),
            release("fw-v4.5", true, false, uf2("orby-4.5.uf2")),
        ]);

        let c = candidatos(&releases, None);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].version, "4.5");
    }

    #[test]
    fn se_coge_el_uf2_y_no_su_sha256() {
        let releases = json!([release(
            "fw-v4.5",
            true,
            false,
            json!([
                { "name": "orby-4.5.uf2.sha256", "browser_download_url": "https://x/s", "size": 64 },
                { "name": "orby-4.5.uf2", "browser_download_url": "https://x/u", "size": 400000 },
            ])
        )]);

        let c = candidatos(&releases, None);
        assert_eq!(c[0].nombre_asset, "orby-4.5.uf2");
        assert_eq!(c[0].tamano, 400_000);
    }

    #[test]
    fn una_release_sin_uf2_se_ignora() {
        let releases = json!([release("fw-v4.5", true, false, json!([]))]);
        assert!(candidatos(&releases, None).is_empty());
    }

    #[test]
    fn no_se_ofrece_un_firmware_mas_nuevo_que_la_app() {
        // La app no sabría hablar con él: ofrecerlo es empujar al usuario a un teclado
        // que su propia app no entiende.
        let releases = json!([
            release("fw-v4.5", true, false, uf2("a.uf2")),
            release("fw-v5.0", true, false, uf2("b.uf2")),
        ]);

        let c = candidatos(&releases, Some("4.5"));
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].version, "4.5");
    }

    #[test]
    fn se_ordenan_por_partes_y_no_como_decimales() {
        // 4.10 es posterior a 4.9. El fallo clásico de leer versiones con parseFloat.
        let releases = json!([
            release("fw-v4.9", true, false, uf2("a.uf2")),
            release("fw-v4.10", true, false, uf2("b.uf2")),
            release("fw-v4.2", true, false, uf2("c.uf2")),
        ]);

        let versiones: Vec<String> =
            candidatos(&releases, None).into_iter().map(|c| c.version).collect();
        assert_eq!(versiones, vec!["4.10", "4.9", "4.2"]);
    }

    #[test]
    fn hay_actualizacion_solo_si_es_mas_nueva() {
        let releases = json!([release("fw-v4.5", true, false, uf2("a.uf2"))]);
        let c = candidatos(&releases, None);
        let ultima = c.first();

        assert!(hay_actualizacion(ultima, Some("4.4")));
        assert!(!hay_actualizacion(ultima, Some("4.5")), "la misma versión no es actualización");
        assert!(!hay_actualizacion(ultima, Some("4.6")), "no se degrada hacia atrás");
        // Sin saber qué lleva el teclado no se propone nada.
        assert!(!hay_actualizacion(ultima, None));
        assert!(!hay_actualizacion(None, Some("4.4")));
    }

    #[test]
    fn una_respuesta_inesperada_no_revienta() {
        // La API puede devolver un error en forma de objeto en vez de una lista.
        assert!(candidatos(&json!({ "message": "rate limit" }), None).is_empty());
        assert!(candidatos(&json!([]), None).is_empty());
        assert!(candidatos(&json!([{ "sin": "nada" }]), None).is_empty());
    }
}
