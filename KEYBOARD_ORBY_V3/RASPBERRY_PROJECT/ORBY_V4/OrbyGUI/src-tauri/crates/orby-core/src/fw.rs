// Comparación de versiones de firmware.
//
// "4.10" es posterior a "4.9": se compara por partes, no como número decimal. Leerlo
// con parseFloat fue un fallo real, y por eso está aquí con su prueba.

use std::cmp::Ordering;

fn partes(v: &str) -> Vec<u32> {
    v.split('.')
        // Lo que no sea un número cuenta como 0, igual que el `parseInt(...) || 0`
        // del original: una versión rara no debe hacer estallar la comparación.
        .map(|n| n.trim().parse::<u32>().unwrap_or(0))
        .collect()
}

pub fn compare_fw(a: &str, b: &str) -> Ordering {
    let x = partes(a);
    let y = partes(b);

    for i in 0..x.len().max(y.len()) {
        // Una versión más corta se rellena con ceros: "4" y "4.0" son la misma.
        let d = x.get(i).copied().unwrap_or(0).cmp(&y.get(i).copied().unwrap_or(0));
        if d != Ordering::Equal {
            return d;
        }
    }

    Ordering::Equal
}

/// Comparar versiones **de la app**, que no tienen el mismo formato que las del firmware.
///
/// El firmware va por `4.5`: dos números y nada más, y para eso vale `compare_fw`. La app
/// va por semver y puede llevar una etiqueta de preversión (`1.0.0-alpha`), y ahí
/// `compare_fw` **miente**: trocea por puntos, `"0-alpha"` no es un número, cuenta como 0
/// y `1.0.0-alpha` sale *igual* que `1.0.0`.
///
/// Eso no es una pega teórica. La versión instalada hoy es `1.0.0-alpha`: con la
/// comparación de firmware, publicar la release `v1.0.0` no habría disparado ninguna
/// actualización, y como todo el proceso es silencioso, nadie se habría enterado. Es
/// exactamente el fallo mudo que el actualizador automático existe para no tener.
///
/// La regla de semver: una versión CON preversión es **anterior** a la misma sin ella
/// (`1.0.0-alpha` < `1.0.0`), y entre dos preversiones manda el orden alfabético
/// (`-alpha` < `-beta`). No se implementa semver entero —los identificadores separados por
/// puntos, los metadatos de build— porque aquí no se usan y sería código sin nadie que lo
/// ejercite.
pub fn compare_app(a: &str, b: &str) -> Ordering {
    let (nucleo_a, pre_a) = parte_pre(a);
    let (nucleo_b, pre_b) = parte_pre(b);

    let d = compare_fw(nucleo_a, nucleo_b);
    if d != Ordering::Equal {
        return d;
    }

    match (pre_a, pre_b) {
        (None, None) => Ordering::Equal,
        // Sin preversión es la definitiva, y va después.
        (None, Some(_)) => Ordering::Greater,
        (Some(_), None) => Ordering::Less,
        (Some(x), Some(y)) => x.cmp(y),
    }
}

/// Parte `1.0.0-alpha` en `("1.0.0", Some("alpha"))`. Se corta por el primer guion y se
/// tira lo que venga después de un `+`: los metadatos de build no cuentan para ordenar.
fn parte_pre(v: &str) -> (&str, Option<&str>) {
    let v = v.split('+').next().unwrap_or(v).trim();
    match v.split_once('-') {
        Some((nucleo, pre)) => (nucleo, Some(pre)),
        None => (v, None),
    }
}

#[cfg(test)]
mod tests {
    use super::{compare_app, compare_fw};
    use std::cmp::Ordering;

    #[test]
    fn cuatro_diez_es_posterior_a_cuatro_nueve() {
        // El caso que rompía con parseFloat.
        assert_eq!(compare_fw("4.10", "4.9"), Ordering::Greater);
        assert_eq!(compare_fw("4.9", "4.10"), Ordering::Less);
    }

    #[test]
    fn versiones_iguales_y_rellenadas_con_ceros() {
        assert_eq!(compare_fw("4.5", "4.5"), Ordering::Equal);
        assert_eq!(compare_fw("4", "4.0"), Ordering::Equal);
        assert_eq!(compare_fw("4.0.0", "4"), Ordering::Equal);
    }

    #[test]
    fn compara_primero_la_parte_mayor() {
        assert_eq!(compare_fw("5.0", "4.99"), Ordering::Greater);
        assert_eq!(compare_fw("2.0", "4.5"), Ordering::Less);
    }

    #[test]
    fn una_version_ilegible_cuenta_como_cero() {
        assert_eq!(compare_fw("", "0.0"), Ordering::Equal);
        assert_eq!(compare_fw("4.x", "4.0"), Ordering::Equal);
    }

    // --- compare_app: versiones de la app, con preversión -----------------------------

    #[test]
    fn una_preversion_es_anterior_a_la_definitiva() {
        // EL caso: la app instalada es 1.0.0-alpha y se publica la v1.0.0. Con compare_fw
        // salían iguales y la actualización no se disparaba nunca.
        assert_eq!(compare_app("1.0.0", "1.0.0-alpha"), Ordering::Greater);
        assert_eq!(compare_app("1.0.0-alpha", "1.0.0"), Ordering::Less);
    }

    #[test]
    fn entre_preversiones_manda_el_orden_alfabetico() {
        assert_eq!(compare_app("1.0.0-beta", "1.0.0-alpha"), Ordering::Greater);
        assert_eq!(compare_app("1.0.0-alpha", "1.0.0-alpha"), Ordering::Equal);
    }

    #[test]
    fn el_nucleo_manda_sobre_la_preversion() {
        assert_eq!(compare_app("1.1.0-alpha", "1.0.0"), Ordering::Greater);
        assert_eq!(compare_app("0.9.0", "1.0.0-alpha"), Ordering::Less);
    }

    #[test]
    fn sin_preversion_se_comporta_como_compare_fw() {
        assert_eq!(compare_app("1.0.10", "1.0.9"), Ordering::Greater);
        assert_eq!(compare_app("2.0.0", "2.0.0"), Ordering::Equal);
    }

    #[test]
    fn los_metadatos_de_build_no_cuentan() {
        assert_eq!(compare_app("1.0.0+abc", "1.0.0"), Ordering::Equal);
        assert_eq!(compare_app("1.0.0-alpha+abc", "1.0.0-alpha"), Ordering::Equal);
    }
}
