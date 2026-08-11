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

#[cfg(test)]
mod tests {
    use super::compare_fw;
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
}
