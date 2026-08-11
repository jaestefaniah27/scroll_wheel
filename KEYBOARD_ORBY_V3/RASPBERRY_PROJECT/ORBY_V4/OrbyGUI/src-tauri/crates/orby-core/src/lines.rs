// El puerto entrega trozos de bytes, no líneas: un mensaje del teclado puede llegar
// partido en dos lecturas y dos mensajes pueden llegar en la misma.
//
// Port de src/web/split-lines.mjs, y con los mismos casos de prueba. Equivocarse aquí
// no se manifiesta como un error sino como peticiones que caducan a los 4 s sin
// ninguna pista de por qué, así que las dos implementaciones se miden contra las
// mismas expectativas.

/// Devuelve las líneas completas y lo que queda a medias para la siguiente lectura.
pub fn split_lines(resto: &str, trozo: &str) -> (Vec<String>, String) {
    let combinado = format!("{resto}{trozo}");
    let mut partes: Vec<&str> = combinado.split('\n').collect();

    // El último elemento no termina en \n: o es una línea a medias o es cadena vacía.
    // `split` siempre devuelve al menos un elemento, así que este `pop` no falla.
    let cola = partes.pop().unwrap_or("").to_string();

    let lineas = partes
        .into_iter()
        .map(str::trim)
        // Una línea vacía atascaría la cola de peticiones de device.js: se descarta
        // aquí, igual que hace la vía del navegador.
        .filter(|l| !l.is_empty())
        .map(str::to_string)
        .collect();

    (lineas, cola)
}

#[cfg(test)]
mod tests {
    use super::split_lines;

    #[test]
    fn guarda_la_linea_a_medias_como_resto() {
        let (lineas, resto) = split_lines("", "ORBY_V4:FW=4.5\nKEY_EV:3");
        assert_eq!(lineas, vec!["ORBY_V4:FW=4.5"]);
        assert_eq!(resto, "KEY_EV:3");
    }

    #[test]
    fn une_el_resto_previo_con_el_trozo_nuevo() {
        let (lineas, resto) = split_lines("KEY_EV:", "3:1\n");
        assert_eq!(lineas, vec!["KEY_EV:3:1"]);
        assert_eq!(resto, "");
    }

    #[test]
    fn descarta_retornos_de_carro_y_lineas_vacias() {
        // Una línea vacía colada aquí atasca la cola de peticiones de device.js.
        let (lineas, resto) = split_lines("", "SAVE:OK\r\n\n\nPROF:0:END\r\n");
        assert_eq!(lineas, vec!["SAVE:OK", "PROF:0:END"]);
        assert_eq!(resto, "");
    }

    #[test]
    fn un_trozo_sin_salto_no_produce_ninguna_linea() {
        let (lineas, resto) = split_lines("", "ORBY");
        assert!(lineas.is_empty());
        assert_eq!(resto, "ORBY");
    }
}
