// De lo que guarda una secuencia a lo que entiende Windows.
//
// El teclado y el editor de atajos hablan en **usage HID de la página 0x07** (es lo que
// viaja por el informe HID y lo que hay guardado en la Flash y en las secuencias del
// usuario). Windows quiere **códigos de tecla virtual**. Esta traducción es la que hacía
// nut-js por debajo; aquí está escrita para poder probarla, porque un error en la tabla
// no se manifiesta como un fallo sino como una tecla que escribe otra cosa.
//
// La lista de códigos cubiertos es exactamente la de `electron/macros.js`
// (`HID_TO_NUT_KEY`), ni uno más: ampliarla cambiaría lo que hace una secuencia ya
// guardada.

/// Una tecla lista para `SendInput`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tecla {
    pub vk: u16,
    /// Las teclas «extendidas» (flechas, bloque de edición, Ctrl y Alt derechos, Intro
    /// del numérico, la división) necesitan su bandera o Windows las confunde con las del
    /// teclado numérico: sin ella, la flecha arriba escribe un 8 con Bloq Num puesto.
    pub extendida: bool,
}

const fn t(vk: u16) -> Tecla {
    Tecla { vk, extendida: false }
}

const fn e(vk: u16) -> Tecla {
    Tecla { vk, extendida: true }
}

/// Usage HID (página 0x07) → tecla virtual. `None` si no está en la tabla: en Electron
/// eso deja la acción sin hacer nada, y aquí igual.
pub fn hid_a_vk(usage: u8) -> Option<Tecla> {
    Some(match usage {
        // Letras 0x04-0x1D → 'A'..'Z'
        0x04..=0x1d => t(0x41 + (usage - 0x04) as u16),
        // Fila superior: 0x1e-0x26 son 1-9 y 0x27 es el 0
        0x1e..=0x26 => t(0x31 + (usage - 0x1e) as u16),
        0x27 => t(0x30),
        // F1-F12 y F13-F24
        0x3a..=0x45 => t(0x70 + (usage - 0x3a) as u16),
        0x68..=0x73 => t(0x7c + (usage - 0x68) as u16),

        // Edición
        0x28 => t(0x0d),  // Intro
        0x29 => t(0x1b),  // Escape
        0x2a => t(0x08),  // Retroceso
        0x2b => t(0x09),  // Tabulador
        0x2c => t(0x20),  // Espacio
        0x49 => e(0x2d),  // Insertar
        0x4c => e(0x2e),  // Suprimir
        0x4a => e(0x24),  // Inicio
        0x4d => e(0x23),  // Fin
        0x4b => e(0x21),  // Re Pág
        0x4e => e(0x22),  // Av Pág
        0x46 => e(0x2c),  // Impr Pant

        // Flechas
        0x50 => e(0x25),
        0x4f => e(0x27),
        0x52 => e(0x26),
        0x51 => e(0x28),

        // Símbolos. Los VK_OEM_* dependen de la distribución del teclado: en una española
        // el que Windows llama «punto y coma» es la ñ. Es lo mismo que hacía nut-js, y lo
        // correcto: la secuencia guarda la posición física de la tecla, no el símbolo.
        0x2d => t(0xbd),  // OEM_MINUS
        0x2e => t(0xbb),  // OEM_PLUS
        0x2f => t(0xdb),  // OEM_4
        0x30 => t(0xdd),  // OEM_6
        0x31 => t(0xdc),  // OEM_5
        0x33 => t(0xba),  // OEM_1
        0x34 => t(0xde),  // OEM_7
        0x35 => t(0xc0),  // OEM_3
        0x36 => t(0xbc),  // OEM_COMMA
        0x37 => t(0xbe),  // OEM_PERIOD
        0x38 => t(0xbf),  // OEM_2

        // Teclado numérico
        0x54 => e(0x6f),  // División (extendida)
        0x55 => t(0x6a),  // Multiplicación
        0x56 => t(0x6d),  // Resta
        0x57 => t(0x6b),  // Suma
        0x58 => e(0x0d),  // Intro del numérico
        0x59..=0x61 => t(0x61 + (usage - 0x59) as u16),
        0x62 => t(0x60),

        _ => return None,
    })
}

/// Los ocho bits de modificador del informe HID, en el orden en que los pone el firmware.
pub fn modificadores_a_vk(bits: u8) -> Vec<Tecla> {
    const TABLA: &[(u8, Tecla)] = &[
        (0x01, Tecla { vk: 0xa2, extendida: false }), // Control izquierdo
        (0x02, Tecla { vk: 0xa0, extendida: false }), // Mayús izquierda
        (0x04, Tecla { vk: 0xa4, extendida: false }), // Alt izquierdo
        (0x08, Tecla { vk: 0x5b, extendida: true }),  // Windows izquierdo
        (0x10, Tecla { vk: 0xa3, extendida: true }),  // Control derecho
        (0x20, Tecla { vk: 0xa1, extendida: false }), // Mayús derecha
        (0x40, Tecla { vk: 0xa5, extendida: true }),  // Alt Gr
        (0x80, Tecla { vk: 0x5c, extendida: true }),  // Windows derecho
    ];

    TABLA
        .iter()
        .filter(|(bit, _)| bits & bit != 0)
        .map(|(_, tecla)| *tecla)
        .collect()
}

/// El paso `key` de una secuencia no guarda un usage HID sino un `KeyboardEvent.code` del
/// navegador, porque lo captura el editor de la app. Se cubren los mismos que cubría
/// `mapCodeToNutKey`: letras, dígitos, Intro, Escape y espacio.
pub fn code_a_vk(code: &str) -> Option<Tecla> {
    if let Some(letra) = code.strip_prefix("Key") {
        let mut c = letra.chars();
        let (Some(l), None) = (c.next(), c.next()) else { return None };
        if l.is_ascii_uppercase() {
            return Some(t(l as u16));
        }
        return None;
    }

    if let Some(digito) = code.strip_prefix("Digit") {
        let mut c = digito.chars();
        let (Some(d), None) = (c.next(), c.next()) else { return None };
        if d.is_ascii_digit() {
            return Some(t(d as u16));
        }
        return None;
    }

    Some(match code {
        "Enter" => t(0x0d),
        "Escape" => t(0x1b),
        "Space" => t(0x20),
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::{code_a_vk, hid_a_vk, modificadores_a_vk};

    #[test]
    fn las_letras_y_los_digitos_salen_donde_deben() {
        assert_eq!(hid_a_vk(0x04).unwrap().vk, 0x41, "A");
        assert_eq!(hid_a_vk(0x1d).unwrap().vk, 0x5a, "Z");
        // El 0 va al final de la fila, no delante del 1: es el fallo clásico de esta tabla.
        assert_eq!(hid_a_vk(0x1e).unwrap().vk, 0x31, "1");
        assert_eq!(hid_a_vk(0x26).unwrap().vk, 0x39, "9");
        assert_eq!(hid_a_vk(0x27).unwrap().vk, 0x30, "0");
    }

    #[test]
    fn las_funciones_llegan_hasta_f24() {
        assert_eq!(hid_a_vk(0x3a).unwrap().vk, 0x70, "F1");
        assert_eq!(hid_a_vk(0x45).unwrap().vk, 0x7b, "F12");
        assert_eq!(hid_a_vk(0x68).unwrap().vk, 0x7c, "F13");
        assert_eq!(hid_a_vk(0x73).unwrap().vk, 0x87, "F24");
    }

    #[test]
    fn las_flechas_y_el_bloque_de_edicion_van_marcadas_como_extendidas() {
        // Sin la bandera, con Bloq Num puesto la flecha arriba escribe un 8.
        for usage in [0x50, 0x4f, 0x52, 0x51, 0x49, 0x4c, 0x4a, 0x4d, 0x4b, 0x4e] {
            assert!(hid_a_vk(usage).unwrap().extendida, "0x{usage:02x} debería ser extendida");
        }
        // Y las del numérico de verdad, no.
        assert!(!hid_a_vk(0x59).unwrap().extendida, "el 1 del numérico no es extendida");
    }

    #[test]
    fn el_numerico_no_se_confunde_con_la_fila_superior() {
        assert_eq!(hid_a_vk(0x59).unwrap().vk, 0x61, "1 del numérico");
        assert_eq!(hid_a_vk(0x61).unwrap().vk, 0x69, "9 del numérico");
        assert_eq!(hid_a_vk(0x62).unwrap().vk, 0x60, "0 del numérico");
        // El Intro del numérico es el mismo VK que el otro, pero extendido: es lo único
        // que los distingue.
        assert_eq!(hid_a_vk(0x58).unwrap().vk, hid_a_vk(0x28).unwrap().vk);
        assert!(hid_a_vk(0x58).unwrap().extendida);
        assert!(!hid_a_vk(0x28).unwrap().extendida);
    }

    #[test]
    fn un_usage_que_no_esta_en_la_tabla_no_hace_nada() {
        // En Electron esto dejaba la acción sin efecto, y hay que conservarlo: inventar
        // una tecla aquí haría que una secuencia vieja escribiera algo que nunca escribió.
        assert!(hid_a_vk(0x00).is_none());
        assert!(hid_a_vk(0x65).is_none());
        assert!(hid_a_vk(0xff).is_none());
    }

    #[test]
    fn los_ocho_modificadores_salen_en_orden() {
        assert!(modificadores_a_vk(0).is_empty());

        let ctrl_alt = modificadores_a_vk(0x01 | 0x04);
        assert_eq!(ctrl_alt.len(), 2);
        assert_eq!(ctrl_alt[0].vk, 0xa2, "Control izquierdo primero");
        assert_eq!(ctrl_alt[1].vk, 0xa4, "Alt izquierdo después");

        // Alt Gr es el Alt derecho, y va extendido.
        let altgr = modificadores_a_vk(0x40);
        assert_eq!(altgr[0].vk, 0xa5);
        assert!(altgr[0].extendida);

        assert_eq!(modificadores_a_vk(0xff).len(), 8);
    }

    #[test]
    fn el_paso_key_traduce_los_mismos_codigos_que_traducia_antes() {
        assert_eq!(code_a_vk("KeyA").unwrap().vk, 0x41);
        assert_eq!(code_a_vk("KeyZ").unwrap().vk, 0x5a);
        assert_eq!(code_a_vk("Digit0").unwrap().vk, 0x30);
        assert_eq!(code_a_vk("Digit9").unwrap().vk, 0x39);
        assert_eq!(code_a_vk("Enter").unwrap().vk, 0x0d);
        assert_eq!(code_a_vk("Escape").unwrap().vk, 0x1b);
        assert_eq!(code_a_vk("Space").unwrap().vk, 0x20);

        // Y nada más, igual que antes: el editor captura cualquier code, pero solo estos
        // se traducían.
        for code in ["F5", "ArrowUp", "Keya", "KeyAB", "Digit", "Tab", ""] {
            assert!(code_a_vk(code).is_none(), "«{code}» no debería traducirse");
        }
    }
}
