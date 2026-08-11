// Los códigos de tecla de libuiohook, que son los que hay guardados en las grabaciones.
//
// La app de Electron capturaba con `uiohook-napi` y reproducía con nut-js, así que en el
// disco quedó el número de uiohook (`UiohookKey`). Aquí no hay ni una cosa ni la otra: se
// captura con `WH_KEYBOARD_LL` y se reproduce con `SendInput`, que hablan en **teclas
// virtuales de Windows**. Esta tabla es el puente, y va en los dos sentidos: al grabar
// para seguir escribiendo el mismo formato, y al reproducir para entender las grabaciones
// que el usuario ya tiene.
//
// Los números salen del volcado de `require('uiohook-napi').UiohookKey` de esta misma
// instalación (versión 1.5.5, 124 entradas), no de la documentación. Son los scancodes
// del juego 1 con tres codificaciones que conviene reconocer al leer la tabla:
//
//   - tecla normal          → el scancode tal cual        (A = 0x1E)
//   - extendida (con E0)    → `0xE00 | sc`                (Inicio = 0xE47)
//   - flechas               → `0xE000 | sc`               (arriba = 0xE048)
//   - bloque numérico con Bloq Num apagado → `0xEE00 | sc` (0xEE47)
//
// La incoherencia entre `0xE00` y `0xE000` es de libuiohook, no una errata: las flechas
// van por un camino distinto en su código. Se copia tal cual porque cambiarla haría
// ilegibles las grabaciones existentes.

use crate::teclas::Tecla;

/// Una fila del puente. `extendida` es la bandera `LLKHF_EXTENDED` del gancho y la
/// `KEYEVENTF_EXTENDEDKEY` del envío: es lo único que distingue el Inicio del bloque de
/// edición del 7 del numérico con Bloq Num apagado, porque comparten tecla virtual.
struct Fila {
    uiohook: u32,
    vk: u16,
    extendida: bool,
}

const fn f(uiohook: u32, vk: u16) -> Fila {
    Fila { uiohook, vk, extendida: false }
}

const fn x(uiohook: u32, vk: u16) -> Fila {
    Fila { uiohook, vk, extendida: true }
}

#[rustfmt::skip]
const TABLA: &[Fila] = &[
    f(1, 0x1b),      // Escape
    f(2, 0x31), f(3, 0x32), f(4, 0x33), f(5, 0x34), f(6, 0x35),
    f(7, 0x36), f(8, 0x37), f(9, 0x38), f(10, 0x39), f(11, 0x30),
    f(12, 0xbd),     // OEM_MINUS
    f(13, 0xbb),     // OEM_PLUS
    f(14, 0x08),     // Retroceso
    f(15, 0x09),     // Tabulador
    f(16, 0x51), f(17, 0x57), f(18, 0x45), f(19, 0x52), f(20, 0x54),
    f(21, 0x59), f(22, 0x55), f(23, 0x49), f(24, 0x4f), f(25, 0x50),
    f(26, 0xdb),     // OEM_4
    f(27, 0xdd),     // OEM_6
    f(28, 0x0d),     // Intro del bloque principal
    f(29, 0xa2),     // Control izquierdo
    f(30, 0x41), f(31, 0x53), f(32, 0x44), f(33, 0x46), f(34, 0x47),
    f(35, 0x48), f(36, 0x4a), f(37, 0x4b), f(38, 0x4c),
    f(39, 0xba),     // OEM_1
    f(40, 0xde),     // OEM_7
    f(41, 0xc0),     // OEM_3
    f(42, 0xa0),     // Mayúsculas izquierda
    f(43, 0xdc),     // OEM_5
    f(44, 0x5a), f(45, 0x58), f(46, 0x43), f(47, 0x56), f(48, 0x42),
    f(49, 0x4e), f(50, 0x4d),
    f(51, 0xbc),     // OEM_COMMA
    f(52, 0xbe),     // OEM_PERIOD
    f(53, 0xbf),     // OEM_2
    f(54, 0xa1),     // Mayúsculas derecha
    f(55, 0x6a),     // * del numérico
    f(56, 0xa4),     // Alt izquierdo
    f(57, 0x20),     // Espacio
    f(58, 0x14),     // Bloq Mayús
    f(59, 0x70), f(60, 0x71), f(61, 0x72), f(62, 0x73), f(63, 0x74),
    f(64, 0x75), f(65, 0x76), f(66, 0x77), f(67, 0x78), f(68, 0x79),
    f(69, 0x90),     // Bloq Num
    f(70, 0x91),     // Bloq Despl
    f(71, 0x67), f(72, 0x68), f(73, 0x69),
    f(74, 0x6d),     // - del numérico
    f(75, 0x64), f(76, 0x65), f(77, 0x66),
    f(78, 0x6b),     // + del numérico
    f(79, 0x61), f(80, 0x62), f(81, 0x63), f(82, 0x60),
    f(83, 0x6e),     // Separador decimal del numérico
    f(87, 0x7a), f(88, 0x7b),
    f(91, 0x7c), f(92, 0x7d), f(93, 0x7e),
    f(99, 0x7f), f(100, 0x80), f(101, 0x81), f(102, 0x82), f(103, 0x83),
    f(104, 0x84), f(105, 0x85), f(106, 0x86), f(107, 0x87),

    // Extendidas: `0xE00 | scancode`.
    x(3612, 0x0d),   // Intro del numérico
    x(3613, 0xa3),   // Control derecho
    x(3637, 0x6f),   // / del numérico
    x(3639, 0x2c),   // Impr Pant
    x(3640, 0xa5),   // Alt derecho
    x(3655, 0x24),   // Inicio
    x(3657, 0x21),   // Re Pág
    x(3663, 0x23),   // Fin
    x(3665, 0x22),   // Av Pág
    x(3666, 0x2d),   // Insertar
    x(3667, 0x2e),   // Suprimir
    x(3675, 0x5b),   // Windows izquierda
    x(3676, 0x5c),   // Windows derecha

    // Flechas: `0xE000 | scancode`, con el prefijo largo.
    x(57416, 0x26),  // Arriba
    x(57419, 0x25),  // Izquierda
    x(57421, 0x27),  // Derecha
    x(57424, 0x28),  // Abajo

    // Bloque numérico con Bloq Num apagado: `0xEE00 | scancode`. Comparten tecla virtual
    // con las de arriba y solo se distinguen por no ser extendidas.
    f(60999, 0x24),  // 7 → Inicio
    f(61000, 0x26),  // 8 → Arriba
    f(61001, 0x21),  // 9 → Re Pág
    f(61003, 0x25),  // 4 → Izquierda
    f(61005, 0x27),  // 6 → Derecha
    f(61007, 0x23),  // 1 → Fin
    f(61008, 0x28),  // 2 → Abajo
    f(61009, 0x22),  // 3 → Av Pág
    f(61010, 0x2d),  // 0 → Insertar
    f(61011, 0x2e),  // , → Suprimir
];

/// Código de uiohook → tecla para `SendInput`. `None` si la grabación trae un código que
/// no está en la tabla, y entonces ese evento se salta: en Electron pasaba lo mismo
/// (`nutKey` devolvía `undefined` y `applyEvent` se iba sin hacer nada).
pub fn uiohook_a_tecla(code: u32) -> Option<Tecla> {
    TABLA
        .iter()
        .find(|fila| fila.uiohook == code)
        .map(|fila| Tecla { vk: fila.vk, extendida: fila.extendida })
}

/// Lo que llega del gancho → código de uiohook, para grabar en el formato de siempre.
///
/// Si la pareja exacta no está pero la tecla virtual solo aparece una vez en la tabla, se
/// acepta igualmente. Esto cubre las teclas cuya bandera de extendida depende del teclado
/// y del controlador —Bloq Num y Bloq Despl son las clásicas—: sin esta salida, en un
/// portátil que las marque al revés no se grabarían, y el fallo aparecería como una tecla
/// que «no se graba» sin más explicación. Las que sí necesitan la bandera para
/// distinguirse (Inicio contra el 7 del numérico) tienen dos filas con la misma tecla
/// virtual, así que nunca caen por aquí.
pub fn tecla_a_uiohook(vk: u16, extendida: bool) -> Option<u32> {
    if let Some(fila) = TABLA
        .iter()
        .find(|fila| fila.vk == vk && fila.extendida == extendida)
    {
        return Some(fila.uiohook);
    }

    let mut candidatas = TABLA.iter().filter(|fila| fila.vk == vk);
    let unica = candidatas.next()?;
    if candidatas.next().is_some() {
        return None;
    }
    Some(unica.uiohook)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn la_tabla_no_tiene_codigos_repetidos() {
        let mut vistos: Vec<u32> = TABLA.iter().map(|f| f.uiohook).collect();
        vistos.sort_unstable();
        let antes = vistos.len();
        vistos.dedup();
        assert_eq!(antes, vistos.len());
    }

    #[test]
    fn la_tabla_no_tiene_parejas_repetidas() {
        let mut vistos: Vec<(u16, bool)> = TABLA.iter().map(|f| (f.vk, f.extendida)).collect();
        vistos.sort_unstable();
        let antes = vistos.len();
        vistos.dedup();
        assert_eq!(antes, vistos.len());
    }

    #[test]
    fn toda_la_tabla_va_y_vuelve() {
        for fila in TABLA {
            let tecla = uiohook_a_tecla(fila.uiohook).expect("la fila tiene que traducirse");
            assert_eq!(tecla.vk, fila.vk);
            assert_eq!(tecla.extendida, fila.extendida);
            assert_eq!(
                tecla_a_uiohook(tecla.vk, tecla.extendida),
                Some(fila.uiohook),
                "no vuelve el código {}",
                fila.uiohook
            );
        }
    }

    #[test]
    fn las_letras_son_las_de_siempre() {
        // 30 es la «A» en libuiohook (scancode 0x1E) y 0x41 en Windows.
        assert_eq!(uiohook_a_tecla(30), Some(Tecla { vk: 0x41, extendida: false }));
    }

    #[test]
    fn el_bloque_de_edicion_y_el_numerico_no_se_confunden() {
        // Misma tecla virtual, distinta bandera: son teclas distintas.
        assert_eq!(uiohook_a_tecla(3655), Some(Tecla { vk: 0x24, extendida: true }));
        assert_eq!(uiohook_a_tecla(60999), Some(Tecla { vk: 0x24, extendida: false }));
        assert_eq!(tecla_a_uiohook(0x24, true), Some(3655));
        assert_eq!(tecla_a_uiohook(0x24, false), Some(60999));
    }

    #[test]
    fn las_flechas_llevan_la_bandera_de_extendida() {
        // Sin ella, con Bloq Num puesto la flecha arriba escribe un 8.
        assert_eq!(uiohook_a_tecla(57416), Some(Tecla { vk: 0x26, extendida: true }));
    }

    #[test]
    fn una_bandera_inesperada_no_impide_grabar_una_tecla_sin_gemela() {
        // Bloq Num no tiene pareja en la tabla: da igual cómo la marque el teclado.
        assert_eq!(tecla_a_uiohook(0x90, false), Some(69));
        assert_eq!(tecla_a_uiohook(0x90, true), Some(69));
    }

    #[test]
    fn un_codigo_desconocido_no_traduce() {
        assert_eq!(uiohook_a_tecla(0), None);
        assert_eq!(uiohook_a_tecla(999_999), None);
        // La tecla de menú contextual no está en el enum de uiohook, así que no se graba.
        assert_eq!(tecla_a_uiohook(0x5d, true), None);
    }
}
