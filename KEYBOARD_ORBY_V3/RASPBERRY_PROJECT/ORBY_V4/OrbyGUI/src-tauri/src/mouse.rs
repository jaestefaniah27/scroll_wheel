// La posición del cursor, para capturar puntos al editar una secuencia.
//
// Sale de `GetCursorPos`, que da **píxeles físicos** del escritorio virtual. Es el mismo
// espacio en el que trabaja `SendInput` con coordenadas absolutas, que es como las
// reproduce macros.rs. Que la captura y la reproducción compartan espacio no es un
// detalle: si una fuera en píxeles lógicos y la otra en físicos, las secuencias con
// posiciones absolutas pincharían en un sitio distinto en cuanto el monitor tuviera
// escalado, y eso se diagnostica fatal porque en un monitor al 100 % funciona.
//
// La app declara conciencia de DPI por monitor (lo pone Tauri en su manifiesto), así que
// Windows no miente en ninguna de las dos llamadas.

use serde_json::{json, Value};
use windows::Win32::Foundation::POINT;
use windows::Win32::UI::WindowsAndMessaging::{
    GetCursorPos, GetSystemMetrics, SM_CXVIRTUALSCREEN, SM_CYVIRTUALSCREEN, SM_XVIRTUALSCREEN,
    SM_YVIRTUALSCREEN,
};

/// El rectángulo del escritorio virtual (todas las pantallas juntas), en el mismo
/// espacio físico que `GetCursorPos`.
///
/// Lo necesita el teclado para reproducir una posición sin la app: su puntero absoluto
/// va de 0 a 32767 sobre la superficie entera, así que un píxel solo se puede traducir
/// sabiendo de qué superficie es. Se guarda junto a cada posición capturada y en cada
/// grabación (ver src/native/recording.mjs).
pub fn escritorio() -> Value {
    unsafe {
        json!({
            "x": GetSystemMetrics(SM_XVIRTUALSCREEN),
            "y": GetSystemMetrics(SM_YVIRTUALSCREEN),
            "w": GetSystemMetrics(SM_CXVIRTUALSCREEN),
            "h": GetSystemMetrics(SM_CYVIRTUALSCREEN),
        })
    }
}

#[tauri::command]
pub fn mouse_get_position() -> Value {
    let mut punto = POINT::default();
    // Solo falla si el escritorio no está accesible (sesión bloqueada, cambio de usuario);
    // devolver el origen es más útil que reventar el comando.
    if unsafe { GetCursorPos(&mut punto) }.is_err() {
        return json!({ "x": 0, "y": 0, "desk": escritorio() });
    }
    json!({ "x": punto.x, "y": punto.y, "desk": escritorio() })
}
