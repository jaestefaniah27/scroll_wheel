// Sin consola detrás de la ventana en la versión publicada, pero con ella en desarrollo:
// es donde salen los rastros del puerto serie cuando la app instalada no detecta el
// teclado, el mismo papel que hoy hace electron/log.js.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod serial;

// El marco de la ventana lo dibuja la propia app (`decorations: false`), así que estos
// tres son la única forma de minimizar, maximizar y cerrar: sin ellos registrados la
// ventana se abre y no hay manera de quitarla más que por el administrador de tareas.
#[tauri::command]
fn window_minimize(ventana: tauri::Window) {
    let _ = ventana.minimize();
}

#[tauri::command]
fn window_maximize(ventana: tauri::Window) {
    // Alterna, como el botón de Electron: el renderer manda el mismo comando las dos
    // veces y espera que la segunda restaure.
    if ventana.is_maximized().unwrap_or(false) {
        let _ = ventana.unmaximize();
    } else {
        let _ = ventana.maximize();
    }
}

#[tauri::command]
fn window_close(ventana: tauri::Window) {
    // De momento cierra de verdad. En Electron esconde a la bandeja, pero la bandeja
    // llega en la Tarea 5: esconderla ahora dejaría la app viva y sin forma de volver.
    let _ = ventana.close();
}

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_notification::init())
        // El hilo del puerto arranca solo y no para: es lo que hace que el teclado se
        // detecte al enchufarlo sin que el usuario toque nada.
        .setup(|app| {
            serial::arrancar(app.handle().clone());
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            window_minimize,
            window_maximize,
            window_close,
            serial::serial_send,
            serial::serial_get_info,
            serial::serial_get_status,
            serial::serial_reconnect
        ])
        .run(tauri::generate_context!())
        .expect("no se pudo arrancar la ventana de OrbyGUI");
}
