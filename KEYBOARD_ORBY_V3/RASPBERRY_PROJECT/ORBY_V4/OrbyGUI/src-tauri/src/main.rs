// Sin consola detrás de la ventana en la versión publicada, pero con ella en desarrollo:
// es donde salen los rastros del puerto serie cuando la app instalada no detecta el
// teclado, el mismo papel que hoy hace electron/log.js.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod apps;
mod backup;
mod config;
mod dialog;
mod firmware;
mod foreground;
mod macros;
mod mouse;
mod plugins;
mod recorder;
mod serial;
mod window;

use tauri::Manager;

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_notification::init())
        // Lo usa el paso de texto de una secuencia: por encima de cinco caracteres se pega
        // en vez de teclearse.
        .plugin(tauri_plugin_clipboard_manager::init())
        .setup(|app| {
            // El hilo del puerto arranca solo y no para: es lo que hace que el teclado se
            // detecte al enchufarlo sin que el usuario toque nada.
            serial::arrancar(app.handle().clone());
            foreground::arrancar_si_procede(app.handle().clone());
            firmware::iniciar(app.handle());
            plugins::iniciar(app.handle());
            window::montar_bandeja(app.handle())?;
            Ok(())
        })
        // Cerrar la ventana esconde a la bandeja en vez de cerrar. Se intercepta aquí y no
        // solo en el comando `window_close` porque Alt+F4 no pasa por el comando, y sin
        // esto se llevaría por delante la app con el teclado esperando que ejecute
        // secuencias.
        .on_window_event(|ventana, evento| {
            if let tauri::WindowEvent::CloseRequested { api, .. } = evento {
                if !window::saliendo() {
                    api.prevent_close();
                    window::esconder(ventana.app_handle());
                }
            }
        })
        .invoke_handler(tauri::generate_handler![
            window::window_minimize,
            window::window_maximize,
            window::window_close,
            serial::serial_send,
            serial::serial_get_info,
            serial::serial_get_status,
            serial::serial_reconnect,
            config::config_get,
            config::config_set,
            backup::backup_save,
            backup::backup_load,
            dialog::dialog_pick_app_or_file,
            apps::apps_list_installed,
            mouse::mouse_get_position,
            recorder::recorder_toggle,
            recorder::recorder_stop,
            recorder::recorder_status,
            foreground::foreground_start,
            foreground::foreground_stop,
            foreground::foreground_current,
            foreground::foreground_available,
            firmware::firmware_get,
            firmware::firmware_check,
            firmware::firmware_update,
            firmware::firmware_cancel,
            plugins::plugins_list,
            plugins::plugins_install,
            plugins::plugins_uninstall,
            plugins::plugins_set_enabled,
            plugins::plugins_get_settings,
            plugins::plugins_set_settings,
            plugins::plugins_test,
            plugins::plugins_read,
            plugins::plugins_open_folder
        ])
        .build(tauri::generate_context!())
        .expect("no se pudo arrancar la ventana de OrbyGUI")
        // Salir a mitad de la reproducción de una grabación dejaría hundidas las teclas
        // que llevara pulsadas, y ahí ya no queda ninguna app viva que pueda soltarlas.
        // `Exit` es el último punto en el que todavía se puede.
        .run(|_app, evento| {
            if let tauri::RunEvent::Exit = evento {
                recorder::apagar();
            }
        });
}
