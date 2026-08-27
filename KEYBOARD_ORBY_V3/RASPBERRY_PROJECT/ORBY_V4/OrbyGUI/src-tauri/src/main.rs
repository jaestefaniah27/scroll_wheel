// Sin consola detrás de la ventana en la versión publicada, pero con ella en desarrollo:
// es donde salen los rastros del puerto serie cuando la app instalada no detecta el
// teclado, el mismo papel que hoy hace electron/log.js.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod apps;
mod autostart;
mod backup;
mod config;
mod dialog;
mod firmware;
mod foreground;
mod log;
mod macros;
mod mouse;
mod plugins;
mod recorder;
mod serial;
mod updater;
mod window;

use tauri::Manager;

fn main() {
    // Arrancar escondido es lo que hace útil el autoarranque: al iniciar sesión la app se
    // queda en la bandeja en vez de plantar la ventana delante de lo que estuvieras
    // haciendo. El argumento lo pone `autostart::escribir` en la entrada del registro.
    let escondido = std::env::args().any(|arg| arg == autostart::ARG_ESCONDIDO);

    tauri::Builder::default()
        // El primero de todos: si esta es una segunda instancia, el plugin se lo dice a la
        // que ya corre y esta se apaga sin haber llegado a abrir el puerto serie.
        .plugin(tauri_plugin_single_instance::init(|app, args, _cwd| {
            // Un segundo arranque con `--hidden` es el autoarranque llegando tarde: sacar
            // la ventana ahí sería lo contrario de lo que pide el argumento. Sin él es un
            // doble clic en el acceso directo, y lo que se espera es ver la app.
            if args.iter().any(|arg| arg == autostart::ARG_ESCONDIDO) {
                return;
            }
            window::mostrar(app);
        }))
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_notification::init())
        // Lo usa el paso de texto de una secuencia: por encima de cinco caracteres se pega
        // en vez de teclearse.
        .plugin(tauri_plugin_clipboard_manager::init())
        // La app se descarga e instala sus propias versiones nuevas (ver updater.rs). El
        // plugin solo aporta el motor: quién comprueba y cuándo se instala lo decide
        // `updater::iniciar`, más abajo.
        .plugin(tauri_plugin_updater::Builder::new().build())
        .setup(move |app| {
            // El hilo del puerto arranca solo y no para: es lo que hace que el teclado se
            // detecte al enchufarlo sin que el usuario toque nada.
            serial::arrancar(app.handle().clone());
            foreground::arrancar_si_procede(app.handle().clone());
            firmware::iniciar(app.handle());
            updater::iniciar(app.handle());
            plugins::iniciar(app.handle());
            window::montar_bandeja(app.handle())?;

            // La ventana se declara `visible: false` en tauri.conf.json, así que enseñarla
            // es un paso explícito. Con `--hidden` no solo no se enseña: se destruye, para
            // no dejar el WebView2 cargado toda la sesión (ver window::arrancar_en_bandeja).
            if escondido {
                window::arrancar_en_bandeja(app.handle());
            } else {
                window::mostrar(app.handle());
            }

            // Entradas de autoarranque anteriores a que existiera `--hidden`: sin el
            // argumento abrirían la ventana al iniciar sesión.
            autostart::normalizar();
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
            plugins::plugins_open_folder,
            autostart::autostart_get,
            autostart::autostart_set,
            updater::updater_get,
            updater::updater_check,
            updater::updater_install,
            updater::updater_ocupado
        ])
        .build(tauri::generate_context!())
        .expect("no se pudo arrancar la ventana de OrbyGUI")
        // Salir a mitad de la reproducción de una grabación dejaría hundidas las teclas
        // que llevara pulsadas, y ahí ya no queda ninguna app viva que pueda soltarlas.
        // `Exit` es el último punto en el que todavía se puede.
        .run(|_app, evento| match evento {
            tauri::RunEvent::Exit => recorder::apagar(),
            // Sin esto, destruir la ventana al ir a la bandeja (ver `window::esconder`)
            // se toma como «se cerró la última ventana» y tao apaga el proceso entero,
            // llevándose por delante el puerto serie y el icono de la bandeja.
            tauri::RunEvent::ExitRequested { api, .. } => {
                if !window::saliendo() {
                    api.prevent_exit();
                }
            }
            _ => {}
        });
}
