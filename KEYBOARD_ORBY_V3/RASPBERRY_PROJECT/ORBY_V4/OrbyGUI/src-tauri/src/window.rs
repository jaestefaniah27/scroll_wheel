// La ventana y el icono de la bandeja.
//
// El marco lo dibuja la app (`decorations: false`), así que los tres botones de arriba a
// la derecha son comandos, no cosa de Windows. Y el de cerrar **no cierra**: esconde a la
// bandeja, igual que hace Electron, porque la app tiene que seguir viva para ejecutar las
// secuencias que el teclado le pide. Cerrar de verdad solo desde el menú de la bandeja.

use std::sync::atomic::{AtomicBool, Ordering};

use tauri::menu::{Menu, MenuItem, PredefinedMenuItem};
use tauri::tray::{MouseButton, MouseButtonState, TrayIconBuilder, TrayIconEvent};
use tauri::{AppHandle, Manager};
use tauri_plugin_notification::NotificationExt;

/// Distingue «esconder» de «salir de verdad». Sin esto, el manejador de cierre se comería
/// también el cierre que pide el menú de la bandeja y la app no habría forma de matarla
/// más que por el administrador de tareas.
static SALIENDO: AtomicBool = AtomicBool::new(false);

/// El aviso de que sigue viva se enseña **una vez**. Repetirlo en cada cierre convierte
/// una explicación útil en una molestia.
static AVISO_ENSENADO: AtomicBool = AtomicBool::new(false);

pub fn saliendo() -> bool {
    SALIENDO.load(Ordering::SeqCst)
}

fn mostrar(app: &AppHandle) {
    let Some(ventana) = app.get_webview_window("main") else { return };
    // Si estaba minimizada, `show()` sola la deja escondida en la barra de tareas.
    if ventana.is_minimized().unwrap_or(false) {
        let _ = ventana.unminimize();
    }
    let _ = ventana.show();
    let _ = ventana.set_focus();
}

/// Esconde la ventana y, la primera vez, explica por qué la app sigue en la bandeja.
pub fn esconder(app: &AppHandle) {
    if let Some(ventana) = app.get_webview_window("main") {
        let _ = ventana.hide();
    }

    if !AVISO_ENSENADO.swap(true, Ordering::SeqCst) {
        let _ = app
            .notification()
            .builder()
            .title("OrbyGUI sigue activo")
            .body(
                "Se quedó en la bandeja del sistema para poder seguir ejecutando \
                 secuencias. Para cerrarlo del todo, usa «Salir» en el icono de la bandeja.",
            )
            .show();
    }
}

pub fn montar_bandeja(app: &AppHandle) -> tauri::Result<()> {
    let abrir = MenuItem::with_id(app, "abrir", "Abrir OrbyGUI", true, None::<&str>)?;
    let separador = PredefinedMenuItem::separator(app)?;
    let salir = MenuItem::with_id(app, "salir", "Salir", true, None::<&str>)?;
    let menu = Menu::with_items(app, &[&abrir, &separador, &salir])?;

    let mut constructor = TrayIconBuilder::with_id("principal")
        .tooltip("OrbyGUI")
        .menu(&menu)
        // Con el menú en el clic izquierdo no quedaría forma de esconder la ventana desde
        // la bandeja: el clic izquierdo alterna, el derecho abre el menú.
        .show_menu_on_left_click(false)
        .on_menu_event(|app, evento| match evento.id.as_ref() {
            "abrir" => mostrar(app),
            "salir" => {
                SALIENDO.store(true, Ordering::SeqCst);
                app.exit(0);
            }
            _ => {}
        })
        .on_tray_icon_event(|bandeja, evento| {
            let app = bandeja.app_handle();
            match evento {
                TrayIconEvent::Click {
                    button: MouseButton::Left,
                    button_state: MouseButtonState::Up,
                    ..
                } => {
                    let visible = app
                        .get_webview_window("main")
                        .and_then(|v| v.is_visible().ok())
                        .unwrap_or(false);
                    if visible {
                        // Aquí se esconde sin el aviso: quien usa el icono de la bandeja
                        // ya sabe dónde está la app.
                        if let Some(v) = app.get_webview_window("main") {
                            let _ = v.hide();
                        }
                    } else {
                        mostrar(app);
                    }
                }
                TrayIconEvent::DoubleClick { .. } => mostrar(app),
                _ => {}
            }
        });

    // El icono sale del que ya lleva la ventana, no de `assets/orby-icon.png`: ese fichero
    // es en realidad un JPEG con extensión `.png` y no todos los cargadores se lo tragan.
    if let Some(icono) = app.default_window_icon() {
        constructor = constructor.icon(icono.clone());
    }

    constructor.build(app)?;
    Ok(())
}

#[tauri::command]
pub fn window_minimize(ventana: tauri::Window) {
    let _ = ventana.minimize();
}

#[tauri::command]
pub fn window_maximize(ventana: tauri::Window) {
    // Alterna, como el botón de Electron: el renderer manda el mismo comando las dos veces
    // y espera que la segunda restaure.
    if ventana.is_maximized().unwrap_or(false) {
        let _ = ventana.unmaximize();
    } else {
        let _ = ventana.maximize();
    }
}

#[tauri::command]
pub fn window_close(app: AppHandle) {
    esconder(&app);
}
