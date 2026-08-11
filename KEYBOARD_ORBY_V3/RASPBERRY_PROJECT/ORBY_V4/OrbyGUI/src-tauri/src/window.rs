// La ventana y el icono de la bandeja.
//
// El marco lo dibuja la app (`decorations: false`), así que los tres botones de arriba a
// la derecha son comandos, no cosa de Windows. Y el de cerrar **no cierra**: esconde a la
// bandeja, igual que hace Electron, porque la app tiene que seguir viva para ejecutar las
// secuencias que el teclado le pide. Cerrar de verdad solo desde el menú de la bandeja.
//
// «Esconder» no es `hide()`: es `destroy()`. `hide()` deja el WebView2 cargado y pintando
// de fondo, que es lo que gastaba RAM en segundo plano sin motivo — todo lo que sigue vivo
// en la bandeja (puerto serie, macros, firmware, complementos) vive en `main.rs::setup` y
// no toca la ventana para nada, así que no hace falta conservarla. Al volver a abrir se
// reconstruye desde el mismo `WindowConfig` de `tauri.conf.json`, para no duplicar aquí
// título, tamaño ni color de fondo.

use std::sync::atomic::{AtomicBool, Ordering};

use tauri::menu::{Menu, MenuItem, PredefinedMenuItem};
use tauri::tray::{MouseButton, MouseButtonState, TrayIconBuilder, TrayIconEvent};
use tauri::{AppHandle, Manager, WebviewWindow, WebviewWindowBuilder};
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

/// Reconstruye la ventana «main» desde el `WindowConfig` de `tauri.conf.json`, porque
/// `esconder_ventana` la destruye del todo en vez de solo esconderla.
fn reconstruir(app: &AppHandle) -> tauri::Result<WebviewWindow> {
    let conf = app
        .config()
        .app
        .windows
        .iter()
        .find(|c| c.label == "main")
        .expect("tauri.conf.json tiene que declarar la ventana «main»")
        .clone();
    WebviewWindowBuilder::from_config(app, &conf)?.build()
}

/// Pública porque el arranque normal también la usa: la ventana se declara `visible: false`
/// en tauri.conf.json —si no, arrancar con `--hidden` la deja asomar un instante antes de
/// que nadie pueda esconderla—, así que enseñarla es siempre un paso explícito.
pub fn mostrar(app: &AppHandle) {
    let ventana = match app.get_webview_window("main") {
        Some(v) => v,
        None => match reconstruir(app) {
            Ok(v) => v,
            Err(_) => return,
        },
    };
    // Si estaba minimizada, `show()` sola la deja escondida en la barra de tareas.
    if ventana.is_minimized().unwrap_or(false) {
        let _ = ventana.unminimize();
    }
    let _ = ventana.show();
    let _ = ventana.set_focus();
}

/// Destruye la ventana y el WebView2 que lleva dentro. El puerto serie, las macros, el
/// firmware y los complementos siguen corriendo en el proceso: no dependen de la ventana.
fn esconder_ventana(app: &AppHandle) {
    if let Some(ventana) = app.get_webview_window("main") {
        let _ = ventana.destroy();
    }
}

/// Arranque con `--hidden` (autoarranque con la sesión): se destruye la ventana que declara
/// la configuración antes de que llegue a enseñarse. No basta con no llamar a `mostrar`:
/// eso dejaría el WebView2 cargado y ocupando memoria toda la sesión, que es justo lo que
/// se evita al irse a la bandeja.
pub fn arrancar_en_bandeja(app: &AppHandle) {
    esconder_ventana(app);
}

/// Esconde la ventana y, la primera vez, explica por qué la app sigue en la bandeja.
pub fn esconder(app: &AppHandle) {
    esconder_ventana(app);

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
                        esconder_ventana(app);
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
