// Elegir un programa o un fichero: la pestaña «App» de una tecla y el paso «Abrir…» de
// una secuencia.
//
// El orden de los filtros cambia según para qué se pide, porque el primero es el que sale
// preseleccionado: pidiendo una aplicación, enseñar por defecto «todos los archivos»
// obliga a rebuscar el `.exe` entre todo lo demás.

use serde_json::{json, Value};
use tauri::{AppHandle, Manager};
use tauri_plugin_dialog::DialogExt;

#[tauri::command]
pub async fn dialog_pick_app_or_file(app: AppHandle, kind: Option<String>) -> Value {
    let es_fichero = kind.as_deref() == Some("file");

    let mut dialogo = app.dialog().file().set_title(if es_fichero {
        "Elegir archivo"
    } else {
        "Elegir aplicación"
    });

    dialogo = if es_fichero {
        dialogo
            .add_filter("Todos los archivos", &["*"])
            .add_filter("Programas", &["exe", "lnk", "bat", "cmd"])
    } else {
        dialogo
            .add_filter("Programas", &["exe", "lnk", "bat", "cmd"])
            .add_filter("Todos los archivos", &["*"])
    };

    if let Ok(documentos) = app.path().document_dir() {
        dialogo = dialogo.set_directory(documentos);
    }

    let (tx, mut rx) = tauri::async_runtime::channel(1);
    dialogo.pick_file(move |elegida| {
        let _ = tx.blocking_send(elegida);
    });

    match rx.recv().await.flatten().and_then(|f| f.into_path().ok()) {
        Some(ruta) => json!({ "ok": true, "path": ruta.to_string_lossy() }),
        None => json!({ "ok": false, "canceled": true }),
    }
}
