// Copias de seguridad a un fichero del PC.
//
// Las tres formas de respuesta (`{ok,path}`, `{ok,data}`, `{ok:false,canceled}`,
// `{ok:false,error}`) son contrato: `src/backup.js` mira `canceled` **antes** que `ok`,
// y es lo que distingue «el usuario ha cerrado el diálogo» —silencioso— de «ha fallado»
// —con aviso en pantalla—. Devolver un `error` al cancelar le echaría la bronca al
// usuario por cerrar una ventana.

use serde_json::{json, Value};
use tauri::{AppHandle, Manager};
use tauri_plugin_dialog::DialogExt;

/// El mismo nombre que propone Electron: `toISOString().slice(0,16)` con los `:` y la `T`
/// cambiados por guiones. Se conserva para que las copias de las dos apps se ordenen
/// juntas en la carpeta.
fn marca_de_tiempo() -> String {
    chrono::Utc::now().format("%Y-%m-%d-%H-%M").to_string()
}

#[tauri::command]
pub async fn backup_save(app: AppHandle, data: Value) -> Value {
    let mut dialogo = app
        .dialog()
        .file()
        .add_filter("Copia de seguridad Orby", &["json"])
        .set_file_name(format!("orby-backup-{}.json", marca_de_tiempo()));

    if let Ok(documentos) = app.path().document_dir() {
        dialogo = dialogo.set_directory(documentos);
    }

    // El diálogo contesta por callback desde otro hilo; el canal es solo para poder
    // esperarlo desde el comando sin bloquear el hilo de la interfaz.
    let (tx, mut rx) = tauri::async_runtime::channel(1);
    dialogo.save_file(move |elegida| {
        let _ = tx.blocking_send(elegida);
    });

    let Some(ruta) = rx.recv().await.flatten().and_then(|f| f.into_path().ok()) else {
        return json!({ "ok": false, "canceled": true });
    };

    // Con sangría, igual que Electron: una copia de seguridad se acaba abriendo a mano.
    let texto = match serde_json::to_string_pretty(&data) {
        Ok(t) => t,
        Err(e) => return json!({ "ok": false, "error": e.to_string() }),
    };

    match std::fs::write(&ruta, texto) {
        Ok(()) => json!({ "ok": true, "path": ruta.to_string_lossy() }),
        Err(e) => json!({ "ok": false, "error": e.to_string() }),
    }
}

#[tauri::command]
pub async fn backup_load(app: AppHandle) -> Value {
    let mut dialogo = app
        .dialog()
        .file()
        .add_filter("Copia de seguridad Orby", &["json"]);

    if let Ok(documentos) = app.path().document_dir() {
        dialogo = dialogo.set_directory(documentos);
    }

    let (tx, mut rx) = tauri::async_runtime::channel(1);
    dialogo.pick_file(move |elegida| {
        let _ = tx.blocking_send(elegida);
    });

    let Some(ruta) = rx.recv().await.flatten().and_then(|f| f.into_path().ok()) else {
        return json!({ "ok": false, "canceled": true });
    };

    let texto = match std::fs::read_to_string(&ruta) {
        Ok(t) => t,
        Err(e) => return json!({ "ok": false, "error": e.to_string() }),
    };

    // El renderer recibe el JSON ya interpretado, no el texto: `src/backup.js` recorre
    // `res.data` directamente.
    match serde_json::from_str::<Value>(&texto) {
        Ok(data) => json!({ "ok": true, "data": data }),
        Err(e) => json!({ "ok": false, "error": e.to_string() }),
    }
}
