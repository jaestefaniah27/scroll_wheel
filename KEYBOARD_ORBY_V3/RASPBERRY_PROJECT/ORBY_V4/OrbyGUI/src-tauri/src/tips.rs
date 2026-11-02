// Vídeos de los consejos: se bajan una vez de GitHub y se guardan en el PC.
//
// Qué falta y qué manifiesto es válido lo decide `orby_core::consejos`; aquí solo está el
// disco y la red. Viven en `%APPDATA%\OrbyGUI\tips\` —la misma base que config.rs, no
// `app_config_dir()`, que resolvería a `com.orby.gui`— y se le entregan a la ventana como
// `data:` URL: son ≤ 300 KB, y así no hace falta abrir el protocolo de assets de Tauri
// ni decidir su alcance sobre el disco del usuario.

use std::path::PathBuf;
use std::time::Duration;

use base64::Engine;
use orby_core::consejos::{faltan, leer_manifiesto, sha256_hex, EntradaVideo};
use serde_json::{json, Value};

use crate::log::log;

const BASE: &str = "https://raw.githubusercontent.com/jaestefaniah27/scroll_wheel/main/\
KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/tips-video/";
const MANIFIESTO: &str = "manifest.json";
const ESPERA: Duration = Duration::from_secs(20);

fn carpeta() -> Option<PathBuf> {
    let appdata = std::env::var_os("APPDATA")?;
    Some(PathBuf::from(appdata).join("OrbyGUI").join("tips"))
}

fn bajar(nombre: &str) -> Result<Vec<u8>, String> {
    let resp = reqwest::blocking::Client::builder()
        .timeout(ESPERA)
        .build()
        .map_err(|e| e.to_string())?
        .get(format!("{BASE}{nombre}"))
        .header("User-Agent", crate::firmware::AGENTE)
        .send()
        .map_err(|e| e.to_string())?;
    if !resp.status().is_success() {
        return Err(format!("{} en {nombre}", resp.status()));
    }
    resp.bytes().map(|b| b.to_vec()).map_err(|e| e.to_string())
}

/// El manifiesto de internet si hay red; si no, el último guardado. Un manifiesto que no
/// da ni una entrada válida no pisa al bueno que ya hubiera.
fn manifiesto(dir: &PathBuf) -> Vec<EntradaVideo> {
    let local = dir.join(MANIFIESTO);
    if let Ok(bytes) = bajar(MANIFIESTO) {
        if let Ok(texto) = String::from_utf8(bytes) {
            let remoto = leer_manifiesto(&texto);
            if !remoto.is_empty() {
                let _ = std::fs::write(&local, texto);
                return remoto;
            }
        }
    }
    std::fs::read_to_string(&local).map(|t| leer_manifiesto(&t)).unwrap_or_default()
}

fn presentes(dir: &PathBuf, m: &[EntradaVideo]) -> Vec<(String, String)> {
    m.iter()
        .filter_map(|e| {
            let datos = std::fs::read(dir.join(&e.fichero)).ok()?;
            Some((e.fichero.clone(), sha256_hex(&datos)))
        })
        .collect()
}

/// Baja lo que falte. Cada vídeo se escribe a `.part` y se renombra solo si el hash
/// coincide: una descarga cortada no puede quedar como vídeo bueno.
pub fn sincronizar() -> Value {
    let Some(dir) = carpeta() else { return json!({ "descargados": 0, "fallos": 0 }) };
    if std::fs::create_dir_all(&dir).is_err() {
        return json!({ "descargados": 0, "fallos": 0 });
    }
    let m = manifiesto(&dir);
    let (mut ok, mut mal) = (0, 0);
    for e in faltan(&m, &presentes(&dir, &m)) {
        let hecho = bajar(&e.fichero).and_then(|datos| {
            if sha256_hex(&datos) != e.sha256 {
                return Err("el hash no coincide".to_string());
            }
            let parcial = dir.join(format!("{}.part", e.fichero));
            std::fs::write(&parcial, datos).map_err(|x| x.to_string())?;
            std::fs::rename(&parcial, dir.join(&e.fichero)).map_err(|x| x.to_string())
        });
        match hecho {
            Ok(()) => ok += 1,
            Err(x) => {
                mal += 1;
                log!("[consejos] no se pudo bajar {}: {x}", e.fichero);
            }
        }
    }
    json!({ "descargados": ok, "fallos": mal })
}

/// Al arrancar y en segundo plano: el primer arranque tras instalar es el que los baja.
pub fn iniciar() {
    std::thread::spawn(|| {
        sincronizar();
    });
}

#[tauri::command]
pub async fn tips_sync() -> Value {
    tauri::async_runtime::spawn_blocking(sincronizar).await.unwrap_or_else(|_| json!({ "descargados": 0, "fallos": 1 }))
}

/// La `data:` URL del vídeo, o `None` si no está bajado o su hash ya no cuadra con el
/// manifiesto guardado (se comprueba aquí para no enseñar un fichero a medias).
#[tauri::command]
pub async fn tips_video_src(id: String) -> Option<String> {
    tauri::async_runtime::spawn_blocking(move || {
        let dir = carpeta()?;
        let m = leer_manifiesto(&std::fs::read_to_string(dir.join(MANIFIESTO)).ok()?);
        let e = m.into_iter().find(|e| e.id == id)?;
        let datos = std::fs::read(dir.join(&e.fichero)).ok()?;
        if sha256_hex(&datos) != e.sha256 {
            return None;
        }
        Some(format!("data:video/mp4;base64,{}", base64::engine::general_purpose::STANDARD.encode(datos)))
    })
    .await
    .ok()
    .flatten()
}
