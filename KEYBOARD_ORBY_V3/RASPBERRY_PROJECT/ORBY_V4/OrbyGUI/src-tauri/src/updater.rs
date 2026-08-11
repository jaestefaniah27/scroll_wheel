// Avisar de que hay una versión nueva de la app.
//
// **Avisa, no instala.** La versión de Electron descargaba y sustituía el instalador ella
// sola (electron-updater); aquí eso costaría un par de claves minisign, la privada
// disponible al compilar y un `latest.json` firmado publicado en cada release, y saltárselo
// una sola vez deja el actualizador mudo sin que nadie se entere. Para un único usuario que
// instala a mano no sale a cuenta: se consulta la release, se dice que hay una nueva y el
// botón abre su página. Si algún día hay a quién actualizar, esto se cambia por
// `tauri-plugin-updater` sin tocar el frontend más que el texto del botón.
//
// Se pregunta por `releases/latest`, igual que hacía electron-updater, y eso es lo que
// obliga a que **las releases de firmware vayan como prerelease**: `latest` de GitHub no
// mira la etiqueta, mira la fecha, así que una `fw-v*` publicada como definitiva se
// convierte en la última release del repositorio y este avisador ofrecería un firmware como
// si fuera la app. Ya pasó una vez, está documentado en `ORBY_V4/docs/COMPATIBILIDAD.md`.
// Por si vuelve a pasar, abajo se detecta y se dice en voz alta en vez de callar.
//
// Funciona también en desarrollo, al revés que el de Electron: aquel necesitaba un
// instalador que sustituir, y este solo abre una página.

use std::sync::Mutex;
use std::time::Duration;

use serde_json::{json, Value};
use tauri::{AppHandle, Emitter, Manager};

use crate::firmware::{AGENTE, REPO};

/// La app puede pasarse semanas viva en la bandeja sin que nadie la reinicie, así que la
/// comprobación se repite. Mismo intervalo que tenía electron/main.js.
const INTERVALO_MS: u64 = 6 * 60 * 60 * 1000;

/// Margen de cortesía antes de la primera consulta: al arrancar hay cosas con más prisa
/// (el puerto serie, la ventana) y esto puede esperar.
const RETRASO_INICIAL_MS: u64 = 5_000;

struct Compartido {
    estado: Value,
}

/// El estado que espera src/updater.js. `percent` no lo mueve nadie —no hay descarga que
/// medir— pero se mantiene para no tener que tocar la forma del objeto si algún día la hay.
fn inicial(version: &str) -> Value {
    json!({
        "status": "idle",
        "version": version,
        "newVersion": null,
        "percent": 0,
        "error": null,
        "url": null,
    })
}

pub fn iniciar(app: &AppHandle) {
    let version = app.package_info().version.to_string();
    app.manage(Mutex::new(Compartido { estado: inicial(&version) }));

    let handle = app.clone();
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(RETRASO_INICIAL_MS));
        loop {
            consultar(&handle);
            std::thread::sleep(Duration::from_millis(INTERVALO_MS));
        }
    });
}

/// Fusiona `patch` sobre el estado y avisa al renderer, igual que firmware.rs.
fn set(app: &AppHandle, patch: Value) -> Value {
    let estado = app.state::<Mutex<Compartido>>();
    let mut c = estado.lock().unwrap();

    if let (Value::Object(actual), Value::Object(nuevo)) = (&mut c.estado, patch) {
        for (k, v) in nuevo {
            actual.insert(k, v);
        }
    }

    let clon = c.estado.clone();
    let _ = app.emit("updater:state", clon.clone());
    clon
}

fn leer(app: &AppHandle, campo: &str) -> Option<Value> {
    app.state::<Mutex<Compartido>>()
        .lock()
        .unwrap()
        .estado
        .get(campo)
        .cloned()
}

/// La consulta en sí. Devuelve el estado ya publicado, que es lo que contesta el comando.
fn consultar(app: &AppHandle) -> Value {
    set(app, json!({ "status": "checking", "error": null }));

    let url = format!("https://api.github.com/repos/{REPO}/releases/latest");
    let respuesta = reqwest::blocking::Client::new()
        .get(&url)
        .header("Accept", "application/vnd.github+json")
        .header("User-Agent", AGENTE)
        .timeout(Duration::from_secs(20))
        .send()
        .and_then(|r| r.error_for_status())
        .map_err(|e| format!("No se pudo consultar la última versión: {e}"))
        // Igual que firmware.rs: se pide el texto y se parsea aparte, porque reqwest viene
        // sin la característica `json` (menos árbol que compilar).
        .and_then(|r| {
            r.text()
                .map_err(|e| format!("No se pudo leer la respuesta de GitHub: {e}"))
        })
        .and_then(|t| {
            serde_json::from_str::<Value>(&t)
                .map_err(|e| format!("No se entendió la respuesta de GitHub: {e}"))
        });

    let release = match respuesta {
        Ok(v) => v,
        Err(mensaje) => return set(app, json!({ "status": "error", "error": mensaje })),
    };

    let etiqueta = release.get("tag_name").and_then(Value::as_str).unwrap_or("");

    // La red de seguridad del apaño de las prerelease: si la última release del repositorio
    // es de firmware, el aviso se calla y lo dice, en vez de ofrecer un .uf2 como si fuera
    // una versión de la app.
    if etiqueta.starts_with(crate::firmware::PREFIJO_FIRMWARE) {
        return set(
            app,
            json!({
                "status": "error",
                "error": format!(
                    "La última release del repositorio es de firmware ({etiqueta}). \
                     Las de firmware tienen que publicarse como prerelease."
                ),
            }),
        );
    }

    let ultima = etiqueta.trim_start_matches('v');
    let actual = leer(app, "version")
        .and_then(|v| v.as_str().map(str::to_string))
        .unwrap_or_default();

    if ultima.is_empty() {
        return set(app, json!({ "status": "error", "error": "La última release no tiene etiqueta" }));
    }

    // compare_fw compara por partes, no como decimal: "0.10.0" es posterior a "0.9.0".
    let hay_nueva = orby_core::fw::compare_fw(ultima, &actual) == std::cmp::Ordering::Greater;

    if hay_nueva {
        set(
            app,
            json!({
                "status": "available",
                "newVersion": ultima,
                "url": release.get("html_url").and_then(Value::as_str),
            }),
        )
    } else {
        set(app, json!({ "status": "idle", "newVersion": null, "url": null }))
    }
}

// --- Comandos ---------------------------------------------------------------------------

#[tauri::command]
pub fn updater_get(app: AppHandle) -> Value {
    app.state::<Mutex<Compartido>>().lock().unwrap().estado.clone()
}

#[tauri::command]
pub fn updater_check(app: AppHandle) -> Value {
    consultar(&app)
}

/// Abre la página de la release en el navegador. Devuelve si había algo que abrir, que es
/// lo que el renderer espera de `install()`.
#[tauri::command]
pub fn updater_install(app: AppHandle) -> bool {
    let Some(url) = leer(&app, "url").and_then(|v| v.as_str().map(str::to_string)) else {
        return false;
    };
    crate::macros::abrir(&url);
    true
}
