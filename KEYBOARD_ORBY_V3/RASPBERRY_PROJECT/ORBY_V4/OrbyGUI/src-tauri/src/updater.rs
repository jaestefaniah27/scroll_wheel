// Actualizar la propia app: **sola, en silencio y sin preguntar**.
//
// Hasta la 1.0 esto solo avisaba: consultaba `releases/latest` y el botón abría la página
// de la release para instalarla a mano. Ahora descarga, instala y reinicia, sin que nadie
// apriete nada. Lo que costaba —y por lo que se aplazó— es la infraestructura: un par de
// claves minisign, la privada disponible al compilar y un `latest.json` firmado publicado
// **en cada release** junto al `.exe`. Todo eso está documentado en `docs/PUBLICACION.md`,
// y el pipeline del panel Orby Manager comprueba la clave *antes* de compilar, porque
// saltárselo una sola vez deja el actualizador mudo sin que nadie se entere.
//
// **La forma del estado no cambia**, a propósito: el renderer ya consumía
// `{status, version, newVersion, percent, error, url}` por `updater:state`, así que aquí
// solo cambia el interior y quién mueve `percent` (antes nadie, porque no había descarga).
//
// ## Dos consultas distintas, y por qué
//
// El plugin de Tauri pide el `latest.json` de la release, que es lo único que sabe firmar
// y verificar. Pero eso **no** detecta el fallo de publicar una release de firmware sin
// `--prerelease`: `releases/latest` de GitHub devuelve la más reciente sin mirar la
// etiqueta, y una `fw-v*` marcada como definitiva se convierte en «la última release» y
// se lleva por delante la URL del `latest.json`. Ya pasó una vez (documentado en
// `ORBY_V4/docs/COMPATIBILIDAD.md`), así que antes de nada se mira la etiqueta por la API
// y, si es de firmware, se dice en voz alta en vez de fallar con un 404 incomprensible.
//
// ## Las guardas antes de reinstalar
//
// «Inmediato» no puede significar «a media faena». Reiniciar la app mientras se está
// flasheando el teclado lo deja a medio firmware —y eso no se arregla solo—, y hacerlo a
// mitad de una grabación o de una secuencia se lleva por delante trabajo del usuario. En
// esos casos la actualización se queda descargada y espera a la siguiente vuelta.

use std::sync::Mutex;
use std::time::Duration;

use serde_json::{json, Value};
use tauri::{AppHandle, Emitter, Manager};
use tauri_plugin_updater::UpdaterExt;

use crate::firmware::{AGENTE, REPO};

/// La app puede pasarse semanas viva en la bandeja sin que nadie la reinicie, así que la
/// comprobación se repite.
const INTERVALO_MS: u64 = 6 * 60 * 60 * 1000;

/// Margen de cortesía antes de la primera consulta: al arrancar hay cosas con más prisa
/// (el puerto serie, la ventana) y esto puede esperar.
const RETRASO_INICIAL_MS: u64 = 5_000;

/// Cada cuánto se vuelve a mirar si ya se puede reinstalar, con una actualización
/// descargada esperando a que el teclado deje de estar en mitad de algo.
const REINTENTO_GUARDA_MS: u64 = 60_000;

struct Compartido {
    estado: Value,
    /// La actualización ya bajada e instalada, esperando el reinicio. No se guarda el
    /// objeto del plugin porque `install` lo consume: basta con saber que se llegó ahí.
    lista_para_reiniciar: bool,
}

/// El estado que espera `src/updater.js`.
fn inicial(version: &str, automatico: bool) -> Value {
    json!({
        "status": "idle",
        "version": version,
        "newVersion": null,
        "percent": 0,
        "error": null,
        "url": null,
        "auto": automatico,
    })
}

/// Si el usuario ha apagado el automático en Ajustes. Vive en la configuración local, no
/// en memoria, porque tiene que sobrevivir al reinicio que provoca la propia actualización.
fn automatico_guardado() -> bool {
    crate::config::leer()
        .get("autoUpdate")
        .and_then(Value::as_bool)
        // Automático por defecto: quien no toca nada se actualiza solo, que es el objetivo.
        .unwrap_or(true)
}

pub fn iniciar(app: &AppHandle) {
    let version = app.package_info().version.to_string();
    app.manage(Mutex::new(Compartido {
        estado: inicial(&version, automatico_guardado()),
        lista_para_reiniciar: false,
    }));

    let handle = app.clone();
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(RETRASO_INICIAL_MS));
        loop {
            ciclo(&handle);
            std::thread::sleep(Duration::from_millis(INTERVALO_MS));
        }
    });
}

/// Una vuelta completa: consultar y, si procede, instalar. Se separa de `iniciar` para que
/// el comando `updater_check` pueda hacer exactamente lo mismo desde su propio hilo.
fn ciclo(app: &AppHandle) {
    let estado = consultar(app);
    let hay_nueva = estado.get("status").and_then(Value::as_str) == Some("available");
    if !hay_nueva || !leer_bool(app, "auto") {
        return;
    }

    if !descargar_e_instalar(app) {
        return;
    }

    // Con la actualización ya instalada, lo único que falta es el reinicio. Si el momento
    // no es bueno, se espera: la app ya está actualizada en disco y el reinicio puede
    // llegar cuando toque, incluso el que haga el usuario por su cuenta.
    loop {
        if let Some(motivo) = mal_momento(app) {
            set(app, json!({ "status": "downloaded", "error": null }));
            eprintln!("[updater] instalada, esperando para reiniciar: {motivo}");
            std::thread::sleep(Duration::from_millis(REINTENTO_GUARDA_MS));
            continue;
        }
        reiniciar(app);
        return;
    }
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

fn leer_bool(app: &AppHandle, campo: &str) -> bool {
    leer(app, campo).and_then(|v| v.as_bool()).unwrap_or(false)
}

/// Por qué **no** se puede reiniciar ahora mismo, o `None` si se puede.
///
/// Devuelve el motivo en texto y no un `bool` porque acaba en el log: con una app que se
/// reinicia sola, «no se ha reiniciado» sin explicación es imposible de diagnosticar.
fn mal_momento(app: &AppHandle) -> Option<String> {
    // Lo más caro de todo: el teclado a medio flashear no arranca, y de ahí solo se sale
    // a mano con el botón BOOTSEL.
    let fw = crate::firmware::firmware_get(app.clone());
    if let Some(s) = fw.get("status").and_then(Value::as_str) {
        if ["downloading", "bootsel", "flashing"].contains(&s) {
            return Some(format!("hay un firmware instalándose ({s})"));
        }
    }

    let grabadora = crate::recorder::recorder_status();
    if grabadora.get("recording").and_then(Value::as_bool) == Some(true) {
        return Some("la grabadora está grabando".into());
    }
    if !grabadora.get("playingId").map(Value::is_null).unwrap_or(true) {
        return Some("hay una secuencia reproduciéndose".into());
    }

    None
}

/// Sale de verdad y vuelve a entrar. **Tiene que pasar por `window::marcar_saliendo`**: el
/// manejador de `CloseRequested` cancela el cierre para esconder a la bandeja, y sin eso
/// el reinicio se quedaría en un intento silencioso de cerrar la ventana.
fn reiniciar(app: &AppHandle) {
    set(app, json!({ "status": "downloaded", "percent": 100 }));
    crate::window::marcar_saliendo();
    app.restart();
}

/// Descarga e instala. Devuelve si quedó lista para reiniciar.
fn descargar_e_instalar(app: &AppHandle) -> bool {
    // En desarrollo no hay instalador que sustituir: el plugin ni siquiera se puede
    // construir. Se sale sin ruido, y el frontend sigue enseñando su estado `dev`.
    let actualizador = match app.updater() {
        Ok(u) => u,
        Err(e) => {
            eprintln!("[updater] no hay actualizador en esta build: {e}");
            return false;
        }
    };

    let pendiente = match tauri::async_runtime::block_on(actualizador.check()) {
        Ok(Some(u)) => u,
        // El plugin no ve nada que instalar aunque la API de GitHub dijera que sí: lo
        // normal es que a la release le falte el `latest.json`. Se dice, porque es
        // exactamente el fallo mudo que este módulo existe para evitar.
        Ok(None) => {
            set(app, json!({
                "status": "error",
                "error": "Hay una versión nueva publicada pero sin latest.json firmado: \
                          no se puede instalar sola. Ver docs/PUBLICACION.md.",
            }));
            return false;
        }
        Err(e) => {
            set(app, json!({ "status": "error", "error": format!("No se pudo preparar la actualización: {e}") }));
            return false;
        }
    };

    set(app, json!({ "status": "downloading", "percent": 0, "error": null }));

    let total = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let bajados = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let (t, b, app_pct) = (total.clone(), bajados.clone(), app.clone());

    let resultado = tauri::async_runtime::block_on(pendiente.download_and_install(
        move |trozo, tam| {
            if let Some(tam) = tam {
                t.store(tam, std::sync::atomic::Ordering::Relaxed);
            }
            let hechos = b.fetch_add(trozo as u64, std::sync::atomic::Ordering::Relaxed) + trozo as u64;
            let tam = t.load(std::sync::atomic::Ordering::Relaxed);
            if tam > 0 {
                let pct = (hechos * 100 / tam).min(100);
                set(&app_pct, json!({ "percent": pct }));
            }
        },
        || {},
    ));

    match resultado {
        Ok(()) => {
            app.state::<Mutex<Compartido>>().lock().unwrap().lista_para_reiniciar = true;
            set(app, json!({ "status": "downloaded", "percent": 100 }));
            true
        }
        Err(e) => {
            set(app, json!({ "status": "error", "error": format!("No se pudo instalar la actualización: {e}") }));
            false
        }
    }
}

/// ¿Hay versión nueva? Por la API de GitHub, no por el plugin: es la única forma de
/// distinguir «no hay nada nuevo» de «la última release es de firmware» (ver la cabecera).
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
    // es de firmware, se dice, en vez de dejar al plugin buscando un latest.json que en esa
    // release no existe y contar un 404 que no explica nada.
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

/// Comprobar a mano desde Ajustes. **No bloquea**: la consulta se va a su hilo y esto
/// contesta el estado `checking` al momento.
///
/// Antes era síncrono y se comía hasta 20 s del hilo de comandos de Tauri, con la interfaz
/// esperando la respuesta del `invoke`. Con el automático puesto encima —que puede
/// encadenar consulta, descarga e instalación— eso ya no vale.
#[tauri::command]
pub fn updater_check(app: AppHandle) -> Value {
    let handle = app.clone();
    std::thread::spawn(move || ciclo(&handle));
    set(&app, json!({ "status": "checking", "error": null }))
}

/// El botón de Ajustes con el automático apagado. Instala y reinicia igual, pero solo
/// porque lo ha pedido el usuario.
///
/// Devuelve si había algo que hacer, que es lo que el renderer espera de `install()`.
#[tauri::command]
pub fn updater_install(app: AppHandle) -> bool {
    let estado = leer(&app, "status").and_then(|v| v.as_str().map(str::to_string));
    match estado.as_deref() {
        // Ya instalada y esperando: el usuario acaba de dar permiso para el reinicio, así
        // que se salta la espera del buen momento. Es su decisión, no la nuestra.
        Some("downloaded") if app.state::<Mutex<Compartido>>().lock().unwrap().lista_para_reiniciar => {
            reiniciar(&app);
            true
        }
        Some("available") => {
            let handle = app.clone();
            std::thread::spawn(move || {
                if descargar_e_instalar(&handle) {
                    reiniciar(&handle);
                }
            });
            true
        }
        _ => false,
    }
}

/// El interruptor de Ajustes. Se guarda en la configuración local para que sobreviva al
/// reinicio que provoca la propia actualización.
#[tauri::command]
pub fn updater_set_auto(app: AppHandle, enabled: bool) -> Value {
    crate::config::config_set(json!({ "autoUpdate": enabled }));
    set(&app, json!({ "auto": enabled }))
}
