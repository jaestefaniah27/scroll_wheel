// Actualizar el firmware del teclado desde la app.
//
// Espejo de electron/firmware.js: el RP2040 no se actualiza por el puerto serie, se
// reinicia en el cargador de la ROM, aparece como una unidad USB llamada RPI-RP2, y
// flashear es copiar un .uf2 dentro. Este módulo automatiza esos tres pasos.
//
// Qué versión ofrecer ya está decidido en `orby_core::releases` (filtrado, orden,
// comparación de versiones): aquí solo falta la llamada HTTP, la descarga con progreso, la
// entrada en el cargador y la copia.
//
// Las releases de firmware van etiquetadas `fw-vX.Y` y publicadas como **prerelease**: es
// el mismo repositorio que la app, y `releases/latest` de GitHub no mira la etiqueta, mira
// la fecha. Una release de firmware publicada como definitiva se convierte en
// `releases/latest` y rompe el actualizador de toda la app instalada. Ya pasó una vez, está
// documentado en `ORBY_V4/docs/COMPATIBILIDAD.md`.

use std::io::Read;
use std::path::PathBuf;
use std::sync::Mutex;
use std::time::Duration;

use orby_core::releases::candidatos;
use serde_json::{json, Value};
use tauri::{AppHandle, Emitter, Manager};

// Compartidos con updater.rs: la app y el firmware salen del mismo repositorio, y tener la
// cadena escrita en dos sitios es cómo se acaba consultando releases distintas.
pub const REPO: &str = "jaestefaniah27/scroll_wheel";
pub const AGENTE: &str = "OrbyGUI";

// Cómo se distinguen las releases de firmware de las de la app (`fw-v`) vive en
// `orby_core::releases::PREFIJO_ETIQUETA`, que es quien filtra por él. Aquí hubo una
// constante que lo repetía para que updater.rs pudiera descartar esas releases; ya no hace
// falta, porque el actualizador de la app no lee la lista de releases: pide el
// `latest.json` de la última directamente.

/// Con el comando BOOTSEL son dos o tres segundos; el margen largo es para el camino
/// manual, en el que hay que desenchufar el cable, buscar el botón y volver a enchufar.
const ESPERA_UNIDAD_MS: u64 = 90_000;
const RASTREO_UNIDAD_MS: u64 = 400;

struct Compartido {
    estado: Value,
    cancelado: bool,
}

impl Default for Compartido {
    fn default() -> Self {
        Self {
            estado: json!({
                "status": "idle",
                "current": null,
                "latest": null,
                "available": false,
                "percent": 0,
                "manual": false,
                "error": null,
            }),
            cancelado: false,
        }
    }
}

/// Monta el estado compartido. Se llama desde el `setup`, igual que `serial::arrancar`.
pub fn iniciar(app: &AppHandle) {
    app.manage(Mutex::new(Compartido::default()));
}

/// Fusiona `patch` sobre el estado (superficial, como el `Object.assign` de
/// electron/firmware.js) y avisa al renderer. Devuelve el estado ya fusionado.
fn set(app: &AppHandle, patch: Value) -> Value {
    let estado = app.state::<Mutex<Compartido>>();
    let mut c = estado.lock().unwrap();

    if let (Value::Object(actual), Value::Object(nuevo)) = (&mut c.estado, patch) {
        for (k, v) in nuevo {
            actual.insert(k, v);
        }
    }

    let clon = c.estado.clone();
    let _ = app.emit("firmware:state", clon.clone());
    clon
}

fn cancelado(app: &AppHandle) -> bool {
    app.state::<Mutex<Compartido>>().lock().unwrap().cancelado
}

fn poner_cancelado(app: &AppHandle, valor: bool) {
    app.state::<Mutex<Compartido>>().lock().unwrap().cancelado = valor;
}

// --- La unidad del cargador ---------------------------------------------------------

/// La letra la asigna Windows y cambia de un día para otro, así que se busca por lo que la
/// Pico deja en la raíz. La etiqueta del volumen (RPI-RP2) obligaría a otra vía; con
/// `INFO_UF2.TXT` basta un `Path::exists` y de paso confirma que es una RP2 y no cualquier
/// pendrive que se llame igual.
fn encontrar_unidad() -> Option<PathBuf> {
    for letra in b'C'..=b'Z' {
        let raiz = PathBuf::from(format!("{}:\\", letra as char));
        let info = raiz.join("INFO_UF2.TXT");
        if let Ok(contenido) = std::fs::read_to_string(&info) {
            if contenido.contains("RP2") {
                return Some(raiz);
            }
        }
    }
    None
}

fn esperar_unidad(app: &AppHandle) -> Result<PathBuf, String> {
    let limite = std::time::Instant::now() + Duration::from_millis(ESPERA_UNIDAD_MS);
    while std::time::Instant::now() < limite {
        if cancelado(app) {
            return Err("Actualización cancelada".into());
        }
        if let Some(unidad) = encontrar_unidad() {
            return Ok(unidad);
        }
        std::thread::sleep(Duration::from_millis(RASTREO_UNIDAD_MS));
    }
    Err("El teclado no ha aparecido en modo de actualización".into())
}

// --- Descarga y copia -----------------------------------------------------------------

/// Descarga el asset a un temporal, con progreso. Se comprueba el tamaño porque una
/// descarga cortada por la mitad sigue siendo un .uf2 válido a ojos del cargador: copiaría
/// medio firmware y dejaría el teclado sin arrancar.
fn descargar(app: &AppHandle, nombre: &str, url: &str, tamano_esperado: u64) -> Result<PathBuf, String> {
    set(app, json!({ "status": "downloading", "percent": 0, "error": null }));

    let cliente = reqwest::blocking::Client::new();
    let mut resp = cliente
        .get(url)
        .header("User-Agent", AGENTE)
        .send()
        .map_err(|e| format!("La descarga falló: {e}"))?;
    if !resp.status().is_success() {
        return Err(format!("La descarga falló ({})", resp.status()));
    }

    let total = resp.content_length().unwrap_or(tamano_esperado);
    let mut datos = Vec::with_capacity(total as usize);
    let mut buf = [0u8; 64 * 1024];
    let mut leidos: u64 = 0;

    loop {
        if cancelado(app) {
            return Err("Actualización cancelada".into());
        }
        let n = resp.read(&mut buf).map_err(|e| format!("La descarga falló: {e}"))?;
        if n == 0 {
            break;
        }
        datos.extend_from_slice(&buf[..n]);
        leidos += n as u64;
        if total > 0 {
            set(app, json!({ "percent": (leidos * 100 / total) as u64 }));
        }
    }

    if total > 0 && leidos != total {
        return Err(format!("La descarga llegó incompleta ({leidos} de {total} bytes)"));
    }

    let destino = std::env::temp_dir().join(nombre);
    std::fs::write(&destino, &datos).map_err(|e| format!("No se pudo guardar la descarga: {e}"))?;
    Ok(destino)
}

/// Copiar el .uf2 a la unidad del cargador. El fallo esperado no es que no copie: la Pico
/// se reinicia en cuanto recibe el último bloque, así que el cierre del fichero se
/// encuentra la unidad ya desaparecida y Windows devuelve EIO/EPERM/ENOENT/EBUSY/EINVAL.
/// Eso **es** el éxito. Tratarlo como fallo es el error clásico aquí.
fn flashear(app: &AppHandle, uf2: &PathBuf, unidad: &PathBuf) {
    set(app, json!({ "status": "flashing", "percent": 100 }));

    let destino = unidad.join(uf2.file_name().unwrap_or_default());
    if let Err(e) = std::fs::copy(uf2, &destino) {
        let toleradas = [
            std::io::ErrorKind::UnexpectedEof,
            std::io::ErrorKind::PermissionDenied,
            std::io::ErrorKind::NotFound,
            std::io::ErrorKind::WouldBlock,
            std::io::ErrorKind::InvalidInput,
        ];
        if !toleradas.contains(&e.kind()) {
            eprintln!("[firmware] fallo copiando a {}: {e}", destino.display());
        }
    }
}

// --- Comandos ---------------------------------------------------------------------------

#[tauri::command]
pub fn firmware_get(app: AppHandle) -> Value {
    app.state::<Mutex<Compartido>>().lock().unwrap().estado.clone()
}

/// Qué firmware hay publicado que esta app sepa usar. `maxFw` sale de `FW_RECOMMENDED` en
/// src/compat.js: publicar un firmware más nuevo que la app no debe ofrecerse aquí, porque
/// la app no sabría hablar con él.
#[tauri::command]
pub fn firmware_check(app: AppHandle, opts: Option<Value>) -> Value {
    let opts = opts.unwrap_or(Value::Null);
    let max_fw = opts.get("maxFw").and_then(Value::as_str).map(str::to_string);
    let current_fw = opts.get("currentFw").and_then(Value::as_str).map(str::to_string);
    let actual = current_fw.or_else(|| {
        app.state::<Mutex<Compartido>>()
            .lock()
            .unwrap()
            .estado
            .get("current")
            .and_then(Value::as_str)
            .map(str::to_string)
    });

    set(&app, json!({ "status": "checking", "error": null, "current": actual }));

    let url = format!("https://api.github.com/repos/{REPO}/releases?per_page=50");
    let resultado = reqwest::blocking::Client::new()
        .get(&url)
        .header("Accept", "application/vnd.github+json")
        .header("User-Agent", AGENTE)
        .send()
        .and_then(|r| r.error_for_status())
        .map_err(|e| format!("No se pudo consultar las versiones: {e}"))
        .and_then(|r| r.text().map_err(|e| format!("No se pudo consultar las versiones: {e}")));

    let releases = match resultado.and_then(|t| {
        serde_json::from_str::<Value>(&t).map_err(|e| format!("No se pudo consultar las versiones: {e}"))
    }) {
        Ok(v) => v,
        Err(mensaje) => return set(&app, json!({ "status": "error", "error": mensaje })),
    };

    let lista = candidatos(&releases, max_fw.as_deref());
    let ultima = lista.first().map(|c| {
        json!({
            "version": c.version,
            "asset": { "name": c.nombre_asset, "browser_download_url": c.url, "size": c.tamano },
        })
    });

    let disponible = match (&lista.first(), &actual) {
        (Some(u), Some(a)) => orby_core::fw::compare_fw(&u.version, a) == std::cmp::Ordering::Greater,
        _ => false,
    };

    set(
        &app,
        json!({ "status": "idle", "latest": ultima, "available": disponible }),
    )
}

/// El proceso entero. `viaBootsel` dice si el teclado sabe reiniciarse solo (bandera
/// BOOTSEL=1 del handshake); si no, se le pide al usuario que lo haga a mano y se espera
/// igual.
#[tauri::command]
pub fn firmware_update(app: AppHandle, opts: Option<Value>) -> Value {
    {
        let estado = app.state::<Mutex<Compartido>>();
        let c = estado.lock().unwrap();
        let en_marcha = c
            .estado
            .get("status")
            .and_then(Value::as_str)
            .is_some_and(|s| ["downloading", "bootsel", "flashing"].contains(&s));
        if en_marcha {
            return c.estado.clone();
        }
    }
    poner_cancelado(&app, false);

    let via_bootsel = opts
        .as_ref()
        .and_then(|o| o.get("viaBootsel"))
        .and_then(Value::as_bool)
        .unwrap_or(false);

    let latest = app.state::<Mutex<Compartido>>().lock().unwrap().estado.get("latest").cloned();
    let Some(asset) = latest.as_ref().and_then(|l| l.get("asset")) else {
        return set(&app, json!({ "status": "error", "error": "No hay ninguna versión que instalar" }));
    };
    let nombre = asset.get("name").and_then(Value::as_str).unwrap_or("firmware.uf2").to_string();
    let url = asset.get("browser_download_url").and_then(Value::as_str).unwrap_or("").to_string();
    let tamano = asset.get("size").and_then(Value::as_u64).unwrap_or(0);

    let intento = (|| -> Result<(), String> {
        let uf2 = descargar(&app, &nombre, &url, tamano)?;

        // Primero se mira si ya está en modo cargador: puede que el usuario lo haya dejado
        // así, y entonces no hay ni teclado al que pedirle nada.
        let unidad = match encontrar_unidad() {
            Some(u) => u,
            None => {
                set(&app, json!({ "status": "bootsel", "manual": !via_bootsel }));
                // El comando no contesta nada más: el teclado se reinicia acto seguido y el
                // puerto desaparece. La confirmación de que ha funcionado es que aparezca
                // la unidad, no una respuesta por el CDC.
                if via_bootsel {
                    let serie = app.state::<crate::serial::Serie>();
                    crate::serial::serial_send(serie, "BOOTSEL".to_string());
                }
                esperar_unidad(&app)?
            }
        };

        flashear(&app, &uf2, &unidad);

        // El teclado tarda un par de segundos en volver a enumerar. El puerto lo reencuentra
        // el hilo de serial.rs por su cuenta; aquí solo se deja de decir «flasheando»
        // cuando ya no está la unidad del cargador.
        for _ in 0..20 {
            if encontrar_unidad().is_none() {
                break;
            }
            std::thread::sleep(Duration::from_millis(500));
        }
        Ok(())
    })();

    match intento {
        Ok(()) => set(&app, json!({ "status": "done", "percent": 100, "available": false, "manual": false })),
        Err(mensaje) => set(&app, json!({ "status": "error", "error": mensaje, "manual": false })),
    }
}

#[tauri::command]
pub fn firmware_cancel(app: AppHandle) -> Value {
    poner_cancelado(&app, true);

    let estado = app.state::<Mutex<Compartido>>();
    let en_espera = estado
        .lock()
        .unwrap()
        .estado
        .get("status")
        .and_then(Value::as_str)
        .is_some_and(|s| ["downloading", "bootsel"].contains(&s));

    if en_espera {
        set(&app, json!({ "status": "idle", "percent": 0, "manual": false }))
    } else {
        estado.lock().unwrap().estado.clone()
    }
}
