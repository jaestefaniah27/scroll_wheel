// Registro en disco de lo que le pasa a la conexión con el teclado.
//
// Es el equivalente de `electron/log.js`, y escribe en el **mismo** fichero:
// `%APPDATA%\OrbyGUI\orby.log`. En desarrollo basta con el terminal, pero la app
// instalada no tiene consola —`windows_subsystem = "windows"` en `main.rs`—, así que sin
// esto un `eprintln!` no se puede leer en ningún sitio. Cuando alguien dice «se me
// desconecta solo», este fichero es la única forma de saber si lo soltó una escritura,
// una lectura o el vigilante.
//
// Solo el ciclo de vida de la conexión: ni la telemetría ni los comandos, que serían
// megas al día y aquí no aportan.

use std::io::Write;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

/// Se parte en dos ficheros de este tamaño: el actual y el anterior. Con eso sobra para
/// ver qué pasó en el último arranque sin dejar el disco lleno.
const MAX_BYTES: u64 = 256 * 1024;

/// El hilo del puerto no es el único que registra (el del firmware también), y dos
/// `append` a la vez pueden entrelazar media línea con media línea.
fn cerrojo() -> &'static Mutex<()> {
    static CERROJO: OnceLock<Mutex<()>> = OnceLock::new();
    CERROJO.get_or_init(|| Mutex::new(()))
}

/// Junto a `orby-config.json`, y por el mismo motivo que aquel no sale de
/// `app_config_dir()`: el identificador de la app es `com.orby.gui` y dejaría el rastro
/// en `%APPDATA%\com.orby.gui`, que no es donde nadie lo va a buscar.
fn ruta() -> Option<PathBuf> {
    let appdata = std::env::var_os("APPDATA")?;
    Some(PathBuf::from(appdata).join("OrbyGUI").join("orby.log"))
}

fn rotar(p: &PathBuf) {
    let Ok(meta) = std::fs::metadata(p) else { return };
    if meta.len() < MAX_BYTES {
        return;
    }
    let anterior = p.with_extension("log.1");
    let _ = std::fs::remove_file(&anterior);
    let _ = std::fs::rename(p, &anterior);
}

/// Escribe una línea con su marca de tiempo. **Nunca falla hacia fuera**: si no se puede
/// registrar, eso no puede romper la conexión que se estaba registrando.
pub fn escribir(linea: &str) {
    // En desarrollo se mira el terminal, así que sale por los dos sitios.
    eprintln!("{linea}");

    let Some(p) = ruta() else { return };
    let _guarda = cerrojo().lock().unwrap_or_else(|e| e.into_inner());

    if let Some(dir) = p.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    rotar(&p);

    let marca = chrono::Local::now().format("%Y-%m-%dT%H:%M:%S%.3f");
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(&p) {
        let _ = writeln!(f, "{marca} {linea}");
    }
}

/// `log!("[serie] ...")`, para no tener que montar el `format!` en cada llamada.
macro_rules! log {
    ($($arg:tt)*) => { crate::log::escribir(&format!($($arg)*)) };
}

pub(crate) use log;
