// Los ajustes que solo viven en el PC: el firmware no sabe nada de aplicaciones de
// Windows ni de secuencias que ejecuta el ordenador.
//
// Aquí solo está el fichero. La fusión y los valores por defecto son de
// `orby_core::config`, que es donde se pueden probar.

use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

use orby_core::config::{defaults, merge};
use serde_json::Value;

/// `config_set` es leer-fusionar-escribir, y los comandos de Tauri pueden correr a la vez
/// en hilos distintos. Sin esto, dos guardados casi simultáneos —el de una vista y el del
/// autoguardado— se pisan: el segundo lee de disco antes de que el primero haya escrito y
/// deja fuera su cambio. En Electron no podía pasar porque todo iba en un hilo.
fn cerrojo() -> &'static Mutex<()> {
    static CERROJO: OnceLock<Mutex<()>> = OnceLock::new();
    CERROJO.get_or_init(|| Mutex::new(()))
}

/// `%APPDATA%\OrbyGUI\orby-config.json`, el **mismo** fichero que usa la app de Electron.
/// Es lo que permite tener las dos instaladas durante la migración sin que el usuario
/// pierda nada al pasar de una a otra.
///
/// No sale de `app_config_dir()` a propósito: ese resuelve a `%APPDATA%\<identifier>`, y
/// el identificador de la app es `com.orby.gui` —reverse-DNS, como piden los
/// empaquetadores y como necesita el instalador de la Tarea 12—, así que dejaría la
/// configuración en `%APPDATA%\com.orby.gui`. El usuario abriría la app nueva y se
/// encontraría sin ninguna de sus secuencias.
fn ruta() -> Option<PathBuf> {
    let appdata = std::env::var_os("APPDATA")?;
    Some(PathBuf::from(appdata).join("OrbyGUI").join("orby-config.json"))
}

/// Lee la configuración del disco. **Nunca falla**: un fichero corrupto no puede impedir
/// que la app abra, y devolver los valores por defecto deja al usuario reconstruirla en
/// vez de encontrarse una ventana muerta.
///
/// Lo leído se fusiona *sobre* los valores por defecto para que una clave añadida en una
/// versión nueva exista aunque el fichero del usuario sea de una anterior.
pub fn leer() -> Value {
    let Some(p) = ruta() else {
        eprintln!("[config] sin APPDATA: se usan los valores por defecto");
        return defaults();
    };

    let texto = match std::fs::read_to_string(&p) {
        Ok(t) => t,
        // Que no exista es lo normal la primera vez, no un error que contar.
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return defaults(),
        Err(e) => {
            eprintln!("[config] no se pudo leer {}: {e}", p.display());
            return defaults();
        }
    };

    match serde_json::from_str::<Value>(&texto) {
        Ok(v) => merge(&defaults(), &v),
        Err(e) => {
            eprintln!("[config] {} está corrupto ({e}): se usan los valores por defecto", p.display());
            defaults()
        }
    }
}

/// Escribe la configuración entera.
///
/// Va por fichero temporal y `rename` porque el original es lo único que hay: si la app
/// muere a mitad de la escritura, un `orby-config.json` truncado se lleva por delante
/// todas las secuencias del usuario. Con el temporal, o está el de antes o está el nuevo.
fn escribir(valor: &Value) -> std::io::Result<()> {
    let Some(p) = ruta() else {
        return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "sin APPDATA"));
    };
    if let Some(carpeta) = p.parent() {
        std::fs::create_dir_all(carpeta)?;
    }

    // Con sangría: estos ficheros se acaban abriendo a mano para ver qué ha pasado.
    let texto = serde_json::to_string_pretty(valor)?;
    let temporal = p.with_extension("json.tmp");
    std::fs::write(&temporal, texto)?;
    std::fs::rename(&temporal, &p)
}

#[tauri::command]
pub fn config_get() -> Value {
    leer()
}

/// Fusiona el parche y devuelve **la configuración ya fusionada**: el renderer se queda
/// con lo devuelto, así que si aquí se devolviera el parche a secas perdería todo lo que
/// no venga mencionado en él.
#[tauri::command]
pub fn config_set(patch: Value) -> Value {
    // Un envenenamiento del cerrojo no es motivo para dejar de guardar: lo único que
    // protege es la secuencia leer-fusionar-escribir, no un dato a medio construir.
    let _guarda = cerrojo().lock().unwrap_or_else(|e| e.into_inner());

    let fusionada = merge(&leer(), &patch);
    if let Err(e) = escribir(&fusionada) {
        // Se devuelve igualmente lo fusionado: la app sigue funcionando en esta sesión
        // aunque el disco esté lleno o el fichero esté en solo lectura.
        eprintln!("[config] no se pudo guardar: {e}");
    }
    fusionada
}
