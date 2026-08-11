// Arrancar con la sesión de Windows.
//
// El estado real lo tiene Windows, en la clave `Run` del usuario. No se guarda además en
// config.json a propósito: dos copias de la misma verdad es como se acaba con un
// interruptor que dice una cosa y un arranque que hace otra.
//
// La entrada se escribe con `--hidden` para que al iniciar sesión la app se quede en la
// bandeja en vez de plantar la ventana delante (ver `main.rs`).
//
// Al LEER, lo que decide si está activado es la **ruta**, no los argumentos. Una entrada
// que apunte a este ejecutable arranca la app la lleve o no: contestar «desactivado»
// porque le falta `--hidden` sería mentir igual que contestar «activado» cuando no hay
// entrada. Lo que se hace con esas entradas a medias es repararlas —`normalizar()`, desde
// el arranque—, no negarlas. Y la ruta se compara resuelta, porque el mismo ejecutable
// escrito de dos formas (mayúsculas distintas, un enlace, nombres 8.3) es el mismo
// fichero y compararlo como texto crudo devolvería que el autoarranque está apagado.

use std::path::{Path, PathBuf};

use windows::core::PCWSTR;
use windows::Win32::Foundation::{ERROR_FILE_NOT_FOUND, ERROR_SUCCESS};
use windows::Win32::System::Registry::{
    RegCloseKey, RegDeleteValueW, RegOpenKeyExW, RegQueryValueExW, RegSetValueExW, HKEY,
    HKEY_CURRENT_USER, KEY_READ, KEY_WRITE, REG_SZ, REG_VALUE_TYPE,
};

const CLAVE: &str = r"Software\Microsoft\Windows\CurrentVersion\Run";
const VALOR: &str = "OrbyGUI";

/// El argumento con el que la app arranca escondida. Lo lee `main.rs`.
pub const ARG_ESCONDIDO: &str = "--hidden";

fn ancha(texto: &str) -> Vec<u16> {
    texto.encode_utf16().chain(std::iter::once(0)).collect()
}

/// Cierra el handle pase lo que pase por el camino: los `?` y los retornos tempranos de
/// abajo dejarían la clave abierta si el cierre fuera una línea más al final.
struct Clave(HKEY);

impl Drop for Clave {
    fn drop(&mut self) {
        unsafe {
            let _ = RegCloseKey(self.0);
        }
    }
}

/// La clave `Run` del usuario existe siempre en un Windows con sesión iniciada, así que se
/// abre en vez de crearse: si esto falla, el problema no es que falte.
fn abrir_run(escritura: bool) -> Option<Clave> {
    let sub = ancha(CLAVE);
    let permisos = if escritura { KEY_READ | KEY_WRITE } else { KEY_READ };
    let mut handle = HKEY::default();

    let r = unsafe {
        RegOpenKeyExW(
            HKEY_CURRENT_USER,
            PCWSTR(sub.as_ptr()),
            None,
            permisos,
            &mut handle,
        )
    };

    if r == ERROR_SUCCESS {
        Some(Clave(handle))
    } else {
        eprintln!("[autoarranque] no se pudo abrir HKCU\\{CLAVE}: error {}", r.0);
        None
    }
}

/// La línea de comandos guardada, o `None` si no hay entrada. Que no la haya es el caso
/// normal (autoarranque desactivado), no un fallo: por eso no se traza.
fn leer() -> Option<String> {
    let clave = abrir_run(false)?;
    let nombre = ancha(VALOR);

    // Dos llamadas: la primera solo para el tamaño. El valor lo puede haber escrito
    // cualquiera y no hay longitud máxima que se pueda suponer sin arriesgarse a cortarlo.
    let mut tipo = REG_VALUE_TYPE::default();
    let mut bytes: u32 = 0;
    let r = unsafe {
        RegQueryValueExW(
            clave.0,
            PCWSTR(nombre.as_ptr()),
            None,
            Some(&mut tipo),
            None,
            Some(&mut bytes),
        )
    };
    if r != ERROR_SUCCESS || tipo != REG_SZ || bytes == 0 {
        return None;
    }

    let mut buf = vec![0u8; bytes as usize];
    let mut cabe = bytes;
    let r = unsafe {
        RegQueryValueExW(
            clave.0,
            PCWSTR(nombre.as_ptr()),
            None,
            None,
            Some(buf.as_mut_ptr()),
            Some(&mut cabe),
        )
    };
    if r != ERROR_SUCCESS {
        eprintln!("[autoarranque] no se pudo leer el valor «{VALOR}»: error {}", r.0);
        return None;
    }

    // REG_SZ es UTF-16, y el tamaño viene en bytes. El nulo final puede venir incluido o
    // no según quién lo escribiera, así que se recorta en vez de darlo por hecho.
    let unidades: Vec<u16> = buf[..cabe as usize]
        .chunks_exact(2)
        .map(|par| u16::from_le_bytes([par[0], par[1]]))
        .collect();

    Some(
        String::from_utf16_lossy(&unidades)
            .trim_end_matches('\0')
            .to_string(),
    )
}

/// La ruta del ejecutable dentro de una línea de comandos. Aquí siempre se escribe
/// entrecomillada, pero una entrada vieja o puesta a mano puede no llevar comillas.
fn ruta_de(linea: &str) -> &str {
    let linea = linea.trim();
    match linea.strip_prefix('"') {
        Some(resto) => resto.split('"').next().unwrap_or(""),
        None => linea.split(' ').next().unwrap_or(""),
    }
}

fn ejecutable() -> PathBuf {
    std::env::current_exe().unwrap_or_default()
}

/// Dos rutas apuntan al mismo fichero. `canonicalize` resuelve enlaces, nombres cortos y
/// mayúsculas; si falla (la ruta guardada puede apuntar a una instalación que ya no
/// existe) se cae a comparar el texto sin distinguir mayúsculas, que es lo que hace
/// Windows con las rutas.
fn mismo_fichero(a: &str, b: &Path) -> bool {
    if a.is_empty() {
        return false;
    }
    match (std::fs::canonicalize(a), std::fs::canonicalize(b)) {
        (Ok(x), Ok(y)) => x == y,
        _ => a.eq_ignore_ascii_case(&b.to_string_lossy()),
    }
}

fn lleva_escondido(linea: &str) -> bool {
    linea.split_whitespace().any(|arg| arg == ARG_ESCONDIDO)
}

fn activo() -> bool {
    leer().is_some_and(|linea| mismo_fichero(ruta_de(&linea), &ejecutable()))
}

fn escribir() {
    let Some(clave) = abrir_run(true) else { return };

    // Entrecomillada siempre: «C:\Program Files\...» sin comillas la parte Windows por el
    // espacio e intenta arrancar «C:\Program».
    let linea = format!("\"{}\" {ARG_ESCONDIDO}", ejecutable().display());
    let datos = ancha(&linea);
    let nombre = ancha(VALOR);

    // RegSetValueExW quiere bytes, y el tamaño incluye el nulo final que ya trae `ancha`.
    let crudos =
        unsafe { std::slice::from_raw_parts(datos.as_ptr() as *const u8, datos.len() * 2) };

    let r = unsafe {
        RegSetValueExW(
            clave.0,
            PCWSTR(nombre.as_ptr()),
            None,
            REG_SZ,
            Some(crudos),
        )
    };
    if r != ERROR_SUCCESS {
        eprintln!("[autoarranque] no se pudo escribir «{VALOR}»: error {}", r.0);
    }
}

fn borrar() {
    let Some(clave) = abrir_run(true) else { return };
    let nombre = ancha(VALOR);

    let r = unsafe { RegDeleteValueW(clave.0, PCWSTR(nombre.as_ptr())) };
    // Borrar lo que no está no es un fallo: es el estado que se pedía.
    if r != ERROR_SUCCESS && r != ERROR_FILE_NOT_FOUND {
        eprintln!("[autoarranque] no se pudo borrar «{VALOR}»: error {}", r.0);
    }
}

/// Repara entradas anteriores a que existiera `--hidden`: sin el argumento, iniciar sesión
/// abriría la ventana en vez de dejar la app en la bandeja. Se llama desde el `setup`.
pub fn normalizar() {
    let Some(linea) = leer() else { return };
    if !mismo_fichero(ruta_de(&linea), &ejecutable()) || lleva_escondido(&linea) {
        return;
    }
    escribir();
}

// --- Comandos ---------------------------------------------------------------------------

#[tauri::command]
pub fn autostart_get() -> bool {
    activo()
}

#[tauri::command]
pub fn autostart_set(enabled: bool) -> bool {
    if enabled {
        escribir();
    } else {
        borrar();
    }
    // Se contesta con lo que ha quedado en el registro, no con lo que se pidió: si la
    // escritura falló, el interruptor de Ajustes tiene que volverse solo en vez de quedarse
    // encendido sobre un autoarranque que no existe.
    activo()
}

#[cfg(test)]
mod tests {
    use super::{lleva_escondido, ruta_de};

    #[test]
    fn saca_la_ruta_con_y_sin_comillas() {
        assert_eq!(
            ruta_de(r#""C:\Program Files\OrbyGUI\OrbyGUI.exe" --hidden"#),
            r"C:\Program Files\OrbyGUI\OrbyGUI.exe"
        );
        // Sin comillas y sin argumentos, que es como quedaría una entrada puesta a mano.
        assert_eq!(ruta_de(r"C:\Orby\OrbyGUI.exe"), r"C:\Orby\OrbyGUI.exe");
    }

    #[test]
    fn reconoce_el_argumento_de_escondido() {
        assert!(lleva_escondido(r#""C:\Orby\OrbyGUI.exe" --hidden"#));
        assert!(!lleva_escondido(r#""C:\Orby\OrbyGUI.exe""#));
        // No vale que sea el prefijo de otro argumento.
        assert!(!lleva_escondido(r#""C:\Orby\OrbyGUI.exe" --hidden-cosas"#));
    }
}
