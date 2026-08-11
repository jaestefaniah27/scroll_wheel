// Las aplicaciones instaladas, sacadas de los dos menús de Inicio.
//
// Sirve para que asignar «abrir un programa» a una tecla no obligue al usuario a ir a
// buscar el `.exe` por el disco. Es exactamente lo que hace electron/apps.js, con el
// mismo filtrado y el mismo orden, para que la lista salga igual en las dos apps.

use std::cmp::Ordering;
use std::collections::HashSet;
use std::os::windows::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

use serde::Serialize;
use windows::core::{Interface, PCWSTR};
use windows::Win32::Globalization::{CompareStringEx, COMPARE_STRING_FLAGS};
use windows::Win32::Storage::FileSystem::WIN32_FIND_DATAW;
use windows::Win32::System::Com::{
    CoCreateInstance, CoInitializeEx, CoUninitialize, IPersistFile, CLSCTX_INPROC_SERVER,
    COINIT_APARTMENTTHREADED, STGM_READ,
};
use windows::Win32::UI::Shell::{IShellLinkW, ShellLink};

#[derive(Clone, Serialize)]
pub struct Aplicacion {
    pub name: String,
    pub target: String,
}

/// Accesos directos que están en el menú de Inicio pero que nadie quiere asignar a una
/// tecla. Es la traducción de la expresión regular de `electron/apps.js`; se pone a mano
/// en vez de traerse el crate `regex` porque son once comparaciones literales.
/// `read ?me` se cubre con las dos formas y `licen[cs]` con las dos raíces.
const RUIDO: &[&str] = &[
    "uninstall", "desinstal", "readme", "read me", "léeme", "help", "ayuda", "website",
    "sitio web", "support", "licenc", "licens",
];

fn es_ruido(nombre: &str) -> bool {
    let n = nombre.to_lowercase();
    RUIDO.iter().any(|p| n.contains(p))
}

fn carpetas_de_inicio() -> Vec<PathBuf> {
    ["ProgramData", "APPDATA"]
        .iter()
        .filter_map(|var| std::env::var_os(var))
        .map(|base| {
            PathBuf::from(base)
                .join("Microsoft")
                .join("Windows")
                .join("Start Menu")
                .join("Programs")
        })
        .collect()
}

fn recorrer(carpeta: &Path, encontrados: &mut Vec<PathBuf>) {
    let Ok(entradas) = std::fs::read_dir(carpeta) else { return };
    for entrada in entradas.flatten() {
        let ruta = entrada.path();
        match entrada.file_type() {
            Ok(t) if t.is_dir() => recorrer(&ruta, encontrados),
            Ok(_) => {
                if ruta
                    .extension()
                    .is_some_and(|e| e.eq_ignore_ascii_case("lnk"))
                {
                    encontrados.push(ruta);
                }
            }
            Err(_) => {}
        }
    }
}

/// A dónde apunta un acceso directo. `None` si está roto o no se puede leer, que es lo
/// que pasa con unos cuantos de cualquier instalación de Windows.
unsafe fn destino(lnk: &Path) -> Option<String> {
    let enlace: IShellLinkW = CoCreateInstance(&ShellLink, None, CLSCTX_INPROC_SERVER).ok()?;
    let fichero: IPersistFile = enlace.cast().ok()?;

    let ruta: Vec<u16> = lnk.as_os_str().encode_wide().chain(std::iter::once(0)).collect();
    fichero.Load(PCWSTR(ruta.as_ptr()), STGM_READ).ok()?;

    // No se llama a `Resolve`: cuando el destino ya no existe se pone a buscarlo por el
    // disco y por la red, y una lista de doscientos accesos directos tardaría minutos.
    let mut buffer = [0u16; 260];
    let mut datos = WIN32_FIND_DATAW::default();
    enlace.GetPath(&mut buffer, &mut datos, 0).ok()?;

    let fin = buffer.iter().position(|&c| c == 0).unwrap_or(buffer.len());
    if fin == 0 {
        return None;
    }
    Some(String::from_utf16_lossy(&buffer[..fin]))
}

/// Ordena como `localeCompare(a, b, 'es')`: se lo pregunta a Windows en vez de comparar
/// bytes, que dejaría «Ñ» y los acentos donde no toca.
fn comparar_en_espanol(a: &str, b: &str) -> Ordering {
    const CSTR_LESS_THAN: i32 = 1;
    const CSTR_GREATER_THAN: i32 = 3;

    let locale: Vec<u16> = "es-ES\0".encode_utf16().collect();
    let ua: Vec<u16> = a.encode_utf16().collect();
    let ub: Vec<u16> = b.encode_utf16().collect();

    let r = unsafe {
        CompareStringEx(
            PCWSTR(locale.as_ptr()),
            // Sin banderas: es lo que hace `localeCompare` sin opciones, que distingue
            // mayúsculas y acentos en vez de amontonarlos.
            COMPARE_STRING_FLAGS(0),
            &ua,
            &ub,
            None,
            None,
            None,
        )
    };

    match r.0 {
        CSTR_LESS_THAN => Ordering::Less,
        CSTR_GREATER_THAN => Ordering::Greater,
        _ => Ordering::Equal,
    }
}

fn cache() -> &'static Mutex<Option<Vec<Aplicacion>>> {
    static CACHE: OnceLock<Mutex<Option<Vec<Aplicacion>>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(None))
}

fn escanear() -> Vec<Aplicacion> {
    // COM es por hilo, y este comando puede caer en cualquiera del grupo de Tauri. Si ya
    // estaba inicializado en otro modo devuelve error y da igual: lo que hace falta es
    // que el hilo pueda usar COM, no haberlo inicializado nosotros.
    let inicializado = unsafe { CoInitializeEx(None, COINIT_APARTMENTTHREADED) }.is_ok();

    let mut accesos = Vec::new();
    for carpeta in carpetas_de_inicio() {
        recorrer(&carpeta, &mut accesos);
    }

    let mut vistos: HashSet<String> = HashSet::new();
    let mut apps = Vec::new();

    for lnk in accesos {
        let Some(nombre) = lnk.file_stem().map(|s| s.to_string_lossy().into_owned()) else {
            continue;
        };
        if es_ruido(&nombre) {
            continue;
        }

        let Some(target) = (unsafe { destino(&lnk) }) else { continue };
        // Solo programas: un acceso directo a un PDF o a una carpeta no se puede «abrir»
        // como aplicación desde una tecla.
        if !target.to_lowercase().ends_with(".exe") {
            continue;
        }
        // Muchos programas dejan el mismo `.exe` en varios sitios del menú de Inicio.
        if !vistos.insert(target.to_lowercase()) {
            continue;
        }

        apps.push(Aplicacion { name: nombre, target });
    }

    if inicializado {
        unsafe { CoUninitialize() };
    }

    apps.sort_by(|a, b| comparar_en_espanol(&a.name, &b.name));
    apps
}

#[tauri::command]
pub fn apps_list_installed() -> Vec<Aplicacion> {
    // Recorrer los dos menús de Inicio y abrir cada `.lnk` por COM cuesta casi un segundo.
    // Se hace una vez por sesión, igual que en Electron: la lista de programas instalados
    // no cambia mientras la app está abierta.
    let mut guarda = cache().lock().unwrap_or_else(|e| e.into_inner());
    guarda.get_or_insert_with(escanear).clone()
}

#[cfg(test)]
mod tests {
    use super::es_ruido;

    #[test]
    fn descarta_los_accesos_que_no_son_programas() {
        for nombre in [
            "Uninstall Foo", "Desinstalar Foo", "ReadMe", "Read Me First", "Léeme",
            "Help", "Ayuda de Foo", "Foo Website", "Sitio web de Foo", "Support",
            "Licencia", "License",
        ] {
            assert!(es_ruido(nombre), "«{nombre}» debería descartarse");
        }
    }

    #[test]
    fn no_se_lleva_por_delante_programas_de_verdad() {
        for nombre in ["Blender", "KiCad", "Altium Designer", "Visual Studio Code"] {
            assert!(!es_ruido(nombre), "«{nombre}» no se puede descartar");
        }
    }
}
