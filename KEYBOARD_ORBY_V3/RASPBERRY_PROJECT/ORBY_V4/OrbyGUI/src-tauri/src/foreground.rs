// La ventana que tiene el foco, que es lo que dispara el cambio automático de perfil
// (`src/views/auto.js`) y las variaciones por aplicación (`src/variants.js`).
//
// En Electron esto era un PowerShell de vida larga: un script metido en base64 que hacía
// P/Invoke contra user32.dll, escupía una línea JSON por cada cambio y había que trocear,
// interpretar y matar a mano. Aquí son cuatro llamadas a Win32 desde un hilo propio, así
// que desaparece el proceso hijo y con él la única forma que había de dejar un
// `powershell.exe` suelto si la app moría mal.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use serde_json::{json, Value};
use tauri::{AppHandle, Emitter};

use windows::core::PWSTR;
use windows::Win32::Foundation::{CloseHandle, HWND};
use windows::Win32::System::Threading::{
    OpenProcess, QueryFullProcessImageNameW, PROCESS_NAME_WIN32,
    PROCESS_QUERY_LIMITED_INFORMATION,
};
use windows::Win32::UI::WindowsAndMessaging::{
    GetForegroundWindow, GetWindowTextW, GetWindowThreadProcessId,
};

/// El mismo ritmo que llevaba el `Start-Sleep -Milliseconds 400` del script de Electron.
/// Bajarlo no gana nada: lo que se vigila es que el usuario cambie de programa.
const SONDEO_MS: u64 = 400;

/// La bandera de vida del hilo. Que sea un `Arc` y no un atómico global es lo que permite
/// que un `parar()` seguido de un `arrancar()` no deje al hilo viejo y al nuevo mirando la
/// misma bandera: el viejo se queda con la suya, ya apagada, y se muere solo.
static VIGILANTE: Mutex<Option<Arc<AtomicBool>>> = Mutex::new(None);

/// La última ventana vista. Sobrevive a `parar()`, igual que el `this.current` de Electron:
/// el editor de secuencias pregunta por ella sin saber si el vigilante está en marcha.
static ACTUAL: Mutex<Option<Value>> = Mutex::new(None);

/// Arranca el vigilante si no lo estaba. Devuelve si queda uno en marcha.
pub fn arrancar(app: AppHandle) -> bool {
    let Ok(mut guarda) = VIGILANTE.lock() else {
        return false;
    };

    // Arrancar dos veces no duplica el vigilante. Hace falta porque `getCurrentWindowInfo`
    // (`src/views/profiles/util.js`) llama a `start()` cada vez que necesita saber qué hay
    // delante, sin comprobar antes si ya estaba.
    if guarda.is_some() {
        return true;
    }

    let vivo = Arc::new(AtomicBool::new(true));
    *guarda = Some(vivo.clone());
    drop(guarda);

    thread::spawn(move || vigilar(app, vivo));
    true
}

pub fn parar() {
    if let Ok(mut guarda) = VIGILANTE.lock() {
        if let Some(vivo) = guarda.take() {
            vivo.store(false, Ordering::SeqCst);
        }
    }
}

fn vigilar(app: AppHandle, vivo: Arc<AtomicBool>) {
    // El estado de comparación es del hilo y no global: al parar y volver a arrancar se
    // vuelve a emitir la ventana que haya delante. Es de lo que depende el botón «App en
    // foco» del editor, que arranca el vigilante y pregunta por la ventana a los 300 ms.
    let mut ultima: Option<(isize, String)> = None;

    while vivo.load(Ordering::SeqCst) {
        if let Some((ventana, titulo)) = ventana_activa() {
            let id = ventana.0 as isize;
            // Se compara por ventana **y** por título, como el script de Electron: dentro
            // de un navegador o de un editor el programa no cambia, cambia el título, y es
            // ahí donde encajan la mitad de las reglas del usuario.
            let cambio = ultima
                .as_ref()
                .map_or(true, |(v, t)| *v != id || *t != titulo);

            if cambio {
                ultima = Some((id, titulo.clone()));
                let (proceso, ruta) = proceso_de(ventana);
                let info = json!({ "process": proceso, "title": titulo, "path": ruta });

                if let Ok(mut guarda) = ACTUAL.lock() {
                    *guarda = Some(info.clone());
                }
                // Se vuelve a mirar la bandera: entre el sondeo y aquí puede haber entrado
                // un `parar()`, y un cambio emitido por un vigilante ya apagado le movería
                // el perfil al teclado sin que nadie lo haya pedido.
                if vivo.load(Ordering::SeqCst) {
                    let _ = app.emit("foreground:change", info);
                }
            }
        }

        thread::sleep(Duration::from_millis(SONDEO_MS));
    }
}

fn ventana_activa() -> Option<(HWND, String)> {
    let ventana = unsafe { GetForegroundWindow() };
    // Sin ventana con el foco (pantalla de bloqueo, cambio de escritorio) no se emite
    // nada. Emitir un hueco haría saltar el perfil de reserva como si el usuario hubiera
    // cambiado de programa, y al desbloquear se encontraría otro perfil.
    if ventana.0.is_null() {
        return None;
    }

    // 512 caracteres, los mismos que el `StringBuilder` de Electron. Un título más largo
    // se recorta, que es lo que ya pasaba: las reglas casan por `includes`, no por
    // igualdad.
    let mut buffer = [0u16; 512];
    let largo = unsafe { GetWindowTextW(ventana, &mut buffer) };
    let titulo = String::from_utf16_lossy(&buffer[..largo.max(0) as usize]);
    Some((ventana, titulo))
}

/// Devuelve `(process, path)` de la ventana. Ante cualquier fallo devuelve cadenas vacías,
/// nunca un error: Electron dejaba los dos campos vacíos cuando `Get-Process` no podía con
/// el proceso, y `auto.js` ya está escrito contra eso (`info.process || ''`).
fn proceso_de(ventana: HWND) -> (String, String) {
    let mut pid = 0u32;
    unsafe { GetWindowThreadProcessId(ventana, Some(&mut pid as *mut u32)) };
    if pid == 0 {
        return (String::new(), String::new());
    }

    // `PROCESS_QUERY_LIMITED_INFORMATION` y no `PROCESS_QUERY_INFORMATION`: es el permiso
    // que dejan pedir los procesos de más integridad, así que con él la ruta también sale
    // para lo que corra como administrador. Con el otro, cualquier programa elevado
    // aparecería sin nombre y sus reglas no encajarían nunca.
    let Ok(proceso) = (unsafe { OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, false, pid) })
    else {
        return (String::new(), String::new());
    };

    // Holgado a propósito: hay rutas que pasan de `MAX_PATH`, y una ruta recortada abriría
    // el programa equivocado desde la pestaña «App» de una tecla.
    let mut buffer = [0u16; 1024];
    let mut largo = buffer.len() as u32;
    let resultado = unsafe {
        QueryFullProcessImageNameW(
            proceso,
            PROCESS_NAME_WIN32,
            PWSTR(buffer.as_mut_ptr()),
            &mut largo,
        )
    };
    let _ = unsafe { CloseHandle(proceso) };

    if resultado.is_err() {
        return (String::new(), String::new());
    }

    let ruta = String::from_utf16_lossy(&buffer[..largo as usize]);
    // `process` es el nombre del ejecutable **sin** extensión, que es lo que daba
    // `Get-Process.ProcessName`. Las reglas guardadas dicen «code», no «code.exe»: añadir
    // la extensión dejaría de casar todas las que el usuario ya tiene.
    let nombre = std::path::Path::new(&ruta)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();

    (nombre, ruta)
}

// --- Comandos ---

#[tauri::command]
pub fn foreground_start(app: AppHandle) -> bool {
    arrancar(app)
}

/// Devuelve `true` aunque no hubiera nada que parar, igual que el canal de Electron: el
/// renderer lo usa como acuse de recibo, no como resultado.
#[tauri::command]
pub fn foreground_stop() -> bool {
    parar();
    true
}

#[tauri::command]
pub fn foreground_current() -> Option<Value> {
    ACTUAL.lock().ok().and_then(|guarda| guarda.clone())
}

/// La app solo se compila para Windows. Se conserva el comando porque la vía de navegador
/// devuelve `false` aquí y la vista de reglas se pinta según esto.
#[tauri::command]
pub fn foreground_available() -> bool {
    true
}

// El evento `foreground:error` no se emite nunca desde aquí y no es un olvido: en Electron
// los errores eran del proceso de PowerShell (no arrancaba, se moría, escupía por stderr) y
// ese proceso ya no existe. Lo que puede fallar ahora —una ventana sin nombre resoluble— ya
// se resuelve devolviendo campos vacíos, que es lo que hacía allí también. El renderer sigue
// suscrito porque el contrato es el de Electron y no se toca.

fn arranque_pedido() -> bool {
    crate::config::leer()
        .get("autoProfile")
        .and_then(|a| a.get("enabled"))
        .and_then(Value::as_bool)
        .unwrap_or(false)
}

/// Arranca el vigilante al abrir la app **solo si** el usuario tiene encendido el cambio
/// automático de perfil. Sondear cada 400 ms para nadie no lo paga nadie.
pub fn arrancar_si_procede(app: AppHandle) {
    if arranque_pedido() {
        arrancar(app);
    }
}
