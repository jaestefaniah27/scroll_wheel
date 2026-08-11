// Las secuencias que ejecuta el PC.
//
// El teclado hace él solo lo que le cabe (retardos, atajos, clics), y manda `MACRO:<id>`
// solo cuando la secuencia tiene algún paso que no sabe hacer: una posición absoluta de
// ratón, un texto, abrir un programa, apagar el equipo o un complemento.
//
// La traducción de códigos de tecla vive en `orby_core::teclas`, que es donde se puede
// probar. Aquí solo está el trato con Windows.

use std::os::windows::process::CommandExt;
use std::sync::{Mutex, OnceLock};
use std::thread;
use std::time::Duration;

use orby_core::teclas::{code_a_vk, hid_a_vk, modificadores_a_vk, Tecla};
use serde_json::Value;
use tauri::AppHandle;
use tauri_plugin_clipboard_manager::ClipboardExt;
use windows::core::PCWSTR;
use windows::Win32::UI::Input::KeyboardAndMouse::{
    SendInput, INPUT, INPUT_0, INPUT_KEYBOARD, INPUT_MOUSE, KEYBDINPUT,
    KEYBD_EVENT_FLAGS, KEYEVENTF_EXTENDEDKEY, KEYEVENTF_KEYUP, KEYEVENTF_UNICODE, MOUSEINPUT,
    MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP, MOUSEEVENTF_MIDDLEDOWN, MOUSEEVENTF_MIDDLEUP,
    MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP, MOUSE_EVENT_FLAGS, VIRTUAL_KEY,
};
use windows::Win32::UI::Shell::ShellExecuteW;
use windows::Win32::UI::WindowsAndMessaging::{
    GetSystemMetrics, SetCursorPos, SM_CXSCREEN, SM_CYSCREEN, SW_SHOWNORMAL,
};

/// Hueco entre repeticiones cuando la acción no trae el suyo. Tiene que coincidir con el
/// `DEFAULT_REPEAT_GAP_MS` del editor (`src/views/profiles.js`).
const HUECO_POR_DEFECTO_MS: u64 = 20;

/// Con 0 hay aplicaciones que se comen letras sueltas porque no llegan a procesar el
/// evento anterior.
const RETARDO_ENTRE_LETRAS_MS: u64 = 4;

/// Por debajo de esto no compensa pisar el portapapeles del usuario. Por encima, escribir
/// letra a letra se ve teclear y no hay ajuste que lo arregle: cada carácter es un evento
/// que el sistema entrega y la app destino procesa.
const MINIMO_PARA_PEGAR: usize = 5;

/// Margen para que el portapapeles quede escrito antes del Ctrl+V, y para que la app
/// destino lo lea antes de devolverle al usuario lo que tenía. Restaurarlo de inmediato
/// hace que algunas apps peguen lo viejo: leen el portapapeles al procesar el atajo, no al
/// recibirlo.
const PORTAPAPELES_ASENTAR_MS: u64 = 30;
const PORTAPAPELES_RESTAURAR_MS: u64 = 120;

const VK_CONTROL_IZQ: u16 = 0xa2;
const VK_V: u16 = 0x56;
const VK_TAB: u16 = 0x09;
const VK_INTRO: u16 = 0x0d;

/// Dos secuencias pueden solaparse —el teclado no espera a que acabe una para mandar la
/// siguiente, y en Electron tampoco se esperaba—, pero el portapapeles es uno solo: sin
/// esto, la segunda lo pisa mientras la primera todavía no ha pegado y el usuario acaba
/// con el texto equivocado y el portapapeles de otro.
fn cerrojo_portapapeles() -> &'static Mutex<()> {
    static CERROJO: OnceLock<Mutex<()>> = OnceLock::new();
    CERROJO.get_or_init(|| Mutex::new(()))
}

fn dormir(ms: u64) {
    if ms > 0 {
        thread::sleep(Duration::from_millis(ms));
    }
}

// --- Envío de eventos a Windows ---

fn enviar(entradas: &[INPUT]) {
    unsafe { SendInput(entradas, std::mem::size_of::<INPUT>() as i32) };
}

fn evento_tecla(tecla: Tecla, soltar: bool) -> INPUT {
    let mut flags = KEYBD_EVENT_FLAGS(0);
    if tecla.extendida {
        flags |= KEYEVENTF_EXTENDEDKEY;
    }
    if soltar {
        flags |= KEYEVENTF_KEYUP;
    }

    INPUT {
        r#type: INPUT_KEYBOARD,
        Anonymous: INPUT_0 {
            ki: KEYBDINPUT {
                wVk: VIRTUAL_KEY(tecla.vk),
                wScan: 0,
                dwFlags: flags,
                time: 0,
                dwExtraInfo: 0,
            },
        },
    }
}

/// Un carácter suelto, por código Unicode en vez de por tecla virtual: así se escribe
/// igual sea cual sea la distribución del teclado del usuario, que es justo lo que
/// rompía escribir acentos con la vía de las teclas virtuales.
fn evento_unicode(unidad: u16, soltar: bool) -> INPUT {
    let mut flags = KEYEVENTF_UNICODE;
    if soltar {
        flags |= KEYEVENTF_KEYUP;
    }

    INPUT {
        r#type: INPUT_KEYBOARD,
        Anonymous: INPUT_0 {
            ki: KEYBDINPUT {
                wVk: VIRTUAL_KEY(0),
                wScan: unidad,
                dwFlags: flags,
                time: 0,
                dwExtraInfo: 0,
            },
        },
    }
}

fn evento_raton(flags: MOUSE_EVENT_FLAGS) -> INPUT {
    INPUT {
        r#type: INPUT_MOUSE,
        Anonymous: INPUT_0 {
            mi: MOUSEINPUT {
                dx: 0,
                dy: 0,
                mouseData: 0,
                dwFlags: flags,
                time: 0,
                dwExtraInfo: 0,
            },
        },
    }
}

fn pulsar(teclas: &[Tecla]) {
    let bajada: Vec<INPUT> = teclas.iter().map(|t| evento_tecla(*t, false)).collect();
    // Se sueltan en orden inverso: con Ctrl+Alt+B, soltar el Control antes que la B deja
    // a la aplicación destino viendo una B suelta.
    let subida: Vec<INPUT> = teclas.iter().rev().map(|t| evento_tecla(*t, true)).collect();
    enviar(&bajada);
    enviar(&subida);
}

fn escribir_caracter(c: char) {
    let mut buffer = [0u16; 2];
    for unidad in c.encode_utf16(&mut buffer) {
        enviar(&[evento_unicode(*unidad, false), evento_unicode(*unidad, true)]);
    }
}

// --- Pasos ---

fn repetir(accion: &Value, mut paso: impl FnMut()) {
    let veces = accion
        .get("count")
        .and_then(Value::as_f64)
        .map(|v| v.round() as i64)
        .filter(|v| *v >= 1)
        .unwrap_or(1);

    let hueco = accion
        .get("gap")
        .and_then(Value::as_f64)
        .map(|v| v.round().max(0.0) as u64)
        .unwrap_or(HUECO_POR_DEFECTO_MS);

    for i in 0..veces {
        paso();
        // Entre repeticiones, no después de la última: si no, cada secuencia arrastraría
        // un hueco de más al acabar.
        if i < veces - 1 {
            dormir(hueco);
        }
    }
}

/// Mete el texto de golpe por el portapapeles. `false` si algo falla, para que el que
/// llama caiga a escribirlo letra a letra.
///
/// Se restaura solo la parte de texto de lo que hubiera: si era una imagen o un archivo,
/// se pierde. Es lo mismo que hacía Electron.
fn pegar(app: &AppHandle, texto: &str) -> bool {
    let _guarda = cerrojo_portapapeles().lock().unwrap_or_else(|e| e.into_inner());

    let previo = app.clipboard().read_text().unwrap_or_default();

    if app.clipboard().write_text(texto.to_string()).is_err() {
        return false;
    }
    dormir(PORTAPAPELES_ASENTAR_MS);

    pulsar(&[
        Tecla { vk: VK_CONTROL_IZQ, extendida: false },
        Tecla { vk: VK_V, extendida: false },
    ]);

    dormir(PORTAPAPELES_RESTAURAR_MS);

    // La restauración va pase lo que pase por encima: dejar al usuario sin su portapapeles
    // es de los fallos que más molestan y que nadie asocia con el teclado.
    if !previo.is_empty() {
        let _ = app.clipboard().write_text(previo);
    }
    true
}

/// Un trozo de texto sin saltos: de golpe por el portapapeles si es largo, letra a letra
/// si es corto o si el pegado falla.
fn volcar(app: &AppHandle, trozo: &str) {
    if trozo.is_empty() {
        return;
    }
    if trozo.chars().count() >= MINIMO_PARA_PEGAR && pegar(app, trozo) {
        return;
    }
    for c in trozo.chars() {
        escribir_caracter(c);
        dormir(RETARDO_ENTRE_LETRAS_MS);
    }
}

fn escribir_texto(app: &AppHandle, texto: &str) {
    // Los saltos de línea y los tabuladores van como pulsaciones de verdad: pegados o
    // escritos como carácter, muchas aplicaciones no los interpretan.
    let mut trozo = String::new();
    let mut restantes = texto.chars().peekable();

    while let Some(c) = restantes.next() {
        match c {
            '\t' => {
                volcar(app, &trozo);
                trozo.clear();
                pulsar(&[Tecla { vk: VK_TAB, extendida: false }]);
            }
            '\n' | '\r' => {
                volcar(app, &trozo);
                trozo.clear();
                // «\r\n» es un salto, no dos.
                if c == '\r' && restantes.peek() == Some(&'\n') {
                    restantes.next();
                }
                pulsar(&[Tecla { vk: VK_INTRO, extendida: false }]);
            }
            _ => trozo.push(c),
        }
    }

    volcar(app, &trozo);
}

fn accion_energia(modo: &str) {
    // Sin opcode de firmware para esto: siempre corre por el camino del PC.
    let comando = match modo {
        "sleep" => "rundll32.exe powrprof.dll,SetSuspendState 0,1,0",
        "hibernate" => "shutdown /h",
        "restart" => "shutdown /r /t 0",
        "shutdown" => "shutdown /s /t 0",
        "lock" => "rundll32.exe user32.dll,LockWorkStation",
        "logoff" => "shutdown /l",
        _ => {
            eprintln!("[secuencias] acción de energía «{modo}» desconocida");
            return;
        }
    };

    // CREATE_NO_WINDOW: sin esto asoma una consola negra un instante cada vez que se
    // bloquea el equipo desde una tecla.
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    if let Err(e) = std::process::Command::new("cmd")
        .args(["/C", comando])
        .creation_flags(CREATE_NO_WINDOW)
        .spawn()
    {
        eprintln!("[secuencias] acción de energía «{modo}»: {e}");
    }
}

fn abrir(destino: &str) {
    let ancho: Vec<u16> = destino.encode_utf16().chain(std::iter::once(0)).collect();
    let verbo: Vec<u16> = "open".encode_utf16().chain(std::iter::once(0)).collect();

    // Equivale al `shell.openPath` de Electron: abre con el programa asociado, valga lo
    // que valga el destino (un .exe, un documento, una carpeta o una URL).
    let r = unsafe {
        ShellExecuteW(
            None,
            PCWSTR(verbo.as_ptr()),
            PCWSTR(ancho.as_ptr()),
            None,
            None,
            SW_SHOWNORMAL,
        )
    };
    // Devuelve un pseudo-HINSTANCE: por debajo de 32 es un código de error.
    if r.0 as usize <= 32 {
        eprintln!("[secuencias] no se pudo abrir «{destino}»");
    }
}

fn ejecutar_accion(app: &AppHandle, accion: &Value) {
    let tipo = accion.get("type").and_then(Value::as_str).unwrap_or("");

    match tipo {
        "delay" => dormir(accion.get("ms").and_then(Value::as_u64).unwrap_or(0)),

        "mouse_position" => {
            let (Some(x), Some(y)) = (
                accion.get("x").and_then(Value::as_i64),
                accion.get("y").and_then(Value::as_i64),
            ) else {
                return;
            };
            // Coordenadas físicas del escritorio virtual, las mismas que devuelve
            // `mouse_get_position`. Ver el comentario de mouse.rs.
            let _ = unsafe { SetCursorPos(x as i32, y as i32) };
        }

        "center_mouse" => {
            // Formato antiguo: el centro de la pantalla principal.
            let ancho = unsafe { GetSystemMetrics(SM_CXSCREEN) };
            let alto = unsafe { GetSystemMetrics(SM_CYSCREEN) };
            let _ = unsafe { SetCursorPos(ancho / 2, alto / 2) };
        }

        "mouse_click" => {
            let (abajo, arriba) = match accion.get("button").and_then(Value::as_str) {
                Some("right") => (MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP),
                Some("middle") => (MOUSEEVENTF_MIDDLEDOWN, MOUSEEVENTF_MIDDLEUP),
                _ => (MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP),
            };
            repetir(accion, || {
                enviar(&[evento_raton(abajo), evento_raton(arriba)]);
            });
        }

        "key" => {
            let Some(tecla) = accion
                .get("code")
                .and_then(Value::as_str)
                .and_then(code_a_vk)
            else {
                return;
            };
            repetir(accion, || pulsar(&[tecla]));
        }

        "hotkey" => {
            let modificador = accion.get("modifier").and_then(Value::as_u64).unwrap_or(0) as u8;
            let mut teclas = modificadores_a_vk(modificador);
            if let Some(principal) = accion
                .get("keycode")
                .and_then(Value::as_u64)
                .and_then(|k| hid_a_vk(k as u8))
            {
                teclas.push(principal);
            }
            if teclas.is_empty() {
                return;
            }
            repetir(accion, || pulsar(&teclas));
        }

        "text" => {
            let Some(texto) = accion.get("text").and_then(Value::as_str) else { return };
            if texto.is_empty() {
                return;
            }
            repetir(accion, || escribir_texto(app, texto));
        }

        "open_app" => {
            if let Some(destino) = accion.get("target").and_then(Value::as_str) {
                if !destino.is_empty() {
                    abrir(destino);
                }
            }
        }

        "system_power" => {
            if let Some(modo) = accion.get("mode").and_then(Value::as_str) {
                accion_energia(modo);
            }
        }

        // Los complementos llegan en la Tarea 10. Hasta entonces se deja constancia en vez
        // de fallar en silencio, que es lo que haría creer que la tecla no está asignada.
        "plugin" => eprintln!("[secuencias] paso de complemento todavía sin portar"),

        // Un tipo desconocido no hace nada, igual que en Electron: una secuencia guardada
        // por una versión más nueva no puede tumbar a la vieja.
        otro => eprintln!("[secuencias] paso «{otro}» desconocido"),
    }
}

/// Mira si la línea es un disparo y, si lo es, ejecuta la secuencia. Se llama desde el
/// hilo del puerto, así que el trabajo se va a otro hilo: una secuencia con un `delay` de
/// diez segundos dejaría de leer el puerto y el vigilante daría la conexión por muerta.
pub fn quizas_disparar(app: &AppHandle, linea: &str) {
    let Some(id) = orby_core::protocolo::macro_trigger(linea) else { return };

    let app = app.clone();
    thread::spawn(move || ejecutar(&app, id));
}

fn ejecutar(app: &AppHandle, id: u32) {
    let config = crate::config::leer();
    let Some(macro_) = config
        .get("macros")
        .and_then(Value::as_array)
        .and_then(|lista| {
            lista
                .iter()
                .find(|m| m.get("id").and_then(Value::as_u64) == Some(id as u64))
        })
    else {
        eprintln!("[secuencias] disparo de la {id}, que no existe en la configuración");
        return;
    };

    // Las grabaciones las lleva recorder.rs (Tarea 7), no este ejecutor.
    match macro_.get("kind").and_then(Value::as_str) {
        Some("recording") | Some("recording-reset") => {
            eprintln!("[secuencias] la {id} es una grabación: todavía sin portar");
            return;
        }
        _ => {}
    }

    let Some(acciones) = macro_.get("actions").and_then(Value::as_array) else { return };

    // Un paso que falla no aborta la secuencia: si el sexto de ocho no se puede hacer, los
    // otros siete siguen valiendo.
    for accion in acciones {
        ejecutar_accion(app, accion);
    }
}
