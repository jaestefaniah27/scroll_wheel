// El puerto de verdad: descubrimiento, apertura y bombeo de bytes.
//
// Aquí NO hay lógica de conexión. Cuándo saludar, cuándo insistir, cuándo darse por
// desconectado y qué se le manda al teclado lo decide `orby_core::serie::Maquina`, que
// vive aparte porque es lo que se puede probar sin un teclado delante. Este módulo solo
// hace lo que la máquina pide: escribir en el puerto, emitir al renderer y soltar.
//
// Tampoco hay correlación entre petición y respuesta: eso es de `src/device.js`, con su
// cola de `pending`. De aquí sale **toda** línea recibida tal cual, incluida la
// telemetría que nadie pidió (`EV:CTX:`, `ENC:`, `KEY_EV:`).

use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::log::log;
use orby_core::lines::split_lines;
use orby_core::protocolo::DeviceInfo;
use orby_core::serie::{Evento, Maquina, Salida};
use serde_json::{Map, Value};
use serialport::{SerialPort, SerialPortType};
use tauri::{AppHandle, Emitter, Manager};

/// VID de los ejemplos de TinyUSB, el que llevan todos los firmwares publicados.
///
/// El PID **no** se filtra: sube con cada cambio de informe HID (Windows cachea el
/// descriptor por VID/PID), así que una lista de PID dejaría fuera a los firmwares
/// viejos.
const VID_TECLADO: u16 = 0xCAFE;
const BAUDIOS: u32 = 115_200;

/// Cada cuánto se mira si ha aparecido un puerto candidato.
const RASTREO_MS: u64 = 3_000;

/// El pulso del hilo. Es a la vez el plazo de la lectura, así que el hilo duerme dentro
/// del `read` en vez de girar en vacío, y marca la latencia máxima de un comando del
/// renderer (frente a los 4 s que espera `device.js`, no cuenta).
const TIC_MS: u64 = 20;

/// Con el puerto cerrado no hay nada que leer y el pulso rápido no aporta nada.
const REPOSO_MS: u64 = 200;

/// Lo que se le da al teclado para tragar una escritura. Nada que ver con `TIC_MS`: el
/// firmware se queda ~120 ms sin atender el USB cada vez que escribe en Flash (borrar y
/// programar 8 KB, `write_oled_slice` en main.cpp), y con el plazo del pulso Windows
/// devolvía ERROR_SEM_TIMEOUT y la conexión se caía sola. Con este margen ese caso deja
/// de darse; si aun así se agota, es que el teclado está de verdad atascado.
const ESCRITURA_MS: u64 = 2_000;

/// Windows cuenta el plazo de escritura agotado como ERROR_SEM_TIMEOUT (121), no como un
/// `TimedOut` de los que Rust clasifica. Sin mirar el número crudo se cuela por el mismo
/// sitio que un desenchufado de verdad, que es lo que tiraba la conexión.
fn es_plazo(e: &std::io::Error) -> bool {
    e.kind() == ErrorKind::TimedOut || e.raw_os_error() == Some(121)
}

/// Lo que el hilo del puerto comparte con los comandos del renderer.
#[derive(Default)]
struct Compartido {
    conectado: bool,
    /// Ya en la forma que espera el renderer, para no reconstruirla en cada consulta.
    info: Option<Value>,
    /// Los comandos no escriben en el puerto: lo hace el hilo que lo posee. Así no hay
    /// dos escritores sobre el mismo handle ni hace falta duplicarlo.
    salientes: VecDeque<String>,
    /// El botón «Reconectar»: fuerza soltar lo que haya y rastrear ya.
    rescan: bool,
}

pub struct Serie(Arc<Mutex<Compartido>>);

/// Monta el estado compartido y lanza el hilo del puerto. Se llama desde el `setup`.
pub fn arrancar(app: AppHandle) {
    let compartido = Arc::new(Mutex::new(Compartido::default()));
    app.manage(Serie(compartido.clone()));
    std::thread::spawn(move || hilo(app, compartido));
}

// --- Comandos ---

#[tauri::command]
pub fn serial_send(estado: tauri::State<'_, Serie>, cmd: String) -> bool {
    // En desarrollo sale todo por el terminal: es la única forma cómoda de ver que una
    // reconexión no está releyendo la configuración entera (si aparecen GET_PROFILE o
    // GET_OLED_PG, es que sí).
    #[cfg(debug_assertions)]
    eprintln!("[serie] > {cmd}");

    let mut c = estado.0.lock().unwrap();
    if !c.conectado {
        return false;
    }
    let linea = if cmd.ends_with('\n') { cmd } else { format!("{cmd}\n") };
    c.salientes.push_back(linea);
    true
}

#[tauri::command]
pub fn serial_get_info(estado: tauri::State<'_, Serie>) -> Option<Value> {
    estado.0.lock().unwrap().info.clone()
}

#[tauri::command]
pub fn serial_get_status(estado: tauri::State<'_, Serie>) -> &'static str {
    // El hilo rastrea siempre mientras la app vive, así que «disconnected» no llega a
    // darse; se conserva en el contrato porque las otras dos vías sí lo usan.
    if estado.0.lock().unwrap().conectado {
        "connected"
    } else {
        "searching"
    }
}

#[tauri::command]
pub fn serial_reconnect(estado: tauri::State<'_, Serie>) -> bool {
    estado.0.lock().unwrap().rescan = true;
    true
}

// --- El hilo del puerto ---

fn hilo(app: AppHandle, compartido: Arc<Mutex<Compartido>>) {
    let mut maquina = Maquina::new();
    let mut puerto: Option<Box<dyn SerialPort>> = None;
    let mut ruta = String::new();
    let mut resto = String::new();
    let mut buf = [0u8; 1024];

    // Todo lo que va al teclado pasa por aquí, venga del renderer o de la máquina de
    // estados: un único escritor y una única cola. Antes la máquina escribía directa al
    // puerto desde `aplicar`, y su `ACK\n` podía colarse en mitad de una línea larga que
    // se hubiera quedado a medias (`OLED_CHUNK` son ~200 bytes y no siempre entran de una
    // vez). El firmware la parseaba truncada: se veía como `OLED:OK:1:3:0:24` en vez de
    // los 90 bytes del trozo. En bytes y no en líneas porque una escritura parcial corta
    // por donde quiere, no por el salto de línea.
    let mut por_escribir: VecDeque<u8> = VecDeque::new();

    let mut ultimo_tic = Instant::now();
    let mut proximo_rastreo = Instant::now();
    // Se rota entre los candidatos en vez de probar siempre el primero: con dos puertos
    // del mismo VID (una Pico de repuesto en modo carga, otro proyecto con el VID de
    // TinyUSB) probar siempre el mismo dejaría el teclado bueno sin abrir para siempre.
    let mut turno: usize = 0;

    let _ = app.emit("serial:searching", ());

    loop {
        let ahora = Instant::now();
        let delta = ahora.duration_since(ultimo_tic).as_millis() as u64;
        ultimo_tic = ahora;
        if delta > 0 {
            let salidas = maquina.al_pasar_tiempo(delta);
            aplicar(salidas, &app, &compartido, &mut puerto, &mut por_escribir, &mut resto, &ruta);
        }

        // El botón «Reconectar» tiene que forzar un intento YA: el caso que cubre es
        // justo que el primer saludo se perdiera y esperar al siguiente rastreo haga
        // parecer que el botón no responde.
        let rescan = {
            let mut c = compartido.lock().unwrap();
            std::mem::take(&mut c.rescan)
        };
        if rescan {
            if puerto.is_some() {
                puerto = None;
                resto.clear();
                por_escribir.clear();
                let salidas = maquina.al_cerrarse();
                aplicar(salidas, &app, &compartido, &mut puerto, &mut por_escribir, &mut resto, &ruta);
            }
            proximo_rastreo = ahora;
        }

        if let Some(p) = puerto.as_mut() {
            // Lo que el renderer haya encolado desde la última vuelta, al final de la cola
            // de bytes: detrás de lo que la máquina de estados ya hubiera puesto y de lo
            // que quedara a medias de la vuelta anterior.
            {
                let mut c = compartido.lock().unwrap();
                for linea in c.salientes.drain(..) {
                    por_escribir.extend(linea.as_bytes());
                }
            }

            let mut se_rompio = false;
            // El plazo del puerto es el pulso del hilo (20 ms), y `serialport` lo usa para
            // leer **y** para escribir: no se pueden separar por su API. Escribir con 20 ms
            // es lo que tiraba la conexión, porque el firmware se queda ~120 ms sin atender
            // el USB mientras escribe en Flash (`write_oled_slice`, main.cpp) y Windows
            // devuelve entonces ERROR_SEM_TIMEOUT. Así que se alarga solo mientras se
            // escribe y se vuelve a dejar corto para que la lectura siga marcando el pulso.
            if !por_escribir.is_empty() {
                let _ = p.set_timeout(Duration::from_millis(ESCRITURA_MS));
            }
            while !por_escribir.is_empty() {
                // `as_slices().0` nunca está vacío si la cola no lo está.
                let frente = por_escribir.as_slices().0;
                match p.write(frente) {
                    Ok(0) => break,
                    Ok(n) => {
                        por_escribir.drain(..n);
                    }
                    // Agotar el plazo tampoco ahora es un teclado desenchufado, solo uno que
                    // no traga. Pero al fallar, `serialport` tira el número de bytes que sí
                    // llegaron, así que no se puede reanudar por donde iba: reintentar
                    // duplicaría lo ya enviado. Se abandona el comando hasta el salto de
                    // línea, que deja el flujo cuadrado en la frontera siguiente; quien lo
                    // pidió lo verá vencer y lo repetirá entero, que es lo correcto.
                    Err(e) if es_plazo(&e) => {
                        log!("[serie] {ruta} no traga: se abandona un comando ({e})");
                        let corte = por_escribir.iter().position(|&b| b == b'\n');
                        match corte {
                            Some(i) => drop(por_escribir.drain(..=i)),
                            None => por_escribir.clear(),
                        }
                        break;
                    }
                    Err(e) => {
                        log!("[serie] {ruta} CAIDA por error de ESCRITURA: {e}");
                        se_rompio = true;
                        break;
                    }
                }
            }

            // Vuelta al plazo corto: es el que hace que el hilo duerma dentro del `read` en
            // vez de girar en vacío, y con los 2 s de la escritura el bucle iría a paso de
            // tortuga cada vez que el teclado no dijera nada.
            let _ = p.set_timeout(Duration::from_millis(TIC_MS));

            if !se_rompio {
                match p.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
                        let trozo = String::from_utf8_lossy(&buf[..n]);
                        let (lineas, nuevo) = split_lines(&resto, &trozo);
                        resto = nuevo;
                        for linea in lineas {
                            let salidas = maquina.al_recibir(&linea);
                            aplicar(salidas, &app, &compartido, &mut puerto, &mut por_escribir, &mut resto, &ruta);
                            // Una de esas líneas pudo hacer que se soltara el puerto.
                            if puerto.is_none() {
                                break;
                            }
                        }
                    }
                    // El plazo de lectura es el pulso del hilo: agotarlo es lo normal.
                    Err(e) if e.kind() == ErrorKind::TimedOut => {}
                    Err(e) => {
                        log!("[serie] {ruta} CAIDA por error de LECTURA: {e}");
                        se_rompio = true;
                    }
                }
            }

            if se_rompio {
                // Se ha desenchufado. Soltar el handle **antes** de anunciar nada: el
                // siguiente intento de apertura se estrella contra «Access denied» si
                // queda un handle huérfano.
                puerto = None;
                resto.clear();
                por_escribir.clear();
                let salidas = maquina.al_cerrarse();
                aplicar(salidas, &app, &compartido, &mut puerto, &mut por_escribir, &mut resto, &ruta);
                proximo_rastreo = Instant::now() + Duration::from_millis(RASTREO_MS);
            }
        } else if ahora >= proximo_rastreo {
            proximo_rastreo = ahora + Duration::from_millis(RASTREO_MS);

            let candidatos = candidatos();
            if !candidatos.is_empty() {
                let elegido = candidatos[turno % candidatos.len()].clone();
                turno = turno.wrapping_add(1);

                match abrir(&elegido) {
                    Ok(p) => {
                        eprintln!("[serie] probando {elegido}");
                        ruta = elegido;
                        resto.clear();
                        // Lo que quedara a medias iba a la conexión anterior: mandarlo
                        // ahora sería empezar la nueva con media línea sin sentido.
                        por_escribir.clear();
                        puerto = Some(p);
                        let salidas = maquina.al_abrir();
                        aplicar(salidas, &app, &compartido, &mut puerto, &mut por_escribir, &mut resto, &ruta);
                        // El saludo empieza a contar ahora, no desde el rastreo.
                        ultimo_tic = Instant::now();
                    }
                    // Que otro proceso lo tenga cogido (la app de Electron, la WebGUI) es
                    // el caso normal, no un error que enseñarle a nadie.
                    Err(e) => eprintln!("[serie] no se pudo abrir {elegido}: {e}"),
                }
            }
        }

        if puerto.is_none() {
            std::thread::sleep(Duration::from_millis(REPOSO_MS));
        }
    }
}

/// Puertos que casan por VID. Nunca se prueba «cualquier COM»: abrir a ciegas el puerto
/// de otro cacharro puede dejar el hilo colgado dentro del driver.
fn candidatos() -> Vec<String> {
    let Ok(puertos) = serialport::available_ports() else {
        return Vec::new();
    };

    puertos
        .into_iter()
        .filter(|p| match &p.port_type {
            SerialPortType::UsbPort(usb) => usb.vid == VID_TECLADO,
            _ => false,
        })
        .map(|p| p.port_name)
        .collect()
}

fn abrir(ruta: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
    let mut p = serialport::new(ruta, BAUDIOS)
        .timeout(Duration::from_millis(TIC_MS))
        .open()?;

    // Sin DTR y RTS, TinyUSB no da por abierto el canal y no llega ni un byte de vuelta:
    // el teclado parece mudo y el saludo se agota siempre.
    p.write_data_terminal_ready(true)?;
    p.write_request_to_send(true)?;
    Ok(p)
}

fn aplicar(
    salidas: Vec<Salida>,
    app: &AppHandle,
    compartido: &Arc<Mutex<Compartido>>,
    puerto: &mut Option<Box<dyn SerialPort>>,
    por_escribir: &mut VecDeque<u8>,
    resto: &mut String,
    ruta: &str,
) {
    for salida in salidas {
        match salida {
            Salida::Mandar(texto) => {
                // Lo que manda la máquina por su cuenta (los ACK del saludo y los del
                // vigilante) también sale por el terminal en desarrollo: es la única
                // forma de ver que una conexión en reposo se está renovando en vez de
                // estar a punto de caerse.
                #[cfg(debug_assertions)]
                eprintln!("[serie] >> {}", texto.trim_end());

                // A la cola, no al puerto. Escribir aquí directamente metía estos cuatro
                // bytes en mitad de la línea que el bucle tuviera a medias.
                if puerto.is_some() {
                    por_escribir.extend(texto.as_bytes());
                }
            }
            Salida::SoltarPuerto => {
                // Lo pide la máquina de estados, no el puerto: o el vigilante dio la
                // conexión por muerta (12 s de silencio + 4 s sin contestar al ACK), o el
                // saludo se rindió. Se distingue de las dos caídas por E/S de arriba.
                log!("[serie] {ruta} lo suelta la MAQUINA (vigilante o saludo)");
                *puerto = None;
                resto.clear();
                por_escribir.clear();
                let mut c = compartido.lock().unwrap();
                c.conectado = false;
                c.info = None;
                c.salientes.clear();
            }
            Salida::Emitir(evento) => emitir(app, compartido, evento, ruta),
        }
    }
}

fn emitir(app: &AppHandle, compartido: &Arc<Mutex<Compartido>>, evento: Evento, ruta: &str) {
    match evento {
        Evento::Conectado(info) => {
            let json = info_a_json(&info, ruta);
            {
                let mut c = compartido.lock().unwrap();
                c.conectado = true;
                c.info = Some(json.clone());
            }
            log!(
                "[serie] CONECTADO: fw {} en {ruta}",
                info.get("fw").unwrap_or("?")
            );
            let _ = app.emit("serial:connected", json);
        }
        Evento::Desconectado => {
            {
                let mut c = compartido.lock().unwrap();
                c.conectado = false;
                c.info = None;
                // Lo que quedara por mandar iba a un teclado que ya no está.
                c.salientes.clear();
            }
            // El motivo va en la línea de arriba (E/S de lectura, de escritura o la
            // máquina): esta solo marca el momento en que la app lo dio por perdido.
            log!("[serie] DESCONECTADO de {ruta}");
            let _ = app.emit("serial:disconnected", ());
        }
        Evento::Buscando => {
            let _ = app.emit("serial:searching", ());
        }
        Evento::Datos(linea) => {
            // El disparo de una secuencia se mira aquí, pero la línea se reenvía igual al
            // renderer: `device.js` no distingue, y quitarle telemetría le rompería la
            // cola de peticiones.
            crate::macros::quizas_disparar(app, &linea);
            // En modo `hold` la reproducción dura lo que la tecla siga pulsada, y eso solo
            // se ve por la telemetría: el firmware no manda un segundo `MACRO:` al soltar.
            crate::recorder::quizas_soltar_hold(&linea);
            let _ = app.emit("serial:data", linea);
        }
    }
}

/// La forma que produce `_parseDeviceInfo` de electron/serial.js: un objeto plano con las
/// claves en minúscula, más `device`, `raw` y `port`. `compat.js` y las vistas la leen
/// así, y no se tocan en esta migración.
///
/// Todos los valores salen como cadena, también en la identidad mínima por telemetría.
/// Allí Electron ponía `keys` y `oleds` como número, pero era una incoherencia consigo
/// mismo: por la vía normal el saludo se parte en cadenas y el único que los lee
/// (`views/settings.js`) los interpola en una plantilla.
fn info_a_json(info: &DeviceInfo, ruta: &str) -> Value {
    let mut m = Map::new();
    for (clave, valor) in &info.campos {
        m.insert(clave.clone(), Value::String(valor.clone()));
    }
    m.insert("device".into(), Value::String(info.device.clone()));
    m.insert("raw".into(), Value::String(info.raw.clone()));
    m.insert(
        "port".into(),
        if ruta.is_empty() {
            Value::Null
        } else {
            Value::String(ruta.to_string())
        },
    );
    Value::Object(m)
}
