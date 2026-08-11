// La grabadora de ratón y teclado.
//
// Una tecla de tipo «grabación» hace tres cosas distintas con la misma pulsación según en
// qué punto esté: sin nada grabado empieza a grabar, grabando para y guarda, y con algo
// grabado lo reproduce. El reparto está en `macros::disparar`; aquí está el trabajo.
//
// Qué se guarda y cuándo se suelta cada evento vive en `orby_core::grabacion`, y la
// traducción de códigos en `orby_core::uiohook`. Aquí solo queda el trato con Windows:
// los ganchos de bajo nivel para capturar y `SendInput` para reproducir.
//
// Dos cosas de esta tarea rompen el equipo entero si se hacen mal, y las dos están
// resueltas a propósito y no por casualidad:
//
//   1. Los ganchos `WH_KEYBOARD_LL` y `WH_MOUSE_LL` **solo** entregan eventos al hilo que
//      los instaló y **solo** mientras ese hilo bombea mensajes. Sin el bucle de
//      `GetMessageW` se instalan sin error, no llega ni un evento y no hay ningún síntoma
//      que lo explique.
//   2. Al parar una reproducción hay que soltar todas las teclas y botones que se
//      quedaran hundidos, también si se corta a mitad. Un bucle cortado con el Control
//      pulsado deja el equipo inservible hasta reiniciar, y el usuario no lo relaciona
//      con el teclado.

use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use orby_core::grabacion::{espera_ms, leer_eventos, Evento, Grabador};
use orby_core::uiohook::{tecla_a_uiohook, uiohook_a_tecla};
use serde_json::{json, Map, Value};
use tauri::{AppHandle, Emitter};
use windows::Win32::Foundation::{LPARAM, LRESULT, WPARAM};
use windows::Win32::System::Threading::GetCurrentThreadId;
use windows::Win32::UI::Input::KeyboardAndMouse::{
    MOUSEEVENTF_HWHEEL, MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP, MOUSEEVENTF_MIDDLEDOWN,
    MOUSEEVENTF_MIDDLEUP, MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP, MOUSEEVENTF_WHEEL,
    MOUSE_EVENT_FLAGS,
};
use windows::Win32::UI::WindowsAndMessaging::{
    CallNextHookEx, GetMessageW, PostThreadMessageW, SetCursorPos, SetWindowsHookExW,
    UnhookWindowsHookEx, KBDLLHOOKSTRUCT, MSG, MSLLHOOKSTRUCT, WH_KEYBOARD_LL, WH_MOUSE_LL,
    WM_KEYDOWN, WM_KEYUP, WM_LBUTTONDOWN, WM_LBUTTONUP, WM_MBUTTONDOWN, WM_MBUTTONUP,
    WM_MOUSEHWHEEL, WM_MOUSEMOVE, WM_MOUSEWHEEL, WM_QUIT, WM_RBUTTONDOWN, WM_RBUTTONUP,
    WM_SYSKEYDOWN, WM_SYSKEYUP, WM_XBUTTONDOWN, WM_XBUTTONUP,
};

/// El gancho solo tiene que mirar el evento cuando el código es `HC_ACTION`; con cualquier
/// otro está obligado a pasarlo sin tocarlo.
const HC_ACTION: i32 = 0;

/// Banderas de `KBDLLHOOKSTRUCT` y `MSLLHOOKSTRUCT` que hacen falta. Se declaran aquí en
/// vez de importarlas porque en el crate `windows` viven en tipos distintos y acabaríamos
/// convirtiendo de un lado a otro para comparar dos bits.
const LLKHF_EXTENDIDA: u32 = 0x01;
const LLKHF_INYECTADO: u32 = 0x10;
const LLMHF_INYECTADO: u32 = 0x01;

/// Un giro de rueda completo, en las unidades que usan tanto el gancho como `SendInput`.
const RUEDA_DELTA: i32 = 120;

/// Modo por defecto y velocidad por defecto cuando la macro no los trae. Son los mismos
/// que pone el editor (`src/views/profiles/recorder.js`) y los mismos que ponía
/// `electron/main.js:314`; si se cambian aquí, una grabación vieja se reproduciría a otro
/// ritmo del que tenía.
const MODO_POR_DEFECTO: &str = "once";
const VELOCIDAD_POR_DEFECTO: f64 = 3.0;

/// La reproducción se corta esperando: sin trocear la espera, parar un bucle con un hueco
/// de tres segundos tardaría hasta tres segundos en obedecer, y en modo `hold` eso son
/// tres segundos de teclas cayendo después de soltar la tecla.
const TROZO_DE_ESPERA_MS: u64 = 10;

// --- Estado ---

struct EnCurso {
    grabador: Grabador,
    /// El reloj se guarda con la grabación y no se lee de la hora del sistema: un cambio
    /// de hora a mitad (o el horario de verano) desordenaría los tiempos.
    reloj: Instant,
}

struct Sesion {
    id: u32,
    modo: String,
    parada: AtomicBool,
    /// Corta sin avisar del final. Lo usa el borrado, que ya repinta la tecla por su
    /// cuenta: el aviso llegaría después y le devolvería el icono anterior por encima.
    silenciosa: AtomicBool,
}

struct Ganchos {
    hilo: JoinHandle<()>,
    /// Identificador de hilo de Windows, que es a quien se le manda el `WM_QUIT`. No vale
    /// el `ThreadId` de Rust: son numeraciones distintas.
    tid: u32,
}

static GRABANDO: Mutex<Option<EnCurso>> = Mutex::new(None);
static REPRO: Mutex<Option<Arc<Sesion>>> = Mutex::new(None);
static GANCHOS: Mutex<Option<Ganchos>> = Mutex::new(None);

/// Lo consultan los ganchos en cada evento, y por eso es un atómico y no el `Mutex` de
/// arriba: coger un cerrojo en el gancho para preguntarlo pondría a todo el sistema a
/// esperar a este proceso en cada tecla.
static REPRODUCIENDO: AtomicBool = AtomicBool::new(false);

fn emitir(app: &AppHandle, id: u32, fase: &str) {
    let _ = app.emit("recorder:state", json!({ "id": id, "phase": fase }));
}

// --- Captura ---

/// `true` si el evento lo ha metido esta misma app mientras reproduce.
///
/// Electron no filtraba nada, y por eso allí empezar a grabar con otra grabación en bucle
/// se grababa a sí misma. Se filtra **solo** mientras hay reproducción en marcha, y no
/// siempre: por escritorio remoto toda la entrada llega marcada como inyectada, y filtrar
/// sin condición dejaría la grabadora muerta en esa sesión sin decir por qué.
fn es_eco_propio(banderas: u32, marca: u32) -> bool {
    REPRODUCIENDO.load(Ordering::SeqCst) && banderas & marca != 0
}

unsafe extern "system" fn gancho_teclado(codigo: i32, wparam: WPARAM, lparam: LPARAM) -> LRESULT {
    if codigo == HC_ACTION {
        let datos = unsafe { &*(lparam.0 as *const KBDLLHOOKSTRUCT) };
        let mensaje = wparam.0 as u32;
        let abajo = mensaje == WM_KEYDOWN || mensaje == WM_SYSKEYDOWN;
        let arriba = mensaje == WM_KEYUP || mensaje == WM_SYSKEYUP;

        if (abajo || arriba) && !es_eco_propio(datos.flags.0, LLKHF_INYECTADO) {
            let extendida = datos.flags.0 & LLKHF_EXTENDIDA != 0;
            if let Some(c) = tecla_a_uiohook(datos.vkCode as u16, extendida) {
                con_grabacion(|en, ahora| en.grabador.tecla(ahora, abajo, c));
            }
        }
    }

    // Siempre se pasa el evento adelante: un gancho que se traga la entrada deja al
    // usuario sin teclado mientras graba.
    unsafe { CallNextHookEx(None, codigo, wparam, lparam) }
}

unsafe extern "system" fn gancho_raton(codigo: i32, wparam: WPARAM, lparam: LPARAM) -> LRESULT {
    if codigo == HC_ACTION {
        let datos = unsafe { &*(lparam.0 as *const MSLLHOOKSTRUCT) };
        if !es_eco_propio(datos.flags, LLMHF_INYECTADO) {
            let (x, y) = (datos.pt.x, datos.pt.y);
            // La parte alta de `mouseData` es el giro de la rueda y el botón lateral.
            let alta = (datos.mouseData >> 16) as i16;

            match wparam.0 as u32 {
                WM_MOUSEMOVE => {
                    con_grabacion(|en, ahora| {
                        en.grabador.mover(ahora, x, y);
                    });
                }
                WM_LBUTTONDOWN => boton(true, 1, x, y),
                WM_LBUTTONUP => boton(false, 1, x, y),
                WM_RBUTTONDOWN => boton(true, 2, x, y),
                WM_RBUTTONUP => boton(false, 2, x, y),
                WM_MBUTTONDOWN => boton(true, 3, x, y),
                WM_MBUTTONUP => boton(false, 3, x, y),
                // Los laterales se numeran 4 y 5, como en libuiohook.
                WM_XBUTTONDOWN => boton(true, 3 + alta as u32, x, y),
                WM_XBUTTONUP => boton(false, 3 + alta as u32, x, y),
                WM_MOUSEWHEEL => {
                    if let Some(r) = muescas(alta as i32) {
                        rueda(r, 3);
                    }
                }
                WM_MOUSEHWHEEL => {
                    if let Some(r) = muescas(alta as i32) {
                        rueda(r, 4);
                    }
                }
                _ => {}
            }
        }
    }

    unsafe { CallNextHookEx(None, codigo, wparam, lparam) }
}

/// Giro de Windows → giro guardado, en muescas y **con el signo cambiado**: así es como lo
/// dejaba libuiohook y así lo entiende `applyEvent` de Electron (negativo es hacia arriba,
/// o hacia la izquierda en el eje horizontal).
///
/// Se redondea siempre a una muesca como mínimo. Los ratones de precisión mandan giros más
/// pequeños que una muesca entera, y con una división a secas quedarían en cero: al
/// reproducirlos, un cero se toma como una muesca **hacia abajo** y el gesto saldría del
/// revés.
fn muescas(delta: i32) -> Option<i32> {
    if delta == 0 {
        return None;
    }
    let signo = if delta > 0 { 1 } else { -1 };
    Some(-signo * (delta.abs() / RUEDA_DELTA).max(1))
}

fn boton(abajo: bool, b: u32, x: i32, y: i32) {
    con_grabacion(|en, ahora| en.grabador.boton(ahora, abajo, b, x, y));
}

fn rueda(r: i32, d: u32) {
    con_grabacion(|en, ahora| en.grabador.rueda(ahora, r, d));
}

/// Corre `f` solo si hay grabación en marcha, con el cerrojo cogido el menor tiempo
/// posible: mientras el gancho no vuelve, Windows tiene parada la entrada de todo el
/// sistema, y si tarda demasiado lo desinstala sin avisar.
fn con_grabacion(f: impl FnOnce(&mut EnCurso, u64)) {
    let Ok(mut guarda) = GRABANDO.lock() else { return };
    let Some(en) = guarda.as_mut() else { return };
    let ahora = en.reloj.elapsed().as_millis() as u64;
    f(en, ahora);
}

/// Instala los ganchos y se queda bombeando mensajes hasta que le llegue un `WM_QUIT`.
///
/// El bucle no es opcional ni es una formalidad: es la única forma de que Windows entregue
/// los eventos de un gancho de bajo nivel.
fn hilo_de_ganchos(aviso: mpsc::Sender<u32>) {
    unsafe {
        let teclado = SetWindowsHookExW(WH_KEYBOARD_LL, Some(gancho_teclado), None, 0);
        let raton = SetWindowsHookExW(WH_MOUSE_LL, Some(gancho_raton), None, 0);

        if let Err(e) = &teclado {
            eprintln!("[grabadora] no se pudo instalar el gancho del teclado: {e}");
        }
        if let Err(e) = &raton {
            eprintln!("[grabadora] no se pudo instalar el gancho del ratón: {e}");
        }

        // El aviso va aunque la instalación falle: quien espera al otro lado se quedaría
        // colgado para siempre esperando un identificador que no va a llegar.
        let _ = aviso.send(GetCurrentThreadId());

        let mut msg = MSG::default();
        // `GetMessageW` devuelve -1 si algo va mal, y con `as_bool()` eso sería `true` y un
        // bucle infinito girando en vacío.
        while GetMessageW(&mut msg, None, 0, 0).0 > 0 {}

        if let Ok(h) = teclado {
            let _ = UnhookWindowsHookEx(h);
        }
        if let Ok(h) = raton {
            let _ = UnhookWindowsHookEx(h);
        }
    }
}

/// Arranca los ganchos si no estaban. Se instalan al empezar a grabar y se quitan al
/// parar, no al abrir la app: un gancho de bajo nivel puesto todo el rato mete a este
/// proceso en el camino de **cada** pulsación del equipo, y no hace falta para nada
/// mientras no se está grabando.
fn arrancar_ganchos() {
    let Ok(mut guarda) = GANCHOS.lock() else { return };
    if guarda.is_some() {
        return;
    }

    let (tx, rx) = mpsc::channel();
    let hilo = thread::spawn(move || hilo_de_ganchos(tx));

    // Sin identificador de hilo no habría forma de pararlo después.
    match rx.recv_timeout(Duration::from_secs(2)) {
        Ok(tid) => *guarda = Some(Ganchos { hilo, tid }),
        Err(e) => eprintln!("[grabadora] el hilo de los ganchos no arrancó: {e}"),
    }
}

fn parar_ganchos() {
    let Ok(mut guarda) = GANCHOS.lock() else { return };
    let Some(ganchos) = guarda.take() else { return };

    // `WM_QUIT` al hilo: es lo que saca a `GetMessageW` de su bucle y deja que desinstale
    // los ganchos antes de morir.
    unsafe {
        let _ = PostThreadMessageW(ganchos.tid, WM_QUIT, WPARAM(0), LPARAM(0));
    }
    let _ = ganchos.hilo.join();
}

// --- Reproducción ---

/// Lo que la grabación dejó hundido. Suelta en `Drop` a propósito: es el equivalente del
/// `finally` de Electron y cubre también el caso de que la reproducción muera por pánico,
/// que es justo cuando peor viene quedarse con el Control pulsado.
#[derive(Default)]
struct Pulsadas {
    teclas: HashSet<u32>,
    botones: HashSet<u32>,
}

impl Pulsadas {
    fn anotar(&mut self, ev: &Evento) {
        match *ev {
            Evento::TeclaAbajo { c, .. } => {
                self.teclas.insert(c);
            }
            Evento::TeclaArriba { c, .. } => {
                self.teclas.remove(&c);
            }
            Evento::BotonAbajo { b, .. } => {
                self.botones.insert(b);
            }
            Evento::BotonArriba { b, .. } => {
                self.botones.remove(&b);
            }
            _ => {}
        }
    }
}

impl Drop for Pulsadas {
    fn drop(&mut self) {
        for c in self.teclas.drain() {
            if let Some(tecla) = uiohook_a_tecla(c) {
                crate::macros::enviar(&[crate::macros::evento_tecla(tecla, true)]);
            }
        }
        for b in self.botones.drain() {
            if let Some((_, arriba)) = banderas_de_boton(b) {
                crate::macros::enviar(&[crate::macros::evento_raton(arriba)]);
            }
        }
    }
}

/// Los botones laterales no tienen traducción y se quedan sin reproducir.
///
/// Electron los pasaba por `BUTTON_TO_NUT[b] || 'LEFT'` y acababa haciendo un clic
/// **izquierdo** donde el usuario había pulsado el de «atrás». Un clic izquierdo que nadie
/// pidió cae sobre lo que haya debajo del puntero; no hacer nada, no.
fn banderas_de_boton(b: u32) -> Option<(MOUSE_EVENT_FLAGS, MOUSE_EVENT_FLAGS)> {
    match b {
        1 => Some((MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP)),
        2 => Some((MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP)),
        3 => Some((MOUSEEVENTF_MIDDLEDOWN, MOUSEEVENTF_MIDDLEUP)),
        _ => None,
    }
}

fn aplicar(ev: &Evento) {
    match *ev {
        // Coordenadas físicas del escritorio virtual, el mismo espacio del que salen al
        // grabarlas y el que usa el paso `mouse_position` de una secuencia.
        Evento::Mover { x, y, .. } => {
            let _ = unsafe { SetCursorPos(x, y) };
        }

        Evento::BotonAbajo { b, x, y, .. } | Evento::BotonArriba { b, x, y, .. } => {
            let Some((abajo, arriba)) = banderas_de_boton(b) else { return };
            // El puntero se recoloca antes de pulsar: el movimiento que llevaba hasta aquí
            // pudo caer en el filtro de los 3 px y entonces el clic iría a donde el ratón
            // se hubiera quedado.
            let _ = unsafe { SetCursorPos(x, y) };
            let flags = if matches!(ev, Evento::BotonAbajo { .. }) { abajo } else { arriba };
            crate::macros::enviar(&[crate::macros::evento_raton(flags)]);
        }

        Evento::TeclaAbajo { c, .. } | Evento::TeclaArriba { c, .. } => {
            // Un código que no está en la tabla se salta, igual que hacía Electron cuando
            // nut-js no sabía traducirlo.
            let Some(tecla) = uiohook_a_tecla(c) else { return };
            let soltar = matches!(ev, Evento::TeclaArriba { .. });
            crate::macros::enviar(&[crate::macros::evento_tecla(tecla, soltar)]);
        }

        Evento::Rueda { r, d, .. } => {
            // Un giro de cero se trata como una muesca hacia abajo, que es lo que acababa
            // haciendo Electron con su `Math.abs(r) || 1`.
            let muescas = if r == 0 { 1 } else { r };
            let flags = if d == 4 { MOUSEEVENTF_HWHEEL } else { MOUSEEVENTF_WHEEL };
            crate::macros::enviar(&[crate::macros::evento_raton_con_datos(
                flags,
                -muescas * RUEDA_DELTA,
            )]);
        }
    }
}

/// Espera troceada. `false` si la sesión se ha parado mientras tanto.
fn dormir_vigilando(sesion: &Sesion, ms: u64) -> bool {
    let mut restante = ms;
    while restante > 0 {
        if sesion.parada.load(Ordering::SeqCst) {
            return false;
        }
        let paso = restante.min(TROZO_DE_ESPERA_MS);
        thread::sleep(Duration::from_millis(paso));
        restante -= paso;
    }
    !sesion.parada.load(Ordering::SeqCst)
}

fn reproducir(app: AppHandle, sesion: Arc<Sesion>, eventos: Vec<Evento>, velocidad: f64) {
    let mut pulsadas = Pulsadas::default();

    loop {
        // El objetivo de cada evento se mide contra el arranque de **esta** pasada, no
        // sumando esperas: así un evento que llega tarde no arrastra a los siguientes.
        let inicio = Instant::now();
        for ev in &eventos {
            if sesion.parada.load(Ordering::SeqCst) {
                break;
            }
            let transcurrido = inicio.elapsed().as_millis() as u64;
            if !dormir_vigilando(&sesion, espera_ms(ev.t(), velocidad, transcurrido)) {
                break;
            }
            pulsadas.anotar(ev);
            aplicar(ev);
        }

        // `loop` y `hold` repiten hasta que alguien las corte; cualquier otro modo, y
        // `once` entre ellos, da una sola pasada.
        if sesion.parada.load(Ordering::SeqCst) || sesion.modo != "loop" && sesion.modo != "hold" {
            break;
        }
    }

    // Antes que nada, y antes de avisar: si el aviso repinta la tecla y todavía hay un
    // Control hundido, el usuario ve «listo» con el equipo atascado.
    drop(pulsadas);

    let ultima = {
        let Ok(mut guarda) = REPRO.lock() else { return };
        // Solo se limpia si sigue siendo esta: otra reproducción pudo entrar ya.
        let mia = guarda.as_ref().is_some_and(|s| Arc::ptr_eq(s, &sesion));
        if mia {
            *guarda = None;
            REPRODUCIENDO.store(false, Ordering::SeqCst);
        }
        mia
    };

    if ultima && !sesion.silenciosa.load(Ordering::SeqCst) {
        emitir(&app, sesion.id, "idle");
    }
}

fn parar_reproduccion(silenciosa: bool) {
    let Ok(guarda) = REPRO.lock() else { return };
    let Some(sesion) = guarda.as_ref() else { return };
    sesion.silenciosa.store(silenciosa, Ordering::SeqCst);
    sesion.parada.store(true, Ordering::SeqCst);
}

fn id_reproduciendo() -> Option<u32> {
    REPRO.lock().ok()?.as_ref().map(|s| s.id)
}

fn modo_reproduciendo() -> Option<String> {
    REPRO.lock().ok()?.as_ref().map(|s| s.modo.clone())
}

fn grabando() -> bool {
    GRABANDO.lock().map(|g| g.is_some()).unwrap_or(false)
}

// --- Guardado ---

/// Deja los eventos en la macro de la configuración del PC.
///
/// Se manda el array `macros` entero porque `orby_core::config::merge` sustituye los
/// arrays enteros: mandar solo la macro tocada borraría las demás.
fn guardar(id: u32, eventos: &[Evento]) {
    let config = crate::config::leer();
    let Some(lista) = config.get("macros").and_then(Value::as_array) else { return };

    let nuevos = serde_json::to_value(eventos).unwrap_or_else(|_| Value::Array(Vec::new()));
    let macros: Vec<Value> = lista
        .iter()
        .map(|m| {
            if m.get("id").and_then(Value::as_u64) != Some(id as u64) {
                return m.clone();
            }
            let mut copia = m.as_object().cloned().unwrap_or_else(Map::new);
            copia.insert("events".into(), nuevos.clone());
            Value::Object(copia)
        })
        .collect();

    crate::config::config_set(json!({ "macros": macros }));
}

// --- Las tres cosas que hace la tecla ---

/// SUPER + la tecla de una grabación: tira lo grabado y la deja lista para volver a
/// grabar. El id que se avisa es el de la grabación (`target`), no el de la tecla de
/// borrado: es la otra la que tiene que repintarse.
pub fn reiniciar(app: &AppHandle, macro_: &Value) {
    let Some(objetivo) = macro_.get("target").and_then(Value::as_u64) else {
        eprintln!("[grabadora] una tecla de borrado sin «target» no sabe qué borrar");
        return;
    };
    let objetivo = objetivo as u32;

    if grabando() {
        let _ = terminar_grabacion();
    }
    parar_reproduccion(true);
    guardar(objetivo, &[]);
    emitir(app, objetivo, "reset");
}

pub fn alternar(app: &AppHandle, id: u32, macro_: &Value) {
    // Grabando: para y guarda, pase lo que pase. Se guarda en la tecla que se acaba de
    // pulsar, que es lo que hacía Electron aunque la grabación la hubiera empezado otra.
    if grabando() {
        let eventos = terminar_grabacion();
        guardar(id, &eventos);
        emitir(app, id, if eventos.is_empty() { "empty" } else { "saved" });
        return;
    }

    // Reproduciéndose esta misma: la segunda pulsación la corta. El aviso de `idle` lo da
    // el hilo de la reproducción al salir, no aquí.
    if id_reproduciendo() == Some(id) {
        parar_reproduccion(false);
        return;
    }

    let eventos = leer_eventos(macro_.get("events").unwrap_or(&Value::Null));
    if !eventos.is_empty() {
        let modo = macro_
            .get("mode")
            .and_then(Value::as_str)
            .unwrap_or(MODO_POR_DEFECTO)
            .to_string();
        let velocidad = macro_
            .get("speed")
            .and_then(Value::as_f64)
            .filter(|v| *v > 0.0)
            .unwrap_or(VELOCIDAD_POR_DEFECTO);

        emitir(app, id, "playing");
        arrancar_reproduccion(app, id, modo, eventos, velocidad);
        return;
    }

    empezar_grabacion();
    emitir(app, id, "recording");
}

fn arrancar_reproduccion(
    app: &AppHandle,
    id: u32,
    modo: String,
    eventos: Vec<Evento>,
    velocidad: f64,
) {
    let sesion = Arc::new(Sesion {
        id,
        modo,
        parada: AtomicBool::new(false),
        silenciosa: AtomicBool::new(false),
    });

    {
        let Ok(mut guarda) = REPRO.lock() else { return };
        *guarda = Some(sesion.clone());
        REPRODUCIENDO.store(true, Ordering::SeqCst);
    }

    let app = app.clone();
    thread::spawn(move || reproducir(app, sesion, eventos, velocidad));
}

fn empezar_grabacion() {
    arrancar_ganchos();
    if let Ok(mut guarda) = GRABANDO.lock() {
        *guarda = Some(EnCurso { grabador: Grabador::nuevo(0), reloj: Instant::now() });
    }
}

fn terminar_grabacion() -> Vec<Evento> {
    // El cerrojo se suelta **antes** de parar los ganchos: pararlos espera a que el hilo
    // salga, y si ese hilo está dentro de un gancho esperando este mismo cerrojo, las dos
    // partes se quedan esperándose para siempre.
    let en_curso = GRABANDO.lock().ok().and_then(|mut g| g.take());
    parar_ganchos();
    en_curso.map(|en| en.grabador.terminar()).unwrap_or_default()
}

/// En modo `hold` la reproducción dura lo que la tecla siga pulsada, y el firmware no
/// manda un segundo `MACRO:` al soltarla: la suelta se ve por la telemetría normal de
/// teclas. Cualquier suelta vale, igual que en Electron.
pub fn quizas_soltar_hold(linea: &str) {
    if linea.starts_with("KEY_EV:")
        && linea.ends_with(":0")
        && modo_reproduciendo().as_deref() == Some("hold")
    {
        parar_reproduccion(false);
    }
}

/// Al cerrar la app: soltar lo que estuviera hundido y quitar los ganchos.
///
/// Salir a mitad de una reproducción sin esto deja el Control pulsado y el equipo
/// inservible, y ahí ya no hay ninguna app viva que pueda arreglarlo.
pub fn apagar() {
    parar_reproduccion(true);

    // Se espera a que el hilo de la reproducción suelte de verdad. Es corto (como mucho un
    // trozo de espera) y es la diferencia entre soltar las teclas y no soltarlas.
    let plazo = Instant::now();
    while REPRODUCIENDO.load(Ordering::SeqCst) && plazo.elapsed() < Duration::from_millis(500) {
        thread::sleep(Duration::from_millis(TROZO_DE_ESPERA_MS));
    }

    if let Ok(mut guarda) = GRABANDO.lock() {
        // Lo que se estuviera grabando se pierde, igual que en Electron: no hay a quién
        // preguntarle en qué tecla guardarlo.
        *guarda = None;
    }
    parar_ganchos();
}

// --- Comandos ---

/// Es el mismo camino que la pulsación de la tecla, para poder empezar y parar una
/// grabación desde el editor sin tener el teclado delante.
#[tauri::command]
pub fn recorder_toggle(app: AppHandle, id: u32) -> Value {
    crate::macros::disparar(&app, id);
    json!({ "recording": grabando(), "playing": id_reproduciendo() == Some(id) })
}

/// Solo para la reproducción. Una grabación en marcha **no** se corta desde aquí: la
/// única forma de cerrarla es la pulsación que la guarda.
#[tauri::command]
pub fn recorder_stop() -> bool {
    parar_reproduccion(false);
    true
}

#[tauri::command]
pub fn recorder_status() -> Value {
    json!({ "recording": grabando(), "playingId": id_reproduciendo() })
}
