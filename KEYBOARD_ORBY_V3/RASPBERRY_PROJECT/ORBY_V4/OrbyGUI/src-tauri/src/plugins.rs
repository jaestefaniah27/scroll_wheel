// Anfitrión de complementos declarativos.
//
// Un complemento ya no es un programa de Node sin caja de arena (eso era la API 1 de
// Electron): un complemento de tipo `http` es un `plugin.json` y nada más. Lo que decide
// QUÉ mandar (el manifiesto, las plantillas, el agrupado de giros) vive en
// `orby_core::plugins`, donde se puede probar sin red. Aquí solo está lo que necesita un
// socket o un disco: cargar la carpeta, mandar la petición de verdad y la instalación.
//
// Los complementos de tipo `process` (Tarea 11, todavía sin empezar) cargan y se ven en
// la lista, pero no hay quién los ejecute: `ejecutar_paso`, `plugins_test` y `plugins_read`
// devuelven un error claro en vez de intentarlo.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use orby_core::plugins::{construye, construye_test, render, Agrupador, Contexto, Manifiesto, Tipo};
use serde_json::{json, Value};
use tauri::{AppHandle, Manager};
use tauri_plugin_dialog::DialogExt;

/// Cada cuánto se mira si hay giros agrupados que ya han vencido su plazo. No hace falta
/// que sea fino: el plazo típico del manifiesto son 120 ms.
const TIC_AGRUPADOR_MS: u64 = 15;

struct Cargado {
    dir: PathBuf,
    resultado: Result<Manifiesto, String>,
}

struct Estado {
    cargados: HashMap<String, Cargado>,
    agrupador: Agrupador,
}

/// `%APPDATA%\OrbyGUI\plugins`, la misma carpeta que usaba Electron: así un complemento
/// instalado con una app lo ve la otra durante la migración.
fn carpeta_base() -> PathBuf {
    let appdata = std::env::var_os("APPDATA").map(PathBuf::from).unwrap_or_default();
    appdata.join("OrbyGUI").join("plugins")
}

fn leer_manifiesto(dir: &Path) -> Result<Manifiesto, String> {
    let texto = std::fs::read_to_string(dir.join("plugin.json"))
        .map_err(|_| "no se encuentra plugin.json".to_string())?;
    Manifiesto::desde_json(&texto).map_err(|e| e.to_string())
}

fn escanear() -> HashMap<String, Cargado> {
    let base = carpeta_base();
    let _ = std::fs::create_dir_all(&base);

    let mut cargados = HashMap::new();
    let Ok(entradas) = std::fs::read_dir(&base) else { return cargados };

    for entrada in entradas.flatten() {
        if !entrada.file_type().map(|t| t.is_dir()).unwrap_or(false) {
            continue;
        }
        let dir = entrada.path();
        let resultado = leer_manifiesto(&dir);
        // Sin manifiesto no hay id que leer: se usa el nombre de la carpeta, que es lo
        // más parecido a lo que el usuario reconocerá en Ajustes.
        let id = match &resultado {
            Ok(m) => m.id.clone(),
            Err(_) => dir.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default(),
        };
        if !id.is_empty() {
            cargados.insert(id, Cargado { dir, resultado });
        }
    }
    cargados
}

/// Monta el estado y lanza el hilo del agrupador. Se llama desde el `setup`, igual que
/// `serial::arrancar`.
pub fn iniciar(app: &AppHandle) {
    app.manage(Mutex::new(Estado { cargados: escanear(), agrupador: Agrupador::new() }));
    let app = app.clone();
    std::thread::spawn(move || hilo_agrupador(app));
}

fn hilo_agrupador(app: AppHandle) {
    let mut anterior = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(TIC_AGRUPADOR_MS));
        let ahora = Instant::now();
        let delta = ahora.duration_since(anterior).as_millis() as u64;
        anterior = ahora;

        let envios = {
            let estado = app.state::<Mutex<Estado>>();
            let mut e = estado.lock().unwrap();
            e.agrupador.avanzar(delta)
        };

        for envio in envios {
            let app = app.clone();
            std::thread::spawn(move || {
                ejecutar_run(&app, &envio.plugin, &envio.op, envio.valor);

                let estado = app.state::<Mutex<Estado>>();
                let mut e = estado.lock().unwrap();
                if let Some(c) = e.cargados.get(&envio.plugin) {
                    if let Ok(m) = &c.resultado {
                        if let Some(ms) = coalesce_ms(m, &envio.op) {
                            e.agrupador.terminado(&envio.plugin, &envio.op, ms);
                        }
                    }
                }
            });
        }
    }
}

/// El `coalesce.ms` de una acción, leído del manifiesto crudo: `Manifiesto::Accion` no lo
/// guarda porque es una decisión del motor de agrupado, no del descriptor que ve el
/// renderer.
fn coalesce_ms(m: &Manifiesto, op: &str) -> Option<u64> {
    m.crudo
        .get("actions")?
        .as_array()?
        .iter()
        .find(|a| a.get("op").and_then(Value::as_str) == Some(op))?
        .get("coalesce")?
        .get("ms")?
        .as_u64()
}

fn manifiesto_de(app: &AppHandle, id: &str) -> Option<Manifiesto> {
    let estado = app.state::<Mutex<Estado>>();
    let e = estado.lock().unwrap();
    e.cargados.get(id)?.resultado.as_ref().ok().cloned()
}

fn habilitado(config: &Value, id: &str) -> bool {
    // Sin nada guardado se considera activo: un complemento recién instalado no debería
    // necesitar un segundo clic para empezar a funcionar.
    config
        .get("plugins")
        .and_then(|p| p.get(id))
        .and_then(|e| e.get("enabled"))
        .and_then(Value::as_bool)
        .unwrap_or(true)
}

/// Ajustes con los que correr: los valores por defecto que declara el manifiesto, con lo
/// que el usuario haya guardado por encima.
fn ajustes_de(config: &Value, id: &str, m: &Manifiesto) -> Value {
    let mut mapa = serde_json::Map::new();
    if let Some(campos) = m.crudo.get("settings").and_then(|s| s.get("fields")).and_then(Value::as_array) {
        for c in campos {
            if let (Some(k), Some(d)) = (c.get("key").and_then(Value::as_str), c.get("default")) {
                mapa.insert(k.to_string(), d.clone());
            }
        }
    }
    if let Some(guardados) =
        config.get("plugins").and_then(|p| p.get(id)).and_then(|e| e.get("settings")).and_then(Value::as_object)
    {
        for (k, v) in guardados {
            mapa.insert(k.clone(), v.clone());
        }
    }
    Value::Object(mapa)
}

fn descriptor_de(id: &str, cargado: &Cargado, config: &Value) -> Value {
    match &cargado.resultado {
        Ok(m) => m.descriptor(habilitado(config, id), &ajustes_de(config, id, m)),
        Err(e) => Manifiesto::descriptor_roto(id, e),
    }
}

// --- Mandar la petición de verdad -------------------------------------------------------

/// Ejecuta una `Peticion` ya montada, con sus reintentos. Devuelve el cuerpo interpretado
/// como JSON (o `Value::Null` si la respuesta no lo era: no todos los cacharros contestan
/// JSON a una escritura, y no tener nada que leer no es un error).
fn ejecutar(p: &orby_core::plugins::Peticion) -> Result<Value, String> {
    let cliente = reqwest::blocking::Client::builder()
        .timeout(Duration::from_millis(p.timeout_ms))
        .build()
        .map_err(|e| e.to_string())?;
    let metodo = reqwest::Method::from_bytes(p.metodo.as_bytes()).unwrap_or(reqwest::Method::GET);

    let mut ultimo_error = String::new();
    // `reintentos` son los que hay ADEMÁS de la primera, así que el bucle da uno más.
    for _ in 0..=p.reintentos {
        match cliente.request(metodo.clone(), &p.url).send() {
            Ok(resp) if resp.status().is_success() => {
                let texto = resp.text().unwrap_or_default();
                return Ok(serde_json::from_str(&texto).unwrap_or(Value::Null));
            }
            Ok(resp) => ultimo_error = format!("respondió {}", resp.status()),
            Err(e) => ultimo_error = e.to_string(),
        }
    }
    Err(ultimo_error)
}

/// La acción de una tecla o de un giro ya agrupado: se dispara y no importa el resultado,
/// solo que quede constancia en el registro si algo va mal.
fn ejecutar_run(app: &AppHandle, id: &str, op: &str, valor: f64) {
    let Some(m) = manifiesto_de(app, id) else {
        eprintln!("[plugins] «{id}» no está instalado");
        return;
    };
    if !habilitado(&crate::config::leer(), id) {
        return; // desactivado a propósito: silencio, no error
    }
    if m.tipo != Tipo::Http {
        eprintln!("[plugins] «{id}»: los complementos de proceso todavía no se ejecutan");
        return;
    }

    let config = crate::config::leer();
    let ajustes = ajustes_de(&config, id, &m);
    let peticion = match construye(&m, op, Some(valor), &ajustes) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("[plugins] «{id}», acción «{op}»: {e}");
            return;
        }
    };
    if let Err(e) = ejecutar(&peticion) {
        eprintln!("[plugins] «{id}», acción «{op}»: {e}");
    }
}

/// Entrada desde `macros.rs` para el paso `{ type: "plugin", plugin, op, value }`. Pasa
/// por el agrupador: si la acción declara `coalesce`, puede quedar en espera en vez de
/// mandarse ya.
pub fn disparar_paso(app: &AppHandle, plugin: &str, op: &str, valor: f64) {
    let envio_inmediato = {
        let estado = app.state::<Mutex<Estado>>();
        let mut e = estado.lock().unwrap();
        let Some(c) = e.cargados.get(plugin) else {
            eprintln!("[plugins] «{plugin}» no está instalado");
            return;
        };
        let ms = c.resultado.as_ref().ok().and_then(|m| coalesce_ms(m, op));
        e.agrupador.empujar(plugin, op, valor, ms)
    };

    if let Some(envio) = envio_inmediato {
        let app = app.clone();
        std::thread::spawn(move || ejecutar_run(&app, &envio.plugin, &envio.op, envio.valor));
    }
}

// --- Instalar / desinstalar --------------------------------------------------------------

/// Protege contra un `.zip` con una entrada `../../etc/algo`: sin esto, descomprimir un
/// complemento hostil podría escribir fuera de la carpeta temporal.
fn ruta_seria(base: &Path, relativa: &Path) -> Option<PathBuf> {
    if relativa.components().any(|c| matches!(c, std::path::Component::ParentDir)) {
        return None;
    }
    Some(base.join(relativa))
}

fn descomprimir(origen: &Path, destino: &Path) -> Result<(), String> {
    let fichero = std::fs::File::open(origen).map_err(|e| e.to_string())?;
    let mut zip = zip::ZipArchive::new(fichero).map_err(|e| e.to_string())?;

    for i in 0..zip.len() {
        let mut entrada = zip.by_index(i).map_err(|e| e.to_string())?;
        let Some(relativa) = entrada.enclosed_name() else { continue };
        let Some(ruta) = ruta_seria(destino, &relativa) else { continue };

        if entrada.is_dir() {
            std::fs::create_dir_all(&ruta).map_err(|e| e.to_string())?;
        } else {
            if let Some(padre) = ruta.parent() {
                std::fs::create_dir_all(padre).map_err(|e| e.to_string())?;
            }
            let mut destino_f = std::fs::File::create(&ruta).map_err(|e| e.to_string())?;
            std::io::copy(&mut entrada, &mut destino_f).map_err(|e| e.to_string())?;
        }
    }
    Ok(())
}

/// Un `.zip` puede traer el `plugin.json` en la raíz o dentro de una única carpeta (que es
/// lo que sale al comprimir la carpeta del complemento con el explorador). Se admiten los
/// dos.
fn buscar_manifiesto(dir: &Path) -> PathBuf {
    if dir.join("plugin.json").is_file() {
        return dir.to_path_buf();
    }
    if let Ok(entradas) = std::fs::read_dir(dir) {
        let subcarpetas: Vec<PathBuf> = entradas
            .flatten()
            .filter(|e| e.file_type().map(|t| t.is_dir()).unwrap_or(false))
            .map(|e| e.path())
            .collect();
        if let [unica] = subcarpetas.as_slice() {
            if unica.join("plugin.json").is_file() {
                return unica.clone();
            }
        }
    }
    dir.to_path_buf()
}

fn copiar_carpeta(origen: &Path, destino: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(destino)?;
    for entrada in std::fs::read_dir(origen)?.flatten() {
        let destino_hijo = destino.join(entrada.file_name());
        if entrada.file_type()?.is_dir() {
            copiar_carpeta(&entrada.path(), &destino_hijo)?;
        } else {
            std::fs::copy(entrada.path(), &destino_hijo)?;
        }
    }
    Ok(())
}

fn instalar_desde(app: &AppHandle, origen: &Path) -> Value {
    let temp = std::env::temp_dir().join(format!("orby-plugin-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&temp);
    if let Err(e) = std::fs::create_dir_all(&temp) {
        return json!({ "ok": false, "error": e.to_string() });
    }
    if let Err(e) = descomprimir(origen, &temp) {
        let _ = std::fs::remove_dir_all(&temp);
        return json!({ "ok": false, "error": format!("no se pudo descomprimir: {e}") });
    }

    let dir_manifiesto = buscar_manifiesto(&temp);
    let manifiesto = match leer_manifiesto(&dir_manifiesto) {
        Ok(m) => m,
        Err(e) => {
            let _ = std::fs::remove_dir_all(&temp);
            return json!({ "ok": false, "error": e });
        }
    };

    let destino = carpeta_base().join(&manifiesto.id);
    let _ = std::fs::remove_dir_all(&destino);
    if let Err(e) = copiar_carpeta(&dir_manifiesto, &destino) {
        let _ = std::fs::remove_dir_all(&temp);
        return json!({ "ok": false, "error": e.to_string() });
    }
    let _ = std::fs::remove_dir_all(&temp);

    let id = manifiesto.id.clone();
    {
        let estado = app.state::<Mutex<Estado>>();
        let mut e = estado.lock().unwrap();
        e.cargados.insert(id.clone(), Cargado { dir: destino, resultado: Ok(manifiesto) });
    }
    // Instalarlo es querer usarlo: si estaba desactivado de una vez anterior, vuelve a
    // activarse.
    crate::config::config_set(json!({ "plugins": { id.clone(): { "enabled": true } } }));

    let config = crate::config::leer();
    let estado = app.state::<Mutex<Estado>>();
    let e = estado.lock().unwrap();
    let cargado = e.cargados.get(&id).unwrap();
    json!({ "ok": true, "plugin": descriptor_de(&id, cargado, &config) })
}

// --- Comandos ------------------------------------------------------------------------

#[tauri::command]
pub fn plugins_list(app: AppHandle) -> Vec<Value> {
    let config = crate::config::leer();
    let estado = app.state::<Mutex<Estado>>();
    let e = estado.lock().unwrap();

    let mut lista: Vec<Value> = e.cargados.iter().map(|(id, c)| descriptor_de(id, c, &config)).collect();
    lista.sort_by(|a, b| {
        crate::apps::comparar_en_espanol(a["name"].as_str().unwrap_or(""), b["name"].as_str().unwrap_or(""))
    });
    lista
}

/// Sin diálogo de zip-o-carpeta como en Electron: un complemento `http` no ejecuta nada,
/// así que no hay permisos que pedir, y la única forma de distribuirlo es un `.zip`.
#[tauri::command]
pub async fn plugins_install(app: AppHandle) -> Value {
    let dialogo = app
        .dialog()
        .file()
        .set_title("Instalar complemento")
        .add_filter("Complemento de Orby", &["zip", "orbyplugin"]);

    let (tx, mut rx) = tauri::async_runtime::channel(1);
    dialogo.pick_file(move |elegida| {
        let _ = tx.blocking_send(elegida);
    });

    let Some(origen) = rx.recv().await.flatten().and_then(|f| f.into_path().ok()) else {
        return json!({ "ok": false, "canceled": true });
    };

    let app2 = app.clone();
    match tauri::async_runtime::spawn_blocking(move || instalar_desde(&app2, &origen)).await {
        Ok(v) => v,
        Err(e) => json!({ "ok": false, "error": e.to_string() }),
    }
}

#[tauri::command]
pub fn plugins_uninstall(app: AppHandle, id: String) -> Value {
    let dir = {
        let estado = app.state::<Mutex<Estado>>();
        let mut e = estado.lock().unwrap();
        match e.cargados.remove(&id) {
            Some(c) => c.dir,
            None => return json!({ "ok": false, "error": "no está instalado" }),
        }
    };
    // El estado guardado (activo, ajustes) se queda: reinstalarlo recupera la dirección
    // que costó encontrar en vez de empezar de cero.
    match std::fs::remove_dir_all(&dir) {
        Ok(()) => json!({ "ok": true }),
        Err(e) => json!({ "ok": false, "error": e.to_string() }),
    }
}

#[tauri::command]
pub fn plugins_set_enabled(app: AppHandle, id: String, enabled: bool) -> Value {
    {
        let estado = app.state::<Mutex<Estado>>();
        if !estado.lock().unwrap().cargados.contains_key(&id) {
            return json!({ "ok": false, "error": "no está instalado" });
        }
    }
    crate::config::config_set(json!({ "plugins": { id: { "enabled": enabled } } }));
    json!({ "ok": true, "enabled": enabled })
}

#[tauri::command]
pub fn plugins_get_settings(app: AppHandle, id: String) -> Value {
    match manifiesto_de(&app, &id) {
        Some(m) => ajustes_de(&crate::config::leer(), &id, &m),
        None => Value::Object(Default::default()),
    }
}

#[tauri::command]
pub fn plugins_set_settings(app: AppHandle, id: String, patch: Value) -> Value {
    crate::config::config_set(json!({ "plugins": { id.clone(): { "settings": patch } } }));
    match manifiesto_de(&app, &id) {
        Some(m) => ajustes_de(&crate::config::leer(), &id, &m),
        None => Value::Object(Default::default()),
    }
}

#[tauri::command]
pub fn plugins_open_folder() -> bool {
    let carpeta = carpeta_base();
    let _ = std::fs::create_dir_all(&carpeta);
    std::process::Command::new("explorer").arg(&carpeta).spawn().is_ok()
}

#[tauri::command]
pub fn plugins_test(app: AppHandle, id: String, values: Option<Value>) -> Value {
    let Some(m) = manifiesto_de(&app, &id) else {
        return json!({ "ok": false, "error": "no está instalado" });
    };
    if m.tipo != Tipo::Http {
        return json!({ "ok": false, "error": "los complementos de proceso todavía no se ejecutan" });
    }

    let mut ajustes = ajustes_de(&crate::config::leer(), &id, &m);
    if let (Value::Object(base), Some(Value::Object(extra))) = (&mut ajustes, values) {
        for (k, v) in extra {
            base.insert(k, v);
        }
    }

    let peticion = match construye_test(&m, &ajustes) {
        Ok(p) => p,
        Err(e) => return json!({ "ok": false, "error": e.to_string() }),
    };
    match ejecutar(&peticion) {
        Ok(cuerpo) => {
            let plantilla =
                m.crudo.get("test").and_then(|t| t.get("detail")).and_then(Value::as_str).unwrap_or("Responde");
            let ctx = Contexto { response: Some(&cuerpo), ..Default::default() };
            let detalle = render(plantilla, &ctx).unwrap_or_else(|_| "Responde".to_string());
            json!({ "ok": true, "detail": detalle })
        }
        Err(e) => json!({ "ok": false, "error": e }),
    }
}

/// Para el visor de una tecla (`src/live-oled.js`): pide el valor actual sin tocar nada.
/// A diferencia de `test`, esto lo llama un temporizador cada par de segundos, así que sin
/// conexión o desactivado se calla en vez de devolver un error que se acumularía en algún
/// sitio.
#[tauri::command]
pub fn plugins_read(app: AppHandle, id: String, op: String) -> Value {
    let no = json!({ "ok": false });

    let Some(m) = manifiesto_de(&app, &id) else { return no };
    if m.tipo != Tipo::Http || !habilitado(&crate::config::leer(), &id) {
        return no;
    }

    let ajustes = ajustes_de(&crate::config::leer(), &id, &m);
    let Ok(peticion) = construye(&m, &op, None, &ajustes) else { return no };
    let Ok(cuerpo) = ejecutar(&peticion) else { return no };

    let Some(plantilla_valor) = m
        .crudo
        .get("views")
        .and_then(Value::as_array)
        .and_then(|vs| vs.iter().find(|v| v.get("op").and_then(Value::as_str) == Some(op.as_str())))
        .and_then(|v| v.get("value"))
        .and_then(Value::as_str)
    else {
        return no;
    };

    let ctx = Contexto { response: Some(&cuerpo), ..Default::default() };
    match render(plantilla_valor, &ctx).ok().and_then(|t| t.parse::<f64>().ok()) {
        Some(v) => json!({ "ok": true, "value": v }),
        None => no,
    }
}

// --- Pruebas de la ejecución HTTP, contra un servidor de mentira ---------------------

#[cfg(test)]
mod tests {
    use super::ejecutar;
    use orby_core::plugins::Peticion;
    use std::io::{Read as _, Write as _};
    use std::net::TcpListener;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    /// Un servidor que contesta las respuestas dadas, una por conexión, y en orden.
    fn servidor_fijo(respuestas: Vec<String>) -> (String, Arc<AtomicU32>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let puerto = listener.local_addr().unwrap().port();
        let peticiones = Arc::new(AtomicU32::new(0));
        let contador = peticiones.clone();

        std::thread::spawn(move || {
            for cuerpo in respuestas {
                let Ok((mut socket, _)) = listener.accept() else { break };
                contador.fetch_add(1, Ordering::SeqCst);
                let mut buf = [0u8; 1024];
                let _ = socket.read(&mut buf); // se descarta: aquí no hace falta mirar la petición
                let _ = socket.write_all(cuerpo.as_bytes());
            }
        });

        (format!("http://127.0.0.1:{puerto}"), peticiones)
    }

    fn peticion(url: &str, reintentos: u32) -> Peticion {
        Peticion { metodo: "GET".into(), url: url.into(), timeout_ms: 500, reintentos }
    }

    #[test]
    fn un_200_se_lee_como_json() {
        // `Content-Length` calculado del cuerpo real: declararlo a mano es la forma más
        // fácil de que este test falle por su cuenta y no por lo que prueba.
        let cuerpo_json = "{\"valor\":42}";
        let respuesta = format!(
            "HTTP/1.1 200 OK\r\nContent-Length: {}\r\n\r\n{}",
            cuerpo_json.len(),
            cuerpo_json
        );
        let (base, _) = servidor_fijo(vec![respuesta]);
        let cuerpo = ejecutar(&peticion(&base, 0)).unwrap();
        assert_eq!(cuerpo["valor"], 42);
    }

    #[test]
    fn un_500_se_reporta_como_error_y_no_como_panico() {
        let (base, _) =
            servidor_fijo(vec!["HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\n\r\n".into()]);
        assert!(ejecutar(&peticion(&base, 0)).is_err());
    }

    #[test]
    fn sin_reintentos_declarados_un_solo_intento() {
        // Es lo que garantiza que una acción (`run`) nunca reintenta: un incremento
        // repetido se aplicaría dos veces.
        let (base, contador) =
            servidor_fijo(vec!["HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\n\r\n".into()]);
        assert!(ejecutar(&peticion(&base, 0)).is_err());
        std::thread::sleep(Duration::from_millis(100));
        assert_eq!(contador.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn con_un_reintento_hace_como_mucho_dos_intentos() {
        let (base, contador) = servidor_fijo(vec![
            "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\n\r\n".into(),
            "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\n{}".into(),
        ]);
        let cuerpo = ejecutar(&peticion(&base, 1)).unwrap();
        assert!(cuerpo.is_object());
        assert_eq!(contador.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn el_tiempo_maximo_corta_de_verdad() {
        // Sin él, un cacharro desenchufado deja la ejecución colgada hasta que agote el
        // TCP: más de un minuto.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let puerto = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            // Acepta la conexión y no contesta nunca: el cacharro colgado.
            let _ = listener.accept();
            std::thread::sleep(Duration::from_secs(5));
        });

        let inicio = Instant::now();
        let err = ejecutar(&peticion(&format!("http://127.0.0.1:{puerto}"), 0)).unwrap_err();
        assert!(!err.is_empty());
        assert!(
            inicio.elapsed() < Duration::from_secs(2),
            "el timeout de 500 ms debería cortar mucho antes de los 5 s reales"
        );
    }
}
