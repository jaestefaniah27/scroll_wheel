// Actualizar la propia app, sola y sin preguntar.
//
// **Descarga e instala.** La primera versión de este fichero solo avisaba y abría la página
// de la release en el navegador, porque firmar las actualizaciones costaba un par de claves
// minisign y un `latest.json` publicado en cada release. Se hizo: la clave privada vive
// fuera del repositorio y la pública está en `tauri.conf.json`, así que aquí ya se puede
// usar `tauri-plugin-updater` de verdad. La regla que queda de aquello es que **una release
// de la app sin su `latest.json` y su `.sig` deja el actualizador mudo sin decir nada**; el
// procedimiento está en `docs/PUBLICACION.md`.
//
// El endpoint es `releases/latest/download/latest.json`, y eso es lo que **obliga a que las
// releases de firmware vayan como prerelease**: `latest` de GitHub no mira la etiqueta, mira
// la fecha, así que una `fw-v*` publicada como definitiva se convertiría en la última
// release del repositorio, no llevaría `latest.json` y el actualizador se quedaría buscando
// un fichero que no existe. Ya pasó una vez con el actualizador de Electron, está
// documentado en `ORBY_V4/docs/COMPATIBILIDAD.md`.
//
// Funciona también en desarrollo, al revés que el de Electron: aquel necesitaba un
// instalador que sustituir. Aquí la comprobación se hace igual; lo que no habrá es una
// versión publicada más alta que la de la rama.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::Duration;

use serde_json::{json, Value};
use tauri::{AppHandle, Emitter, Manager};
use tauri_plugin_updater::UpdaterExt;

/// La app puede pasarse semanas viva en la bandeja sin que nadie la reinicie, así que la
/// comprobación se repite. Mismo intervalo que tenía electron/main.js.
const INTERVALO_MS: u64 = 6 * 60 * 60 * 1000;

/// Margen de cortesía antes de la primera consulta: al arrancar hay cosas con más prisa
/// (el puerto serie, la ventana) y esto puede esperar.
const RETRASO_INICIAL_MS: u64 = 5_000;

/// Si el renderer tiene cambios de configuración sin escribir en la Flash del teclado.
///
/// Instalar significa cerrar el proceso, y el guardado automático es un temporizador de
/// 1500 ms en `store.js`: morirse dentro de esa ventana se lleva por delante una escritura
/// que nadie va a repetir, y el ajuste se queda solo en la RAM del teclado hasta que se
/// desenchufe. Es el único punto donde actualizar «en cuanto esté» puede perder datos, así
/// que la instalación —solo la instalación, la descarga va igual— espera a que se apague.
/// Lo pone al día `updater_ocupado`, desde `src/updater.js`.
static HAY_CAMBIOS_SIN_GUARDAR: AtomicBool = AtomicBool::new(false);

/// El usuario ha pulsado la insignia: instalar aunque haya cambios sin guardar. Lo lee la
/// espera de `consultar`, que es quien tiene el paquete descargado en la mano.
static INSTALAR_YA: AtomicBool = AtomicBool::new(false);

/// Cada cuánto se mira si ya se puede instalar. Corto a propósito: lo que se está esperando
/// es un guardado de segundo y medio, no una tarea larga.
const ESPERA_GUARDADO_MS: u64 = 3_000;

/// Solo una comprobación a la vez. Sin esto, el botón «Buscar actualizaciones» pulsado
/// mientras corre el repaso automático descargaría el mismo instalador dos veces y las dos
/// tareas pelearían por instalarlo.
static EN_MARCHA: AtomicBool = AtomicBool::new(false);

struct Compartido {
    estado: Value,
}

/// El estado que espera src/updater.js, con la misma forma que tenía el de Electron:
/// `checking` → `downloading` (con `percent`) → `downloaded`, y de ahí el proceso muere
/// para instalarse. `available` ya no se usa: era el estado del avisador que solo abría
/// la página, y ahora no hay nada que el usuario tenga que ir a buscar.
fn inicial(version: &str) -> Value {
    json!({
        "status": "idle",
        "version": version,
        "newVersion": null,
        "percent": 0,
        "error": null,
    })
}

pub fn iniciar(app: &AppHandle) {
    let version = app.package_info().version.to_string();
    app.manage(Mutex::new(Compartido { estado: inicial(&version) }));

    let handle = app.clone();
    tauri::async_runtime::spawn(async move {
        tokio::time::sleep(Duration::from_millis(RETRASO_INICIAL_MS)).await;
        loop {
            consultar(&handle).await;
            tokio::time::sleep(Duration::from_millis(INTERVALO_MS)).await;
        }
    });
}

/// Fusiona `patch` sobre el estado y avisa al renderer, igual que firmware.rs.
fn set(app: &AppHandle, patch: Value) -> Value {
    let estado = app.state::<Mutex<Compartido>>();
    let mut c = estado.lock().unwrap();

    if let (Value::Object(actual), Value::Object(nuevo)) = (&mut c.estado, patch) {
        for (k, v) in nuevo {
            actual.insert(k, v);
        }
    }

    let clon = c.estado.clone();
    let _ = app.emit("updater:state", clon.clone());
    clon
}

fn leer_estado(app: &AppHandle) -> Value {
    app.state::<Mutex<Compartido>>().lock().unwrap().estado.clone()
}

/// La consulta y, si hay algo, la descarga y la instalación enteras.
async fn consultar(app: &AppHandle) -> Value {
    // Ver EN_MARCHA. Quien llega segundo se va por donde vino: la que ya corre hace el
    // mismo trabajo y publica el mismo estado.
    if EN_MARCHA.swap(true, Ordering::SeqCst) {
        return leer_estado(app);
    }
    let resultado = consultar_de_verdad(app).await;
    EN_MARCHA.store(false, Ordering::SeqCst);
    resultado
}

async fn consultar_de_verdad(app: &AppHandle) -> Value {
    set(app, json!({ "status": "checking", "error": null }));

    let actualizacion = match app.updater() {
        Ok(u) => u.check().await,
        Err(e) => Err(e),
    };

    let actualizacion = match actualizacion {
        Ok(v) => v,
        Err(e) => {
            // El fallo que hay que saber leer: el endpoint es el `latest.json` de la
            // última release, y una release de firmware publicada SIN marcar como
            // prerelease se convierte en la última y no lleva ese fichero. El error de
            // red que sale entonces no lo dice, así que se dice aquí.
            return set(
                app,
                json!({
                    "status": "error",
                    "error": format!(
                        "No se pudo comprobar si hay versión nueva: {e}. Si acabas de \
                         publicar una release de firmware, comprueba que va marcada como \
                         prerelease."
                    ),
                }),
            )
        }
    };

    let Some(actualizacion) = actualizacion else {
        return set(app, json!({ "status": "idle", "newVersion": null, "percent": 0 }));
    };

    let nueva = actualizacion.version.clone();
    set(app, json!({ "status": "downloading", "newVersion": nueva, "percent": 0, "error": null }));

    let progreso = {
        let app = app.clone();
        let mut descargado: u64 = 0;
        move |trozo: usize, total: Option<u64>| {
            descargado += trozo as u64;
            if let Some(total) = total.filter(|t| *t > 0) {
                set(&app, json!({ "percent": descargado * 100 / total }));
            }
        }
    };

    // Descargar va siempre, y esperar es cosa solo de instalar: bajar unos megas de fondo no
    // molesta a nadie, y así cuando toque instalar no hay que volver a esperar la red.
    let paquete = match actualizacion.download(progreso, || {}).await {
        Ok(bytes) => bytes,
        Err(e) => {
            return set(
                app,
                json!({
                    "status": "error",
                    "error": format!("No se pudo descargar la versión {nueva}: {e}"),
                }),
            )
        }
    };

    set(app, json!({ "status": "downloaded", "percent": 100 }));

    // Instalar mata el proceso, y el guardado automático a Flash es un temporizador de
    // 1500 ms: morirse dentro de esa ventana pierde una escritura que nadie va a repetir.
    // Se espera aquí, con el paquete ya en memoria, en vez de dejarlo para el repaso de
    // dentro de seis horas: así se instala a los pocos segundos de guardar y no al cabo de
    // media tarde. La insignia está encendida mientras tanto, y pulsarla es lo que pone
    // INSTALAR_YA.
    while HAY_CAMBIOS_SIN_GUARDAR.load(Ordering::SeqCst) && !INSTALAR_YA.swap(false, Ordering::SeqCst) {
        tokio::time::sleep(Duration::from_millis(ESPERA_GUARDADO_MS)).await;
    }

    // `install` lanza el instalador NSIS en modo `passive` (barra de progreso, ningún clic)
    // y este proceso se cierra: lo que venga detrás solo se ejecuta si algo ha fallado.
    if let Err(e) = actualizacion.install(paquete) {
        return set(
            app,
            json!({
                "status": "error",
                "error": format!("No se pudo instalar la versión {nueva}: {e}"),
            }),
        );
    }

    // En Windows el instalador ya ha pedido el cierre de este proceso, así que aquí no se
    // suele llegar. Se deja el relanzado explícito porque es lo que cierra el ciclo si el
    // instalador decidiera no matarlo: sin esto la app se quedaría corriendo con la versión
    // vieja en memoria y la nueva ya escrita en disco.
    app.restart();
}

// --- Comandos ---------------------------------------------------------------------------

#[tauri::command]
pub fn updater_get(app: AppHandle) -> Value {
    leer_estado(&app)
}

/// El botón «Buscar actualizaciones» de Ajustes. No devuelve el resultado: la consulta y la
/// instalación pueden tardar, y el renderer ya se entera por `updater:state`. Devolver el
/// estado de ahora mismo (`checking`) es justo lo que la tarjeta necesita para pintarse.
#[tauri::command]
pub fn updater_check(app: AppHandle) -> Value {
    let handle = app.clone();
    tauri::async_runtime::spawn(async move {
        consultar(&handle).await;
    });
    leer_estado(&app)
}

/// La insignia de la barra de título. Solo se ve mientras la instalación está esperando a
/// que se guarden los cambios, así que lo único que hace es levantar esa espera: el paquete
/// ya está descargado y la tarea que lo tiene sigue viva dentro de `consultar`.
#[tauri::command]
pub fn updater_install(app: AppHandle) -> bool {
    if leer_estado(&app).get("status").and_then(Value::as_str) != Some("downloaded") {
        return false;
    }
    INSTALAR_YA.store(true, Ordering::SeqCst);
    true
}

/// Lo llama el renderer cuando cambia si hay algo sin escribir en la Flash del teclado.
#[tauri::command]
pub fn updater_ocupado(ocupado: bool) {
    HAY_CAMBIOS_SIN_GUARDAR.store(ocupado, Ordering::SeqCst);
}
