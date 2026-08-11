// El descriptor es el contrato con el renderer, y el renderer no se toca en toda la
// migración: `src/plugins.js`, la tarjeta de Ajustes y el editor de teclas ya lo
// consumen tal cual.
//
// Esta prueba compara el descriptor que produce el motor declarativo con el que producía
// electron/plugins.js para el MISMO complemento. El fichero de referencia no está escrito
// a mano: lo generó `tests/fixtures/generar-fixture.mjs` extrayendo las funciones del
// propio electron/plugins.js y ejecutándolas sobre el lampdesk de la API 1. Si algún día
// hay que regenerarlo:
//
//     node src-tauri/crates/orby-core/tests/fixtures/generar-fixture.mjs
//
// Si esta prueba se pone roja, el frontend va a ver algo distinto de lo que espera, y el
// fallo aparecerá en una vista lejos de aquí.

use orby_core::plugins::Manifiesto;
use serde_json::Value;

// El manifiesto de verdad vive en `plugins/lampdesk/plugin.json` (Tarea 10, paso 5): se
// lee de ahí y no de una copia en `fixtures/`, para que esta prueba no pueda quedarse
// verde comparando un fichero que ya no es el que instala la app. El lampdesk de la API 1
// dejó de funcionar en Electron al hacer este cambio: es una excepción deliberada a que
// «Electron sigue funcionando hasta la Tarea 13», aceptada porque es solo el complemento
// de ejemplo y Electron se retira igualmente pronto.
const MANIFIESTO: &str = include_str!("../../../../plugins/lampdesk/plugin.json");
const REFERENCIA: &str = include_str!("fixtures/descriptor-lampdesk-electron.json");

fn descriptor_actual() -> Value {
    let m = Manifiesto::desde_json(MANIFIESTO).expect("lampdesk debería ser un manifiesto válido");
    // `enabled` y `values` los pone el motor desde la configuración del usuario; para
    // comparar la forma se usan los mismos que usó el generador de la referencia.
    m.descriptor(true, &serde_json::json!({}))
}

#[test]
fn el_descriptor_de_lampdesk_es_el_mismo_que_producia_electron() {
    let esperado: Value = serde_json::from_str(REFERENCIA).unwrap();
    let actual = descriptor_actual();

    // Se comparan campo a campo en vez de de golpe: un `assert_eq` de dos JSON gordos
    // dice "no son iguales" y te deja buscándolo a ojo.
    let esperado = esperado.as_object().unwrap();
    let actual_obj = actual.as_object().unwrap();

    let mut claves: Vec<&String> = esperado.keys().collect();
    claves.sort();
    for clave in claves {
        assert_eq!(
            actual_obj.get(clave),
            esperado.get(clave),
            "el campo «{clave}» del descriptor no coincide con el que daba Electron"
        );
    }

    let sobran: Vec<&String> =
        actual_obj.keys().filter(|k| !esperado.contains_key(*k)).collect();
    assert!(sobran.is_empty(), "el descriptor trae campos que Electron no daba: {sobran:?}");
}

#[test]
fn lampdesk_conserva_sus_siete_acciones_y_sus_dos_visores() {
    let m = Manifiesto::desde_json(MANIFIESTO).unwrap();

    let ops: Vec<&str> = m.actions.iter().map(|a| a.op.as_str()).collect();
    assert_eq!(
        ops,
        vec!["toggle", "on", "off", "brightness_delta", "color_delta", "brightness_set", "color_set"]
    );

    let visores: Vec<&str> = m.views.iter().map(|v| v.op.as_str()).collect();
    assert_eq!(visores, vec!["brightness_view", "color_view"]);
}

#[test]
fn los_giros_de_lampdesk_declaran_su_agrupado() {
    // Sin esto, un giro rápido son veinte peticiones contra una ESP32 que las atiende de
    // una en una. Es la única lógica de verdad que tenía el lampdesk de la API 1.
    let m = Manifiesto::desde_json(MANIFIESTO).unwrap();

    for op in ["brightness_delta", "color_delta"] {
        let accion = m.crudo["actions"]
            .as_array()
            .unwrap()
            .iter()
            .find(|a| a["op"] == op)
            .unwrap_or_else(|| panic!("falta la acción {op}"));

        assert_eq!(accion["coalesce"]["ms"], 120, "«{op}» debería agrupar a 120 ms");
        assert_eq!(accion["coalesce"]["mode"], "sum");
    }

    // Y las que NO se pueden agrupar no lo declaran: dos pulsaciones de «alterna»
    // fundidas en una se anularían entre ellas.
    for op in ["toggle", "on", "off"] {
        let accion = m.crudo["actions"].as_array().unwrap().iter().find(|a| a["op"] == op).unwrap();
        assert!(accion.get("coalesce").is_none(), "«{op}» no se puede agrupar");
    }
}

#[test]
fn la_comprobacion_reintenta_pero_las_acciones_no() {
    let m = Manifiesto::desde_json(MANIFIESTO).unwrap();

    // Leer es inofensivo y la lámpara se bloquea unos cientos de ms avisando a HomeKit.
    assert_eq!(m.crudo["test"]["retries"], 1);

    // Ninguna acción declara reintentos: un incremento repetido se aplicaría dos veces.
    for accion in m.crudo["actions"].as_array().unwrap() {
        assert!(
            accion.get("retries").is_none(),
            "la acción «{}» no debe reintentar",
            accion["op"]
        );
    }
}

#[test]
fn el_tiempo_maximo_esta_declarado() {
    // Sin él, una lámpara desenchufada deja la ejecución de secuencias colgada hasta que
    // agote el TCP, más de un minuto.
    let m = Manifiesto::desde_json(MANIFIESTO).unwrap();
    assert_eq!(m.crudo["request"]["timeoutMs"], 4000);
}
