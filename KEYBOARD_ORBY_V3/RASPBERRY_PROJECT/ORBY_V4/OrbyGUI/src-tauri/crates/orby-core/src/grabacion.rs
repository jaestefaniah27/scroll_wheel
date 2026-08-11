// La grabadora de ratón y teclado, sin la parte que toca Windows.
//
// Aquí está lo que decide **qué** se guarda y **cuándo** se reproduce cada evento; los
// ganchos de bajo nivel y el `SendInput` viven en `src-tauri/src/recorder.rs`. La
// separación es la de siempre en este crate, pero aquí paga doble: el filtrado del
// movimiento y el recorte del silencio inicial son dos reglas de tres líneas que, mal
// puestas, producen grabaciones que se reproducen «casi» bien, y eso no se detecta
// mirando la pantalla.
//
// El formato de los eventos es **el que ya hay guardado en las configuraciones de los
// usuarios** (lo escribía `electron/recorder.js`). No se puede cambiar: una grabación
// hecha con la app de Electron tiene que reproducirse igual en la de Tauri.

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Un evento cada 16 ms como mucho. El ratón dispara cientos por segundo y guardarlos
/// todos hincha la grabación sin que se note nada al reproducir.
const MOVIMIENTO_MIN_MS: u64 = 16;

/// Y solo si se ha movido de verdad. El umbral es por eje: basta con que uno de los dos
/// se mueva lo suficiente.
const MOVIMIENTO_MIN_PX: i32 = 3;

/// Fuera del escritorio virtual de cualquier equipo, para que el primer movimiento pase
/// siempre el filtro de distancia sin necesitar un caso especial.
const LEJOS: i32 = -9999;

/// Un evento grabado, con la forma exacta que ya está en `orby-config.json`.
///
/// `t` son milisegundos desde el principio de la grabación, **ya recortados** (ver
/// [`Grabador::terminar`]).
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(tag = "k")]
pub enum Evento {
    /// `c` es el código de tecla de uiohook, no una tecla virtual de Windows: es lo que
    /// guardaba la app de Electron y lo que hay en las grabaciones del usuario.
    #[serde(rename = "kdown")]
    TeclaAbajo { c: u32, t: u64 },
    #[serde(rename = "kup")]
    TeclaArriba { c: u32, t: u64 },
    /// `b` numera como libuiohook: 1 izquierdo, 2 derecho, 3 central.
    #[serde(rename = "mdown")]
    BotonAbajo { b: u32, x: i32, y: i32, t: u64 },
    #[serde(rename = "mup")]
    BotonArriba { b: u32, x: i32, y: i32, t: u64 },
    /// `d` es el eje (3 vertical, 4 horizontal) y `r` el giro con su signo.
    #[serde(rename = "wheel")]
    Rueda { r: i32, d: u32, t: u64 },
    #[serde(rename = "move")]
    Mover { x: i32, y: i32, t: u64 },
}

impl Evento {
    pub fn t(&self) -> u64 {
        match *self {
            Evento::TeclaAbajo { t, .. }
            | Evento::TeclaArriba { t, .. }
            | Evento::BotonAbajo { t, .. }
            | Evento::BotonArriba { t, .. }
            | Evento::Rueda { t, .. }
            | Evento::Mover { t, .. } => t,
        }
    }

    fn restar(&mut self, cuanto: u64) {
        let t = match self {
            Evento::TeclaAbajo { t, .. }
            | Evento::TeclaArriba { t, .. }
            | Evento::BotonAbajo { t, .. }
            | Evento::BotonArriba { t, .. }
            | Evento::Rueda { t, .. }
            | Evento::Mover { t, .. } => t,
        };
        *t = t.saturating_sub(cuanto);
    }
}

/// Interpreta la lista de eventos de una macro.
///
/// Los que no se entiendan se descartan uno a uno en vez de tumbar la grabación entera:
/// una versión futura que añada un tipo de evento dejaría, si no, inservibles todas las
/// grabaciones al abrirlas con una versión vieja.
pub fn leer_eventos(valor: &Value) -> Vec<Evento> {
    valor
        .as_array()
        .map(|lista| {
            lista
                .iter()
                .filter_map(|v| serde_json::from_value(v.clone()).ok())
                .collect()
        })
        .unwrap_or_default()
}

/// Acumula lo que capturan los ganchos. No sabe nada del reloj: el tiempo entra por
/// parámetro, que es lo que permite probar el filtrado sin esperas de verdad.
#[derive(Debug)]
pub struct Grabador {
    eventos: Vec<Evento>,
    t0: u64,
    ultimo_movimiento: u64,
    ultimo_x: i32,
    ultimo_y: i32,
}

impl Grabador {
    pub fn nuevo(ahora_ms: u64) -> Self {
        Self {
            eventos: Vec::new(),
            t0: ahora_ms,
            ultimo_movimiento: 0,
            ultimo_x: LEJOS,
            ultimo_y: LEJOS,
        }
    }

    fn desde_el_inicio(&self, ahora_ms: u64) -> u64 {
        ahora_ms.saturating_sub(self.t0)
    }

    pub fn tecla(&mut self, ahora_ms: u64, abajo: bool, c: u32) {
        let t = self.desde_el_inicio(ahora_ms);
        self.eventos.push(if abajo {
            Evento::TeclaAbajo { c, t }
        } else {
            Evento::TeclaArriba { c, t }
        });
    }

    /// Los clics **no** pasan por el filtro del movimiento: se guardan todos y con las
    /// coordenadas del momento, que es lo que permite recolocar el puntero al
    /// reproducirlos aunque el movimiento que llevaba hasta allí se descartara.
    pub fn boton(&mut self, ahora_ms: u64, abajo: bool, b: u32, x: i32, y: i32) {
        let t = self.desde_el_inicio(ahora_ms);
        self.eventos.push(if abajo {
            Evento::BotonAbajo { b, x, y, t }
        } else {
            Evento::BotonArriba { b, x, y, t }
        });
    }

    pub fn rueda(&mut self, ahora_ms: u64, r: i32, d: u32) {
        let t = self.desde_el_inicio(ahora_ms);
        self.eventos.push(Evento::Rueda { r, d, t });
    }

    /// `true` si el movimiento se ha guardado. Las dos condiciones son las de
    /// `electron/recorder.js` y las comparaciones son estrictas por el mismo motivo que
    /// allí: a 60 Hz justos, un `<=` en el tiempo se comería uno de cada dos eventos.
    pub fn mover(&mut self, ahora_ms: u64, x: i32, y: i32) -> bool {
        if ahora_ms.saturating_sub(self.ultimo_movimiento) < MOVIMIENTO_MIN_MS {
            return false;
        }
        if (x - self.ultimo_x).abs() < MOVIMIENTO_MIN_PX
            && (y - self.ultimo_y).abs() < MOVIMIENTO_MIN_PX
        {
            return false;
        }
        self.ultimo_movimiento = ahora_ms;
        self.ultimo_x = x;
        self.ultimo_y = y;
        let t = self.desde_el_inicio(ahora_ms);
        self.eventos.push(Evento::Mover { x, y, t });
        true
    }

    pub fn cuantos(&self) -> usize {
        self.eventos.len()
    }

    /// Cierra la grabación restando a todos los tiempos el del primer evento.
    ///
    /// Sin esto, la reproducción empieza esperando lo que tardó el usuario en moverse
    /// después de pulsar la tecla, **y en bucle arrastra esa espera en cada vuelta**.
    pub fn terminar(mut self) -> Vec<Evento> {
        let Some(cabeza) = self.eventos.first().map(Evento::t) else {
            return self.eventos;
        };
        if cabeza > 0 {
            for ev in &mut self.eventos {
                ev.restar(cabeza);
            }
        }
        self.eventos
    }
}

/// Lo que queda por esperar antes de soltar el evento, en milisegundos.
///
/// El objetivo se calcula siempre contra el arranque de la pasada y no sumando esperas:
/// así un evento que se retrasa no desplaza a todos los siguientes, y una grabación de
/// dos minutos acaba durando dos minutos.
///
/// `velocidad` **divide** los tiempos: 2 es el doble de rápido. Un valor de cero o
/// negativo se trata como 1, igual que en Electron, porque viene de la configuración y
/// una velocidad de cero colgaría la reproducción para siempre.
pub fn espera_ms(t: u64, velocidad: f64, transcurrido_ms: u64) -> u64 {
    let ritmo = if velocidad > 0.0 { velocidad } else { 1.0 };
    let objetivo = t as f64 / ritmo;
    let espera = objetivo - transcurrido_ms as f64;
    if espera > 0.0 {
        espera.ceil() as u64
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn el_primer_movimiento_siempre_se_guarda() {
        let mut g = Grabador::nuevo(1000);
        // Mismo instante que el arranque y a un píxel del origen: pasa igualmente porque
        // la posición anterior está fuera de cualquier pantalla.
        assert!(g.mover(1000, 1, 1));
    }

    #[test]
    fn el_movimiento_se_filtra_por_tiempo() {
        let mut g = Grabador::nuevo(0);
        assert!(g.mover(100, 0, 0));
        // 15 ms después: no llega a los 16.
        assert!(!g.mover(115, 500, 500));
        // A los 16 justos sí entra.
        assert!(g.mover(116, 500, 500));
    }

    #[test]
    fn el_movimiento_se_filtra_por_distancia() {
        let mut g = Grabador::nuevo(0);
        assert!(g.mover(100, 500, 500));
        // Dos píxeles en cada eje: no llega.
        assert!(!g.mover(200, 502, 502));
        // Tres en uno solo basta.
        assert!(g.mover(300, 503, 502));
    }

    #[test]
    fn los_clics_no_se_filtran() {
        let mut g = Grabador::nuevo(0);
        g.boton(10, true, 1, 5, 5);
        g.boton(11, false, 1, 5, 5);
        g.boton(12, true, 1, 5, 5);
        assert_eq!(g.cuantos(), 3);
    }

    #[test]
    fn se_recorta_el_silencio_inicial() {
        let mut g = Grabador::nuevo(0);
        g.tecla(3000, true, 30);
        g.tecla(3100, false, 30);
        let eventos = g.terminar();
        assert_eq!(eventos[0].t(), 0);
        assert_eq!(eventos[1].t(), 100);
    }

    #[test]
    fn recortar_una_grabacion_vacia_no_revienta() {
        let g = Grabador::nuevo(0);
        assert!(g.terminar().is_empty());
    }

    #[test]
    fn los_tiempos_se_cuentan_desde_el_arranque() {
        let mut g = Grabador::nuevo(5000);
        g.tecla(5000, true, 30);
        g.tecla(5250, false, 30);
        let eventos = g.terminar();
        assert_eq!(eventos[0].t(), 0);
        assert_eq!(eventos[1].t(), 250);
    }

    #[test]
    fn la_velocidad_divide_los_tiempos() {
        // A velocidad 2, el evento del segundo 1 toca a los 500 ms.
        assert_eq!(espera_ms(1000, 2.0, 0), 500);
        assert_eq!(espera_ms(1000, 2.0, 400), 100);
        // Ya se ha pasado el momento: no se espera nada.
        assert_eq!(espera_ms(1000, 2.0, 600), 0);
    }

    #[test]
    fn una_velocidad_invalida_se_trata_como_uno() {
        assert_eq!(espera_ms(1000, 0.0, 0), 1000);
        assert_eq!(espera_ms(1000, -3.0, 0), 1000);
    }

    #[test]
    fn los_eventos_se_leen_con_la_forma_que_guardaba_electron() {
        let valor = serde_json::json!([
            { "k": "kdown", "c": 30, "t": 0 },
            { "k": "mdown", "b": 1, "x": 100, "y": 200, "t": 50 },
            { "k": "wheel", "r": -1, "d": 3, "t": 80 },
            { "k": "move", "x": 10, "y": 20, "t": 90 },
            { "k": "loquesea", "t": 100 },
        ]);
        let eventos = leer_eventos(&valor);
        // El desconocido cae, los otros cuatro sobreviven.
        assert_eq!(eventos.len(), 4);
        assert_eq!(eventos[0], Evento::TeclaAbajo { c: 30, t: 0 });
        assert_eq!(eventos[1], Evento::BotonAbajo { b: 1, x: 100, y: 200, t: 50 });
        assert_eq!(eventos[2], Evento::Rueda { r: -1, d: 3, t: 80 });
        assert_eq!(eventos[3], Evento::Mover { x: 10, y: 20, t: 90 });
    }

    #[test]
    fn lo_grabado_se_serializa_como_lo_hacia_electron() {
        let mut g = Grabador::nuevo(0);
        g.tecla(0, true, 30);
        g.boton(20, false, 2, -1920, 40);
        let eventos = g.terminar();
        let json = serde_json::to_value(&eventos).unwrap();
        assert_eq!(
            json,
            serde_json::json!([
                { "k": "kdown", "c": 30, "t": 0 },
                { "k": "mup", "b": 2, "x": -1920, "y": 40, "t": 20 },
            ])
        );
    }
}
