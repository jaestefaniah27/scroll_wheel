// Agrupar los giros del mando antes de mandarlos.
//
// El firmware manda un `MACRO:<id>` **por cada muesca** del encoder, así que un giro
// rápido son veinte disparos en un suspiro. Sin agrupar, eso son veinte peticiones en
// cola contra un cacharro que las atiende de una en una, y el efecto llega tarde y a
// tirones.
//
// Solo se pueden agrupar los incrementos: sumar dos da el mismo resultado que aplicarlos
// por separado. Encender y apagar NO se agrupan —dos pulsaciones son dos órdenes, y
// fundidas en una sola «alterna» se anularían entre ellas—, y por eso el agrupado se
// declara por acción y no de forma global.
//
// El tiempo entra por parámetro y no se lee del reloj: es lo que permite probar esto en
// microsegundos en vez de esperar de verdad.

use std::collections::HashMap;

/// Qué hacer con lo acumulado, decidido por el llamante al vencer el plazo.
#[derive(Debug, PartialEq)]
pub struct Envio {
    pub plugin: String,
    pub op: String,
    pub valor: f64,
}

#[derive(Debug, Clone, Copy)]
struct Pendiente {
    valor: f64,
    vence_en: u64,
}

#[derive(Default)]
pub struct Agrupador {
    ahora: u64,
    pendientes: HashMap<(String, String), Pendiente>,
    /// Acciones con una petición en vuelo. Mientras la haya, lo que llegue se sigue
    /// acumulando pero no se manda: encadenar peticiones cada 60 ms contra un cacharro
    /// que tarda 20 en resolver una hace que la mediana se dispare.
    en_vuelo: HashMap<(String, String), bool>,
}

impl Agrupador {
    pub fn new() -> Self {
        Self::default()
    }

    /// Llega una muesca. Si la acción no agrupa, devuelve el envío inmediato.
    pub fn empujar(
        &mut self,
        plugin: &str,
        op: &str,
        valor: f64,
        agrupa_ms: Option<u64>,
    ) -> Option<Envio> {
        let Some(ms) = agrupa_ms else {
            return Some(Envio { plugin: plugin.into(), op: op.into(), valor });
        };

        let clave = (plugin.to_string(), op.to_string());
        let entrada = self.pendientes.entry(clave).or_insert(Pendiente {
            valor: 0.0,
            vence_en: self.ahora + ms,
        });
        entrada.valor += valor;
        None
    }

    /// Avanza el reloj y devuelve lo que toque mandar.
    pub fn avanzar(&mut self, ms: u64) -> Vec<Envio> {
        self.ahora += ms;

        let vencidas: Vec<(String, String)> = self
            .pendientes
            .iter()
            .filter(|(clave, p)| {
                p.vence_en <= self.ahora && !self.en_vuelo.get(*clave).copied().unwrap_or(false)
            })
            .map(|(clave, _)| clave.clone())
            .collect();

        let mut envios: Vec<Envio> = vencidas
            .into_iter()
            .filter_map(|clave| {
                let p = self.pendientes.remove(&clave)?;
                self.en_vuelo.insert(clave.clone(), true);
                Some(Envio { plugin: clave.0, op: clave.1, valor: p.valor })
            })
            .collect();

        // Orden estable: sin esto el resultado depende del recorrido del mapa y los
        // tests salen intermitentes.
        envios.sort_by(|a, b| (&a.plugin, &a.op).cmp(&(&b.plugin, &b.op)));
        envios
    }

    /// La petición terminó (bien o mal). Si mientras tanto llegaron más muescas, se
    /// programa otra ronda.
    pub fn terminado(&mut self, plugin: &str, op: &str, agrupa_ms: u64) {
        let clave = (plugin.to_string(), op.to_string());
        self.en_vuelo.remove(&clave);

        // Lo acumulado durante el vuelo esperaba a un plazo ya vencido: se le da uno
        // nuevo para que no salga disparado en el mismo instante.
        if let Some(p) = self.pendientes.get_mut(&clave) {
            p.vence_en = self.ahora + agrupa_ms;
        }
    }

    pub fn hay_pendientes(&self) -> bool {
        !self.pendientes.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::{Agrupador, Envio};

    const MS: u64 = 120;

    #[test]
    fn dos_muescas_se_funden_en_una_peticion() {
        let mut a = Agrupador::new();
        assert!(a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS)).is_none());
        assert!(a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS)).is_none());

        let envios = a.avanzar(MS);
        assert_eq!(
            envios,
            vec![Envio { plugin: "lampdesk".into(), op: "brightness_delta".into(), valor: 10.0 }]
        );
    }

    #[test]
    fn antes_del_plazo_no_se_manda_nada() {
        let mut a = Agrupador::new();
        a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS));

        assert!(a.avanzar(MS - 1).is_empty());
        assert_eq!(a.avanzar(1).len(), 1);
    }

    #[test]
    fn una_accion_sin_agrupar_sale_por_cada_muesca() {
        // Dos pulsaciones de «alterna» fundidas en una se anularían entre ellas.
        let mut a = Agrupador::new();
        let e1 = a.empujar("lampdesk", "toggle", 0.0, None);
        let e2 = a.empujar("lampdesk", "toggle", 0.0, None);

        assert!(e1.is_some());
        assert!(e2.is_some());
        assert!(a.avanzar(MS).is_empty());
    }

    #[test]
    fn lo_que_llega_en_vuelo_encadena_otra_ronda() {
        let mut a = Agrupador::new();
        a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS));
        assert_eq!(a.avanzar(MS).len(), 1);

        // Con la petición en vuelo llegan más muescas: se acumulan y NO se mandan.
        a.empujar("lampdesk", "brightness_delta", 3.0, Some(MS));
        assert!(a.avanzar(MS).is_empty(), "no debe mandar nada con una petición en vuelo");

        a.terminado("lampdesk", "brightness_delta", MS);
        assert!(a.avanzar(MS - 1).is_empty(), "el plazo se reinicia al terminar");

        let envios = a.avanzar(1);
        assert_eq!(envios.len(), 1);
        assert_eq!(envios[0].valor, 3.0);
    }

    #[test]
    fn los_incrementos_negativos_se_restan() {
        // Girar adelante y atrás deja el brillo donde estaba, y sin mandar nada absurdo.
        let mut a = Agrupador::new();
        a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS));
        a.empujar("lampdesk", "brightness_delta", -5.0, Some(MS));

        let envios = a.avanzar(MS);
        assert_eq!(envios.len(), 1);
        assert_eq!(envios[0].valor, 0.0);
    }

    #[test]
    fn cada_accion_se_agrupa_por_su_cuenta() {
        // Girar el brillo no puede llevarse por delante lo acumulado del color.
        let mut a = Agrupador::new();
        a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS));
        a.empujar("lampdesk", "color_delta", 7.0, Some(MS));

        let envios = a.avanzar(MS);
        assert_eq!(envios.len(), 2);
        assert_eq!(envios[0].op, "brightness_delta");
        assert_eq!(envios[0].valor, 5.0);
        assert_eq!(envios[1].op, "color_delta");
        assert_eq!(envios[1].valor, 7.0);
    }

    #[test]
    fn dos_complementos_no_se_mezclan() {
        let mut a = Agrupador::new();
        a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS));
        a.empujar("otra-lampara", "brightness_delta", 9.0, Some(MS));

        let envios = a.avanzar(MS);
        assert_eq!(envios.len(), 2);
        assert_eq!(envios[0].plugin, "lampdesk");
        assert_eq!(envios[1].plugin, "otra-lampara");
    }

    #[test]
    fn un_giro_rapido_son_veinte_muescas_y_una_sola_peticion() {
        // El caso real que motiva todo esto: un golpe de muñeca son ~20 muescas en unas
        // décimas, y sin agrupar serían 20 peticiones en cola.
        let mut a = Agrupador::new();
        for _ in 0..20 {
            a.empujar("lampdesk", "brightness_delta", 5.0, Some(MS));
            a.avanzar(3);
        }

        let envios = a.avanzar(MS);
        assert_eq!(envios.len(), 1, "veinte muescas seguidas deben salir en una sola petición");
        assert_eq!(envios[0].valor, 100.0);
    }

    #[test]
    fn un_giro_sostenido_no_espera_indefinidamente() {
        // Lo contrario del anterior, y es a propósito: si alguien tiene el mando girando
        // sin parar, la ventana vence igual y sale una primera petición. Acumular hasta
        // que suelte dejaría la lámpara sin responder mientras gira, que es justo lo que
        // se quería arreglar.
        let mut a = Agrupador::new();
        let mut envios_totales = 0;

        for _ in 0..60 {
            a.empujar("lampdesk", "brightness_delta", 1.0, Some(MS));
            for envio in a.avanzar(10) {
                envios_totales += 1;
                a.terminado(&envio.plugin, &envio.op, MS);
            }
        }

        assert!(envios_totales >= 2, "un giro de 600 ms debe dar más de una petición");
        assert!(envios_totales <= 6, "pero no una por muesca: iban 60 muescas");
    }
}
