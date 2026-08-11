// La máquina de estados de la conexión con el teclado.
//
// Vive aquí, separada del puerto de verdad, porque es donde están los fallos caros: un
// handshake que se rinde antes de tiempo descarta el puerto bueno y la app se queda en
// «Desconectado» hasta desenchufar; un vigilante mal ajustado tira la conexión sola cada
// pocos minutos. Nada de eso se puede probar cómodamente con un teclado delante, y aquí
// se prueba en microsegundos.
//
// El tiempo entra por parámetro (`al_pasar_tiempo`) y no se lee del reloj: es lo único
// que hace falta para que todo esto sea comprobable.
//
// Lo que NO está aquí, y no debe estarlo: la correlación entre petición y respuesta.
// Eso vive en src/device.js con su cola de `pending`, y sigue igual. Esta máquina solo
// saluda, vigila que siga vivo y reenvía.

use crate::protocolo::{parse_device_info, DeviceInfo};

/// Tras abrir, se espera un poco antes de saludar: el teclado puede estar acabando de
/// enumerarse.
const PRIMER_ACK_MS: u64 = 400;
/// Se pregunta DOS veces antes de descartar un puerto. El primer ACK se pierde a menudo
/// si el teclado está ocupado (arranque de las pantallas, telemetría de la rueda), y con
/// un solo intento se descartaba el puerto bueno.
const ESPERA_SALUDO_MS: u64 = 1200;
const ACKS_DE_SALUDO: u8 = 2;

/// El teclado no habla si no lo tocas, así que el silencio no significa nada por sí solo:
/// hay que preguntar. `ACK` siempre se contesta.
const SILENCIO_MS: u64 = 12_000;
const RESPUESTA_MS: u64 = 4_000;

/// Cuando hay telemetría pero no saludo, se insiste antes de rendirse.
const PERSECUCION_ACKS: u8 = 4;
const PERSECUCION_CADA_MS: u64 = 600;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Estado {
    /// Sin puerto: rastreando.
    Buscando,
    /// Puerto abierto, esperando a que se presente.
    Saludando,
    Conectado,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Evento {
    Conectado(Box<DeviceInfo>),
    Desconectado,
    Datos(String),
    Buscando,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Salida {
    /// Escribir esto en el puerto (ya lleva el salto de línea).
    Mandar(String),
    /// Avisar al renderer.
    Emitir(Evento),
    /// Cerrar el puerto. Va sola cuando el saludo se rinde —nunca llegó a estar
    /// conectado, así que no hay desconexión que anunciar— y acompañada de
    /// `Desconectado` cuando el vigilante da por muerta una conexión que sí lo estuvo.
    SoltarPuerto,
}

pub struct Maquina {
    estado: Estado,
    ahora: u64,
    info: Option<DeviceInfo>,

    ultima_linea_en: u64,
    ping_en: Option<u64>,

    acks_enviados: u8,
    siguiente_ack_en: Option<u64>,

    /// Hay telemetría pero no saludo: se insiste.
    provisional: bool,
    persecucion_intentos: u8,
    persecucion_en: Option<u64>,
}

impl Default for Maquina {
    fn default() -> Self {
        Self::new()
    }
}

impl Maquina {
    pub fn new() -> Self {
        Self {
            estado: Estado::Buscando,
            ahora: 0,
            info: None,
            ultima_linea_en: 0,
            ping_en: None,
            acks_enviados: 0,
            siguiente_ack_en: None,
            provisional: false,
            persecucion_intentos: 0,
            persecucion_en: None,
        }
    }

    pub fn estado(&self) -> Estado {
        self.estado
    }

    pub fn info(&self) -> Option<&DeviceInfo> {
        self.info.as_ref()
    }

    /// Se acaba de abrir un puerto candidato.
    pub fn al_abrir(&mut self) -> Vec<Salida> {
        self.estado = Estado::Saludando;
        self.info = None;
        self.provisional = false;
        self.acks_enviados = 0;
        self.siguiente_ack_en = Some(self.ahora + PRIMER_ACK_MS);
        self.ultima_linea_en = self.ahora;
        self.ping_en = None;
        Vec::new()
    }

    /// El puerto se cerró por su cuenta (se desenchufó el teclado).
    pub fn al_cerrarse(&mut self) -> Vec<Salida> {
        let estaba_conectado = self.estado == Estado::Conectado;
        self.reiniciar();

        let mut salidas = Vec::new();
        if estaba_conectado {
            salidas.push(Salida::Emitir(Evento::Desconectado));
        }
        salidas.push(Salida::Emitir(Evento::Buscando));
        salidas
    }

    fn reiniciar(&mut self) {
        self.estado = Estado::Buscando;
        self.info = None;
        self.provisional = false;
        self.ping_en = None;
        self.siguiente_ack_en = None;
        self.persecucion_en = None;
        self.persecucion_intentos = 0;
        self.acks_enviados = 0;
    }

    pub fn al_recibir(&mut self, linea: &str) -> Vec<Salida> {
        // Cualquier línea vale como señal de vida.
        self.ultima_linea_en = self.ahora;

        if let Some(info) = parse_device_info(linea) {
            return self.al_saludar(info);
        }

        let mut salidas = Vec::new();

        // Hay teclado —está mandando telemetría— pero su presentación no ha llegado.
        // Antes se daba por conectado con una identidad inventada, y eso hacía que la
        // app lo tratara como un firmware antiguo: sin GET_HASH se descargaban todos los
        // perfiles y todos los iconos en cada conexión. Ahora se le vuelve a pedir.
        if self.estado != Estado::Conectado
            && !self.provisional
            && (linea.starts_with("KEY_EV:") || linea.starts_with("ENC:"))
        {
            self.provisional = true;
            self.persecucion_intentos = 1;
            self.persecucion_en = Some(self.ahora + PERSECUCION_CADA_MS);
            salidas.push(Salida::Mandar("ACK\n".into()));
        }

        salidas.push(Salida::Emitir(Evento::Datos(linea.to_string())));
        salidas
    }

    fn al_saludar(&mut self, info: DeviceInfo) -> Vec<Salida> {
        // El vigilante pregunta con ACK cada tanto, así que esta línea llega también con
        // la conexión ya hecha: entonces no es una conexión nueva, y volver a anunciarla
        // haría a la app resincronizarse cada pocos segundos.
        let ya_conectado = self.estado == Estado::Conectado
            && self.info.as_ref().is_some_and(|i| i.raw == info.raw);

        let anuncia_host_app = info.get("hostapp") == Some("1");

        self.info = Some(info);
        self.provisional = false;
        self.persecucion_en = None;
        self.estado = Estado::Conectado;
        self.siguiente_ack_en = None;
        self.ping_en = None;

        let mut salidas = Vec::new();

        // El teclado tacha en sus pantallas las teclas que solo puede ejecutar el PC. No
        // lo deduce de tener el puerto abierto —la WebGUI también lo abre y no ejecuta
        // nada—, así que hay que decírselo. Va en CADA presentación, no solo en la
        // primera, porque el aviso caduca en el teclado: el ACK del vigilante lo renueva.
        if anuncia_host_app {
            salidas.push(Salida::Mandar("HOST_APP:1\n".into()));
        }

        if !ya_conectado {
            salidas.push(Salida::Emitir(Evento::Conectado(Box::new(
                self.info.clone().unwrap(),
            ))));
        }

        // La línea del saludo no se reenvía como dato: no es telemetría, y device.js no
        // la espera por ahí.
        salidas
    }

    pub fn al_pasar_tiempo(&mut self, ms: u64) -> Vec<Salida> {
        self.ahora += ms;
        let mut salidas = Vec::new();

        match self.estado {
            Estado::Saludando => salidas.extend(self.tic_saludo()),
            Estado::Conectado => salidas.extend(self.tic_vigilante()),
            Estado::Buscando => {}
        }

        salidas.extend(self.tic_persecucion());
        salidas
    }

    fn tic_saludo(&mut self) -> Vec<Salida> {
        // Si ya sabemos que hay un teclado ahí porque está mandando telemetría, manda la
        // persecución y esta cadena se aparta.
        //
        // Es una corrección respecto a electron/serial.js, donde las dos corren a la vez:
        // allí los dos temporizadores mandan ACK por su cuenta (inofensivo, pero ruidoso)
        // y, peor, la cadena del saludo suelta el puerto a los 2,8 s aunque la
        // persecución estuviera a punto de conectar con identidad mínima. Soltar un
        // puerto del que ha llegado telemetría es justo lo contrario de lo que quiere esa
        // función.
        if self.provisional {
            return Vec::new();
        }

        let Some(cuando) = self.siguiente_ack_en else { return Vec::new() };
        if self.ahora < cuando {
            return Vec::new();
        }

        if self.acks_enviados < ACKS_DE_SALUDO {
            self.acks_enviados += 1;
            self.siguiente_ack_en = Some(self.ahora + ESPERA_SALUDO_MS);
            return vec![Salida::Mandar("ACK\n".into())];
        }

        // No es un Orby, o no contesta. Se suelta el puerto: uno abierto que ya no se va
        // a usar y que nadie recuerda sigue cogido, y el siguiente intento se estrella
        // contra «Access denied».
        self.reiniciar();
        vec![Salida::SoltarPuerto, Salida::Emitir(Evento::Buscando)]
    }

    fn tic_vigilante(&mut self) -> Vec<Salida> {
        if let Some(ping) = self.ping_en {
            // Contestó: cualquier línea posterior al ping vale.
            if self.ultima_linea_en > ping {
                self.ping_en = None;
                return Vec::new();
            }
            if self.ahora.saturating_sub(ping) > RESPUESTA_MS {
                self.reiniciar();
                // El puerto se suelta AQUÍ, y no solo se anuncia la desconexión: quien
                // ejecuta las salidas únicamente toca el puerto cuando se lo piden, así
                // que sin esto el handle se quedaba abierto para siempre con la máquina
                // ya en «Buscando». El rastreo no vuelve a abrir nada mientras haya un
                // puerto cogido, así que la app se quedaba encallada: el renderer
                // enseñando «buscando» y nadie buscando de verdad.
                return vec![
                    Salida::SoltarPuerto,
                    Salida::Emitir(Evento::Desconectado),
                    Salida::Emitir(Evento::Buscando),
                ];
            }
            return Vec::new();
        }

        if self.ahora.saturating_sub(self.ultima_linea_en) > SILENCIO_MS {
            self.ping_en = Some(self.ahora);
            return vec![Salida::Mandar("ACK\n".into())];
        }

        Vec::new()
    }

    fn tic_persecucion(&mut self) -> Vec<Salida> {
        let Some(cuando) = self.persecucion_en else { return Vec::new() };
        if !self.provisional || self.ahora < cuando {
            return Vec::new();
        }

        if self.persecucion_intentos < PERSECUCION_ACKS {
            self.persecucion_intentos += 1;
            self.persecucion_en = Some(self.ahora + PERSECUCION_CADA_MS);
            return vec![Salida::Mandar("ACK\n".into())];
        }

        // Sin presentación tras varios intentos: identidad mínima. Es mejor una app en
        // modo básico que una que dice «Desconectado» con el teclado escribiendo. No
        // lleva ni FW ni banderas, así que la app degradará a lo que sabía hacer un
        // firmware antiguo.
        self.provisional = false;
        self.persecucion_en = None;
        self.estado = Estado::Conectado;
        self.siguiente_ack_en = None;

        let mut info = DeviceInfo {
            device: "ORBY_V4".into(),
            campos: Default::default(),
            raw: "ORBY_V4 (detectado por telemetría)".into(),
        };
        info.campos.insert("keys".into(), "12".into());
        info.campos.insert("oleds".into(), "10".into());
        self.info = Some(info.clone());

        vec![Salida::Emitir(Evento::Conectado(Box::new(info)))]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SALUDO: &str = "ORBY_V4:FW=4.5:KEYS=12:OLEDS=10:ENCODERS=2:HOSTAPP=1:MODE=NORMAL";
    const SALUDO_VIEJO: &str = "ORBY_V4:FW=2.0:KEYS=12:OLEDS=10:MODE=NORMAL";

    fn mando(s: &[Salida]) -> Vec<String> {
        s.iter()
            .filter_map(|x| match x {
                Salida::Mandar(c) => Some(c.clone()),
                _ => None,
            })
            .collect()
    }

    fn abierta() -> Maquina {
        let mut m = Maquina::new();
        m.al_abrir();
        m
    }

    #[test]
    fn saluda_a_los_400_ms_y_no_antes() {
        let mut m = abierta();
        assert!(mando(&m.al_pasar_tiempo(399)).is_empty());
        assert_eq!(mando(&m.al_pasar_tiempo(1)), vec!["ACK\n"]);
    }

    #[test]
    fn insiste_una_segunda_vez_antes_de_rendirse() {
        // Con un solo intento se descartaba el puerto bueno cuando el teclado estaba
        // ocupado arrancando las pantallas.
        let mut m = abierta();
        assert_eq!(mando(&m.al_pasar_tiempo(400)), vec!["ACK\n"]);
        assert!(mando(&m.al_pasar_tiempo(1199)).is_empty());
        assert_eq!(mando(&m.al_pasar_tiempo(1)), vec!["ACK\n"], "debería insistir");
    }

    #[test]
    fn si_no_contesta_a_los_dos_ack_suelta_el_puerto_sin_anunciar_desconexion() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_pasar_tiempo(1200);
        let salidas = m.al_pasar_tiempo(1200);

        assert!(salidas.contains(&Salida::SoltarPuerto));
        // Nunca llegó a estar conectado: anunciar «desconectado» haría a la interfaz
        // enseñar un aviso por cada puerto ajeno del PC.
        assert!(!salidas.contains(&Salida::Emitir(Evento::Desconectado)));
        assert_eq!(m.estado(), Estado::Buscando);
    }

    #[test]
    fn con_el_saludo_se_conecta_y_anuncia_la_app() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);

        let salidas = m.al_recibir(SALUDO);
        assert_eq!(m.estado(), Estado::Conectado);
        assert_eq!(mando(&salidas), vec!["HOST_APP:1\n"]);

        let conectado = salidas.iter().any(|s| matches!(s, Salida::Emitir(Evento::Conectado(_))));
        assert!(conectado, "debería anunciar la conexión");
        assert_eq!(m.info().unwrap().get("fw"), Some("4.5"));
    }

    #[test]
    fn un_firmware_sin_hostapp_no_recibe_el_aviso() {
        // La bandera manda sobre la versión: si el teclado no dice HOSTAPP=1, mandárselo
        // sería un comando desconocido.
        let mut m = abierta();
        m.al_pasar_tiempo(400);

        let salidas = m.al_recibir(SALUDO_VIEJO);
        assert!(mando(&salidas).is_empty());
        assert_eq!(m.estado(), Estado::Conectado);
    }

    #[test]
    fn el_saludo_no_se_reenvia_como_dato() {
        // device.js no lo espera por ahí; lo recibe como evento de conexión.
        let mut m = abierta();
        m.al_pasar_tiempo(400);

        let salidas = m.al_recibir(SALUDO);
        let hay_datos = salidas.iter().any(|s| matches!(s, Salida::Emitir(Evento::Datos(_))));
        assert!(!hay_datos);
    }

    #[test]
    fn toda_la_telemetria_llega_al_renderer() {
        // device.js depende de recibirlo TODO, incluida la no solicitada.
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir(SALUDO);

        for linea in ["KEY_EV:3:1", "ENC:1:-2", "EV:CTX:0:1:3", "PROF:0:END", "SAVE:OK"] {
            let salidas = m.al_recibir(linea);
            assert!(
                salidas.contains(&Salida::Emitir(Evento::Datos(linea.to_string()))),
                "«{linea}» debería llegar al renderer"
            );
        }
    }

    #[test]
    fn el_vigilante_pregunta_a_los_12_segundos_de_silencio() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir(SALUDO);

        assert!(mando(&m.al_pasar_tiempo(12_000)).is_empty());
        assert_eq!(mando(&m.al_pasar_tiempo(1)), vec!["ACK\n"]);
    }

    #[test]
    fn si_contesta_al_vigilante_la_conexion_sigue_viva() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir(SALUDO);
        m.al_pasar_tiempo(12_001);

        // Contesta con el saludo, que es lo que responde de verdad a un ACK.
        let salidas = m.al_recibir(SALUDO);
        // Y el aviso se renueva, porque en el teclado caduca a los 25 s.
        assert_eq!(mando(&salidas), vec!["HOST_APP:1\n"]);

        // Con la conexión ya hecha NO se vuelve a anunciar: la app se resincronizaría
        // entera cada pocos segundos.
        let reanuncia = salidas.iter().any(|s| matches!(s, Salida::Emitir(Evento::Conectado(_))));
        assert!(!reanuncia, "no debe re-anunciar una conexión que ya estaba");

        assert!(m.al_pasar_tiempo(10_000).is_empty(), "la conexión debería seguir viva");
        assert_eq!(m.estado(), Estado::Conectado);
    }

    #[test]
    fn si_no_contesta_al_vigilante_se_suelta_la_conexion() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir(SALUDO);

        m.al_pasar_tiempo(12_001); // manda el ACK
        assert!(m.al_pasar_tiempo(4_000).is_empty(), "aún dentro de plazo");

        let salidas = m.al_pasar_tiempo(1);
        assert!(salidas.contains(&Salida::Emitir(Evento::Desconectado)));
        assert_eq!(m.estado(), Estado::Buscando);

        // Y se suelta el puerto, no solo se anuncia. Quien ejecuta las salidas solo lo
        // cierra cuando se lo piden, y el rastreo no abre nada mientras haya uno cogido:
        // sin esto la app se queda enseñando «buscando» sin buscar.
        assert!(
            salidas.contains(&Salida::SoltarPuerto),
            "una conexión muerta tiene que soltar su puerto"
        );
    }

    #[test]
    fn una_conexion_en_reposo_no_se_cae_sola() {
        // El fallo que más se nota: dejar la app abierta sin tocar nada y encontrarla
        // «Desconectado». Cinco minutos de silencio con el teclado contestando.
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir(SALUDO);

        for _ in 0..30 {
            for salida in m.al_pasar_tiempo(10_000) {
                if let Salida::Mandar(c) = salida {
                    assert_eq!(c, "ACK\n");
                    m.al_recibir(SALUDO); // el teclado siempre contesta al ACK
                }
            }
        }

        assert_eq!(m.estado(), Estado::Conectado, "no debería haberse caído sola");
    }

    #[test]
    fn con_telemetria_pero_sin_saludo_insiste_y_acaba_con_identidad_minima() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);

        // Llega telemetría: hay teclado, aunque no se haya presentado.
        assert_eq!(mando(&m.al_recibir("KEY_EV:3:1")), vec!["ACK\n"]);

        // Insiste tres veces más.
        for _ in 0..3 {
            assert_eq!(mando(&m.al_pasar_tiempo(600)), vec!["ACK\n"]);
        }

        // Y se rinde con lo mínimo: mejor una app en modo básico que uno que dice
        // «Desconectado» con el teclado escribiendo.
        let salidas = m.al_pasar_tiempo(600);
        let conectado = salidas.iter().any(|s| matches!(s, Salida::Emitir(Evento::Conectado(_))));
        assert!(conectado);
        assert_eq!(m.estado(), Estado::Conectado);

        let info = m.info().unwrap();
        assert_eq!(info.get("keys"), Some("12"));
        // Sin FW: la app degradará a lo que sabía hacer un firmware antiguo.
        assert_eq!(info.get("fw"), None);
    }

    #[test]
    fn un_puerto_del_que_llega_telemetria_no_se_suelta_nunca() {
        // Corrección respecto a electron/serial.js: allí la cadena del saludo suelta el
        // puerto a los 2,8 s aunque haya llegado telemetría, y se lleva por delante la
        // persecución que estaba a punto de conectar. Si el teclado está escribiendo, el
        // puerto es el bueno.
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir("KEY_EV:3:1");

        // Se deja pasar de largo el plazo en el que la cadena del saludo se rendía.
        for _ in 0..10 {
            let salidas = m.al_pasar_tiempo(600);
            assert!(
                !salidas.contains(&Salida::SoltarPuerto),
                "no se puede soltar un puerto del que llega telemetría"
            );
        }

        assert_eq!(m.estado(), Estado::Conectado);
    }

    #[test]
    fn mientras_se_persigue_el_saludo_no_se_manda_ack_por_duplicado() {
        // Las dos cadenas mandaban ACK a la vez en varios instantes.
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir("KEY_EV:3:1");

        for _ in 0..8 {
            let acks = mando(&m.al_pasar_tiempo(600));
            assert!(acks.len() <= 1, "se han mandado {} ACK de golpe", acks.len());
        }
    }

    #[test]
    fn si_el_saludo_llega_tarde_sustituye_a_la_identidad_minima() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir("KEY_EV:3:1");

        // Contesta al segundo intento.
        m.al_pasar_tiempo(600);
        let salidas = m.al_recibir(SALUDO);

        assert!(salidas.iter().any(|s| matches!(s, Salida::Emitir(Evento::Conectado(_)))));
        assert_eq!(m.info().unwrap().get("fw"), Some("4.5"), "debe ganar la identidad real");

        // Y ya no quedan intentos sueltos dando vueltas.
        assert!(mando(&m.al_pasar_tiempo(5_000)).is_empty());
    }

    #[test]
    fn desenchufar_el_teclado_anuncia_la_desconexion() {
        let mut m = abierta();
        m.al_pasar_tiempo(400);
        m.al_recibir(SALUDO);

        let salidas = m.al_cerrarse();
        assert!(salidas.contains(&Salida::Emitir(Evento::Desconectado)));
        assert!(salidas.contains(&Salida::Emitir(Evento::Buscando)));
        assert_eq!(m.estado(), Estado::Buscando);
    }

    #[test]
    fn cerrar_un_puerto_que_nunca_conecto_no_anuncia_desconexion() {
        let mut m = abierta();
        let salidas = m.al_cerrarse();
        assert!(!salidas.contains(&Salida::Emitir(Evento::Desconectado)));
    }
}
