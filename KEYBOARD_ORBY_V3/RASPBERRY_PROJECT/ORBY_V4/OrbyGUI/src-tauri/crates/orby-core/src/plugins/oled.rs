// Arbitraje de lo que los complementos pintan en las pantallas del teclado.
//
// Un complemento de tipo `process` (un monitor de hardware, el REC de OBS) puede empujar
// imágenes a las OLED. Eso NO desgasta la Flash: `OLED_CHUNK` escribe en la RAM del
// teclado y solo `SAVE_STATE` persiste, que es la misma vía que ya usa src/live-oled.js.
//
// Lo que sí es un límite duro es el ancho de banda. El CDC va a 115200 baudios, o sea
// unos 11,5 KB/s, y por ahí pasan TAMBIÉN las pulsaciones y los giros. Una pantalla son
// 360 bytes que viajan en cuatro `OLED_CHUNK` de ~200 caracteres: unos 800 bytes por
// imagen. Salen ~14 pantallas por segundo si no circulara nada más, y si circula, el
// teclado se vuelve pastoso.
//
// Por eso el arbitraje vive AQUÍ y no en el complemento: un programa de un tercero no
// puede tener la potestad de dejar el teclado lento. Aunque se porte mal, de aquí no sale
// más de lo que cabe.

use std::collections::HashMap;

/// Lo que ocupa una pantalla en el cable, contando los cuatro `OLED_CHUNK` y sus
/// cabeceras. Medido sobre el formato real del protocolo, no estimado a ojo.
pub const BYTES_POR_IMAGEN: u64 = 800;

/// Tope por complemento, para que uno solo no se coma el presupuesto de los demás.
///
/// Cuatro por segundo es de sobra para un panel de temperaturas o para el parpadeo del
/// REC: por encima, el ojo ya no lo distingue en una pantalla de 72×40 y solo gasta cable.
pub const IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO: u64 = 4;

/// Lo que se le deja gastar al conjunto de los complementos. El resto del enlace queda
/// para lo que no puede esperar: pulsaciones, giros y las respuestas a la propia app.
///
/// Sale de las dos constantes de arriba a propósito: así un complemento solo puede llegar
/// a su cupo, y dos se lo reparten. Si se pone a mano un número que no sea múltiplo del
/// tamaño de una imagen, el cupo por complemento deja de ser alcanzable y cuesta entender
/// por qué. Lo vigila una prueba.
pub const BYTES_POR_SEGUNDO: u64 = BYTES_POR_IMAGEN * IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO;

/// Lo que da el enlace: 115200 baudios en 8N1 son unos 11 520 bytes por segundo.
const ENLACE_BYTES_POR_SEGUNDO: u64 = 11_520;

// Comprobaciones en tiempo de COMPILACIÓN, no en un test: si alguien sube el presupuesto
// «porque el dashboard va a tirones», esto no compila. Es el sitio donde más barato sale
// enterarse, porque el síntoma real —el teclado pastoso al teclear mientras un
// complemento pinta— aparece muy lejos de esta decisión y se le echa la culpa a otra cosa.
const _: () = {
    assert!(
        BYTES_POR_SEGUNDO * 3 < ENLACE_BYTES_POR_SEGUNDO,
        "los complementos no pueden llevarse más de un tercio del enlace: por ahí pasan \
         también las pulsaciones y los giros"
    );
    assert!(
        BYTES_POR_SEGUNDO.is_multiple_of(BYTES_POR_IMAGEN),
        "el presupuesto debe ser múltiplo del tamaño de una imagen, o el cupo por \
         complemento deja de ser alcanzable y cuesta entender por qué"
    );
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Imagen {
    pub plugin: String,
    /// Hueco de pantalla en el teclado (0..=9).
    pub slot: u8,
    /// Los 360 bytes del framebuffer de 1 bit.
    pub datos: Vec<u8>,
}

pub struct Arbitro {
    ahora_ms: u64,
    /// Página que el teclado tiene puesta ahora mismo. Pintar en una tecla de otra página
    /// no se ve y gasta enlace igual.
    pagina_activa: u8,
    /// Solo la ÚLTIMA imagen de cada hueco: las intermedias no las llega a ver nadie.
    pendientes: HashMap<(String, u8), (Vec<u8>, u8)>,
    creditos: u64,
    enviadas_este_segundo: HashMap<String, u64>,
    ventana_abierta_en: u64,
    /// Para repartir con justicia cuando varios complementos compiten.
    orden: Vec<String>,
}

impl Default for Arbitro {
    fn default() -> Self {
        Self::new()
    }
}

impl Arbitro {
    pub fn new() -> Self {
        Self {
            ahora_ms: 0,
            pagina_activa: 0,
            pendientes: HashMap::new(),
            creditos: BYTES_POR_SEGUNDO,
            enviadas_este_segundo: HashMap::new(),
            ventana_abierta_en: 0,
            orden: Vec::new(),
        }
    }

    pub fn set_pagina_activa(&mut self, pagina: u8) {
        self.pagina_activa = pagina;
    }

    /// Un complemento quiere pintar. `pagina` es la del hueco al que apunta.
    ///
    /// Devuelve `false` si se descarta por no estar a la vista. Que devuelva `true` no
    /// significa que se mande ya: significa que entra en la cola.
    pub fn empujar(&mut self, plugin: &str, slot: u8, pagina: u8, datos: Vec<u8>) -> bool {
        // Nunca se fuerza un cambio de página para pintar de fondo: es la regla que ya
        // sigue live-oled.js y la que evita que un complemento te mueva el teclado bajo
        // los dedos.
        if pagina != self.pagina_activa {
            return false;
        }

        if !self.orden.iter().any(|p| p == plugin) {
            self.orden.push(plugin.to_string());
        }

        // De un mismo hueco solo importa la última: las intermedias se tiran sin
        // mandarlas. Es lo que hace que un complemento que pinta a 30 Hz no sature nada.
        self.pendientes.insert((plugin.to_string(), slot), (datos, pagina));
        true
    }

    /// Avanza el reloj y devuelve lo que toca mandar de verdad.
    pub fn avanzar(&mut self, ms: u64) -> Vec<Imagen> {
        self.ahora_ms += ms;

        if self.ahora_ms.saturating_sub(self.ventana_abierta_en) >= 1_000 {
            self.ventana_abierta_en = self.ahora_ms;
            self.creditos = BYTES_POR_SEGUNDO;
            self.enviadas_este_segundo.clear();
        }

        let mut salida = Vec::new();

        // Se recorre por orden de llegada de los complementos y se rota, para que el
        // primero en registrarse no acapare el presupuesto de todos los segundos.
        let orden = self.orden.clone();
        for plugin in &orden {
            if self.creditos < BYTES_POR_IMAGEN {
                break;
            }

            let usadas = self.enviadas_este_segundo.get(plugin).copied().unwrap_or(0);
            if usadas >= IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO {
                continue;
            }

            let mut slots: Vec<u8> = self
                .pendientes
                .keys()
                .filter(|(p, _)| p == plugin)
                .map(|(_, s)| *s)
                .collect();
            slots.sort_unstable();

            for slot in slots {
                if self.creditos < BYTES_POR_IMAGEN {
                    break;
                }
                let usadas = self.enviadas_este_segundo.get(plugin).copied().unwrap_or(0);
                if usadas >= IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO {
                    break;
                }

                if let Some((datos, _)) = self.pendientes.remove(&(plugin.clone(), slot)) {
                    self.creditos -= BYTES_POR_IMAGEN;
                    *self.enviadas_este_segundo.entry(plugin.clone()).or_insert(0) += 1;
                    salida.push(Imagen { plugin: plugin.clone(), slot, datos });
                }
            }
        }

        if !self.orden.is_empty() {
            self.orden.rotate_left(1);
        }

        salida
    }

    pub fn pendientes(&self) -> usize {
        self.pendientes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn imagen(n: u8) -> Vec<u8> {
        vec![n; 360]
    }

    #[test]
    fn de_un_mismo_hueco_solo_se_manda_la_ultima() {
        // Un complemento que pinta a 30 Hz no puede meter 30 imágenes por el cable: las
        // intermedias no las ve nadie.
        let mut a = Arbitro::new();
        for n in 0..30 {
            a.empujar("monitor", 3, 0, imagen(n));
        }

        let salida = a.avanzar(1_000);
        assert_eq!(salida.len(), 1);
        assert_eq!(salida[0].datos[0], 29, "debe quedarse con la última");
    }

    #[test]
    fn huecos_distintos_se_mandan_los_dos() {
        let mut a = Arbitro::new();
        a.empujar("monitor", 1, 0, imagen(1));
        a.empujar("monitor", 2, 0, imagen(2));

        let salida = a.avanzar(1_000);
        assert_eq!(salida.len(), 2);
        assert_eq!(salida[0].slot, 1);
        assert_eq!(salida[1].slot, 2);
    }

    #[test]
    fn lo_que_no_esta_en_la_pagina_activa_se_descarta() {
        // Pintar en una tecla de otra página no se ve y gasta enlace igual. Y NUNCA se
        // fuerza un cambio de página para pintar de fondo.
        let mut a = Arbitro::new();
        a.set_pagina_activa(0);

        assert!(!a.empujar("monitor", 1, 2, imagen(1)), "otra página: se descarta");
        assert!(a.empujar("monitor", 1, 0, imagen(1)));
        assert_eq!(a.avanzar(1_000).len(), 1);
    }

    #[test]
    fn un_complemento_no_pasa_de_su_cupo_por_segundo() {
        let mut a = Arbitro::new();
        for slot in 0..10 {
            a.empujar("monitor", slot, 0, imagen(slot));
        }

        let salida = a.avanzar(1_000);
        assert_eq!(salida.len() as u64, IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO);
        assert!(a.pendientes() > 0, "el resto espera, no se pierde");
    }

    #[test]
    fn el_cupo_se_renueva_cada_segundo() {
        let mut a = Arbitro::new();
        for slot in 0..10 {
            a.empujar("monitor", slot, 0, imagen(slot));
        }

        let primera = a.avanzar(1_000).len();
        let segunda = a.avanzar(1_000).len();

        assert_eq!(primera as u64, IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO);
        assert_eq!(segunda as u64, IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO);
    }

    #[test]
    fn el_presupuesto_global_no_se_pasa_aunque_haya_muchos_complementos() {
        // Lo que protege al teclado de un mercado lleno de dashboards.
        let mut a = Arbitro::new();
        for i in 0..8 {
            let plugin = format!("complemento-{i}");
            for slot in 0..4 {
                a.empujar(&plugin, slot, 0, imagen(slot));
            }
        }

        let salida = a.avanzar(1_000);
        let gastado = salida.len() as u64 * BYTES_POR_IMAGEN;
        assert!(
            gastado <= BYTES_POR_SEGUNDO,
            "se han gastado {gastado} bytes y el tope es {BYTES_POR_SEGUNDO}"
        );
    }

    #[test]
    fn un_complemento_pesado_no_deja_sin_sitio_a_los_demas() {
        // Justicia: el que se registró primero no puede acaparar todos los segundos.
        let mut a = Arbitro::new();

        let mut vistos_educado = 0;
        for _ in 0..10 {
            for slot in 0..10 {
                a.empujar("acaparador", slot, 0, imagen(slot));
            }
            a.empujar("educado", 0, 0, imagen(99));

            for img in a.avanzar(1_000) {
                if img.plugin == "educado" {
                    vistos_educado += 1;
                }
            }
        }

        assert!(vistos_educado >= 5, "el educado solo ha pintado {vistos_educado} veces de 10");
    }

    #[test]
    fn sin_nada_que_pintar_no_se_manda_nada() {
        let mut a = Arbitro::new();
        assert!(a.avanzar(5_000).is_empty());
    }

    #[test]
    fn el_cupo_por_complemento_es_alcanzable() {
        // La coherencia de las constantes se comprueba al compilar (ver el bloque `const`
        // de arriba). Esto solo fija que un complemento solo puede, de hecho, llegar a su
        // cupo: si el reparto cambiara y no lo alcanzara, el número mentiría.
        let mut a = Arbitro::new();
        for slot in 0..10 {
            a.empujar("monitor", slot, 0, imagen(slot));
        }
        assert_eq!(a.avanzar(1_000).len() as u64, IMAGENES_POR_SEGUNDO_Y_COMPLEMENTO);
    }
}
