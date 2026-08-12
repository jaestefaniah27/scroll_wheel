// ¿Se puede flashear el teclado ahora mismo, sin avisar a nadie?
//
// Está separado de `firmware-auto.js` —que es quien recoge el estado y quien dispara— para
// poder probarlo sin navegador, sin teclado y sin esperar cinco minutos de reloj. Es la
// misma razón por la que `coalesce.rs` recibe el tiempo por parámetro en vez de leerlo:
// esta decisión es la que puede dejar un teclado a medio firmware, y merece tests.
//
// `.mjs` y sin imports a propósito, como `split-lines.mjs` y `config-merge.mjs`: así lo
// carga `node --test` directamente.

// Cuánto tiene que llevar el teclado sin actividad. Cinco minutos no es un número afinado:
// es «se ha ido a por café», que es exactamente el momento que se busca.
export const OCIO_MS = 5 * 60 * 1000;

/**
 * Devuelve **el motivo por el que NO** se puede flashear, o `null` si se puede.
 *
 * Se devuelve el motivo en texto y no un booleano porque acaba en la consola: con algo que
 * pasa solo, «no ha pasado» sin explicación es imposible de diagnosticar.
 *
 * El orden importa poco para el resultado pero mucho para el mensaje: se comprueba primero
 * lo que es un «aquí no hay nada que hacer» (sin teclado, sin versión nueva) y después lo
 * que es «ahora no toca», que es lo que de verdad interesa leer en el log.
 */
export function porQueNo(s) {
  if (!s.conectado || !s.deviceInfo) return 'no hay teclado conectado';
  if (!s.disponible || !s.ultimaVersion) return 'no hay firmware nuevo';
  if (s.ocupado) return 'ya hay algo en marcha';
  if (s.ultimaVersion === s.versionFallida) return 'esta versión ya falló en esta sesión';
  if (s.automatico === false) return 'el automático está apagado';

  // El reinicio en el cargador se lleva por delante lo que solo esté en RAM, y las
  // variaciones por aplicación viven exactamente ahí (ver variants.js). Es el mismo motivo
  // por el que el botón manual de Ajustes obliga a guardar antes de dejarte seguir.
  if (s.sinGuardar) return 'hay cambios sin guardar en Flash';

  if (s.grabando) return 'la grabadora está grabando';
  if (s.reproduciendo) return 'hay una secuencia reproduciéndose';

  // `quietoMs` lo calcula quien llama a partir de su propio reloj: aquí no se lee la hora,
  // que es lo que permite probar los cinco minutos sin esperarlos.
  if (s.quietoMs < OCIO_MS) {
    return `el teclado se ha usado hace ${Math.round(s.quietoMs / 1000)} s`;
  }

  return null;
}
