// Cuándo se puede actualizar el firmware del teclado sin preguntar.
//
// Va aparte de firmware.js, sin tocar `window` ni el estado, para poder probarlo
// con node --test: es la parte que decide, y equivocarse aquí es reiniciar el
// teclado debajo de los dedos de alguien.
//
// Durante la copia el teclado desaparece unos segundos (sale como unidad USB),
// así que la regla es no hacerlo mientras alguien lo esté usando. Lo único que
// molesta es perder el teclado de macros, así que se mira SOLO su actividad
// (teclas, mandos, rueda y menús), no la del PC: con el ordenador en uso, pero
// sin tocar el teclado, se puede actualizar. Hasta la 1.1.2 se esperaba a que el
// PC entero llevara 10 minutos sin entrada, y con un PC que se usa todo el día
// no llegaba nunca.

// Sin tocar el teclado durante este tiempo se da por desatendido. Es lo que
// separa «lo ha dejado» de «está pensando qué pulsar». Un teclado recién
// conectado no espera: quien lo enchufa ya cuenta con que tarde unos segundos
// en estar listo, y si lo toca antes de que empiece, entonces sí rige esto.
export const INACTIVO_MS = 2 * 60 * 1000;

// Cada cuánto se mira. Con el umbral en minutos, medio minuto basta y la
// actualización empieza como mucho 30 s tarde.
export const VUELTA_MS = 30 * 1000;

// Qué líneas de la telemetría del teclado son «alguien lo ha tocado». El resto
// (EV:SCR, EV:INV, EV:DISP, STATE:...) lo manda el teclado solo, sin que nadie
// lo toque: contarlas lo mantendría siempre en uso.
export function esActividadDelTeclado(linea) {
  return /^(KEY_EV|ENC|ENC_SW|WHEEL|MODE|EV:CTX):/.test(linea);
}

// Cada cuánto se vuelve a preguntar a GitHub con la app abierta. La comprobación
// normal es al conectar el teclado, y la app puede pasar días abierta en la bandeja.
export const RECOMPROBAR_MS = 6 * 60 * 60 * 1000;

// null si se puede actualizar ya; si no, por qué no (sirve para el registro y
// para los tests, no se enseña).
export function motivoParaEsperar({
  activado, conectado, disponible, version, ocupado, sinGuardar, sincronizando, bootsel, inactivoMs, fallida,
}) {
  if (!activado) return 'desactivado';
  if (!conectado) return 'sin teclado';
  if (!disponible || !version) return 'nada que instalar';
  if (ocupado) return 'ya hay una actualización en marcha';
  // Leer o subir configuración al teclado (perfiles, macros) a medias y
  // reiniciarlo cortaría esa escritura.
  if (sincronizando) return 'sincronizando con el teclado';
  // El reinicio se lleva lo que solo esté en RAM, variaciones por app incluidas.
  if (sinGuardar) return 'cambios sin guardar';
  // Sin BOOTSEL por serie habría que desenchufar el teclado con el botón pulsado:
  // eso no se le puede pedir a quien no está delante.
  if (!bootsel) return 'el firmware no sabe reiniciarse solo';
  // Una versión que ya falló en esta sesión no se reintenta sola: repetirla cada
  // vuelta sería reiniciar el teclado en bucle. Queda el botón de Ajustes.
  if (fallida === version) return 'ya falló en esta sesión';
  if (inactivoMs == null) return 'no se sabe si el teclado está en uso';
  if (inactivoMs < INACTIVO_MS) return 'el teclado está en uso';
  return null;
}
