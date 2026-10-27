// Cuándo se puede actualizar el firmware del teclado sin preguntar.
//
// Va aparte de firmware.js, sin tocar `window` ni el estado, para poder probarlo
// con node --test: es la parte que decide, y equivocarse aquí es reiniciar el
// teclado debajo de los dedos de alguien.
//
// Durante la copia el teclado desaparece unos segundos (sale como unidad USB),
// así que la regla es no hacerlo nunca mientras alguien pueda estar usándolo.
// El teclado de macros es un dispositivo HID más del PC: sus teclas, mandos y
// rueda cuentan como entrada para Windows, igual que el ratón y el teclado
// normal. Que el PC lleve un rato sin entrada es, por tanto, que nadie toca
// ninguno de los tres.

// Sin entrada durante este tiempo se da el PC por desatendido. Corto, y saltaría
// en mitad de una lectura; largo, y en un PC que se suspende no llegaría nunca.
export const INACTIVO_MS = 10 * 60 * 1000;

// Cada cuánto se mira. La inactividad se mide en minutos: no hace falta más.
export const VUELTA_MS = 60 * 1000;

// Cada cuánto se vuelve a preguntar a GitHub con la app abierta. La comprobación
// normal es al conectar el teclado, y la app puede pasar días abierta en la bandeja.
export const RECOMPROBAR_MS = 6 * 60 * 60 * 1000;

// null si se puede actualizar ya; si no, por qué no (sirve para el registro y
// para los tests, no se enseña).
export function motivoParaEsperar({
  activado, conectado, disponible, version, ocupado, sinGuardar, bootsel, inactivoMs, fallida,
}) {
  if (!activado) return 'desactivado';
  if (!conectado) return 'sin teclado';
  if (!disponible || !version) return 'nada que instalar';
  if (ocupado) return 'ya hay una actualización en marcha';
  // El reinicio se lleva lo que solo esté en RAM, variaciones por app incluidas.
  if (sinGuardar) return 'cambios sin guardar';
  // Sin BOOTSEL por serie habría que desenchufar el teclado con el botón pulsado:
  // eso no se le puede pedir a quien no está delante.
  if (!bootsel) return 'el firmware no sabe reiniciarse solo';
  // Una versión que ya falló en esta sesión no se reintenta sola: repetirla cada
  // diez minutos sería reiniciar el teclado en bucle. Queda el botón de Ajustes.
  if (fallida === version) return 'ya falló en esta sesión';
  if (inactivoMs == null) return 'no se sabe si el PC está en uso';
  if (inactivoMs < INACTIVO_MS) return 'el PC está en uso';
  return null;
}
