// Cuándo toca enseñar un consejo y cuál. Sin DOM ni almacenamiento, para poder
// comprobarlo con node --test (la parte con efectos está en tips-notice.js).

export const MIN_USO_MS = 2 * 60 * 1000;
export const ENTRE_AVISOS_MS = 24 * 60 * 60 * 1000;

// Todas las condiciones son "no molestar": el aviso sale solo si nada de lo que el
// usuario está haciendo se interrumpe.
export function puedeAvisar(c) {
  if (!c.activo || !c.visible || !c.conectado || c.sinGuardar || c.hayAviso) return false;
  if (c.ahora - c.inicioApp < MIN_USO_MS) return false;
  if (c.ultimoAviso == null) return true;
  // Un último aviso en el futuro es un reloj que se retrasó: si contara, los consejos
  // se quedarían mudos hasta que el reloj alcanzara esa fecha.
  if (c.ultimoAviso > c.ahora) return true;
  return c.ahora - c.ultimoAviso >= ENTRE_AVISOS_MS;
}

// El primero de la lista que no se ha visto y que este teclado sabe hacer. Los ids vistos
// de consejos que ya no existen no estorban: solo se compara con los de la lista.
export function siguienteConsejo(tips, vistos, soporta) {
  const hechos = new Set(Array.isArray(vistos) ? vistos : []);
  const t = tips.find((x) => !hechos.has(x.id) && (!x.requiere || soporta(x.requiere)));
  return t ? t.id : null;
}
