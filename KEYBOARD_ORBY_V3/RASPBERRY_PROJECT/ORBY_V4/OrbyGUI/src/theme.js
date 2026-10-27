// Apariencia clara u oscura.
//
// Por defecto sigue al sistema, como cualquier app de Apple: si Windows (o el
// Mac, en la WebGUI) cambia de modo a media tarde, la app cambia con él sin
// reiniciar. Desde Ajustes se puede fijar una de las dos.
//
// El tema resuelto se escribe en <html data-theme>, y la hoja de estilos solo
// mira ese atributo: así hay un único bloque de colores claros y no dos copias
// (una para la media query y otra para la elección manual) que acabarían
// desincronizándose.
//
// La preferencia va en localStorage y no en la configuración del PC: hay que
// leerla de forma síncrona antes del primer pintado, y getConfig() es una
// promesa. Es cosa de este navegador o de esta instalación, no del teclado.

const CLAVE = 'orby.apariencia';
const OPCIONES = ['auto', 'light', 'dark'];
const sistema = window.matchMedia('(prefers-color-scheme: light)');

export function preferencia() {
  try {
    const v = localStorage.getItem(CLAVE);
    return OPCIONES.includes(v) ? v : 'auto';
  } catch {
    // Navegación privada o almacenamiento bloqueado: se sigue al sistema.
    return 'auto';
  }
}

export function aplicar() {
  const p = preferencia();
  const tema = p === 'auto' ? (sistema.matches ? 'light' : 'dark') : p;
  document.documentElement.dataset.theme = tema;
}

export function elegir(p) {
  if (!OPCIONES.includes(p)) return;
  try { localStorage.setItem(CLAVE, p); } catch { /* se aplica igual, sin recordarlo */ }
  aplicar();
}

sistema.addEventListener('change', aplicar);
aplicar();
