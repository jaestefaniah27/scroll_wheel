// Dónde corre esta copia de la app y, por tanto, qué puede hacer.
//
// La vía de escritorio ('tauri') tiene un backend detrás: puede abrir programas, mover el
// ratón, cargar complementos y vigilar qué ventana está en primer plano. La versión de
// navegador no tiene nada de eso —solo el puerto serie del teclado—, así que hay funciones
// del editor que ahí no van a funcionar.
//
// Por eso la pregunta que se hace abajo es "¿es navegador?" y no "¿es Tauri?": lo que
// recorta funciones es la falta de backend, no cómo se llame quien lo pone. Se escribió
// así cuando había dos vías de escritorio (Electron y Tauri) y se deja igual: si algún día
// vuelve a haber dos, no hay que tocar nada.
//
// La diferencia se pregunta SIEMPRE aquí. Repartir comprobaciones del tipo
// "¿existe window.orby.recorder?" por las vistas es cómo se acaba con una función
// medio deshabilitada en cinco sitios y viva en el sexto.

let current = 'tauri';

export function setPlatform(name) {
  current = name;
}

export function isWeb() {
  return current === 'web';
}

export function isTauri() {
  return current === 'tauri';
}

// Lo que necesita un proceso con permisos de escritorio. Cada nombre es lo que
// preguntan las vistas, no el módulo que lo implementa: si mañana las secuencias
// las tocara el navegador, se quita de aquí y ya está.
const PC_ONLY = new Set([
  'openApp',      // abrir una aplicación o un archivo
  'text',         // escribir un texto por el teclado del sistema
  'power',        // suspender, apagar, bloquear
  'recorder',     // grabar y reproducir ratón y teclado
  'plugins',      // complementos
  'autoProfile',  // cambio de perfil según la ventana en primer plano
  'autostart',    // arrancar con la sesión de Windows
  'appUpdate',    // actualizar la propia app
  'firmwareUpdate', // actualizar el firmware del teclado
  'windowChrome', // minimizar / maximizar / cerrar
  'pcSequences',  // secuencias con pasos que solo sabe tocar el PC
]);

export function can(feature) {
  return !isWeb() || !PC_ONLY.has(feature);
}
