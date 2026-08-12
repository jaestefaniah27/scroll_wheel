// Los valores por defecto de la configuración local y cómo se fusiona un parche.
//
// Es la misma lógica que src-tauri/src/config.rs, repetida porque aquel es CommonJS del
// proceso principal y este es ESM que corre en el navegador. Doce líneas duplicadas
// salen más baratas que un módulo neutro que carguen los dos mundos. Al tocar
// REPLACE_WHOLE o DEFAULTS aquí, hay que tocarlo también allí.
//
// Se han quitado las claves que en navegador no puede rellenar nadie (complementos,
// cambio automático por ventana): dejarlas sería prometer una función que no existe.

export const DEFAULTS = {
  // Variaciones de perfil por aplicación. En navegador nadie las escribe —no hay
  // detector de ventanas—, pero variants.js las lee al arrancar y sin la clave
  // tendría que aprender a vivir sin ella.
  profileVariants: [],

  // Secuencias. En navegador solo se editan las que el teclado sabe tocar solo
  // (ver macroDeviceEligible en views/profiles/macros-store.js); el resto se ven
  // pero no se tocan.
  macros: [],
  macrosOnDevice: null,

  // Calibración del dibujo de la rueda. Es del PC, no del firmware: depende de cómo
  // estén montados el imán y el marcador de la tapa.
  wheelDial: {
    invert: true,
    offsetDeg: 62,
    marker: 'dot',
  },

  // Espejo de lo que hay en el teclado, en el mismo formato que una copia de
  // seguridad. Permite abrir la app sin el Orby (en modo lectura) y, con él
  // conectado, no descargar los perfiles cuya huella no ha cambiado.
  deviceMirror: null,
};

// Claves que se sustituyen enteras en vez de fusionarse.
const REPLACE_WHOLE = new Set(['deviceMirror']);

export function merge(base, patch) {
  const out = { ...base };
  for (const [key, value] of Object.entries(patch || {})) {
    out[key] = (value && typeof value === 'object' && !Array.isArray(value) && !REPLACE_WHOLE.has(key))
      ? merge(base[key] || {}, value)
      : value;
  }
  return out;
}
