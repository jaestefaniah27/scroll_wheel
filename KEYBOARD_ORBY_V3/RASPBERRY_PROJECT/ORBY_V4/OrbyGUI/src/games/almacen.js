// Dónde se guardan las puntuaciones: la configuración local del PC (`cfg.juegos`), que en
// la versión de navegador va a IndexedDB. Un único sitio porque lo leen la vista de Juegos
// y el gestor, que escribe al acabar cada partida aunque la vista no esté abierta.

let datos = null;

export async function cargar() {
  if (datos) return datos;
  try {
    const cfg = await window.orby.getConfig();
    datos = cfg?.juegos || {};
  } catch {
    datos = {};
  }
  return datos;
}

// Lo guardado de un juego, o undefined si nunca se ha jugado. Hay que haber llamado a
// cargar() antes.
export function leer(id) {
  return datos?.[id];
}

export async function guardar(id, estado) {
  datos ??= {};
  datos[id] = estado;
  await window.orby.setConfig({ juegos: { [id]: estado } });
}
