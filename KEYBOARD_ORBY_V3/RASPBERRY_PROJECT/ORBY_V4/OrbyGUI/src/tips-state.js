// Lo que la app recuerda de los consejos, en la configuración local del PC (no en el
// teclado: qué has leído no tiene que viajar con él). Tres claves planas, sin objeto
// anidado, para que ninguna versión anterior de la app las pise al fusionar.

async function config() {
  try {
    return (await window.orby.getConfig()) || {};
  } catch {
    return {};
  }
}

export async function leer() {
  const c = await config();
  return {
    activo: c.consejos !== false,
    vistos: Array.isArray(c.consejosVistos) ? c.consejosVistos : [],
    ultimoAviso: Number.isFinite(c.consejoUltimoAviso) ? c.consejoUltimoAviso : null,
  };
}

export async function marcarVisto(id) {
  const { vistos } = await leer();
  if (vistos.includes(id)) return vistos;
  const nuevos = [...vistos, id];
  await window.orby.setConfig({ consejosVistos: nuevos });
  return nuevos;
}

export function guardarAviso(id, ahora) {
  return leer().then(({ vistos }) => window.orby.setConfig({
    consejosVistos: vistos.includes(id) ? vistos : [...vistos, id],
    consejoUltimoAviso: ahora,
  }));
}

export function setActivo(activo) {
  return window.orby.setConfig({ consejos: Boolean(activo) });
}
