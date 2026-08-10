// Con qué firmware sabe hablar esta versión de la app.
//
// Hasta ahora la respuesta estaba repartida: un `parseFloat(info.fw) < 3` en
// main.js, un `info.macros !== '1'` en macros-store.js, un `info.hash === '1'`
// más allá, y la lista de qué trajo cada versión solo en el CLAUDE.md. Cada
// función nueva del firmware añadía una comprobación más en un sitio distinto,
// y el usuario no tenía forma de saber si le tocaba flashear.
//
// Aquí está en un único sitio: qué versiones se admiten, qué trajo cada una y
// qué se pierde con la que tenga el teclado delante. La regla al añadir una
// función al firmware es tocar tres cosas en el mismo commit:
//   1. include/orby_version.h  — subir la versión
//   2. este fichero            — la entrada en FEATURES y FW_RECOMMENDED
//   3. docs/COMPATIBILIDAD.md  — la tabla que lee una persona
//
// Ojo con la diferencia entre versión y bandera del handshake: la versión dice
// qué firmware es, las banderas (MACROS=1, HASH=1, MAXMACROS=64) dicen qué
// trae *este* aparato. Cuando el handshake lo anuncia, se prefiere la bandera:
// sobrevive a un firmware a medio camino o recompilado con opciones distintas.

// La versión más antigua con la que la app hace algo útil. Por debajo no hay
// GET_STATE ni GET_PROFILE: no se puede ni leer la configuración.
export const FW_MIN = '2.0';

// La que trae este repositorio (include/orby_version.h). Con una anterior la
// app funciona recortada; con una posterior, la app es la vieja.
export const FW_RECOMMENDED = '4.5';

// Qué versión hace falta para cada función, y qué bandera del handshake la
// confirma cuando el firmware la anuncia.
export const FEATURES = {
  read:       { since: '2.0', label: 'leer la configuración del teclado' },
  profiles:   { since: '3.0', label: 'crear y borrar perfiles' },
  superDial:  { since: '3.0', label: 'mandos en la capa SUPER' },
  perProfileWheel: { since: '3.0', label: 'rueda ajustable por perfil' },
  pages:      { since: '4.0', label: 'páginas dentro de un perfil', flag: 'maxpages' },
  macros:     { since: '4.0', label: 'secuencias reproducidas por el teclado', flag: 'macros' },
  hash:       { since: '4.1', label: 'sincronización por huella y precarga de iconos', flag: 'hash' },
  bootsel:    { since: '4.2', label: 'actualizar el firmware desde la app sin tocar el botón', flag: 'bootsel' },
  profileIcon: { since: '4.4', label: 'icono propio para cada perfil', flag: 'picon' },
  hostApp:    { since: '4.5', label: 'tachar en las pantallas las teclas que necesitan la app', flag: 'hostapp' },
};

// Quien manda el HOST_APP:1 es electron/serial.js, en el proceso principal, y
// allí no se puede importar este fichero: mira la bandera `hostapp` del
// handshake a pelo. Es la misma regla de siempre —la bandera manda—, solo que
// escrita dos veces.

// El `MAXMACROS` del handshake (firmware 4.2) no entra en FEATURES a propósito:
// no es una función que el usuario gane o pierda, es un límite que el teclado
// declara. Lo lee macros-store.js, y con un firmware anterior se asume el 64 de
// siempre. Meterlo aquí solo serviría para avisar de que "falta" algo que no se
// nota.

// "4.10" es posterior a "4.9": comparar como número decimal (el parseFloat de
// antes) los ponía al revés.
export function compareFw(a, b) {
  const partsOf = (v) => String(v ?? '').split('.').map((n) => parseInt(n, 10) || 0);
  const x = partsOf(a);
  const y = partsOf(b);
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    const d = (x[i] || 0) - (y[i] || 0);
    if (d) return d < 0 ? -1 : 1;
  }
  return 0;
}

export function fwOf(info) {
  return info?.fw || '0.0';
}

// Si el teclado que hay delante trae una función. La bandera manda sobre la
// versión cuando el handshake la trae: un firmware puede anunciar 4.2 y estar
// compilado sin secuencias.
export function supports(info, feature) {
  const f = FEATURES[feature];
  if (!f) return false;
  if (f.flag && info && f.flag in info) {
    const raw = info[f.flag];
    return raw !== '0' && raw !== '' && raw != null;
  }
  return compareFw(fwOf(info), f.since) >= 0;
}

// Lo que le falta a este teclado respecto a lo que la app sabe usar.
export function missingFeatures(info) {
  return Object.entries(FEATURES)
    .filter(([name]) => !supports(info, name))
    .map(([, f]) => f.label);
}

// El veredicto para enseñárselo al usuario. `level` es lo que decide el color y
// si la app puede seguir: 'blocked' significa que no hay nada que hacer sin
// flashear.
export function check(info) {
  const fw = fwOf(info);

  if (compareFw(fw, FW_MIN) < 0) {
    return {
      level: 'blocked', fw,
      title: `Firmware ${fw}: demasiado antiguo`,
      detail: `Esta versión de OrbyGUI necesita el firmware ${FW_MIN} o posterior. `
            + `Flashea el ${FW_RECOMMENDED} para usar el editor.`,
    };
  }

  if (compareFw(fw, FW_RECOMMENDED) > 0) {
    return {
      level: 'app-vieja', fw,
      title: `Firmware ${fw}: más nuevo que esta app`,
      detail: `OrbyGUI conoce hasta el ${FW_RECOMMENDED}. Lo que ya funciona sigue `
            + `funcionando, pero las funciones nuevas del teclado no se ven hasta `
            + `que actualices la app.`,
    };
  }

  const missing = missingFeatures(info);
  if (missing.length) {
    return {
      level: 'recortado', fw,
      title: `Firmware ${fw}: faltan funciones`,
      detail: `Sin flashear el ${FW_RECOMMENDED} no tendrás: ${missing.join(', ')}.`,
      missing,
    };
  }

  return { level: 'ok', fw, title: `Firmware ${fw}`, detail: 'Al día.' };
}
