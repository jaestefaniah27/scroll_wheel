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
//   2. compat.json             — la entrada del firmware nuevo (y `rev` + 1);
//                                en FEATURES (aquí) solo si trae una función
//   3. docs/COMPATIBILIDAD.md  — la tabla que lee una persona
// test/compat-tabla.test.mjs falla si falta la entrada del paso 2.
//
// Ojo con la diferencia entre versión y bandera del handshake: la versión dice
// qué firmware es, las banderas (MACROS=1, HASH=1, MAXMACROS=64) dicen qué
// trae *este* aparato. Cuando el handshake lo anuncia, se prefiere la bandera:
// sobrevive a un firmware a medio camino o recompilado con opciones distintas.


import { compareFw, masReciente, fwRecomendado as topeDe, appNecesaria, validar } from './compat-tabla.mjs';
import base from './compat.json';

export { compareFw };

// La tabla de qué firmware ofrece cada versión de la app. Va en compat.json, y
// ese mismo fichero se baja de GitHub al arrancar: así sacar un firmware que la
// app ya sabe manejar no obliga a sacar una app nueva (ver docs/COMPATIBILIDAD.md).
// Dentro de la app queda la copia con la que se compiló, por si no hay red.
const URL_TABLA = 'https://raw.githubusercontent.com/jaestefaniah27/scroll_wheel/main/'
  + 'KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/src/compat.json';
const CLAVE_CACHE = 'orby.compat.tabla';
const ESPERA_RED_MS = 5000;

// Lo define vite.config.mjs desde package.json. Sin él (un harness sin vite) la
// tabla no puede decidir nada y se queda con el mínimo.
const VERSION_APP = typeof __APP_VERSION__ === 'string' ? __APP_VERSION__ : '0.0';

let tabla = base;
let carga = null;

function leerCache() {
  try {
    return JSON.parse(localStorage.getItem(CLAVE_CACHE));
  } catch {
    return null;
  }
}

function guardarCache(t) {
  try {
    localStorage.setItem(CLAVE_CACHE, JSON.stringify(t));
  } catch { /* sin almacenamiento (ventana privada) se vuelve a bajar la próxima */ }
}

// Idempotente: lo llaman main.js al arrancar y quien necesite la tabla antes
// de decidir (la conexión, la comprobación de firmware). Nunca rechaza: sin red
// o con una tabla rota la app sigue con la que ya tenía.
export function tablaLista() {
  carga ??= (async () => {
    tabla = masReciente(base, leerCache()) ?? base;
    try {
      const ctl = new AbortController();
      const corte = setTimeout(() => ctl.abort(), ESPERA_RED_MS);
      const resp = await fetch(URL_TABLA, { cache: 'no-store', signal: ctl.signal });
      clearTimeout(corte);
      if (!resp.ok) throw new Error(String(resp.status));
      const remota = validar(await resp.json());
      if (!remota) throw new Error('tabla no válida');
      tabla = masReciente(tabla, remota);
      guardarCache(tabla);
    } catch (err) {
      console.warn('[compat] se usa la tabla que ya había:', err?.message ?? err);
    }
    return tabla;
  })();
  return carga;
}

// Hasta dónde llega esta app: con un firmware anterior funciona recortada, con
// uno posterior la app es la vieja. Sale de la tabla, no de una constante.
export function fwRecomendado() {
  return topeDe(tabla, VERSION_APP);
}

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
  macroStore: { since: '4.7', label: 'llevar en el teclado lo que hace cada macro, para usarlo en otro PC', flag: 'mstore' },
  animSpeed:  { since: '4.8', label: 'ajustar la velocidad de las animaciones de las pantallas', flag: 'animspeed' },
  mirror:     { since: '4.12', label: 'ver en el dashboard las animaciones de las pantallas', flag: 'mirror' },
  menuKey:    { since: '4.15', label: 'cambiar de página y abrir menús con los mandos, y dejar libre la tecla de menú', flag: 'menukey' },
  rotInvert:  { since: '4.16', label: 'invertir el sentido de cada mando y de la rueda', flag: 'rotinv' },
  menuEncoder: { since: '4.16', label: 'un mando de menú: mantenerlo, girar y soltar para elegir perfil', flag: 'menuenc' },
  macroText:  { since: '4.20', label: 'escribir un texto desde el teclado, sin la app', flag: 'macrotext' },
  intro:      { since: '4.25', label: 'elegir la intro de arranque (o que salga una al azar) y verla desde la app', flag: 'intro' },
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
  const recomendado = fwRecomendado();

  if (compareFw(fw, tabla.fwMin) < 0) {
    return {
      level: 'blocked', fw,
      title: `Firmware ${fw}: demasiado antiguo`,
      detail: `Esta versión de Orvy necesita el firmware ${tabla.fwMin} o posterior. `
            + `Flashea el ${recomendado} para usar el editor.`,
    };
  }

  if (compareFw(fw, recomendado) > 0) {
    const necesaria = appNecesaria(tabla, fw);
    return {
      level: 'app-vieja', fw,
      title: `Firmware ${fw}: más nuevo que esta app`,
      detail: `Orvy conoce hasta el ${recomendado}`
            + (necesaria ? ` y el ${fw} pide la ${necesaria}` : '')
            + `. Lo que ya funciona sigue `
            + `funcionando, pero las funciones nuevas del teclado no se ven hasta `
            + `que actualices la app.`,
    };
  }

  const missing = missingFeatures(info);
  if (missing.length) {
    return {
      level: 'recortado', fw,
      title: `Firmware ${fw}: faltan funciones`,
      detail: `Sin flashear el ${recomendado} no tendrás: ${missing.join(', ')}.`,
      missing,
    };
  }

  return { level: 'ok', fw, title: `Firmware ${fw}`, detail: 'Al día.' };
}
