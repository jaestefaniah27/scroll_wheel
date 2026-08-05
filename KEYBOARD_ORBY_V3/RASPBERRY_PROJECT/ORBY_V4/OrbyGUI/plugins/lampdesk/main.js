// Complemento: lámpara LampDesk.
//
// Habla directamente con la API HTTP del firmware de la lámpara (ver el README
// de lampDesk). No hay opcode de firmware que valga para esto: el teclado avisa
// con MACRO:<id> y es el PC quien sale a la red, así que OrbyGUI tiene que
// estar abierto (basta con el icono de la bandeja).
//
// Los incrementos se mandan como tales (`dbrightness`, `dcolor`) en vez de leer
// el estado, sumar y escribir el resultado: la lámpara resuelve la suma, y así
// el encoder no se pelea con la app Casa ni con el panel de la bandeja por
// quién tenía el valor bueno.

const HOST_POR_DEFECTO = '192.168.1.35:8080';

// Direcciones que hubo durante el desarrollo y que no llevan a ninguna parte:
// el nombre mDNS nunca llegó a existir en la red, y el valor guardado gana al
// valor por defecto, así que sin esto la acción falla con un "fetch failed" que
// no dice contra qué dirección lo intentó.
const HOSTS_MUERTOS = new Set(['lampdesk.local', 'lampdesk.local:80', 'lampdesk.local:8080']);

// Cada escritura hace que la lámpara avise por HomeKit a los iPhone
// emparejados, y esa ida y vuelta le bloquea el bucle principal unos cientos de
// ms: la petición siguiente se come la espera. Lo normal son 20-35 ms; el pico
// medido pasa de dos segundos.
const TIMEOUT_MS = 4000;

// El firmware del Orby manda un MACRO:<id> **por cada muesca** del encoder (ver
// emit_discrete en main.cpp), así que un giro rápido son veinte disparos en un
// suspiro. Sin agrupar, eso son veinte peticiones HTTP en cola contra una ESP32
// que las atiende de una en una, y el brillo llega tarde y a tirones.
// 120 ms y no menos: la lámpara resuelve una petición suelta en unos 20 ms,
// pero encadenadas cada 60 ms se le acumulan y la mediana se va a 240 ms. Como
// el incremento se agrupa, esperar un poco más no pierde ninguna muesca — solo
// las junta en una petición más gorda.
const COALESCE_MS = 120;

// Qué parámetro de la API corresponde a cada acción.
const OPS = {
  toggle:           () => ({ on: 'toggle' }),
  on:               () => ({ on: 1 }),
  off:              () => ({ on: 0 }),
  brightness_delta: (v) => ({ dbrightness: v }),
  color_delta:      (v) => ({ dcolor: v }),
};

// Las que llevan incremento: son las únicas que se pueden agrupar, porque sumar
// dos incrementos da el mismo resultado que aplicarlos por separado.
const OPS_DELTA = new Set(['brightness_delta', 'color_delta']);

module.exports = (api) => {
  let pending = null;
  let flushTimer = null;
  let inFlight = false;

  function host(settings) {
    const guardado = (settings?.host || '').trim();
    if (!guardado || HOSTS_MUERTOS.has(guardado)) return HOST_POR_DEFECTO;
    return guardado;
  }

  // Una lámpara desenchufada no puede dejar colgada la ejecución de macros: sin
  // AbortController, fetch espera al agotamiento del TCP (más de un minuto).
  async function pedir(ruta, destino) {
    const control = new AbortController();
    const timer = setTimeout(() => control.abort(), TIMEOUT_MS);
    try {
      const res = await fetch(`http://${destino}${ruta}`, { signal: control.signal });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return await res.json();
    } finally {
      clearTimeout(timer);
    }
  }

  function escribir(params, destino) {
    return pedir(`/set?${new URLSearchParams(params)}`, destino);
  }

  function programarEnvio(destino) {
    if (flushTimer || inFlight) return;
    flushTimer = setTimeout(async () => {
      flushTimer = null;
      if (!pending) return;
      const params = pending;
      pending = null;
      inFlight = true;
      try {
        await escribir(params, destino);
      } catch (err) {
        api.error(`(${destino})`, err.message);
      } finally {
        inFlight = false;
        if (pending) programarEnvio(destino);
      }
    }, COALESCE_MS);
  }

  return {
    // Lo que la app ofrece en la pestaña Multimedia de una tecla ('key'), en la
    // pulsación de un encoder ('click') y en su giro ('turn'). `step` es cuánto
    // se mueve por muesca: los dos van en el mismo 0-100 que enseña la app Casa,
    // así que 5 son 20 muescas de punta a punta.
    actions: [
      { op: 'toggle', label: 'Encender / apagar', targets: ['key', 'click'] },
      { op: 'on',     label: 'Encender',          targets: ['key', 'click'] },
      { op: 'off',    label: 'Apagar',            targets: ['key', 'click'] },
      { op: 'brightness_delta', label: 'Brillo de la lámpara', targets: ['turn'], step: 5 },
      { op: 'color_delta',      label: 'Color de la lámpara',  targets: ['turn'], step: 5 },
    ],

    settings: {
      fields: [{
        key: 'host',
        label: 'Dirección de la lámpara',
        type: 'text',
        placeholder: HOST_POR_DEFECTO,
        default: HOST_POR_DEFECTO,
      }],
      description: 'Dirección y puerto de la lámpara en la red. El 8080 lo sirve su firmware; el 80 '
                 + 'lo ocupa HomeKit dentro de la propia lámpara. Las órdenes las manda el PC, así '
                 + 'que OrbyGUI tiene que estar abierto.',
    },

    async run(step, settings) {
      const build = OPS[step.op];
      if (!build) {
        api.error(`acción "${step.op}" desconocida`);
        return;
      }

      const destino = host(settings);
      const params = build(Math.trunc(step.value) || 0);

      // Encender y apagar van sin agrupar: dos pulsaciones seguidas son dos
      // órdenes, y fundidas en una sola "alterna" se anularían entre ellas.
      if (!OPS_DELTA.has(step.op)) {
        try {
          await escribir(params, destino);
        } catch (err) {
          api.error(`(${destino})`, err.message);
        }
        return;
      }

      pending = pending || {};
      for (const [key, value] of Object.entries(params)) {
        pending[key] = Number(pending[key] || 0) + Number(value);
      }
      programarEnvio(destino);
    },

    // Comprobación desde los ajustes: pide el estado sin cambiar nada.
    //
    // Con un reintento, porque leer es inofensivo: si la lámpara estaba ocupada
    // avisando a HomeKit, el primer intento se queda sin respuesta y la tarjeta
    // de ajustes diría "no responde" con la lámpara perfectamente viva. Las
    // escrituras **no** se reintentan: un incremento repetido se aplicaría dos
    // veces.
    async test(settings) {
      const destino = host(settings);
      const leer = () => pedir('/state', destino);

      let estado;
      try {
        estado = await leer();
      } catch {
        try {
          estado = await leer();
        } catch (err) {
          return { ok: false, error: String(err.message || err) };
        }
      }
      return {
        ok: true,
        detail: `Responde: ${estado.on ? 'encendida' : 'apagada'}, brillo ${estado.brightness} %`,
      };
    },

    // Desinstalar con un giro a medio agrupar dejaría el temporizador vivo (y
    // con él el proceso sin poder cerrarse del todo).
    dispose() {
      clearTimeout(flushTimer);
      flushTimer = null;
      pending = null;
    },
  };
};
