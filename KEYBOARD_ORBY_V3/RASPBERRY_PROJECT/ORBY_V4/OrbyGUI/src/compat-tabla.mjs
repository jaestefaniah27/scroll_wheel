// La parte de compat.js que decide sin tocar red ni disco, para poder probarla
// con `node --test`. La tabla de compatibilidad (compat.json) la baja la app de
// GitHub al arrancar; aquí solo se valida y se interpreta.
//
// Por qué existe: hasta ahora el tope de firmware que la app ofrecía
// (`FW_RECOMMENDED`) iba compilado dentro, así que cada firmware nuevo obligaba
// a sacar app nueva solo para que la app supiera que existía. Con la tabla
// fuera, un firmware que no cambia nada de lo que la app usa se publica solo.

// "4.10" es posterior a "4.9": comparar como número decimal (el parseFloat de
// antes) los ponía al revés. Sirve igual para versiones de app ("1.1.4").
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

export const SCHEMA = 1;

// Una tabla que no cuadra se descarta entera: una mitad bien formada y otra
// rota daría un tope de firmware distinto del que quiso quien la escribió, y
// ofrecer de más es justo lo que esta tabla existe para evitar.
export function validar(t) {
  if (!t || typeof t !== 'object') return null;
  if (t.schema !== SCHEMA) return null;
  if (!Number.isInteger(t.rev) || t.rev < 1) return null;
  if (typeof t.fwMin !== 'string' || !t.fwMin) return null;
  if (!t.firmware || typeof t.firmware !== 'object') return null;
  for (const [fw, entrada] of Object.entries(t.firmware)) {
    if (!/^\d+(\.\d+)*$/.test(fw)) return null;
    if (!entrada || !/^\d+(\.\d+)*$/.test(String(entrada.minApp))) return null;
  }
  return t;
}

// Entre la que va dentro de la app, la que quedó guardada de otra vez y la
// recién bajada, manda la de `rev` más alto. Sin esto, una copia guardada por
// una app vieja ganaría a la de la app nueva recién instalada.
export function masReciente(...tablas) {
  let mejor = null;
  for (const t of tablas) {
    const ok = validar(t);
    if (ok && (!mejor || ok.rev > mejor.rev)) mejor = ok;
  }
  return mejor;
}

// El firmware más alto que esta versión de la app sabe ofrecer: el último de la
// tabla cuya `minApp` ya cumple. Un firmware que no sale en la tabla no se
// ofrece, y el veredicto de conexión lo trata como «más nuevo que esta app».
export function fwRecomendado(tabla, appVersion) {
  let elegido = null;
  for (const [fw, { minApp }] of Object.entries(tabla.firmware)) {
    if (compareFw(minApp, appVersion) > 0) continue;
    if (!elegido || compareFw(fw, elegido) > 0) elegido = fw;
  }
  return elegido ?? tabla.fwMin;
}

// La app que hace falta para aprovechar ese firmware, o null si la tabla no lo
// conoce todavía.
export function appNecesaria(tabla, fw) {
  return tabla.firmware[fw]?.minApp ?? null;
}
