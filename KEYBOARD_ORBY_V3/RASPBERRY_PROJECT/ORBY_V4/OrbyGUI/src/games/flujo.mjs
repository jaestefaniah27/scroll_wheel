// El recorrido de una sesión de juegos, sin tocar el teclado ni el disco.
//
// Va aparte de gestor.js para poder probarlo: aquí solo hay decisiones —qué se enseña
// después de qué, y cuándo hay que cerrar el modo juego—, y todo lo que huele a puerto,
// reloj o configuración llega por `ops`.
//
//   menú ──elige juego──▶ partida ──acaba──▶ pantalla final ──┬─ OTRA VEZ ─▶ partida
//     ▲                      │ tecla 10                        ├─ MENU ─────▶ menú
//     └──────────────────────┴─────────────────────────────────┘
//   SALIR (o la tecla 10 en el menú) cierra el modo juego.
//
// `ops`:
//   menu()                 corre el menú; devuelve el resultado del motor ({ motivo, valor })
//   partida(juego)         corre una partida
//   final(juego, puntos)   registra la puntuación y corre la pantalla final
//   cerrar()               devuelve las pantallas al teclado (GAME:0)
//   juegoPorId(id)

// Lo que cada motivo de fin del motor significa para el recorrido:
//   fin         la sesión acabó sola (se eligió algo, o terminó la partida)
//   salir       el jugador pulsó la tecla 10
//   cancelado   la app paró la sesión (el botón «Parar»)
//   menu, timeout, desconectado   el teclado ya había salido del modo juego por su cuenta
const TECLADO_YA_SALIO = new Set(['menu', 'timeout', 'desconectado']);

export async function recorrer(ops, juegoInicial = null) {
  let siguiente = juegoInicial ? { t: 'jugar', juego: juegoInicial } : { t: 'menu' };
  // Solo se manda GAME:0 si el teclado sigue en el modo; si salió él, ya está fuera.
  let cerrar = true;

  try {
    for (;;) {
      if (siguiente.t === 'menu') {
        const r = await ops.menu();
        if (r.motivo !== 'fin') { cerrar = !TECLADO_YA_SALIO.has(r.motivo); break; }
        if (r.valor?.accion !== 'jugar') break;
        const juego = ops.juegoPorId(r.valor.id);
        if (!juego) break;
        siguiente = { t: 'jugar', juego };
        continue;
      }

      const { juego } = siguiente;
      const r = await ops.partida(juego);
      // La tecla 10 en mitad de una partida la abandona y vuelve al menú, sin puntuar.
      if (r.motivo === 'salir') { siguiente = { t: 'menu' }; continue; }
      if (r.motivo !== 'fin') { cerrar = !TECLADO_YA_SALIO.has(r.motivo); break; }

      const f = await ops.final(juego, r.puntos, r.detalle);
      if (f.motivo === 'salir') { siguiente = { t: 'menu' }; continue; }
      if (f.motivo !== 'fin') { cerrar = !TECLADO_YA_SALIO.has(f.motivo); break; }

      const accion = f.valor?.accion;
      if (accion === 'otra') siguiente = { t: 'jugar', juego };
      else if (accion === 'menu') siguiente = { t: 'menu' };
      else break;   // 'salir'
    }
  } finally {
    if (cerrar) await ops.cerrar();
  }
}
