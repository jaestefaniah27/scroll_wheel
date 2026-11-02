// Los avisos de consejos: cada minuto mira si toca enseñar uno.
//
// La decisión (cuándo y cuál) está en tips-pick.mjs, con sus tests; aquí solo se recogen
// los datos de la app y se pinta el aviso. Se pregunta por intervalo y no por eventos
// porque ninguna de las condiciones (cambios sin guardar, conexión, ventana visible) es
// la que dispara el aviso: son las que lo frenan, y tienen que cumplirse a la vez.

import { state } from './store.js';
import * as compat from './compat.js';
import { avisoConsejo } from './ui.js';
import { goTo } from './nav.js';
import { TIPS } from './tips.mjs';
import { puedeAvisar, siguienteConsejo } from './tips-pick.mjs';
import * as tipsState from './tips-state.js';

const REVISION_MS = 60 * 1000;

async function revisar(inicioApp) {
  const { activo, vistos, ultimoAviso } = await tipsState.leer();
  const ahora = Date.now();

  const hayAviso = document.querySelector('#toast-host .toast') !== null;
  // Sobre la propia vista de consejos no tiene sentido avisar de un consejo, y dibujando
  // un icono tampoco: el editor guarda lo suyo aparte y no cuenta como "sin guardar".
  const activa = (id) => document.getElementById(id)?.classList.contains('active');
  const enConsejos = activa('view-tips') || activa('view-oled');
  const ok = puedeAvisar({
    ahora, inicioApp, ultimoAviso, activo,
    visible: document.visibilityState === 'visible' && !enConsejos,
    conectado: state.connected,
    sinGuardar: state.dirty,
    hayAviso,
  });
  if (!ok) return;

  const id = siguienteConsejo(TIPS, vistos, (clave) => compat.supports(state.deviceInfo, clave));
  if (!id) return;

  // Se anota antes de enseñarlo: si algo falla después, no se repite en el minuto siguiente.
  await tipsState.guardarAviso(id, ahora);
  const tip = TIPS.find((t) => t.id === id);
  avisoConsejo({ titulo: tip.titulo, onAbrir: () => goTo('view-tips', { id }) });
}

export function iniciarAvisos() {
  const inicioApp = Date.now();
  setInterval(() => revisar(inicioApp).catch(() => {}), REVISION_MS);
}
