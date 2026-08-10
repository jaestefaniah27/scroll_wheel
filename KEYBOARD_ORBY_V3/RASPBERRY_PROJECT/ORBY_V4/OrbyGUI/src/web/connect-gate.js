// La pantalla que pide conectar el teclado.
//
// Web Serial no deja elegir puerto sin un gesto del usuario, así que la primera
// visita necesita sí o sí un botón. En las siguientes el permiso ya está concedido
// y el puerto se abre solo al cargar, sin que se vea nada de esto.

import * as serie from '../web/transport-serial.js';

const NAVEGADOR_NO_VALE =
  'Este navegador no sabe hablar con puertos serie. La WebGUI necesita Chrome, Edge '
  + 'u otro navegador basado en Chromium, en un ordenador (en móvil no existe la API).';

let host = null;

// El último motivo por el que falló una apertura. El transporte lo cuenta por su
// evento 'error', que llega antes de que pedirPuerto() devuelva false: guardarlo aquí
// es la forma de poder enseñarlo en la tarjeta en vez de un "no se pudo" a secas.
let ultimoError = null;

function pintar({ mensaje, error, ocupado }) {
  host.innerHTML = `
    <div class="gate-card">
      <h1>Orby WebGUI</h1>
      <p class="gate-desc">${mensaje}</p>
      ${error ? `<p class="gate-error">${error}</p>` : ''}
      <button class="primary-btn gate-btn" id="gate-connect" ${ocupado ? 'disabled' : ''}>
        ${ocupado ? 'Conectando…' : 'Conectar teclado'}
      </button>
      <p class="gate-note">
        Elige <strong>Orby V4 Control</strong> en la lista del navegador. Si no aparece,
        comprueba que el cable está enchufado y que no tienes abierta la app de
        escritorio: solo un programa puede usar el teclado a la vez.
      </p>
    </div>`;

  const btn = document.getElementById('gate-connect');
  if (btn) btn.addEventListener('click', conectar);
}

async function conectar() {
  pintar({ mensaje: 'Elige el teclado en la lista del navegador.', ocupado: true });
  const ok = await serie.pedirPuerto();
  if (ok) { ocultar(); return; }
  pintar({
    mensaje: 'No se ha conectado ningún teclado.',
    error: ultimoError || 'Vuelve a intentarlo.',
  });
}

export function ocultar() {
  host?.classList.add('hidden');
}

export function mostrar() {
  host?.classList.remove('hidden');
}

export function montarGate() {
  host = document.getElementById('web-gate');
  serie.on('error', (err) => { ultimoError = err; });

  if (!serie.disponible()) {
    host.classList.remove('hidden');
    host.innerHTML = `
      <div class="gate-card">
        <h1>Orby WebGUI</h1>
        <p class="gate-desc">${NAVEGADOR_NO_VALE}</p>
      </div>`;
    return;
  }

  pintar({ mensaje: 'Conecta el teclado por USB para empezar a configurarlo.' });
  host.classList.remove('hidden');

  // Con el teclado ya autorizado en una visita anterior, la conexión se hace sola y
  // esta pantalla no llega a verse.
  serie.on('connected', ocultar);
}
