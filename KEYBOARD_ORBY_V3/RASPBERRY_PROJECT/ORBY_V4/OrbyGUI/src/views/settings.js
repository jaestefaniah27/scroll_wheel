// Ajustes: dispositivo, persistencia y la propia aplicación.

import * as device from '../device.js';
import * as updater from '../updater.js';
import { state, markDirty, notify, syncFromDevice, pageCountOf, withPinnedPage } from '../store.js';
import { ROTARY_TYPES, ROTARY_SLOTS } from '../hid-keys.js';
import { toast, requireDevice, confirmar } from '../ui.js';
import { runExport, runImport } from '../backup.js';
import * as cache from '../oled-cache.js';
import * as wheelDial from '../wheel-dial.js';
import * as plugins from '../plugins.js';
import * as compat from '../compat.js';
import * as firmware from '../firmware.js';
import * as platform from '../platform.js';
import { icon } from '../icons.js';
import * as dashboard from './dashboard.js';
import * as theme from '../theme.js';

// Giro acumulado de la rueda para la prueba en vivo de esta tarjeta.
let liveTotal = 0;
let liveDecay = null;

// Deshabilita los botones mientras hay una copia en marcha: leer los 20 huecos
// de icono de cada perfil lleva su tiempo y pulsar dos veces solaparía comandos
// en el CDC.
function backupBusy(message) {
  const status = document.getElementById('backup-status');
  const save = document.getElementById('btn-backup-save');
  const load = document.getElementById('btn-backup-load');
  if (status) status.textContent = message || '';
  if (save) save.disabled = Boolean(message);
  if (load) load.disabled = Boolean(message);
}

export function init() {
  // Las tarjetas que dependen del PC se ocultan enteras en vez de deshabilitarse: un
  // interruptor de autoarranque de Windows dentro de una pestaña de Chrome no es un
  // ajuste apagado, es un ajuste que no significa nada.
  //
  // Va antes de los addEventListener porque los botones de dentro dejan de existir a
  // efectos prácticos: engancharlos igual no rompe nada, pero deja manejadores vivos
  // que llaman a un window.orby que aquí no hace nada.
  const ocultarSiFalta = (capacidad, selector) => {
    if (platform.can(capacidad)) return true;
    document.querySelector(selector)?.classList.add('hidden');
    return false;
  };

  const hayAutostart = ocultarSiFalta('autostart', '#settings-autostart');
  const hayPlugins   = ocultarSiFalta('plugins', '#settings-plugins');
  ocultarSiFalta('appUpdate', '#settings-app');
  ocultarSiFalta('firmwareUpdate', '#settings-firmware');

  document.getElementById('btn-backup-save').addEventListener('click', () => runExport(backupBusy));
  document.getElementById('btn-backup-load').addEventListener('click', () => runImport(backupBusy));

  // La apariencia no depende del teclado: se puede cambiar en solo lectura y
  // en la WebGUI sin conectar nada.
  const pintarTema = () => {
    const p = theme.preferencia();
    document.querySelectorAll('#theme-selector .opt-btn')
      .forEach((b) => b.classList.toggle('active', b.dataset.val === p));
  };
  document.querySelectorAll('#theme-selector .opt-btn').forEach((btn) => {
    btn.addEventListener('click', () => { theme.elegir(btn.dataset.val); pintarTema(); });
  });
  // El botón de tema de la barra superior lo cambia sin pasar por aquí.
  new MutationObserver(pintarTema)
    .observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  pintarTema();

  document.querySelectorAll('#timeout-selector .opt-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const val = Number(btn.dataset.val);
      if (!requireDevice()) return;
      try {
        await device.setTimeout_(val);
        state.timeout = val;
        markDirty();
        render();
      } catch {
        toast('El teclado no confirmó el tiempo de reposo', 'error');
      }
    });
  });

  initAnimSpeed();
  initMenuKey();
  initRotInvert();
  if (hayAutostart) initAutostart();
  if (hayPlugins) initPlugins();

  document.getElementById('btn-reconnect').addEventListener('click', () => {
    window.orby.reconnect();
    toast('Buscando el dispositivo…', 'info');
  });

  document.getElementById('btn-resync').addEventListener('click', async () => {
    try {
      await syncFromDevice();
      toast('Configuración releída del teclado');
    } catch {
      toast('No se pudo leer la configuración', 'error');
    }
  });

  document.getElementById('btn-factory-reset').addEventListener('click', async () => {
    if (!requireDevice()) return;
    if (!await confirmar({
      titulo: '¿Restaurar valores de fábrica?',
      mensaje: 'Se restaurarán los perfiles, iconos y la calibración de fábrica en la memoria del teclado.\n\n'
             + 'El cambio no será permanente hasta que pulses "Guardar en Flash".',
      aceptar: 'Restaurar',
      peligro: true,
    })) return;
    try {
      await device.resetDefaults();
      // Vuelven los cuatro perfiles de fábrica: los índices y los iconos que
      // tuviéramos leídos ya no valen.
      cache.invalidate();
      await syncFromDevice();
      markDirty();
      toast('Valores de fábrica restaurados (sin guardar todavía)', 'info');
    } catch {
      toast('No se pudo restaurar la configuración', 'error');
    }
  });

  initWheelCalib();
  initAppCard();
  initFirmwareCard();
}

// ================= Aplicación (versión y actualizaciones) =================
function initAppCard() {
  const check = document.getElementById('btn-check-update');
  const install = document.getElementById('btn-install-update');
  if (!check || !install) return;

  // La explicación se escribe aquí y no en index.html porque hubo un tiempo en que las
  // dos vías no hacían lo mismo (Tauri solo avisaba y abría la página de la release).
  // Desde que las actualizaciones van firmadas, las dos descargan e instalan solas y el
  // texto es el mismo; se deja escrito desde el código para que no vuelva a haber dos
  // versiones del mismo párrafo que se contradigan.
  document.getElementById('app-update-desc').textContent =
    'Se comprueba al arrancar y cada seis horas. Las versiones nuevas se descargan e '
    + 'instalan solas: la app se cierra un momento y vuelve a abrirse sola. Si hay '
    + 'cambios sin guardar en la Flash del teclado, espera a que se guarden.';

  check.addEventListener('click', async () => {
    if (updater.update.status === 'dev') {
      toast('En modo desarrollo no hay actualizaciones que buscar', 'info');
      return;
    }
    await updater.check();
    // El resultado llega por el canal de estado; esto solo confirma el clic.
    toast('Buscando actualizaciones…', 'info');
  });

  install.addEventListener('click', () => updater.install());

  updater.onChange(renderAppCard);
  renderAppCard();
}

function renderAppCard() {
  // render() se llama en cada notify(): sin appUpdate estos elementos ni existen.
  if (!platform.can('appUpdate')) return;

  const version = document.getElementById('app-version');
  const status = document.getElementById('app-update-status');
  const install = document.getElementById('btn-install-update');
  if (!version || !status || !install) return;

  version.textContent = updater.update.version || '—';
  status.textContent = updater.describe();
  status.style.color = updater.update.status === 'error' ? 'var(--danger)' : '';
  // 'downloaded' es de Electron (ya bajada) y 'available' de Tauri (solo se sabe que
  // existe): en los dos hay algo que ofrecer, aunque el botón no haga lo mismo.
  const estado = updater.update.status;
  install.classList.toggle('hidden', estado !== 'downloaded' && estado !== 'available');
}

// ================= Firmware del teclado =================
// Actualizar el teclado no es como actualizar la app: hay que reiniciarlo en el
// cargador de la ROM, y mientras dura la copia el teclado *no existe* (aparece
// una unidad USB en su lugar). Por eso esta tarjeta avisa antes de empezar y
// deja el estado a la vista durante todo el proceso.
function initFirmwareCard() {
  const card = document.getElementById('settings-firmware');
  if (!card) return;

  // Sin preload (la interfaz abierta en un navegador para maquetar) no hay
  // manera de flashear nada: la tarjeta sobra.
  if (!firmware.available()) {
    card.classList.add('hidden');
    return;
  }

  document.getElementById('btn-fw-check').addEventListener('click', async () => {
    const st = await firmware.check();
    // Un fallo al consultar GitHub no es "no hay firmware": decir eso mandaba a
    // buscar un problema que no existe (la release estaba publicada).
    if (!st) { toast('La actualización de firmware no está disponible en esta sesión', 'error', 6000); return; }
    if (st.status === 'error') { toast(st.error, 'error', 6000); return; }
    if (st.status === 'idle' && !st.available) {
      toast(st.latest
        ? `El teclado ya tiene el último firmware (${st.latest.version})`
        : 'No hay firmware publicado que esta versión de la app sepa instalar', 'info');
    }
  });

  document.getElementById('btn-fw-update').addEventListener('click', async () => {
    if (!requireDevice()) return;

    // Guardar primero, y no después: el reinicio en el cargador se lleva por
    // delante lo que hubiera solo en RAM, y las variaciones por aplicación
    // viven exactamente ahí (ver variants.js).
    if (state.dirty) {
      toast('Guarda los cambios en Flash antes de actualizar el firmware', 'error', 6000);
      return;
    }

    const manual = !compat.supports(state.deviceInfo, 'bootsel');
    const aviso = manual
      ? 'Este firmware no sabe reiniciarse solo: cuando te lo pida, desenchufa el teclado y vuelve a enchufarlo con BOOTSEL pulsado.\n\n'
      : '';
    if (!await confirmar({
      titulo: `¿Actualizar a la ${firmware.fw.latest?.version}?`,
      mensaje: `${aviso}El teclado dejará de funcionar durante la copia (unos segundos). No lo desconectes.`,
      aceptar: 'Actualizar',
    })) return;

    await firmware.update();
    if (firmware.fw.status === 'done') toast('Firmware actualizado');
    else if (firmware.fw.status === 'error') toast(firmware.fw.error, 'error', 9000);
  });

  document.getElementById('btn-fw-cancel').addEventListener('click', () => {
    firmware.cancel();
    toast('Actualización cancelada', 'info');
  });

  // iniciarAuto() lee la configuración de forma asíncrona: el interruptor se
  // pinta después de esa lectura, en la primera vuelta de renderFirmwareCard.
  const auto = document.getElementById('btn-fw-auto');
  auto.addEventListener('click', async () => {
    const on = !auto.classList.contains('on');
    await firmware.setAutoEnabled(on);
    auto.classList.toggle('on', on);
    toast(on ? 'El firmware se actualizará solo cuando no uses el ordenador'
             : 'Actualización automática del firmware desactivada');
  });

  firmware.onChange(renderFirmwareCard);
  firmware.init().then(renderFirmwareCard);
}

function renderFirmwareCard() {
  // render() se llama en cada notify(): sin firmwareUpdate estos elementos ni existen.
  if (!platform.can('firmwareUpdate')) return;

  const current = document.getElementById('fw-current');
  if (!current) return;

  const st = firmware.fw;
  const latest = document.getElementById('fw-latest');
  const status = document.getElementById('fw-status');
  const btnUpdate = document.getElementById('btn-fw-update');
  const btnCheck = document.getElementById('btn-fw-check');
  const btnCancel = document.getElementById('btn-fw-cancel');

  // La versión que manda es la del teclado que hay delante ahora mismo, no la
  // que guardó el proceso principal la última vez que se comprobó.
  current.textContent = state.connected ? (state.deviceInfo?.fw || '?') : 'sin teclado';
  latest.textContent = st.latest?.version || '—';
  status.textContent = firmware.describe();
  status.style.color = st.status === 'error' ? 'var(--danger)' : '';

  btnCheck.disabled = firmware.busy();
  document.getElementById('btn-fw-auto')?.classList.toggle('on', firmware.autoEnabled());
  btnCancel.classList.toggle('hidden', !['downloading', 'bootsel'].includes(st.status));
  // Se ofrece cuando hay algo que instalar y hay teclado al que instalárselo.
  btnUpdate.classList.toggle('hidden', !st.latest || !state.connected || firmware.busy());
  // Solo la etiqueta: escribir sobre el botón entero se llevaría por delante el
  // <span data-icon>, que ya está hidratado y no se vuelve a pintar.
  document.getElementById('fw-update-label').textContent =
    st.available ? `Actualizar a ${st.latest.version}` : 'Reinstalar firmware';
}

// ================= Autoarranque con Windows =================
// El estado real vive en el registro de Windows (app.setLoginItemSettings), no
// en config.json, así que se pregunta al proceso principal en vez de guardarlo
// aparte: así el botón nunca puede desincronizarse de lo que Windows hace.
async function initAutostart() {
  const btn = document.getElementById('btn-autostart');
  if (!btn) return;

  btn.classList.toggle('on', await window.orby.autostart.get());

  btn.addEventListener('click', async () => {
    const enabled = await window.orby.autostart.set(!btn.classList.contains('on'));
    btn.classList.toggle('on', enabled);
    toast(enabled ? 'Orvy arrancará con Windows' : 'Autoarranque desactivado');
  });
}

// ================= Complementos =================
// La lista de instalados y, debajo, una tarjeta por cada uno que pida ajustes
// propios. Los ajustes se declaran (nombre, tipo, valor por defecto) y se
// pintan solos: la app no sabe qué es una "dirección de lámpara", solo que ese
// complemento pide un texto llamado así.
//
// Las acciones de los complementos no pasan por esta vista: las dispara el
// firmware con MACRO:<id> y las ejecuta el proceso principal (ver
// electron/plugins.js).
function initPlugins() {
  const install = document.getElementById('btn-plugin-install');
  const folder = document.getElementById('btn-plugin-folder');
  const list = document.getElementById('plugin-list');
  const cards = document.getElementById('plugin-settings-cards');
  if (!install || !list || !cards) return;

  install.addEventListener('click', async () => {
    install.disabled = true;
    const res = await window.orby.plugins.install();
    install.disabled = false;
    if (res.canceled) return;
    if (!res.ok) {
      toast(`No se pudo instalar: ${res.error}`, 'error', 6000);
      return;
    }
    await plugins.refresh();
    toast(`Complemento «${res.plugin.name}» instalado`);
  });

  folder?.addEventListener('click', () => window.orby.plugins.openFolder());

  list.addEventListener('click', async (e) => {
    const el = e.target.closest('[data-act]');
    if (!el) return;
    const id = el.dataset.plugin;

    if (el.dataset.act === 'plugin-enable') {
      const p = plugins.byId(id);
      await window.orby.plugins.setEnabled(id, !p?.enabled);
      await plugins.refresh();
    } else if (el.dataset.act === 'plugin-remove') {
      const p = plugins.byId(id);
      if (!await confirmar({
        titulo: `¿Desinstalar «${p?.name || id}»?`,
        mensaje: 'Las teclas y mandos que lo usaran dejarán de hacer nada, pero conservarán su '
               + 'ajuste: si vuelves a instalarlo, volverán a funcionar.',
        aceptar: 'Desinstalar',
        peligro: true,
      })) return;
      const res = await window.orby.plugins.uninstall(id);
      if (!res.ok) {
        toast(`No se pudo desinstalar: ${res.error}`, 'error', 5000);
        return;
      }
      await plugins.refresh();
      toast('Complemento desinstalado');
    }
  });

  // Cada campo se guarda al salir de él, como la dirección de la lámpara hacía
  // antes: escribir a cada tecla dispararía una escritura del archivo por letra.
  cards.addEventListener('change', (e) => {
    const el = e.target.closest('[data-plugin][data-key]');
    if (!el) return;
    const value = el.type === 'number' ? Number(el.value) : el.value;
    window.orby.plugins.setSettings(el.dataset.plugin, { [el.dataset.key]: value });
  });

  cards.addEventListener('click', async (e) => {
    const el = e.target.closest('[data-act]');
    if (!el) return;

    if (el.dataset.act === 'plugin-toggle-field') {
      const on = !el.classList.contains('on');
      el.classList.toggle('on', on);
      window.orby.plugins.setSettings(el.dataset.plugin, { [el.dataset.key]: on });
    } else if (el.dataset.act === 'plugin-test') {
      await runPluginTest(el.dataset.plugin, el);
    }
  });

  plugins.onChange(renderPlugins);
  renderPlugins();
}

// La comprobación se hace con lo que hay escrito en los campos ahora mismo, no
// con lo guardado: probar una dirección nueva sin haber salido antes del campo
// es justo lo que uno intenta hacer cuando algo no responde.
async function runPluginTest(id, btn) {
  const status = document.getElementById(`plugin-status-${id}`);
  const values = {};
  document.querySelectorAll(`#plugin-settings-cards [data-plugin="${id}"][data-key]`).forEach((el) => {
    values[el.dataset.key] = el.classList?.contains('switch')
      ? el.classList.contains('on')
      : (el.type === 'number' ? Number(el.value) : el.value);
  });
  window.orby.plugins.setSettings(id, values);

  if (status) status.textContent = 'Probando…';
  btn.disabled = true;
  const res = await window.orby.plugins.test(id, values);
  btn.disabled = false;

  if (status) status.textContent = res.ok ? res.detail : `Sin respuesta (${res.error})`;
  toast(res.ok ? 'El complemento responde' : 'El complemento no responde', res.ok ? 'success' : 'error');
}

function renderPlugins() {
  // render() se llama en cada notify(): sin plugins estos elementos ni existen.
  if (!platform.can('plugins')) return;
  renderPluginList();
  renderPluginCards();
}

function renderPluginList() {
  const list = document.getElementById('plugin-list');
  if (!list) return;
  const all = plugins.all();

  if (!all.length) {
    list.innerHTML = '<p class="plugin-empty">Todavía no hay ninguno instalado.</p>';
    return;
  }

  list.innerHTML = all.map((p) => `
    <div class="plugin-row ${p.error ? 'has-error' : ''}">
      <span class="plugin-row-icon">${icon(p.icon, 18)}</span>
      <div class="plugin-row-main">
        <span class="plugin-row-name">${escape(p.name)} <b>${escape(p.version)}</b></span>
        <span class="plugin-row-desc">${escape(p.error ? `No se pudo cargar: ${p.error}` : p.description)}</span>
      </div>
      <div class="plugin-row-actions">
        ${p.error ? '' : `
          <button class="switch ${p.enabled ? 'on' : ''}" data-act="plugin-enable" data-plugin="${p.id}"
                  title="${p.enabled ? 'Desactivar' : 'Activar'}">
            <span class="switch-knob"></span>
          </button>`}
        <button class="tool-btn small danger" data-act="plugin-remove" data-plugin="${p.id}"
                title="Desinstalar">${icon('trash', 14)}</button>
      </div>
    </div>`).join('');
}

function renderPluginCards() {
  const host = document.getElementById('plugin-settings-cards');
  if (!host) return;

  // Solo los activos y con ajustes que pedir: uno desactivado no debe dejar una
  // tarjeta que parezca que sigue haciendo algo.
  const conAjustes = plugins.active().filter((p) => p.settings?.fields?.length || p.settings?.hasTest);

  host.innerHTML = conAjustes.map((p) => `
    <div class="settings-card glass-panel">
      <div class="card-header">
        <span>${icon(p.icon, 22)}</span>
        <h2>${escape(p.name)}</h2>
      </div>
      <div class="setting-body">
        ${(p.settings.fields || []).map((f) => renderPluginField(p, f)).join('')}
        ${p.settings.hasTest ? `
          <div class="row-inline">
            <button class="secondary-btn" data-act="plugin-test" data-plugin="${p.id}">Probar</button>
            <span class="setting-desc" id="plugin-status-${p.id}"></span>
          </div>` : ''}
        ${p.settings.description ? `<p class="setting-desc">${escape(p.settings.description)}</p>` : ''}
      </div>
    </div>`).join('');
}

function renderPluginField(p, f) {
  const value = p.values?.[f.key];
  const attrs = `data-plugin="${p.id}" data-key="${escape(f.key)}"`;

  if (f.type === 'toggle') {
    return `
      <div class="row-inline">
        <button class="switch ${value ? 'on' : ''}" data-act="plugin-toggle-field" ${attrs}>
          <span class="switch-knob"></span>
        </button>
        <span class="switch-label">${escape(f.label)}</span>
      </div>
      ${f.hint ? `<p class="setting-desc">${escape(f.hint)}</p>` : ''}`;
  }

  const control = f.type === 'select'
    ? `<select class="select-input" ${attrs}>
         ${f.options.map((o) => `
           <option value="${escape(o.value)}" ${String(value) === o.value ? 'selected' : ''}>${escape(o.label)}</option>`).join('')}
       </select>`
    : `<input class="text-input" type="${f.type === 'number' ? 'number' : f.type}" ${attrs}
              value="${escape(value ?? '')}" placeholder="${escape(f.placeholder)}" spellcheck="false">`;

  return `
    <label class="field">
      <span class="field-label">${escape(f.label)}</span>
      ${control}
    </label>
    ${f.hint ? `<p class="setting-desc">${escape(f.hint)}</p>` : ''}`;
}

// Lo que llega de un complemento es texto de terceros: acaba en innerHTML, así
// que no puede entrar sin escapar.
function escape(s) {
  return String(s ?? '').replace(/[&<>"']/g, (c) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// ================= Velocidad de las animaciones =================
// Cada grupo de transición (ver OledAnim::Group en el firmware) con su propio
// multiplicador sobre el ritmo de fábrica de ese grupo. Opciones fijas y no un
// slider: entre 135% y 140% no se ve diferencia en el panel, y con cinco
// escalones se puede volver a "Normal" sin buscar el número.
//
// `base` es el ritmo de fábrica medido como lo contaba el firmware 4.8/4.9
// (ANIMSPEED=1): en % del muelle de Apple. Desde la 4.10 (ANIMSPEED=2) el
// teclado ya habla en % sobre esa base y no hay que traducir nada.
const ANIM_GROUPS = [
  { key: 'menu',     group: 0, base: 200, label: 'Menú y SUPER' },
  { key: 'carousel', group: 1, base: 65,  label: 'Carrusel de páginas' },
  { key: 'submenu',  group: 2, base: 200, label: 'Entrar/salir de submenú' },
  { key: 'profile',  group: 3, base: 140, label: 'Cambio de perfil' },
  { key: 'fade',     group: 4, base: 100, label: 'Fundido' },
];

const ANIM_OPTIONS = [50, 75, 100, 125, 150];

const animAbsolute = () => state.deviceInfo?.animspeed === '1';

// Lo que dice el teclado → % sobre la base, ajustado a la opción más cercana
// (un 4.8 puede traer cualquier número que se dejara con el slider).
function animOption(g) {
  const raw = state.animSpeed[g.key] ?? 100;
  const rel = animAbsolute() ? (raw * 100) / g.base : raw;
  return ANIM_OPTIONS.reduce((a, b) => (Math.abs(b - rel) < Math.abs(a - rel) ? b : a));
}

// El 4.8/4.9 guarda el % en un byte sin comprobarlo: por encima de 255 da la
// vuelta. Se recorta aquí, aunque el menú más rápido se quede algo corto.
function animWire(g, rel) {
  if (!animAbsolute()) return rel;
  return Math.min(255, Math.max(25, Math.round((g.base * rel) / 100)));
}

function initAnimSpeed() {
  const list = document.getElementById('anim-speed-list');
  if (!list) return;

  list.addEventListener('click', async (e) => {
    const btn = e.target.closest('[data-act="animspeed"]');
    if (!btn) return;
    const g = ANIM_GROUPS[Number(btn.dataset.group)];
    const rel = Number(btn.dataset.pct);
    if (!requireDevice()) return;
    const wire = animWire(g, rel);
    try {
      await device.setAnimSpeed(g.group, wire);
      state.animSpeed = { ...state.animSpeed, [g.key]: wire };
      markDirty();
    } catch {
      toast('El teclado no confirmó la velocidad de la animación', 'error');
    }
    renderAnimSpeed();
  });

  renderAnimSpeed();
}

function renderAnimSpeed() {
  const card = document.getElementById('settings-animspeed');
  const list = document.getElementById('anim-speed-list');
  if (!card || !list) return;

  // Sin la bandera `animspeed` el teclado no entiende SET_ANIMSPEED: mejor no
  // ofrecer un control que se quedaría esperando una respuesta que no llega.
  if (!compat.supports(state.deviceInfo, 'animSpeed')) {
    card.classList.add('hidden');
    return;
  }
  card.classList.remove('hidden');

  list.innerHTML = ANIM_GROUPS.map((g) => {
    const cur = animOption(g);
    return `
    <div class="anim-speed-row">
      <span class="field-label">${g.label}</span>
      <div class="timeout-selector">
        ${ANIM_OPTIONS.map((pct) => `
          <button class="opt-btn${pct === cur ? ' active' : ''}" data-act="animspeed"
                  data-group="${g.group}" data-pct="${pct}">${pct}%</button>`).join('')}
      </div>
    </div>`;
  }).join('');
}

// ================= Tecla 10 =================
// Con la tecla de menú libre, las páginas y el selector de perfiles se llevan
// con un mando (ROTARY_TYPES.PAGE y MENU, firmware 4.15). El menú principal no
// se pierde nunca: los dos mandos pulsados 2 s lo abren en cualquier caso.

// Los tres huecos de cada mando en la capa normal (enum RotarySlot).
const ENCODER_SLOTS = {
  1: { cw: ROTARY_SLOTS.ENC1_CW, ccw: ROTARY_SLOTS.ENC1_CCW, click: ROTARY_SLOTS.ENC1_CLICK },
  2: { cw: ROTARY_SLOTS.ENC2_CW, ccw: ROTARY_SLOTS.ENC2_CCW, click: ROTARY_SLOTS.ENC2_CLICK },
};

// Qué mando lleva ya páginas y menú en todas las páginas de todos los
// perfiles, o null. Es lo que hace falta para soltar la tecla sin perder nada.
function navEncoder() {
  for (const enc of [1, 2]) {
    const { cw, click } = ENCODER_SLOTS[enc];
    const everywhere = state.profiles.length > 0 && state.profiles.every((prof) =>
      (prof.pages || []).slice(0, pageCountOf(prof)).every((pg) =>
        pg.rotary?.[cw]?.type === ROTARY_TYPES.PAGE && pg.rotary?.[click]?.type === ROTARY_TYPES.MENU));
    if (everywhere) return enc;
  }
  return null;
}

// El mando de menú actual: el que dice el firmware (4.16, global) o, en uno
// anterior, el que tenga páginas y menú en todos los perfiles.
function currentMenuEncoder() {
  return compat.supports(state.deviceInfo, 'menuEncoder') ? (state.menuEncoder || null) : navEncoder();
}

function renderRotInvert() {
  const card = document.getElementById('settings-rotinv');
  if (!card) return;
  if (!compat.supports(state.deviceInfo, 'rotInvert')) {
    card.classList.add('hidden');
    return;
  }
  card.classList.remove('hidden');
  document.querySelectorAll('#rotinv-selector .opt-btn').forEach((b) => {
    b.classList.toggle('active', (state.rotInvert & Number(b.dataset.bit)) !== 0);
  });
  const desc = document.getElementById('rotinv-desc');
  if (desc) {
    desc.textContent = 'Marca lo que quieras que gire al revés de como viene de fábrica. Vale para '
      + 'todos los perfiles y para los menús del teclado, y es independiente de la inversión de la '
      + 'rueda de cada perfil.';
  }
}

function initRotInvert() {
  document.querySelectorAll('#rotinv-selector .opt-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
      if (!requireDevice()) return;
      const mask = state.rotInvert ^ Number(btn.dataset.bit);
      try {
        await device.setRotInvert(mask);
        state.rotInvert = mask;
        markDirty();
        render();
      } catch {
        toast('El teclado no confirmó el cambio de sentido', 'error');
      }
    });
  });
}

function renderMenuKey() {
  const card = document.getElementById('settings-menukey');
  if (!card) return;
  if (!compat.supports(state.deviceInfo, 'menuKey')) {
    card.classList.add('hidden');
    return;
  }
  card.classList.remove('hidden');
  document.querySelectorAll('#menukey-selector .opt-btn').forEach((b) => {
    b.classList.toggle('active', Number(b.dataset.val) === state.menuKeyMode);
  });
  const enc = currentMenuEncoder();
  const global = compat.supports(state.deviceInfo, 'menuEncoder');
  document.querySelectorAll('#menukey-encoder .opt-btn').forEach((b) => {
    // «Ninguno» solo existe con el ajuste global: antes el mando se montaba
    // reescribiendo los perfiles, y deshacerlo era cosa de cada uno.
    if (Number(b.dataset.enc) === 0) b.hidden = !global;
    b.classList.toggle('active', Number(b.dataset.enc) === (enc || 0));
  });
  const encDesc = document.getElementById('menukey-enc-desc');
  if (encDesc) {
    encDesc.textContent = global
      ? 'Girarlo cambia de página. Mantenlo pulsado y gira, sin soltar, hasta el perfil que '
        + 'quieras: al soltar, el teclado cambia a él (con SUPER, el gestor de páginas). Ese mando '
        + 'deja de configurarse por perfil. Con el menú abierto, las teclas siguen haciendo lo '
        + 'que tengan asignado.'
      : 'Girarlo cambia de página y pulsarlo abre el selector de perfiles, en la capa normal de '
        + 'todas las páginas. Se puede cambiar después perfil a perfil, en Rueda y mandos.';
  }
  const desc = document.getElementById('menukey-desc');
  if (!desc) return;
  if (state.menuKeyMode !== 1) {
    desc.textContent = 'Es la tecla de menú: una pulsación corta pasa de página y mantenida '
      + 'abre el selector de perfiles. Como tecla normal hace lo que le asignes en cada página.';
  } else if (enc) {
    desc.textContent = `Es una tecla normal. Las páginas y el selector de perfiles van en el mando `
      + `${enc === 1 ? 'izquierdo' : 'derecho'}; el menú principal, con los dos mandos pulsados 2 s.`;
  } else {
    desc.textContent = 'Es una tecla normal. Ojo: no todos los perfiles tienen un mando para '
      + 'cambiar de página y abrir el selector de perfiles. El menú principal sigue a mano con '
      + 'los dos mandos pulsados 2 s.';
  }
}

function initMenuKey() {
  document.querySelectorAll('#menukey-selector .opt-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const val = Number(btn.dataset.val);
      if (val === state.menuKeyMode || !requireDevice()) return;
      if (val === 1 && !currentMenuEncoder()) {
        const ok = await confirmar({
          titulo: 'Dejar libre la tecla 10',
          mensaje: 'Sin ella, cambiar de página y abrir el selector de perfiles solo se puede '
            + 'desde un mando que tenga «Cambiar de página» y «Abrir menú». Puedes ponerlo '
            + 'en todos los perfiles con los botones de abajo. El menú principal se abre '
            + 'siempre con los dos mandos pulsados 2 s.',
          aceptar: 'Dejarla libre',
        });
        if (!ok) return;
      }
      try {
        await device.setMenuKey(val);
        state.menuKeyMode = val;
        markDirty();
        render();
        dashboard.render();
      } catch {
        toast('El teclado no confirmó el cambio de la tecla 10', 'error');
      }
    });
  });

  document.querySelectorAll('#menukey-encoder .opt-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
      if (!requireDevice()) return;
      const enc = Number(btn.dataset.enc);

      // Firmware 4.16: es un ajuste del teclado, no hay perfiles que reescribir
      // ni nada que perder (lo que cada perfil tenga en ese mando queda guardado
      // y vuelve si se quita).
      if (compat.supports(state.deviceInfo, 'menuEncoder')) {
        if (enc === (state.menuEncoder || 0)) return;
        try {
          await device.setMenuEncoder(enc);
          state.menuEncoder = enc;
          markDirty();
          render();
          dashboard.render();
        } catch {
          toast('El teclado no confirmó el mando de menú', 'error');
        }
        return;
      }

      const ok = await confirmar({
        titulo: `Páginas y menú en el mando ${enc === 1 ? 'izquierdo' : 'derecho'}`,
        mensaje: `Cambia el giro y el clic de ese mando en la capa normal de todas las páginas `
          + `de los ${state.profiles.length} perfiles. Lo que tuvieran asignado se pierde.`,
        aceptar: 'Cambiar',
      });
      if (!ok) return;
      btn.disabled = true;
      try {
        await applyEncoderNav(enc);
        markDirty();
        toast('Mando configurado en todos los perfiles');
      } catch (err) {
        toast(`No se pudo terminar: ${err?.message || 'el teclado no respondió'}`, 'error');
        try { await syncFromDevice(); } catch { /* sin teclado no hay de dónde releer */ }
      } finally {
        btn.disabled = false;
        render();
        dashboard.render();
      }
    });
  });
}

// Escribe páginas y menú en un mando, en todas las páginas de todos los
// perfiles. El firmware solo deja escribir las páginas de un perfil que esté
// puesto (ver ensureEditTarget en store.js), así que se va activando cada uno
// y al final se vuelve al que estaba.
async function applyEncoderNav(enc) {
  const { cw, ccw, click } = ENCODER_SLOTS[enc];
  const page = { type: ROTARY_TYPES.PAGE, modifier: 0, keycode: 0 };
  const menu = { type: ROTARY_TYPES.MENU, modifier: 0, keycode: 0 };
  const writes = [[cw, page], [ccw, page], [click, menu]];
  const original = state.activeProfileIdx;
  try {
    for (let p = 0; p < state.profiles.length; p++) {
      const prof = state.profiles[p];
      if (state.activeProfileIdx !== p) {
        await device.setProfile(p);
        state.activeProfileIdx = p;
        state.pageIdx = 0;
      }
      for (let pg = 0; pg < pageCountOf(prof); pg++) {
        await withPinnedPage(p, pg, async () => {
          for (const [slot, a] of writes) {
            await device.setRotary(p, slot, a.type, a.modifier, a.keycode);
            if (prof.pages[pg]) prof.pages[pg].rotary[slot] = { ...a };
          }
        });
      }
    }
  } finally {
    if (state.activeProfileIdx !== original) {
      await device.setProfile(original);
      state.activeProfileIdx = original;
      state.pageIdx = 0;
    }
    notify();
  }
}

// ================= Calibración de la rueda =================
// Cómo se dibuja la rueda magnética en pantalla: no toca el teclado, solo hace
// que el dibujo (forma del marcador, sentido, desfase) coincida con el
// marcador físico que lleve pegado la tapa. Vive aquí, no en Perfiles, porque
// es un ajuste de montaje del PC, no de un perfil ni de una capa.
function initWheelCalib() {
  const box = document.getElementById('settings-wheel-calib-body');
  if (!box) return;

  box.addEventListener('click', (e) => {
    const el = e.target.closest('[data-act]');
    if (!el) return;
    const act = el.dataset.act;

    if (act === 'dial-marker') {
      applyDialSetting({ marker: el.dataset.marker });
    } else if (act === 'dial-invert') {
      applyDialSetting({ invert: !wheelDial.dial.invert });
    } else if (act === 'dial-nudge') {
      applyDialSetting({ offsetDeg: wheelDial.normalize(wheelDial.dial.offsetDeg + Number(el.dataset.d)) });
    } else if (act === 'dial-align') {
      wheelDial.alignHere();
      renderWheelCalib();
      dashboard.refreshMarker();
    }
  });

  // El desfase se ve al momento en la aguja, sin repintar la tarjeta entera.
  box.addEventListener('input', (e) => {
    if (e.target.dataset.act !== 'dial-offset') return;
    const deg = Number(e.target.value);
    wheelDial.setDial({ offsetDeg: deg });
    const label = document.getElementById('dial-offset-val');
    if (label) label.textContent = `${deg}°`;
  });

  // El dibujo de la rueda lo lleva wheel-dial.js con el ángulo absoluto del
  // sensor, así que la aguja se queda donde esté la rueda de verdad.
  wheelDial.onUpdate(paintDialMarker);

  device.on('telemetry', (line) => {
    if (line.startsWith('WHEEL:')) onWheelTelemetry(parseInt(line.slice(6), 10));
  });

  renderWheelCalib();
}

function paintDialMarker(deg) {
  const marker = document.getElementById('settings-wheel-needle');
  if (marker) marker.style.transform = `rotate(${deg}deg)`;
}

function onWheelTelemetry(delta) {
  if (!Number.isFinite(delta)) return;
  liveTotal += delta;

  const readout = document.getElementById('wheel-calib-readout');
  if (!readout) return;

  const detentsPerRev = state.scroll.detentsPerRev || 60;
  const detents = (liveTotal * detentsPerRev) / 4096;
  readout.textContent =
    `Rueda en ${wheelDial.normalize(wheelDial.angle()).toFixed(0)}°  ·  `
    + `${detents >= 0 ? '+' : ''}${detents.toFixed(2)} clics en este giro`;

  clearTimeout(liveDecay);
  liveDecay = setTimeout(() => { liveTotal = 0; }, 1200);
}

// Ajustes de cómo se dibuja la rueda: sentido de giro, desfase del marcador de
// la tapa y forma. Son de montaje, así que se guardan en el PC.
function applyDialSetting(patch) {
  wheelDial.setDial(patch);
  renderWheelCalib();
  // El dashboard tiene el mismo marcador y también hay que rehacerlo.
  dashboard.refreshMarker();
}

function renderWheelCalib() {
  const box = document.getElementById('settings-wheel-calib-body');
  if (!box) return;
  const d = wheelDial.dial;

  box.innerHTML = `
    <p class="setting-desc">
      Cómo se dibuja la rueda magnética en pantalla. No afecta al teclado: solo hace que el
      dibujo coincida con el marcador que lleve pegado la tapa.
    </p>

    <div class="wheel-dial">
      <div class="wheel-dial-face">${wheelDial.markerHtml('settings-wheel-needle')}</div>
    </div>
    <p class="wheel-live-readout" id="wheel-calib-readout">Gira la rueda para comprobar el recorrido</p>

    <div class="dial-calib">
      <span class="field-label">Marcador en pantalla</span>

      <div class="chip-row">
        <button class="chip ${d.marker === 'dot' ? 'on' : ''}" data-act="dial-marker" data-marker="dot">Círculo</button>
        <button class="chip ${d.marker === 'line' ? 'on' : ''}" data-act="dial-marker" data-marker="line">Raya</button>
        <button class="chip ${d.invert ? 'on' : ''}" data-act="dial-invert"
                title="Si el dibujo gira al revés que la rueda">Invertir giro</button>
      </div>

      <div class="dial-offset">
        <button class="tool-btn small" data-act="dial-nudge" data-d="-1" title="1° menos">${icon('minus', 14)}</button>
        <input type="range" class="premium-slider" data-act="dial-offset"
               min="0" max="359" step="1" value="${Math.round(d.offsetDeg)}">
        <button class="tool-btn small" data-act="dial-nudge" data-d="1" title="1° más">${icon('plus', 14)}</button>
        <b id="dial-offset-val">${Math.round(d.offsetDeg)}°</b>
      </div>

      <button class="secondary-btn full" data-act="dial-align">
        ${icon('fit', 16)} El marcador está arriba: alinear aquí
      </button>
      <p class="setting-desc">
        Pon el circulito de la tapa mirando hacia arriba y pulsa el botón: el de la
        pantalla se coloca en el mismo sitio. Con la barra afinas grado a grado.
      </p>
    </div>`;

  paintDialMarker(wheelDial.angle());
}

export function render() {
  renderAppCard();
  renderFirmwareCard();

  document.querySelectorAll('#timeout-selector .opt-btn').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.val) === state.timeout);
  });

  renderAnimSpeed();
  renderMenuKey();
  renderRotInvert();

  const list = document.getElementById('device-info-list');
  if (!list) return;

  if (state.connected) {
    const info = state.deviceInfo || {};
    // La versión de firmware sola no dice nada: lo que importa es si la app
    // puede con ella. El veredicto sale de compat.js, el mismo que decide qué
    // comandos se mandan, para que la pantalla no pueda contar otra cosa.
    const verdict = compat.check(info);
    const verdictColor = verdict.level === 'ok' ? 'var(--ok, inherit)' : 'var(--danger)';
    list.innerHTML = `
      <li><span class="lbl">Dispositivo</span><span class="val">${info.device || 'ORBY_V4'}</span></li>
      <li><span class="lbl">Firmware</span><span class="val">${info.fw || '?'}</span></li>
      <li><span class="lbl">Compatibilidad</span><span class="val" style="color:${verdictColor}"
          title="${verdict.detail}">${verdict.level === 'ok' ? 'al día' : verdict.title}</span></li>
      <li><span class="lbl">Puerto</span><span class="val">${info.port || '—'}</span></li>
      <li><span class="lbl">Teclas / OLEDs</span><span class="val">${info.keys || 12} / ${info.oleds || 10}</span></li>
      <li><span class="lbl">Perfiles</span><span class="val">${state.profiles.length} / ${state.maxProfiles}</span></li>
      <li><span class="lbl">Modo</span><span class="val">${state.deviceMode}</span></li>
      <li><span class="lbl">Scroll alta res.</span><span class="val">${state.scroll.hires ? 'sí' : 'no'}</span></li>`;
  } else {
    list.innerHTML = '<li><span class="lbl">Estado</span><span class="val" style="color:var(--danger)">Desconectado</span></li>';
  }

  renderWheelCalib();
}
