// Estado Global
const state = {
  activeProfileIdx: 0,
  deviceMode: 'NORMAL',
  superActive: false,
  brightness: 207,
  timeout: 5,
  layerView: 'normal', // 'normal' o 'super'
};

// Datos de perfiles hardcodeados (espejo del firmware V1)
const profilesData = [
  {
    idx: 0, name: 'Ofimática', icon: 'ph-file-text', id: 'OFIM',
    normal: ["COPY", "PAST", "CUT", "UNDO", "REDO", "ALL", "SAVE", "FIND", "CLSE", "-", "SNAP", "-"],
    super: ["EXCL", "WORD", "BROW", "CALC", "TASK", "LOCK", "PLAY", "STOP", "NEXT", "-", "PREV", "-"]
  },
  {
    idx: 1, name: 'Photoshop', icon: 'ph-paint-brush', id: 'PHTS',
    normal: ["BRSH", "ERAS", "PENC", "UNDO", "REDO", "LASS", "CROP", "MOVE", "NEWL", "-", "DUPL", "-"],
    super: ["ZOOM", "HAND", "TEXT", "GRAD", "PEN", "SHAP", "EYE", "SELC", "SLIC", "-", "PATH", "-"]
  },
  {
    idx: 2, name: 'Altium Designer', icon: 'ph-cpu', id: 'ALTM',
    normal: ["WIRE", "PORT", "PAD", "UNDO", "REDO", "VIA", "MOVE", "TRAC", "FIT", "-", "3D", "-"],
    super: ["SCH", "PCB", "LIB", "COMP", "GRID", "MEAS", "RULE", "ERC", "DRC", "-", "GEN", "-"]
  },
  {
    idx: 3, name: 'Premiere Pro', icon: 'ph-film-strip', id: 'PREM',
    normal: ["CUT", "SELC", "PLAY", "UNDO", "REDO", "MARK", "LINK", "RIPL", "EXPO", "-", "NEST", "-"],
    super: ["SLIP", "SLID", "PEN", "HAND", "TEXT", "ZOOM", "TRIM", "RATE", "ROLL", "-", "SPED", "-"]
  }
];

// --- NAVEGACIÓN Y UI BASE ---
function initUI() {
  // Controles de ventana
  document.getElementById('btn-minimize').addEventListener('click', () => window.orby.minimize());
  document.getElementById('btn-maximize').addEventListener('click', () => window.orby.maximize());
  document.getElementById('btn-close').addEventListener('click', () => window.orby.close());

  // Navegación Sidebar
  const navItems = document.querySelectorAll('.nav-item');
  const views = document.querySelectorAll('.view');

  navItems.forEach(item => {
    item.addEventListener('click', () => {
      navItems.forEach(n => n.classList.remove('active'));
      views.forEach(v => v.classList.remove('active'));
      
      item.classList.add('active');
      const targetId = item.getAttribute('data-target');
      document.getElementById(targetId).classList.add('active');
    });
  });

  initDashboard();
  initSettings();
  initProfiles();
  initConsole();
}

// --- DASHBOARD (Visualizador Hardware) ---
function initDashboard() {
  const container = document.getElementById('keyboard-visualizer');
  
  // Dibujar teclado
  let html = `
    <div class="orby-board">
      <div class="controls-area">
        <div class="hw-encoder" id="hw-enc-1"></div>
        <div class="hw-scroll" id="hw-scroll"></div>
        <div class="hw-encoder" id="hw-enc-2"></div>
      </div>
      <div class="key-matrix">
  `;
  
  // 12 teclas (1 a 12)
  for (let i = 1; i <= 12; i++) {
    const hasScreen = (i !== 10 && i !== 12);
    html += `
      <div class="hw-key ${hasScreen ? '' : 'no-screen'}" id="hw-key-${i}">
        ${hasScreen ? `<div class="oled-screen" id="hw-oled-${i}">--</div>` : ''}
      </div>
    `;
  }
  
  html += `</div></div>`;
  container.innerHTML = html;
  updateDashboardLabels();
}

function triggerHwAnim(id) {
  const el = document.getElementById(id);
  if (el) {
    el.classList.add('active');
    setTimeout(() => el.classList.remove('active'), 150);
  }
}

function updateDashboardLabels() {
  document.getElementById('lbl-active-profile').textContent = profilesData[state.activeProfileIdx].id;
  document.getElementById('lbl-active-mode').textContent = state.deviceMode;
  document.getElementById('lbl-super-state').textContent = state.superActive ? 'Activa' : 'Inactiva';
  document.getElementById('lbl-super-state').style.color = state.superActive ? 'var(--primary)' : 'var(--text-main)';

  // Actualizar OLEDs virtuales
  const labels = state.superActive 
    ? profilesData[state.activeProfileIdx].super 
    : profilesData[state.activeProfileIdx].normal;

  for (let i = 1; i <= 12; i++) {
    const oled = document.getElementById(`hw-oled-${i}`);
    if (oled) {
      oled.textContent = labels[i-1] || '--';
    }
  }
}

function showBigEvent(text) {
  const feed = document.getElementById('live-event-feed');
  feed.innerHTML = `<div class="big-event">${text}</div>`;
}

// --- AJUSTES ---
function initSettings() {
  const slider = document.getElementById('brightness-slider');
  const valDisp = document.getElementById('brightness-val');

  slider.addEventListener('input', (e) => {
    const val = e.target.value;
    const pct = Math.round((val / 255) * 100);
    valDisp.textContent = `${pct}%`;
  });

  slider.addEventListener('change', (e) => {
    window.orby.sendCommand(`SET_BRIGHTNESS:${e.target.value}`);
  });

  const timeBtns = document.querySelectorAll('.opt-btn');
  timeBtns.forEach(btn => {
    btn.addEventListener('click', (e) => {
      timeBtns.forEach(b => b.classList.remove('active'));
      e.target.classList.add('active');
      const val = e.target.getAttribute('data-val');
      window.orby.sendCommand(`SET_TIMEOUT:${val}`);
    });
  });

  document.getElementById('btn-reconnect').addEventListener('click', () => {
    window.orby.reconnect();
  });
}

function updateDeviceInfo(info) {
  const list = document.getElementById('device-info-list');
  if (info) {
    list.innerHTML = `
      <li><span class="lbl">Dispositivo:</span> <span class="val">${info.device || 'ORBY_V4'}</span></li>
      <li><span class="lbl">Firmware:</span> <span class="val">${info.fw || '1.0'}</span></li>
      <li><span class="lbl">Teclas/OLEDs:</span> <span class="val">${info.keys || 12} / ${info.oleds || 10}</span></li>
      <li><span class="lbl">Modo Actual:</span> <span class="val">${info.mode || 'NORMAL'}</span></li>
    `;
    if (info.mode) {
      state.deviceMode = info.mode;
      updateDashboardLabels();
    }
  } else {
    list.innerHTML = `<li><span class="lbl">Estado:</span> <span class="val" style="color:var(--danger)">Desconectado</span></li>`;
  }
}

// --- PERFILES ---
function initProfiles() {
  const container = document.getElementById('profiles-container');
  
  document.getElementById('btn-layer-normal').addEventListener('click', (e) => {
    document.getElementById('btn-layer-super').classList.remove('active');
    e.target.classList.add('active');
    state.layerView = 'normal';
    renderProfiles(container);
  });
  
  document.getElementById('btn-layer-super').addEventListener('click', (e) => {
    document.getElementById('btn-layer-normal').classList.remove('active');
    e.target.classList.add('active');
    state.layerView = 'super';
    renderProfiles(container);
  });

  renderProfiles(container);
}

function renderProfiles(container) {
  let html = '';
  profilesData.forEach(p => {
    const isActive = p.idx === state.activeProfileIdx;
    const labels = state.layerView === 'normal' ? p.normal : p.super;
    
    html += `
      <div class="profile-card glass-panel ${isActive ? 'is-active' : ''}">
        <div class="profile-header">
          <div class="profile-title">
            <i class="${p.icon}"></i>
            <h2>${p.name}</h2>
          </div>
          <button class="btn-activate" data-idx="${p.idx}">${isActive ? 'ACTIVO' : 'Activar'}</button>
        </div>
        <div class="macro-grid">
    `;
    
    for (let i=0; i<12; i++) {
      html += `
        <div class="macro-item">
          <span class="key-num">T${i+1}</span>
          <span class="label">${labels[i]}</span>
        </div>
      `;
    }
    
    html += `</div></div>`;
  });
  
  container.innerHTML = html;
  
  // Bind events
  container.querySelectorAll('.btn-activate').forEach(btn => {
    btn.addEventListener('click', (e) => {
      const idx = parseInt(e.target.getAttribute('data-idx'));
      if (idx !== state.activeProfileIdx) {
        window.orby.sendCommand(`SET_PROFILE:${idx}`);
      }
    });
  });
}

// --- CONSOLA SERIE ---
function initConsole() {
  const out = document.getElementById('console-output');
  const input = document.getElementById('console-input');
  
  document.getElementById('btn-clear-console').addEventListener('click', () => {
    out.innerHTML = '<div class="console-line system">Consola limpia.</div>';
  });

  input.addEventListener('keypress', (e) => {
    if (e.key === 'Enter' && input.value.trim() !== '') {
      const cmd = input.value.trim();
      logConsole(cmd, 'tx');
      window.orby.sendCommand(cmd);
      input.value = '';
    }
  });
}

function logConsole(text, type='system') {
  const out = document.getElementById('console-output');
  const time = new Date().toLocaleTimeString('en-US', {hour12: false, hour: "numeric", minute: "numeric", second: "numeric"});
  out.innerHTML += `<div class="console-line ${type}">[${time}] ${text}</div>`;
  out.scrollTop = out.scrollHeight;
}

// --- EVENTOS SERIALES (IPC) ---
function setupSerialEvents() {
  const badge = document.getElementById('connection-status-badge');
  const badgeText = badge.querySelector('.status-text');

  window.orby.onConnected((info) => {
    badge.className = 'status-badge connected';
    badgeText.textContent = 'Orby V4 Conectado';
    logConsole('Dispositivo conectado: ' + info.raw, 'system');
    updateDeviceInfo(info);
    
    // Preguntar por estado inicial enviando una pulsación falsa para forzar refresco
    // En el futuro, el FW debería responder a GET_STATE
  });

  window.orby.onDisconnected(() => {
    badge.className = 'status-badge disconnected';
    badgeText.textContent = 'Desconectado';
    logConsole('Dispositivo desconectado.', 'error');
    updateDeviceInfo(null);
  });

  window.orby.onSearching(() => {
    if (!badge.classList.contains('connected')) {
      badge.className = 'status-badge searching';
      badgeText.textContent = 'Buscando USB...';
    }
  });

  window.orby.onData((line) => {
    logConsole(line, 'rx');
    
    // Parseo de telemetría bruta
    if (line.startsWith('KEY_EV:')) {
      const parts = line.split(':');
      const keyId = parseInt(parts[1]);
      const isPressed = parts[2] === '1';
      if (isPressed) {
        triggerHwAnim(`hw-key-${keyId}`);
        showBigEvent(`Tecla ${keyId}`);
        // Handle SUPER key local state
        if (keyId === 10) {
          state.superActive = true;
          updateDashboardLabels();
        }
      } else {
        if (keyId === 10) {
          state.superActive = false;
          updateDashboardLabels();
        }
      }
    } 
    else if (line.startsWith('ENC:')) {
      const parts = line.split(':');
      const encId = parts[1];
      const delta = parseInt(parts[2]);
      triggerHwAnim(`hw-enc-${encId}`);
      showBigEvent(encId === '1' ? `Volumen ${delta>0?'+':'-'}` : `Brillo ${delta>0?'+':'-'}`);
    }
    else if (line.startsWith('ENC_SW:')) {
      const parts = line.split(':');
      const encId = parts[1];
      const isPressed = parts[2] === '1';
      if (isPressed) {
        triggerHwAnim(`hw-enc-${encId}`);
        showBigEvent(`Click Enc ${encId}`);
      }
    }
    else if (line.startsWith('WHEEL:')) {
      triggerHwAnim('hw-scroll');
      showBigEvent('Scroll');
    }
    else if (line.startsWith('MODE:')) {
      state.deviceMode = line.split(':')[1];
      updateDashboardLabels();
      showBigEvent(`Modo: ${state.deviceMode}`);
    }
    else if (line.startsWith('PROFILE:OK:')) {
      state.activeProfileIdx = parseInt(line.split(':')[2]);
      updateDashboardLabels();
      renderProfiles(document.getElementById('profiles-container'));
      showBigEvent(`Perfil: ${profilesData[state.activeProfileIdx].id}`);
    }
    else if (line.startsWith('BRIGHTNESS:OK:')) {
      state.brightness = parseInt(line.split(':')[2]);
      document.getElementById('brightness-slider').value = state.brightness;
      document.getElementById('brightness-val').textContent = `${Math.round((state.brightness/255)*100)}%`;
    }
  });

  window.orby.onError((err) => {
    logConsole('ERROR: ' + err, 'error');
  });
}

// Inicialización
document.addEventListener('DOMContentLoaded', () => {
  initUI();
  setupSerialEvents();
});
