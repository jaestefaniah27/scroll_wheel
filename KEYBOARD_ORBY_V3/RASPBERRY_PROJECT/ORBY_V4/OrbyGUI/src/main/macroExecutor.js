const { keyboard, Key, mouse } = require('@nut-tree-fork/nut-js');
const { shell } = require('electron');
const { spawn } = require('child_process');

keyboard.config.autoDelayMs = 20;

const KEY_MAP = {
  // Letras
  'a': Key.A, 'b': Key.B, 'c': Key.C, 'd': Key.D, 'e': Key.E, 'f': Key.F,
  'g': Key.G, 'h': Key.H, 'i': Key.I, 'j': Key.J, 'k': Key.K, 'l': Key.L,
  'm': Key.M, 'n': Key.N, 'o': Key.O, 'p': Key.P, 'q': Key.Q, 'r': Key.R,
  's': Key.S, 't': Key.T, 'u': Key.U, 'v': Key.V, 'w': Key.W, 'x': Key.X,
  'y': Key.Y, 'z': Key.Z,
  'A': Key.A, 'B': Key.B, 'C': Key.C, 'D': Key.D, 'E': Key.E, 'F': Key.F,
  'G': Key.G, 'H': Key.H, 'I': Key.I, 'J': Key.J, 'K': Key.K, 'L': Key.L,
  'M': Key.M, 'N': Key.N, 'O': Key.O, 'P': Key.P, 'Q': Key.Q, 'R': Key.R,
  'S': Key.S, 'T': Key.T, 'U': Key.U, 'V': Key.V, 'W': Key.W, 'X': Key.X,
  'Y': Key.Y, 'Z': Key.Z,

  // Números
  '0': Key.Num0, '1': Key.Num1, '2': Key.Num2, '3': Key.Num3, '4': Key.Num4,
  '5': Key.Num5, '6': Key.Num6, '7': Key.Num7, '8': Key.Num8, '9': Key.Num9,

  // Especiales
  'Space': Key.Space, 'Enter': Key.Enter, 'Escape': Key.Escape,
  'Tab': Key.Tab, 'Delete': Key.Delete, 'Backspace': Key.Backspace,
  'Home': Key.Home, 'End': Key.End, 'PageDown': Key.PageDown, 'PageUp': Key.PageUp,
  'ArrowUp': Key.Up, 'ArrowDown': Key.Down, 'ArrowLeft': Key.Left, 'ArrowRight': Key.Right,
  
  // F-keys
  'F1': Key.F1, 'F2': Key.F2, 'F3': Key.F3, 'F4': Key.F4,
  'F5': Key.F5, 'F6': Key.F6, 'F7': Key.F7, 'F8': Key.F8,
  'F9': Key.F9, 'F10': Key.F10, 'F11': Key.F11, 'F12': Key.F12,

  // Símbolos
  '-': Key.Minus, '=': Key.Equal, '[': Key.LeftBracket, ']': Key.RightBracket,
  '\\': Key.Backslash, ';': Key.Semicolon, '\'': Key.Quote, '`': Key.Grave,
  ',': Key.Comma, '.': Key.Period, '/': Key.Slash
};

const MOD_MAP = {
  'Ctrl': Key.LeftControl,
  'Alt': Key.LeftAlt,
  'Shift': Key.LeftShift,
  'Win': Key.LeftSuper
};

class MacroExecutor {
  constructor() {
    this.activeModifiers = [];
  }

  async executeAction(action) {
    if (!action || action.type === 'none') return;
    console.log(`[MacroExecutor] Executing ${action.type}:`, action.value);

    try {
      if (action.type === 'shortcut') {
        await this.handleShortcut(action.value);
      } else if (action.type === 'text') {
        await keyboard.type(action.value);
      } else if (action.type === 'app_launch' || action.type === 'url') {
        this.handleSystemAction(action);
      } else if (action.type === 'media') {
        this.handleMediaAction(action.value);
      }
    } catch (err) {
      console.error(`[MacroExecutor] Error executing ${action.type}:`, err);
    }
  }

  /**
   * Ejecuta un shortcut (ej. "Ctrl+Shift+c") usando nut.js
   * @param {string} combo - Múltiples teclas unidas por '+'
   */
  async handleShortcut(combo) {
    if (!combo) return;
    const parts = combo.split('+');
    
    let modifiersToPress = [];
    let keysToPress = [];

    // Parse string to nut.js Keys
    parts.forEach(p => {
      // Intentamos matchear como modificador primero (sensible a mayúsculas como está escrito)
      if (MOD_MAP[p]) {
        modifiersToPress.push(MOD_MAP[p]);
      } else {
        // En caso de teclas normales, el grabador de react podría meter 'c' o 'C'
        const normalizedKey = p.length === 1 ? p.toLowerCase() : p;
        if (KEY_MAP[normalizedKey]) {
           keysToPress.push(KEY_MAP[normalizedKey]);
        } else if (KEY_MAP[p]) {
           keysToPress.push(KEY_MAP[p]); // Fallback por si acaso
        } else {
           console.warn(`[MacroExecutor] Key not found in map: ${p}`);
        }
      }
    });

    if (keysToPress.length === 0 && modifiersToPress.length === 0) {
       console.log(`[MacroExecutor] Empty valid combo parsed: ${combo}`);
       return;
    }

    try {
      // Press modifiers
      for (const mod of modifiersToPress) {
        await keyboard.pressKey(mod);
      }
      
      // Press and release normal keys
      if (keysToPress.length > 0) {
        await keyboard.pressKey(...keysToPress);
        await keyboard.releaseKey(...keysToPress);
      }

      // Release modifiers in reverse order
      for (const mod of modifiersToPress.reverse()) {
        await keyboard.releaseKey(mod);
      }
    } catch (e) {
      console.error('[MacroExecutor] Error during keystroke injection:', e);
      // Failsafe: soltar siempre los modificadores si algo revienta para no dejar la PC pillada
      for (const mod of modifiersToPress) {
        keyboard.releaseKey(mod).catch(() => {});
      }
    }
  }

  handleSystemAction(action) {
    if (action.type === 'url') {
      const url = action.value.startsWith('http') ? action.value : `https://${action.value}`;
      shell.openExternal(url).catch(e => console.error('[MacroExecutor] Error opening URL:', e));
    } else if (action.type === 'app_launch') {
      let path = action.value;
      if (path === 'cmd' || path === 'powershell') {
        spawn(path, [], { detached: true, shell: true });
      } else {
        spawn(path, [], { detached: true, shell: true }).on('error', (err) => {
          console.error('[MacroExecutor] Failed to launch app:', err.message);
        });
      }
    }
  }

  handleMediaAction(value) {
    // Para nut.js las teclas multimedia universales actualmente se pueden simular
    // presionando su correspondiente keyCode virtual si Key.AudioPlay no está disponible,
    // o podemos utilizar otro método como lanzar un script en powershell.
    const psCommand = (code) => `powershell -c "$wshell = New-Object -ComObject wscript.shell; $wshell.SendKeys([char]${code})"`;
    const codes = {
      'play_pause': 179,
      'next': 176,
      'prev': 177,
      'vol_up': 175,
      'vol_down': 174,
      'mute': 173
    };
    if (codes[value]) {
       spawn('powershell', ['-c', `$wshell = New-Object -ComObject wscript.shell; $wshell.SendKeys([char]${codes[value]})`], { detached: true, shell: true });
    }
  }
}

module.exports = { MacroExecutor };
