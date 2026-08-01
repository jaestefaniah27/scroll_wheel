const config = require('./config');

// Ejecuta en el PC las macros que el firmware solo anuncia (MACRO:<id> por
// CDC): el teclado no sabe mover el ratón ni pulsar teclas del sistema, así
// que cada acción de la secuencia se reproduce aquí con nut.js.

let nutjs = null;
let nutjsFailed = false;

function loadNutJs() {
  if (nutjs || nutjsFailed) return nutjs;
  try {
    nutjs = require('@nut-tree-fork/nut-js');
  } catch (err) {
    nutjsFailed = true;
    console.error('No se pudo cargar @nut-tree-fork/nut-js:', err.message);
  }
  return nutjs;
}

// Solo cubre lo que se puede grabar desde el editor de la app (letras, dígitos,
// Enter, Escape, Espacio). Para ampliarlo basta con añadir la entrada aquí: el
// editor ya captura cualquier KeyboardEvent.code, solo falta traducirlo.
function mapCodeToNutKey(code, Key) {
  if (/^Key[A-Z]$/.test(code)) return Key[code.slice(3)];
  if (/^Digit[0-9]$/.test(code)) return Key[`Num${code.slice(5)}`];
  if (code === 'Enter') return Key.Enter;
  if (code === 'Escape') return Key.Escape;
  if (code === 'Space') return Key.Space;
  return null;
}

const CLICK_BUTTONS = { left: 'LEFT', middle: 'MIDDLE', right: 'RIGHT' };

// Traduce el mismo usage HID (Usage Page 0x07) que usa el editor de atajos
// (ver src/hid-keys.js, KEY_GROUPS) al nombre de tecla de nut.js. Cubre
// exactamente los códigos que ofrece el selector de teclas de la app.
const HID_TO_NUT_KEY = {
  // Letras 0x04-0x1D: A-Z
  ...Object.fromEntries(Array.from({ length: 26 }, (_, i) => [0x04 + i, String.fromCharCode(65 + i)])),
  // Números (fila superior) 0x1e-0x26: 1-9, 0x27: 0
  0x1e: 'Num1', 0x1f: 'Num2', 0x20: 'Num3', 0x21: 'Num4', 0x22: 'Num5',
  0x23: 'Num6', 0x24: 'Num7', 0x25: 'Num8', 0x26: 'Num9', 0x27: 'Num0',
  // F1-F12 (0x3a-0x45), F13-F24 (0x68-0x73)
  ...Object.fromEntries(Array.from({ length: 12 }, (_, i) => [0x3a + i, `F${i + 1}`])),
  ...Object.fromEntries(Array.from({ length: 12 }, (_, i) => [0x68 + i, `F${i + 13}`])),
  // Edición
  0x28: 'Return', 0x29: 'Escape', 0x2a: 'Backspace', 0x2b: 'Tab', 0x2c: 'Space',
  0x49: 'Insert', 0x4c: 'Delete', 0x4a: 'Home', 0x4d: 'End',
  0x4b: 'PageUp', 0x4e: 'PageDown', 0x46: 'Print',
  // Flechas
  0x50: 'Left', 0x4f: 'Right', 0x52: 'Up', 0x51: 'Down',
  // Símbolos
  0x2d: 'Minus', 0x2e: 'Equal', 0x2f: 'LeftBracket', 0x30: 'RightBracket',
  0x31: 'Backslash', 0x33: 'Semicolon', 0x34: 'Quote', 0x35: 'Grave',
  0x36: 'Comma', 0x37: 'Period', 0x38: 'Slash',
  // Teclado numérico
  0x54: 'Divide', 0x55: 'Multiply', 0x56: 'Subtract', 0x57: 'Add', 0x58: 'Enter',
  0x59: 'NumPad1', 0x5a: 'NumPad2', 0x5b: 'NumPad3', 0x5c: 'NumPad4', 0x5d: 'NumPad5',
  0x5e: 'NumPad6', 0x5f: 'NumPad7', 0x60: 'NumPad8', 0x61: 'NumPad9', 0x62: 'NumPad0',
};

// Bits de src/hid-keys.js MODIFIERS.
const HID_MODIFIER_TO_NUT_KEY = {
  0x01: 'LeftControl', 0x02: 'LeftShift', 0x04: 'LeftAlt', 0x08: 'LeftSuper',
  0x10: 'RightControl', 0x20: 'RightShift', 0x40: 'RightAlt', 0x80: 'RightSuper',
};

function modifierKeys(modifier, Key) {
  const keys = [];
  for (const [bit, name] of Object.entries(HID_MODIFIER_TO_NUT_KEY)) {
    if (modifier & Number(bit)) keys.push(Key[name]);
  }
  return keys;
}

async function runAction(action) {
  const nut = loadNutJs();
  if (!nut) return;
  const { mouse, keyboard, screen, Point, Key, Button } = nut;

  if (action.type === 'center_mouse') {
    // Formato antiguo, de antes de poder capturar cualquier posición: se deja
    // por compatibilidad con secuencias ya guardadas.
    const [w, h] = await Promise.all([screen.width(), screen.height()]);
    await mouse.setPosition(new Point(w / 2, h / 2));
  } else if (action.type === 'mouse_position') {
    await mouse.setPosition(new Point(action.x, action.y));
  } else if (action.type === 'mouse_click') {
    const button = Button[CLICK_BUTTONS[action.button] || 'LEFT'];
    await mouse.click(button);
  } else if (action.type === 'delay') {
    await new Promise((resolve) => setTimeout(resolve, Math.max(0, action.ms || 0)));
  } else if (action.type === 'key') {
    const key = mapCodeToNutKey(action.code, Key);
    if (key === null || key === undefined) return;
    await keyboard.pressKey(key);
    await keyboard.releaseKey(key);
  } else if (action.type === 'hotkey') {
    // Cualquier tecla (con o sin modificadores) del mismo selector que usa la
    // pestaña Atajo: { modifier, keycode } en usage HID.
    const mods = modifierKeys(action.modifier || 0, Key);
    const mainName = HID_TO_NUT_KEY[action.keycode];
    const main = mainName ? Key[mainName] : undefined;
    const keys = main !== undefined ? [...mods, main] : mods;
    if (!keys.length) return;
    await keyboard.pressKey(...keys);
    await keyboard.releaseKey(...keys);
  }
}

async function executeMacro(macroId) {
  const macro = (config.load().macros || []).find((m) => m.id === macroId);
  if (!macro) return;

  for (const action of macro.actions || []) {
    try {
      await runAction(action);
    } catch (err) {
      console.error(`Macro ${macroId}, acción "${action.type}":`, err.message);
    }
  }
}

// Posición del cursor en el mismo sistema de coordenadas que usa mouse.setPosition.
// Antes se leía con el módulo screen de Electron, pero en pantallas con escalado
// (125%/150% de Windows) ese sistema no coincide con el de nut.js y las posiciones
// capturadas quedaban con un desplazamiento al reproducirlas. Preguntándoselo a
// nut.js directamente, captura y reproducción siempre hablan el mismo idioma.
async function getMousePosition() {
  const nut = loadNutJs();
  if (!nut) return { x: 0, y: 0 };
  const p = await nut.mouse.getPosition();
  return { x: Math.round(p.x), y: Math.round(p.y) };
}

module.exports = { executeMacro, getMousePosition };
