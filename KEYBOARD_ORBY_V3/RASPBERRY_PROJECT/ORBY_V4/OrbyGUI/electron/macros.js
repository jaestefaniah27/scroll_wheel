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

async function runAction(action) {
  const nut = loadNutJs();
  if (!nut) return;
  const { mouse, keyboard, screen, Point, Key } = nut;

  if (action.type === 'center_mouse') {
    const [w, h] = await Promise.all([screen.width(), screen.height()]);
    await mouse.setPosition(new Point(w / 2, h / 2));
  } else if (action.type === 'mouse_click') {
    await mouse.leftClick();
  } else if (action.type === 'delay') {
    await new Promise((resolve) => setTimeout(resolve, Math.max(0, action.ms || 0)));
  } else if (action.type === 'key') {
    const key = mapCodeToNutKey(action.code, Key);
    if (key === null || key === undefined) return;
    await keyboard.pressKey(key);
    await keyboard.releaseKey(key);
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

module.exports = { executeMacro };
