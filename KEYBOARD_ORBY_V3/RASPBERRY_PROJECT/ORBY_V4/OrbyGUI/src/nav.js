// Salto entre vistas desde cualquier módulo.
//
// Existe para que el editor de perfiles pueda mandarte al editor de iconos con
// la tecla ya seleccionada. Sin esto, las vistas tendrían que importarse entre
// sí y main.js dejaría de ser el único sitio que sabe qué vista está activa.

let navigator = null;

export function setNavigator(fn) {
  navigator = fn;
}

export function goTo(target, params) {
  navigator?.(target, params);
}

// Volver a la vista de la que se vino. El editor de iconos se abre desde el
// panel del Inicio o desde la vista de Perfiles, y tiene que volver a la misma.
let back = null;

export function setBack(fn) {
  back = fn;
}

export function goBack() {
  back?.();
}
