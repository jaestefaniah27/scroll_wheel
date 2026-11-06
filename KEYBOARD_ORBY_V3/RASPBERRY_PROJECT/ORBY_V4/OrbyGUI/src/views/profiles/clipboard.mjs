// Las reglas de copiar y pegar varias teclas, sin DOM ni teclado: qué teclas forman la
// selección múltiple y a qué tecla va cada una de las copiadas al pegar. keys.js hace
// la escritura; esto solo decide, para poder probarlo en Node.

// Ctrl+clic en una tecla la añade a la selección o la quita. La primera vez, la tecla
// que ya estaba elegida entra también: es lo que hace cualquier explorador de archivos.
// Devuelve la lista ordenada; con una sola tecla deja de ser una selección múltiple.
export function toggleKey(multi, selected, key) {
  const set = multi.length ? [...multi]
    : (selected !== null && selected !== undefined && selected !== key ? [selected] : []);
  const at = set.indexOf(key);
  if (at >= 0) set.splice(at, 1);
  else set.push(key);
  return set.sort((a, b) => a - b);
}

// A qué tecla va cada una de las copiadas. `items` lleva en `key` la tecla de origen.
//
// - Una copiada: va a cada tecla elegida (rellenar varias con la misma) o a la elegida.
// - Varias, con tantas elegidas como copiadas: en orden, la primera en la primera.
// - Varias, sin eso: cada una a su misma posición. Así se copia media rejilla de un
//   perfil a otro y queda igual.
export function pasteTargets(items, multi, selected) {
  if (!items?.length) return [];
  if (items.length === 1) {
    const keys = multi.length ? multi : (selected === null || selected === undefined ? [] : [selected]);
    return keys.map((key) => ({ item: items[0], key }));
  }
  const ordered = [...items].sort((a, b) => a.key - b.key);
  if (multi.length === items.length) {
    const keys = [...multi].sort((a, b) => a - b);
    return ordered.map((item, i) => ({ item, key: keys[i] }));
  }
  return ordered.map((item) => ({ item, key: item.key }));
}
