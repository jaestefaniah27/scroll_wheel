// El puerto entrega trozos de bytes, no líneas: un mensaje del teclado puede
// llegar partido en dos lecturas y dos mensajes pueden llegar en la misma.
//
// Es lo mismo que hace el ReadlineParser de src-tauri/src/serial.rs, pero aquí no hay
// ningún parser de serialport que lo haga por nosotros. Se saca a su propio fichero
// porque es la única parte de todo el transporte que se puede probar sin un teclado
// enchufado, y porque equivocarse aquí se manifiesta como peticiones que caducan a
// los 4 s sin ninguna pista de por qué.

export function splitLines(resto, trozo) {
  const partes = (resto + trozo).split('\n');
  // El último elemento no termina en \n: o es una línea a medias o es cadena vacía.
  const cola = partes.pop() ?? '';
  const lineas = [];
  for (const parte of partes) {
    const limpia = parte.trim();
    if (limpia) lineas.push(limpia);
  }
  return { lineas, resto: cola };
}
