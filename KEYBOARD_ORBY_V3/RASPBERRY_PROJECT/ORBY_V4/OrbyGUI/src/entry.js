// La misma app arranca en tres sitios: dentro de Tauri, dentro de Electron y dentro de
// un navegador.
//
// Electron trae su puente puesto por `electron/preload.js` antes de que este script
// corra, así que el renderer puede empezar directamente. En Tauri y en el navegador no
// hay preload: hay que montar el equivalente antes de que ningún módulo de src/ toque
// `window.orby`, y por eso main.js entra por importación dinámica y no por `import`
// normal.
//
// Tauri se comprueba el primero a propósito: si algún día montara también un
// `window.orby` propio, la rama de Electron se lo tragaría y arrancaría por la vía
// equivocada sin decir nada.

if (window.__TAURI_INTERNALS__) {
  await import('./tauri-main.js');
} else if (window.orby) {
  await import('./main.js');
} else {
  await import('./web-main.js');
}
