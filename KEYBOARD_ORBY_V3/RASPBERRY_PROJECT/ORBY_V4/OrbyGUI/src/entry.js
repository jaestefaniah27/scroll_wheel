// La misma app arranca en dos sitios: dentro de Tauri y dentro de un navegador.
//
// En ninguno de los dos hay preload que monte el puente por su cuenta —lo tenía la vía
// de Electron, que ya no existe—, así que hay que montar el equivalente antes de que
// ningún módulo de src/ toque `window.orby`. Por eso el renderer entra por importación
// dinámica y no por `import` normal: un `import` estático se resolvería antes de que
// este fichero llegue a ejecutarse.

if (window.__TAURI_INTERNALS__) {
  await import('./tauri-main.js');
} else {
  await import('./web-main.js');
}
