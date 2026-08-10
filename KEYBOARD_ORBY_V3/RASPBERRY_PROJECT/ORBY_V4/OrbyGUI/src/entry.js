// La misma app arranca en dos sitios: dentro de Electron y dentro de un navegador.
//
// En Electron, `electron/preload.js` ya ha montado `window.orby` antes de que este
// script corra, así que el renderer puede empezar directamente. En un navegador no
// hay preload ni proceso principal: hay que montar el equivalente (Web Serial +
// IndexedDB) antes de que ningún módulo de src/ toque `window.orby`, y por eso
// main.js entra por importación dinámica y no por `import` normal.

if (window.orby) {
  await import('./main.js');
} else {
  await import('./web-main.js');
}
