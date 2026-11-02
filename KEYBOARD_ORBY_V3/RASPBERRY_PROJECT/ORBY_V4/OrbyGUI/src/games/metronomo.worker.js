// El reloj del motor de juegos, en un Worker.
//
// El jugador mira el teclado, no la ventana, así que la ventana de la app está casi
// siempre tapada o en segundo plano. Chromium frena entonces los temporizadores y
// `requestAnimationFrame` de la página (a uno por segundo, o a nada), y un juego a
// 100 fotogramas por segundo se quedaría a uno. Los Workers no se frenan: este
// solo cuenta y avisa, y la lógica del juego corre en el hilo principal al recibir el
// mensaje (los mensajes no se frenan aunque la página esté oculta).

let id = null;

self.onmessage = (e) => {
  clearInterval(id);
  id = null;
  if (e.data?.ms) id = setInterval(() => self.postMessage(0), e.data.ms);
};
