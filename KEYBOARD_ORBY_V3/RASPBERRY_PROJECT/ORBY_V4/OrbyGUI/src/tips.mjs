// Los consejos: lo que enseña la vista Consejos y los avisos de vez en cuando.
//
// Es solo datos. Qué consejo toca y cuándo lo decide tips-pick.mjs; el vídeo de cada uno
// es `tips-video/<id>.mp4` (ver tips-video/README.md) y, si aún no está grabado, la vista
// enseña el texto con su hueco.
//
// `requiere` es una clave de FEATURES (compat.js): con un firmware que no la tiene, el
// consejo sale atenuado en la lista y no se avisa de él. Los gestos que el firmware trae
// desde antes de que existiera FEATURES (SUPER, zoom...) no llevan `requiere`.
//
// `accion` es el acceso directo: `vista` es una clave de VIEWS (main.js) y `params` lo que
// su `openTarget` entiende. Para Ajustes, `card` es el id del elemento al que llevar.

export const CATEGORIAS = [
  'Teclas y menús',
  'Rueda y mandos',
  'Macros',
  'Perfiles',
  'Pantallas',
  'App y teclado',
];

export const TIPS = [
  {
    id: 'modo-ayuda',
    categoria: 'Teclas y menús',
    titulo: 'Cada tecla puede decirte qué hace',
    texto: 'Pulsa SUPER dos veces seguidas, rápido, y las pantallas cambian el icono por el nombre de la acción. Es la forma de repasar un perfil que no recuerdas. Se sale con otro doble toque, o solo al cabo del tiempo que elijas.',
    requiere: 'helpMode',
    accion: { etiqueta: 'Elegir cuánto dura', vista: 'view-settings', params: { card: 'settings-help' } },
  },
  {
    id: 'super-menu',
    categoria: 'Teclas y menús',
    titulo: 'SUPER cambia lo que hace la tecla de menú',
    texto: 'Con SUPER sujeta, una pulsación corta de la tecla de menú retrocede de página (sin SUPER avanza) y una larga abre el menú principal con los ajustes del teclado, en lugar del selector de perfiles.',
  },
  {
    id: 'mando-menu',
    categoria: 'Rueda y mandos',
    titulo: 'Un mando puede ser tu menú',
    texto: 'Deja la tecla de menú libre y dedica un mando a ello: mantenlo pulsado, gíralo hasta el perfil que quieres y suéltalo para elegir. Girado sin mantener, cambia de página.',
    requiere: 'menuEncoder',
    accion: { etiqueta: 'Configurar el mando de menú', vista: 'view-settings', params: { card: 'settings-menukey' } },
  },
  {
    id: 'zoom-touchpad',
    categoria: 'Rueda y mandos',
    titulo: 'El zoom de la rueda es un pellizco de touchpad',
    texto: 'Un giro de rueda o de mando con la acción Zoom hace el mismo gesto que dos dedos en un touchpad: funciona en más programas que Ctrl + rueda y va suave, sin tirones. Si el equipo no admite touchpad, vuelve a Ctrl + rueda.',
    accion: { etiqueta: 'Poner Zoom en la rueda', vista: 'view-dashboard', params: { zoom: 'wheel' } },
  },
  {
    id: 'rueda-calibrar',
    categoria: 'Rueda y mandos',
    titulo: 'Haz que la rueda de la pantalla gire como la tuya',
    texto: 'La réplica del Inicio se alinea con la rueda real: gírala hasta tu marca de referencia, fíjala y elige hacia dónde cuenta. Así lo que ves en la app coincide con lo que tienes entre los dedos.',
    accion: { etiqueta: 'Calibrar la rueda', vista: 'view-settings', params: { card: 'settings-wheel-calib' } },
  },
  {
    id: 'invertir-giro',
    categoria: 'Rueda y mandos',
    titulo: 'Si un mando gira al revés, dale la vuelta',
    texto: 'Puedes invertir el sentido de cada mando y de la rueda para todo a la vez: acciones, menús y páginas. No toca lo que ya tengas configurado en cada perfil.',
    requiere: 'rotInvert',
    accion: { etiqueta: 'Invertir el giro', vista: 'view-settings', params: { card: 'settings-rotinv' } },
  },
  {
    id: 'paginas',
    categoria: 'Perfiles',
    titulo: 'Un perfil puede tener varias páginas',
    texto: 'Si diez teclas se te quedan cortas, añade páginas a un perfil y pasa de una a otra con la tecla de menú o con un mando. En la barra de arriba, los puntos del perfil activo son sus páginas.',
    requiere: 'pages',
    accion: { etiqueta: 'Gestionar las páginas', vista: 'view-profiles', params: { section: 'pages' } },
  },
  {
    id: 'biblioteca',
    categoria: 'Perfiles',
    titulo: 'Perfiles ya hechos para tus programas',
    texto: 'La biblioteca trae perfiles listos para Altium, KiCad, VS Code, Photoshop u Office, con sus iconos. Instala uno y cámbialo a tu gusto.',
    accion: { etiqueta: 'Abrir la biblioteca', vista: 'view-profiles', params: { section: 'library' } },
  },
  {
    id: 'cambio-automatico',
    categoria: 'Perfiles',
    titulo: 'El teclado cambia de perfil solo',
    texto: 'Dile qué programa va con qué perfil y, al ponerlo delante, el teclado cambia al perfil que toca. Necesita la app abierta.',
    accion: { etiqueta: 'Crear una regla', vista: 'view-auto' },
  },
  {
    id: 'perfiles-desactivados',
    categoria: 'Perfiles',
    titulo: 'Más de nueve perfiles, sin perder ninguno',
    texto: 'El teclado tiene nueve perfiles activos a la vez. Los demás se desactivan y se guardan, comprimidos, dentro del propio teclado, para volver a activarlos cuando los necesites.',
    requiere: 'profileArchive',
    accion: { etiqueta: 'Ver mis perfiles', vista: 'view-profiles' },
  },
  {
    id: 'macros-viajan',
    categoria: 'Macros',
    titulo: 'Tus macros viajan con el teclado',
    texto: 'Lo que hace cada tecla de macro se guarda también en el teclado. Si lo enchufas en otro ordenador con Orvy, sabe qué hace cada una sin que tengas que rehacerlo.',
    requiere: 'macroStore',
  },
  {
    id: 'texto-largo',
    categoria: 'Macros',
    titulo: 'Una tecla que escribe un texto entero',
    texto: 'Una tecla puede escribir una dirección, una firma o un párrafo. Si cabe, lo teclea el propio teclado; si no, lo pega Orvy de golpe.',
    requiere: 'macros',
    accion: { etiqueta: 'Configurar una tecla', vista: 'view-dashboard', params: { zoom: 'keys' } },
  },
  {
    id: 'grabadora',
    categoria: 'Macros',
    titulo: 'Grábate haciéndolo una vez',
    texto: 'En vez de montar una secuencia paso a paso, grábala: Orvy captura tu ratón y tu teclado y los repite al pulsar la tecla.',
    requiere: 'macros',
    accion: { etiqueta: 'Configurar una tecla', vista: 'view-dashboard', params: { zoom: 'keys' } },
  },
  {
    id: 'iconos-propios',
    categoria: 'Pantallas',
    titulo: 'Pon tu propio icono en cada tecla',
    texto: 'Cada tecla tiene su pantalla. Elige un icono del catálogo, dibuja el tuyo o sube una imagen, y Orvy la deja en blanco y negro para que se vea bien.',
    accion: { etiqueta: 'Elegir un icono', vista: 'view-dashboard', params: { zoom: 'keys' } },
  },
  {
    id: 'intro',
    categoria: 'Pantallas',
    titulo: 'Elige cómo arranca el teclado',
    texto: 'Al enchufarlo, las pantallas hacen una pequeña intro. Puedes quedarte con una, dejar que salga una al azar cada vez o verlas todas desde la app.',
    requiere: 'intro',
    accion: { etiqueta: 'Elegir la intro', vista: 'view-settings', params: { card: 'settings-intro' } },
  },
  {
    id: 'anim-velocidad',
    categoria: 'Pantallas',
    titulo: 'Las animaciones, a tu ritmo',
    texto: 'Abrir menús, pasar de página o cambiar de perfil se anima. Si te parece lento o rápido, ajusta la velocidad de cada grupo.',
    requiere: 'animSpeed',
    accion: { etiqueta: 'Ajustar la velocidad', vista: 'view-settings', params: { card: 'settings-animspeed' } },
  },
  {
    id: 'firmware-sin-boton',
    categoria: 'App y teclado',
    titulo: 'Actualiza el teclado sin tocar ningún botón',
    texto: 'Orvy puede poner el teclado en modo de carga y copiarle el firmware nuevo por sí sola. Ya no hace falta pulsar BOOTSEL ni desenchufar nada.',
    requiere: 'bootsel',
    accion: { etiqueta: 'Buscar actualizaciones', vista: 'view-settings', params: { card: 'settings-firmware' } },
  },
  {
    id: 'copias',
    categoria: 'App y teclado',
    titulo: 'Haz una copia de seguridad',
    texto: 'Guarda toda tu configuración en un archivo: perfiles, teclas y secuencias. Con él restauras el teclado, o lo llevas a otro ordenador.',
    accion: { etiqueta: 'Abrir las copias', vista: 'view-settings', params: { card: 'pane-datos' } },
  },
  {
    id: 'complementos',
    categoria: 'App y teclado',
    titulo: 'Conecta Orvy con otros aparatos',
    texto: 'Los complementos añaden acciones nuevas a las teclas, como encender una lámpara o llamar a un servicio de tu casa u oficina.',
    accion: { etiqueta: 'Ver los complementos', vista: 'view-settings', params: { card: 'settings-plugins' } },
  },
];
