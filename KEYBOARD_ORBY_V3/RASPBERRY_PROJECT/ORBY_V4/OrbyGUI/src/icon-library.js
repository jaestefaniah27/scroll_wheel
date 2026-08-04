// Biblioteca de iconos lista para usar en las pantallas de las teclas.
//
// El editor OLED ya sabía importar imágenes y generar texto, pero eso obliga a
// que cada usuario se busque la vida con sus propios PNG. Aquí van ~170 iconos
// vectoriales dibujados a mano en una rejilla de 24x24, pensados para que se
// lean bien una vez bajados a 1 bit en una pantalla de 72x40.
//
// No se cargan de internet ni de node_modules: son cadenas SVG en este archivo,
// igual que icons.js, porque la app tiene que funcionar sin conexión.
//
// Convenios del trazado:
//   - El <svg> raíz pone fill="none", stroke="currentColor" y grosor 2.
//   - Lo que tenga que ir macizo lleva F (fill="currentColor" stroke="none").
//   - Todo usa currentColor: el panel lo pinta del color del tema y el
//     rasterizador lo pinta de blanco (ver makeSvgSource en oled-fb.js).

const F = 'fill="currentColor" stroke="none"';

export const CATEGORIES = [
  { id: 'basic',  label: 'Formas y flechas' },
  { id: 'media',  label: 'Multimedia' },
  { id: 'edit',   label: 'Edición' },
  { id: 'system', label: 'Sistema' },
  { id: 'dev',    label: 'Programación' },
  { id: 'comm',   label: 'Comunicación' },
  { id: 'misc',   label: 'Varios' },
];

// [id, nombre, palabras clave extra, trazado]
const RAW = {
  basic: [
    ['arrow-up', 'Flecha arriba', 'subir norte', '<path d="M12 20V4M5 11l7-7 7 7"/>'],
    ['arrow-down', 'Flecha abajo', 'bajar sur', '<path d="M12 4v16M5 13l7 7 7-7"/>'],
    ['arrow-left', 'Flecha izquierda', 'atras oeste', '<path d="M20 12H4M11 5l-7 7 7 7"/>'],
    ['arrow-right', 'Flecha derecha', 'siguiente este', '<path d="M4 12h16M13 5l7 7-7 7"/>'],
    ['arrow-ul', 'Flecha arriba izquierda', 'diagonal', '<path d="M18 18L6 6M6 14V6h8"/>'],
    ['arrow-ur', 'Flecha arriba derecha', 'diagonal', '<path d="M6 18L18 6M18 14V6h-8"/>'],
    ['arrow-dl', 'Flecha abajo izquierda', 'diagonal', '<path d="M18 6L6 18M6 10v8h8"/>'],
    ['arrow-dr', 'Flecha abajo derecha', 'diagonal', '<path d="M6 6l12 12M18 10v8h-8"/>'],
    ['chevron-up', 'Punta arriba', 'desplegar', '<path d="M5 15l7-7 7 7"/>'],
    ['chevron-down', 'Punta abajo', 'desplegar menu', '<path d="M5 9l7 7 7-7"/>'],
    ['chevron-left', 'Punta izquierda', 'anterior', '<path d="M15 5l-7 7 7 7"/>'],
    ['chevron-right', 'Punta derecha', 'siguiente', '<path d="M9 5l7 7-7 7"/>'],
    ['chevrons-left', 'Doble punta izquierda', 'inicio retroceder', '<path d="M11 18l-6-6 6-6M19 18l-6-6 6-6"/>'],
    ['chevrons-right', 'Doble punta derecha', 'fin avanzar', '<path d="M13 6l6 6-6 6M5 6l6 6-6 6"/>'],
    ['arrows-h', 'Flechas horizontales', 'ancho ajustar', '<path d="M3 12h18M6.5 8.5L3 12l3.5 3.5M17.5 8.5L21 12l-3.5 3.5"/>'],
    ['arrows-v', 'Flechas verticales', 'alto ajustar', '<path d="M12 3v18M8.5 6.5L12 3l3.5 3.5M8.5 17.5L12 21l3.5-3.5"/>'],
    ['rotate-cw', 'Girar a la derecha', 'rehacer recargar', '<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>'],
    ['rotate-ccw', 'Girar a la izquierda', 'deshacer recargar', '<path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/>'],
    ['shuffle', 'Aleatorio', 'mezclar random', '<path d="M3 6h4l10 12h4M3 18h4l2.5-3M14.5 8.5L17 6h4"/><path d="M18 3l3 3-3 3M18 15l3 3-3 3"/>'],
    ['repeat', 'Repetir', 'bucle loop', '<path d="M4 10V8a3 3 0 0 1 3-3h13"/><path d="M17 2l3 3-3 3"/><path d="M20 14v2a3 3 0 0 1-3 3H4"/><path d="M7 22l-3-3 3-3"/>'],
    ['plus', 'Más', 'añadir sumar nuevo', '<path d="M12 4v16M4 12h16"/>'],
    ['minus', 'Menos', 'quitar restar', '<path d="M4 12h16"/>'],
    ['close', 'Cerrar', 'equis cancelar salir', '<path d="M5 5l14 14M19 5L5 19"/>'],
    ['check', 'Confirmar', 'ok aceptar tick visto', '<path d="M4 12.5l5.5 5.5L20 6"/>'],
    ['check-circle', 'Confirmado', 'ok aceptar', '<circle cx="12" cy="12" r="9"/><path d="M7.5 12.5l3 3 6-6.5"/>'],
    ['x-circle', 'Cancelar', 'error borrar', '<circle cx="12" cy="12" r="9"/><path d="M8.5 8.5l7 7M15.5 8.5l-7 7"/>'],
    ['circle', 'Círculo', 'redondo', '<circle cx="12" cy="12" r="8"/>'],
    ['circle-fill', 'Círculo macizo', 'punto bola', `<circle cx="12" cy="12" r="8" ${F}/>`],
    ['square', 'Cuadrado', 'caja', '<rect x="4" y="4" width="16" height="16" rx="2"/>'],
    ['square-fill', 'Cuadrado macizo', 'caja bloque', `<rect x="4" y="4" width="16" height="16" rx="2" ${F}/>`],
    ['triangle', 'Triángulo', 'aviso', '<path d="M12 4l9 16H3z"/>'],
    ['triangle-fill', 'Triángulo macizo', 'aviso', `<path d="M12 4l9 16H3z" ${F}/>`],
    ['star', 'Estrella', 'favorito destacar', '<path d="M12 3.5l2.7 5.5 6.1.9-4.4 4.3 1 6.1-5.4-2.9-5.4 2.9 1-6.1L3.2 9.9l6.1-.9z"/>'],
    ['star-fill', 'Estrella maciza', 'favorito destacar', `<path d="M12 3.5l2.7 5.5 6.1.9-4.4 4.3 1 6.1-5.4-2.9-5.4 2.9 1-6.1L3.2 9.9l6.1-.9z" ${F}/>`],
    ['heart', 'Corazón', 'favorito megusta', '<path d="M12 20.5S4 15.5 4 10.2A4.2 4.2 0 0 1 12 7.6a4.2 4.2 0 0 1 8 2.6c0 5.3-8 10.3-8 10.3z"/>'],
    ['heart-fill', 'Corazón macizo', 'favorito megusta', `<path d="M12 20.5S4 15.5 4 10.2A4.2 4.2 0 0 1 12 7.6a4.2 4.2 0 0 1 8 2.6c0 5.3-8 10.3-8 10.3z" ${F}/>`],
    ['diamond', 'Rombo', 'diamante', '<path d="M12 3l9 9-9 9-9-9z"/>'],
    ['hexagon', 'Hexágono', 'panal', '<path d="M12 2.5l8 4.6v9.8l-8 4.6-8-4.6V7.1z"/>'],
  ],

  media: [
    ['play', 'Reproducir', 'play iniciar', `<path d="M7 4.5l12 7.5-12 7.5z" ${F}/>`],
    ['play-circle', 'Reproducir en círculo', 'play', `<circle cx="12" cy="12" r="9"/><path d="M10 8.5l6 3.5-6 3.5z" ${F}/>`],
    ['pause', 'Pausa', 'pausar', `<rect x="6" y="4" width="4" height="16" rx="1" ${F}/><rect x="14" y="4" width="4" height="16" rx="1" ${F}/>`],
    ['pause-circle', 'Pausa en círculo', 'pausar', '<circle cx="12" cy="12" r="9"/><path d="M10 8v8M14 8v8"/>'],
    ['stop', 'Parar', 'detener stop', `<rect x="5" y="5" width="14" height="14" rx="2" ${F}/>`],
    ['record', 'Grabar', 'rec grabacion', `<circle cx="12" cy="12" r="7" ${F}/>`],
    ['skip-next', 'Siguiente pista', 'next adelante', `<path d="M5 4.5l11 7.5-11 7.5z" ${F}/><rect x="17" y="4.5" width="3" height="15" rx="1" ${F}/>`],
    ['skip-prev', 'Pista anterior', 'prev atras', `<path d="M19 4.5v15L8 12z" ${F}/><rect x="4" y="4.5" width="3" height="15" rx="1" ${F}/>`],
    ['fast-forward', 'Avance rápido', 'ff acelerar', `<path d="M3 5l9 7-9 7z" ${F}/><path d="M13 5l9 7-9 7z" ${F}/>`],
    ['rewind', 'Retroceso rápido', 'rw rebobinar', `<path d="M21 5l-9 7 9 7z" ${F}/><path d="M11 5l-9 7 9 7z" ${F}/>`],
    ['volume-high', 'Volumen alto', 'sonido subir altavoz', `<path d="M4 9h4l5-4v14l-5-4H4z" ${F}/><path d="M16.5 8.5a5 5 0 0 1 0 7M19.5 6a9 9 0 0 1 0 12"/>`],
    ['volume-low', 'Volumen bajo', 'sonido bajar altavoz', `<path d="M4 9h4l5-4v14l-5-4H4z" ${F}/><path d="M16.5 8.5a5 5 0 0 1 0 7"/>`],
    ['volume-mute', 'Silencio', 'mute sonido apagado', `<path d="M4 9h4l5-4v14l-5-4H4z" ${F}/><path d="M16.5 9l5 6M21.5 9l-5 6"/>`],
    ['mic', 'Micrófono', 'grabar voz', '<rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v4M8 22h8"/>'],
    ['mic-off', 'Micrófono apagado', 'silenciar mute voz', '<rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v4M8 22h8"/><path d="M3 3l18 18"/>'],
    ['headphones', 'Auriculares', 'cascos audio', '<path d="M4 15v-3a8 8 0 0 1 16 0v3"/><rect x="2" y="14" width="5" height="7" rx="2"/><rect x="17" y="14" width="5" height="7" rx="2"/>'],
    ['music', 'Música', 'nota cancion', '<path d="M9 18V5l11-2v13"/><circle cx="6" cy="18" r="3"/><circle cx="17" cy="16" r="3"/>'],
    ['playlist', 'Lista de reproducción', 'cola', '<path d="M3 6h12M3 11h12M3 16h7"/><circle cx="17" cy="17.5" r="2.5"/><path d="M19.5 17.5V9l2.5 1"/>'],
    ['camera', 'Cámara', 'foto captura', '<path d="M3 9a2 2 0 0 1 2-2h2l1.5-2.5h7L17 7h2a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><circle cx="12" cy="13" r="3.5"/>'],
    ['video', 'Vídeo', 'camara grabar', '<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10.5l6-3.5v10l-6-3.5z"/>'],
    ['video-off', 'Vídeo apagado', 'camara cortar', '<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10.5l6-3.5v10l-6-3.5z"/><path d="M3 3l18 18"/>'],
    ['image', 'Imagen', 'foto galeria', '<rect x="3" y="4" width="18" height="16" rx="2"/><circle cx="8.5" cy="9.5" r="1.8"/><path d="M4 18l5-5 3.5 3.5L16 13l4 4"/>'],
    ['film', 'Película', 'cine video', '<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M7 4v16M17 4v16M2 9h5M2 15h5M17 9h5M17 15h5"/>'],
    ['tv', 'Televisión', 'pantalla monitor', '<rect x="2" y="6" width="20" height="13" rx="2"/><path d="M8 2l4 4 4-4"/>'],
    ['cast', 'Enviar a pantalla', 'streaming airplay', '<path d="M2 16.5a5.5 5.5 0 0 1 5.5 5.5M2 12a10 10 0 0 1 10 10"/><path d="M2 8V6a2 2 0 0 1 2-2h16a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2h-5"/>'],
    ['eject', 'Expulsar', 'sacar disco', `<path d="M12 4l8 10H4z" ${F}/><rect x="4" y="17" width="16" height="3" rx="1" ${F}/>`],
  ],

  edit: [
    ['copy', 'Copiar', 'duplicar ctrlc', '<rect x="8" y="8" width="12" height="13" rx="2"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>'],
    ['paste', 'Pegar', 'ctrlv portapapeles', '<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v1"/><path d="M9 12h6M9 16h6"/>'],
    ['scissors', 'Cortar', 'tijeras ctrlx', '<circle cx="6" cy="6" r="2.5"/><circle cx="6" cy="18" r="2.5"/><path d="M8 7.5L20 19M20 5L8 16.5"/>'],
    ['undo', 'Deshacer', 'ctrlz atras', '<path d="M4 12h11a5 5 0 0 1 0 10h-3"/><path d="M8 8l-4 4 4 4"/>'],
    ['redo', 'Rehacer', 'ctrly adelante', '<path d="M20 12H9a5 5 0 0 0 0 10h3"/><path d="M16 8l4 4-4 4"/>'],
    ['save', 'Guardar', 'ctrls disquete', '<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><path d="M17 21v-8H7v8M7 3v5h8"/>'],
    ['trash', 'Borrar', 'papelera eliminar', '<path d="M3 6h18M8 6V4h8v2"/><path d="M6 6l1 14h10l1-14"/><path d="M10 10v7M14 10v7"/>'],
    ['pencil', 'Lápiz', 'editar escribir', '<path d="M17 3l4 4L8 20l-5 1 1-5z"/><path d="M14.5 5.5l4 4"/>'],
    ['brush', 'Pincel', 'pintar dibujar', '<path d="M9 14l-3 3c-1 1-1 3 0 4s3 1 4 0l3-3z"/><path d="M11 12l8-8 3 3-8 8z"/>'],
    ['eraser', 'Borrador', 'goma', '<path d="M8.5 20.5l-5-5a2 2 0 0 1 0-2.8L13.6 2.6a2 2 0 0 1 2.8 0l4.5 4.5a2 2 0 0 1 0 2.8L11 20.5z"/><path d="M9 20.5h11M8.5 8l7 7"/>'],
    ['text', 'Texto', 'letra fuente tipografia', '<path d="M4 7V4h16v3M12 4v16M8 20h8"/>'],
    ['bold', 'Negrita', 'grueso', '<path d="M7 4h6a4 4 0 0 1 0 8H7zM7 12h7a4 4 0 0 1 0 8H7z"/>'],
    ['italic', 'Cursiva', 'inclinada', '<path d="M10 4h8M6 20h8M14.5 4l-5 16"/>'],
    ['underline', 'Subrayado', 'linea', '<path d="M7 4v7a5 5 0 0 0 10 0V4M5 20h14"/>'],
    ['align-left', 'Alinear a la izquierda', 'parrafo', '<path d="M3 5h18M3 10h11M3 15h15M3 20h8"/>'],
    ['align-center', 'Centrar', 'parrafo', '<path d="M3 5h18M6.5 10h11M4 15h16M8 20h8"/>'],
    ['align-right', 'Alinear a la derecha', 'parrafo', '<path d="M3 5h18M10 10h11M6 15h15M13 20h8"/>'],
    ['list-ul', 'Lista', 'viñetas puntos', `<path d="M9 6h12M9 12h12M9 18h12"/><circle cx="4.5" cy="6" r="1.4" ${F}/><circle cx="4.5" cy="12" r="1.4" ${F}/><circle cx="4.5" cy="18" r="1.4" ${F}/>`],
    ['list-ol', 'Lista numerada', 'orden', '<path d="M9 6h12M9 12h12M9 18h12"/><path d="M3 4.5h1.5V9M3 10.5h2.5L3 14.5h2.5M3 16h2.5v2H3.5v2H6"/>'],
    ['link', 'Enlace', 'url vinculo', '<path d="M10.5 13.5a4.5 4.5 0 0 0 6.4 0l2.5-2.5a4.5 4.5 0 0 0-6.4-6.4l-1.4 1.4"/><path d="M13.5 10.5a4.5 4.5 0 0 0-6.4 0l-2.5 2.5a4.5 4.5 0 0 0 6.4 6.4l1.4-1.4"/>'],
    ['unlink', 'Quitar enlace', 'romper vinculo', '<path d="M9 15l-1.5 1.5a4.5 4.5 0 0 1-6.4-6.4L4 7.5"/><path d="M15 9l1.5-1.5a4.5 4.5 0 0 1 6.4 6.4L20 16.5"/><path d="M3 3l18 18"/>'],
    ['crop', 'Recortar', 'encuadre', '<path d="M6 2v16h16M2 6h16v16"/>'],
    ['layers', 'Capas', 'niveles', '<path d="M12 3l9 5-9 5-9-5z"/><path d="M3 13l9 5 9-5"/>'],
    ['palette', 'Paleta', 'color pintura', `<path d="M12 3a9 9 0 0 0 0 18c1.5 0 2-1 1.5-2s0-2 1.5-2H18a3 3 0 0 0 3-3c0-5-4-9-9-9z"/><circle cx="7.5" cy="12.5" r="1.3" ${F}/><circle cx="9.5" cy="8.5" r="1.3" ${F}/><circle cx="14" cy="7.5" r="1.3" ${F}/>`],
    ['zoom-in', 'Acercar', 'lupa aumentar zoom', '<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21M8 10.5h5M10.5 8v5"/>'],
    ['zoom-out', 'Alejar', 'lupa reducir zoom', '<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21M8 10.5h5"/>'],
    ['grid', 'Rejilla', 'cuadricula tabla', '<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18"/>'],
    ['move', 'Mover', 'desplazar arrastrar', '<path d="M12 3v18M3 12h18"/><path d="M9 6l3-3 3 3M9 18l3 3 3-3M6 9l-3 3 3 3M18 9l3 3-3 3"/>'],
    ['magic-wand', 'Varita mágica', 'automatico efecto', '<path d="M15 4.5l4.5 4.5L9 19.5 4.5 15z"/><path d="M12.5 7l4.5 4.5"/><path d="M19 2v3M22.5 4.5h-3M21 9l1.5.8"/>'],
    ['pipette', 'Cuentagotas', 'color muestra', '<path d="M18 2l4 4-2.5 2.5-1-1L8 20l-4 1 1-4L16.5 5.5l-1-1z"/>'],
  ],

  system: [
    ['power', 'Encendido', 'apagar boton power', '<path d="M12 3v9"/><path d="M7.5 6.5a7 7 0 1 0 9 0"/>'],
    ['gear', 'Ajustes', 'configuracion opciones tuerca', '<path d="M10.3 3h3.4l.4 2.3 1.9.8 1.9-1.3 2.4 2.4-1.3 1.9.8 1.9 2.3.4v3.4l-2.3.4-.8 1.9 1.3 1.9-2.4 2.4-1.9-1.3-1.9.8-.4 2.3h-3.4l-.4-2.3-1.9-.8-1.9 1.3-2.4-2.4 1.3-1.9-.8-1.9L2.5 13.7v-3.4l2.3-.4.8-1.9-1.3-1.9 2.4-2.4 1.9 1.3 1.9-.8z"/><circle cx="12" cy="12" r="3"/>'],
    ['sliders', 'Controles', 'ecualizador ajustes', '<path d="M4 6h16M4 12h16M4 18h16"/><circle cx="9" cy="6" r="2.2"/><circle cx="15" cy="12" r="2.2"/><circle cx="8" cy="18" r="2.2"/>'],
    ['home', 'Inicio', 'casa principal', '<path d="M3 11l9-7 9 7"/><path d="M5.5 9.5V20h13V9.5"/><path d="M10 20v-5h4v5"/>'],
    ['search', 'Buscar', 'lupa encontrar', '<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5L21 21"/>'],
    ['lock', 'Bloqueado', 'candado seguridad', '<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/>'],
    ['unlock', 'Desbloqueado', 'candado abierto', '<rect x="4" y="10" width="16" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 7.6-2"/>'],
    ['key', 'Llave', 'clave contraseña', '<circle cx="8" cy="12" r="4"/><path d="M12 12h9M17 12v4M20 12v3"/>'],
    ['user', 'Usuario', 'perfil persona cuenta', '<circle cx="12" cy="8" r="4"/><path d="M4 21c0-4.4 3.6-7 8-7s8 2.6 8 7"/>'],
    ['users', 'Usuarios', 'grupo equipo', '<circle cx="9" cy="8" r="3.5"/><path d="M2 21c0-4 3.1-6.5 7-6.5s7 2.5 7 6.5"/><path d="M16 5.3a3.5 3.5 0 0 1 0 6.9M18 14.4c2.4.8 4 2.8 4 5.6"/>'],
    ['bell', 'Aviso', 'notificacion campana alerta', '<path d="M6 16.5V11a6 6 0 0 1 12 0v5.5l2 2.5H4z"/><path d="M10 22h4"/>'],
    ['bell-off', 'Avisos apagados', 'silenciar notificacion', '<path d="M6 16.5V11a6 6 0 0 1 12 0v5.5l2 2.5H4z"/><path d="M10 22h4"/><path d="M3 3l18 18"/>'],
    ['wifi', 'Wi-Fi', 'red inalambrica', `<path d="M2.5 8.5a15 15 0 0 1 19 0"/><path d="M5.8 12.2a10 10 0 0 1 12.4 0"/><path d="M9 15.9a5 5 0 0 1 6 0"/><circle cx="12" cy="19.5" r="1.4" ${F}/>`],
    ['wifi-off', 'Sin Wi-Fi', 'red caida', `<path d="M5.8 12.2a10 10 0 0 1 8-2.1"/><path d="M9 15.9a5 5 0 0 1 4-.8"/><circle cx="12" cy="19.5" r="1.4" ${F}/><path d="M3 3l18 18"/>`],
    ['bluetooth', 'Bluetooth', 'emparejar', '<path d="M7 7l10 10-5 4V3l5 4L7 17"/>'],
    ['battery', 'Batería', 'pila carga', `<rect x="2" y="7" width="17" height="10" rx="2.5"/><path d="M21.5 10.5v3"/><rect x="4.5" y="9.5" width="7" height="5" rx="1" ${F}/>`],
    ['battery-charging', 'Cargando', 'bateria enchufe', '<path d="M14 7h3a2.5 2.5 0 0 1 2 2v6a2.5 2.5 0 0 1-2 2h-4"/><path d="M8 7H4.5a2.5 2.5 0 0 0-2.5 2.5v5A2.5 2.5 0 0 0 4.5 17H8"/><path d="M22 10.5v3"/><path d="M12 5.5L8.5 12H12l-1 6.5 4.5-7H12z"/>'],
    ['usb', 'USB', 'conector cable', `<circle cx="12" cy="20.5" r="1.6" ${F}/><path d="M12 19V4"/><path d="M9 7l3-4 3 4"/><path d="M12 13l4-3V7.5"/><circle cx="16" cy="6.5" r="1.6" ${F}/><path d="M12 16l-4-3V10"/><rect x="6.5" y="7.5" width="3" height="2.5" ${F}/>`],
    ['monitor', 'Monitor', 'pantalla ordenador', '<rect x="2" y="4" width="20" height="13" rx="2"/><path d="M8 21h8M12 17v4"/>'],
    ['laptop', 'Portátil', 'ordenador', '<rect x="4" y="5" width="16" height="11" rx="2"/><path d="M2 19h20"/>'],
    ['keyboard', 'Teclado', 'teclas escribir', '<rect x="2" y="6" width="20" height="12" rx="2"/><path d="M6 10h.01M10 10h.01M14 10h.01M18 10h.01M8 14h8"/>'],
    ['mouse', 'Ratón', 'puntero clic', '<rect x="7" y="3" width="10" height="18" rx="5"/><path d="M12 7v3.5"/>'],
    ['cpu', 'Procesador', 'chip micro', `<rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9.5" y="9.5" width="5" height="5" rx="1" ${F}/><path d="M9 3v3M15 3v3M9 18v3M15 18v3M3 9h3M3 15h3M18 9h3M18 15h3"/>`],
    ['hdd', 'Disco', 'almacenamiento unidad', `<rect x="2" y="9" width="20" height="10" rx="2"/><path d="M5 4h14l3 5H2z"/><circle cx="17.5" cy="14" r="1.4" ${F}/>`],
    ['printer', 'Impresora', 'imprimir', '<path d="M6 9V3h12v6"/><rect x="2" y="9" width="20" height="8" rx="2"/><path d="M6 14h12v7H6z"/>'],
    ['calculator', 'Calculadora', 'numeros cuentas', '<rect x="4" y="2" width="16" height="20" rx="2"/><rect x="7" y="5" width="10" height="4" rx="1"/><path d="M8 13h.01M12 13h.01M16 13h.01M8 17.5h.01M12 17.5h.01M16 17.5h.01"/>'],
    ['clock', 'Reloj', 'hora tiempo', '<circle cx="12" cy="12" r="9"/><path d="M12 6.5V12l3.5 2"/>'],
    ['alarm', 'Alarma', 'despertador aviso', '<circle cx="12" cy="13.5" r="7.5"/><path d="M12 9.5v4l3 2"/><path d="M5.5 3L2.5 6M18.5 3l3 3"/>'],
    ['calendar', 'Calendario', 'fecha agenda', '<rect x="3" y="5" width="18" height="16" rx="2"/><path d="M3 10h18M8 3v4M16 3v4"/>'],
    ['folder', 'Carpeta', 'directorio archivos', '<path d="M3 7a2 2 0 0 1 2-2h4l2 2.5h8a2 2 0 0 1 2 2V18a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/>'],
    ['folder-open', 'Carpeta abierta', 'directorio', '<path d="M3 8a2 2 0 0 1 2-2h4l2 2.5h6a2 2 0 0 1 2 2V12"/><path d="M3 20l2.6-7.3a1.5 1.5 0 0 1 1.4-1H20.5a1 1 0 0 1 .95 1.3L19 20z"/>'],
    ['file', 'Archivo', 'documento fichero', '<path d="M14 3H7a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V8z"/><path d="M14 3v5h5"/>'],
    ['file-text', 'Documento', 'texto informe', '<path d="M14 3H7a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V8z"/><path d="M14 3v5h5M9 13h6M9 17h4"/>'],
    ['files', 'Archivos', 'documentos varios', '<rect x="8" y="3" width="12" height="15" rx="2"/><path d="M16 21H6a2 2 0 0 1-2-2V7"/>'],
    ['download', 'Descargar', 'bajar guardar', '<path d="M12 3v12M7 11l5 5 5-5"/><path d="M4 20h16"/>'],
    ['upload', 'Subir', 'enviar cargar', '<path d="M12 21V9M7 13l5-5 5 5"/><path d="M4 4h16"/>'],
    ['sync', 'Sincronizar', 'actualizar refrescar', '<path d="M20 11a8 8 0 0 0-13.7-5.2L4 8"/><path d="M4 3.5V8h4.5"/><path d="M4 13a8 8 0 0 0 13.7 5.2L20 16"/><path d="M20 20.5V16h-4.5"/>'],
    ['cloud', 'Nube', 'internet online', '<path d="M6.8 19a4.5 4.5 0 0 1-.4-9 6 6 0 0 1 11.4 1 4.2 4.2 0 0 1-.6 8z"/>'],
    ['cloud-up', 'Subir a la nube', 'backup copia', '<path d="M7 16.5a4.2 4.2 0 0 1-.6-8.4 6 6 0 0 1 11.4 1 4 4 0 0 1 .2 7.4"/><path d="M12 21v-8M9 15.5l3-3 3 3"/>'],
    ['cloud-down', 'Bajar de la nube', 'restaurar descarga', '<path d="M7 16.5a4.2 4.2 0 0 1-.6-8.4 6 6 0 0 1 11.4 1 4 4 0 0 1 .2 7.4"/><path d="M12 13v8M9 18.5l3 3 3-3"/>'],
    ['database', 'Base de datos', 'datos sql', '<ellipse cx="12" cy="6" rx="8" ry="3"/><path d="M4 6v12c0 1.7 3.6 3 8 3s8-1.3 8-3V6"/><path d="M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3"/>'],
    ['server', 'Servidor', 'rack hosting', '<rect x="3" y="4" width="18" height="7" rx="2"/><rect x="3" y="13" width="18" height="7" rx="2"/><path d="M7 7.5h.01M7 16.5h.01"/>'],
    ['network', 'Red', 'nodos conexion', '<rect x="9" y="2" width="6" height="6" rx="1"/><rect x="2" y="16" width="6" height="6" rx="1"/><rect x="16" y="16" width="6" height="6" rx="1"/><path d="M12 8v3M5 16v-3h14v3"/>'],
    ['shield', 'Escudo', 'seguridad proteccion', '<path d="M12 3l8 3v6c0 4.5-3.3 8-8 9-4.7-1-8-4.5-8-9V6z"/>'],
    ['shield-check', 'Protegido', 'seguridad ok antivirus', '<path d="M12 3l8 3v6c0 4.5-3.3 8-8 9-4.7-1-8-4.5-8-9V6z"/><path d="M8.5 12l2.5 2.5 4.5-5"/>'],
    ['plug', 'Enchufe', 'conectar corriente', '<path d="M9 2v6M15 2v6M6 8h12v3a6 6 0 0 1-12 0z"/><path d="M12 17v5"/>'],
    ['menu', 'Menú', 'hamburguesa opciones', '<path d="M4 7h16M4 12h16M4 17h16"/>'],
    ['more-h', 'Más opciones', 'puntos horizontal', `<circle cx="5" cy="12" r="1.8" ${F}/><circle cx="12" cy="12" r="1.8" ${F}/><circle cx="19" cy="12" r="1.8" ${F}/>`],
    ['more-v', 'Más opciones vertical', 'puntos', `<circle cx="12" cy="5" r="1.8" ${F}/><circle cx="12" cy="12" r="1.8" ${F}/><circle cx="12" cy="19" r="1.8" ${F}/>`],
    ['filter', 'Filtrar', 'embudo criterios', '<path d="M3 5h18l-7 8v6l-4 2v-8z"/>'],
    ['sort', 'Ordenar', 'clasificar', '<path d="M3 7h10M3 12h7M3 17h4"/><path d="M17 5v14M14 16l3 3 3-3"/>'],
    ['eye', 'Ver', 'ojo mostrar visible', '<path d="M2 12s3.6-6.5 10-6.5S22 12 22 12s-3.6 6.5-10 6.5S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>'],
    ['eye-off', 'Ocultar', 'ojo invisible', '<path d="M4.5 7.5C3 9.2 2 12 2 12s3.6 6.5 10 6.5c2 0 3.7-.5 5.1-1.3M9.5 5.8A11 11 0 0 1 12 5.5c6.4 0 10 6.5 10 6.5s-1 1.8-2.7 3.5"/><path d="M9.9 9.9a3 3 0 0 0 4.2 4.2"/><path d="M3 3l18 18"/>'],
    ['maximize', 'Maximizar', 'pantalla completa', '<path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/>'],
    ['minimize', 'Minimizar', 'reducir ventana', '<path d="M9 4v5H4M15 4v5h5M9 20v-5H4M15 20v-5h5"/>'],
    ['external-link', 'Abrir fuera', 'enlace externo ventana', '<path d="M14 4h6v6M20 4L10 14"/><path d="M18 14v5a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V7a1 1 0 0 1 1-1h5"/>'],
    ['window', 'Ventana', 'app programa', '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M3 9h18M6.5 6.5h.01M9.5 6.5h.01"/>'],
  ],

  dev: [
    ['code', 'Código', 'programar corchetes', '<path d="M9 18l-6-6 6-6M15 6l6 6-6 6"/>'],
    ['braces', 'Llaves', 'json bloque', '<path d="M9 3c-2 0-3 1-3 3v2c0 2-1 3-2 3 1 0 2 1 2 3v2c0 2 1 3 3 3"/><path d="M15 3c2 0 3 1 3 3v2c0 2 1 3 2 3-1 0-2 1-2 3v2c0 2-1 3-3 3"/>'],
    ['terminal', 'Terminal', 'consola shell cmd', '<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 9l3 3-3 3M13 15h5"/>'],
    ['bug', 'Depurar', 'error bicho debug', '<rect x="8" y="7" width="8" height="12" rx="4"/><path d="M8 11H4M8 15.5H3.5M16 11h4M16 15.5h4.5M9.5 7.5L8 5M14.5 7.5L16 5M12 19v3"/>'],
    ['git-branch', 'Rama', 'git branch', '<circle cx="7" cy="6" r="2.5"/><circle cx="7" cy="18" r="2.5"/><circle cx="17" cy="9" r="2.5"/><path d="M7 8.5v7M17 11.5c0 3.5-3.5 4-7 4.5"/>'],
    ['git-commit', 'Commit', 'git guardar', '<circle cx="12" cy="12" r="3.2"/><path d="M2 12h6.8M15.2 12H22"/>'],
    ['git-merge', 'Fusionar', 'git merge', '<circle cx="7" cy="6" r="2.5"/><circle cx="7" cy="18" r="2.5"/><circle cx="17" cy="13" r="2.5"/><path d="M7 8.5v7M14.5 13c-5 0-7.5-1.5-7.5-4.5"/>'],
    ['package', 'Paquete', 'npm modulo caja', '<path d="M21 8l-9-5-9 5v8l9 5 9-5z"/><path d="M3 8l9 5 9-5M12 13v8"/>'],
    ['box', 'Caja', 'contenedor almacen', '<rect x="3" y="7" width="18" height="13" rx="2"/><path d="M3 11h18M8 7V4h8v3"/>'],
    ['robot', 'Robot', 'bot automatizar ia', `<rect x="4" y="8" width="16" height="12" rx="3"/><circle cx="9" cy="14" r="1.6" ${F}/><circle cx="15" cy="14" r="1.6" ${F}/><path d="M12 4v4M9 3.5h6"/>`],
    ['activity', 'Actividad', 'pulso monitor rendimiento', '<path d="M3 12h4l3 8 4-16 3 8h4"/>'],
    ['chart-bar', 'Gráfico de barras', 'estadisticas datos', '<path d="M3 21h18"/><path d="M6 21V11M12 21V4M18 21v-6"/>'],
    ['chart-line', 'Gráfico de líneas', 'tendencia datos', '<path d="M3 3v18h18"/><path d="M6.5 15l4-4.5 3 3 6.5-7.5"/>'],
    ['chart-pie', 'Gráfico circular', 'tarta porcentaje', '<path d="M12 3a9 9 0 1 0 9 9h-9z"/><path d="M14.5 2.3A9 9 0 0 1 21.7 9.5H14.5z"/>'],
  ],

  comm: [
    ['mail', 'Correo', 'email sobre mensaje', '<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M3 7l9 6 9-6"/>'],
    ['mail-open', 'Correo abierto', 'email leido', '<path d="M3 10l9-6 9 6v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><path d="M3 10l9 6 9-6"/>'],
    ['send', 'Enviar', 'mandar avion mensaje', '<path d="M21 3L10.5 13.5M21 3l-7 18-3.5-7.5L3 10z"/>'],
    ['chat', 'Chat', 'mensaje conversacion', '<path d="M21 15a2 2 0 0 1-2 2H8l-5 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"/>'],
    ['chat-dots', 'Mensaje', 'chat escribiendo', `<path d="M21 15a2 2 0 0 1-2 2H8l-5 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"/><circle cx="8" cy="10" r="1.2" ${F}/><circle cx="12" cy="10" r="1.2" ${F}/><circle cx="16" cy="10" r="1.2" ${F}/>`],
    ['phone', 'Teléfono', 'llamar llamada', '<path d="M6 3h3l2 5-2.5 1.5a12 12 0 0 0 6 6L16 13l5 2v3a2 2 0 0 1-2.2 2A17 17 0 0 1 4 5.2 2 2 0 0 1 6 3z"/>'],
    ['phone-off', 'Colgar', 'llamada cortar', '<path d="M6 3h3l2 5-2.5 1.5a12 12 0 0 0 6 6L16 13l5 2v3a2 2 0 0 1-2.2 2A17 17 0 0 1 4 5.2 2 2 0 0 1 6 3z"/><path d="M3 3l18 18"/>'],
    ['at', 'Arroba', 'email usuario', '<circle cx="12" cy="12" r="4"/><path d="M16 8v5.5a2.5 2.5 0 0 0 5 0V12A9 9 0 1 0 17.4 19.2"/>'],
    ['share', 'Compartir', 'enviar nodos', '<circle cx="18" cy="5" r="2.5"/><circle cx="6" cy="12" r="2.5"/><circle cx="18" cy="19" r="2.5"/><path d="M8.2 10.8l7.6-4.4M8.2 13.2l7.6 4.4"/>'],
    ['megaphone', 'Megáfono', 'anuncio aviso', '<path d="M3 10.5v3l11 5V5.5z"/><path d="M14 9a3 3 0 0 1 0 6"/><path d="M6 15v4.5h3.5V16.5"/>'],
    ['rss', 'RSS', 'feed suscripcion', `<circle cx="6" cy="18" r="1.9" ${F}/><path d="M4 11a9 9 0 0 1 9 9M4 4a16 16 0 0 1 16 16"/>`],
  ],

  misc: [
    ['flag', 'Bandera', 'marcar objetivo', '<path d="M5 21V4M5 5h13l-2.5 4.5L18 14H5"/>'],
    ['pin', 'Marcador', 'ubicacion mapa chincheta', '<path d="M12 21.5S19 14.5 19 10a7 7 0 1 0-14 0c0 4.5 7 11.5 7 11.5z"/><circle cx="12" cy="10" r="2.5"/>'],
    ['map', 'Mapa', 'ubicacion ruta', '<path d="M9 4L3 6.5v13L9 17l6 2.5 6-2.5v-13L15 6.5z"/><path d="M9 4v13M15 6.5v13"/>'],
    ['globe', 'Mundo', 'internet web idioma', '<circle cx="12" cy="12" r="9"/><path d="M3 12h18"/><path d="M12 3a14 14 0 0 1 0 18 14 14 0 0 1 0-18z"/>'],
    ['compass', 'Brújula', 'norte direccion', '<circle cx="12" cy="12" r="9"/><path d="M15.5 8.5l-2 5-5 2 2-5z"/>'],
    ['sun', 'Sol', 'brillo claro dia', '<circle cx="12" cy="12" r="4"/><path d="M12 2v2.5M12 19.5V22M4.2 4.2l1.8 1.8M18 18l1.8 1.8M2 12h2.5M19.5 12H22M4.2 19.8L6 18M18 6l1.8-1.8"/>'],
    ['moon', 'Luna', 'noche oscuro dormir', '<path d="M21 13a8.5 8.5 0 0 1-10-10 8.5 8.5 0 1 0 10 10z"/>'],
    ['flame', 'Fuego', 'llama caliente racha', '<path d="M12 2.5s5.5 4.8 5.5 9.5a5.5 5.5 0 0 1-11 0c0-2.2 1-3.9 2.2-5 0 2.2 1 3.2 2 3.2 1.6 0 1.1-4.3-1.2-8z"/>'],
    ['drop', 'Gota', 'agua humedad', '<path d="M12 3.5s6 6.4 6 10.5a6 6 0 0 1-12 0C6 9.9 12 3.5 12 3.5z"/>'],
    ['leaf', 'Hoja', 'planta eco natural', '<path d="M4 20c0-9 6-14 16-14 0 9-5 14-12 14H4z"/><path d="M9 15c2-3 5-5 8-6"/>'],
    ['bolt', 'Rayo', 'energia rapido', '<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>'],
    ['rocket', 'Cohete', 'lanzar rapido inicio', '<path d="M12 2c3.5 2.6 5.5 6.6 5.5 11l-2.5 3h-6l-2.5-3C6.5 8.6 8.5 4.6 12 2z"/><circle cx="12" cy="9.5" r="2"/><path d="M9 19l-2.5 3M15 19l2.5 3"/>'],
    ['gift', 'Regalo', 'premio caja', '<rect x="3" y="8" width="18" height="4" rx="1"/><path d="M5 12v9h14v-9M12 8v13"/><path d="M12 8S10.5 4 8.5 4a2 2 0 0 0 0 4zM12 8s1.5-4 3.5-4a2 2 0 0 1 0 4z"/>'],
    ['cart', 'Carrito', 'compra tienda', `<circle cx="9.5" cy="20" r="1.7" ${F}/><circle cx="18" cy="20" r="1.7" ${F}/><path d="M2 3h3l2.5 12h12L22 6.5H6"/>`],
    ['tag', 'Etiqueta', 'precio marcar', `<path d="M11 3H4a1 1 0 0 0-1 1v7l10 10 8-8z"/><circle cx="7.5" cy="7.5" r="1.5" ${F}/>`],
    ['wallet', 'Cartera', 'dinero pagos', `<rect x="3" y="6" width="18" height="14" rx="2"/><path d="M3 10h18"/><circle cx="17" cy="15" r="1.4" ${F}/>`],
    ['credit-card', 'Tarjeta', 'pago banco', '<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M2 10h20M6 15h4"/>'],
    ['coffee', 'Café', 'descanso taza', '<path d="M3 8h14v6a5 5 0 0 1-10 0z"/><path d="M17 9h2a2.5 2.5 0 0 1 0 5h-2"/><path d="M3 21h16"/>'],
    ['gamepad', 'Mando', 'juego consola', `<rect x="2" y="7" width="20" height="11" rx="4"/><path d="M7 10.5v4M5 12.5h4"/><circle cx="16" cy="11.5" r="1.3" ${F}/><circle cx="18.5" cy="14" r="1.3" ${F}/>`],
    ['dice', 'Dado', 'azar juego', `<rect x="3" y="3" width="18" height="18" rx="3"/><circle cx="8" cy="8" r="1.5" ${F}/><circle cx="12" cy="12" r="1.5" ${F}/><circle cx="16" cy="16" r="1.5" ${F}/>`],
    ['trophy', 'Trofeo', 'premio ganar', '<path d="M8 3h8v6a4 4 0 0 1-8 0z"/><path d="M8 5H5v1.5A3.5 3.5 0 0 0 8.5 10M16 5h3v1.5A3.5 3.5 0 0 1 15.5 10"/><path d="M12 13v4M8.5 21h7l-1-4h-5z"/>'],
    ['target', 'Objetivo', 'diana precision', `<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="5"/><circle cx="12" cy="12" r="1.6" ${F}/>`],
    ['timer', 'Temporizador', 'cronometro tiempo', '<circle cx="12" cy="13.5" r="7.5"/><path d="M12 9.5v4M9.5 2h5"/>'],
    ['hourglass', 'Reloj de arena', 'espera tiempo', '<path d="M6 2h12M6 22h12"/><path d="M8 2v4l4 4 4-4V2M8 22v-4l4-4 4 4v4"/>'],
    ['thermometer', 'Temperatura', 'calor frio', '<path d="M14 14.2V5a2 2 0 0 0-4 0v9.2a4 4 0 1 0 4 0z"/>'],
    ['wrench', 'Llave inglesa', 'herramienta arreglar', '<path d="M20 5.5A5.5 5.5 0 0 1 12.6 13L5 20.6 3.4 19 11 11.4A5.5 5.5 0 0 1 18.5 4L15 7.5l1.5 1.5L20 5.5z"/>'],
    ['hammer', 'Martillo', 'herramienta construir', '<path d="M14 3l7 7-3 3-7-7z"/><path d="M11.5 8.5L3 17v4h4l8.5-8.5"/>'],
    ['toolbox', 'Herramientas', 'caja utiles', '<rect x="2" y="8" width="20" height="12" rx="2"/><path d="M8 8V5h8v3M2 13h20"/>'],
    ['lightbulb', 'Idea', 'bombilla consejo', '<path d="M9 18.5h6M10 21.5h4"/><path d="M12 2.5a6 6 0 0 0-3.5 10.9v2.1h7v-2.1A6 6 0 0 0 12 2.5z"/>'],
    ['magnet', 'Imán', 'atraer', '<path d="M6 3H3v9a9 9 0 0 0 18 0V3h-3v9a6 6 0 0 1-12 0z"/><path d="M3 8h3M18 8h3"/>'],
    ['anchor', 'Ancla', 'fijar barco', '<circle cx="12" cy="5" r="2.5"/><path d="M12 7.5V21"/><path d="M5 12H3a9 9 0 0 0 18 0h-2"/><path d="M8 11h8"/>'],
    ['plane', 'Avión', 'viaje vuelo', '<path d="M10 3.5a2 2 0 0 1 4 0V9l8 4.5v2.5l-8-2.5v4l2.5 2v2L12 20l-4.5 1.5v-2l2.5-2v-4L2 16v-2.5L10 9z"/>'],
    ['car', 'Coche', 'vehiculo transporte', `<path d="M3 16v-3.5L5 7h14l2 5.5V16z"/><path d="M4.5 16v3H7v-3M17 16v3h2.5v-3"/><circle cx="7.5" cy="12.5" r="1.3" ${F}/><circle cx="16.5" cy="12.5" r="1.3" ${F}/>`],
    ['book', 'Libro', 'leer manual', '<path d="M4 4.5A2.5 2.5 0 0 1 6.5 2H20v16H6.5A2.5 2.5 0 0 0 4 20.5z"/><path d="M4 20.5A2.5 2.5 0 0 1 6.5 18H20v4H6.5A2.5 2.5 0 0 1 4 20.5z"/>'],
    ['bookmark', 'Marcador', 'guardar favorito', '<path d="M6 3h12v18l-6-4.5L6 21z"/>'],
    ['graduation', 'Formación', 'estudios birrete', '<path d="M2 8l10-4 10 4-10 4z"/><path d="M6 10.5V16c0 1.7 2.7 3 6 3s6-1.3 6-3v-5.5"/>'],
    ['thumbs-up', 'Me gusta', 'aprobar bien', '<path d="M7 21V10l5-8a2.5 2.5 0 0 1 2.4 3.2L13.5 9H20a2 2 0 0 1 2 2.4l-1.6 7.2A2 2 0 0 1 18.4 21z"/><rect x="2" y="10" width="5" height="11" rx="1"/>'],
    ['smiley', 'Sonrisa', 'cara emoji feliz', `<circle cx="12" cy="12" r="9"/><circle cx="9" cy="10" r="1.3" ${F}/><circle cx="15" cy="10" r="1.3" ${F}/><path d="M7.8 14.3a5 5 0 0 0 8.4 0"/>`],
    ['skull', 'Calavera', 'peligro muerte', `<path d="M5 11.5a7 7 0 1 1 14 0V14l-2 1.5V19h-3v-2h-4v2H7v-3.5L5 14z"/><circle cx="9.3" cy="11.5" r="1.8" ${F}/><circle cx="14.7" cy="11.5" r="1.8" ${F}/>`],
    ['crown', 'Corona', 'rey premium', '<path d="M3 8l4.5 4L12 4.5 16.5 12 21 8v11H3z"/>'],
  ],
};

// Índice plano: [{ id, name, cat, keywords, body }]
export const ICONS = Object.entries(RAW).flatMap(([cat, list]) =>
  list.map(([id, name, keywords, body]) => ({ id, name, cat, keywords, body })));

// --- Búsqueda ---------------------------------------------------------------

// Sin acentos y en minúsculas: "camara" tiene que encontrar "Cámara".
function fold(s) {
  return String(s).toLowerCase().normalize('NFD').replace(/[̀-ͯ]/g, '');
}

export function search(query = '', cat = 'all') {
  const terms = fold(query).split(/\s+/).filter(Boolean);
  return ICONS.filter((it) => {
    if (cat !== 'all' && it.cat !== cat) return false;
    if (!terms.length) return true;
    const hay = fold(`${it.id} ${it.name} ${it.keywords}`);
    return terms.every((t) => hay.includes(t));
  });
}

// --- Salida SVG -------------------------------------------------------------

// `color` es lo que resuelve currentColor. En el panel se deja heredar del tema;
// para rasterizar se fuerza blanco, que es lo que el umbral traduce a píxel
// encendido (ver canvasToFramebuffer).
export function svgMarkup(item, { size = 24, color = null, strokeWidth = 2 } = {}) {
  const style = color ? ` style="color:${color}"` : '';
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" viewBox="0 0 24 24"${style}
    fill="none" stroke="currentColor" stroke-width="${strokeWidth}"
    stroke-linecap="round" stroke-linejoin="round">${item.body}</svg>`;
}

export function byId(id) {
  return ICONS.find((it) => it.id === id) || null;
}
