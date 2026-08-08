// Tablas fijas del editor de perfiles: cómo se agrupan los mandos, qué tipos se
// ofrecen y las constantes de tiempo y tamaño. Solo datos, sin estado.

import { ROTARY_SLOTS, ROTARY_TYPES } from '../../hid-keys.js';

// Mandos giratorios agrupados como se ven en el teclado.
export const ROTARY_GROUPS = [
  {
    name: 'Izquierdo', icon: 'reset',
    parts: [
      { slot: ROTARY_SLOTS.ENC1_CW,    label: 'Giro horario',     short: '↻' },
      { slot: ROTARY_SLOTS.ENC1_CCW,   label: 'Giro antihorario', short: '↺' },
      { slot: ROTARY_SLOTS.ENC1_CLICK, label: 'Pulsación',        short: '⏺', discrete: true },
    ],
  },
  {
    name: 'Derecho', icon: 'reset',
    parts: [
      { slot: ROTARY_SLOTS.ENC2_CW,    label: 'Giro horario',     short: '↻' },
      { slot: ROTARY_SLOTS.ENC2_CCW,   label: 'Giro antihorario', short: '↺' },
      { slot: ROTARY_SLOTS.ENC2_CLICK, label: 'Pulsación',        short: '⏺', discrete: true },
    ],
  },
];

// La rueda ya no comparte rejilla con los encoders (tiene su propia tarjeta,
// más abajo, con botones directos para "hacia abajo" y "hacia arriba"), pero
// el inspector necesita su nombre y su etiqueta cuando se abre desde ahí.
export const WHEEL_GROUP = {
  name: 'Rueda de scroll', icon: 'wheel',
  parts: [
    { slot: ROTARY_SLOTS.WHEEL_CW,  label: 'Hacia abajo',  short: '↓' },
    { slot: ROTARY_SLOTS.WHEEL_CCW, label: 'Hacia arriba', short: '↑' },
  ],
};

// Tipos ofrecidos en el inspector. Los de desplazamiento no tienen sentido en
// una pulsación, que es un evento suelto sin dirección.
export const ROTARY_TYPE_OPTIONS = [
  { type: ROTARY_TYPES.NONE,     label: 'Nada' },
  { type: ROTARY_TYPES.CONSUMER, label: 'Multimedia / sistema' },
  { type: ROTARY_TYPES.KEY,      label: 'Atajo de teclado' },
  { type: ROTARY_TYPES.SCROLL_V, label: 'Desplazar vertical', turnOnly: true },
  { type: ROTARY_TYPES.SCROLL_H, label: 'Desplazar horizontal', turnOnly: true },
  { type: ROTARY_TYPES.ZOOM,     label: 'Zoom (Ctrl + rueda)', turnOnly: true },
];

export const SCROLL_PRESETS = [
  { value: 12,  name: 'Preciso',  desc: 'Timeline, PCB, edición fina' },
  { value: 30,  name: 'Suave',    desc: 'Lectura y navegación lenta' },
  { value: 60,  name: 'Estándar', desc: 'Equivalente a rueda de ratón' },
  { value: 120, name: 'Rápido',   desc: 'Documentos y logs largos' },
];

// Tiempo de espera que se inserta automáticamente entre dos pasos de una
// secuencia. Bastante bajo para no notarse, pero suficiente para que el
// sistema/la app de destino registre el paso anterior (mover el ratón, un
// clic) antes del siguiente.
export const DEFAULT_STEP_DELAY_MS = 50;

// Bytes de un framebuffer de pantalla (72x40 monocromo).
export const OLED_FRAME_BYTES = 360;

// Lo mismo para un mando. El signo lo pone el hueco, así que con elegir "Brillo"
// en los dos sentidos el encoder ya queda montado sin escribir ningún número:
// en los encoders baja el giro antihorario, y en la rueda el "hacia abajo"
// (WHEEL_CW, ver WHEEL_GROUP), que es el sentido en el que se baja cualquier
// otra cosa. Si el encoder está montado del revés, el botón "Invertir giro" del
// inspector le da la vuelta al par entero (ver invertirGiroPlugin en macro-tabs.js).
export const ROTARY_DOWN_SLOTS = new Set([
  ROTARY_SLOTS.ENC1_CCW, ROTARY_SLOTS.ENC2_CCW, ROTARY_SLOTS.WHEEL_CW,
]);

// El otro sentido del mismo mando.
export const ROTARY_TWIN = {
  [ROTARY_SLOTS.ENC1_CW]:  ROTARY_SLOTS.ENC1_CCW,
  [ROTARY_SLOTS.ENC1_CCW]: ROTARY_SLOTS.ENC1_CW,
  [ROTARY_SLOTS.ENC2_CW]:  ROTARY_SLOTS.ENC2_CCW,
  [ROTARY_SLOTS.ENC2_CCW]: ROTARY_SLOTS.ENC2_CW,
  [ROTARY_SLOTS.WHEEL_CW]: ROTARY_SLOTS.WHEEL_CCW,
  [ROTARY_SLOTS.WHEEL_CCW]: ROTARY_SLOTS.WHEEL_CW,
};
