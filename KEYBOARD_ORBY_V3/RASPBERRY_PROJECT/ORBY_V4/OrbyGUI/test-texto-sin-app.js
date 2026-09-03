// Prueba el protocolo de texto de la Fase 1 (docs/PLAN_TEXTO_SIN_APP.md) hablando
// directo por CDC, sin pasar por la app: hoy OrbyGUI todavía no sabe mandar
// SET_TEXT/TEXT_END/MSTEP_TEXT, eso es la Fase 2. Mismo patrón que test-serial.js.
//
// Qué hace: sube dos textos, los cuelga de las dos primeras teclas del perfil 0
// (perfil e id de macro/hueco de texto fijos a propósito, es una prueba de un
// solo uso) y cierra el puerto para dejarlo libre. La prueba de verdad es
// física: pulsar esas teclas con esto YA CERRADO (o con OrbyGUI cerrada del
// todo, "Salir" desde la bandeja) y comprobar qué escribe el teclado por su
// cuenta.
//
// Uso, desde OrbyGUI/ (para que 'serialport' resuelva de node_modules):
//   node test-texto-sin-app.js COM25

const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const path = process.argv[2];
if (!path) {
  console.error('Uso: node test-texto-sin-app.js <COM_PORT>  (ej. COM25)');
  process.exit(1);
}

const port = new SerialPort({ path, baudRate: 115200, autoOpen: false });
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

// No correlaciona petición/respuesta de verdad (eso lo hace device.js en la
// app): basta con ver la traza en orden para esta comprobación manual.
const pending = [];
parser.on('data', (line) => {
  console.log('  <', line);
  if (pending.length) pending.shift()();
});

function send(cmd) {
  return new Promise((resolve) => {
    console.log('>', cmd);
    const timer = setTimeout(resolve, 800);
    pending.push(() => { clearTimeout(timer); resolve(); });
    port.write(cmd + '\n');
  });
}

// SET_TEXT va en hex, a trozos de 90 bytes (180 caracteres hex) por línea,
// igual que trocea OLED_CHUNK la app de verdad.
function toHexChunks(text) {
  const bytes = Buffer.from(text, 'utf8');
  const hex = bytes.toString('hex');
  const chunks = [];
  for (let i = 0; i < hex.length; i += 180) chunks.push(hex.slice(i, i + 180));
  return { len: bytes.length, chunks };
}

async function uploadText(slot, text) {
  const { len, chunks } = toHexChunks(text);
  let offset = 0;
  for (const chunk of chunks) {
    await send(`SET_TEXT:${slot}:${offset}:${chunk}`);
    offset += chunk.length / 2;
  }
  await send(`TEXT_END:${slot}:${len}`);
}

// Usa el mismo número para el id de macro y el hueco de texto: son espacios
// de numeración independientes, pero para esta prueba de un solo uso no hace
// falta repartirlos por separado.
async function wireKey(profileIdx, keySlot, id, text) {
  await uploadText(id, text);
  await send(`SET_MACRO_STEP:${id}:0:6:${id}:0:1:0`); // tipo 6 = MSTEP_TEXT, a = hueco de texto
  await send(`MACRO_TRUNC:${id}:1`);
  await send(`SET_KEYMAP:${profileIdx}:${keySlot}:251:${id}`); // 251 = KEYACT_MACRO
}

(async () => {
  await new Promise((resolve, reject) => port.open((err) => (err ? reject(err) : resolve())));
  port.set({ dtr: true, rts: true });
  await new Promise((r) => setTimeout(r, 300));

  console.log('\n== 1. Handshake: debe salir TEXT=1:MAXTEXT=510:LAYOUT=es ==');
  await send('ACK');

  console.log('\n== 2. Distribución explícita a ES ==');
  await send('SET_LAYOUT:es');

  console.log('\n== 3. Texto corto en la 1a tecla del perfil 0 ==');
  await wireKey(0, 0, 0, 'café ñoño @2026 ¿qué tal?');

  console.log('\n== 4. Texto de 400 caracteres en la 2a tecla — prueba de la cola ==');
  const largo = 'Lorem ipsum dolor sit amet, áéíóúñÑ¿¡ '.repeat(11).slice(0, 400);
  await wireKey(0, 1, 1, largo);

  console.log('\n== 5. Guardar en Flash (sobrevive a un reinicio) ==');
  await send('SAVE_STATE');

  console.log('\nListo. Cierro el puerto para dejarlo libre.');
  console.log('Con esto cerrado (y OrbyGUI de escritorio TAMBIÉN cerrada del todo, "Salir"');
  console.log('desde la bandeja, no la X), en el perfil por defecto (perfil 0):');
  console.log('  - No se sabe a ciegas qué posición física son los huecos 0 y 1 del mapa de');
  console.log('    teclas: prueba las 12 teclas una a una. Dos de ellas ya no harán lo de');
  console.log('    siempre — esas son las tocadas por este script.');
  console.log('  - Una debería escribir: "café ñoño @2026 ¿qué tal?"');
  console.log('  - La otra, el texto largo de 400 caracteres, ENTERO y sin faltar letras.');
  port.close();
  process.exit(0);
})().catch((e) => {
  console.error(e);
  process.exit(1);
});
