// Huella de la configuración, calculada sobre lo que la app tiene en memoria.
//
// El teclado calcula la suya con GET_HASH (ver include/config_hash.h). Si las
// dos coinciden, la copia local del PC (mirror.js) es exactamente lo que hay en
// el Orby y no hace falta descargar nada al conectar; si no, los CRC por perfil
// dicen cuáles hay que releer.
//
// LA SERIALIZACIÓN TIENE QUE SER BYTE A BYTE LA MISMA QUE LA DEL FIRMWARE.
// Cualquier cambio en el orden o en el formato de los campos hay que hacerlo en
// los dos sitios; si no, las huellas dejan de cuadrar y la app vuelve a
// releerlo todo en cada conexión (molesto, pero no rompe nada: ante la duda se
// descarga, que es lo que hacía antes de existir esto).
// tools/test/test_config_hash.cpp compara las dos implementaciones.
//
// Este módulo no importa nada a propósito: los datos y los iconos entran por
// parámetro. Así la prueba puede cargarlo en Node, donde no hay ni `window` ni
// el resto de la app.

const CRC32_INIT = 0xffffffff;

// La misma tabla de 16 entradas que el firmware.
const CRC32_NIBBLE = new Uint32Array([
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
  0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
  0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
]);

const TEXT_ENCODER = new TextEncoder();

// Un acumulador en vez de una función sobre un búfer entero: los bitmaps son
// 360 bytes cada uno y montar el mensaje completo en memoria para hashearlo de
// una vez serían varios megas por nada.
class Crc32 {
  constructor() {
    this.value = CRC32_INIT;
  }

  bytes(data) {
    let crc = this.value;
    for (let i = 0; i < data.length; i++) {
      crc = (crc ^ data[i]) >>> 0;
      crc = ((crc >>> 4) ^ CRC32_NIBBLE[crc & 0x0f]) >>> 0;
      crc = ((crc >>> 4) ^ CRC32_NIBBLE[crc & 0x0f]) >>> 0;
    }
    this.value = crc;
    return this;
  }

  u8(...values) {
    return this.bytes(Uint8Array.from(values, (v) => v & 0xff));
  }

  u32le(v) {
    return this.u8(v, v >>> 8, v >>> 16, v >>> 24);
  }

  // Los char[8] del firmware: el texto hasta el terminador y ceros hasta ocho
  // bytes. Se corta por BYTES, no por caracteres, porque es lo que cuenta el
  // firmware.
  text8(str) {
    const buf = new Uint8Array(8);
    buf.set(TEXT_ENCODER.encode(String(str ?? '')).subarray(0, 8));
    return this.bytes(buf);
  }

  done() {
    return ((this.value ^ 0xffffffff) >>> 0);
  }
}

export function toHex(crc) {
  return (crc >>> 0).toString(16).padStart(8, '0');
}

// Huella de un perfil. `iconOf(slot, page)` devuelve los 360 bytes del bitmap
// de ese hueco, o null si no está descargado.
//
// Devuelve null si falta algún icono por leer: sin él no se puede calcular, y
// devolver un número a medias haría creer que el perfil ha cambiado (o peor,
// que no) sin fundamento.
export function profileHash(prof, iconOf) {
  if (!prof || !prof.pages?.length) return null;

  const pages = Math.max(1, Math.min(prof.pageCount || 1, prof.pages.length));
  const crc = new Crc32();
  crc.text8(prof.name);
  crc.u8(pages);

  for (let pg = 0; pg < pages; pg++) {
    const page = prof.pages[pg];
    if (!page) return null;

    for (let s = 0; s < 20; s++) crc.text8(page.labels?.[s] ?? '');
    for (let s = 0; s < 24; s++) {
      const k = page.keys?.[s] || { modifier: 0, keycode: 0 };
      crc.u8(k.modifier || 0, k.keycode || 0);
    }
    for (let s = 0; s < 16; s++) {
      const r = page.rotary?.[s] || { type: 0, modifier: 0, keycode: 0 };
      crc.u8(r.type || 0, r.modifier || 0, r.keycode || 0);
    }
    for (let l = 0; l < 2; l++) {
      const sc = page.scroll?.[l] || { detentsPerRev: 60, invert: false };
      crc.u8(sc.detentsPerRev || 0, sc.invert ? 1 : 0);
    }

    const mask = (page.oledMask || 0) & 0xfffff;
    crc.u32le(mask);

    for (let s = 0; s < 20; s++) {
      if (!(mask & (1 << s))) continue;
      const bytes = iconOf(s, pg);
      if (!bytes || bytes.length !== 360) return null;  // aún sin descargar
      crc.bytes(bytes);
    }
  }

  return crc.done();
}

// Huella de todo: número de perfiles, tiempo de reposo y las huellas de cada
// perfil encadenadas. No entran ni el perfil ni la página activos, que cambian
// solo con usar el teclado y no son configuración.
//
// `iconOf(profileIdx, slot, page)` es el acceso a los bitmaps (en la app, el de
// oled-cache). `all` sale null si a algún perfil le faltaban iconos: no se
// puede afirmar nada del conjunto sin ellos.
export function snapshotHash(profiles, timeout, iconOf) {
  const per = [];
  const crc = new Crc32();
  crc.u8(profiles.length, timeout);

  for (const prof of profiles) {
    const h = profileHash(prof, (slot, page) => iconOf(prof.idx, slot, page));
    per.push(h);
    if (h === null) return { all: null, per };
    crc.u32le(h);
  }

  return { all: crc.done(), per };
}
