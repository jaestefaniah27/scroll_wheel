// La configuración local, que en la app de escritorio vive en un JSON de
// %APPDATA%, aquí vive en IndexedDB del navegador.
//
// No vale localStorage: el espejo del teclado lleva los iconos OLED en hexadecimal
// (360 bytes por pantalla, veinte por página) y con unos cuantos perfiles se pasa
// del límite de 5 MB. IndexedDB no tiene ese techo.
//
// Se guarda un único registro con todo el objeto, no una clave por ajuste: quien
// llama pide y escribe la configuración entera (mirror.js vuelca el espejo completo
// en cada guardado), así que partirlo solo añadiría transacciones.

import { DEFAULTS, merge } from './config-merge.mjs';

const DB_NAME = 'orby-webgui';
const DB_VERSION = 1;
const STORE = 'config';
const KEY = 'local';

let db = null;
let cache = null;
let saveTimer = null;

function openDb() {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION);
    req.onupgradeneeded = () => req.result.createObjectStore(STORE);
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
}

function read() {
  return new Promise((resolve) => {
    const req = db.transaction(STORE, 'readonly').objectStore(STORE).get(KEY);
    req.onsuccess = () => resolve(req.result ?? null);
    req.onerror = () => resolve(null);
  });
}

function write(value) {
  return new Promise((resolve) => {
    const tx = db.transaction(STORE, 'readwrite');
    tx.objectStore(STORE).put(value, KEY);
    tx.oncomplete = () => resolve(true);
    tx.onerror = () => resolve(false);
  });
}

// Hay que llamarlo antes de montar window.orby: getConfig no puede devolver una
// promesa que espere a la base sin más, porque mirror.js la llama en el arranque y
// una configuración vacía haría creer a la app que no hay copia local.
export async function initConfig() {
  try {
    db = await openDb();
    cache = merge(DEFAULTS, (await read()) || {});
  } catch {
    // Navegación privada, IndexedDB deshabilitada por política del navegador… La
    // app funciona igual, pero cada recarga empieza de cero: mejor eso que no
    // arrancar.
    console.warn('[web] sin IndexedDB: la configuración no sobrevivirá a la recarga');
    db = null;
    cache = structuredClone(DEFAULTS);
  }
}

export async function getConfig() {
  return cache;
}

// Escribe agrupando: mirror.js guarda el espejo entero después de cada cambio y
// durante la precarga de iconos eso son decenas de guardados seguidos con el mismo
// objeto grande dentro.
const SAVE_DELAY_MS = 300;

export async function setConfig(patch) {
  cache = merge(cache, patch);
  if (!db) return cache;

  if (saveTimer) clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    saveTimer = null;
    write(cache).catch(() => {});
  }, SAVE_DELAY_MS);

  return cache;
}
