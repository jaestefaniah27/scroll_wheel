// Arranque de la vía Tauri.
//
// Aquí no hay preload que monte `window.orby` antes de tiempo: lo monta este módulo, y
// tiene que estar puesto antes de que main.js importe nada de src/, porque varios de
// esos módulos tocan `window.orby` al evaluarse. Por eso main.js entra por importación
// dinámica y no por `import` normal, igual que en la vía del navegador.

import { setPlatform } from './platform.js';
import { instalarOrbyTauri } from './tauri/orby-tauri.js';

setPlatform('tauri');
await instalarOrbyTauri();

await import('./main.js');
