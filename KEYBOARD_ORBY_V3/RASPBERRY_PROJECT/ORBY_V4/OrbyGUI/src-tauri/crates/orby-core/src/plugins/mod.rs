// Complementos.
//
// Se reescribieron de cero al dejar Electron: antes eran módulos de Node que corrían
// dentro del proceso principal y sin caja de arena, y Tauri no lleva Node con el que
// ejecutarlos. Hay dos tipos, con el mismo manifiesto y el mismo descriptor hacia el
// renderer:
//
// - `http`: sin código. El manifiesto declara qué petición mandar por cada acción.
// - `process`: un programa externo que habla un protocolo de líneas JSON. Es el que hace
//   posibles los complementos que necesitan una conexión persistente o acceso nativo
//   (OBS, monitor de hardware, audio, Chrome, Altium).
//
// Aquí solo vive lo que no depende de la plataforma. Mandar las peticiones y lanzar los
// procesos es cosa de la cáscara.

pub mod coalesce;
pub mod manifiesto;
pub mod plantilla;

pub use coalesce::{Agrupador, Envio};
pub use manifiesto::{Manifiesto, Tipo, API_VERSION};
pub use plantilla::{render, Contexto};
