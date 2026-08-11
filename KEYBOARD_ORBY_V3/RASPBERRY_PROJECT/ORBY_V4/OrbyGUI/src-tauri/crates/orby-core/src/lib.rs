// La lógica de OrbyGUI que no depende de la plataforma.
//
// Aquí no entra nada de Tauri ni de Windows, y es a propósito: es lo que permite
// comprobar con `cargo test` en cualquier máquina lo que da miedo perder al migrar
// desde Electron. La cáscara que abre ventanas y llama a Win32 vive aparte.

pub mod config;
pub mod fw;
pub mod lines;
pub mod plugins;
pub mod protocolo;
pub mod releases;
pub mod serie;
