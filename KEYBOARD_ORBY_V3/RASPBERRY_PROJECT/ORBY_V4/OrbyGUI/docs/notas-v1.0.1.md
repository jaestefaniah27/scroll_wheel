# OrbyGUI 1.0.1

## Cambios

- **La ventana abre maximizada**, como hacía la versión de Electron. El port a
  Tauri se había quedado con el 1280x820 de `tauri.conf.json`.
- **Arreglado el aviso "Firmware 4.6: más nuevo que esta app"** que salía al
  conectar el teclado. El firmware 4.6 ya estaba publicado (`fw-v4.6`) y la app
  ya lo soportaba en el código, pero el bundle que se empaquetó en la
  `1.0.0-alpha` se compiló con la tabla de compatibilidad vieja (llegaba solo
  hasta 4.5). Reconstruido: ahora `FW_RECOMMENDED` es 4.6.

## Instalación

Windows mostrará el aviso de SmartScreen: el instalador todavía no va firmado
(ver `docs/PUBLICACION.md`). Para continuar: **Más información → Ejecutar de
todas formas**.

Antes de ejecutarlo, comprueba que el fichero es el que publicamos:

    Get-FileHash .\OrbyGUI_1.0.1_x64-setup.exe -Algorithm SHA256

    SHA256: <pegar aquí la salida de abajo>
