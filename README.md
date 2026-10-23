# ORVY · scroll_wheel

Teclado macro con rueda de scroll de alta resolución, una pantalla OLED por tecla y
una app para configurarlo todo. Proyecto de Jorge Alejandro Estefanía Hidalgo y
Javier Píriz González.

## Enlaces

| | |
|---|---|
| **Web del proyecto** | **https://orvy.duckdns.org** · lista de espera y presentación |
| **Configurar desde el navegador** | **https://jaestefaniah27.github.io/scroll_wheel/app/** · sin instalar nada (Chrome/Edge de escritorio) |
| Landing técnica | https://jaestefaniah27.github.io/scroll_wheel/ |
| Descargas (app y firmware) | https://github.com/jaestefaniah27/scroll_wheel/releases |

## Qué es

- **12 teclas** con **10 pantallas OLED** (un icono por tecla, cambian con el perfil).
- **Rueda** con sensor magnético AS5600: scroll de alta resolución por HID, con
  detents y giro libre.
- **2 encoders EC11** para volumen, brillo, zoom o lo que se asigne.
- **Raspberry Pi Pico (RP2040)**. La configuración vive en su Flash: el teclado
  funciona solo, sin la app abierta.

## La app: OrbyGUI

Configura perfiles, páginas, atajos, mandos, rueda, iconos OLED y secuencias.

- **Windows** (Tauri): instalador en
  [Releases](https://github.com/jaestefaniah27/scroll_wheel/releases/latest). Se
  actualiza sola. Además de configurar, ejecuta lo que necesita el PC: abrir apps,
  escribir texto, grabar ratón y teclado, complementos y perfiles que cambian según
  la ventana activa.
- **Navegador** ([WebGUI](https://jaestefaniah27.github.io/scroll_wheel/app/)): la
  misma app por Web Serial, para equipos donde no se puede instalar nada. Hace todo
  lo que se guarda en el teclado. No hace lo que necesita el PC (lista de arriba).

  1. Abre el enlace en **Chrome o Edge de escritorio** (Firefox, Safari y móvil no
     tienen Web Serial).
  2. Pulsa **Conectar teclado** y elige **Orby V4 Control**.
  3. Si no aparece: cierra la app de escritorio, que solo un programa puede usar el
     puerto a la vez. En un Chrome de empresa, la política
     `DefaultSerialGuardSetting` puede bloquear Web Serial.

Más detalle en [WEBGUI.md](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/docs/WEBGUI.md).

## Versiones actuales

| | Versión | Etiqueta |
|---|---|---|
| App OrbyGUI | 1.0.4 | `v1.0.4` |
| Firmware | 4.6 | `fw-v4.6` |

Qué versión de app funciona con qué firmware:
[COMPATIBILIDAD.md](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/docs/COMPATIBILIDAD.md).

## Estructura del repositorio

| Ruta | Qué hay |
|---|---|
| [`KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/`](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/) | Firmware (C++17, Pico SDK 2.2.0 + TinyUSB): `main.cpp`, `include/`, `src/` |
| [`…/ORBY_V4/OrbyGUI/`](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/) | App: frontend JS vanilla + Vite, backend Rust (Tauri) en `src-tauri/` |
| [`…/ORBY_V4/web/`](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/web/) | Landing técnica de GitHub Pages |
| [`KEYBOARD_ORBY_V3/3D/`](KEYBOARD_ORBY_V3/3D/) · [`3d/`](3d/) | Piezas de la carcasa (`.3mf`) y modelo completo (`.step`) |

La guía para trabajar en el código (comandos, arquitectura y protocolo serie) está en
[ORBY_V4/CLAUDE.md](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/CLAUDE.md).

## Compilar

**Firmware** (desde `ORBY_V4/`):

```powershell
.\flash.ps1                                        # compila y flashea, sin pulsar BOOTSEL
cmake -B build -G Ninja && cmake --build build     # solo compilar → build/ORBY_V4.uf2
```

**App** (desde `ORBY_V4/OrbyGUI/`):

```bash
npm install
npm run tauri:dev     # app de escritorio con recarga
npm run preview:web   # la versión de navegador en local
npm test              # tests del frontend
cd src-tauri && cargo test -p orby-core   # tests del backend
```

Antes de publicar una versión de la app, lee
[PUBLICACION.md](KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/docs/PUBLICACION.md):
sin la firma, el actualizador de las apps ya instaladas deja de funcionar.

## Publicación automática

- Push a `main` que toque la app o la landing → GitHub Pages publica la landing y la
  WebGUI (`.github/workflows/pages.yml`).
- Etiqueta `fw-vX.Y` → compila y publica el firmware (`.github/workflows/firmware.yml`).
