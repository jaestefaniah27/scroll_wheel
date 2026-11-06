---
name: publicar
description: Publica una versión de Orvy (app OrbyGUI y/o firmware del teclado) en jaestefaniah27/scroll_wheel, con firma para que la app se actualice sola. Úsala cuando pidan publicar, sacar una release, subir una versión, que la app se autoactualice, o subir el firmware a GitHub.
---

# Publicar una release de Orvy

Las releases viven en **jaestefaniah27/scroll_wheel** (no en el monorepo Orvy). Hay dos tipos y
no se mezclan:

| Tipo | Etiqueta | En GitHub | Assets |
|---|---|---|---|
| Firmware | `fw-vX.Y` | **siempre prerelease** | `ORBY_V4-fw-X.Y.uf2` + `.sha256` |
| App | `vX.Y.Z` | **la última, nunca prerelease** | `Orvy_X.Y.Z_x64-setup.exe` + `.exe.sig` + `latest.json` |

**Por qué importa:** las apps instaladas buscan `releases/latest/download/latest.json`.
- Si la última release no tiene `latest.json`, o su firma no es válida, el actualizador de todas
  las copias instaladas deja de funcionar y no avisa.
- Por eso el firmware va como prerelease (no cuenta como "latest") y la release de la app se
  publica **después** que la del firmware.

Más detalle en `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/docs/PUBLICACION.md` y
`.../ORBY_V4/docs/COMPATIBILIDAD.md`. Rutas abajo relativas a `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/` (`ORBY_V4/`).

## Lo que no hay que hacer nunca

- Publicar la app sin `.sig` ni `latest.json`, o marcar como prerelease la release de la app.
- Pedir, copiar o imprimir la clave de firma. Está en el servidor de Jorge y **no se comparte**:
  se firma con el conector ORVY (`sign_update`).
- Imprimir el token de GitHub. Se obtiene con `git credential fill` dentro del script.
- Marcar el firmware como release normal: se convertiría en "latest" y rompería el actualizador.

## 0. Comprobaciones previas

Usa la skill `comprobar`, que lo pasa todo de una vez:

```powershell
powershell -ExecutionPolicy Bypass -File D:\ORVY\scroll_wheel\.claude\skills\comprobar\comprobar.ps1
```

Tiene que acabar con todo en `[ OK ]` y sin `[behind N]` en git.

Para dejar un firmware probado en el teclado antes de publicarlo, usa la skill `teclado`.

## 1. Subir versiones (en un único commit con los cambios)

**App** (los cuatro, con el mismo número):
- `OrbyGUI/package.json` → `"version"`
- `OrbyGUI/src-tauri/tauri.conf.json` → `"version"`
- `OrbyGUI/src-tauri/Cargo.toml` → `version` de `[package]`
- `OrbyGUI/src-tauri/Cargo.lock` → el `version` del paquete `orby-app`

**Firmware** (solo si cambia `main.cpp` o `include/`):
- `include/orby_version.h` → `ORBY_FW_MINOR` (o `MAJOR`)
- `OrbyGUI/src/compat.json` → entrada `"X.Y": { "minApp": "..." }` y subir `"rev"`
- `docs/COMPATIBILIDAD.md` → fila nueva en la tabla
- `OrbyGUI/src/compat.js` → `FEATURES`, solo si hay una función nueva con bandera

`test/compat-tabla.test.mjs` falla si el firmware no tiene entrada en `compat.json` o si su
`minApp` es mayor que la versión de `package.json`.

Commit y `git push origin main`. **`compat.json` tiene que estar en `main` antes de publicar el
firmware**: la app descarga esa tabla de GitHub para saber qué firmware ofrecer.

## 2. Firmware

Compilar (Windows, en este PC):

```bash
cd ORBY_V4
export PATH=/c/msys64/mingw64/bin:$PATH
export PICO_SDK_PATH="$(cygpath -m ~/.pico-sdk/sdk/2.2.0)"   # SDK 2.2.0 con lib/tinyusb
cmake -S . -B build -G Ninja && cmake --build build
grep -a -o "ORBY_V4:FW=[0-9.]*" build/ORBY_V4.elf             # comprobar la versión
cp build/ORBY_V4.uf2 build/ORBY_V4-fw-X.Y.uf2
sha256sum build/ORBY_V4-fw-X.Y.uf2 > build/ORBY_V4-fw-X.Y.uf2.sha256
```

En Linux (nube o servidor): `tools/compilar-linux.sh`.

Publicar:

```bash
git tag -a fw-vX.Y -m "Firmware X.Y" && git push origin fw-vX.Y
python -I .claude/skills/publicar/crear_release.py fw-vX.Y "Firmware X.Y — <resumen>" notas.md --prerelease \
    ORBY_V4/build/ORBY_V4-fw-X.Y.uf2 ORBY_V4/build/ORBY_V4-fw-X.Y.uf2.sha256
```

Notas del firmware: qué cambia, con qué versión de Orvy funciona y una línea `SHA256: \`<hash>\``.
Mira `fw-v4.30` o `fw-v4.31` como modelo.

## 3. App

### 3.1 Compilar sin firmar

```powershell
cd ORBY_V4\OrbyGUI
npm install --ignore-scripts      # npm ci falla: el package-lock está desincronizado
git checkout -- package-lock.json # no subir los cambios del lockfile
npm run tauri:build
```

Termina con `Error A public key has been found, but no private key`. Es lo esperado: el `.exe`
ya está en `src-tauri/target/release/bundle/nsis/Orvy_X.Y.Z_x64-setup.exe`. Falta solo la firma.
Necesita Rust (cadena msvc), Visual Studio Build Tools 2022 con C++, y WebView2.

### 3.2 Firmar en el servidor

1. `sign_update_check` (conector ORVY). Tiene que responder
   «OK: la clave del servidor firma y casa con la clave pública de la app». Si dice que falta la
   clave, para y avisa a Jorge.
   - Si no aparecen `sign_update` / `sign_update_check`, hay que reconectar el conector ORVY en
     claude.ai y **abrir una sesión nueva**: las herramientas se cargan al empezar la conversación.
2. Subir el `.exe` a una rama huérfana temporal que solo contenga el instalador:

   ```bash
   EXE=$(pwd)/ORBY_V4/OrbyGUI/src-tauri/target/release/bundle/nsis/Orvy_X.Y.Z_x64-setup.exe
   WT="$TMP/ra-vX.Y.Z"; git worktree add -q --detach "$WT" && cd "$WT"
   git checkout -q --orphan release-assets/vX.Y.Z && git rm -rq --cached . && git clean -fdxq
   cp "$EXE" . && git add . && git commit -q -m "release-assets: instalador sin firmar (temporal)"
   git push -q origin release-assets/vX.Y.Z
   ```
3. `sign_update` con `repo: "scroll_wheel"`, `git_ref: "release-assets/vX.Y.Z"`,
   `git_path: "Orvy_X.Y.Z_x64-setup.exe"`. Devuelve la firma y el SHA-256.
4. **El SHA-256 devuelto tiene que coincidir con el del `.exe` local.** Si no coincide, no es el
   mismo fichero y la firma no vale: para.

### 3.3 Preparar los ficheros

En la carpeta `nsis/`, junto al `.exe`:

```bash
python -I .claude/skills/publicar/latest_json.py X.Y.Z "<firma devuelta>" "<notas de una línea>" <carpeta nsis>
```

Escribe `Orvy_X.Y.Z_x64-setup.exe.sig` (sin salto de línea al final) y `latest.json` (UTF-8 sin
BOM) con la URL `https://github.com/jaestefaniah27/scroll_wheel/releases/download/vX.Y.Z/Orvy_X.Y.Z_x64-setup.exe`.

### 3.4 Publicar

```bash
git tag -a vX.Y.Z -m "Orvy X.Y.Z" <commit> && git push origin vX.Y.Z
N=ORBY_V4/OrbyGUI/src-tauri/target/release/bundle/nsis
python -I .claude/skills/publicar/crear_release.py vX.Y.Z "Orvy X.Y.Z" notas.md --latest \
    $N/Orvy_X.Y.Z_x64-setup.exe $N/Orvy_X.Y.Z_x64-setup.exe.sig $N/latest.json
```

Notas de la app: `# Orvy X.Y.Z`, `## Novedades` en viñetas y `## Instalación` con el aviso de
SmartScreen y el SHA-256 en mayúsculas (copia el formato de la última release `v*`).

Si el modo automático bloquea la creación de la release, no lo esquives: pide al usuario que
ejecute él el comando de `crear_release.py`.

### 3.5 Comprobar y limpiar

```bash
curl -sL https://github.com/jaestefaniah27/scroll_wheel/releases/latest/download/latest.json   # version = X.Y.Z
git push origin --delete release-assets/vX.Y.Z
git worktree remove --force "$TMP/ra-vX.Y.Z"; git branch -D release-assets/vX.Y.Z
```

## Trampas conocidas

- **No hay `gh`** en este PC. Todo va por la API de GitHub con `crear_release.py`.
- **GitHub Actions está bloqueado** en la cuenta: no hay CI, todo es manual.
- **Ficheros con CRLF:** `sed` con un patrón de varias líneas no encuentra nada. Edita con la
  herramienta Edit o abre los ficheros en Python con `newline=''`.
- **`sha256sum` en Git Bash** pone una `\` delante del hash cuando la ruta lleva barras invertidas.
- **`could not read Username` / `Cannot prompt because user interactivity has been disabled`**
  al hacer push o `git credential fill` desde Claude: el gestor de credenciales tiene dos
  cuentas de GitHub guardadas y, sin saber cuál, intenta abrir una ventana, que aquí no existe
  (y un `git push` normal se queda colgado esperándola). Se arregló fijando la cuenta en la
  configuración local de este repo: `git config --local credential.https://github.com.username 3Javier3`.
  Si vuelve a pasar, comprueba que sigue ahí (`git config --local --get-all credential.https://github.com.username`).
  Si el login ha caducado de verdad, hay que repetirlo a mano en una terminal (`git push`).
  Lanza siempre los push con `timeout 60` y `GIT_TERMINAL_PROMPT=0 GCM_INTERACTIVE=never`.
- **Un 403 en `git push`** suele ser una invitación de colaborador sin aceptar, o una sesión de
  GitHub guardada con otra cuenta.
- **No subas los `tools/test/*.exe`** recompilados en otro PC: cambian de tamaño sin motivo.
