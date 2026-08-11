#!/bin/bash
# Comprueba que cada dato concreto de PLAN_TAURI.md existe de verdad en el código.
#
# El plan de Tauri cita constantes, rutas y nombres de canal del código actual. Si el
# código deriva, el plan miente y quien lo ejecute se pierde. Este script lo comprueba.
#   bash tools/test/verifica_plan_tauri.sh
# Relativo al propio script: el repo se clona en sitios distintos (contenedor Linux,
# Windows bajo OneDrive) y una ruta fija solo valía en la máquina donde se escribió.
cd "$(dirname "$0")/../../OrbyGUI" || exit 1
fallos=0
ok()   { echo "  ok   $1"; }
mal()  { echo "  MAL  $1"; fallos=$((fallos+1)); }

chk() { # chk <descripcion> <patron> <fichero>
  if grep -qF -- "$2" "$3" 2>/dev/null; then ok "$1"; else mal "$1  (no encontrado: $2  en $3)"; fi
}
chk_re() {
  if grep -qE -- "$2" "$3" 2>/dev/null; then ok "$1"; else mal "$1  (regex sin match: $2  en $3)"; fi
}

echo "== Ficheros que el plan nombra =="
for f in ../docs/COMPATIBILIDAD.md src/entry.js src/platform.js src/web/orby-web.js src/live-oled.js \
         src/plugins.js src/device.js src/backup.js plugins/lampdesk/main.js \
         plugins/lampdesk/plugin.json electron/preload.js electron/serial.js \
         electron/macros.js electron/recorder.js electron/firmware.js \
         electron/foreground.js electron/apps.js electron/config.js \
         electron/plugins.js electron/main.js index.html vite.config.mjs \
         docs/PLUGINS.md docs/WEBGUI.md docs/PUBLICACION.md docs/TODO.md; do
  [ -f "$f" ] && ok "$f" || mal "$f NO EXISTE"
done

echo "== Constantes del puerto serie =="
chk "VID 0xCafe"                "vid: 'cafe'"        electron/serial.js
chk "115200 baudios"            "115200"             electron/serial.js
chk "DTR y RTS"                 "dtr: true"          electron/serial.js
chk_re "espera de handshake 1200" "HANDSHAKE_WAIT_MS *= *1200" electron/serial.js
chk_re "watchdog: 12 s silencio"  "12000|12_000"     electron/serial.js
chk_re "rastreo de puertos 3 s"   "3000"             electron/serial.js

echo "== Constantes de secuencias =="
chk_re "texto por portapapeles >=5" "TEXT_PASTE_MIN_CHARS *= *5"   electron/macros.js
chk_re "portapapeles: asentar 30"   "CLIPBOARD_SETTLE_MS *= *30"   electron/macros.js
chk_re "portapapeles: restaurar 120" "CLIPBOARD_RESTORE_MS *= *120" electron/macros.js
chk_re "4 ms entre letras"          "TEXT_CHAR_DELAY_MS *= *4"     electron/macros.js
chk_re "20 ms entre repeticiones"   "DEFAULT_REPEAT_GAP_MS *= *20" electron/macros.js
chk "energía: suspender"  "SetSuspendState 0,1,0"    electron/macros.js
chk "energía: bloquear"   "LockWorkStation"          electron/macros.js

echo "== Constantes de la grabadora =="
chk_re "movimiento cada 16 ms" "MOVE_MIN_MS *= *16" electron/recorder.js
chk_re "movimiento 3 px"       "MOVE_MIN_PX *= *3"  electron/recorder.js

echo "== Constantes del firmware =="
chk_re "etiqueta fw-v"       "TAG_PREFIX *= *'fw-v'"      electron/firmware.js
chk_re "espera de unidad 90 s" "DRIVE_TIMEOUT_MS *= *90_?000" electron/firmware.js
chk_re "sondeo de unidad 400"  "DRIVE_POLL_MS *= *400"    electron/firmware.js
chk "INFO_UF2.TXT"           "INFO_UF2.TXT"               electron/firmware.js
chk "errores que son éxito"  "EIO"                        electron/firmware.js

echo "== Ventana en primer plano =="
chk "GetForegroundWindow"        "GetForegroundWindow"        electron/foreground.js
chk "GetWindowThreadProcessId"   "GetWindowThreadProcessId"   electron/foreground.js
chk_re "sondeo 400 ms"           "400"                        electron/foreground.js

echo "== Apps instaladas =="
chk "readShortcutLink"  "readShortcutLink"  electron/apps.js
chk "menú de inicio"    "Start Menu"        electron/apps.js

echo "== Los 39 canales del plan existen en main.js =="
for c in updater:get updater:check updater:install firmware:get firmware:check \
         firmware:update firmware:cancel serial:send serial:getInfo serial:getStatus \
         serial:reconnect mouse:getPosition config:get config:set plugins:list \
         plugins:install plugins:uninstall plugins:setEnabled plugins:getSettings \
         plugins:setSettings plugins:test plugins:read plugins:openFolder \
         autostart:get autostart:set backup:save backup:load dialog:pickAppOrFile \
         apps:listInstalled recorder:toggle recorder:stop recorder:status \
         foreground:start foreground:stop foreground:current foreground:available \
         window:minimize window:maximize window:close; do
  grep -qF "'$c'" electron/main.js && ok "canal $c" || mal "canal $c NO ESTA en main.js"
done

echo "== Los 10 eventos =="
for e in serial:connected serial:disconnected serial:error serial:searching serial:data \
         firmware:state recorder:state updater:state foreground:change foreground:error; do
  grep -qF "'$e'" electron/main.js && ok "evento $e" || mal "evento $e NO ESTA en main.js"
done

echo "== Superficie de preload.js =="
for m in sendCommand getDeviceInfo getStatus reconnect onConnected onDisconnected \
         onData onError onSearching getMousePosition getConfig setConfig saveBackup \
         loadBackup pickAppOrFile listInstalledApps minimize maximize close; do
  grep -qE "^\s*$m[:(]" electron/preload.js && ok "orby.$m" || mal "orby.$m NO ESTA en preload.js"
done

echo "== Espejo en RAM de las OLED (la base de la Tarea 11) =="
chk "live-oled usa uploadOled"  "device.uploadOled"  src/live-oled.js
chk "y no toca la Flash"        "Guardar en Flash"   src/live-oled.js

echo "== Descriptor de complementos que consume el frontend =="
chk "hasRead"  "hasRead"  electron/plugins.js
chk "apiVersion" "apiVersion" electron/plugins.js
chk_re "id de complemento" "a-z0-9" electron/plugins.js

echo "== Costura de Tauri (Tarea 1) =="
chk "entry.js detecta Tauri"        "__TAURI_INTERNALS__"  src/entry.js
chk "orby-tauri usa el global"      "window.__TAURI__"     src/tauri/orby-tauri.js
if grep -qE "^\s*import .*@tauri-apps|from .@tauri-apps" src/tauri/orby-tauri.js; then
  mal "orby-tauri.js importa @tauri-apps/api: eso obliga a instalarlo para compilar tambien las otras dos vias"
else
  ok "orby-tauri.js no depende de ningun paquete npm nuevo"
fi
chk "los permisos del nucleo estan concedidos" "core:default" src-tauri/capabilities/default.json
if grep -q "@tauri-apps" package.json; then
  if grep -q "@tauri-apps" package-lock.json; then ok "package.json y el lock cuadran"
  else mal "package.json declara @tauri-apps pero el lock no lo tiene: npm ci fallaria"; fi
else
  ok "package.json sin dependencias nuevas"
fi

echo "== Superficie: las tres vias tienen los mismos metodos =="
node --test test/superficie-orby.test.mjs >/dev/null 2>&1 && ok "test de superficie en verde" || mal "test de superficie ROJO (node --test test/superficie-orby.test.mjs)"

echo
echo "=================================="
if [ "$fallos" -eq 0 ]; then echo "TODO CUADRA: 0 discrepancias"; else echo "DISCREPANCIAS: $fallos"; fi
exit "$fallos"
