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
# Para lo que falta pero NO es una discrepancia entre el plan y el código: algo que solo
# puede hacer una persona en su máquina. Grita, pero no tumba el script — si contara como
# fallo, «0 discrepancias» dejaría de ser la señal de que todo cuadra y nadie volvería a
# mirarlo.
pendiente() { echo "  PENDIENTE  $1"; }

chk() { # chk <descripcion> <patron> <fichero>
  if grep -qF -- "$2" "$3" 2>/dev/null; then ok "$1"; else mal "$1  (no encontrado: $2  en $3)"; fi
}
chk_re() {
  if grep -qE -- "$2" "$3" 2>/dev/null; then ok "$1"; else mal "$1  (regex sin match: $2  en $3)"; fi
}

# Hasta la Tarea 13 este script leía `electron/` como *referencia* de la migración: los
# canales, las constantes y la superficie del preload se comprobaban allí, porque era el
# original del que Tauri copiaba. Al retirar Electron esa referencia desaparece y todo
# apunta ya a `src-tauri/`. Lo que se pierde con el cambio es la comparación entre las dos
# vías; lo que se gana es que estas comprobaciones miran el código que se ejecuta de verdad.

echo "== Ficheros que el plan nombra =="
for f in ../docs/COMPATIBILIDAD.md src/entry.js src/platform.js src/web/orby-web.js src/live-oled.js \
         src/plugins.js src/device.js src/backup.js \
         plugins/lampdesk/plugin.json src/tauri/orby-tauri.js src-tauri/src/serial.rs \
         src-tauri/src/macros.rs src-tauri/src/recorder.rs src-tauri/src/firmware.rs \
         src-tauri/src/foreground.rs src-tauri/src/apps.rs src-tauri/src/config.rs \
         src-tauri/src/plugins.rs src-tauri/src/main.rs src-tauri/src/updater.rs \
         src-tauri/src/autostart.rs src-tauri/crates/orby-core/src/serie.rs \
         src-tauri/crates/orby-core/src/grabacion.rs src-tauri/crates/orby-core/src/releases.rs \
         index.html vite.config.mjs \
         docs/PLUGINS.md docs/WEBGUI.md docs/PUBLICACION.md docs/TODO.md; do
  [ -f "$f" ] && ok "$f" || mal "$f NO EXISTE"
done

echo "== Electron retirado (Tarea 13) =="
[ -d electron ] && mal "la carpeta electron/ sigue ahí" || ok "no queda carpeta electron/"
grep -qE '"(electron|electron-builder|electron-updater|serialport|uiohook-napi)"' package.json \
  && mal "package.json sigue declarando dependencias de Electron" \
  || ok "package.json sin dependencias de Electron"

echo "== Constantes del puerto serie =="
chk_re "VID 0xCafe"               "VID_TECLADO: u16 = 0xCAFE"   src-tauri/src/serial.rs
chk_re "115200 baudios"           "BAUDIOS: u32 = 115_200"      src-tauri/src/serial.rs
chk    "DTR"                      "write_data_terminal_ready"   src-tauri/src/serial.rs
chk_re "espera de handshake 1200" "ESPERA_SALUDO_MS: u64 = 1200" src-tauri/crates/orby-core/src/serie.rs
chk_re "watchdog: 12 s silencio"  "SILENCIO_MS: u64 = 12_000"   src-tauri/crates/orby-core/src/serie.rs
chk_re "rastreo de puertos 3 s"   "RASTREO_MS: u64 = 3_000"     src-tauri/src/serial.rs

echo "== Constantes de secuencias =="
chk_re "texto por portapapeles >=5"  "MINIMO_PARA_PEGAR: usize = 5"           src-tauri/src/macros.rs
chk_re "portapapeles: asentar 30"    "PORTAPAPELES_ASENTAR_MS: u64 = 30"      src-tauri/src/macros.rs
chk_re "portapapeles: restaurar 120" "PORTAPAPELES_RESTAURAR_MS: u64 = 120"   src-tauri/src/macros.rs
chk_re "4 ms entre letras"           "RETARDO_ENTRE_LETRAS_MS: u64 = 4"       src-tauri/src/macros.rs
chk_re "20 ms entre repeticiones"    "HUECO_POR_DEFECTO_MS: u64 = 20"         src-tauri/src/macros.rs
chk "energía: suspender"  "SetSuspendState 0,1,0"    src-tauri/src/macros.rs
chk "energía: bloquear"   "LockWorkStation"          src-tauri/src/macros.rs

echo "== Constantes de la grabadora =="
chk_re "movimiento cada 16 ms" "MOVIMIENTO_MIN_MS: u64 = 16" src-tauri/crates/orby-core/src/grabacion.rs
chk_re "movimiento 3 px"       "MOVIMIENTO_MIN_PX: i32 = 3"  src-tauri/crates/orby-core/src/grabacion.rs

echo "== Constantes del firmware =="
chk_re "etiqueta fw-v"         "PREFIJO_ETIQUETA: &str = \"fw-v\"" src-tauri/crates/orby-core/src/releases.rs
chk_re "espera de unidad 90 s" "ESPERA_UNIDAD_MS: u64 = 90_000"    src-tauri/src/firmware.rs
chk_re "sondeo de unidad 400"  "RASTREO_UNIDAD_MS: u64 = 400"      src-tauri/src/firmware.rs
chk "INFO_UF2.TXT"           "INFO_UF2.TXT"       src-tauri/src/firmware.rs
chk "errores que son éxito"  "UnexpectedEof"      src-tauri/src/firmware.rs

echo "== Ventana en primer plano =="
chk "GetForegroundWindow"        "GetForegroundWindow"        src-tauri/src/foreground.rs
chk "GetWindowThreadProcessId"   "GetWindowThreadProcessId"   src-tauri/src/foreground.rs
chk_re "sondeo 400 ms"           "SONDEO_MS: u64 = 400"       src-tauri/src/foreground.rs

echo "== Apps instaladas =="
chk "accesos directos por COM"  "IShellLinkW"  src-tauri/src/apps.rs
chk "menú de inicio"            "Start Menu"   src-tauri/src/apps.rs

echo "== Los 40 comandos existen en el invoke_handler de Rust =="
# El `generate_handler!` de main.rs es lo único que decide si un `invoke` existe. Que el
# comando esté declarado en orby-tauri.js no basta: `platform.can()` diría que sí y la
# tarjeta se pintaría muerta, que es justo lo que pasó con autostart_* y updater_* hasta
# la Tarea 12.
handler=$(sed -n '/generate_handler!/,/]);/p' src-tauri/src/main.rs)
for c in updater_get updater_check updater_install updater_set_auto firmware_get firmware_check \
         firmware_update firmware_cancel serial_send serial_get_info serial_get_status \
         serial_reconnect mouse_get_position config_get config_set plugins_list \
         plugins_install plugins_uninstall plugins_set_enabled plugins_get_settings \
         plugins_set_settings plugins_test plugins_read plugins_open_folder \
         autostart_get autostart_set backup_save backup_load dialog_pick_app_or_file \
         apps_list_installed recorder_toggle recorder_stop recorder_status \
         foreground_start foreground_stop foreground_current foreground_available \
         window_minimize window_maximize window_close; do
  echo "$handler" | grep -qE "(^|[^a-z_])$c *,?$" && ok "comando $c" \
    || mal "comando $c NO ESTA registrado en el invoke_handler de Rust"
done

echo "== Los 10 eventos, tambien en Rust =="
# Ocho se emiten. Los otros dos no existen en esta vía a propósito, y lo que se comprueba
# es que siga escrito por qué: una ausencia sin explicar es indistinguible de un olvido.
for e in serial:connected serial:disconnected serial:searching serial:data \
         firmware:state recorder:state updater:state foreground:change; do
  grep -rqF "\"$e\"" src-tauri/src/ && ok "evento $e" || mal "evento $e NO SE EMITE en Rust"
done
chk "serial:error: ausencia justificada"     "El evento \`serial:error\`"     src-tauri/src/serial.rs
chk "foreground:error: ausencia justificada" "El evento \`foreground:error\`" src-tauri/src/foreground.rs

echo "== Autoarranque y avisador de versiones (Tarea 12) =="
chk    "la entrada va en la clave Run"        "CurrentVersion\\Run"  src-tauri/src/autostart.rs
chk    "y conserva --hidden"                  '--hidden'             src-tauri/src/autostart.rs
chk    "main.rs lee el argumento"             "ARG_ESCONDIDO"        src-tauri/src/main.rs
# Sin esto, arrancar por autoarranque enseña la ventana un instante antes de esconderla.
chk    "la ventana nace invisible"            '"visible": false'     src-tauri/tauri.conf.json
# Un segundo proceso pelearia por el COM con el que ya arrancó escondido.
chk    "una sola instancia"                   "single_instance"      src-tauri/src/main.rs
chk    "el avisador mira releases/latest"     "releases/latest"      src-tauri/src/updater.rs
chk    "y descarta las de firmware"           "PREFIJO_FIRMWARE"     src-tauri/src/updater.rs
chk_re "el frontend pinta el estado nuevo"    "case 'available'"     src/updater.js

echo "== Actualizaciones automaticas y silenciosas =="
# La clave publica sin poner deja el actualizador sin poder verificar nada, y el fallo no
# se ve hasta que hay una release que instalar.
chk    "el actualizador declara su endpoint"  "latest.json"          src-tauri/tauri.conf.json
# La clave la tiene que generar el autor en su maquina (`npm run tauri signer generate`):
# la privada no puede vivir en el repositorio, asi que esto no se puede dejar hecho desde
# aqui. Hasta entonces el actualizador automatico no funciona.
if grep -q "PEGAR-AQUI-LA-CLAVE-PUBLICA" src-tauri/tauri.conf.json; then
  pendiente "falta generar el par de claves del actualizador y pegar la publica en tauri.conf.json (docs/PUBLICACION.md, seccion 3)"
else
  ok "la clave publica del actualizador esta puesta"
fi
chk    "el NSIS se instala sin dialogos"      '"installMode": "passive"' src-tauri/tauri.conf.json
chk    "y el build genera los artefactos"     "createUpdaterArtifacts"   src-tauri/tauri.conf.json
chk    "permiso de reinicio concedido"        "process:allow-restart"    src-tauri/capabilities/default.json
# Sin esto el reinicio se lo come el cierre a la bandeja y no pasa nada, sin error.
chk    "el reinicio marca la salida de verdad" "marcar_saliendo"     src-tauri/src/updater.rs
# La guarda que impide reiniciar a mitad de un flasheo, que es lo que dejaria el teclado
# sin firmware.
chk    "no se reinicia a mitad de un flasheo" "fn mal_momento"       src-tauri/src/updater.rs
chk    "el firmware se instala solo si hay ocio" "OCIO_MS"           src/firmware-auto.js
# `telemetry` y no `rx`: el latido HOST_APP cada 8 s haria que nunca hubiera ocio.
chk    "y el ocio se mide con la telemetria"  "'telemetry'"          src/firmware-auto.js
chk    "los dos automaticos se pueden apagar" "autoFirmware"         src-tauri/crates/orby-core/src/config.rs
# Cuatro niveles arriba: el repositorio de git es scroll_wheel entero y este proyecto vive
# anidado dentro.
chk    "las releases de firmware van como prerelease" "--prerelease" ../../../../.github/workflows/firmware.yml

echo "== Superficie de orby-tauri.js =="
for m in sendCommand getDeviceInfo getStatus reconnect onConnected onDisconnected \
         onData onError onSearching getMousePosition getConfig setConfig saveBackup \
         loadBackup pickAppOrFile listInstalledApps minimize maximize close; do
  grep -qE "^\s*$m[:(]" src/tauri/orby-tauri.js && ok "orby.$m" || mal "orby.$m NO ESTA en orby-tauri.js"
done

echo "== Espejo en RAM de las OLED (la base de la Tarea 11) =="
chk "live-oled usa uploadOled"  "device.uploadOled"  src/live-oled.js
chk "y no toca la Flash"        "Guardar en Flash"   src/live-oled.js

echo "== Descriptor de complementos que consume el frontend =="
chk "hasRead"    "hasRead"    src-tauri/crates/orby-core/src/plugins/manifiesto.rs
chk "apiVersion" "apiVersion" src-tauri/crates/orby-core/src/plugins/manifiesto.rs
chk    "id de complemento" "fn id_valido" src-tauri/crates/orby-core/src/plugins/manifiesto.rs

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

echo "== Superficie: las dos vias tienen los mismos metodos =="
node --test test/superficie-orby.test.mjs >/dev/null 2>&1 && ok "test de superficie en verde" || mal "test de superficie ROJO (node --test test/superficie-orby.test.mjs)"

echo
echo "=================================="
if [ "$fallos" -eq 0 ]; then echo "TODO CUADRA: 0 discrepancias"; else echo "DISCREPANCIAS: $fallos"; fi
exit "$fallos"
