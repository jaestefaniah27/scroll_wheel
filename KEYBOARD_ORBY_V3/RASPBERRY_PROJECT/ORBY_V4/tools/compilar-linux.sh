#!/usr/bin/env bash
# Compila el firmware en Linux (sesiones de Claude en la nube, servidores...).
# En esos entornos NO viene la toolchain ARM ni el Pico SDK: ni el contenedor de
# la sesión ni el servidor Oracle los tienen, así que se instalan aquí. Cuesta
# un par de minutos y se pierde al acabar la sesión.
#
# Uso:  tools/compilar-linux.sh [carpeta_de_trabajo]    (por defecto /tmp/pico)
# Salida: <carpeta>/build/ORBY_V4.uf2
set -euo pipefail

WORK="${1:-/tmp/pico}"
SDK_VERSION=2.2.0   # el mismo que fija CMakeLists.txt (sdkVersion)
V4="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v arm-none-eabi-gcc >/dev/null; then
    # Ubuntu/Debian. Hacen falta también newlib y la libstdc++ para ARM.
    apt-get update -qq
    apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib cmake ninja-build build-essential python3
fi

mkdir -p "$WORK"
if [ ! -d "$WORK/pico-sdk/lib/tinyusb/src" ]; then
    rm -rf "$WORK/pico-sdk"
    git clone --depth 1 --branch "$SDK_VERSION" --recurse-submodules --shallow-submodules \
        https://github.com/raspberrypi/pico-sdk.git "$WORK/pico-sdk"
fi

export PICO_SDK_PATH="$WORK/pico-sdk"
# El build va FUERA del repo: en el árbol dejaría build/ (que ya es material muerto).
cmake -S "$V4" -B "$WORK/build" -G Ninja
cmake --build "$WORK/build"
ls -la "$WORK/build/ORBY_V4.uf2"
