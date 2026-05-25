@echo off
title Compilador y Flasheador de ORBY_V4

echo =========================================
echo 1. Iniciando Compilacion...
echo =========================================

:: Verificar e inicializar compilacion limpia si es necesario
if exist build\CMakeCache.txt (
    findstr /c:"scroll_wheel/scroll_wheel" build\CMakeCache.txt >nul
    if not errorlevel 1 (
        echo [!] Detectada cache antigua conflictiva. Limpiando carpeta build...
        rd /s /q build
    )
)

:: Crear carpeta build si no existe y configurar CMake
if not exist build (
    echo Generando archivos de compilacion con CMake...
    cmake -B build -G "Ninja"
    if %errorlevel% neq 0 (
        echo [ERROR] Fallo la configuracion de CMake.
        pause
        exit /b %errorlevel%
    )
)

:: Compilar
echo Compilando firmware...
cmake --build build
if %errorlevel% neq 0 (
    echo [ERROR] La compilacion fallo.
    pause
    exit /b %errorlevel%
)

echo.
echo [ OK ] Proyecto compilado con exito.
echo.
echo =========================================
echo 2. Esperando a la Raspberry Pi Pico...
echo =========================================
echo Por favor, manten pulsado BOOTSEL y conecta la Pico al USB.

:: Bucle de espera a que aparezca la unidad D:
:wait_loop
if exist D:\ (
    if exist D:\INFO_UF2.TXT (
        goto do_flash
    )
)
<nul set /p =.
timeout /t 1 /nobreak >nul
goto wait_loop

:do_flash
echo.
echo [ OK ] Pico detectada en D:\. Copiando firmware...
copy build\ORBY_V4.uf2 D:\
if %errorlevel% neq 0 (
    echo [ERROR] Fallo al copiar el archivo a la unidad D:
) else (
    echo [ OK ] ¡Flasheo completado! La Pico se reiniciara sola.
)

pause
