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

:: Bucle de espera. La letra de unidad la asigna Windows y no siempre es la
:: misma, asi que se recorren todas buscando el INFO_UF2.TXT que la Pico deja
:: en la raiz. Antes estaba fija en D: y el script se quedaba colgado si el
:: sistema la montaba en otra letra.
:wait_loop
for %%D in (D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
    if exist %%D:\INFO_UF2.TXT (
        set PICO_DRIVE=%%D:
        goto do_flash
    )
)
<nul set /p =.
timeout /t 1 /nobreak >nul
goto wait_loop

:do_flash
echo.
echo [ OK ] Pico detectada en %PICO_DRIVE%\. Copiando firmware...
copy build\ORBY_V4.uf2 %PICO_DRIVE%\
if %errorlevel% neq 0 (
    echo [ERROR] Fallo al copiar el archivo a la unidad %PICO_DRIVE%
) else (
    echo [ OK ] ¡Flasheo completado! La Pico se reiniciara sola.
)

pause
