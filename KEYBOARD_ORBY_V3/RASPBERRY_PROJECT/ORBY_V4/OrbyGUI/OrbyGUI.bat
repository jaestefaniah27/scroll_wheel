@echo off
REM Lanzador de OrbyGUI. Doble clic y listo.
REM Instala dependencias la primera vez, recompila si el codigo cambio y abre la app.

setlocal
cd /d "%~dp0"
title OrbyGUI

REM VS Code y otras apps Electron exportan estas variables a sus terminales.
REM Si se heredan, electron.exe arranca como Node pelado y main.js revienta con
REM "Cannot read properties of undefined (reading 'handle')". Se limpian siempre.
set "ELECTRON_RUN_AS_NODE="
set "NODE_OPTIONS="

where node >nul 2>nul
if errorlevel 1 (
  echo [ERROR] No se encuentra Node.js en el PATH.
  echo         Instalalo desde https://nodejs.org  ^(version 20.19 o superior^)
  echo.
  pause
  exit /b 1
)

REM Se comprueba cli.js y no solo la carpeta: una instalacion interrumpida deja
REM node_modules\electron a medias, la carpeta existe pero la app no arranca.
if not exist "node_modules\electron\cli.js" (
  echo Instalando dependencias, esto tarda un rato la primera vez...
  call npm install
  if errorlevel 1 (
    echo.
    echo [ERROR] Fallo la instalacion de dependencias.
    echo         Si el error es EBUSY o "resource busy", pausa la sincronizacion
    echo         de OneDrive, borra la carpeta node_modules y vuelve a intentarlo.
    pause
    exit /b 1
  )
)

echo Compilando interfaz...
call npx vite build
if errorlevel 1 (
  echo.
  echo [ERROR] Fallo la compilacion.
  pause
  exit /b 1
)

echo Abriendo OrbyGUI...
call npx electron .

endlocal
