@echo off
REM Lanzador de OrbyGUI en modo desarrollo. Doble clic y listo.
REM Instala dependencias la primera vez y arranca la app de Tauri con recarga.
REM
REM Para el uso normal esto no hace falta: la app se instala con el .exe que produce
REM `npm run tauri:build` y se abre desde el menu de inicio. Esto es para tocar el codigo.

setlocal
cd /d "%~dp0"
title OrbyGUI (desarrollo)

where node >nul 2>nul
if errorlevel 1 (
  echo [ERROR] No se encuentra Node.js en el PATH.
  echo         Instalalo desde https://nodejs.org  ^(version 20.19 o superior^)
  echo.
  pause
  exit /b 1
)

where cargo >nul 2>nul
if errorlevel 1 (
  echo [ERROR] No se encuentra Cargo en el PATH.
  echo         La cascara de la app es Rust: instala Rust desde https://rustup.rs
  echo.
  pause
  exit /b 1
)

REM Se comprueba el binario del CLI y no solo la carpeta: una instalacion interrumpida
REM deja node_modules a medias, la carpeta existe pero nada arranca.
if not exist "node_modules\.bin\tauri.cmd" (
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

REM `tauri dev` levanta Vite el solo (beforeDevCommand en tauri.conf.json) y compila
REM la cascara de Rust. La primera vez tarda varios minutos.
echo Arrancando OrbyGUI...
call npm run tauri:dev

endlocal
