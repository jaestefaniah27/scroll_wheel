@echo off
REM Lanzador de OrbyGUI. Doble clic y listo.
REM Instala dependencias la primera vez, recompila si el codigo cambio y abre la app.

setlocal
cd /d "%~dp0"
title OrbyGUI

where node >nul 2>nul
if errorlevel 1 (
  echo [ERROR] No se encuentra Node.js en el PATH.
  echo         Instalalo desde https://nodejs.org  ^(version 20.19 o superior^)
  echo.
  pause
  exit /b 1
)

if not exist "node_modules\electron" (
  echo Instalando dependencias, esto tarda un rato la primera vez...
  call npm install
  if errorlevel 1 (
    echo.
    echo [ERROR] Fallo la instalacion de dependencias.
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
