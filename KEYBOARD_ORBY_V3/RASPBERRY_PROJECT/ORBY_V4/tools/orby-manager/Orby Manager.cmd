@echo off
rem Lanzador de Orby Manager. Se invoca desde un acceso directo del escritorio,
rem asi que el directorio de trabajo heredado puede ser cualquier cosa (el
rem escritorio, System32...): nos colocamos primero en la carpeta del propio
rem script para que "node server.js" encuentre sus ficheros.
cd /d "%~dp0"

rem Este .cmd se llama a si mismo con el parametro "abrir" para abrir el
rem navegador un par de segundos despues de arrancar node, sin bloquear la
rem ventana donde vive el servidor (ver mas abajo). Si venimos por ahi, solo
rem hacemos eso y salimos.
if "%~1"=="abrir" goto abrir

rem Solo puede haber una instancia por maquina: las tareas del panel mutan el
rem repositorio (bump de version, build, publicar) y dos paneles a la vez
rem podrian lanzar dos builds al mismo tiempo sobre los mismos ficheros.
rem Comprobamos el puerto con netstat en vez de con curl porque curl no esta
rem garantizado en todas las maquinas (versiones antiguas de Windows no lo
rem traen de serie); netstat si.
rem El filtro con corchete exige que despues de "7788" venga algo que no sea
rem un digito (normalmente un espacio), para no confundirlo con otro puerto
rem que empiece igual, como el 77880.
netstat -ano | findstr /r /c:":7788[^0-9]" | findstr "LISTENING" >nul
if %errorlevel%==0 (
    rem Ya hay un Orby Manager escuchando: no arrancamos otro, solo abrimos
    rem la pestana del navegador.
    start "" http://localhost:7788
    exit /b 0
)

where node >nul 2>nul
if errorlevel 1 (
    echo No se encuentra "node" en el PATH de este equipo.
    echo Instala Node.js o abre esta ventana desde una consola donde "node" funcione.
    pause
    exit /b 1
)

rem Lanzamos en una ventana minimizada aparte el aviso al navegador, que
rem espera un par de segundos a que el servidor levante antes de abrir la
rem pestana. Se hace en una ventana propia (no con "start /b") porque el
rem siguiente comando de este script es "node server.js" y ese si ocupa esta
rem ventana durante toda la sesion del panel.
start "" /min cmd /c "%~f0" abrir

rem El servidor se queda en esta ventana a proposito: por eso el acceso
rem directo del escritorio la abre minimizada en vez de oculta. Si lo
rem lanzaramos con "start /b" el proceso "node" quedaria huerfano, sin
rem ventana que cerrar para pararlo cuando el usuario quiera terminar.
node server.js
exit /b 0

:abrir
timeout /t 2 /nobreak >nul
start "" http://localhost:7788
exit /b 0
