# Crea el acceso directo del escritorio para Orby Manager.
#
# Sin acentos a proposito: powershell.exe 5.1 (el que trae Windows por
# defecto) interpreta un .ps1 sin BOM como ANSI, y cualquier caracter no
# ASCII sale mal en consola. Evitarlos aqui evita depender de que el fichero
# se guarde con la codificacion correcta.

# $PSScriptRoot y no una ruta fija: este script vive junto al .cmd al que
# apunta, pero la carpeta del repo puede estar en cualquier sitio segun la
# maquina (aqui cuelga de OneDrive, en otra puede no ser asi).
$carpeta = $PSScriptRoot
$cmdPath = Join-Path $carpeta 'Orby Manager.cmd'

# El icono es el de Tauri, no el de Electron: assets/orby-icon.png es en
# realidad un JPEG con la extension cambiada a .png (electron-builder lo
# traga sin quejarse, pero Windows Shell no acepta un JPEG como icono de
# acceso directo). El .ico de src-tauri si es un .ico de verdad.
$icoPath = Join-Path $carpeta '..\..\OrbyGUI\src-tauri\icons\icon.ico'
$icoPath = [System.IO.Path]::GetFullPath($icoPath)

# [Environment]::GetFolderPath('Desktop') y no "$env:USERPROFILE\Desktop":
# cuando OneDrive redirige el escritorio real, la carpeta bajo el perfil
# puede no existir o no ser la que el usuario ve. GetFolderPath devuelve la
# ruta que Windows usa de verdad, redirigida o no.
$escritorio = [Environment]::GetFolderPath('Desktop')
$lnkPath = Join-Path $escritorio 'Orby Manager.lnk'

$faltan = @()
if (-not (Test-Path -LiteralPath $cmdPath)) {
    $faltan += "el lanzador ($cmdPath)"
}
if (-not (Test-Path -LiteralPath $icoPath)) {
    $faltan += "el icono ($icoPath)"
}

if ($faltan.Count -gt 0) {
    Write-Host "No se puede crear el acceso directo, falta:" -ForegroundColor Red
    foreach ($item in $faltan) {
        Write-Host "  - $item" -ForegroundColor Red
    }
    exit 1
}

$WshShell = New-Object -ComObject WScript.Shell
$shortcut = $WshShell.CreateShortcut($lnkPath)
$shortcut.TargetPath = $cmdPath
$shortcut.WorkingDirectory = $carpeta
# 7 = minimizado. El servidor vive en la ventana de consola del .cmd durante
# toda la sesion del panel; minimizada en vez de oculta para que el usuario
# pueda encontrarla y cerrarla cuando quiera parar el servidor.
$shortcut.WindowStyle = 7
$shortcut.IconLocation = $icoPath
$shortcut.Description = 'Panel de control de ORBY_V4: versiones, lanzar la app y publicar releases'
$shortcut.Save()

Write-Host "Acceso directo creado en: $lnkPath"
