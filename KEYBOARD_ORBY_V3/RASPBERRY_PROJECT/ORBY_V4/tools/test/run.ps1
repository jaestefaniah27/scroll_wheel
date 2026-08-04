# Pruebas que se ejecutan en el PC, sin teclado.
# Uso:  powershell -ExecutionPolicy Bypass -File .\tools\test\run.ps1
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not (Get-Command g++ -ErrorAction SilentlyContinue)) {
    Write-Host "No encuentro g++ en el PATH." -ForegroundColor Red
    Write-Host "Con Strawberry Perl instalado suele estar en C:\Strawberry\c\bin." -ForegroundColor Gray
    exit 1
}

$failed = $false

foreach ($t in @('test_hid_out', 'test_wheel')) {
    Write-Host "Compilando $t..." -ForegroundColor Gray
    & g++ -O1 -Wall -Wextra -Wno-unused-function -I"$here" -I"$here\..\..\include" `
          -o "$here\$t.exe" "$here\$t.cpp"
    if ($LASTEXITCODE -ne 0) { Write-Host "No compila." -ForegroundColor Red; exit 1 }
    & "$here\$t.exe"
    if ($LASTEXITCODE -ne 0) { $failed = $true }
}

# La huella con la que el teclado y la app deciden si hay que sincronizar esta
# escrita dos veces, en C++ y en JavaScript. Si dejan de dar lo mismo, la app
# vuelve a descargarlo todo en cada conexion sin que nadie se entere.
Write-Host "`nComparando la huella de la app con la del firmware..." -ForegroundColor Gray
& g++ -O1 -Wall -Wextra -Wno-unused-function -I"$here\..\..\include" `
      -o "$here\test_config_hash.exe" "$here\test_config_hash.cpp"
if ($LASTEXITCODE -ne 0) {
    Write-Host "No compila." -ForegroundColor Red
    $failed = $true
} else {
    & node "$here\test_config_hash.mjs"
    if ($LASTEXITCODE -ne 0) { $failed = $true }
}

# El descriptor HID: si esta mal formado, Windows no reconoce el teclado.
Write-Host "`nValidando el descriptor HID..." -ForegroundColor Gray
& python "$here\check_descriptor.py"
if ($LASTEXITCODE -ne 0) { $failed = $true }

# Firmware y app tienen que estar de acuerdo en las teclas de sistema.
Write-Host "`nComprobando las teclas de sistema..." -ForegroundColor Gray
& python "$here\check_keys.py"
if ($LASTEXITCODE -ne 0) { $failed = $true }

# Disposicion de la Flash: un tramo mal calculado se pisa iconos o ajustes.
Write-Host "`nComprobando la disposicion de la Flash..." -ForegroundColor Gray
& python "$here\check_flash_map.py"
if ($LASTEXITCODE -ne 0) { $failed = $true }

if ($failed) {
    Write-Host "`n[ FALLO ] Hay pruebas en rojo." -ForegroundColor Red
    exit 1
}
Write-Host "`n[ OK ] Todo en verde." -ForegroundColor Green
