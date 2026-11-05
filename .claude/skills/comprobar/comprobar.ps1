# Pasa todas las pruebas de ORBY que corren en el PC, sin teclado, y dice si se puede subir.
#
#   powershell -ExecutionPolicy Bypass -File comprobar.ps1          # todo
#   powershell -ExecutionPolicy Bypass -File comprobar.ps1 -SoloApp # solo los tests de OrbyGUI
#
# 1. Tests de la app con `node --test` (npm test está roto con Node 24).
# 2. tools/test/run.ps1: tests C++ del firmware, iconos regenerados, perfiles, huella de
#    configuración app/firmware, descriptor HID, teclas de sistema y mapa de la Flash.
# 3. git: rama, si va por delante o por detrás de origin y qué hay sin subir.
#
# run.ps1 recompila tools/test/*.exe y los deja modificados. Los que estaban limpios antes
# se restauran al acabar para que no acaben en un commit por error.
param([switch]$SoloApp)
$ErrorActionPreference = 'Continue'

$repo = (git -C $PSScriptRoot rev-parse --show-toplevel).Trim()
$v4 = Join-Path $repo 'KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4'
$resultado = [ordered]@{}

# msys64 delante siempre, no solo si falta g++: con otra carpeta antes en el PATH (Git Bash
# trae la suya), g++ carga DLLs que no son las suyas y falla sin decir nada («No compila.»).
if (Test-Path 'C:\msys64\mingw64\bin\g++.exe') { $env:PATH = "C:\msys64\mingw64\bin;$env:PATH" }

Write-Host "`n=== Tests de la app (OrbyGUI) ===" -ForegroundColor Cyan
Push-Location (Join-Path $v4 'OrbyGUI')
$tests = Get-ChildItem test -Filter *.test.mjs | ForEach-Object { "test/$($_.Name)" }
$salida = & node --test --test-reporter=tap @tests 2>&1
Pop-Location
$salida | Select-String -Pattern '^# (tests|pass|fail)|^not ok' | ForEach-Object { Write-Host $_.Line }
$resultado['App (node --test)'] = ($LASTEXITCODE -eq 0)
if ($LASTEXITCODE -ne 0) {
    # Los fallos con su contexto, sin las cientos de líneas de los que pasan.
    $salida | Select-String -Pattern '^not ok' -Context 0, 25 | ForEach-Object { Write-Host $_ -ForegroundColor Red }
}

if (-not $SoloApp) {
    Write-Host "`n=== Firmware en el PC (tools/test/run.ps1) ===" -ForegroundColor Cyan
    $exes = git -C $v4 ls-files 'tools/test/*.exe'
    $limpios = @($exes | Where-Object { git -C $v4 diff --quiet -- $_; $LASTEXITCODE -eq 0 })
    Push-Location $v4
    & powershell -NoProfile -ExecutionPolicy Bypass -File tools\test\run.ps1
    $resultado['Firmware (run.ps1)'] = ($LASTEXITCODE -eq 0)
    if ($limpios) { git checkout -q -- @limpios }
    Pop-Location
}

Write-Host "`n=== git ===" -ForegroundColor Cyan
git -C $repo fetch -q origin 2>$null
git -C $repo status -sb | Select-Object -First 25

Write-Host "`n=== Resumen ===" -ForegroundColor Cyan
$ok = $true
foreach ($k in $resultado.Keys) {
    if ($resultado[$k]) { Write-Host "[ OK ] $k" -ForegroundColor Green }
    else { Write-Host "[FALLO] $k" -ForegroundColor Red; $ok = $false }
}
if ($ok) { exit 0 } else { exit 1 }
