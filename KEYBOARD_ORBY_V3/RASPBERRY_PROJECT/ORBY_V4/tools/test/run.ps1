# Pruebas que se ejecutan en el PC, sin teclado.
# Uso:  powershell -ExecutionPolicy Bypass -File .\tools\test\run.ps1
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not (Get-Command g++ -ErrorAction SilentlyContinue)) {
    Write-Host "No encuentro g++ en el PATH." -ForegroundColor Red
    Write-Host "Con Strawberry Perl instalado suele estar en C:\Strawberry\c\bin." -ForegroundColor Gray
    exit 1
}

Write-Host "Compilando las pruebas..." -ForegroundColor Gray
& g++ -O1 -Wall -Wextra -I"$here" -I"$here\..\..\include" -o "$here\test_hid_out.exe" "$here\test_hid_out.cpp"
if ($LASTEXITCODE -ne 0) { Write-Host "No compila." -ForegroundColor Red; exit 1 }

& "$here\test_hid_out.exe"
if ($LASTEXITCODE -ne 0) {
    Write-Host "`n[ FALLO ] Hay pruebas en rojo." -ForegroundColor Red
    exit 1
}
Write-Host "`n[ OK ] Todo en verde." -ForegroundColor Green
