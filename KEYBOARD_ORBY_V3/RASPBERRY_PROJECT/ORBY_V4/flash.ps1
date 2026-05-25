# flash.ps1 - Script de compilación y flasheo automático para ORBY_V4
$ErrorActionPreference = "Stop"

# 1. Compilación
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "1. Iniciando Compilación..." -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

# Comprobar si hay que limpiar caché antigua
if (Test-Path .\build\CMakeCache.txt) {
    $cache = Get-Content .\build\CMakeCache.txt -ErrorAction SilentlyContinue
    if ($cache -match "scroll_wheel/scroll_wheel") {
        Write-Host "Detectada caché antigua inválida. Limpiando carpeta build..." -ForegroundColor Yellow
        Remove-Item -Recurse -Force build
    }
}

try {
    if (-not (Test-Path .\build)) {
        Write-Host "Generando archivos del proyecto con CMake..." -ForegroundColor Gray
        cmake -B build -G "Ninja"
    }
    
    Write-Host "Compilando firmware..." -ForegroundColor Gray
    cmake --build build
    Write-Host "`n[ OK ] Proyecto compilado con éxito." -ForegroundColor Green
} catch {
    Write-Host "`n[ ERROR ] La compilación ha fallado. Revisa los mensajes de arriba." -ForegroundColor Red
    exit 1
}

# 2. Espera y Flasheo de la Raspberry Pi Pico
$uf2_path = ".\build\ORBY_V4.uf2"
$target_drive = "D:\"

Write-Host "`n=========================================" -ForegroundColor Cyan
Write-Host "2. Esperando a la Raspberry Pi Pico..." -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "Por favor, mantén pulsado BOOTSEL y conecta la Pico al USB (Unidad $target_drive)." -ForegroundColor Yellow

# Bucle de espera
$found = $false
while (-not $found) {
    if (Test-Path $target_drive) {
        # Verificar si es un volumen RP2
        try {
            $volume = Get-Volume -DriveLetter D -ErrorAction SilentlyContinue
            if ($volume.FileSystemLabel -eq "RPI-RP2" -or (Test-Path (Join-Path $target_drive "INFO_UF2.TXT"))) {
                $found = $true
            }
        } catch {
            # Si Get-Volume no está disponible o falla, confiamos en Test-Path básico
            $found = $true
        }
    }
    
    if (-not $found) {
        Start-Sleep -Seconds 1
        Write-Host "." -NoNewline -ForegroundColor Gray
    }
}

Write-Host "`n[ OK ] Pico detectada en $target_drive. Copiando firmware..." -ForegroundColor Green

try {
    Copy-Item -Path $uf2_path -Destination $target_drive
    Write-Host "[ OK ] ¡Flasheo completado! La Pico se reiniciará sola." -ForegroundColor Green
} catch {
    Write-Host "[ ERROR ] Fallo al transferir el firmware a la unidad $target_drive." -ForegroundColor Red
}
