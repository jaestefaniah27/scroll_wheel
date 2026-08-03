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
    # Se comprueba CMakeCache.txt, no solo que exista la carpeta: un build
    # interrumpido a medias deja "build\" creada (p.ej. solo CMakeFiles\) sin
    # llegar a generar la caché, y "cmake --build" contra eso falla con
    # "could not load cache" en vez de volver a configurar.
    if (-not (Test-Path .\build\CMakeCache.txt)) {
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

Write-Host "`n=========================================" -ForegroundColor Cyan
Write-Host "2. Esperando a la Raspberry Pi Pico..." -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "Por favor, mantén pulsado BOOTSEL y conecta la Pico al USB." -ForegroundColor Yellow

# La letra de unidad la asigna Windows y no siempre es la misma: buscamos el
# volumen por su etiqueta (o por el INFO_UF2.TXT que la Pico deja en la raíz).
# Antes estaba fija en D: y el script se quedaba esperando para siempre si el
# sistema la montaba en cualquier otra letra.
function Find-PicoDrive {
    $volume = Get-Volume -ErrorAction SilentlyContinue |
              Where-Object { $_.DriveLetter -and $_.FileSystemLabel -eq "RPI-RP2" } |
              Select-Object -First 1
    if ($volume) { return "$($volume.DriveLetter):\" }

    foreach ($v in (Get-Volume -ErrorAction SilentlyContinue | Where-Object { $_.DriveLetter })) {
        $root = "$($v.DriveLetter):\"
        if (Test-Path (Join-Path $root "INFO_UF2.TXT")) { return $root }
    }
    return $null
}

$target_drive = $null
while (-not $target_drive) {
    $target_drive = Find-PicoDrive
    if (-not $target_drive) {
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
