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
Write-Host "2. Poniendo el teclado en modo de carga..." -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

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

# El puerto CDC del teclado. Se busca por VID y no por PID: el firmware sube el
# PID cada vez que cambia la estructura de un informe HID, así que la lista de
# PIDs se quedaría corta a la primera. Mismo criterio que KNOWN_IDS en
# OrbyGUI/electron/serial.js.
function Find-OrbyPort {
    $dispositivos = Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
                    Where-Object { $_.PNPClass -eq 'Ports' -and $_.PNPDeviceID -like '*VID_CAFE*' }
    foreach ($d in $dispositivos) {
        if ($d.Name -match '\((COM\d+)\)') { return $Matches[1] }
    }
    return $null
}

# Desde el firmware 4.2 el teclado sabe reiniciarse solo en el cargador de la
# ROM: se le manda BOOTSEL por el puerto serie y sale como unidad RPI-RP2. Es lo
# mismo que hace OrbyGUI al actualizar (ver OrbyGUI/electron/firmware.js), y
# ahorra tener que desenchufar el cable con el botón hundido.
#
# Devuelve $true solo si el comando llegó a salir. Todo lo demás —firmware
# anterior al 4.2, teclado desenchufado, puerto ocupado— cae al camino de
# siempre: pedirlo a mano y esperar.
function Request-Bootloader {
    $com = Find-OrbyPort
    if (-not $com) {
        Write-Host "[ .. ] No se ve el teclado en ningún puerto COM." -ForegroundColor Gray
        return $false
    }

    Write-Host "[ .. ] Teclado en $com. Pidiéndole que entre en modo de carga..." -ForegroundColor Gray
    $puerto = $null
    try {
        # La velocidad da igual en un CDC (no hay UART detrás), pero el objeto la
        # exige. El firmware corta la línea con \r o con \n, así que WriteLine
        # vale tal cual.
        $puerto = New-Object System.IO.Ports.SerialPort $com, 115200
        $puerto.Open()
        $puerto.WriteLine("BOOTSEL")
        # Sin esta pausa se cierra el puerto antes de que el búfer salga por el
        # cable y el teclado no llega a ver el comando.
        Start-Sleep -Milliseconds 300
        return $true
    } catch {
        # El caso habitual: OrbyGUI está abierto y tiene el puerto cogido. No es
        # un fallo del script, y además desde la app se puede actualizar igual.
        Write-Host "[ !! ] No se pudo abrir $com ($($_.Exception.Message))." -ForegroundColor Yellow
        Write-Host "       Si OrbyGUI está abierto, ciérralo y vuelve a intentarlo." -ForegroundColor Yellow
        return $false
    } finally {
        if ($puerto -and $puerto.IsOpen) { $puerto.Close() }
        if ($puerto) { $puerto.Dispose() }
    }
}

# Puede estar ya en modo de carga (un intento anterior que se quedó a medias, o
# el usuario se adelantó con el botón). Entonces no hay teclado al que pedirle
# nada y preguntar solo serviría para perder tiempo.
$target_drive = Find-PicoDrive
if ($target_drive) {
    Write-Host "[ OK ] Ya estaba en modo de carga." -ForegroundColor Green
} else {
    $automatico = Request-Bootloader
    if (-not $automatico) {
        Write-Host "`nPor favor, mantén pulsado BOOTSEL y conecta la Pico al USB." -ForegroundColor Yellow
    }

    # El margen corto es para el camino automático (el teclado tarda un par de
    # segundos en reenumerar); el largo, para el manual, que incluye buscar el
    # botón y enchufar el cable.
    $limite = (Get-Date).AddSeconds($(if ($automatico) { 20 } else { 120 }))
    while (-not $target_drive -and (Get-Date) -lt $limite) {
        Start-Sleep -Milliseconds 500
        $target_drive = Find-PicoDrive
        if (-not $target_drive) { Write-Host "." -NoNewline -ForegroundColor Gray }
    }

    if (-not $target_drive) {
        Write-Host "`n[ ERROR ] El teclado no ha aparecido en modo de carga." -ForegroundColor Red
        if ($automatico) {
            Write-Host "          El comando BOOTSEL salió, así que puede ser un firmware anterior al 4.2." -ForegroundColor Red
            Write-Host "          Enchúfalo con el botón pulsado y vuelve a lanzar el script." -ForegroundColor Red
        }
        exit 1
    }
}

Write-Host "`n=========================================" -ForegroundColor Cyan
Write-Host "3. Copiando el firmware a $target_drive" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

try {
    Copy-Item -Path $uf2_path -Destination $target_drive
    Write-Host "[ OK ] ¡Flasheo completado! La Pico se reiniciará sola." -ForegroundColor Green
} catch {
    # La Pico se reinicia en cuanto recibe el último bloque, así que la unidad
    # puede desaparecer bajo los pies de Windows antes de que se cierre el
    # fichero. Eso no es un fallo: el fallo es que la unidad siga ahí después.
    Start-Sleep -Milliseconds 500
    if (Find-PicoDrive) {
        Write-Host "[ ERROR ] Fallo al transferir el firmware a la unidad $target_drive." -ForegroundColor Red
        Write-Host "          $($_.Exception.Message)" -ForegroundColor Red
        exit 1
    }
    Write-Host "[ OK ] ¡Flasheo completado! La Pico se reiniciará sola." -ForegroundColor Green
}
