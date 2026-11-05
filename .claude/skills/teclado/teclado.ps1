# Habla con el teclado ORBY por su puerto serie.
#
#   powershell -ExecutionPolicy Bypass -File teclado.ps1 -Version
#   powershell -ExecutionPolicy Bypass -File teclado.ps1 -Flashear <ruta\al.uf2>
#
# -Version   pregunta con ACK y escribe la presentación (ORBY_V4:FW=X.Y:...).
# -Flashear  le manda BOOTSEL (firmware 4.2 o posterior), espera la unidad RPI-RP2, copia
#            el .uf2 y, cuando vuelve, lee la versión para confirmarlo.
#
# El puerto se busca en el registro y no con Get-CimInstance, que en este PC falla con
# «Clase no válida». Sirve cualquier PID con el VID CAFE: el firmware cambia el PID cuando
# cambia un informe HID.
param([switch]$Version, [string]$Flashear)
$ErrorActionPreference = 'Stop'

function Find-OrbyPorts {
    $vivos = [System.IO.Ports.SerialPort]::GetPortNames()
    $raiz = 'HKLM:\SYSTEM\CurrentControlSet\Enum\USB'
    $puertos = foreach ($dev in Get-ChildItem $raiz -ErrorAction SilentlyContinue | Where-Object { $_.PSChildName -like 'VID_CAFE*' }) {
        foreach ($inst in Get-ChildItem $dev.PSPath -ErrorAction SilentlyContinue) {
            $p = (Get-ItemProperty (Join-Path $inst.PSPath 'Device Parameters') -ErrorAction SilentlyContinue).PortName
            # El registro recuerda puertos de enchufes antiguos: solo valen los que existen ahora.
            if ($p -and $vivos -contains $p) { $p }
        }
    }
    $puertos | Select-Object -Unique
}

function Find-PicoDrive {
    foreach ($d in [System.IO.DriveInfo]::GetDrives()) {
        if ($d.IsReady -and (Test-Path (Join-Path $d.RootDirectory 'INFO_UF2.TXT'))) { return $d.RootDirectory.FullName }
    }
    $null
}

# Abre el puerto, manda una línea y devuelve lo que conteste en `ms` milisegundos.
function Send-Line($com, $linea, $ms = 1500) {
    $sp = New-Object System.IO.Ports.SerialPort $com, 115200
    $sp.DtrEnable = $true; $sp.RtsEnable = $true; $sp.ReadTimeout = 200
    try {
        $sp.Open()
        Start-Sleep -Milliseconds 100
        $sp.DiscardInBuffer()
        $sp.Write("$linea`n")
        $fin = (Get-Date).AddMilliseconds($ms); $txt = ''
        while ((Get-Date) -lt $fin) {
            try { $txt += $sp.ReadExisting() } catch {}
            Start-Sleep -Milliseconds 50
        }
        $txt
    } finally { if ($sp.IsOpen) { $sp.Close() }; $sp.Dispose() }
}

function Get-Presentacion {
    $ports = @(Find-OrbyPorts)
    if (-not $ports) { Write-Host 'No se ve el teclado en ningún puerto COM.'; return $null }
    foreach ($com in $ports) {
        try {
            # El primer ACK se pierde a menudo si el teclado acaba de enumerarse: dos intentos.
            foreach ($i in 1..2) {
                $r = Send-Line $com 'ACK'
                $l = ($r -split "`r?`n") | Where-Object { $_ -like 'ORBY_V4:*' } | Select-Object -First 1
                if ($l) { return @{ Puerto = $com; Linea = $l } }
            }
            Write-Host "$com no contesta al ACK."
        } catch [System.UnauthorizedAccessException] {
            Write-Host "$com está ocupado por otro programa (casi siempre Orvy). Ciérralo, también desde la bandeja, y repite."
        }
    }
    $null
}

if ($Version) {
    $p = Get-Presentacion
    if (-not $p) { exit 1 }
    Write-Host "$($p.Puerto): $($p.Linea)"
    exit 0
}

if ($Flashear) {
    if (-not (Test-Path $Flashear)) { Write-Host "No existe $Flashear"; exit 1 }
    $drive = Find-PicoDrive
    if (-not $drive) {
        $com = @(Find-OrbyPorts) | Select-Object -First 1
        if (-not $com) { Write-Host 'No se ve el teclado. Enchúfalo con BOOTSEL pulsado y repite.'; exit 1 }
        try { [void](Send-Line $com 'BOOTSEL' 300) }
        catch { Write-Host "No se pudo abrir $com ($($_.Exception.Message)). Cierra Orvy y repite."; exit 1 }
        $limite = (Get-Date).AddSeconds(20)
        while (-not $drive -and (Get-Date) -lt $limite) { Start-Sleep -Milliseconds 500; $drive = Find-PicoDrive }
        if (-not $drive) { Write-Host 'No ha aparecido RPI-RP2 (¿firmware anterior al 4.2?). Enchúfalo con BOOTSEL pulsado.'; exit 1 }
    }
    Write-Host "Copiando a $drive ..."
    # La Pico se reinicia al recibir el último bloque; si la unidad desaparece a medias no es un fallo.
    try { Copy-Item $Flashear $drive } catch { Start-Sleep -Milliseconds 500; if (Find-PicoDrive) { throw } }
    $limite = (Get-Date).AddSeconds(15); $p = $null
    while (-not $p -and (Get-Date) -lt $limite) { Start-Sleep -Seconds 1; if (Find-OrbyPorts) { $p = Get-Presentacion } }
    if ($p) { Write-Host "Hecho. $($p.Puerto): $($p.Linea)" } else { Write-Host 'Copiado, pero el teclado no ha contestado aún. Mira la versión con -Version.' }
    exit 0
}

Write-Host 'Uso: teclado.ps1 -Version | -Flashear <ruta\al.uf2>'
exit 1
