# Publica la versión del repositorio: firmware y OrbyGUI, de una vez.
#
#   .\tools\publicar.ps1              # ENSAYO: compila todo y lo deja en build\publicar\
#   .\tools\publicar.ps1 -Publicar    # además etiqueta, crea las releases y lo comprueba
#
# Hace a mano lo que dicen docs/COMPATIBILIDAD.md y OrbyGUI/docs/PUBLICACION.md,
# porque GitHub Actions está bloqueado en esta cuenta. Lo importante, por lo que
# no conviene improvisarlo:
#
# - La app se actualiza sola leyendo `latest.json` de `releases/latest`. Una
#   release sin ese fichero, o sin la firma del instalador, deja mudo al
#   actualizador de todas las copias instaladas sin que nadie se entere.
# - Por eso la de firmware va como prerelease (no cuenta para `latest`) y se
#   publica ANTES que la de la app, que tiene que quedar como la última.
# - La firma sale de %USERPROFILE%\.tauri\orby.key, que no está en el
#   repositorio y no se puede sustituir.
#
# Se puede volver a lanzar si algo falla a medias: lo que ya esté publicado
# (etiquetas, releases, ficheros subidos) se aprovecha en vez de repetirse.
#
# Necesita: cmake + ninja + el toolchain de la Pico (lo mismo que flash.ps1),
# Node y Rust para OrbyGUI, y GH_TOKEN en el entorno para publicar.

param(
    [switch]$Publicar
)

$ErrorActionPreference = 'Stop'
$Repo = 'jaestefaniah27/scroll_wheel'
$Raiz = Split-Path -Parent $PSScriptRoot          # ORBY_V4
$Gui  = Join-Path $Raiz 'OrbyGUI'
$Salida = Join-Path $Raiz 'build\publicar'
$Clave = Join-Path $env:USERPROFILE '.tauri\orby.key'
$Utf8SinBom = New-Object System.Text.UTF8Encoding $false

function Paso($texto) { Write-Host "`n== $texto" -ForegroundColor Cyan }
function Ok($texto)   { Write-Host "   $texto" -ForegroundColor Green }
function Nota($texto) { Write-Host "   $texto" -ForegroundColor Gray }
function Parar($texto) { Write-Host "`n[ PARADO ] $texto" -ForegroundColor Red; exit 1 }

function Comprobar-Salida($que) {
    if ($LASTEXITCODE -ne 0) { Parar "$que ha fallado (código $LASTEXITCODE)." }
}

# --- GitHub ------------------------------------------------------------------
function Cabeceras {
    @{
        Authorization          = "Bearer $env:GH_TOKEN"
        Accept                 = 'application/vnd.github+json'
        'X-GitHub-Api-Version' = '2022-11-28'
    }
}

# PowerShell 5.1 manda las cadenas en la codificación del sistema: las tildes de
# las notas llegarían rotas. Se manda el JSON ya convertido a bytes UTF-8.
function GitHub($metodo, $ruta, $cuerpo) {
    $params = @{ Method = $metodo; Uri = "https://api.github.com$ruta"; Headers = (Cabeceras) }
    if ($cuerpo) {
        $params.Body = $Utf8SinBom.GetBytes(($cuerpo | ConvertTo-Json -Depth 8))
        $params.ContentType = 'application/json; charset=utf-8'
    }
    Invoke-RestMethod @params
}

function Release-Existente($tag) {
    try { GitHub 'Get' "/repos/$Repo/releases/tags/$tag" $null }
    catch {
        if ($_.Exception.Response -and [int]$_.Exception.Response.StatusCode -eq 404) { return $null }
        throw
    }
}

function Subir-Fichero($release, $ruta, $tipo) {
    $nombre = [IO.Path]::GetFileName($ruta)
    if ($release.assets | Where-Object { $_.name -eq $nombre }) { Nota "$nombre ya estaba subido"; return }
    $url = ($release.upload_url -replace '\{.*\}', '') + "?name=$([uri]::EscapeDataString($nombre))"
    Invoke-RestMethod -Method Post -Uri $url -Headers (Cabeceras) -InFile $ruta -ContentType $tipo | Out-Null
    Ok "subido $nombre"
}

# --- Versiones -----------------------------------------------------------------
Paso 'Versiones'
$vApp   = (Get-Content (Join-Path $Gui 'package.json') -Raw | ConvertFrom-Json).version
$vTauri = (Get-Content (Join-Path $Gui 'src-tauri\tauri.conf.json') -Raw | ConvertFrom-Json).version
$vCargo = (Select-String -Path (Join-Path $Gui 'src-tauri\Cargo.toml') -Pattern '^version = "(.+)"' |
           Select-Object -First 1).Matches[0].Groups[1].Value
$cab = Get-Content (Join-Path $Raiz 'include\orby_version.h') -Raw
$vFw = '{0}.{1}' -f ([regex]::Match($cab, 'ORBY_FW_MAJOR (\d+)').Groups[1].Value),
                    ([regex]::Match($cab, 'ORBY_FW_MINOR (\d+)').Groups[1].Value)

if ($vApp -ne $vTauri -or $vApp -ne $vCargo) {
    Parar "Las versiones de la app no coinciden: package.json $vApp, tauri.conf.json $vTauri, Cargo.toml $vCargo."
}
$compat = Get-Content (Join-Path $Gui 'src\compat.js') -Raw
if ($compat -notmatch "FW_RECOMMENDED = '$([regex]::Escape($vFw))'") {
    Parar "compat.js no recomienda el firmware $vFw que dice orby_version.h."
}
$tagApp = "v$vApp"
$tagFw  = "fw-v$vFw"
Ok "OrbyGUI $vApp ($tagApp) · firmware $vFw ($tagFw)"

$notasApp = Join-Path $Gui "docs\notas-v$vApp.md"
$notasFw  = Join-Path $Raiz "docs\notas-fw-$vFw.md"
foreach ($n in @($notasApp, $notasFw)) { if (-not (Test-Path $n)) { Parar "Faltan las notas: $n" } }

# --- Estado del repositorio --------------------------------------------------------
Paso 'Repositorio'
Push-Location $Raiz
try {
    $rama = (git rev-parse --abbrev-ref HEAD).Trim()
    if ($rama -ne 'main') { Parar "Estás en la rama '$rama'. Se publica desde main." }
    if (git status --porcelain) { Parar 'Hay cambios sin commitear: lo publicado tiene que ser exactamente lo del repositorio.' }
    git fetch origin main --quiet; Comprobar-Salida 'git fetch'
    if ((git rev-parse HEAD).Trim() -ne (git rev-parse origin/main).Trim()) {
        Parar 'main no coincide con origin/main: haz pull o push antes de publicar.'
    }
} finally { Pop-Location }
Ok 'main limpio y al día con GitHub'

if (-not (Test-Path $Clave)) { Parar "No encuentro la clave de firma en $Clave. Sin ella el actualizador rechazaría el instalador." }
Ok 'clave de firma encontrada'

if ($Publicar) {
    if (-not $env:GH_TOKEN) { Parar 'Falta GH_TOKEN en el entorno: hace falta para crear las releases.' }
    $previa = Release-Existente $tagApp
    if ($previa -and -not $previa.draft) {
        $completa = ($previa.assets | Where-Object { $_.name -eq 'latest.json' })
        if ($completa) { Parar "La release $tagApp ya está publicada y completa: sube la versión de la app." }
    }
}

New-Item -ItemType Directory -Force -Path $Salida | Out-Null

# --- Firmware ------------------------------------------------------------------------
Paso "Firmware $vFw"
Push-Location $Raiz
try {
    if (-not (Test-Path 'build\CMakeCache.txt')) { cmake -B build -G Ninja; Comprobar-Salida 'cmake' }   # como flash.ps1
    cmake --build build; Comprobar-Salida 'La compilación del firmware'
} finally { Pop-Location }
$uf2 = Join-Path $Salida "ORBY_V4-fw-$vFw.uf2"
Copy-Item (Join-Path $Raiz 'build\ORBY_V4.uf2') $uf2 -Force
$shaFw = (Get-FileHash $uf2 -Algorithm SHA256).Hash.ToLower()
[IO.File]::WriteAllText("$uf2.sha256", "$shaFw  $([IO.Path]::GetFileName($uf2))`n", $Utf8SinBom)
Ok "$([IO.Path]::GetFileName($uf2)) · SHA256 $shaFw"

# --- App -------------------------------------------------------------------------------
Paso "OrbyGUI $vApp (firmada)"
Push-Location $Gui
try {
    $env:TAURI_SIGNING_PRIVATE_KEY = $Clave
    $env:TAURI_SIGNING_PRIVATE_KEY_PASSWORD = ''
    npm run tauri:build; Comprobar-Salida 'npm run tauri:build'
} finally {
    Remove-Item Env:\TAURI_SIGNING_PRIVATE_KEY -ErrorAction SilentlyContinue
    Remove-Item Env:\TAURI_SIGNING_PRIVATE_KEY_PASSWORD -ErrorAction SilentlyContinue
    Pop-Location
}
$nsis = Join-Path $Gui 'src-tauri\target\release\bundle\nsis'
$exeOrigen = Get-ChildItem $nsis -Filter "*_${vApp}_x64-setup.exe" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $exeOrigen) { Parar "No aparece el instalador *_${vApp}_x64-setup.exe en $nsis." }
if (-not (Test-Path "$($exeOrigen.FullName).sig")) {
    Parar 'El instalador ha salido sin .sig: la firma no se aplicó y la release nacería rota.'
}
if ($exeOrigen.Name -match '\s') { Parar "El nombre del instalador lleva espacios ($($exeOrigen.Name)): GitHub los cambiaría y la URL de latest.json no casaría." }
$exe = Join-Path $Salida $exeOrigen.Name
Copy-Item $exeOrigen.FullName $exe -Force
Copy-Item "$($exeOrigen.FullName).sig" "$exe.sig" -Force
$shaApp = (Get-FileHash $exe -Algorithm SHA256).Hash.ToLower()
Ok "$($exeOrigen.Name) · SHA256 $shaApp"

# latest.json: lo que lee el actualizador. Sin BOM: con él, el JSON no se lee.
$latest = [ordered]@{
    version  = $vApp
    notes    = "OrbyGUI $vApp. Detalles: https://github.com/$Repo/releases/tag/$tagApp"
    pub_date = (Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ')
    platforms = [ordered]@{
        'windows-x86_64' = [ordered]@{
            signature = (Get-Content "$exe.sig" -Raw).Trim()
            url       = "https://github.com/$Repo/releases/download/$tagApp/$($exeOrigen.Name)"
        }
    }
}
$latestRuta = Join-Path $Salida 'latest.json'
[IO.File]::WriteAllText($latestRuta, ($latest | ConvertTo-Json -Depth 5), $Utf8SinBom)
Ok 'latest.json compuesto'

# --- Notas ---------------------------------------------------------------------------
$tabla = [regex]::Match((Get-Content (Join-Path $Raiz 'docs\COMPATIBILIDAD.md') -Raw),
                        '(?s)<!-- tabla:inicio -->\s*(.*?)\s*<!-- tabla:fin -->').Groups[1].Value
$cuerpoFw = (Get-Content $notasFw -Raw) + @"

## Instalación

Lo más cómodo: OrbyGUI avisa al conectar el teclado y lo instala desde Ajustes, sin tocar el botón BOOTSEL.

A mano: conecta el teclado con el botón **BOOTSEL** pulsado, aparece una unidad ``RPI-RP2`` y copias dentro ``$([IO.Path]::GetFileName($uf2))``.

    SHA256: $shaFw

## Compatibilidad con OrbyGUI

$tabla
"@
$cuerpoApp = (Get-Content $notasApp -Raw) + @"

## Instalación

Si ya tienes OrbyGUI 1.0.2 o posterior, se actualiza sola.

Windows mostrará el aviso de SmartScreen: el instalador todavía no va firmado para Windows (ver ``docs/PUBLICACION.md``). Para continuar: **Más información → Ejecutar de todas formas**.

Antes de ejecutarlo, comprueba que el fichero es el que publicamos:

    Get-FileHash .\$($exeOrigen.Name) -Algorithm SHA256

    SHA256: $shaApp
"@
[IO.File]::WriteAllText((Join-Path $Salida "notas-$tagFw.md"), $cuerpoFw, $Utf8SinBom)
[IO.File]::WriteAllText((Join-Path $Salida "notas-$tagApp.md"), $cuerpoApp, $Utf8SinBom)

if (-not $Publicar) {
    Paso 'Ensayo terminado'
    Nota "Todo lo que se publicaría está en $Salida"
    Nota 'Pruébalo (instala el .exe, flashea el .uf2) y, si está bien:  .\tools\publicar.ps1 -Publicar'
    exit 0
}

# --- Etiquetas -------------------------------------------------------------------------
Paso 'Etiquetas'
Push-Location $Raiz
try {
    foreach ($t in @(@($tagFw, "Firmware $vFw"), @($tagApp, "OrbyGUI $vApp"))) {
        if (git tag --list $t[0]) { Nota "$($t[0]) ya existía" } else { git tag -a $t[0] -m $t[1]; Comprobar-Salida "git tag $($t[0])" }
    }
    git push origin $tagFw $tagApp; Comprobar-Salida 'git push de las etiquetas'
} finally { Pop-Location }
Ok "$tagFw y $tagApp en GitHub"

# --- Release del firmware: PRERELEASE, y antes que la de la app --------------------------
Paso "Release $tagFw (prerelease)"
$relFw = Release-Existente $tagFw
if (-not $relFw) {
    $relFw = GitHub 'Post' "/repos/$Repo/releases" ([ordered]@{
        tag_name = $tagFw; name = "Firmware $vFw"; body = $cuerpoFw
        prerelease = $true; make_latest = 'false'
    })
}
Subir-Fichero $relFw $uf2 'application/octet-stream'
Subir-Fichero $relFw "$uf2.sha256" 'text/plain'
Ok $relFw.html_url

# --- Release de la app: la última, NO prerelease -------------------------------------------
Paso "Release $tagApp"
$relApp = Release-Existente $tagApp
if (-not $relApp) {
    # Como borrador hasta que estén los tres ficheros: así el actualizador nunca
    # ve una release a medias como la última.
    $relApp = GitHub 'Post' "/repos/$Repo/releases" ([ordered]@{
        tag_name = $tagApp; name = "OrbyGUI $vApp"; body = $cuerpoApp
        draft = $true; prerelease = $false
    })
}
Subir-Fichero $relApp $exe 'application/octet-stream'
Subir-Fichero $relApp "$exe.sig" 'text/plain'
Subir-Fichero $relApp $latestRuta 'application/json'
$relApp = GitHub 'Patch' "/repos/$Repo/releases/$($relApp.id)" ([ordered]@{
    draft = $false; prerelease = $false; make_latest = 'true'
})
Ok $relApp.html_url

# --- Comprobar lo que verá el actualizador -------------------------------------------------
Paso 'Comprobación'
$visto = $null
for ($i = 0; $i -lt 6 -and -not $visto; $i++) {
    try {
        $leido = Invoke-RestMethod "https://github.com/$Repo/releases/latest/download/latest.json" -Headers @{ 'Cache-Control' = 'no-cache' }
        if ($leido.version -eq $vApp) { $visto = $leido } else { Start-Sleep 5 }
    } catch { Start-Sleep 5 }
}
if (-not $visto) {
    Parar "releases/latest/download/latest.json no devuelve la $vApp todavía. Revisa en GitHub que $tagApp sea la última y no esté marcada como prerelease."
}
Ok "el actualizador ya ve la $vApp"
Write-Host "`n[ OK ] Publicados el firmware $vFw y OrbyGUI $vApp." -ForegroundColor Green
