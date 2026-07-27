const EventEmitter = require('events');
const { spawn } = require('child_process');

// Detector de la ventana en primer plano de Windows.
//
// Se resuelve con un PowerShell de vida larga que llama a GetForegroundWindow
// por P/Invoke y emite una línea JSON cada vez que cambia la ventana activa.
// La alternativa habitual (paquetes como active-win) arrastra un módulo nativo
// que hay que recompilar por cada versión de Electron; esto no depende de nada.
const PS_SCRIPT = `
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Add-Type @"
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class OrbyFg {
  [DllImport("user32.dll")]
  public static extern IntPtr GetForegroundWindow();
  [DllImport("user32.dll")]
  public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint processId);
  [DllImport("user32.dll", CharSet = CharSet.Unicode)]
  public static extern int GetWindowText(IntPtr hWnd, StringBuilder text, int count);
}
"@

$lastHandle = [IntPtr]::Zero
$lastTitle  = ''

while ($true) {
  try {
    $handle = [OrbyFg]::GetForegroundWindow()
    if ($handle -ne [IntPtr]::Zero) {
      $buffer = New-Object System.Text.StringBuilder 512
      [void][OrbyFg]::GetWindowText($handle, $buffer, $buffer.Capacity)
      $title = $buffer.ToString()

      if ($handle -ne $lastHandle -or $title -ne $lastTitle) {
        $lastHandle = $handle
        $lastTitle  = $title

        # $pid es una variable automática de solo lectura: usamos otro nombre.
        $procId = 0
        [void][OrbyFg]::GetWindowThreadProcessId($handle, [ref]$procId)

        $name = ''
        try { $name = (Get-Process -Id $procId -ErrorAction Stop).ProcessName } catch { }

        $payload = [ordered]@{ process = $name; title = $title }
        Write-Output ($payload | ConvertTo-Json -Compress)
      }
    }
  } catch { }
  Start-Sleep -Milliseconds 400
}
`;

class ForegroundWatcher extends EventEmitter {
  constructor() {
    super();
    this.child = null;
    this.current = null;
    this._buffer = '';
  }

  get available() {
    return process.platform === 'win32';
  }

  start() {
    if (this.child || !this.available) return this.available;

    // -EncodedCommand evita por completo los problemas de comillas al pasar un
    // script multilínea por la línea de órdenes.
    const encoded = Buffer.from(PS_SCRIPT, 'utf16le').toString('base64');

    try {
      this.child = spawn('powershell.exe',
        ['-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-EncodedCommand', encoded],
        { windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
    } catch (err) {
      this.emit('error', `No se pudo iniciar el detector: ${err.message}`);
      return false;
    }

    this.child.stdout.on('data', (chunk) => this._onData(chunk));
    this.child.stderr.on('data', (chunk) => {
      const text = chunk.toString().trim();
      // PowerShell escribe una cabecera CLIXML en stderr al arrancar; no es un error.
      if (text && !text.startsWith('#< CLIXML')) this.emit('error', text);
    });

    this.child.on('exit', (code) => {
      this.child = null;
      if (code !== 0 && code !== null) this.emit('error', `Detector terminado (código ${code})`);
    });

    return true;
  }

  stop() {
    if (!this.child) return;
    const child = this.child;
    this.child = null;
    try { child.kill(); } catch { /* ya no existe */ }
  }

  _onData(chunk) {
    this._buffer += chunk.toString();
    const lines = this._buffer.split('\n');
    this._buffer = lines.pop() || '';

    for (const line of lines) {
      const trimmed = line.trim();
      if (!trimmed.startsWith('{')) continue;
      try {
        const info = JSON.parse(trimmed);
        this.current = info;
        this.emit('change', info);
      } catch { /* línea parcial o ruido */ }
    }
  }
}

module.exports = { ForegroundWatcher };
