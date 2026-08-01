const EventEmitter = require('events');

class OrbySerial extends EventEmitter {
  constructor() {
    super();
    this.port = null;
    this.parser = null;
    this.deviceInfo = null;
    this.scanInterval = null;
    this.isConnected = false;
    this.SerialPort = null;
    this.ReadlineParser = null;
    this._buffer = '';
  }

  async _loadSerialPort() {
    if (this.SerialPort) return true;
    try {
      const sp = require('serialport');
      this.SerialPort = sp.SerialPort;
      this.ReadlineParser = sp.ReadlineParser;
      return true;
    } catch (err) {
      console.error('Failed to load serialport module:', err.message);
      this.emit('error', `serialport module not available: ${err.message}`);
      return false;
    }
  }

  async listPorts() {
    if (!(await this._loadSerialPort())) return [];
    try {
      const { SerialPort } = require('serialport');
      const ports = await SerialPort.list();
      return ports;
    } catch (err) {
      console.error('Error listing ports:', err);
      return [];
    }
  }

  async tryConnect(portPath) {
    if (!(await this._loadSerialPort())) return false;

    return new Promise((resolve) => {
      let isResolved = false;
      const finish = (result) => {
        if (!isResolved) {
          isResolved = true;
          resolve(result);
        }
      };

      // Timeout just in case the port hangs during open
      setTimeout(() => finish(false), 3000);

      try {
        this.port = new this.SerialPort({
          path: portPath,
          baudRate: 115200,
          autoOpen: false,
        });

        this.port.open((err) => {
          if (err) {
            console.log(`Failed to open ${portPath}: ${err.message}`);
            this.port = null;
            finish(false);
            return;
          }

          // Force DTR/RTS to true (TinyUSB needs this to accept RX sometimes)
          this.port.set({ dtr: true, rts: true }, () => {});

          this._buffer = '';
          this._portPath = portPath;

          // Set up line-based data parsing
          this.port.on('data', (data) => {
            this._buffer += data.toString();
            let lines = this._buffer.split('\n');
            this._buffer = lines.pop() || '';
            for (const line of lines) {
              const trimmed = line.trim();
              if (trimmed.length > 0) {
                this._handleLine(trimmed);
              }
            }
          });

          this.port.on('close', () => {
            console.log(`Port ${portPath} closed`);
            this.isConnected = false;
            this.deviceInfo = null;
            this.port = null;
            this.emit('disconnected');
            // Restart scanning
            this.startAutoScan();
          });

          this.port.on('error', (err) => {
            console.error(`Port error: ${err.message}`);
            this.emit('error', err.message);
          });

          // Send handshake
          setTimeout(() => {
            this._sendRaw('ACK\n');
            // Wait for response
            setTimeout(() => {
              if (this.deviceInfo) {
                finish(true);
              } else {
                // Not an Orby device, close
                this.port?.close(() => {});
                this.port = null;
                finish(false);
              }
            }, 1500);
          }, 500);
        });
      } catch (err) {
        console.error(`Error connecting to ${portPath}: ${err.message}`);
        finish(false);
      }
    });
  }

  _handleLine(line) {
    // Check for handshake response
    if (line.startsWith('ORBY_V4:')) {
      this._parseDeviceInfo(line);
      this.isConnected = true;
      this.stopAutoScan();
      this.emit('connected', this.deviceInfo);
      return;
    }

    // Fallback: If we missed the ACK but receive Orby telemetry, assume connected
    if (!this.isConnected && (line.startsWith('KEY_EV:') || line.startsWith('ENC:'))) {
      this.deviceInfo = {
        device: 'ORBY_V4', raw: 'ORBY_V4 (detectado por telemetría)',
        keys: 12, oleds: 10, port: this._portPath || null,
      };
      this.isConnected = true;
      this.stopAutoScan();
      this.emit('connected', this.deviceInfo);
    }

    // Check for command responses
    if (line.startsWith('PROFILE:OK:') ||
        line.startsWith('TIMEOUT:OK:') ||
        line.startsWith('OLED:OK:') ||
        line.startsWith('MACRO:OK:')) {
      this.emit('data', line);
      return;
    }

    // Forward telemetry
    this.emit('data', line);
  }

  _parseDeviceInfo(line) {
    // Format: ORBY_V4:FW=1.0:KEYS=12:OLEDS=10:ENCODERS=2:MODE=NORMAL
    const parts = line.split(':');
    const info = { raw: line };
    for (const part of parts) {
      if (part.includes('=')) {
        const [key, val] = part.split('=');
        info[key.toLowerCase()] = val;
      }
    }
    info.device = parts[0] || 'ORBY_V4';
    info.port = this._portPath || null;
    this.deviceInfo = info;
  }

  _sendRaw(data) {
    if (this.port && this.port.isOpen) {
      this.port.write(data, (err) => {
        if (err) {
          console.error('Write error:', err.message);
        }
      });
    }
  }

  sendCommand(cmd) {
    if (!this.isConnected || !this.port) return false;
    const data = cmd.endsWith('\n') ? cmd : cmd + '\n';
    this._sendRaw(data);
    return true;
  }

  disconnect() {
    if (this.port && this.port.isOpen) {
      this.port.close(() => {});
    }
    this.port = null;
    this.isConnected = false;
    this.deviceInfo = null;
  }

  getDeviceInfo() {
    return this.deviceInfo;
  }

  getStatus() {
    if (this.isConnected) return 'connected';
    if (this.scanInterval) return 'searching';
    return 'disconnected';
  }

  startAutoScan() {
    if (this.scanInterval) return;
    this.emit('searching');
    this._doScan();
    this.scanInterval = setInterval(() => this._doScan(), 3000);
  }

  stopAutoScan() {
    if (this.scanInterval) {
      clearInterval(this.scanInterval);
      this.scanInterval = null;
    }
  }

  // El botón "Reconectar" necesita forzar un intento nuevo YA, no esperar al
  // próximo tick del intervalo de 3s. Si el teclado ya estaba enchufado al
  // abrir la app, el primer handshake pudo fallar (p.ej. el puerto entrega
  // telemetría a medio escribir) y startAutoScan() por sí solo no hace nada
  // porque el intervalo ya está corriendo (early return de arriba): por eso
  // el botón parecía no responder. Aquí se cierra cualquier puerto a medio
  // abrir y se relanza el escaneo desde cero.
  forceRescan() {
    this.stopAutoScan();
    this.isScanning = false;
    if (this.port) {
      try {
        this.port.removeAllListeners('close');
        if (this.port.isOpen) this.port.close(() => {});
      } catch {}
      this.port = null;
    }
    this.startAutoScan();
  }

  async _doScan() {
    if (this.isConnected || this.isScanning) {
      if (this.isConnected) this.stopAutoScan();
      return;
    }
    
    this.isScanning = true;

    try {
      const ports = await this.listPorts();
      
      // 4006 es el PID del firmware 2.0 (scroll de alta resolución); 4005 se
      // mantiene para no dejar tirados a los teclados sin actualizar.
      const KNOWN_PIDS = ['4005', '4006'];

      const candidates = ports.filter(p => {
        if (p.vendorId && p.vendorId.toLowerCase() === 'cafe') return true;
        if (p.productId && KNOWN_PIDS.includes(p.productId.toLowerCase())) return true;
        if (p.manufacturer && p.manufacturer.toLowerCase().includes('orby')) return true;
        // Se ha eliminado el fallback genérico (starts with COM) para evitar cuelgues nativos
        return false;
      });

      for (const candidate of candidates) {
        if (this.isConnected) break;
        console.log(`Trying port: ${candidate.path} (${candidate.manufacturer || 'Unknown'})`);
        const success = await this.tryConnect(candidate.path);
        if (success) {
          console.log(`Connected to Orby V4 on ${candidate.path}`);
          break;
        }
      }
    } finally {
      this.isScanning = false;
    }
  }
}

module.exports = { OrbySerial };
