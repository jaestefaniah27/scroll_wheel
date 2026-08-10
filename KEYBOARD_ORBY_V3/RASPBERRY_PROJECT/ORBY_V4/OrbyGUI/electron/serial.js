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
    // Hay teclado hablando pero todavía no se ha presentado (ver _chaseHandshake).
    this.provisional = false;
    this._provisionalTimer = null;
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

      // Red por si el puerto se cuelga al abrirse. Tiene que dar tiempo a los
      // dos intentos de presentación de abajo (400 + 1200 + 1200), o se daría
      // por fallido justo antes de que llegue la buena.
      setTimeout(() => finish(false), 6000);

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
            this.provisional = false;
            clearTimeout(this._provisionalTimer);
            this.emit('disconnected');
            // Restart scanning
            this.startAutoScan();
          });

          this.port.on('error', (err) => {
            console.error(`Port error: ${err.message}`);
            this.emit('error', err.message);
          });

          // Presentación. Se pregunta DOS veces antes de descartar el puerto: el
          // primer ACK se pierde a menudo si el teclado acaba de enumerarse o
          // está ocupado (arranque de las pantallas, telemetría de la rueda), y
          // con un solo intento el puerto bueno se descartaba y la app se
          // quedaba en "Desconectado" hasta desenchufar y volver a enchufar.
          const HANDSHAKE_WAIT_MS = 1200;
          setTimeout(() => {
            this._sendRaw('ACK\n');
            setTimeout(() => {
              if (this.deviceInfo) { finish(true); return; }

              this._sendRaw('ACK\n');
              setTimeout(() => {
                if (this.deviceInfo) { finish(true); return; }
                // No es un Orby (o no contesta): se suelta el puerto.
                this.port?.close(() => {});
                this.port = null;
                finish(false);
              }, HANDSHAKE_WAIT_MS);
            }, HANDSHAKE_WAIT_MS);
          }, 400);
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
      const wasProvisional = this.provisional;
      this._parseDeviceInfo(line);
      this.provisional = false;
      clearTimeout(this._provisionalTimer);
      this.isConnected = true;
      this.stopAutoScan();
      // Si se había dado por conectado por telemetría, esto NO es un aviso
      // repetido: es la identidad de verdad sustituyendo a la provisional, y la
      // app tiene que enterarse para dejar de tratar al teclado como si llevara
      // un firmware sin páginas ni huellas.
      if (wasProvisional) console.log('Handshake recibido: identidad real del teclado');
      this.emit('connected', this.deviceInfo);
      return;
    }

    // Hay teclado —está mandando telemetría— pero su presentación no ha llegado.
    //
    // Antes se daba por conectado con una identidad inventada, sin FW ni
    // banderas de capacidades. Eso hacía que la app lo tratara como un firmware
    // antiguo: nada de GET_HASH, así que se descargaban TODOS los perfiles y
    // TODOS los iconos en cada conexión, y encima las funciones nuevas
    // desaparecían de la interfaz.
    //
    // Ahora se le vuelve a pedir la presentación y solo se cae en la identidad
    // inventada si no contesta en varios intentos: un teclado que habla pero no
    // se presenta sigue valiendo para lo básico, que es lo que cubría el apaño.
    if (!this.isConnected && !this.provisional
        && (line.startsWith('KEY_EV:') || line.startsWith('ENC:'))) {
      this.provisional = true;
      this._chaseHandshake();
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

  // Persigue la presentación del teclado cuando se sabe que está ahí pero no se
  // ha presentado. Reintenta el ACK unas cuantas veces y, si aun así calla, se
  // da por conectado con lo mínimo: es mejor una app en modo básico que una que
  // dice "Desconectado" con el teclado escribiendo.
  _chaseHandshake(attempt = 0) {
    const ATTEMPTS = 4;
    const EVERY_MS = 600;

    if (this.isConnected && !this.provisional) return;   // ya llegó
    if (!this.port?.isOpen) { this.provisional = false; return; }

    if (attempt < ATTEMPTS) {
      this._sendRaw('ACK\n');
      this._provisionalTimer = setTimeout(() => this._chaseHandshake(attempt + 1), EVERY_MS);
      return;
    }

    // Sin presentación: identidad mínima. No lleva ni FW ni banderas, así que la
    // app degradará a lo que sabía hacer un firmware antiguo.
    this.provisional = false;
    this.deviceInfo = {
      device: 'ORBY_V4', raw: 'ORBY_V4 (detectado por telemetría)',
      keys: 12, oleds: 10, port: this._portPath || null,
    };
    this.isConnected = true;
    this.stopAutoScan();
    console.log('El teclado no contesta al ACK: se sigue con identidad mínima');
    this.emit('connected', this.deviceInfo);
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
    this.provisional = false;
    clearTimeout(this._provisionalTimer);
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
    // Puerto abierto, teclado hablando, esperando a que se presente: abrir otro
    // puerto ahora solo tropezaría con el que ya tenemos cogido.
    if (this.provisional) return;
    
    this.isScanning = true;

    try {
      const ports = await this.listPorts();
      
      // El teclado ha ido cambiando de PID (el firmware lo sube cada vez que
      // cambia la estructura de un informe HID, porque Windows cachea el report
      // descriptor por VID/PID), así que aquí hay que aceptar todos los que
      // existen ahí fuera. `pids: null` significa "cualquier PID de ese VID".
      //
      // Cuando Raspberry Pi conceda un PID bajo su VID 0x2e8a hay que añadir la
      // entrada aquí Y cambiar ORBY_USB_VID/PID en src/usb_descriptors.c en el
      // mismo commit: si solo se toca el firmware, la app deja de encontrar el
      // teclado; si solo se toca esto, no pasa nada pero tampoco sirve.
      const KNOWN_IDS = [
        // VID de los ejemplos de TinyUSB. Lo han usado todos los firmwares hasta
        // el 4.1 incluido; se acepta entero para no dejar tirado a nadie.
        { vid: 'cafe', pids: null },
      ];

      const candidates = ports.filter(p => {
        const vid = p.vendorId?.toLowerCase();
        const pid = p.productId?.toLowerCase();
        // El VID tiene que coincidir siempre. Antes bastaba con que el PID
        // estuviera en la lista, viniera de quien viniera: eso abría puertos de
        // dispositivos ajenos que casualmente compartían PID.
        if (vid && KNOWN_IDS.some(k => k.vid === vid && (!k.pids || (pid && k.pids.includes(pid))))) return true;
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
