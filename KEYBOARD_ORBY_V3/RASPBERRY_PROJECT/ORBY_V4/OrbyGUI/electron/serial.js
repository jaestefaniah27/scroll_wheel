const EventEmitter = require('events');
const { log } = require('./log');

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
    // Señales de vida de la conexión (ver _startWatchdog).
    this._watchdog = null;
    this._lastLineAt = 0;
    this._pingAt = 0;
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

    // Nunca se prueba un puerto teniendo otro a medias: dejarlo abierto sería
    // quedarse su handle para siempre y estrellarse contra "Access denied" en
    // todos los intentos siguientes.
    if (this.port && !this.isConnected) this._releasePort('quedaba un intento anterior sin cerrar');

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
      //
      // Suelta el puerto además de rendirse: darlo por fallido y dejarlo abierto
      // deja un puerto huérfano que nadie va a cerrar nunca, con el agravante de
      // que sigue cogido. El intento siguiente choca contra él —"Access denied"—
      // y la app no vuelve a encontrar el teclado por mucho que se enchufe.
      setTimeout(() => {
        if (isResolved || this.isConnected) return;
        this._releasePort('el puerto no contestó al abrirse');
        finish(false);
      }, 6000);

      try {
        this.port = new this.SerialPort({
          path: portPath,
          baudRate: 115200,
          autoOpen: false,
        });

        this.port.open((err) => {
          if (err) {
            log(`[serie] no se pudo abrir ${portPath}: ${err.message}`);
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
            log(`[serie] puerto ${portPath} cerrado`);
            this.isConnected = false;
            this.deviceInfo = null;
            this.port = null;
            this.provisional = false;
            clearTimeout(this._provisionalTimer);
            this._stopWatchdog();
            this.emit('disconnected');
            // Restart scanning
            this.startAutoScan();
          });

          this.port.on('error', (err) => {
            log(`[serie] error del puerto: ${err.message}`);
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
                this._releasePort('no se ha presentado tras dos ACK');
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
    // Cualquier línea vale como señal de vida (ver _startWatchdog).
    this._lastLineAt = Date.now();

    // Check for handshake response
    if (line.startsWith('ORBY_V4:')) {
      // El vigilante pregunta con ACK cada tanto, así que esta línea llega
      // también con la conexión ya hecha: entonces no es una conexión nueva y
      // volver a anunciarla haría a la app resincronizarse cada pocos segundos.
      const yaConectado = this.isConnected && this.deviceInfo?.raw === line;
      const wasProvisional = this.provisional;
      this._parseDeviceInfo(line);
      this.provisional = false;
      clearTimeout(this._provisionalTimer);
      this.isConnected = true;
      this.stopAutoScan();
      this._startWatchdog();
      if (yaConectado) return;
      // Si se había dado por conectado por telemetría, esto NO es un aviso
      // repetido: es la identidad de verdad sustituyendo a la provisional, y la
      // app tiene que enterarse para dejar de tratar al teclado como si llevara
      // un firmware sin páginas ni huellas.
      if (wasProvisional) log('[serie] presentación recibida: identidad real del teclado');
      log(`[serie] teclado detectado: fw ${this.deviceInfo.fw ?? '?'} en ${this.deviceInfo.port ?? '?'}`);
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

  // Cierra y suelta el puerto que se estaba probando. Todo intento fallido pasa
  // por aquí: un puerto abierto que ya no se va a usar y que nadie recuerda
  // sigue cogido, y el siguiente intento se estrella contra "Access denied".
  _releasePort(motivo) {
    const port = this.port;
    this.port = null;
    this.provisional = false;
    clearTimeout(this._provisionalTimer);
    if (!port) return;
    log(`[serie] se suelta el puerto: ${motivo}`);
    try {
      // Sin el manejador de cierre: no había conexión que anunciar como perdida,
      // y dispararlo mandaría un 'disconnected' por un teclado que nunca estuvo.
      port.removeAllListeners('close');
      if (port.isOpen) port.close(() => {});
    } catch { /* ya estaba cerrado */ }
  }

  // --- Vigilante de la conexión ---------------------------------------------
  // Una conexión puede morirse sin que el puerto avise: el equipo suspende, el
  // cable hace mal contacto, Windows se queda el descriptor de un aparato que
  // ya no está. El puerto sigue "abierto", nadie emite 'close', y la app se
  // queda con un teclado que no existe y sin volver a escanear — que es cuando
  // había que ir a Ajustes y pulsar Reconectar a mano.
  //
  // El teclado no habla si no lo tocas, así que el silencio por sí solo no dice
  // nada: hay que preguntar. ACK siempre se contesta.
  _startWatchdog() {
    if (this._watchdog) return;
    const SILENCIO_MS = 12_000;   // sin oír nada, se pregunta
    const RESPUESTA_MS = 4_000;   // sin contestar al ACK, se da por muerta

    this._watchdog = setInterval(() => {
      if (!this.isConnected || !this.port?.isOpen) return;
      const ahora = Date.now();

      if (this._pingAt) {
        if (this._lastLineAt > this._pingAt) { this._pingAt = 0; return; }  // contestó
        if (ahora - this._pingAt > RESPUESTA_MS) {
          this._dropConnection('el teclado no contesta al ACK');
        }
        return;
      }

      if (ahora - (this._lastLineAt || 0) > SILENCIO_MS) {
        this._pingAt = ahora;
        this._sendRaw('ACK\n');
      }
    }, 2000);
  }

  _stopWatchdog() {
    clearInterval(this._watchdog);
    this._watchdog = null;
    this._pingAt = 0;
  }

  // Suelta la conexión y vuelve a buscar. Hace a mano lo que haría el evento
  // 'close' del puerto, porque el caso que cubre es justo ese: que ese evento no
  // llegue nunca.
  _dropConnection(motivo) {
    log(`[serie] se suelta la conexión: ${motivo}`);
    const port = this.port;
    this.port = null;
    this.isConnected = false;
    this.deviceInfo = null;
    this.provisional = false;
    clearTimeout(this._provisionalTimer);
    this._stopWatchdog();

    if (port) {
      try {
        port.removeAllListeners('close');   // el cierre ya lo estamos anunciando aquí
        if (port.isOpen) port.close(() => {});
      } catch { /* el puerto ya no existe: es justo lo que se sospechaba */ }
    }

    this.emit('disconnected');
    this.startAutoScan();
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
    this._startWatchdog();
    log('[serie] sin presentación tras varios ACK: se sigue con identidad mínima');
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
    this._stopWatchdog();
    clearTimeout(this._provisionalTimer);
    this.provisional = false;
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
        log(`[serie] probando ${candidate.path} (${candidate.manufacturer || 'desconocido'})`);
        const success = await this.tryConnect(candidate.path);
        if (success) {
          log(`[serie] conectado al Orby en ${candidate.path}`);
          break;
        }
      }
    } finally {
      this.isScanning = false;
    }
  }
}

module.exports = { OrbySerial };
