// src/main/serialManager.js
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const EventEmitter = require('events');

class SerialManager extends EventEmitter {
  constructor() {
    super();
    this.port = null;
    this.parser = null;
    this.isConnected = false;
    this.picoVendorId = 'CAFE'; // TinyUSB default VID
    this.reconnectInterval = null;
    this._portPath = null;
  }

  async findPicoPort() {
    try {
      const ports = await SerialPort.list();
      console.log('[Serial] Available ports:', ports.map(p => `${p.path} VID:${p.vendorId}`).join(', '));
      const picoPort = ports.find(p =>
        p.vendorId && p.vendorId.toUpperCase() === this.picoVendorId
      );
      // Fallback: scan for known Raspberry Pi PIDs if VID not found
      if (!picoPort) {
        const rpiPort = ports.find(p =>
          p.vendorId && ['2E8A', '239A'].includes(p.vendorId.toUpperCase())
        );
        return rpiPort ? rpiPort.path : null;
      }
      return picoPort.path;
    } catch (err) {
      console.error('[Serial] Error listing ports:', err);
      return null;
    }
  }

  async connect() {
    if (this.isConnected) return;

    const path = await this.findPicoPort();
    if (!path) {
      console.log('[Serial] Orby Keyboard not found, retrying in 2s...');
      return;
    }

    this._portPath = path;
    console.log(`[Serial] Connecting to ${path}...`);

    this.port = new SerialPort({ path, baudRate: 115200, autoOpen: false });
    this.parser = this.port.pipe(new ReadlineParser({ delimiter: '\n' }));

    this.port.open((err) => {
      if (err) {
        console.error('[Serial] Error opening port:', err.message);
        this.port = null;
        return;
      }
      this.isConnected = true;
      console.log('[Serial] Connected to Orby Keyboard!');
      this.emit('statusChange', true, path);

      this.parser.on('data', (data) => {
        console.log('[Serial] Pico →', data.trim());
        this.emit('data', data.trim());
      });
    });

    this.port.on('close', () => {
      console.log('[Serial] Orby disconnected.');
      this.isConnected = false;
      this.port = null;
      this.emit('statusChange', false, null);
    });

    this.port.on('error', (err) => {
      console.error('[Serial] Error:', err.message);
      this.isConnected = false;
      this.emit('statusChange', false, null);
    });
  }

  startAutoConnect() {
    this.connect();
    this.reconnectInterval = setInterval(() => {
      if (!this.isConnected) this.connect();
    }, 2000);
  }

  stopAutoConnect() {
    clearInterval(this.reconnectInterval);
    if (this.port && this.isConnected) this.port.close();
  }

  sendCommand(command) {
    if (this.isConnected && this.port) {
      this.port.write(`${command}\n`, (err) => {
        if (err) console.error('[Serial] Write error:', err.message);
      });
    }
  }

  // Send raw binary data (for OLED bitmaps)
  sendBinary(prefix, data) {
    if (!this.isConnected || !this.port) return;
    this.port.write(prefix + data + '\n', (err) => {
      if (err) console.error('[Serial] Binary write error:', err.message);
    });
  }
}

module.exports = { SerialManager };
