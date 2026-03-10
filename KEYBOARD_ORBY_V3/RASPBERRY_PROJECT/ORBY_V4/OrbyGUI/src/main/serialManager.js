// src/main/serialManager.js
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

class SerialManager {
  constructor() {
    this.port = null;
    this.parser = null;
    this.isConnected = false;
    this.picoVendorId = 'CAFE'; // TinyUSB default VID
    this.reconnectInterval = null;
  }

  async findPicoPort() {
    try {
      const ports = await SerialPort.list();
      const picoPort = ports.find(p => 
        p.vendorId && p.vendorId.toUpperCase() === this.picoVendorId
      );
      return picoPort ? picoPort.path : null;
    } catch (err) {
      console.error('Error listing ports:', err);
      return null;
    }
  }

  async connect() {
    if (this.isConnected) return;

    const path = await this.findPicoPort();
    
    if (!path) {
      console.log('Orby Keyboard not found. Retrying in 2s...');
      return;
    }

    console.log(`Attempting to connect to Orby at ${path}...`);

    this.port = new SerialPort({
      path: path,
      baudRate: 115200,
      autoOpen: false
    });

    this.parser = this.port.pipe(new ReadlineParser({ delimiter: '\n' }));

    this.port.open((err) => {
      if (err) {
        console.error('Error opening port:', err.message);
        return;
      }
      this.isConnected = true;
      console.log('Connected to Orby Keyboard!');
      
      this.parser.on('data', (data) => {
        console.log('Orby says:', data);
      });
    });

    this.port.on('close', () => {
      console.log('Orby disconnected.');
      this.isConnected = false;
      this.port = null;
    });

    this.port.on('error', (err) => {
      console.error('Serial Error:', err.message);
      this.isConnected = false;
    });
  }

  startAutoConnect() {
    this.connect();
    this.reconnectInterval = setInterval(() => {
      if (!this.isConnected) {
        this.connect();
      }
    }, 2000);
  }

  stopAutoConnect() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
    }
    if (this.port && this.isConnected) {
      this.port.close();
    }
  }

  sendCommand(command) {
    if (this.isConnected && this.port) {
      this.port.write(`${command}\n`, (err) => {
        if (err) console.error('Error sending command:', err.message);
      });
    }
  }
}

module.exports = { SerialManager };
