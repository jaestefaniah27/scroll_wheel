const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const port = new SerialPort({ path: 'COM25', baudRate: 115200, autoOpen: false });
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

port.open((err) => {
  if (err) {
    return console.log('Error opening port: ', err.message);
  }
  
  // Try with DTR/RTS
  port.set({ dtr: true, rts: true }, (err) => {
    if (err) console.log('Error setting DTR/RTS: ', err.message);
    
    console.log('Port opened, sending ACK...');
    port.write('ACK\n', (err) => {
      if (err) console.log('Error writing: ', err.message);
    });

    setTimeout(() => {
      console.log('Sending SET_PROFILE:1...');
      port.write('SET_PROFILE:1\n');
    }, 2000);
  });
});

parser.on('data', console.log);
port.on('error', console.error);

setTimeout(() => {
    console.log('Done.');
    port.close();
}, 5000);
