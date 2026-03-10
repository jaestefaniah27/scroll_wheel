const { spawn } = require('child_process');
const path = require('path');

// THE CRITICAL FIX: The host environment injects ELECTRON_RUN_AS_NODE=1 globally,
// causing any nested electron binary to boot sequentially as a silent Node.js app!
// We MUST delete it from the environment before spawning the actual electron instance.
delete process.env.ELECTRON_RUN_AS_NODE;

const electronPath = path.join(__dirname, 'node_modules', '.bin', 'electron.cmd');

console.log("Spawning pure Electron instance...");

const electronProcess = spawn(electronPath, ['.'], {
  stdio: 'inherit',
  env: process.env, // Pass the sanitized environment
  shell: true
});

electronProcess.on('close', (code) => {
  process.exit(code);
});
