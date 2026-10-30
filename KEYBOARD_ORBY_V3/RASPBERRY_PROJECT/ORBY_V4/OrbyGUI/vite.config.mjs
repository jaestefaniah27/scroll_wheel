import { readFileSync } from 'node:fs';
import { defineConfig } from 'vite';

// compat.js decide qué firmware ofrece según la versión de la app, y la única
// fuente de esa versión es package.json.
const { version } = JSON.parse(readFileSync(new URL('./package.json', import.meta.url), 'utf8'));

export default defineConfig({
  root: '.',
  base: './',
  define: { __APP_VERSION__: JSON.stringify(version) },
  server: {
    port: 5173,
    strictPort: true,
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
});
