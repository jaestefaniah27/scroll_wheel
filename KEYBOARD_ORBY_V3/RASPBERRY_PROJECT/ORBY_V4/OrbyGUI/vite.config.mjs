import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  root: 'src/renderer',
  base: './',
  build: {
    outDir: '../../dist',
  },
  server: {
    port: 5173,
    strictPort: true
  },
  plugins: [react()],
})
