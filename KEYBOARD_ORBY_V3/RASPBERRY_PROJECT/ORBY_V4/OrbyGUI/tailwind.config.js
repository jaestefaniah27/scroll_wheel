module.exports = {
  content: [
    "./src/renderer/index.html",
    "./src/renderer/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'orby-dark': '#0d1117',
        'orby-panel': '#161b22',
        'orby-accent': '#58a6ff',
      }
    },
  },
  plugins: [],
}
