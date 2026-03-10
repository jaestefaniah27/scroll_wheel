import React from 'react'
import ReactDOM from 'react-dom/client'
import './index.css'

const App = () => {
  return (
    <div className="flex h-screen items-center justify-center p-8 bg-orby-dark">
      <div className="bg-orby-panel p-8 rounded-2xl border border-gray-800 shadow-2xl">
        <h1 className="text-3xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-orby-accent to-purple-500 mb-4">
          Orby Keyboard Connected
        </h1>
        <p className="text-gray-400">
          The PC is now listening for context changes. Check the terminal.
        </p>
      </div>
    </div>
  )
}

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
)
