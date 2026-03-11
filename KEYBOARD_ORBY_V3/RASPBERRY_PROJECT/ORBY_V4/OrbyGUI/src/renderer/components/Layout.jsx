import React from 'react';

const Layout = ({ children }) => {
  return (
    <div className="flex h-screen bg-orby-dark text-white font-sans overflow-hidden">
      {/* Sidebar with Glassmorphism */}
      <aside className="w-64 bg-white/5 backdrop-blur-xl border-r border-white/10 flex flex-col pt-8 pb-4 shadow-2xl">
        <div className="px-6 mb-8">
          <h2 className="text-2xl font-black bg-clip-text text-transparent bg-gradient-to-r from-orby-accent to-purple-400 tracking-tight">
            ORBY<span className="font-light text-white">V4</span>
          </h2>
          <p className="text-xs text-gray-500 uppercase tracking-widest mt-1 font-semibold">Macro Studio</p>
        </div>

        <nav className="flex-1 px-4 space-y-2">
          {/* Default Profile */}
          <div className="flex items-center px-4 py-3 bg-orby-accent/10 border border-orby-accent/20 rounded-xl cursor-pointer transition-all hover:bg-orby-accent/20 group">
            <div className="w-8 h-8 rounded-lg bg-orby-accent/20 flex items-center justify-center mr-3 group-hover:bg-orby-accent/40 shadow-lg shadow-orby-accent/20">
              <svg className="w-4 h-4 text-orby-accent" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2H6a2 2 0 01-2-2V6zM14 6a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2V6zM4 16a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2H6a2 2 0 01-2-2v-2zM14 16a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2v-2z" />
              </svg>
            </div>
            <div>
              <p className="text-sm font-semibold text-white">Default P1</p>
              <p className="text-xs text-gray-400">Main Settings</p>
            </div>
          </div>

          {/* Example Profile 2 */}
          <div className="flex items-center px-4 py-3 rounded-xl cursor-pointer transition-all hover:bg-white/5 group border border-transparent">
             <div className="w-8 h-8 rounded-lg bg-white/5 flex items-center justify-center mr-3 group-hover:bg-white/10">
              <svg className="w-4 h-4 text-gray-400 group-hover:text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
            </div>
            <div>
              <p className="text-sm font-semibold text-gray-300 group-hover:text-white">Video Editing</p>
              <p className="text-xs text-gray-500">Premiere Pro</p>
            </div>
          </div>
          
          {/* Example Profile 3 */}
          <div className="flex items-center px-4 py-3 rounded-xl cursor-pointer transition-all hover:bg-white/5 group border border-transparent">
             <div className="w-8 h-8 rounded-lg bg-white/5 flex items-center justify-center mr-3 group-hover:bg-white/10">
              <svg className="w-4 h-4 text-gray-400 group-hover:text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M11 4a2 2 0 114 0v1a1 1 0 001 1h3a1 1 0 011 1v3a1 1 0 01-1 1h-1a2 2 0 100 4h1a1 1 0 011 1v3a1 1 0 01-1 1h-3a1 1 0 01-1-1v-1a2 2 0 10-4 0v1a1 1 0 01-1 1H7a1 1 0 01-1-1v-3a1 1 0 00-1-1H4a2 2 0 110-4h1a1 1 0 001-1V7a1 1 0 011-1h3a1 1 0 001-1V4z" />
              </svg>
            </div>
            <div>
              <p className="text-sm font-semibold text-gray-300 group-hover:text-white">Gaming</p>
              <p className="text-xs text-gray-500">Valorant / CS2</p>
            </div>
          </div>
        </nav>
        
        <div className="px-6 mt-auto">
          <div className="h-px w-full bg-gradient-to-r from-transparent via-white/10 to-transparent mb-4"></div>
          <div className="flex items-center justify-center space-x-2 text-xs text-gray-500">
            <span className="w-2 h-2 rounded-full bg-green-500 animate-pulse"></span>
            <span>Orby Connected via USB</span>
          </div>
        </div>
      </aside>

      {/* Main Content Area */}
      <main className="flex-1 relative overflow-y-auto">
        {/* Background ambient glow */}
        <div className="absolute top-[-10%] left-[-10%] w-[50%] h-[50%] bg-purple-600/20 blur-[120px] rounded-full pointer-events-none"></div>
        <div className="absolute bottom-[-10%] right-[-10%] w-[50%] h-[50%] bg-orby-accent/10 blur-[120px] rounded-full pointer-events-none"></div>
        
        <div className="relative z-10 p-10 h-full flex flex-col">
          <header className="mb-8 flex justify-between items-end">
            <div>
              <h1 className="text-3xl font-bold text-white mb-2">Default Profile</h1>
              <p className="text-gray-400">Configure your Orby macro keys and OLED displays.</p>
            </div>
            <button className="px-5 py-2.5 bg-white/5 hover:bg-white/10 border border-white/10 rounded-xl text-sm font-semibold transition-all">
              Save to Device
            </button>
          </header>
          
          <div className="flex-1 bg-black/40 backdrop-blur-md rounded-3xl border border-white/5 p-8 shadow-2xl overflow-hidden flex items-center justify-center">
            {children}
          </div>
        </div>
      </main>
    </div>
  );
};

export default Layout;
