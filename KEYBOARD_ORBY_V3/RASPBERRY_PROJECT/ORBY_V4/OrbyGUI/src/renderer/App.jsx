import React, { useState, useEffect } from 'react';
import { useProfiles } from './hooks/useProfiles';
import ProfileSidebar from './components/ProfileSidebar';
import MacroGrid from './components/MacroGrid';
import KeyConfigModal from './components/KeyConfigModal';

let ipc = null;
try { ipc = window.require('electron').ipcRenderer; } catch (_) {}

const App = () => {
  const hook = useProfiles();
  const [selectedKeyId, setSelectedKeyId] = useState(null);
  const [isConnected, setIsConnected] = useState(false);

  // Poll connection status every 2s
  useEffect(() => {
    const check = async () => {
      if (!ipc) return;
      const { connected } = await ipc.invoke('device:status');
      setIsConnected(connected);
    };
    check();
    const t = setInterval(check, 2000);
    return () => clearInterval(t);
  }, []);

  const handleSaveToDevice = async () => {
    if (!ipc || !isConnected) return;
    for (const key of hook.activeProfile.keys) {
      if (key.action.value) {
        await ipc.invoke('device:send', `KEY:${key.id}:${key.action.type}:${key.action.value}`);
      }
    }
  };

  const selectedKey = selectedKeyId
    ? hook.activeProfile.keys.find(k => k.id === selectedKeyId)
    : null;

  return (
    <div className="flex h-screen bg-orby-dark text-white font-sans overflow-hidden select-none">
      <ProfileSidebar {...hook} isConnected={isConnected} />

      <main className="flex-1 relative overflow-hidden flex flex-col">
        {/* Ambient glows */}
        <div className="absolute top-0 left-0 w-1/2 h-1/2 bg-blue-600/10 blur-[120px] rounded-full pointer-events-none" />
        <div className="absolute bottom-0 right-0 w-1/2 h-1/2 bg-purple-600/10 blur-[120px] rounded-full pointer-events-none" />

        <div className="relative z-10 flex flex-col h-full p-8">
          {/* Header */}
          <header className="flex justify-between items-end mb-8 shrink-0">
            <div>
              <h1 className="text-3xl font-bold text-white">{hook.activeProfile?.name}</h1>
              <p className="text-gray-400 text-sm mt-1">
                {hook.activeProfile?.appTrigger
                  ? `Auto: ${hook.activeProfile.appTrigger}`
                  : 'Configure macro keys and OLED displays'}
              </p>
            </div>
            <button
              onClick={handleSaveToDevice}
              disabled={!isConnected}
              className={`px-5 py-2.5 rounded-xl text-sm font-bold border transition-all ${
                isConnected
                  ? 'bg-orby-accent/20 border-orby-accent/40 text-orby-accent hover:bg-orby-accent/30'
                  : 'bg-white/5 border-white/10 text-gray-600 cursor-not-allowed'
              }`}
            >
              {isConnected ? '⬆ Save to Device' : '○ Device Offline'}
            </button>
          </header>

          {/* Main content */}
          <div className="flex-1 flex items-center justify-center">
            <MacroGrid
              keys={hook.activeProfile?.keys ?? []}
              encoders={hook.activeProfile?.encoders ?? []}
              onKeyClick={setSelectedKeyId}
              onEncoderUpdate={hook.updateEncoder}
            />
          </div>
        </div>
      </main>

      {selectedKey && (
        <KeyConfigModal
          keyConfig={selectedKey}
          profiles={hook.profiles}
          activeProfileId={hook.activeId}
          onUpdate={(ch) => hook.updateKey(selectedKeyId, ch)}
          onClose={() => setSelectedKeyId(null)}
        />
      )}
    </div>
  );
};

export default App;
