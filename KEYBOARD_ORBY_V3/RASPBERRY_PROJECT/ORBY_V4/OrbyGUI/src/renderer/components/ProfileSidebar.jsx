import React, { useState, useRef, useEffect } from 'react';

const ProfileSidebar = ({ profiles, activeId, switchProfile, addProfile, deleteProfile, renameProfile, setAppTrigger, isConnected }) => {
  const [editId, setEditId] = useState(null);
  const [editName, setEditName] = useState('');
  const [showAdd, setShowAdd] = useState(false);
  const [newName, setNewName] = useState('');
  const [triggerEditId, setTriggerEditId] = useState(null);
  const [triggerValue, setTriggerValue] = useState('');
  const addInputRef = useRef(null);

  useEffect(() => { if (showAdd) addInputRef.current?.focus(); }, [showAdd]);

  const startRename = (p, e) => { e.stopPropagation(); setEditId(p.id); setEditName(p.name); };
  const commitRename = () => { if (editName.trim()) renameProfile(editId, editName.trim()); setEditId(null); };
  const handleAdd = () => { if (newName.trim()) { addProfile(newName.trim()); setNewName(''); setShowAdd(false); } };

  return (
    <aside className="w-60 flex flex-col bg-white/5 backdrop-blur-2xl border-r border-white/10 shadow-2xl shrink-0">
      {/* Logo */}
      <div className="px-5 pt-6 pb-4 border-b border-white/5">
        <h2 className="text-2xl font-black tracking-tight">
          <span className="bg-clip-text text-transparent bg-gradient-to-r from-orby-accent to-purple-400">ORBY</span>
          <span className="text-white font-light">V4</span>
        </h2>
        <p className="text-[10px] text-gray-500 uppercase tracking-[0.2em] font-semibold mt-0.5">Macro Studio</p>
      </div>

      <div className="px-4 pt-4 pb-1">
        <p className="text-[10px] text-gray-600 uppercase tracking-widest font-bold">Profiles</p>
      </div>

      {/* Profile list */}
      <nav className="flex-1 px-3 space-y-0.5 overflow-y-auto py-1">
        {profiles.map(p => (
          <div key={p.id}>
            <div
              onClick={() => switchProfile(p.id)}
              className={`flex items-center gap-2.5 px-3 py-2.5 rounded-xl cursor-pointer transition-all group border ${
                p.id === activeId
                  ? 'bg-orby-accent/10 border-orby-accent/20'
                  : 'border-transparent hover:bg-white/5'
              }`}
            >
              <div className={`w-2 h-2 rounded-full shrink-0 transition-colors ${p.id === activeId ? 'bg-orby-accent shadow-[0_0_6px_#58a6ff]' : 'bg-gray-700'}`} />
              <div className="flex-1 min-w-0">
                {editId === p.id ? (
                  <input autoFocus className="w-full bg-transparent text-sm text-white outline-none border-b border-orby-accent/50"
                    value={editName} onChange={e => setEditName(e.target.value)}
                    onBlur={commitRename}
                    onKeyDown={e => { if (e.key === 'Enter') commitRename(); if (e.key === 'Escape') setEditId(null); }}
                    onClick={e => e.stopPropagation()} />
                ) : (
                  <p className={`text-sm font-semibold truncate transition-colors ${p.id === activeId ? 'text-white' : 'text-gray-400 group-hover:text-white'}`}>{p.name}</p>
                )}
                {p.appTrigger && <p className="text-[10px] text-gray-600 truncate mt-0.5">{p.appTrigger}</p>}
              </div>
              <div className="opacity-0 group-hover:opacity-100 flex gap-0.5 shrink-0">
                <button onClick={e => startRename(p, e)} className="w-5 h-5 rounded flex items-center justify-center hover:bg-white/10 text-gray-500 hover:text-white text-[10px]">✏</button>
                {profiles.length > 1 && (
                  <button onClick={e => { e.stopPropagation(); deleteProfile(p.id); }} className="w-5 h-5 rounded flex items-center justify-center hover:bg-red-500/20 text-gray-500 hover:text-red-400 text-[10px]">✕</button>
                )}
              </div>
            </div>

            {/* App trigger (shown for active profile) */}
            {p.id === activeId && (triggerEditId === p.id ? (
              <div className="px-4 pb-2">
                <input
                  autoFocus
                  placeholder="App name, e.g. Code.exe"
                  className="w-full bg-black/30 border border-orby-accent/30 rounded-lg px-2 py-1 text-[10px] text-white placeholder-gray-600 outline-none"
                  value={triggerValue}
                  onChange={e => setTriggerValue(e.target.value)}
                  onBlur={() => { setAppTrigger(p.id, triggerValue); setTriggerEditId(null); }}
                  onKeyDown={e => { if (e.key === 'Enter') { setAppTrigger(p.id, triggerValue); setTriggerEditId(null); } }}
                />
              </div>
            ) : (
              <button
                onClick={() => { setTriggerEditId(p.id); setTriggerValue(p.appTrigger || ''); }}
                className="w-full text-left px-9 pb-2 text-[10px] text-gray-600 hover:text-gray-400 transition-colors"
              >
                {p.appTrigger ? `⚡ ${p.appTrigger}` : '+ set app trigger'}
              </button>
            ))}
          </div>
        ))}

        {/* Add profile */}
        {showAdd ? (
          <div className="px-3 py-2">
            <input ref={addInputRef} placeholder="Profile name..." value={newName}
              className="w-full bg-black/40 border border-orby-accent/30 rounded-xl px-3 py-1.5 text-sm text-white placeholder-gray-600 outline-none"
              onChange={e => setNewName(e.target.value)}
              onKeyDown={e => { if (e.key === 'Enter') handleAdd(); if (e.key === 'Escape') setShowAdd(false); }}
              onBlur={() => { if (!newName.trim()) setShowAdd(false); }}
            />
          </div>
        ) : (
          <button onClick={() => setShowAdd(true)}
            className="w-full flex items-center gap-2 px-3 py-2.5 rounded-xl text-gray-500 hover:text-white hover:bg-white/5 transition-all text-sm mt-1">
            <span className="text-lg leading-none font-light">+</span>
            <span className="font-medium">Add Profile</span>
          </button>
        )}
      </nav>

      {/* Status bar */}
      <div className="px-5 py-3 border-t border-white/10 flex items-center gap-2">
        <div className={`w-2 h-2 rounded-full transition-all ${isConnected ? 'bg-green-400 shadow-[0_0_6px_#4ade80] animate-pulse' : 'bg-gray-700'}`} />
        <span className="text-xs text-gray-500">{isConnected ? 'Orby Connected' : 'Disconnected'}</span>
      </div>
    </aside>
  );
};

export default ProfileSidebar;
