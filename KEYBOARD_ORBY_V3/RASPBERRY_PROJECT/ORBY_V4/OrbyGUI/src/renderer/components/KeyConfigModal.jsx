import React, { useState, useEffect } from 'react';
import { ALL_ICONS, ICON_CATEGORIES } from '../iconLibrary.js';

const ACTION_TYPES = [
  { v: 'none', l: 'None', d: 'No action' },
  { v: 'shortcut', l: 'Shortcut', d: 'Keyboard combo' },
  { v: 'text', l: 'Text', d: 'Type text' },
  { v: 'media', l: 'Media', d: 'Media control' },
  { v: 'profile_switch', l: 'Profile', d: 'Switch profile' },
];

const MEDIA_OPTS = [
  { v: 'play_pause', l: '⏯', t: 'Play/Pause' },
  { v: 'next', l: '⏭', t: 'Next Track' },
  { v: 'prev', l: '⏮', t: 'Prev Track' },
  { v: 'vol_up', l: '🔊', t: 'Volume Up' },
  { v: 'vol_down', l: '🔉', t: 'Volume Down' },
  { v: 'mute', l: '🔇', t: 'Mute' },
];

// Keyboard shortcut recorder
const ShortcutRecorder = ({ value, onChange }) => {
  const [rec, setRec] = useState(false);
  useEffect(() => {
    if (!rec) return;
    const handler = (e) => {
      e.preventDefault();
      const parts = [];
      if (e.ctrlKey) parts.push('Ctrl');
      if (e.altKey) parts.push('Alt');
      if (e.shiftKey) parts.push('Shift');
      if (e.metaKey) parts.push('Win');
      const k = e.key;
      if (!['Control', 'Alt', 'Shift', 'Meta'].includes(k)) {
        parts.push(k.length === 1 ? k.toUpperCase() : k);
        onChange(parts.join('+'));
        setRec(false);
      }
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [rec, onChange]);

  return (
    <div className="flex gap-2">
      <div className="flex-1 bg-black/40 border border-gray-800 rounded-xl px-3 py-2 text-sm font-mono text-white min-h-[38px] flex items-center">
        {rec ? <span className="text-orby-accent animate-pulse text-xs">Press keys...</span>
          : value ? <span>{value}</span>
            : <span className="text-gray-600 text-xs">Not configured</span>}
      </div>
      <button onClick={() => setRec(r => !r)}
        className={`px-3 rounded-xl text-xs font-bold border transition-all ${rec ? 'bg-red-500/20 border-red-500/40 text-red-400' : 'bg-white/5 border-white/10 text-white hover:bg-white/10'}`}>
        {rec ? 'Cancel' : 'Capture'}
      </button>
      {value && !rec && (
        <button onClick={() => onChange('')} className="px-2 text-gray-600 hover:text-white text-xs">✕</button>
      )}
    </div>
  );
};

const KeyConfigModal = ({ keyConfig, profiles, activeProfileId, onUpdate, onClose }) => {
  const [tab, setTab] = useState('icon');
  const [iconCat, setIconCat] = useState(Object.keys(ICON_CATEGORIES)[0]);
  const [label, setLabel] = useState(keyConfig.label || '');
  const [aType, setAType] = useState(keyConfig.action?.type || 'none');
  const [aValue, setAValue] = useState(keyConfig.action?.value || '');
  const currentIcon = ALL_ICONS.find(i => i.id === keyConfig.iconId);

  // Commit changes on close
  const applyAndClose = () => {
    onUpdate({ label, action: { type: aType, value: aValue } });
    onClose();
  };

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 backdrop-blur-sm" onClick={applyAndClose}>
      <div className="bg-[#0d1117] border border-white/10 rounded-3xl w-[560px] shadow-2xl overflow-hidden" onClick={e => e.stopPropagation()}>

        {/* Header */}
        <div className="flex items-center justify-between px-6 py-4 border-b border-white/10">
          <div className="flex items-center gap-3">
            <div className="w-12 h-8 bg-black rounded-lg border border-gray-800 flex items-center justify-center overflow-hidden">
              {currentIcon
                ? <img src={currentIcon.svgPath} alt="" className="w-full h-full object-contain p-0.5" style={{ filter: 'brightness(0) invert(1)' }} />
                : <span className="text-gray-600 text-[10px] font-mono">K{keyConfig.id}</span>}
            </div>
            <div>
              <h2 className="text-white font-bold text-base">Key {keyConfig.id}</h2>
              <p className="text-gray-500 text-xs">{currentIcon?.label || 'No icon assigned'}</p>
            </div>
          </div>
          <button onClick={applyAndClose} className="w-8 h-8 rounded-full bg-white/5 hover:bg-white/10 flex items-center justify-center text-gray-400 text-sm">✕</button>
        </div>

        {/* Tabs */}
        <div className="flex border-b border-white/10">
          {[{ id: 'icon', l: '🖼  Icon' }, { id: 'action', l: '⚡ Action' }].map(t => (
            <button key={t.id} onClick={() => setTab(t.id)}
              className={`flex-1 py-3 text-sm font-semibold transition-all border-b-2 ${tab === t.id ? 'text-white border-orby-accent' : 'text-gray-500 border-transparent hover:text-white'}`}>
              {t.l}
            </button>
          ))}
        </div>

        <div className="p-5">

          {/* ── ICON TAB ── */}
          {tab === 'icon' && (
            <div className="space-y-4">
              <div>
                <label className="text-[10px] font-bold uppercase tracking-widest text-gray-500 mb-1.5 block">Key Label</label>
                <input className="w-full bg-black/40 border border-gray-800 rounded-xl px-3 py-2 text-sm text-white placeholder-gray-600 outline-none focus:border-orby-accent/50 transition-colors"
                  placeholder="Copy, Debug, Play..." value={label} onChange={e => setLabel(e.target.value)} />
              </div>

              <div className="flex gap-1 flex-wrap">
                {Object.entries(ICON_CATEGORIES).map(([key, cat]) => (
                  <button key={key} onClick={() => setIconCat(key)}
                    className={`px-3 py-1.5 rounded-lg text-xs font-semibold transition-all ${iconCat === key ? 'bg-orby-accent/20 text-orby-accent border border-orby-accent/30' : 'text-gray-400 hover:text-white hover:bg-white/5'}`}>
                    {cat.label}
                  </button>
                ))}
              </div>

              <div className="grid grid-cols-5 gap-2 max-h-44 overflow-y-auto pr-1">
                {(ICON_CATEGORIES[iconCat]?.icons || []).map(ico => {
                  const active = keyConfig.iconId === ico.id;
                  return (
                    <button key={ico.id}
                      onClick={() => onUpdate({ iconId: ico.id })}
                      className={`flex flex-col items-center gap-1.5 p-2 rounded-xl border transition-all ${active ? 'border-orby-accent bg-orby-accent/10' : 'border-white/10 bg-white/5 hover:border-white/25 hover:bg-white/10'}`}>
                      <div className="w-16 h-9 bg-black rounded border border-gray-800 flex items-center justify-center overflow-hidden">
                        <img src={ico.svgPath} alt={ico.label} className="w-full h-full object-contain" style={{ filter: 'brightness(0) invert(1)' }} />
                      </div>
                      <span className="text-[9px] text-gray-400 text-center leading-none">{ico.label}</span>
                    </button>
                  );
                })}
              </div>
            </div>
          )}

          {/* ── ACTION TAB ── */}
          {tab === 'action' && (
            <div className="space-y-4">
              {/* Type selector */}
              <div>
                <label className="text-[10px] font-bold uppercase tracking-widest text-gray-500 mb-2 block">Action Type</label>
                <div className="grid grid-cols-5 gap-1.5">
                  {ACTION_TYPES.map(type => (
                    <button key={type.v} onClick={() => { setAType(type.v); setAValue(''); }}
                      className={`flex flex-col items-center py-2 px-1 rounded-xl border text-xs font-semibold transition-all ${aType === type.v ? 'border-orby-accent bg-orby-accent/10 text-orby-accent' : 'border-white/10 text-gray-400 hover:text-white hover:bg-white/5'}`}>
                      <span className="font-bold">{type.l}</span>
                      <span className="text-[8px] opacity-60 mt-0.5">{type.d}</span>
                    </button>
                  ))}
                </div>
              </div>

              {/* Shortcut */}
              {aType === 'shortcut' && (
                <div>
                  <label className="text-[10px] font-bold uppercase tracking-widest text-gray-500 mb-1.5 block">Key Combination</label>
                  <ShortcutRecorder value={aValue} onChange={setAValue} />
                </div>
              )}

              {/* Text */}
              {aType === 'text' && (
                <div>
                  <label className="text-[10px] font-bold uppercase tracking-widest text-gray-500 mb-1.5 block">Text to Type</label>
                  <textarea rows={3} className="w-full resize-none bg-black/40 border border-gray-800 rounded-xl px-3 py-2 text-sm text-white placeholder-gray-600 outline-none focus:border-orby-accent/50"
                    placeholder="Enter text..." value={aValue} onChange={e => setAValue(e.target.value)} />
                </div>
              )}

              {/* Media */}
              {aType === 'media' && (
                <div>
                  <label className="text-[10px] font-bold uppercase tracking-widest text-gray-500 mb-2 block">Media Action</label>
                  <div className="grid grid-cols-3 gap-2">
                    {MEDIA_OPTS.map(opt => (
                      <button key={opt.v} onClick={() => setAValue(opt.v)}
                        className={`py-3 rounded-xl border text-center transition-all ${aValue === opt.v ? 'border-orby-accent bg-orby-accent/10' : 'border-white/10 bg-white/5 hover:bg-white/10'}`}>
                        <div className="text-2xl">{opt.l}</div>
                        <div className="text-[10px] text-gray-400 mt-1">{opt.t}</div>
                      </button>
                    ))}
                  </div>
                </div>
              )}

              {/* Profile switch */}
              {aType === 'profile_switch' && (
                <div>
                  <label className="text-[10px] font-bold uppercase tracking-widest text-gray-500 mb-1.5 block">Switch to Profile</label>
                  <div className="space-y-1">
                    {profiles.filter(p => p.id !== activeProfileId).map(p => (
                      <button key={p.id} onClick={() => setAValue(p.id)}
                        className={`w-full text-left px-3 py-2.5 rounded-xl border text-sm transition-all ${aValue === p.id ? 'border-orby-accent bg-orby-accent/10 text-white' : 'border-white/10 text-gray-400 hover:bg-white/5 hover:text-white'}`}>
                        {p.name}
                        {p.appTrigger && <span className="text-[10px] text-gray-600 ml-2">({p.appTrigger})</span>}
                      </button>
                    ))}
                    {profiles.filter(p => p.id !== activeProfileId).length === 0 && (
                      <p className="text-gray-600 text-sm text-center py-4">Create another profile first</p>
                    )}
                  </div>
                </div>
              )}

              {/* No action */}
              {aType === 'none' && (
                <div className="py-8 text-center text-gray-600 text-sm">No action assigned to this key.</div>
              )}
            </div>
          )}
        </div>

        {/* Footer buttons */}
        <div className="flex gap-2 px-5 pb-5">
          <button onClick={() => { onUpdate({ iconId: null, label: '', action: { type: 'none', value: '' } }); onClose(); }}
            className="px-4 py-2 text-xs rounded-xl border border-white/10 text-gray-500 hover:text-white hover:bg-white/5 transition-all">
            Clear Key
          </button>
          <button onClick={applyAndClose} className="flex-1 py-2 rounded-xl bg-orby-accent text-[#0d1117] font-bold text-sm hover:bg-orby-accent/90 transition-all">
            Apply
          </button>
        </div>
      </div>
    </div>
  );
};

export default KeyConfigModal;
