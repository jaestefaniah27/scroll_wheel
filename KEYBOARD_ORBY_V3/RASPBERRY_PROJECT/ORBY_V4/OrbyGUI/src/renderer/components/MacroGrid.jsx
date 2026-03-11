import React, { useState } from 'react';
import { ALL_ICONS } from '../iconLibrary.js';
import { KEY_LAYOUT } from '../hooks/useProfiles.js';

const ENCODER_ACTIONS_SHORT = [
  { v: 'none', l: 'None' },
  { v: 'vol_up_down', l: 'Volume' },
  { v: 'media_next_prev', l: 'Track' },
  { v: 'scroll', l: 'Scroll' },
  { v: 'zoom', l: 'Zoom' },
  { v: 'undo_redo', l: 'Undo/Redo' },
];

const SmallKnob = ({ encoder, onUpdate }) => {
  const [open, setOpen] = useState(false);
  return (
    <div className="flex flex-col items-center gap-2">
      <div className="relative">
        <div
          onClick={() => setOpen(o => !o)}
          className="w-16 h-16 rounded-full bg-gradient-to-b from-gray-600 to-gray-900 border-2 border-gray-600 shadow-[0_8px_20px_rgba(0,0,0,0.8),inset_0_2px_6px_rgba(255,255,255,0.08)] relative flex items-center justify-center cursor-pointer transition-transform hover:scale-110"
        >
          <div className="w-10 h-10 rounded-full bg-black/50 border border-gray-800 absolute" />
          <div className="w-1.5 h-1.5 rounded-full bg-orby-accent absolute top-1.5 shadow-[0_0_6px_#58a6ff]" />
        </div>
        {open && (
          <div className="absolute top-full mt-2 left-1/2 -translate-x-1/2 z-20 bg-[#161b22] border border-white/10 rounded-xl shadow-2xl w-32 overflow-hidden">
            {ENCODER_ACTIONS_SHORT.map(a => (
              <button key={a.v} onClick={() => { onUpdate({ cwAction: { type: a.v, value: '' }, ccwAction: { type: a.v, value: '' } }); setOpen(false); }}
                className={`w-full text-left px-3 py-1.5 text-xs transition-colors ${encoder.cwAction?.type === a.v ? 'bg-orby-accent/20 text-orby-accent' : 'text-gray-400 hover:bg-white/5 hover:text-white'}`}>
                {a.l}
              </button>
            ))}
          </div>
        )}
      </div>
      <p className="text-[10px] font-semibold text-gray-500 max-w-[72px] text-center leading-tight">{encoder.label}</p>
      {encoder.cwAction?.type && encoder.cwAction.type !== 'none' && (
        <span className="text-[8px] text-orby-accent/80 bg-orby-accent/10 px-1.5 py-0.5 rounded-md">{ENCODER_ACTIONS_SHORT.find(a => a.v === encoder.cwAction.type)?.l}</span>
      )}
    </div>
  );
};

const BigWheel = ({ encoder, onUpdate }) => {
  const [open, setOpen] = useState(false);
  return (
    <div className="flex flex-col items-center gap-2 my-2">
      <div className="relative">
        <div onClick={() => setOpen(o => !o)}
          className="w-28 h-28 rounded-full bg-gradient-to-b from-gray-500 to-gray-900 border-[4px] border-gray-500 shadow-[0_12px_36px_rgba(0,0,0,0.9),inset_0_3px_12px_rgba(255,255,255,0.1)] relative flex items-center justify-center cursor-pointer transition-transform hover:scale-105">
          <div className="w-20 h-20 rounded-full bg-black/60 border border-gray-700 absolute" />
          <div className="w-2.5 h-2.5 rounded-full bg-gray-500 z-10 hover:bg-orby-accent transition-colors shadow-lg" />
          {[0, 60, 120, 180, 240, 300].map(d => (
            <div key={d} className="absolute w-0.5 h-3 bg-gray-700 rounded-full"
              style={{ transform: `rotate(${d}deg) translateY(-42px)`, transformOrigin: 'center' }} />
          ))}
        </div>
        {open && (
          <div className="absolute top-full mt-2 left-1/2 -translate-x-1/2 z-20 bg-[#161b22] border border-white/10 rounded-xl shadow-2xl w-36 overflow-hidden">
            {ENCODER_ACTIONS_SHORT.map(a => (
              <button key={a.v} onClick={() => { onUpdate({ cwAction: { type: a.v, value: '' }, ccwAction: { type: a.v, value: '' } }); setOpen(false); }}
                className={`w-full text-left px-3 py-1.5 text-xs transition-colors ${encoder.cwAction?.type === a.v ? 'bg-orby-accent/20 text-orby-accent' : 'text-gray-400 hover:bg-white/5 hover:text-white'}`}>
                {a.l}
              </button>
            ))}
          </div>
        )}
      </div>
      <p className="text-sm font-bold text-gray-300">{encoder.label}</p>
      {encoder.cwAction?.type && encoder.cwAction.type !== 'none' && (
        <span className="text-[9px] text-orby-accent/80 bg-orby-accent/10 px-2 py-0.5 rounded-md">{ENCODER_ACTIONS_SHORT.find(a => a.v === encoder.cwAction.type)?.l}</span>
      )}
    </div>
  );
};

const MacroCell = ({ keyConfig, onClick, layoutInfo, isPressed }) => {
  const [hovered, setHovered] = useState(false);
  const icon = ALL_ICONS.find(i => i.id === keyConfig?.iconId);

  if (!layoutInfo.hasOled) {
    return (
      <div onClick={onClick} onMouseEnter={() => setHovered(true)} onMouseLeave={() => setHovered(false)}
        className={`w-24 h-14 rounded-xl flex flex-col items-center justify-center cursor-pointer transition-all border ${isPressed ? 'bg-white border-white scale-95' : hovered ? 'bg-white/10 border-white/25' : 'bg-white/5 border-white/10'}`}>
        <span className={`text-xs font-bold uppercase tracking-widest ${isPressed ? 'text-black' : 'text-gray-400'}`}>K{keyConfig.id}</span>
        <span className={`text-[9px] mt-0.5 ${isPressed ? 'text-gray-800' : 'text-gray-600'}`}>{keyConfig.label || 'profile'}</span>
      </div>
    );
  }

  return (
    <div onClick={onClick} onMouseEnter={() => setHovered(true)} onMouseLeave={() => setHovered(false)}
      className={`w-24 h-24 rounded-2xl flex flex-col items-center justify-center cursor-pointer relative overflow-hidden border-2 transition-all duration-75 ${
        isPressed ? 'border-white bg-white/20 scale-95 shadow-[0_0_30px_rgba(255,255,255,0.4)]' :
        hovered ? 'border-orby-accent bg-orby-accent/10 shadow-[0_0_18px_rgba(88,166,255,0.2)]' : 'border-gray-700/50 bg-black/60'
      }`}>
      {/* OLED Screen sim */}
      <div className={`w-[72px] h-[40px] bg-black rounded border flex items-center justify-center overflow-hidden transition-colors ${hovered || isPressed ? 'border-orby-accent/40' : 'border-gray-800'}`}>
        {icon
          ? <img src={icon.svgPath} alt={icon.label} className="w-full h-full object-contain" style={{ filter: 'brightness(0) invert(1)' }} />
          : <span className={`text-sm font-mono ${hovered || isPressed ? 'text-orby-accent' : 'text-gray-800'}`}>K{keyConfig.id}</span>
        }
      </div>
      <div className="mt-1.5 text-[9px] font-bold text-gray-600 uppercase tracking-wide truncate max-w-[88px] text-center px-1">
        {keyConfig.label || (icon ? icon.label : `Key ${keyConfig.id}`)}
      </div>
      {/* Hover overlay */}
      {hovered && (
        <div className="absolute inset-0 bg-black/50 flex flex-col items-center justify-center opacity-0 hover:opacity-100 transition-opacity">
          <span className="text-[9px] text-white font-bold tracking-widest">{icon ? '✏ EDIT' : '+ ADD ICON'}</span>
        </div>
      )}
    </div>
  );
};

const MacroGrid = ({ keys, encoders, onKeyClick, onEncoderUpdate }) => {
  const enc = (id) => encoders.find(e => e.id === id) || { id, label: `Enc ${id}`, cwAction: { type: 'none' }, ccwAction: { type: 'none' }, pressAction: { type: 'none' } };

  const [pressedKeys, setPressedKeys] = useState({});

  React.useEffect(() => {
    let ipc;
    try { ipc = window.require('electron').ipcRenderer; } catch (_) { return; }

    const handleTelemetry = (_e, data) => {
      if (!data || !data.startsWith('KEY_EV:')) return;
      const parts = data.split(':');
      const keyId = parseInt(parts[1], 10);
      const state = parseInt(parts[2], 10);
      
      setPressedKeys(prev => ({
        ...prev,
        [keyId]: state === 1
      }));
    };

    ipc.on('device-telemetry', handleTelemetry);
    return () => ipc.removeListener('device-telemetry', handleTelemetry);
  }, []);

  return (
    <div className="flex items-center gap-6">
      {/* Key grid 3×4 */}
      <div className="bg-white/5 p-4 rounded-3xl border border-white/10 shadow-xl backdrop-blur-md">
        <div className="grid grid-cols-3 gap-3">
          {KEY_LAYOUT.map(layout => {
            const keyConfig = keys.find(k => k.id === layout.id) || { id: layout.id, iconId: null, label: '', action: { type: 'none', value: '' } };
            return (
              <MacroCell 
                key={layout.id} 
                keyConfig={keyConfig} 
                layoutInfo={layout} 
                onClick={() => onKeyClick(layout.id)} 
                isPressed={pressedKeys[layout.id]}
              />
            );
          })}
        </div>
      </div>

      {/* Encoder panel */}
      <div className="flex flex-col items-center gap-1 bg-white/5 backdrop-blur-md border border-white/10 rounded-3xl px-6 py-5 shadow-xl min-w-[160px]">
        {/* 2 small knobs */}
        <div className="flex gap-6 mb-3">
          <SmallKnob encoder={enc(1)} onUpdate={(ch) => onEncoderUpdate(1, ch)} />
          <SmallKnob encoder={enc(2)} onUpdate={(ch) => onEncoderUpdate(2, ch)} />
        </div>
        <div className="w-full h-px bg-white/10" />
        {/* Big wheel */}
        <BigWheel encoder={enc(3)} onUpdate={(ch) => onEncoderUpdate(3, ch)} />
        {/* Screw mount dots */}
        <div className="flex gap-5 mt-1">
          {[0, 1, 2].map(i => <div key={i} className="w-1.5 h-1.5 rounded-full border border-gray-700 bg-gray-900" />)}
        </div>
      </div>
    </div>
  );
};

export default MacroGrid;
