import React, { useState } from 'react';
import IconPicker from './IconPicker';

const MacroCell = ({ id, hasOled = true, role }) => {
  const [isHovered, setIsHovered] = useState(false);
  const [showPicker, setShowPicker] = useState(false);
  const [selectedIcon, setSelectedIcon] = useState(null);

  // ── Tecla sin OLED (sistema / perfil) ─────────────────────────
  if (!hasOled) {
    return (
      <div
        className={`
          w-24 h-14 rounded-xl flex flex-col items-center justify-center
          cursor-pointer transition-all duration-200 border
          ${isHovered
            ? 'bg-white/10 border-white/30 shadow-[0_0_12px_rgba(255,255,255,0.1)]'
            : 'bg-white/5 border-white/10'
          }
        `}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        title={`Key ${id} — sin OLED`}
      >
        <span className="text-xs font-bold text-gray-400 uppercase tracking-widest">K{id}</span>
        <span className="text-[9px] text-gray-600 mt-0.5">profile</span>
      </div>
    );
  }

  // ── Tecla con OLED ─────────────────────────────────────────────
  return (
    <>
      <div
        className={`
          w-24 h-24 rounded-2xl flex flex-col items-center justify-center
          transition-all duration-300 cursor-pointer relative overflow-hidden border-2
          ${isHovered
            ? 'border-orby-accent bg-orby-accent/10 shadow-[0_0_20px_rgba(88,166,255,0.25)]'
            : 'border-gray-700/50 bg-black/60'
          }
        `}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        onClick={() => setShowPicker(true)}
      >
        {/* Simulación pantalla OLED 72×40 */}
        <div className={`
          w-16 h-10 bg-black rounded border flex items-center justify-center overflow-hidden
          ${isHovered ? 'border-orby-accent/50' : 'border-gray-800'}
          transition-colors duration-300
        `}>
          {selectedIcon ? (
            <img
              src={selectedIcon.svgPath}
              alt={selectedIcon.label}
              className="w-full h-full object-contain p-1"
              style={{ filter: 'brightness(0) invert(1)' }}
            />
          ) : (
            <span className={`text-base font-mono ${isHovered ? 'text-orby-accent' : 'text-gray-700'}`}>
              K{id}
            </span>
          )}
        </div>

        {/* Key label */}
        <div className="mt-2 text-[10px] font-bold text-gray-600 uppercase tracking-wide">
          {selectedIcon ? selectedIcon.label : `Key ${id}`}
        </div>

        {/* Hover overlay */}
        {isHovered && (
          <div className="absolute inset-0 bg-black/50 backdrop-blur-[2px] flex flex-col items-center justify-center opacity-0 hover:opacity-100 transition-opacity duration-150">
            <svg className="w-5 h-5 text-white mb-1" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 4v16m8-8H4" />
            </svg>
            <span className="text-[9px] text-white font-bold tracking-widest">
              {selectedIcon ? 'CHANGE ICON' : 'ADD ICON'}
            </span>
          </div>
        )}
      </div>

      {/* Icon Picker Modal */}
      {showPicker && (
        <IconPicker
          keyId={id}
          onSelect={(ico) => setSelectedIcon(ico)}
          onClose={() => setShowPicker(false)}
        />
      )}
    </>
  );
};

export default MacroCell;
