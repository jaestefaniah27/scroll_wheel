import React, { useState } from 'react';
import { ICON_CATEGORIES } from '../iconLibrary.js';

/**
 * IconPicker — Modal para seleccionar iconos del Orby Icon Set
 * 
 * Props:
 *   onSelect(icon) — callback cuando el usuario elige un icono
 *   onClose()      — callback para cerrar el picker
 *   keyId          — número de tecla para mostrar en el título
 */
const IconPicker = ({ onSelect, onClose, keyId }) => {
  const categories = Object.values(ICON_CATEGORIES);
  const [activeCategory, setActiveCategory] = useState(categories[0].label);
  const [hoveredIcon, setHoveredIcon] = useState(null);

  const currentIcons = categories.find((c) => c.label === activeCategory)?.icons ?? [];

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-sm" onClick={onClose}>
      <div
        className="bg-[#161b22] border border-white/10 rounded-3xl w-[520px] shadow-2xl overflow-hidden"
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div className="px-6 py-4 border-b border-white/10 flex items-center justify-between">
          <div>
            <h2 className="text-white font-bold text-lg">Icon Picker</h2>
            <p className="text-gray-400 text-xs mt-0.5">Select icon for Key {keyId}</p>
          </div>
          <button
            onClick={onClose}
            className="w-8 h-8 rounded-full bg-white/10 hover:bg-white/20 flex items-center justify-center transition-colors text-gray-300"
          >
            ✕
          </button>
        </div>

        {/* Category Tabs */}
        <div className="flex gap-1 px-4 pt-4">
          {categories.map((cat) => (
            <button
              key={cat.label}
              onClick={() => setActiveCategory(cat.label)}
              className={`px-3 py-1.5 rounded-lg text-xs font-semibold transition-all ${
                activeCategory === cat.label
                  ? 'bg-orby-accent/20 text-orby-accent border border-orby-accent/30'
                  : 'text-gray-400 hover:text-white hover:bg-white/5'
              }`}
            >
              {cat.label}
            </button>
          ))}
        </div>

        {/* Icon Grid */}
        <div className="p-4 grid grid-cols-5 gap-3 max-h-64 overflow-y-auto">
          {currentIcons.map((ico) => (
            <button
              key={ico.id}
              onClick={() => { onSelect(ico); onClose(); }}
              onMouseEnter={() => setHoveredIcon(ico.id)}
              onMouseLeave={() => setHoveredIcon(null)}
              className={`
                flex flex-col items-center gap-2 p-3 rounded-xl border transition-all
                ${hoveredIcon === ico.id
                  ? 'border-orby-accent bg-orby-accent/10 shadow-[0_0_12px_rgba(88,166,255,0.2)]'
                  : 'border-white/10 bg-white/5 hover:border-white/20'
                }
              `}
            >
              {/* SVG preview */}
              <div className="w-16 h-9 bg-black rounded flex items-center justify-center border border-gray-800 overflow-hidden">
                <img
                  src={ico.svgPath}
                  alt={ico.label}
                  className="w-full h-full object-contain"
                  draggable={false}
                />
              </div>
              <span className="text-[10px] text-gray-400 text-center leading-tight">{ico.label}</span>
            </button>
          ))}
        </div>

        {/* Footer */}
        <div className="px-6 py-3 border-t border-white/10 text-center">
          <p className="text-xs text-gray-500">Click an icon to assign it to Key {keyId}</p>
        </div>
      </div>
    </div>
  );
};

export default IconPicker;
