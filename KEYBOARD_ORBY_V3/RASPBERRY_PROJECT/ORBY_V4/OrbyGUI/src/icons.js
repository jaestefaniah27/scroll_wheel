// Iconos SVG en línea.
// Sustituyen a @phosphor-icons/web, que se cargaba por CDN y dejaba la app sin
// iconografía en cuanto no había conexión a internet.

const PATHS = {
  dashboard: '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',
  profiles: '<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M9 9v12"/>',
  wheel: '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="2.4" fill="currentColor" stroke="none"/>',
  oled: '<rect x="2" y="5" width="20" height="14" rx="2"/><path d="M6 15l3.5-4 2.5 3 2-2.5L18 15"/>',
  settings: '<path d="M4 6h16M4 12h16M4 18h16"/><circle cx="9" cy="6" r="2"/><circle cx="15" cy="12" r="2"/><circle cx="8" cy="18" r="2"/>',
  console: '<rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 9l3 3-3 3M13 15h5"/>',

  minus: '<path d="M5 12h14"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  fit: '<path d="M3 8V4h4M21 8V4h-4M3 16v4h4M21 16v4h-4"/><rect x="8" y="9" width="8" height="6" rx="1"/>',
  up: '<path d="M6 15l6-6 6 6"/>',
  down: '<path d="M6 9l6 6 6-6"/>',
  square: '<rect x="5" y="5" width="14" height="14" rx="1"/>',
  select: '<rect x="4" y="4" width="16" height="16" rx="1" stroke-dasharray="3 3"/>',
  close: '<path d="M6 6l12 12M18 6L6 18"/>',

  save: '<path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><path d="M17 21v-8H7v8M7 3v5h8"/>',
  refresh: '<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>',
  trash: '<path d="M3 6h18M8 6V4h8v2M6 6l1 14h10l1-14"/>',
  upload: '<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M12 3v13M7 8l5-5 5 5"/>',
  pencil: '<path d="M17 3l4 4L8 20l-5 1 1-5z"/>',
  eraser: '<path d="M4 16L13 7l6 6-6 6H7z"/><path d="M9 21h11"/>',
  fill: '<path d="M4 12L12 4l8 8-8 8z"/><path d="M19 15c1.5 2 2 2.8 2 3.6a2 2 0 0 1-4 0c0-.8.5-1.6 2-3.6z"/>',
  invert: '<circle cx="12" cy="12" r="9"/><path d="M12 3a9 9 0 0 1 0 18z" fill="currentColor" stroke="none"/>',
  text: '<path d="M4 6V4h16v2M12 4v16M8 20h8"/>',
  check: '<path d="M4 12l5 5L20 6"/>',
  key: '<circle cx="8" cy="12" r="4"/><path d="M12 12h9M17 12v4M20 12v3"/>',
  bolt: '<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>',
  sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/>',
  moon: '<path d="M21 13a8.5 8.5 0 0 1-10-10 8.5 8.5 0 1 0 10 10z"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/>',
  plug: '<path d="M9 2v6M15 2v6M6 8h12v3a6 6 0 0 1-12 0z"/><path d="M12 17v5"/>',
  reset: '<path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/>',
  download: '<path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><path d="M12 16V3M7 11l5 5 5-5"/>',
  copy: '<rect x="8" y="8" width="12" height="13" rx="1.5"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>',
  paste: '<rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v1"/><path d="M9 12h6M9 16h6"/>',
};

export function icon(name, size = 20) {
  const body = PATHS[name] || PATHS.info;
  return `<svg class="icn" width="${size}" height="${size}" viewBox="0 0 24 24" fill="none"
    stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"
    aria-hidden="true">${body}</svg>`;
}

// Inyecta iconos en cualquier elemento con data-icon="<nombre>".
export function hydrateIcons(root = document) {
  root.querySelectorAll('[data-icon]').forEach((el) => {
    const size = parseInt(el.getAttribute('data-icon-size') || '20', 10);
    el.innerHTML = icon(el.getAttribute('data-icon'), size);
    el.removeAttribute('data-icon');
  });
}
