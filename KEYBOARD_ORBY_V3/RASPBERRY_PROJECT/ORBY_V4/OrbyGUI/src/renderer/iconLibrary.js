/**
 * Orby Icon Library
 * 
 * Categorized collections of SVG icons optimized for OLED displays (72×40px, monochrome).
 * Each icon is defined as inline SVG path data for rendering at any resolution.
 * The `oledPath` field (if needed) can hold pre-converted 72×40 bitmap data.
 */

// Helper to create icon objects
const icon = (id, label, category, svgPath) => ({ id, label, category, svgPath });

export const ICON_CATEGORIES = {
  utilities: {
    label: '🛠️ Utilities',
    description: 'Everyday actions',
    icons: [
      icon('copy',     'Copy',     'utilities', 'assets/icons/utilities/copy.svg'),
      icon('paste',    'Paste',    'utilities', 'assets/icons/utilities/paste.svg'),
      icon('cut',      'Cut',      'utilities', 'assets/icons/utilities/cut.svg'),
      icon('undo',     'Undo',     'utilities', 'assets/icons/utilities/undo.svg'),
      icon('redo',     'Redo',     'utilities', 'assets/icons/utilities/redo.svg'),
      icon('save',     'Save',     'utilities', 'assets/icons/utilities/save.svg'),
      icon('search',   'Search',   'utilities', 'assets/icons/utilities/search.svg'),
      icon('download', 'Download', 'utilities', 'assets/icons/utilities/download.svg'),
      icon('upload',   'Upload',   'utilities', 'assets/icons/utilities/upload.svg'),
      icon('delete',   'Delete',   'utilities', 'assets/icons/utilities/delete.svg'),
    ],
  },

  apps: {
    label: '🖥️ Apps',
    description: 'Application icons',
    icons: [
      icon('chrome',  'Chrome',  'apps', 'assets/icons/apps/chrome.svg'),
      icon('vscode',  'VS Code', 'apps', 'assets/icons/apps/vscode.svg'),
      icon('spotify', 'Spotify', 'apps', 'assets/icons/apps/spotify.svg'),
      icon('figma',   'Figma',   'apps', 'assets/icons/apps/figma.svg'),
      icon('youtube', 'YouTube', 'apps', 'assets/icons/apps/youtube.svg'),
      icon('discord', 'Discord', 'apps', 'assets/icons/apps/discord.svg'),
    ],
  },

  vscode: {
    label: '💻 VS Code',
    description: 'VS Code actions',
    icons: [
      icon('vsc-terminal', 'Terminal', 'vscode', 'assets/icons/vscode/terminal.svg'),
      icon('vsc-debug',    'Debug',    'vscode', 'assets/icons/vscode/debug.svg'),
      icon('vsc-split',    'Split',    'vscode', 'assets/icons/vscode/split.svg'),
      icon('vsc-format',   'Format',   'vscode', 'assets/icons/vscode/format.svg'),
      icon('vsc-git',      'Git',      'vscode', 'assets/icons/vscode/git.svg'),
      icon('vsc-comment',  'Comment',  'vscode', 'assets/icons/vscode/comment.svg'),
    ],
  },
};

// Flat map for quick lookup by id
export const ALL_ICONS = Object.values(ICON_CATEGORIES)
  .flatMap((cat) => cat.icons);

export const getIconById = (id) => ALL_ICONS.find((icon) => icon.id === id) || null;
