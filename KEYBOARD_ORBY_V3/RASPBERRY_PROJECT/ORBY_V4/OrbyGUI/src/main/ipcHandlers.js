const { ipcMain, app } = require('electron');
const fs = require('fs');
const path = require('path');

function getProfilesPath() {
  return path.join(app.getPath('userData'), 'orby-profiles.json');
}

function setupIpcHandlers(serialManager) {
  ipcMain.handle('profiles:load', async () => {
    try {
      const file = getProfilesPath();
      if (fs.existsSync(file)) return JSON.parse(fs.readFileSync(file, 'utf8'));
      return null;
    } catch (e) { console.error('[IPC] profiles:load', e); return null; }
  });

  ipcMain.handle('profiles:save', async (_e, data) => {
    try {
      fs.writeFileSync(getProfilesPath(), JSON.stringify(data, null, 2), 'utf8');
      return { success: true };
    } catch (e) { return { success: false, error: e.message }; }
  });

  ipcMain.handle('device:send', async (_e, command) => {
    if (serialManager?.isConnected) {
      serialManager.sendCommand(command);
      return { success: true };
    }
    return { success: false, error: 'Not connected' };
  });

  ipcMain.handle('device:status', async () => ({
    connected: serialManager?.isConnected ?? false
  }));
}

module.exports = { setupIpcHandlers };
