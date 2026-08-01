# TODO: Background service + autostart + plug notification

Goal: OrbyGUI behaves like Razer Synapse / Logitech G HUB — runs in background at
low resource cost, autostarts at login, notifies on device plug, full config
window only opens on demand. No user should ever have to manually launch it.

Decided NOT doing: true "new-hardware-wizard install popup" for a never-seen
device. That requires Device Metadata Authoring + WHQL driver signing +
Microsoft Store listing tied to VID/PID — not worth the cost for this project.
A toast notification on plug achieves the same UX goal for far less effort.

## Subtasks (attack one per conversation, in this order)

### 1. Single-instance lock + lazy window creation — DONE
- Added `app.requestSingleInstanceLock()` in [electron/main.js](../electron/main.js):
  second launch attempt quits itself and instead triggers `second-instance` on
  the first instance, which just shows/focuses the existing window.
- Correction to original plan: window-close was already correct as-is —
  `mainWindow.on('close', ...)` intentionally hides instead of destroying,
  because macro execution (`MACRO:<id>` over serial, see `serial.on('data', ...)`)
  runs in this same process and must keep working while the window is
  "closed"/minimized to tray. Destroying the window on close would have
  broken background macro execution — do not do that.
- Remaining known cost: BrowserWindow still exists hidden (Chromium renderer
  stays loaded) even when not shown — that's inherent to Electron, not a bug
  here. Only a native (non-Electron) background agent would avoid it; not
  pursuing that now (see subtask 4 area / general resource note above).

### 2. Autostart at Windows login — DONE
- `app.setLoginItemSettings({ openAtLogin })` wired behind two new IPC
  handlers in [electron/main.js](../electron/main.js) (`autostart:get` /
  `autostart:set`), exposed via [electron/preload.js](../electron/preload.js)
  as `window.orby.autostart`.
- Toggle lives in the "Autoarranque" card in [index.html](../index.html)
  (Ajustes view) and is wired in [src/views/settings.js](../src/views/settings.js)
  (`initAutostart`).
- Correction to original plan: state is NOT duplicated into `config.json`.
  `setLoginItemSettings` already persists in the Windows registry by itself,
  so the toggle just asks Electron for the real current value on every load —
  storing a second copy risked drifting from what Windows actually has set.
- Decision taken: default OFF, no first-run prompt. Simplest option from the
  original TODO; user opts in from Settings whenever they want it.

### 3. USB plug detection + toast notification
- Use `usb-detection` npm (wraps WM_DEVICECHANGE on Windows) to detect Orby
  VID/PID arrival, instead of polling `serialport.list()`.
- On detect: fire Electron `Notification` (renders as native Windows toast).
  Click → focus/open main config window.
- This is the "Razer popup" substitute — cheap, no driver/signing needed.

### 4. (Optional, later, only if needed) True Windows Service — SKIPPED
- Only pursue if something requires running before any user logs in
  (SYSTEM-level, not per-user). Most companion apps (incl. Razer Synapse)
  don't need this — per-user autostart is enough.
- Would need `node-windows` or NSSM wrapping + admin install step.
- Checked 2026-08-02: no concrete need exists. Also has a hard architectural
  blocker — a real SYSTEM service runs in Windows Session 0, which is
  isolated from the interactive desktop. It could keep a serial listener
  alive pre-login, but could NOT run macros (nut.js needs an interactive
  desktop session for mouse/keyboard simulation) or show the BrowserWindow
  GUI. Splitting into "SYSTEM listener service" + "per-user app for
  macros/GUI" would be a real architecture change, not a small wrapper.
  Not doing this without a concrete driving need.

## Current state (as of writing)
- `OrbyGUI/electron/main.js` already has Tray + Menu + BrowserWindow lazy-ish
  creation (`createWindow()` only called if `BrowserWindow.getAllWindows().length === 0`).
- No autostart code yet.
- No USB hotplug detection yet (device comms likely via `serialport`, see
  `package.json` deps).
- electron-builder NSIS installer already configured in `package.json` (`build.nsis`).
