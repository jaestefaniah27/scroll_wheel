const activeWindow = require('active-win');

class ActiveWindowTracker {
  constructor() {
    this.currentApp = '';
    this.intervalId = null;
  }

  startTracking(onAppChangeCallback) {
    if (this.intervalId) return;

    this.intervalId = setInterval(async () => {
      try {
        const windowInfo = await activeWindow();
        if (windowInfo && windowInfo.owner && windowInfo.owner.name) {
          const newApp = windowInfo.owner.name;
          // Si cambió la app activa, disparamos el callback
          if (newApp !== this.currentApp) {
            this.currentApp = newApp;
            console.log(`Active App Changed: ${this.currentApp}`);
            
            if (onAppChangeCallback) {
              onAppChangeCallback(this.currentApp);
            }
          }
        }
      } catch (error) {
        console.error('Error tracking active window:', error);
      }
    }, 500); // 500ms poling rate
  }

  stopTracking() {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
  }
}

module.exports = { ActiveWindowTracker };
