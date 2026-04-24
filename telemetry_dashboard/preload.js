const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('navi', {
  // Run a test file; returns nothing — results stream via the callbacks below
  runTest: (file, testsDir) => ipcRenderer.send('test:run', { file, testsDir }),

  // Open native folder picker; resolves to chosen path string or null
  pickDir: () => ipcRenderer.invoke('dialog:pickDir'),

  // Register streaming callbacks (call once at startup)
  onTestLine:  (cb) => ipcRenderer.on('test:line',  (_e, d) => cb(d)),
  onTestDone:  (cb) => ipcRenderer.on('test:done',  (_e, d) => cb(d)),
  onTestError: (cb) => ipcRenderer.on('test:error', (_e, d) => cb(d)),
});