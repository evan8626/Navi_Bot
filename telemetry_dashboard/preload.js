const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('navi', {
  // Run a test file (whole suite, or a single test when testNum is given);
  // returns nothing — results stream via the callbacks below
  runTest: (file, testsDir, testNum) => ipcRenderer.send('test:run', { file, testsDir, testNum }),

  // List a suite's test cases; resolves to ["TEST 1: ...", ...]
  listTests: (file, testsDir) => ipcRenderer.invoke('tests:list', { file, testsDir }),

  // Open native folder picker; resolves to chosen path string or null
  pickDir: () => ipcRenderer.invoke('dialog:pickDir'),

  // Resolve the default tests/ dir (sibling of the dashboard) or null
  defaultTestsDir: () => ipcRenderer.invoke('dialog:defaultTestsDir'),

  // Register streaming callbacks (call once at startup)
  onTestLine:  (cb) => ipcRenderer.on('test:line',  (_e, d) => cb(d)),
  onTestDone:  (cb) => ipcRenderer.on('test:done',  (_e, d) => cb(d)),
  onTestError: (cb) => ipcRenderer.on('test:error', (_e, d) => cb(d)),
});