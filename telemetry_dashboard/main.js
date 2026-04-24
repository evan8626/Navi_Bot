const { app, BrowserWindow, ipcMain } = require('electron');
const path  = require('path');
const { spawn } = require('child_process');

let win;

function createWindow() {
  win = new BrowserWindow({
    width: 1200,
    height: 820,
    minWidth: 900,
    minHeight: 600,
    backgroundColor: '#0e0e0c',
    titleBarStyle: 'hiddenInset',
    title: 'Navi_Bot Telemetry',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });
  win.loadFile('index.html');
}

app.whenReady().then(createWindow);
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });

// ─────────────────────────────────────────────────────────────────────────────
//  IPC: run a test file via Python subprocess
//
//  Renderer sends:  { file: 'AStar_planner_test.py', testsDir: '/abs/path/tests' }
//  Main streams back via:
//    'test:line'   { file, text }   — stdout/stderr line
//    'test:done'   { file, code }   — process exit
//    'test:error'  { file, msg  }   — spawn error
// ─────────────────────────────────────────────────────────────────────────────
ipcMain.on('test:run', (event, { file, testsDir }) => {
  const scriptPath = path.join(testsDir, file);

  // Resolve the python executable — honours virtualenvs on Windows (MINGW64)
  const pythonCmd = process.platform === 'win32' ? 'python' : 'python3';

  let proc;
  try {
    proc = spawn(pythonCmd, ['-u', scriptPath], {
      cwd: path.dirname(testsDir),   // repo root so navi_bot package is on path
      env: { ...process.env },
    });
  } catch (err) {
    event.sender.send('test:error', { file, msg: err.message });
    return;
  }

  const send = (text) => event.sender.send('test:line', { file, text });

  proc.stdout.on('data', (d) => d.toString().split('\n').forEach(l => l && send(l)));
  proc.stderr.on('data', (d) => d.toString().split('\n').forEach(l => l && send(l)));
  proc.on('error', (err) => event.sender.send('test:error', { file, msg: err.message }));
  proc.on('close', (code) => event.sender.send('test:done', { file, code }));
});

// ─────────────────────────────────────────────────────────────────────────────
//  IPC: open native folder picker so user can locate their tests/ directory
// ─────────────────────────────────────────────────────────────────────────────
ipcMain.handle('dialog:pickDir', async () => {
  const { dialog } = require('electron');
  const result = await dialog.showOpenDialog(win, {
    title: 'Select tests/ directory',
    properties: ['openDirectory'],
  });
  return result.canceled ? null : result.filePaths[0];
});