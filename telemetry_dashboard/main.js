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
ipcMain.on('test:run', (event, { file, testsDir, testNum }) => {
  const scriptPath = path.join(testsDir, file);

  // Resolve the python executable — honours virtualenvs on Windows (MINGW64)
  const pythonCmd = process.platform === 'win32' ? 'python' : 'python3';

  const args = ['-u', scriptPath];
  const n = parseInt(testNum, 10);
  if (Number.isInteger(n) && n > 0) args.push(String(n)); // run a single test

  let proc;
  try {
    proc = spawn(pythonCmd, args, {
      cwd: path.dirname(testsDir),   // repo root so navi_bot package is on path
      env: { ...process.env, PYTHONIOENCODING: 'utf-8' },
    });
  } catch (err) {
    event.sender.send('test:error', { file, msg: err.message });
    return;
  }

  const send = (text) => event.sender.send('test:line', { file, text });

  // Buffer chunks into whole lines and strip \r — Windows Python emits \r\n,
  // and a stray trailing \r breaks the renderer's log-format regex.
  const streamLines = (stream) => {
    let buf = '';
    stream.on('data', (d) => {
      buf += d.toString();
      const lines = buf.split(/\r?\n/);
      buf = lines.pop();
      lines.forEach(l => l && send(l));
    });
    stream.on('end', () => { if (buf.trim()) send(buf); });
  };
  streamLines(proc.stdout);
  streamLines(proc.stderr);
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

// ─────────────────────────────────────────────────────────────────────────────
//  IPC: default tests/ directory — the sibling tests/ folder of this dashboard
// ─────────────────────────────────────────────────────────────────────────────
ipcMain.handle('dialog:defaultTestsDir', () => {
  const candidate = path.join(path.dirname(__dirname), 'tests');
  return require('fs').existsSync(candidate) ? candidate : null;
});

// ─────────────────────────────────────────────────────────────────────────────
//  IPC: list a suite's test cases via `python <file> --list`
//  Resolves to an array of "TEST n: <name>" strings (empty on failure).
// ─────────────────────────────────────────────────────────────────────────────
ipcMain.handle('tests:list', (_event, { file, testsDir }) => {
  const scriptPath = path.join(testsDir, file);
  const pythonCmd = process.platform === 'win32' ? 'python' : 'python3';
  return new Promise((resolve) => {
    const { execFile } = require('child_process');
    execFile(pythonCmd, [scriptPath, '--list'], {
      cwd: path.dirname(testsDir),
      env: { ...process.env, PYTHONIOENCODING: 'utf-8' },
      timeout: 15000,
    }, (err, stdout) => {
      if (err && !stdout) return resolve([]);
      resolve(String(stdout).split(/\r?\n/).filter(l => /^TEST\s+\d+:/i.test(l)));
    });
  });
});