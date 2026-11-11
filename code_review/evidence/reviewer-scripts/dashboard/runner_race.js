// Headless repro of the Tests-tab runner (index.html lines 471-1364) with a universal DOM stub
// and a fake window.navi that mimics main.js's IPC (runningProcs keyed by file).
const {lines} = require('./extract.js');
function stub() {
  const f = function () { return stub(); };
  return new Proxy(f, {
    get(t, k) {
      if (k === Symbol.toPrimitive) return () => 0;
      if (k === 'children') return t.__children || (t.__children = []);
      if (k in t) return t[k];
      if (k === 'appendChild') return (c) => { (t.__children = t.__children || []).push(c); return c; };
      if (k === 'textContent' || k === 'innerHTML' || k === 'className') return t['__' + k] || '';
      return stub();
    },
    set(t, k, v) { if (k === 'textContent' || k === 'innerHTML' || k === 'className') t['__' + k] = v; else t[k] = v; return true; },
  });
}
const els = {};
global.document = { getElementById: (id) => els[id] || (els[id] = stub()), createElement: () => stub(), querySelectorAll: () => [] };
global.localStorage = { getItem: () => 'C:/repo/tests', setItem() {} };
global.ResizeObserver = class { observe() {} };
global.performance = { now: () => Date.now() };
global.requestAnimationFrame = () => 0; global.cancelAnimationFrame = () => {};

// ---- fake main process -------------------------------------------------------------------
const listeners = {};
const runningProcs = {}; let pid = 0; const procs = [];
function mainRun(file) {
  const p = { id: ++pid, file, alive: true };
  runningProcs[file] = p; procs.push(p);
  return p;
}
function mainClose(p, code) { delete runningProcs[p.file]; listeners.done({ file: p.file, code }); }   // main.js:87
global.window = {
  navi: {
    runTest: (file) => { mainRun(file); },
    listTests: async () => [],
    stopTest: (file) => { const p = runningProcs[file]; if (p) p.alive = false; },               // main.js:41-46
    pickDir: async () => null, defaultTestsDir: async () => null,
    onTestLine: (cb) => listeners.line = cb, onTestDone: (cb) => listeners.done = cb, onTestError: (cb) => listeners.err = cb,
  },
};
eval(lines(471, 1364) + '\nsetTestsDir("C:/repo/tests"); globalThis.R={get activeRun(){return activeRun}, runResults, suiteCases, startRun, stopRun, selectFile};');
const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const emit = (p, text) => listeners.line({ file: p.file, text });

(async () => {
  console.log('--- Scenario 1: stop A, start B, then A\'s killed process finally closes');
  R.selectFile('AStar_planner_test.py');
  R.startRun('AStar_planner_test.py'); const pA = procs.at(-1);
  emit(pA, 'INFO:__main__:TEST 1: first'); await sleep(100);
  R.stopRun();
  console.log('after stop: activeRun =', R.activeRun, ' A.exit =', R.runResults['AStar_planner_test.py'].exit);
  R.startRun('geometry_test.py'); const pB = procs.at(-1);
  console.log('B started: activeRun =', R.activeRun);
  emit(pA, 'WARNING:__main__:  FAIL: late line from the killed A process');  // late stdout from A
  mainClose(pA, 1);                                                          // A's 'close'
  await sleep(200);
  console.log('after A close: activeRun =', R.activeRun, '(B is still running, pid', pB.id, ')');
  console.log('A finished result now has lines:', R.runResults['AStar_planner_test.py'].lines.map(l => l.text));
  console.log('A case verdicts:', JSON.stringify(R.suiteCases['AStar_planner_test.py'].map(c => [c.num, c.passed])));
  R.startRun('kinematics_test.py');
  console.log('a THIRD run was allowed to start concurrently with B:', R.activeRun === 'kinematics_test.py', '| procs alive:', procs.filter(p => p.alive && p !== pA).map(p => p.file));
  mainClose(pB, 0); mainClose(procs.at(-1), 0); await sleep(300);

  console.log('--- Scenario 2: stop A then immediately re-run A (same file)');
  const F = 'DWA_planner_test.py';
  R.selectFile(F);
  R.startRun(F); const p1 = procs.at(-1);
  emit(p1, 'INFO:__main__:TEST 1: one'); await sleep(50);
  R.stopRun();
  R.startRun(F); const p2 = procs.at(-1);
  console.log('runningProcs[F] is new proc?', runningProcs[F] === p2);
  mainClose(p1, 1);  // old proc closes AFTER the new one registered
  console.log('runningProcs[F] after old close:', runningProcs[F]);
  await sleep(150); console.log("NEW run finalized by OLD close? exit =", R.runResults[F].exit, " activeRun =", R.activeRun, " new proc still alive:", p2.alive); R.startRun(F); console.log("Stop now cannot reach p2:"); R.stopRun();
  console.log('new proc killed by Stop?', !p2.alive);
})();
