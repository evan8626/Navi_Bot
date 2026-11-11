// Runs the Demo-tab planners from index.html (lines 1476-1987) headless with DOM stubs.
const {lines} = require('./extract.js');
const stub = `
const document={getElementById:(id)=>({value:globalThis.__vals[id], classList:{add(){},remove(){}}, textContent:''})};
globalThis.__vals={'coord-sx':'1','coord-sy':'1','coord-gx':'18','coord-gy':'18'};
const performance={now:()=>Date.now()};
let STATS={}, LOG=[];
function setDStat(id,v){STATS[id]=v;} function demoLog(m){LOG.push(m);} function stopDemo(){}
function trailHeading(){return null;}
`;
// take lines 1476-1601 (consts + draw), skip demoLog/setDStat (1603-1614) and keep 1616-1987
const code = stub + lines(1476, 1507) + '\n' + lines(1616, 1676) + '\n' + lines(1678, 1987) +
  '\nglobalThis.D={BASE_MAP,buildDemoState,stepAstar,stepDstar,stepDwa,STATS:()=>STATS,LOG:()=>LOG};';
eval(code);

function cost(path) { let c = 0; for (let i = 1; i < path.length; i++) c += Math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]); return c; }
function cornerCuts(path, grid) { let n = 0, cuts=[]; for (let i = 1; i < path.length; i++) { const [r0,c0]=path[i-1],[r1,c1]=path[i]; if (r0!==r1&&c0!==c1&&(grid[r1][c0]||grid[r0][c1])) { n++; cuts.push(`${r0},${c0}->${r1},${c1}`);} } return [n,cuts]; }

function runAstar(start, goal, n) {
  const s = D.buildDemoState('astar', start, goal);
  for (let k = 0; k < 100000 && s.phase !== 'follow' && !s.done; k++) D.stepAstar(s, n);
  return s;
}
console.log('=== A*: open list sorted once per tick, n pops per tick');
const pairs = [[[1,1],[18,18]], [[18,1],[1,18]], [[9,9],[2,17]], [[1,10],[18,10]], [[12,2],[5,16]], [[16,9],[4,12]]];
for (const [st, gl] of pairs) {
  const r = [1, 3, 20].map(n => { const s = runAstar(st, gl, n); return `n=${n}: cost ${cost(s.path).toFixed(3)} wp ${s.path.length} d-len ${D.STATS()['d-len']}`; });
  console.log(JSON.stringify(st), '->', JSON.stringify(gl), '|', r.join(' | '));
}
let total = 0, examples = [];
for (let r0 = 1; r0 < 19; r0++) for (let c0 = 1; c0 < 19; c0++) {
  if (D.BASE_MAP[r0][c0]) continue;
  const s = runAstar([r0, c0], [18, 18], 1); if (!s.path) continue;
  const [n, cuts] = cornerCuts(s.path, D.BASE_MAP); if (n) { total++; if (examples.length < 3) examples.push(`(${r0},${c0}): ${cuts.join(' ')}`); }
}
console.log('A* paths (n=1) to (18,18) that cut an obstacle corner:', total, examples);
let worse = 0, tried = 0, worst = null;
for (let r0 = 1; r0 < 19; r0++) for (let c0 = 1; c0 < 19; c0++) {
  if (D.BASE_MAP[r0][c0] || (r0===18&&c0===18)) continue; tried++;
  const a = runAstar([r0,c0],[2,17],1), b = runAstar([r0,c0],[2,17],20);
  const d = cost(b.path) - cost(a.path); if (d > 1e-9) { worse++; if (!worst || d > worst[1]) worst = [[r0,c0], d, cost(a.path), cost(b.path)]; }
}
console.log(`A* speed 20 returned a longer path than speed 1 in ${worse}/${tried} starts to (2,17); worst`, JSON.stringify(worst));

console.log('=== D* Lite demo');
{
  const s = D.buildDemoState('dstar', [1,1], [18,18]);
  for (let k = 0; k < 200000 && !s.done; k++) D.stepDstar(s, 3);
  console.log('done', s.done, 'replans', s.replans, 'final path wp', s.path && s.path.length, 'cost', s.path && cost(s.path).toFixed(3), 'log tail', D.LOG().slice(-3));
  const [n, cuts] = cornerCuts(s.originalPath, D.BASE_MAP); console.log('initial D* path corner cuts', n, cuts.slice(0,3));
}
console.log('=== DWA demo');
for (const [st, gl] of [[[1,1],[18,18]], [[9,9],[5,9]], [[8,9],[4,9]], [[5,15],[5,11]]]) {
  const s = D.buildDemoState('dwa', st, gl); let k = 0;
  for (; k < 20000 && !s.done; k++) D.stepDwa(s);
  const realLen = cost(s.trail);
  console.log(JSON.stringify(st), '->', JSON.stringify(gl), 'done', s.done, 'steps', k, 'status', D.STATS()['d-status'], 'd-len', D.STATS()['d-len'], 'true len(cells)', realLen.toFixed(2), 'replans', s.replans, 'stuck', s.stuckCount, 'pos', [s.floatR.toFixed(2), s.floatC.toFixed(2)]);
}
