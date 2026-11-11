const {lines}=require('./extract.js');
const fs=require('fs');
eval(lines(476,598)+'\n'+lines(804,831)+'\nglobalThis.F={MAPS,parseLine,groupIntoCases,resolveMapKey,inferStartGoal,inferPath,inferPoseTrail};');
const dir='C:/Users/evan8/AppData/Local/Temp/claude/738ef417e18052f3/0a4a6808-43fe-4936-9d58-6cdc90f47f1a/scratchpad/testruns/';
for (const f of fs.readdirSync(dir).filter(f=>f.endsWith('.log'))) {
  const file=f.replace('.log','.py');
  const cases=F.groupIntoCases(fs.readFileSync(dir+f,'utf8').split(/\r?\n/).filter(Boolean).map(F.parseLine));
  for (const c of cases) {
    const path=F.inferPath(c.lines), pose=F.inferPoseTrail(c.lines);
    if (!path&&!pose) continue;
    const k=F.resolveMapKey(c.lines,file); const m=F.MAPS[k];
    const sg=F.inferStartGoal(c.lines);
    const tr=pose||path;
    let bad=[];
    for (const [x,y] of tr){ const r=Math.round(x),cc=Math.round(y); if(r<0||cc<0||r>=m.rows||cc>=m.cols) bad.push('OOB'+r+','+cc); else if(m.grid[r][cc]) bad.push('OBS'+r+','+cc); }
    console.log(file.slice(0,26).padEnd(27),c.header.slice(0,48).padEnd(49),k.padEnd(10),'S',JSON.stringify(sg.start),'G',JSON.stringify(sg.goal),pose?'pose':'path',tr.length, bad.length?'BAD '+[...new Set(bad)].slice(0,4).join(' '):'');
  }
}
