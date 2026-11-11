const {lines}=require('./extract.js');
const fs=require('fs');
eval(lines(476,598)+'\n'+lines(804,831)+'\nglobalThis.F={parseLine,classForLine,isTestHeader,isSuiteSummary,isPass,isFail,groupIntoCases,inferMapKey,inferLoggedGrid,inferStartGoal,inferPath,inferPoseTrail,inferHazardTrail,inferPointCloud,startRe,goalRe};');
const dir='C:/Users/evan8/AppData/Local/Temp/claude/738ef417e18052f3/0a4a6808-43fe-4936-9d58-6cdc90f47f1a/scratchpad/testruns/';
for (const f of fs.readdirSync(dir).filter(f=>f.endsWith('.log'))) {
  const raw=fs.readFileSync(dir+f,'utf8').split(/\r?\n/).filter(Boolean);
  const parsed=raw.map(F.parseLine);
  const cases=F.groupIntoCases(parsed);
  const p=cases.filter(c=>c.passed===true).length, fl=cases.filter(c=>c.passed===false).length, nv=cases.filter(c=>c.passed===null).length;
  const summ=raw.find(l=>/Results:/.test(l));
  console.log(f.padEnd(34), 'cases',cases.length,'pass',p,'fail',fl,'noverdict',nv,'|',summ&&summ.replace(/.*Results: /,''));
  cases.filter(c=>c.passed===null).forEach(c=>console.log('    NOVERDICT:',c.header.slice(0,90)));
}
