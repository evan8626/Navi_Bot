const {lines}=require('./extract.js');
eval(lines(476,598)+'\n'+lines(804,831)+'\nglobalThis.F={MAPS,parseLine,classForLine,isPass,isFail,isSuiteSummary,groupIntoCases,inferMapKey,inferStartGoal,inferPath,inferPoseTrail};');
const P=s=>F.parseLine(s);
console.log('--- single-line verdicts: isPass / isFail / classForLine');
for (const t of [
 'INFO:__main__:  FAIL: only 2/3 waypoints reached',
 'INFO:__main__:  PASS: visited 3/3 waypoints',
 'INFO:__main__:Retries: 0 failed attempts',
 'INFO:__main__:Engaging fail-safe stop',
 'INFO:__main__:Robot will pass the obstacle on the left',
 'INFO:__main__:Loading map from /data/pass/warehouse.pgm',
 'INFO:__main__:  FAIL: robot did not pass the waypoint',
 'INFO:__main__:  PASS: no planner call failed',
 'INFO:__main__:TEST 19: Planning — path must not pass through occupied cells',
 'INFO:__main__:  FAIL off by 1/2 cell',
]) { const p=P(t); console.log(JSON.stringify(p.text).padEnd(66),'isPass',F.isPass(p.text),'isFail',F.isFail(p.text),'cls',F.classForLine(p)); }
console.log('--- case verdict, last-match-wins');
const mk=a=>F.groupIntoCases(a.map(P)).map(c=>c.passed);
console.log('FAIL then later chatter:', mk(['INFO:__main__:TEST 1: obstacle','WARNING:__main__:  FAIL: collided at (3, 4)','INFO:__main__:Robot passed waypoint 2 while recovering']));
console.log('crash traceback no verdict:', mk(['INFO:__main__:TEST 1: x','Traceback (most recent call last):','  File "t.py", line 3, in <module>','ZeroDivisionError: division by zero']));
console.log('parseLine unanchored Error:', JSON.stringify(P('checking that ValueError is raised for bad input')));
console.log('--- start/goal');
for (const t of ['Restarting planner: 3, 2','Heading to goal: 1.57 rad','Goal error (m): 0.12, 0.05','goal_tolerance=(0.4, 0.4)','Kickstart (2 3)']) console.log(JSON.stringify(t).padEnd(34), JSON.stringify(F.inferStartGoal([P(t)])));
console.log('--- inferPath picks up warnings');
console.log(JSON.stringify(F.inferPath(['INFO:__main__:TEST 3: invalid start','WARNING:navi_bot.planners.astar:Start coords are X: 9, Y: 9','INFO:__main__:X: 0, Y: 0','INFO:__main__:X: 1, Y: 1'].map(P))));
console.log('float X/Y truncated:', JSON.stringify(F.inferPath(['X: 2.75, Y: 3.9','X: 3.1, Y: 4.0'].map(P))));
console.log('pose with spaces:', JSON.stringify(F.inferPoseTrail(['pose=(1.0, 2.0)','pose=(1.5, 2.5)'].map(P))), ' sci-notation:', JSON.stringify(F.inferPoseTrail(['pose=(1e-3,2.0)','pose=(1.5,2.5)'].map(P))));
console.log('--- inferMapKey');
for (const [txt,file] of [['Start: (0,0) Goal: (7,7) on a map with no obstacles','AStar_planner_test.py'],['entire map clear; obstacle avoidance not needed','AStar_planner_test.py'],['Start (2,2) goal (12,15) open floor','AStar_planner_test.py'],['Test mission state machine','foo_test.py']]) console.log(JSON.stringify(txt).padEnd(56), file, '->', F.inferMapKey([P(txt)],file));
