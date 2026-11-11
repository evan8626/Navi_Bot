const path = require('path');
// main.js line 49: scriptPath = path.join(testsDir, file); cwd = path.dirname(testsDir)
const unc = String.raw`\\attacker\share\tests`;
console.log('UNC testsDir   ->', path.win32.join(unc, 'x.py'), '| cwd', path.win32.dirname(unc));
console.log('traversal file ->', path.win32.join(String.raw`C:\Users\me\Navi Bot\tests`, String.raw`..\..\Downloads\evil.py`));
console.log('abs file       ->', path.win32.join(String.raw`C:\Users\me\Navi Bot\tests`, String.raw`C:\evil.py`));
