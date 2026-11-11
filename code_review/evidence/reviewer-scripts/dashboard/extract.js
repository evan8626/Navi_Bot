// Extract the inline <script> from index.html and eval only pure functions by line range
const fs=require('fs');
const src=fs.readFileSync('C:/Users/evan8/Desktop/Navi Bot/telemetry_dashboard/index.html','utf8').split('\n');
function lines(a,b){return src.slice(a-1,b).join('\n');}
module.exports={lines};
