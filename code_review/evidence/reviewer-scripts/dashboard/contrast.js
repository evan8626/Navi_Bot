const hex = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16) / 255);
const lin = c => c <= 0.03928 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4;
const L = h => { const [r, g, b] = hex(h).map(lin); return 0.2126 * r + 0.7152 * g + 0.0722 * b; };
const ratio = (a, b) => { const [x, y] = [L(a), L(b)].sort((p, q) => q - p); return (x + 0.05) / (y + 0.05); };
const bgs = { 'bg-0': '#0e0e0c', 'bg-1': '#141412', 'bg-2': '#1a1a17', 'bg-3': '#222220' };
for (const [n, fg] of [['text-2', '#5a5852'], ['text-1', '#9a9890'], ['text-0', '#e8e6df'], ['blue', '#378add'], ['red', '#e24b4a'], ['green', '#1d9e75'], ['amber', '#ba7517']])
  console.log(n.padEnd(7), Object.entries(bgs).map(([k, b]) => `${k} ${ratio(fg, b).toFixed(2)}`).join('  '));
