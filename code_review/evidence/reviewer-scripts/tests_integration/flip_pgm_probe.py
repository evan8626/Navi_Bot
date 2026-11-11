import sys, os, tempfile, logging, builtins
logging.disable(logging.CRITICAL)
sys.path.insert(0, 'tests'); sys.path.insert(0, 'maps')
import numpy as np
import warehouse_map_test as w
from navi_bot.map_publisher import read_pgm, parse_map_yaml, classify

# 1) would warehouse TEST 2 / map_publisher TEST 5 notice a vertical flip?
good = w.binarize(w.load_world_grid())
flipped = good[::-1]
print('stations free in flipped map:', all(flipped[w.station_cell(n)] == 0 for n in w.STATIONS))
print('border walled in flipped:', bool(np.concatenate([flipped[0], flipped[-1], flipped[:, 0], flipped[:, -1]]).all()))
print('flipped[0][0]=', flipped[0][0], 'flipped[18][60]=', flipped[18][60])
tr = good.T
print('stations free in transposed map:', all(tr[w.station_cell(n)] == 0 for n in w.STATIONS))

tmp = tempfile.mkdtemp()
def wr(name, data):
    p = os.path.join(tmp, name); open(p, 'wb').write(data); return p
def tryit(label, fn):
    try: print(label, '->', fn())
    except Exception as e: print(label, '-> EXC', type(e).__name__, e)
# 2) maxval != 255: maxval 100, value 100 == white/free
p = wr('m100.pgm', b'P5\n2 1\n100\n' + bytes([100, 0]))
tryit('P5 maxval=100 [100,0] classify', lambda: classify(read_pgm(p), {'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}).tolist())
p = wr('m16.pgm', b'P5\n2 1\n65535\n' + bytes([255, 255, 0, 0]))
tryit('P5 16-bit 2x1', lambda: read_pgm(p).tolist())
p = wr('p2c.pgm', b'P2\n2 1\n255\n# comment\n254 0\n')
tryit('P2 with raster comment', lambda: read_pgm(p).tolist())
p = wr('trunc.pgm', b'P5\n4 4\n255\n' + bytes(5))
tryit('P5 truncated', lambda: read_pgm(p).tolist())
y = os.path.join(tmp, 'q.yaml'); open(y, 'w').write('image: "warehouse.pgm"\nresolution: 0.05\nnegate: 0.0\n')
tryit('yaml quoted image + negate 0.0', lambda: parse_map_yaml(y))
y = os.path.join(tmp, 'q2.yaml'); open(y, 'w').write("image: 'map.pgm'\nresolution: 0.05\n")
tryit('yaml single-quoted image', lambda: parse_map_yaml(y)['image'])
