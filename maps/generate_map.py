#!/usr/bin/env python3
"""
Warehouse map generator -> .pgm (+ optional companion .yaml)

Generates an occupancy-grid image compatible with the map_server conventions
declared in warehouse_simple.yaml (negate: 0, occupied_thresh: 0.65,
free_thresh: 0.196):

    pixel 0   (black) -> p = 1.0    -> OCCUPIED
    pixel 205 (gray)  -> p = 0.196  -> UNKNOWN  (between the thresholds)
    pixel 254 (white) -> p ~ 0.004  -> FREE

ORIENTATION — read this before writing a map publisher:
    The in-memory grid here is WORLD-oriented: grid[0][0] is the map origin
    (x=0, y=0, bottom-left), rows run along +y, columns along +x.
    PGM images put row 0 at the TOP, so the image on disk is np.flipud(grid).
    A publisher consuming the .pgm must flip back:
        grid = np.flipud(read_pgm(path))
    and may then binarize with the thresholds above before filling
    nav_msgs/OccupancyGrid.data (row-major starting from grid row 0).

Modes:
    default            built-in 10m x 10m warehouse (shelves along the sides,
                       central aisle, charging corner, pickup stations at the
                       bottom, delivery stations at the top) + N seeded random
                       boxes on the open floor.
    --layout FILE      rasterize a hand-drawn ASCII layout instead (see
                       example_layout.txt). Legend:
                           '#' occupied     '.' or ' ' free      '?' unknown
                           'C' charging     'P' pickup           'D' delivery
                       Station letters are kept free and reported as goals.
                       The FIRST text line is the TOP of the map, as drawn.

Safety guarantee: obstacles are never placed in the central aisle or on
stations, and the finished map is verified — every station must be reachable
from every other by a robot of --robot-radius (obstacles inflated, BFS).
Random scatters that break connectivity are re-rolled; persistent failure
exits with code 2 rather than emitting an unusable map.

Examples:
    python maps/generate_map.py --preview
    python maps/generate_map.py --seed 7 --n-random 10 -o maps/warehouse_7.pgm --write-yaml
    python maps/generate_map.py --layout maps/example_layout.txt -o maps/custom.pgm --preview
"""

import argparse
import sys
from collections import deque
from pathlib import Path

import numpy as np

# PGM pixel values under negate:0 with the yaml's thresholds
FREE, OCCUPIED, UNKNOWN = 254, 0, 205


# ──────────────────────────────────────────────────────────────────────────
#  Grid primitives (world-oriented: row 0 = y 0 = bottom)
# ──────────────────────────────────────────────────────────────────────────

def rect(grid, res, x0, y0, x1, y1, value):
    """Stamp value into the axis-aligned box [x0,x1) x [y0,y1) (metres)."""
    h, w = grid.shape
    c0 = max(0, int(round(x0 / res)))
    c1 = min(w, int(round(x1 / res)))
    r0 = max(0, int(round(y0 / res)))
    r1 = min(h, int(round(y1 / res)))
    grid[r0:r1, c0:c1] = value


def station_keepout(shape, stations, res, margin=0.6):
    """Boolean mask of margin-sized boxes around each station."""
    mask = np.zeros(shape, dtype=np.uint8)
    for x, y in stations.values():
        rect(mask, res, x - margin, y - margin, x + margin, y + margin, 1)
    return mask.astype(bool)


def inflate(occ, radius_px):
    """Chebyshev dilation of a boolean mask (conservative robot inflation)."""
    out = occ.copy()
    for _ in range(radius_px):
        d = out.copy()
        d[1:, :] |= out[:-1, :]
        d[:-1, :] |= out[1:, :]
        d[:, 1:] |= out[:, :-1]
        d[:, :-1] |= out[:, 1:]
        d[1:, 1:] |= out[:-1, :-1]
        d[1:, :-1] |= out[:-1, 1:]
        d[:-1, 1:] |= out[1:, :-1]
        d[:-1, :-1] |= out[1:, 1:]
        out = d
    return out


def stations_connected(grid, stations, res, robot_radius):
    """
    True if every station can reach every other through free space wide
    enough for the robot (non-free cells inflated by robot_radius, 4-conn BFS).
    """
    if len(stations) < 2:
        return True
    blocked = inflate(grid != FREE, int(np.ceil(robot_radius / res)))
    h, w = blocked.shape
    cells = {name: (int(round(y / res)), int(round(x / res)))
             for name, (x, y) in stations.items()}
    start = next(iter(cells.values()))
    if blocked[start]:
        return False
    seen = np.zeros_like(blocked)
    seen[start] = True
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not seen[nr, nc] and not blocked[nr, nc]:
                seen[nr, nc] = True
                q.append((nr, nc))
    return all(seen[cell] for cell in cells.values())


# ──────────────────────────────────────────────────────────────────────────
#  Built-in warehouse layout
# ──────────────────────────────────────────────────────────────────────────

def built_in_layout(size, res):
    """The canonical warehouse: walls, side shelving, aisle, stations."""
    n = int(round(size / res))
    grid = np.full((n, n), FREE, dtype=np.uint8)
    wall = 0.1

    # Perimeter walls
    rect(grid, res, 0, 0, size, wall, OCCUPIED)              # bottom
    rect(grid, res, 0, size - wall, size, size, OCCUPIED)    # top
    rect(grid, res, 0, 0, wall, size, OCCUPIED)              # left
    rect(grid, res, size - wall, 0, size, size, OCCUPIED)    # right

    # Shelving units along the sides (1 m deep bands, gaps for access)
    for y0, y1 in ((2.0, 4.0), (4.5, 6.5), (7.0, 8.0)):
        rect(grid, res, 0.5, y0, 1.5, y1, OCCUPIED)                # left band
        rect(grid, res, size - 1.5, y0, size - 0.5, y1, OCCUPIED)  # right band

    # Stations: kept-clear FREE zones, returned as world-coordinate goals.
    stations = {
        'charging':   (0.9, 9.0),
        'pickup_1':   (3.0, 0.9),
        'pickup_2':   (5.0, 0.9),
        'pickup_3':   (7.0, 0.9),
        'delivery_1': (3.0, 9.1),
        'delivery_2': (5.0, 9.1),
        'delivery_3': (7.0, 9.1),
    }

    # Keep-out for random obstacles: central aisle (full height) + stations.
    aisle = np.zeros((n, n), dtype=np.uint8)
    rect(aisle, res, 4.2, 0.0, 5.8, size, 1)
    protected = aisle.astype(bool) | station_keepout((n, n), stations, res)

    return grid, stations, protected


def scatter_boxes(grid, protected, res, rng, count):
    """Drop random pallet-sized boxes on the open floor, avoiding keep-outs."""
    size = grid.shape[0] * res
    placed = 0
    for _ in range(count):
        for _try in range(200):
            w = rng.uniform(0.3, 0.9)
            h = rng.uniform(0.3, 0.9)
            cx = rng.uniform(2.0, size - 2.0)
            cy = rng.uniform(1.8, size - 1.8)
            c0, c1 = int((cx - w / 2) / res), int((cx + w / 2) / res)
            r0, r1 = int((cy - h / 2) / res), int((cy + h / 2) / res)
            if protected[r0:r1, c0:c1].any():
                continue
            grid[r0:r1, c0:c1] = OCCUPIED
            placed += 1
            break
    return placed


# ──────────────────────────────────────────────────────────────────────────
#  ASCII layout mode ("draw your own map")
# ──────────────────────────────────────────────────────────────────────────

STATION_CHARS = {'C': 'charging', 'P': 'pickup', 'D': 'delivery'}

def from_ascii(text, cell, res):
    """
    Rasterize an ASCII layout. Each character covers cell x cell metres.
    The first text line is the TOP of the map (drawn as seen), so text rows
    are reversed into the world-oriented grid. Interior blank lines count as
    free space; leading/trailing blank lines are ignored.
    """
    lines = text.splitlines()
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines:
        raise ValueError('layout file is empty')
    width = max(len(ln) for ln in lines)
    lines = [ln.ljust(width) for ln in lines]
    ppc = int(round(cell / res))          # pixels per layout cell
    n_rows, n_cols = len(lines), width
    grid = np.full((n_rows * ppc, n_cols * ppc), FREE, dtype=np.uint8)
    stations, counts = {}, {}
    for tr, line in enumerate(lines):
        wr = n_rows - 1 - tr              # world row (text top = max y)
        for tc, ch in enumerate(line):
            r0, c0 = wr * ppc, tc * ppc
            if ch == '#':
                grid[r0:r0 + ppc, c0:c0 + ppc] = OCCUPIED
            elif ch == '?':
                grid[r0:r0 + ppc, c0:c0 + ppc] = UNKNOWN
            elif ch.upper() in STATION_CHARS:
                base = STATION_CHARS[ch.upper()]
                counts[base] = counts.get(base, 0) + 1
                x = (tc + 0.5) * cell     # station goal = cell centre (metres)
                y = (wr + 0.5) * cell
                stations[f'{base}_{counts[base]}'] = (round(x, 3), round(y, 3))
            # '.' and ' ' fall through as FREE
    return grid, stations


# ──────────────────────────────────────────────────────────────────────────
#  PGM I/O
# ──────────────────────────────────────────────────────────────────────────

def write_pgm(path, grid, comment=''):
    """Write the world-oriented grid as binary PGM (P5), flipped to image order."""
    img = np.flipud(grid).astype(np.uint8)
    h, w = img.shape
    header = f'P5\n# {comment}\n{w} {h}\n255\n'.encode('ascii')
    with open(path, 'wb') as f:
        f.write(header + img.tobytes())


def read_pgm(path):
    """Read a binary (P5) or ascii (P2) PGM. Returns the IMAGE array
    (row 0 = top); callers wanting the world grid should np.flipud() it."""
    data = Path(path).read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:                # magic, width, height, maxval
        if data[i:i + 1] == b'#':         # comment: skip to end of line
            i = data.index(b'\n', i) + 1
            continue
        if data[i:i + 1].isspace():
            i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h = tokens[0], int(tokens[1]), int(tokens[2])
    if magic == b'P5':
        img = np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8)
    elif magic == b'P2':
        img = np.array(data[i:].split()[:w * h], dtype=np.uint8)
    else:
        raise ValueError(f'not a PGM file: {magic!r}')
    return img.reshape(h, w)


def write_yaml(path, image_name, res, stations):
    lines = [
        f'image: {image_name}',
        f'resolution: {res}',
        'origin: [0.0, 0.0, 0.0]',
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.196',
        '',
        '# Station goals (world metres, cell centres) — usable as mission coordinates:',
    ]
    lines += [f'#   {name}: ({x}, {y})' for name, (x, y) in sorted(stations.items())]
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


# ──────────────────────────────────────────────────────────────────────────
#  Preview
# ──────────────────────────────────────────────────────────────────────────

def preview(grid, stations, res, cols=50):
    """ASCII rendering (top-down, like the PGM), stations overlaid."""
    h, w = grid.shape
    step = max(1, round(h / cols))
    rows_out = []
    for r1 in range(h, 0, -step):                     # top of map first
        r0 = max(0, r1 - step)
        row = ''
        for c0 in range(0, w, step):
            block = grid[r0:r1, c0:c0 + step]
            row += ('#' if (block == OCCUPIED).any()
                    else '?' if (block == UNKNOWN).any() else '.')
        rows_out.append(list(row))
    for name, (x, y) in stations.items():
        rr = len(rows_out) - 1 - int(y / res / step)  # flip to display order
        cc = int(x / res / step)
        if 0 <= rr < len(rows_out) and 0 <= cc < len(rows_out[rr]):
            rows_out[rr][cc] = name[0].upper()
    return '\n'.join(''.join(r) for r in rows_out)


# ──────────────────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────────────────

def main(argv=None):
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description='Warehouse occupancy-map generator (.pgm)',
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('-o', '--out', type=Path, default=here / 'warehouse_simple.pgm',
                    help='output .pgm path')
    ap.add_argument('--layout', type=Path, default=None,
                    help='ASCII layout file (overrides the built-in warehouse)')
    ap.add_argument('--seed', type=int, default=42, help='RNG seed for random boxes')
    ap.add_argument('--n-random', type=int, default=None,
                    help='random boxes to scatter (default: 6 built-in, 0 with --layout)')
    ap.add_argument('--size', type=float, default=10.0, help='map side length (m)')
    ap.add_argument('--resolution', type=float, default=0.05, help='metres per pixel')
    ap.add_argument('--cell', type=float, default=0.5, help='metres per ASCII layout character')
    ap.add_argument('--robot-radius', type=float, default=0.25,
                    help='inflation radius for the connectivity guarantee (m)')
    ap.add_argument('--write-yaml', action='store_true',
                    help='write a companion map_server .yaml next to the .pgm')
    ap.add_argument('--preview', action='store_true', help='print an ASCII preview')
    args = ap.parse_args(argv)

    res = args.resolution
    rng = np.random.default_rng(args.seed)

    if args.layout:
        n_random = 0 if args.n_random is None else args.n_random
        grid, stations = from_ascii(args.layout.read_text(encoding='utf-8'), args.cell, res)
        if n_random:
            protected = station_keepout(grid.shape, stations, res) | (grid != FREE)
            scatter_boxes(grid, protected, res, rng, n_random)
        if not stations_connected(grid, stations, res, args.robot_radius):
            print('ERROR: stations are not mutually reachable in this layout '
                  f'(robot radius {args.robot_radius} m).', file=sys.stderr)
            return 2
    else:
        n_random = 6 if args.n_random is None else args.n_random
        # Re-roll the scatter until the connectivity guarantee holds.
        for _attempt in range(25):
            grid, stations, protected = built_in_layout(args.size, res)
            scatter_boxes(grid, protected | (grid != FREE), res, rng, n_random)
            if stations_connected(grid, stations, res, args.robot_radius):
                break
        else:
            print('ERROR: could not scatter obstacles without cutting off a '
                  'station after 25 attempts — lower --n-random.', file=sys.stderr)
            return 2

    comment = (f'generated by generate_map.py seed={args.seed} '
               f'n_random={n_random} layout={args.layout or "built-in"}')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(args.out, grid, comment)

    # Round-trip verification: what we wrote is exactly what a reader gets back.
    back = np.flipud(read_pgm(args.out))
    assert np.array_equal(back, grid), 'PGM round-trip mismatch — file is corrupt'

    if args.write_yaml:
        write_yaml(args.out.with_suffix('.yaml'), args.out.name, res, stations)

    h, w = grid.shape
    print(f'wrote {args.out}  ({w}x{h} px, {w*res:g}x{h*res:g} m @ {res} m/px)')
    print(f'occupied: {(grid == OCCUPIED).mean():.1%}   unknown: {(grid == UNKNOWN).mean():.1%}')
    if stations:
        print('stations (world metres — usable as mission goals):')
        for name, (x, y) in sorted(stations.items()):
            print(f'  {name:<12} x={x:<5g} y={y:<5g}  -> grid cell '
                  f'(row {int(y/res)}, col {int(x/res)})')
        print(f'connectivity: all stations mutually reachable '
              f'(robot radius {args.robot_radius} m) OK')
    if args.preview:
        print()
        print(preview(grid, stations, res))
    return 0


if __name__ == '__main__':
    sys.exit(main())
