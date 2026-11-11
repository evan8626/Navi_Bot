"""Check the scan_matching_test fixtures are solvable by a textbook point-to-point ICP
(SVD/Kabsch, accumulated transform, column-vector rotation), and probe scan_match quirks."""
import math, sys
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
import scan_matching_test as T
from navi_bot.sensors.lidar_processing import LidarProcessor
from navi_bot.utils.geometry import transform_point


def ref_icp(cur, ref, guess=(0, 0, 0), iters=60):
    ref = np.asarray(ref, float); cur = np.asarray(cur, float)
    tree = cKDTree(ref)
    x, y, th = guess
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        moved = cur @ np.array([[c, -s], [s, c]]).T + [x, y]
        d, idx = tree.query(moved)
        keep = d < np.percentile(d, 90) + 1e-12
        A, B = cur[keep], ref[idx[keep]]
        ma, mb = A.mean(0), B.mean(0)
        H = (A - ma).T @ (B - mb)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1; R = Vt.T @ U.T
        t = mb - R @ ma
        x, y, th = t[0], t[1], math.atan2(R[1, 0], R[0, 0])
    return (x, y, th)

room = T.synthetic_room()
print("points in room:", len(room))
cases = [((0.2, -0.1, 0.0), None), ((0.0, 0.0, math.radians(8)), None),
         ((0.15, 0.1, math.radians(5)), None),
         ((1.0, 0.6, math.radians(25)), (0.95, 0.55, math.radians(23)))]
for tt, prior in cases:
    cur = T.make_current(room, tt)
    got = ref_icp(cur, room, prior or (0, 0, 0))
    ok, dl, da = T.transform_close(got, tt, 0.02, 0.5)
    print(f"reference ICP true={tuple(round(v,4) for v in tt)} got={tuple(round(float(v),4) for v in got)} ok={ok}")
# partial overlap
tt = (0.1, -0.05, math.radians(4)); full = T.make_current(room, tt)
got = ref_icp(full[: int(len(full) * 0.6)], room)
print("partial (60%) ->", tuple(round(float(v), 4) for v in got), T.transform_close(got, tt, 0.06, 3.0)[0])

# Probe source quirks
p = LidarProcessor()
# 1) row-vector rotation: cur @ R rotates by -theta
th = 0.3; R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
v = np.array([[1.0, 0.0]])
print("row-vector cur@R for theta=+0.3 gives", np.round(v @ R, 4), "(CCW should give", np.round(R @ v[0], 4), ")")
# 2) initial_guess ignored when current == reference
(res, err) = p.scan_match(room, room, initial_guess=(5.0, 5.0, 1.0))
print("identity scans with initial_guess=(5,5,1) ->", tuple(round(float(v), 4) for v in res))
# 3) max_iterations=0
try:
    p.scan_match(room, room, max_iterations=0)
except Exception as e:
    print("max_iterations=0 ->", type(e).__name__, e)
# 4) bearing-difference wrap: point just across the +-pi seam
cur = [(-1.0, -0.001)]; ref = [(-1.0, 0.001)]
d = math.atan2(0.001, -1.0) - math.atan2(-0.001, -1.0)
print("bearing diff across seam (true ~0.002 rad):", round(d, 4))
