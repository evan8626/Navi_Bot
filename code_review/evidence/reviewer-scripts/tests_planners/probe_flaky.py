import sys, logging
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
import numpy as np
import DWA_planner_test as W
import DStar_Lite_planner_test as D
logging.disable(logging.CRITICAL)
for name, fn, n in [("DWA4", W.test_valid_path, 150), ("DWA6", W.test_valid_path_facing_away, 150), ("D7", D.test_random_coords_clear_map, 200), ("D8", D.test_random_coords_obstacle_map, 200), ("D9", D.test_random_coords_on_moving_map, 200)]:
    fails=[]
    for seed in range(n):
        np.random.seed(seed)
        if not fn(): fails.append(seed)
    print(name, "runs", n, "fails", len(fails), fails[:10])
