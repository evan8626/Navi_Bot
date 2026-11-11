import sys, cProfile, pstats, logging, time
sys.path.insert(0,'.'); logging.disable(logging.CRITICAL)
import numpy as np
import navi_bot.planners.dstar_lite as dl
dl.heuristic_backward = lambda a, b: max(abs(a[0]-b[0]), abs(a[1]-b[1]))
G = np.zeros((200,200),dtype=int); d = dl.DStarLitePlanner(); d.set_occupancy_grid(G)
t0=time.perf_counter(); p = d.plan((0,0),(199,199)); print('initial (chebyshev)', round(1000*(time.perf_counter()-t0)), 'ms')
G2 = G.copy(); G2[50:60, 40:60] = 1; d.set_occupancy_grid(G2)
t0=time.perf_counter(); e = d.edge_changed(); print('edge_changed alone', round(1000*(time.perf_counter()-t0)), 'ms', len(e), 'edges')
pr = cProfile.Profile(); pr.enable(); d.plan(p[1],(199,199)); pr.disable()
pstats.Stats(pr).sort_stats('tottime').print_stats(6)
