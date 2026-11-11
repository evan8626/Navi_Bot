import math, time, threading
from navi_bot.utils.geometry import *
print("normalize(pi)=",normalize_angle(math.pi)," normalize(-pi)=",normalize_angle(-math.pi))
print("normalize(nan)=",normalize_angle(float('nan')))
t=time.perf_counter(); normalize_angle(1e7); print("normalize(1e7) took %.3fs"%(time.perf_counter()-t))
t=time.perf_counter(); normalize_angle(1e9); print("normalize(1e9) took %.3fs"%(time.perf_counter()-t))
# inf: run in thread with timeout
th=threading.Thread(target=normalize_angle,args=(float('inf'),),daemon=True); th.start(); th.join(2); print("normalize(inf) still running after 2s:", th.is_alive())
th=threading.Thread(target=normalize_angle,args=(1e300,),daemon=True); th.start(); th.join(2); print("normalize(1e300) still running after 2s:", th.is_alive())
for wp,res in [([(0,0),(0.15,0)],0.1),([(0,0),(0.3,0)],0.1),([(0,0),(0.25,0)],0.1),([(0,0),(0.05,0)],0.1)]:
    out=interpolate_path(wp,res); gaps=[distance(out[i-1],out[i]) for i in range(1,len(out))]
    print("interp",wp,res,"-> n=",len(out),"max gap=%.4f"%max(gaps))
print("interp duplicate wp:", interpolate_path([(0,0),(0,0),(1,0)],0.5))
sq=[(0,0),(1,0),(1,1),(0,1)]
for p in [(0,0.5),(1,0.5),(0.5,0),(0.5,1),(0,0),(1,1),(1,0),(0,1)]:
    print("pip edge/vertex",p,point_in_polygon(p,sq))
tri=[(0,0),(2,0),(1,1)]  # ray through vertex (1,1)? y=1 only vertex
print("pip ray-through-vertex (0.5,1) for triangle apex:", point_in_polygon((0.5,1),tri))
concave=[(0,0),(4,0),(4,4),(2,2),(0,4)]
print("concave notch (2,3) ->",point_in_polygon((2,3),concave)," (2,1)->",point_in_polygon((2,1),concave), " (1,2) ray through vertex (2,2)->", point_in_polygon((1,2),concave))
print("world_to_grid border 0.3 @0.1 ->",world_to_grid(0.3,0.0,(0,0),0.1), " 0.7->", world_to_grid(0.7,0.0,(0,0),0.1))
