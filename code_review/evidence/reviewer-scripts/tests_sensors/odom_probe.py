import math
from navi_bot.sensors.odometry import WheelOdometry
o = WheelOdometry()
x,y,th,v,w = o.update(1000,1100,1.0)
dl=2*math.pi*0.1; dr=dl*1.1; ds=(dl+dr)/2; dth=(dr-dl)/0.4; R=ds/dth
print("impl  x=%.4f y=%.4f th=%.4f"%(x,y,th))
print("exact x=%.4f y=%.4f"%(R*math.sin(dth), R*(1-math.cos(dth))))
print("midpt x=%.4f y=%.4f"%(ds*math.cos(dth/2), ds*math.sin(dth/2)))
# non-zero initial encoder
o=WheelOdometry(); print("first update with encoders already at 50000 ->", o.update(50000,50000,0.1)[:2])
# 16-bit wrap
o=WheelOdometry(); o.update(32760,32760,0.1); print("int16 wrap 32760->-32766:", o.update(-32766,-32766,0.1)[:2])
# reset normalisation / velocity
o=WheelOdometry(); o.update(1000,1000,1.0); o.reset(theta=10.0); print("reset(theta=10) ->", o.get_pose(), "vel kept", o.get_velocity())
# negative dt
o=WheelOdometry(); o.update(100,100,1.0); print("dt=-1 ->", o.update(200,200,-1.0))
