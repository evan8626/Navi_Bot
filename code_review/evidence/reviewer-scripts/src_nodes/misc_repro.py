import sys, math, warnings, time
sys.path.insert(0, '.')
warnings.simplefilter('ignore')
import numpy as np
from navi_bot.sensors.lidar_processing import LidarProcessor

print('=== scan_match ===')
lp = LidarProcessor()
rng = np.random.default_rng(0)
# L-shaped room corner points
ref = np.array([(x, 2.0) for x in np.linspace(-2, 2, 80)] + [(2.0, y) for y in np.linspace(-2, 2, 80)])
def apply(pts, dx, dy, th):
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return pts @ R.T + np.array([dx, dy])
# pure rotation of +0.1 rad: cur = R(-0.1) ref  => true transform cur->ref is +0.1 rad
cur = apply(ref, 0, 0, -0.1)
(dx, dy, dth), err = lp.scan_match(cur, ref)
print(f'true (0,0,+0.100) -> returned ({dx:.3f},{dy:.3f},{dth:+.3f}) err {err:.4f}')
cur = apply(ref, -0.3, 0.0, 0.0)
(dx, dy, dth), err = lp.scan_match(cur, ref)
print(f'true (+0.300,0,0) -> returned ({dx:.3f},{dy:.3f},{dth:+.3f})  (only the LAST iteration increment is returned)')
# rotation direction of np.dot(cur, R): row-vector times R rotates by -theta
p = np.array([[1.0, 0.0]]); th = 0.5
R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
print('np.dot([1,0], R(+0.5)) =', np.round(np.dot(p, R), 3), '-> angle', round(math.atan2(*np.dot(p, R)[0][::-1]), 3), '(rotates by -theta)')
# initial guess ignored
(dx1, dy1, dt1), _ = lp.scan_match(apply(ref, -0.3, 0, 0), ref, initial_guess=(0, 0, 0))
(dx2, dy2, dt2), _ = lp.scan_match(apply(ref, -0.3, 0, 0), ref, initial_guess=(5.0, 5.0, 1.0))
print('initial_guess (0,0,0) vs (5,5,1) give identical result:', np.allclose((dx1, dy1, dt1), (dx2, dy2, dt2)))
try:
    lp.scan_match(cur, ref, max_iterations=0)
except Exception as e:
    print('max_iterations=0 ->', type(e).__name__, e)
# angle averaging across the +-pi seam
a = np.array([[-1.0, 0.01]]); b = np.array([[-1.0, -0.01]])
print('per-point bearing diff across seam:', (np.arctan2(a[:, 1], a[:, 0]) - np.arctan2(b[:, 1], b[:, 0]))[0], '(true diff 0.02)')

print('=== odometry ===')
from navi_bot.sensors.odometry import WheelOdometry
o = WheelOdometry()
# drive a full circle of radius 1 m in N steps: left/right arc lengths
R, L = 1.0, 0.4
for N in (10, 36, 100):
    o = WheelOdometry(); circ = 2*math.pi
    tl = tr = 0.0
    per = 1/(2*math.pi*0.1) * 1000   # ticks per metre
    for i in range(N):
        tl += (R - L/2)*circ/N * per; tr += (R + L/2)*circ/N * per
        o.update(tl, tr, 0.1)
    print(f'circle R=1 m in {N} updates: end pose ({o.x:+.3f},{o.y:+.3f}) err {math.hypot(o.x, o.y):.3f} m (exact: 0,0)')
o = WheelOdometry()
x, y, th, v, w = o.update(123456, 123456, 0.02)
print('first update with encoders already at 123456 ticks -> x jumps to', round(x, 2), 'm, v =', round(v, 1), 'm/s')
o = WheelOdometry(); o.update(1000, 1000, 0.1); o.reset()
x, y, *_ = o.update(1000, 1000, 0.1)
print('reset() keeps prev ticks:', o.prev_left_ticks, '-> next update after reset x =', x)
o = WheelOdometry(); o.update(65530, 65530, 0.1)
x0 = o.x; o.update(4, 4, 0.1)   # 16-bit wrap: +10 ticks
print('16-bit counter wrap 65530 -> 4 (+10 ticks): dx =', round(o.x - x0, 3), 'm (expected +0.006)')
o = WheelOdometry(); o.update(0, 0, 0.1); o.update(100, 100, 0.1); vel = o.linear_velocity
o.update(200, 200, 0.0)
print('dt=0: pose advances but velocity stays stale', vel, '==', o.linear_velocity)

print('=== imu ===')
try:
    from navi_bot.sensors.imu_processor import ImuProcessor
    ImuProcessor()
except Exception as e:
    print('ImuProcessor() ->', type(e).__name__, e)
import navi_bot.sensors.imu_processor as im
from collections import deque
class Obj: pass
imu = im.ImuProcessor.__new__(im.ImuProcessor)
imu.alpha = 0.98; imu.orientation = 1.0; imu.angular_velocity = 0.0
for _ in range(100): imu.update_orientation(0.01)
print('stationary (gyro=0) heading 1.0 rad after 1 s at 100 Hz ->', round(imu.orientation, 4), '(should stay 1.0)')
imu.linear_acceleration = np.array([0.0, 0.0, 9.81])
print('detect_motion() at rest with gravity on z ->', imu.detect_motion())

print('=== profiler ===')
from navi_bot.utils.profiler import PerformanceProfiler, profile, global_profiler
p = PerformanceProfiler(history_size=100)
for _ in range(250):
    p.start_timing('t'); p.stop_timing('t', deadline=0.0)
s = p.get_statistics('t')
print('250 misses with history 100 -> miss_rate', s['deadline_miss_rate'], 'count', s['count'])
p.reset('x'); p.start_timing('a'); p.reset()
print('reset() leaves active timers:', list(p.active_timers))
@profile('f', deadline=0.001)
def f():
    """doc"""
    raise ValueError('boom')
print('decorated name/doc:', f.__name__, f.__doc__)
try: f()
except ValueError: pass
print('after exception: execution_times recorded?', 'f' in global_profiler.execution_times, '; active timer leaked?', 'f' in global_profiler.active_timers)
p = PerformanceProfiler()
p.start_timing('loop'); p.start_timing('loop'); time.sleep(0.01)
print('re-entrant same name: second stop ->', p.stop_timing('loop'), p.stop_timing('loop'))
p = PerformanceProfiler(); p.start_timing('t'); p.stop_timing('t', deadline=0.5); p.start_timing('t'); p.stop_timing('t', deadline=0.0)
print('deadline changes ignored after first call:', p.deadlines['t'])
print('print_report miss rate format:'); p.print_report()

print('=== websocket ===')
try:
    import navi_bot.utils.websocket
except Exception as e:
    print('import navi_bot.utils.websocket ->', type(e).__name__, e)
import asyncio
async def coro(): return 1
async def main():
    try:
        await asyncio.wait([coro()])
    except Exception as e:
        print('asyncio.wait([bare coroutine]) on', sys.version.split()[0], '->', type(e).__name__, e)
asyncio.run(main())
