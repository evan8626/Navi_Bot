import math, sys, io, contextlib
from navi_bot.control.kinematics import DifferentialDriveKinematics
from navi_bot.control.motion_controller import PIDController, MotionController
from navi_bot.control.pure_pursuit import PurePursuitController

print("== kinematics clip distortion")
k = DifferentialDriveKinematics()
for v, w in [(10.0, 100.0), (1.5, 1.0), (0.9, 2.0)]:
    vl, vr = k.inverse_kinematics(v, w)
    v2, w2 = k.forward_kinematics(vl, vr)
    R_cmd = v / w
    R_out = float(v2 / w2) if w2 else float('inf')
    print(f"cmd v={v} w={w} R={R_cmd:.3f} -> wheels ({float(vl):.3f},{float(vr):.3f}) -> v={float(v2):.3f} w={float(w2):.3f} R={R_out:.3f}")

print("== PID derivative kick")
mc = MotionController()
mc.set_goal(0.5, 0.0)
v, w = mc.compute_control((0.0, 0.0, 0.0), 0.02)
print("first v", v, "linear d-term contrib", 0.1 * (0.5 - 0) / 0.02)
print("== set_goal does not reset PID state")
mc = MotionController()
mc.set_goal(5.0, 0.0)
for _ in range(50):
    mc.compute_control((0.0, 0.0, 0.0), 0.1)
print("integral after goal1:", mc.linear_pid.integral, "prev_error", mc.linear_pid.prev_error)
mc.set_goal(0.5, 0.0)
print("integral after set_goal(new):", mc.linear_pid.integral, "prev_error", mc.linear_pid.prev_error)
mc2 = MotionController(); mc2.set_goal(0.5, 0.0)
print("new goal first cmd (carried) :", mc.compute_control((0.0, 0.0, 0.0), 0.1))
print("new goal first cmd (fresh)   :", mc2.compute_control((0.0, 0.0, 0.0), 0.1))
print("== negative dt")
p = PIDController(kp=0, ki=1.0, kd=0)
print("neg dt out", p.update(1.0, -0.1), "integral", p.integral)
print("== goal behind: v sign")
mc = MotionController(); mc.set_goal(-5.0, 0.0)
print("goal behind cmd", mc.compute_control((0.0, 0.0, 0.0), 0.1))

print("== pure pursuit: sim to end of path")
logging_off = io.StringIO()
import logging; logging.disable(logging.CRITICAL)
path = [(float(i), 0.0) for i in range(11)]
c = PurePursuitController(lookahead=2.0, linear_velocity=0.5)
x, y, th = 0.0, 0.0, 0.0
dt = 0.05
for step in range(2000):
    v, w = c.pure_pursuit(path, (x, y, th))
    if v == 0 and w == 0:
        break
    x += v * math.cos(th) * dt; y += v * math.sin(th) * dt; th += w * dt
print(f"stopped at step {step} pose=({x:.2f},{y:.2f}) goal={path[-1]} dist={math.hypot(10-x, y):.2f} waypoint idx={c.waypoint}")

print("== pure pursuit: lookahead point behind robot")
c = PurePursuitController(lookahead=2.0)
print("robot at (6,0) fresh ->", c.find_next_lookahead(path, (6.0, 0.0)))
print("== pure pursuit: facing away (theta=pi) omega")
c = PurePursuitController(lookahead=2.0)
print(c.pure_pursuit(path, (0.0, 0.0, math.pi)))
print("== pure pursuit: curvature uses ld not actual dist")
c = PurePursuitController(lookahead=1.0)
sparse = [(0.0, 0.0), (4.0, 4.0)]
v, w = c.pure_pursuit(sparse, (0.0, 0.0, 0.0))
L = math.hypot(4, 4)
print(f"omega={w:.4f}, true pure-pursuit omega (2 sin a / L)={2*math.sin(math.pi/4)/L:.4f}")
