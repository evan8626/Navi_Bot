import sys, math
sys.path.insert(0, '.')
import numpy as np
from navi_bot.control.kinematics import DifferentialDriveKinematics
from navi_bot.control.motion_controller import PIDController, MotionController
from navi_bot.control.pure_pursuit import PurePursuitController

print('=== kinematics ===')
k = DifferentialDriveKinematics()   # max wheel 10 rad/s * 0.1 m = 1.0 m/s
v, w = 1.0, 2.0
vl, vr = k.inverse_kinematics(v, w)
v2, w2 = k.forward_kinematics(vl, vr)
print(f'cmd v={v}, w={w}: curvature {w/v:.2f} 1/m  -> wheels ({float(vl):.2f},{float(vr):.2f}) -> v={float(v2):.2f}, w={float(w2):.2f}, curvature {float(w2/v2):.2f} 1/m')
print('type returned by inverse_kinematics:', type(vl).__name__)
try:
    DifferentialDriveKinematics(wheel_base=0).forward_kinematics(0.5, 0.6)
except Exception as e:
    print('wheel_base=0 forward_kinematics ->', type(e).__name__, e)
print('wheel_base=0 inverse ok, forward ->', end=' ')
import warnings; warnings.simplefilter('ignore')
print(DifferentialDriveKinematics(wheel_base=0.0).forward_kinematics(np.float64(0.5), np.float64(0.6)))
print('docstring says max_wheel_speed is rad/s and IK returns m/s; validate_velocities(1.0,0)=', k.validate_velocities(1.0, 0.0))

print('=== PID ===')
p = PIDController(kp=0.5, ki=0.01, kd=0.1)
print('first update(error=5.0, dt=0.02) ->', p.update(5.0, 0.02), '(kp*e = 2.5; the rest is derivative kick (5-0)/0.02*0.1 =', 0.1*5/0.02, ')')
p = PIDController(kp=1.0, ki=1.0, kd=0.0, output_limits=(-1, 1))
for _ in range(100): p.update(5.0, 0.1)
print('anti-windup: integral after 100 saturated steps', p.integral)
# conditional integration: integral already large then error flips sign; clamped output case
p = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limits=(-1, 1))
p.integral = 0.99
out = p.update(-0.5, 0.1)   # integral should move toward 0 (unwinding) -- allowed?
print('unwinding step: output', out, 'integral', p.integral)
p = PIDController(kp=10.0, ki=1.0, kd=0.0, output_limits=(-1, 1))
p.integral = 3.0
out = p.update(-0.05, 0.1)  # P term -0.5, I term 3 -> saturated at +1 -> integral is NOT allowed to unwind
print('saturated while error opposes integral: output', out, 'integral', p.integral, '(expected to decrease to 2.995)')
p = PIDController(kp=1.0, ki=0.0, kd=1.0)
print('dt=0 ->', p.update(1.0, 0.0), '; dt<0 ->', p.update(1.0, -0.1), 'integral after dt<0', p.integral)
p = PIDController(kp=1.0, ki=0.0, kd=0.0); print('prev_time attr unused:', p.prev_time)

mc = MotionController()
mc.set_goal(0.0, 0.0, theta=math.pi/2)
# inside goal tolerance: angular PID fed absolute goal-angle error; then on new goal fed heading error
o1 = mc.angular_pid.update(1.5, 0.02)
mc.set_goal(5.0, 0.0)
v, w = mc.compute_control((0.0, 0.0, 0.0), 0.02)
print('after set_goal with stale angular PID state (prev_error=1.5): heading_error=0 -> omega', round(float(w), 2),
      ', integral carried', mc.angular_pid.integral)
mc2 = MotionController(); mc2.set_goal(1.0, 0.0)
pose = [0.0, 0.0, 0.0]
for i in range(400):
    v, w = mc2.compute_control(tuple(pose), 0.02)
    pose[0] += v*math.cos(pose[2])*0.02; pose[1] += v*math.sin(pose[2])*0.02; pose[2] += w*0.02
print('linear PID drives toward goal 1 m ahead, final x', round(pose[0], 3))
# overshoot behind robot: distance always positive -> drives forward even when goal is behind but heading wrong
mc3 = MotionController(); mc3.set_goal(-1.0, 0.0)
v, w = mc3.compute_control((0.0, 0.0, 0.0), 0.02)
print('goal 1 m BEHIND (heading err pi): v =', round(float(v), 3), '(positive -> drives away from goal while turning)')

print('=== pure pursuit ===')
def drive(path, pose, steps=2000, dt=0.05, pp=None):
    pp = pp or PurePursuitController(lookahead=1.0, linear_velocity=0.5)
    x, y, th = pose
    for i in range(steps):
        v, w = pp.pure_pursuit(path, (x, y, th))
        if v == 0 and w == 0:
            break
        x += v*math.cos(th)*dt; y += v*math.sin(th)*dt; th += w*dt
    return (round(x, 2), round(y, 2)), i, pp
straight = [(float(i), 0.0) for i in range(0, 11)]
end, it, pp = drive(straight, (0.0, 0.0, 0.0))
print('straight path to (10,0): stopped at', end, 'after', it, 'steps; waypoint idx', pp.waypoint)
L = [(float(i), 0.0) for i in range(0, 6)] + [(5.0, float(j)) for j in range(1, 6)]
end, it, pp = drive(L, (0.0, 0.0, 0.0))
print('L path to (5,5): stopped at', end, 'after', it, 'steps; dist to goal', round(math.dist(end, (5, 5)), 2))
pp = PurePursuitController(lookahead=1.0)
print('find_next_lookahead when robot within lookahead of every remaining point ->', pp.find_next_lookahead([(0, 0), (0.5, 0)], (0.0, 0.0)))
pp = PurePursuitController(lookahead=1.0)
pp.pure_pursuit(straight, (8.0, 0.0, 0.0)); idx = pp.waypoint
newpath = [(8.0, float(j)) for j in range(0, 6)]
print('stale index: after following old path waypoint=', idx, '; new 6-point path ->', pp.pure_pursuit(newpath, (8.0, 0.0, math.pi/2)))
pp = PurePursuitController(lookahead=1.0)
try:
    print('empty path ->', pp.pure_pursuit([], (0, 0, 0)))
except Exception as e:
    print('empty path ->', type(e).__name__, e)
pp = PurePursuitController(lookahead=1.0)
pp.waypoint = 99
print('waypoint>=len, robot 50 m from goal of 3-pt path: returns', pp.find_next_lookahead([(0, 0), (1, 0), (2, 0)], (52.0, 0.0)),
      ' (threshold is path_len + lookahead = 3 + 1 = 4 "m")')
# curvature with nominal lookahead vs true distance: point 3 m away when lookahead 1 m
pp = PurePursuitController(lookahead=1.0, linear_velocity=1.0)
v, w = pp.pure_pursuit([(0.0, 0.0), (0.0, 3.0)], (0.0, 0.0, 0.0))
true_k = 2*math.sin(math.pi/2)/3.0
print(f'target 3 m away at 90 deg: omega {w:.3f} (uses ld=1.0), geometric arc needs {true_k:.3f}')
# lookahead point behind the robot
pp = PurePursuitController(lookahead=1.0, linear_velocity=1.0)
print('first point >= lookahead is BEHIND robot -> cmd', pp.pure_pursuit([(-2.0, 0.0), (5.0, 0.0)], (0.0, 0.0, 0.0)))
print('constructor: lookahead=0 accepted ->', PurePursuitController(lookahead=0).lookahead_distance,
      '; negative velocity accepted ->', PurePursuitController(linear_velocity=-3).linear_velocity)
