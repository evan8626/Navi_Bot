"""robot_controller safety governor, stale sensors, last_cmd feedback."""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import numpy as np
import navi_bot.mock_ros2 as m
from navi_bot.mock_ros2 import LaserScan, Pose2D, Twist
from navi_bot.robot_controller import RobotController

def scan_with(fn, n=360):
    s = LaserScan(); s.angle_min = -math.pi; s.angle_increment = 2*math.pi/n
    s.ranges = [fn(s.angle_min + i*s.angle_increment) for i in range(n)]
    return s

buf = io.StringIO()
m.init()
with contextlib.redirect_stdout(buf):
    rc = RobotController()
cmds = []
rc.cmd_vel_pub.publish = lambda msg: cmds.append((msg.linear.x, msg.angular.z))

# 1) before any scan: closest = inf, governor never engages
print('initial closest_obstacle_dist:', rc.closest_obstacle_dist)

# 2) obstacle 0.4 m directly BEHIND, goal straight ahead (point-goal mode)
rc.set_goal(5.0, 0.0)
with contextlib.redirect_stdout(buf):
    rc.lidar_callback(scan_with(lambda a: 0.4 if abs(abs(a) - math.pi) < 0.05 else 8.0))
    rc.control_loop()
print('obstacle 0.4 m behind, clear ahead -> cmd', cmds[-1])

# 3) goal requires turning in place (behind the robot); obstacle 0.45 m to the side
rc.set_goal(-5.0, 0.0)
with contextlib.redirect_stdout(buf):
    rc.lidar_callback(scan_with(lambda a: 0.45 if abs(a - math.pi/2) < 0.05 else 8.0))
    rc.control_loop()
print('turn-in-place needed, obstacle 0.45 m at side -> cmd', cmds[-1])

# 4) 0.9 m wide aisle, robot centred: walls at 0.45 m on both sides
def aisle(a, half=0.45):
    s = abs(math.sin(a))
    return min(half / s, 10.0) if s > 1e-6 else 10.0
rc.set_goal(5.0, 0.0)
with contextlib.redirect_stdout(buf):
    for _ in range(50):
        rc.lidar_callback(scan_with(aisle))
        rc.control_loop()
print('0.9 m aisle, 50 cycles -> distinct cmds', set(cmds[-50:]), 'closest', round(rc.closest_obstacle_dist, 3))

# 5) stale scan: the last scan said "clear"; scans then stop. Governor keeps using it.
with contextlib.redirect_stdout(buf):
    rc.lidar_callback(scan_with(lambda a: 8.0))
    for _ in range(500):          # 10 s of control cycles with no new /scan
        rc.control_loop()
print('after 500 cycles w/o any new scan, closest still', rc.closest_obstacle_dist, 'cmd', cmds[-1])

# 6) obstacle_threshold: a wall at 1.6 m dead ahead never reaches DWA / valid_obstacles
with contextlib.redirect_stdout(buf):
    rc.lidar_callback(scan_with(lambda a: 1.6 if abs(a) < 0.3 else 8.0))
print('wall at 1.6 m ahead -> valid_obstacles passed to DWA:', len(rc.valid_obstacles))
print('stopping distance from 1 m/s at 0.5 m/s^2 = 1.0 m; + DWA CONTACT 0.875 m =', 1.0 + 0.875)

# 7) last_cmd fed back as current velocity: per-cycle acceleration in PATH mode
from navi_bot.planners.dwa import DWAPlanner
cmds.clear()
rc.current_pose = Pose2D(); rc.current_pose.x, rc.current_pose.y = 2.0, 2.0
rc.set_path([(2.0, 2.0 + i*0.5) for i in range(1, 2)] + [(2.0 + i, 3.0) for i in range(1, 8)])
rc.current_pose.theta = 0.0
rc.set_path([(2.0 + i*0.5, 2.0) for i in range(0, 16)])
with contextlib.redirect_stdout(buf):
    rc.lidar_callback(scan_with(lambda a: 8.0))
    for _ in range(5):
        rc.control_loop()
print('PATH mode first 5 cycles (20 ms each) v:', [round(c[0], 3) for c in cmds[:5]],
      '-> implied accel', round((cmds[1][0]-cmds[0][0])/0.02, 1), 'm/s^2 vs max_accel_x 0.5')

# 8) deadline-miss log string (missing separator)
out = buf.getvalue()
miss = [l for l in out.splitlines() if 'Deadline miss' in l]
print('deadline-miss log sample:', miss[:1], 'misses', rc.deadline_misses)
print('unused attrs: current_velocity', type(rc.current_velocity).__name__, 'battery_level', rc.battery_level)
