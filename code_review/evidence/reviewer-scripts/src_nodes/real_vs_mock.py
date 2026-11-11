"""Shape/semantics differences real rclpy vs navi_bot.mock_ros2 (run in BOTH envs)."""
import sys, time
sys.path.insert(0, r'C:\Users\evan8\Desktop\Navi Bot')
import navi_bot.ros_compat as rc
print('USING_REAL_ROS', rc.USING_REAL_ROS)
def t(label, fn):
    try:
        print(f'{label}: OK ->', fn())
    except BaseException as e:
        print(f'{label}: {type(e).__name__}: {str(e)[:120]}')

t('OccupancyGrid().width', lambda: rc.OccupancyGrid().width)
t('OccupancyGrid(width=4,height=4,resolution=0.05)', lambda: type(rc.OccupancyGrid(width=4, height=4, resolution=0.05)).__name__)
t('hasattr(OccupancyGrid(), "info")', lambda: hasattr(rc.OccupancyGrid(), 'info'))
t('Pose2D(x=1.0)', lambda: rc.Pose2D(x=1.0).x)
t('LaserScan().angle_increment', lambda: rc.LaserScan().angle_increment)

from navi_bot.sensors.lidar_processing import LidarProcessor
def costmap_real():
    cm = rc.OccupancyGrid()
    LidarProcessor().update_costmap([{'centroid': (0.5, 0.5)}], cm)
    return 'ok'
t('LidarProcessor.update_costmap(real-shaped OccupancyGrid())', costmap_real)

rc.rclpy.init()
n = rc.Node('probe')
t('get_parameter(undeclared)', lambda: n.get_parameter('nope').get_parameter_value().double_value)
pub = n.create_publisher(rc.Twist, '/probe', 10)
t('publish(String) on a Twist publisher', lambda: pub.publish(rc.String(data='x')))
fired = []
n.create_timer(0.05, lambda: fired.append(time.monotonic()))
if rc.USING_REAL_ROS:
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        rc.rclpy.spin_once(n, timeout_sec=0.05)
print('timer(0.05 s) fired in 0.5 s of spinning:', len(fired), '(mock: never spins timers)')
got = []
n.create_subscription(rc.String, '/sync', lambda m: got.append(m.data), 10)
p2 = n.create_publisher(rc.String, '/sync', 10)
p2.publish(rc.String(data='hello'))
print('delivered synchronously inside publish():', got)
n.destroy_node()
rc.rclpy.shutdown()
