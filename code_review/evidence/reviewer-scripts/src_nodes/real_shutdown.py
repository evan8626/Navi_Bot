"""Real rclpy: emulate Ctrl-C during spin of the actual RobotController main()."""
import sys, signal, threading, os
sys.path.insert(0, r'C:\Users\evan8\Desktop\Navi Bot')
import rclpy
import navi_bot.ros_compat as rc
print('USING_REAL_ROS', rc.USING_REAL_ROS)
import navi_bot.robot_controller as ctl

orig_init = ctl.RobotController.__init__
def patched(self):
    orig_init(self)
    # raise SIGINT from inside spin after 0.3 s, like a user pressing Ctrl-C
    self._t = self.create_timer(0.3, lambda: signal.raise_signal(signal.SIGINT))
ctl.RobotController.__init__ = patched
try:
    ctl.main()
    print('main() returned normally')
except BaseException as e:
    print('main() raised:', type(e).__module__, type(e).__name__, str(e)[:200])
