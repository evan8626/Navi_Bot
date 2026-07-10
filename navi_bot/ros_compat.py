"""
ROS 2 compatibility shim.

One import surface for everything ROS-ish, backed by whichever runtime is
actually available:

* When a real ROS 2 environment is active (the pixi Python 3.12 env with the
  install + navi_bot_interfaces workspace sourced), this re-exports the real
  ``rclpy`` stack and message types.
* Otherwise it falls back to the in-process ``mock_ros2`` shim, so the test
  suite keeps running on plain Python (3.14) with no ROS 2 installed.

Nodes import from here instead of hard-wiring either backend::

    from navi_bot.ros_compat import Node, Pose2D, Twist, rclpy, init, spin, shutdown

``USING_REAL_ROS`` tells callers which backend is live.

Note: if real ``rclpy`` is importable but ``navi_bot_interfaces`` is not, the
import below raises loudly on purpose — that means the ROS 2 env is sourced but
the interfaces workspace isn't, which is a misconfiguration we want surfaced
rather than silently papered over with the mock.
"""

try:
    import rclpy  # noqa: F401  (probe for a real ROS 2 environment)
    _REAL_ROS = True
except ImportError:
    _REAL_ROS = False

if _REAL_ROS:
    from rclpy.node import Node
    from rclpy import init, shutdown, spin
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from geometry_msgs.msg import Twist, Point
    from nav_msgs.msg import Path, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32, String
    from navi_bot_interfaces.msg import Pose2D  # our replacement for the removed geometry_msgs/Pose2D
    USING_REAL_ROS = True
else:
    import navi_bot.mock_ros2 as rclpy
    from navi_bot.mock_ros2 import (
        Node, Twist, Pose2D, Point, Path, OccupancyGrid,
        LaserScan, Float32, String, init, shutdown, spin,
    )
    USING_REAL_ROS = False

    # QoS stand-ins: the mock bus delivers everything to everyone, so these
    # only need to be constructible/carryable — the semantics live in real DDS.
    class QoSDurabilityPolicy:
        TRANSIENT_LOCAL = 'transient_local'
        VOLATILE = 'volatile'

    class QoSReliabilityPolicy:
        RELIABLE = 'reliable'
        BEST_EFFORT = 'best_effort'

    class QoSProfile:
        def __init__(self, depth=10, durability=None, reliability=None, **kwargs):
            self.depth = depth
            self.durability = durability
            self.reliability = reliability
            self.__dict__.update(kwargs)


def latched_map_qos():
    """
    The QoS both sides of /map must share: depth-1, RELIABLE, TRANSIENT_LOCAL
    ("latched") — the publisher sends the map once and late-joining
    subscribers still receive it. A default (volatile) subscriber is
    incompatible with a transient-local publisher and silently gets nothing,
    so every /map publisher AND subscriber takes its QoS from here.
    """
    return QoSProfile(depth=1,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)

def occupancy_to_grid(msg):
    """
    Normalize a map message to the 2D int grid the planners consume
    (0 = free, 1 = wall). Real nav_msgs/OccupancyGrid carries flat row-major
    int8 data with dims under .info; occupancy >= 50 and unknown (-1) both
    count as wall. A bare 2D array passes through unchanged, so tests can
    feed grids directly. Returns a FRESH array per message (D* Lite's
    previous/current diffing depends on that). Callers wanting resolution or
    origin read them off msg.info themselves.
    """
    import numpy as np
    if hasattr(msg, 'info'):
        raw = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return ((raw >= 50) | (raw == -1)).astype(int)
    return np.asarray(msg)


def make_path_msg(waypoints):
    """
    Build a correctly-shaped Path message from [(x, y), ...] world waypoints
    for whichever backend is live: real nav_msgs/Path requires PoseStamped
    poses (bare Points fail rclpy's type check on publish); the mock Path
    keeps its simple Point list. The symmetric partner of path_to_waypoints.
    """
    msg = Path()
    if USING_REAL_ROS:
        from geometry_msgs.msg import PoseStamped
        for x, y in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            msg.poses.append(ps)
        msg.header.frame_id = 'map'
    else:
        for x, y in waypoints:
            p = Point()
            p.x, p.y = float(x), float(y)
            msg.poses.append(p)
    return msg


def path_to_waypoints(msg):
    """
    Normalize a path message to [(x, y), ...]. Handles both the mock Path
    (poses = list of Points) and real nav_msgs/Path (poses = PoseStamped with
    .pose.position). A bare list of pairs passes through.
    """
    poses = getattr(msg, 'poses', msg)
    out = []
    for p in poses:
        if hasattr(p, 'pose'):            # nav_msgs/Path: PoseStamped
            out.append((p.pose.position.x, p.pose.position.y))
        elif hasattr(p, 'x'):             # mock Path: bare Point
            out.append((p.x, p.y))
        else:                             # already a pair
            out.append((p[0], p[1]))
    return out


__all__ = [
    "rclpy", "Node", "init", "shutdown", "spin",
    "Twist", "Point", "Pose2D", "Path", "OccupancyGrid",
    "LaserScan", "Float32", "String", "USING_REAL_ROS",
    "occupancy_to_grid", "path_to_waypoints", "make_path_msg",
    "QoSProfile", "QoSDurabilityPolicy", "QoSReliabilityPolicy", "latched_map_qos",
]
