#!/usr/bin/env python3
"""
Map Publisher Node

Loads a map_server-style map (a .yaml describing a .pgm image) and publishes
it ONCE, latched, on /map. With TRANSIENT_LOCAL durability the message is
retained by DDS, so nodes that start later (path_planner, robot_controller)
still receive it — the standard ROS pattern for static maps.

Published values are true occupancy: 0 = free, 100 = occupied, -1 = unknown
(classified from pixels using the yaml's negate/occupied_thresh/free_thresh).
Consumers apply their own policy (path_planner thresholds at >= 50 and treats
unknown as wall).

Orientation: PGM row 0 is the image TOP, but OccupancyGrid.data is row-major
from the map ORIGIN (bottom-left), so the image is vertically flipped before
flattening — the same convention maps/generate_map.py documents.

Parameters:
    map_yaml — path to the map yaml (default: the installed
               share/navi_bot/maps/warehouse_simple.yaml, falling back to the
               source tree's maps/ when running uninstalled/under the mock).
"""

from navi_bot.ros_compat import (
    Node, OccupancyGrid, rclpy, init, shutdown, spin,
    latched_map_qos, USING_REAL_ROS,
)

from pathlib import Path as FilePath

import numpy as np


def default_map_yaml():
    """The installed map, or the source-tree copy when not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = FilePath(get_package_share_directory('navi_bot'))
        candidate = share / 'maps' / 'warehouse_simple.yaml'
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass
    return str(FilePath(__file__).resolve().parent.parent / 'maps' / 'warehouse_simple.yaml')


def parse_map_yaml(path):
    """
    Minimal parser for the flat map_server yaml schema (image, resolution,
    origin, negate, occupied_thresh, free_thresh). Deliberately dependency-free
    so the node runs on any interpreter without PyYAML.
    """
    spec = {'origin': [0.0, 0.0, 0.0], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.196}
    for line in FilePath(path).read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].strip()
        if ':' not in line:
            continue
        key, value = (part.strip() for part in line.split(':', 1))
        if key == 'image':
            spec['image'] = value
        elif key == 'origin':
            spec['origin'] = [float(v) for v in value.strip('[] ').split(',')]
        elif key in ('resolution', 'occupied_thresh', 'free_thresh'):
            spec[key] = float(value)
        elif key == 'negate':
            spec[key] = int(value)
    if 'image' not in spec or 'resolution' not in spec:
        raise ValueError(f'{path}: not a map yaml (needs image + resolution)')
    return spec


def read_pgm(path):
    """Read a binary (P5) or ascii (P2) PGM. Returns the IMAGE array (row 0 =
    top). Mirrors maps/generate_map.py's reader — kept in-package so the
    runtime node doesn't reach into the dev-tools directory."""
    data = FilePath(path).read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:                # magic, width, height, maxval
        if data[i:i + 1] == b'#':
            i = data.index(b'\n', i) + 1
            continue
        if data[i:i + 1].isspace():
            i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h = tokens[0], int(tokens[1]), int(tokens[2])
    if magic == b'P5':
        img = np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8)
    elif magic == b'P2':
        img = np.array(data[i:].split()[:w * h], dtype=np.uint8)
    else:
        raise ValueError(f'not a PGM file: {magic!r}')
    return img.reshape(h, w)


def classify(image, spec):
    """
    Pixels -> occupancy values per map_server semantics, on the WORLD-oriented
    grid (row 0 = origin): p > occupied_thresh -> 100, p < free_thresh -> 0,
    otherwise -1 (unknown).
    """
    grid = np.flipud(image).astype(float)
    p = grid / 255.0 if spec['negate'] else (255.0 - grid) / 255.0
    occ = np.full(grid.shape, -1, dtype=np.int8)
    occ[p > spec['occupied_thresh']] = 100
    occ[p < spec['free_thresh']] = 0
    return occ


class MapPublisherNode(Node):
    """Publishes the static map once, latched, on /map."""

    def __init__(self):
        super().__init__('map_publisher')

        self.declare_parameter('map_yaml', default_map_yaml())
        yaml_path = self.get_parameter('map_yaml').get_parameter_value().string_value

        spec = parse_map_yaml(yaml_path)
        image_path = FilePath(yaml_path).parent / spec['image']
        occ = classify(read_pgm(image_path), spec)
        self.occupancy = occ                       # (rows, cols) int8, world-oriented
        self.spec = spec

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched_map_qos())
        self.publish_map()

        h, w = occ.shape
        self.get_logger().info(
            f'Published {w}x{h} map from {FilePath(yaml_path).name} '
            f'@ {spec["resolution"]} m/cell (latched)')

    def build_msg(self):
        """Fill an OccupancyGrid for whichever backend is live. The real
        message carries geometry under .info; the mock message is given the
        same .info shape (plus its native flat fields) so consumers like
        path_planner.map_callback take one code path on both backends."""
        occ = self.occupancy
        h, w = occ.shape
        msg = OccupancyGrid()
        msg.data = [int(v) for v in occ.ravel()]
        if USING_REAL_ROS:
            msg.header.frame_id = 'map'
            msg.info.resolution = float(self.spec['resolution'])
            msg.info.width = w
            msg.info.height = h
            msg.info.origin.position.x = float(self.spec['origin'][0])
            msg.info.origin.position.y = float(self.spec['origin'][1])
        else:
            from types import SimpleNamespace
            msg.width = w                      # mock's native flat fields
            msg.height = h
            msg.resolution = float(self.spec['resolution'])
            msg.info = SimpleNamespace(        # real-shaped view of the same map
                resolution=float(self.spec['resolution']),
                width=w, height=h,
                origin=SimpleNamespace(position=SimpleNamespace(
                    x=float(self.spec['origin'][0]),
                    y=float(self.spec['origin'][1]))))
        return msg

    def publish_map(self):
        self.map_pub.publish(self.build_msg())


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
