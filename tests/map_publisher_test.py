#!/usr/bin/env python3
"""
Test script for MapPublisherNode from map_publisher.py

The map publisher is the /map source for the whole graph: it loads the
map_server-style yaml + pgm, classifies pixels into true occupancy values
(0 free / 100 occupied / -1 unknown), and publishes ONCE, latched.

These tests pin: the yaml parser, the pixel classification (thresholds +
negate + the image->world vertical flip), the published message shape on the
mock backend (real-shaped .info AND the mock's flat fields), the latched QoS,
and — end to end — that PathPlannerNode.map_callback consumes the published
message into planner-ready grids with stations free and walls occupied.
"""
import logging
import math
import sys
from pathlib import Path as FilePath

import numpy as np

from navi_bot.map_publisher import (
    MapPublisherNode, parse_map_yaml, read_pgm, classify, default_map_yaml,
)
from navi_bot.path_planner import PathPlannerNode
from navi_bot.ros_compat import QoSDurabilityPolicy

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

REPO = FilePath(__file__).resolve().parent.parent
MAP_YAML = REPO / 'maps' / 'warehouse_simple.yaml'


# MARK: Parsing / classification

def test_yaml_parser():
    """parse_map_yaml — reads image/resolution/origin/thresholds from the real yaml."""
    passed = True
    logger.info("TEST 1: parse_map_yaml — warehouse_simple.yaml parses")
    spec = parse_map_yaml(MAP_YAML)
    checks = {
        "image == 'warehouse_simple.pgm'": spec.get('image') == 'warehouse_simple.pgm',
        'resolution == 0.05': spec.get('resolution') == 0.05,
        'origin == [0, 0, 0]': spec.get('origin') == [0.0, 0.0, 0.0],
        'negate == 0': spec.get('negate') == 0,
        'occupied_thresh == 0.65': spec.get('occupied_thresh') == 0.65,
        'free_thresh == 0.196': spec.get('free_thresh') == 0.196,
    }
    for name, ok in checks.items():
        if ok:
            logger.info(f"  OK   {name}")
        else:
            logger.warning(f"  FAIL {name} (spec={spec})")
            passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_classification_values_and_flip():
    """classify — pixels map to {0, 100, -1} and the image is flipped so
    grid row 0 is the map origin (bottom of the image)."""
    passed = True
    logger.info("TEST 2: classify — occupancy values and image->world flip")
    spec = {'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
    # 2x2 image: TOP row = [occupied, unknown], BOTTOM row = [free, occupied]
    image = np.array([[0, 205],
                      [254, 0]], dtype=np.uint8)
    occ = classify(image, spec)
    # After flipud, grid row 0 (bottom of image) = [free, occupied]
    if not (occ[0][0] == 0 and occ[0][1] == 100):
        logger.warning(f"  FAIL grid row 0 = {occ[0].tolist()}, expected [0, 100]")
        passed = False
    else:
        logger.info("  OK   bottom image row became grid row 0 (free, occupied)")
    if not (occ[1][0] == 100 and occ[1][1] == -1):
        logger.warning(f"  FAIL grid row 1 = {occ[1].tolist()}, expected [100, -1]")
        passed = False
    else:
        logger.info("  OK   top image row became grid row 1 (occupied, unknown)")
    if occ.dtype != np.int8:
        logger.warning(f"  FAIL dtype {occ.dtype}, expected int8")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_negate_flag():
    """classify — negate: 1 inverts the pixel->probability mapping."""
    passed = False
    logger.info("TEST 3: classify — negate flag inverts the mapping")
    spec = {'negate': 1, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
    image = np.array([[255, 0]], dtype=np.uint8)   # negate: white=occupied, black=free
    occ = classify(image, spec)
    if occ[0][0] == 100 and occ[0][1] == 0:
        logger.info("  OK   white -> occupied, black -> free under negate")
        passed = True
    else:
        logger.warning(f"  FAIL got {occ.tolist()}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Node behavior

def test_node_publishes_latched_map():
    """MapPublisherNode — publishes once on /map with TRANSIENT_LOCAL QoS."""
    passed = True
    logger.info("TEST 4: node — publishes the warehouse once, latched")
    node = MapPublisherNode()
    if len(node.map_pub.published_msgs) != 1:
        logger.warning(f"  FAIL published {len(node.map_pub.published_msgs)} messages, expected 1")
        passed = False
    else:
        logger.info("  OK   exactly one publish on construction")
    qos = node.map_pub.qos
    if getattr(qos, 'durability', None) != QoSDurabilityPolicy.TRANSIENT_LOCAL:
        logger.warning(f"  FAIL durability={getattr(qos, 'durability', None)}")
        passed = False
    else:
        logger.info("  OK   TRANSIENT_LOCAL (latched) durability")
    msg = node.map_pub.last_msg
    if msg.info.width != 200 or msg.info.height != 200 or msg.info.resolution != 0.05:
        logger.warning(f"  FAIL info {msg.info.width}x{msg.info.height}@{msg.info.resolution}")
        passed = False
    else:
        logger.info("  OK   info reports 200x200 @ 0.05 m/cell")
    values = set(int(v) for v in msg.data)
    if not values <= {0, 100, -1}:
        logger.warning(f"  FAIL illegal occupancy values {sorted(values - {0, 100, -1})}")
        passed = False
    else:
        logger.info(f"  OK   data values {sorted(values)} are legal occupancy")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_end_to_end_into_path_planner():
    """Integration — path_planner.map_callback consumes the published message:
    walls become walls, stations stay free, geometry is stashed."""
    passed = True
    logger.info("TEST 5: integration — published map lands in the planners")
    pub = MapPublisherNode()
    planner = PathPlannerNode()
    planner.map_callback(pub.map_pub.last_msg)

    grid = planner.global_planner.occupancy_grid
    if grid is None or np.asarray(grid).shape != (200, 200):
        logger.warning(f"  FAIL planner grid shape {np.asarray(grid).shape if grid is not None else None}")
        logger.info("FAIL")
        return False
    logger.info("  OK   planner received a 200x200 grid")
    if planner.map_resolution != 0.05 or planner.map_origin != (0.0, 0.0):
        logger.warning(f"  FAIL geometry res={planner.map_resolution} origin={planner.map_origin}")
        passed = False
    else:
        logger.info("  OK   resolution/origin stashed for world<->grid")
    if grid[0][0] != 1:
        logger.warning("  FAIL corner wall cell (0,0) is not a wall after conversion")
        passed = False
    else:
        logger.info("  OK   perimeter wall survives conversion")
    # pickup_1 world (3.0, 0.9) -> cell (row 18, col 60) must be free floor.
    if grid[18][60] != 0:
        logger.warning("  FAIL pickup_1 cell (18, 60) is not free")
        passed = False
    else:
        logger.info("  OK   pickup_1 station cell is free")
    if planner.DStar_local_planner.occupancy_grid is None:
        logger.warning("  FAIL D* planner did not receive the grid")
        passed = False
    else:
        logger.info("  OK   D* Lite received the same grid")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Runner

def main():
    tests = [
        test_yaml_parser,
        test_classification_values_and_flip,
        test_negate_flag,
        test_node_publishes_latched_map,
        test_end_to_end_into_path_planner,
    ]

    args = sys.argv[1:]
    if args and args[0] == '--list':
        for i, t in enumerate(tests, 1):
            doc = (t.__doc__ or t.__name__).strip().splitlines()[0]
            print(f"TEST {i}: {doc}")
        return

    selected = tests
    if args:
        try:
            n = int(args[0])
            if not 1 <= n <= len(tests):
                raise ValueError
        except ValueError:
            logger.error(f"Invalid test selector {args[0]!r} — use 1..{len(tests)} or --list")
            sys.exit(2)
        selected = [tests[n - 1]]

    logger.info("Map Publisher Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
