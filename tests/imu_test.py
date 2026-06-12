#!/usr/bin/env python3
"""
Test script for the IMU processor in navi_bot/sensors/imu_processor.py

The IMU code is still in progress, so this suite doubles as its
specification — tests define the intended behavior and will pass as the
implementation is completed:
- Construction and buffer setup
- Moving-average filtering of gyro and accelerometer streams
- Bias correction after calibration
- Motion detection from acceleration magnitude
- Orientation tracking from gyro integration (complementary filter)
"""
import logging
import math
import sys
from types import SimpleNamespace

import numpy as np

from navi_bot.sensors.imu_processor import ImuProcessor

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def make_imu_msg(gyro_z=0.0, accel=(0.0, 0.0, 0.0)):
    """Build a minimal sensor_msgs/Imu-shaped message."""
    return SimpleNamespace(
        angular_velocity=SimpleNamespace(z=gyro_z),
        linear_acceleration=SimpleNamespace(x=accel[0], y=accel[1], z=accel[2]),
    )


def setup_imu(buffer_size=10):
    return ImuProcessor(buffer_size=buffer_size)


# MARK: Construction

def test_construction():
    """ImuProcessor() must construct with empty buffers and an uncalibrated state."""
    passed = True
    logger.info("TEST 1: construction defaults")
    try:
        imu = setup_imu()
    except Exception as e:
        logger.error(f"  ImuProcessor() raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    if imu.calibrated:
        logger.warning("  FAIL should start uncalibrated")
        passed = False
    elif len(imu.gyro_buffer) != 0 or len(imu.accel_buffer) != 0:
        logger.warning("  FAIL buffers should start empty")
        passed = False
    elif not math.isclose(imu.orientation, 0.0, abs_tol=1e-9):
        logger.warning(f"  FAIL orientation should start at 0, got {imu.orientation}")
        passed = False
    else:
        logger.info("  OK   uncalibrated, empty buffers, orientation 0")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Filtering

def test_moving_average_scalar():
    """moving_average must return 0.0 for empty buffers and the mean otherwise."""
    passed = True
    logger.info("TEST 2: scalar moving average")
    try:
        imu = setup_imu()
    except Exception as e:
        logger.error(f"  setup raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    if not math.isclose(imu.moving_average([]), 0.0, abs_tol=1e-9):
        logger.warning("  FAIL empty buffer should average to 0.0")
        passed = False
    elif not math.isclose(imu.moving_average([1.0, 2.0, 3.0]), 2.0, abs_tol=1e-9):
        logger.warning("  FAIL [1,2,3] should average to 2.0")
        passed = False
    else:
        logger.info("  OK   empty -> 0.0, [1,2,3] -> 2.0")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_moving_average_vector():
    """moving_average_vector must return zeros(3) when empty, elementwise mean otherwise."""
    passed = True
    logger.info("TEST 3: vector moving average")
    try:
        imu = setup_imu()
    except Exception as e:
        logger.error(f"  setup raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    empty = imu.moving_average_vector([])
    if not np.allclose(empty, np.zeros(3)):
        logger.warning(f"  FAIL empty buffer gave {empty}")
        passed = False
    else:
        vecs = [np.array([1.0, 0.0, -1.0]), np.array([3.0, 2.0, 1.0])]
        mean = imu.moving_average_vector(vecs)
        if not np.allclose(mean, np.array([2.0, 1.0, 0.0])):
            logger.warning(f"  FAIL elementwise mean gave {mean}")
            passed = False
        else:
            logger.info("  OK   empty -> zeros, vectors -> elementwise mean")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_process_imu_data_returns_dict():
    """process_imu_data must return angular_velocity, linear_acceleration, orientation."""
    passed = True
    logger.info("TEST 4: process_imu_data output structure")
    try:
        imu = setup_imu()
        result = imu.process_imu_data(make_imu_msg(gyro_z=0.5, accel=(0.1, 0.0, 9.8)))
    except Exception as e:
        logger.error(f"  process_imu_data raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    for key in ('angular_velocity', 'linear_acceleration', 'orientation'):
        if key not in result:
            logger.warning(f"  FAIL missing key {key!r}")
            passed = False
    if passed and not math.isclose(result['angular_velocity'], 0.5, abs_tol=1e-9):
        logger.warning(f"  FAIL single-sample gyro average {result['angular_velocity']}, expected 0.5")
        passed = False
    if passed:
        logger.info("  OK   dict keys present, single sample passes through")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_filtering_smooths_noise():
    """The filtered gyro value must equal the mean of the buffered noisy samples."""
    passed = True
    logger.info("TEST 5: moving-average filtering smooths a noisy stream")
    try:
        imu = setup_imu(buffer_size=4)
        samples = [1.0, 1.2, 0.8, 1.0]
        result = None
        for s in samples:
            result = imu.process_imu_data(make_imu_msg(gyro_z=s))
        expected = sum(samples) / len(samples)
        if not math.isclose(result['angular_velocity'], expected, abs_tol=1e-9):
            logger.warning(f"  FAIL filtered={result['angular_velocity']}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   noisy stream filtered to {expected}")
    except Exception as e:
        logger.error(f"  raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Calibration

def test_calibrate_sets_flag():
    """calibrate() must mark the processor as calibrated."""
    passed = True
    logger.info("TEST 6: calibrate sets the calibrated flag")
    try:
        imu = setup_imu()
        imu.calibrate(num_samples=10)
        if not imu.calibrated:
            logger.warning("  FAIL calibrated flag not set")
            passed = False
        else:
            logger.info("  OK   calibrated flag set")
    except Exception as e:
        logger.error(f"  calibrate raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_bias_correction_applied():
    """Once calibrated, the stored gyro bias must be subtracted from readings."""
    passed = True
    logger.info("TEST 7: gyro bias correction after calibration")
    try:
        imu = setup_imu(buffer_size=1)
        imu.gyro_bias = 0.2
        imu.calibrated = True
        result = imu.process_imu_data(make_imu_msg(gyro_z=0.2))
        if not math.isclose(result['angular_velocity'], 0.0, abs_tol=1e-9):
            logger.warning(f"  FAIL bias not removed: {result['angular_velocity']}, expected 0.0")
            passed = False
        else:
            logger.info("  OK   stationary reading equal to bias filters to 0")
    except Exception as e:
        logger.error(f"  raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Motion / Orientation

def test_detect_motion():
    """detect_motion must trip above the acceleration threshold and not below."""
    passed = True
    logger.info("TEST 8: motion detection threshold")
    try:
        imu = setup_imu()
        imu.linear_acceleration = np.array([0.02, 0.0, 0.0])
        still = imu.detect_motion(threshold=0.1)
        imu.linear_acceleration = np.array([0.5, 0.5, 0.0])
        moving = imu.detect_motion(threshold=0.1)
        if still:
            logger.warning("  FAIL motion detected while (nearly) still")
            passed = False
        elif not moving:
            logger.warning("  FAIL no motion detected for large acceleration")
            passed = False
        else:
            logger.info("  OK   below threshold -> False, above -> True")
    except Exception as e:
        logger.error(f"  raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_orientation_tracks_gyro():
    """Integrating a constant gyro rate must track the true angle (within 10%).

    Spec for the complementary filter: with the gyro reporting a constant
    0.5 rad/s and no contradicting accelerometer reference, 2 seconds of
    updates must yield an orientation near 1.0 rad. A correct filter
    follows the gyro short-term; it must not decay the estimate toward 0.
    """
    passed = True
    logger.info("TEST 9: orientation tracks integrated gyro rate")
    try:
        imu = setup_imu()
        imu.angular_velocity = 0.5  # rad/s, as if filtered from the gyro stream
        dt = 0.01
        for _ in range(200):  # 2 seconds
            imu.update_orientation(dt)
        expected = 0.5 * 2.0
        if abs(imu.orientation - expected) > 0.1 * expected:
            logger.warning(f"  FAIL orientation={imu.orientation:.4f}, expected ~{expected} (+/-10%)")
            passed = False
        else:
            logger.info(f"  OK   orientation {imu.orientation:.4f} rad after 2 s at 0.5 rad/s")
    except Exception as e:
        logger.error(f"  raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_construction,
        test_moving_average_scalar,
        test_moving_average_vector,
        test_process_imu_data_returns_dict,
        test_filtering_smooths_noise,
        test_calibrate_sets_flag,
        test_bias_correction_applied,
        test_detect_motion,
        test_orientation_tracks_gyro,
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

    logger.info("IMU Processor Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
