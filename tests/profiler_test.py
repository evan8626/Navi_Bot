#!/usr/bin/env python3
"""
Test script for the performance profiler in navi_bot/utils/profiler.py

Tests that the profiler correctly:
- Times tasks via start/stop, the Timer context manager, and @profile
- Tracks deadline misses
- Caps history at history_size
- Computes summary statistics
- Resets per-task and globally

Note: the class name is imported as spelled in the source
(PerforamnceProfiler). The statistics tests are acceptance tests for the
intended behavior of get_statistics.
"""
import logging
import math
import sys
import time

from navi_bot.utils.profiler import PerformanceProfiler, Timer, profile, global_profiler

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_profiler(history_size=100):
    return PerformanceProfiler(history_size=history_size)


# MARK: Timing

def test_start_stop_returns_elapsed():
    """start_timing/stop_timing must return a plausible elapsed time."""
    passed = True
    logger.info("TEST 1: start/stop returns elapsed wall time")
    prof = setup_profiler()
    prof.start_timing('task')
    time.sleep(0.02)
    elapsed = prof.stop_timing('task')
    if elapsed is None or not (0.015 <= elapsed <= 0.5):
        logger.warning(f"  FAIL elapsed={elapsed}, expected roughly 0.02s")
        passed = False
    else:
        logger.info(f"  OK   slept 20 ms, measured {elapsed*1000:.2f} ms")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_stop_without_start():
    """stop_timing on a task that was never started must return None."""
    passed = True
    logger.info("TEST 2: stop without start returns None")
    prof = setup_profiler()
    result = prof.stop_timing('never_started')
    if result is not None:
        logger.warning(f"  FAIL got {result}, expected None")
        passed = False
    else:
        logger.info("  OK   returned None")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_samples_recorded():
    """Each start/stop cycle must append one sample to the task's history."""
    passed = True
    logger.info("TEST 3: samples are recorded per stop")
    prof = setup_profiler()
    for _ in range(3):
        prof.start_timing('task')
        prof.stop_timing('task')
    count = len(prof.execution_times['task'])
    if count != 3:
        logger.warning(f"  FAIL recorded {count} samples, expected 3")
        passed = False
    else:
        logger.info("  OK   3 cycles -> 3 samples")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_history_size_cap():
    """The sample history must be capped at history_size (oldest dropped)."""
    passed = True
    logger.info("TEST 4: history capped at history_size")
    prof = setup_profiler(history_size=5)
    for _ in range(8):
        prof.start_timing('task')
        prof.stop_timing('task')
    count = len(prof.execution_times['task'])
    if count != 5:
        logger.warning(f"  FAIL history holds {count} samples, expected cap of 5")
        passed = False
    else:
        logger.info("  OK   8 cycles with history_size=5 -> 5 retained")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Deadlines

def test_deadline_misses_counted():
    """Exceeding the deadline must increment the miss count; meeting it must not."""
    passed = True
    logger.info("TEST 5: deadline misses counted")
    prof = setup_profiler()
    # Guaranteed miss: zero deadline
    prof.start_timing('task')
    prof.stop_timing('task', deadline=0.0)
    # Guaranteed hit: huge deadline
    prof.start_timing('task')
    prof.stop_timing('task', deadline=10.0)
    deadline, misses = prof.deadlines['task']
    if misses != 1:
        logger.warning(f"  FAIL miss count {misses}, expected 1")
        passed = False
    else:
        logger.info("  OK   one miss and one hit -> miss count 1")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Statistics

def test_get_statistics_values():
    """get_statistics must report mean/max/min/std/count over the history."""
    passed = True
    logger.info("TEST 6: get_statistics computes correct values")
    prof = setup_profiler()
    samples = [0.01, 0.02, 0.03]
    prof.execution_times['task'].extend(samples)
    try:
        stats = prof.get_statistics('task')
        expected = {
            'mean': 0.02, 'max': 0.03, 'min': 0.01, 'count': 3,
        }
        for key, value in expected.items():
            if not math.isclose(stats[key], value, abs_tol=1e-9):
                logger.warning(f"  FAIL {key}={stats[key]}, expected {value}")
                passed = False
        if passed:
            logger.info(f"  OK   mean/max/min/count correct over {samples}")
    except Exception as e:
        logger.error(f"  get_statistics raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_get_statistics_unknown_task():
    """get_statistics for an unknown task must return None."""
    passed = True
    logger.info("TEST 7: get_statistics unknown task returns None")
    prof = setup_profiler()
    stats = prof.get_statistics('nope')
    if stats is not None:
        logger.warning(f"  FAIL got {stats}, expected None")
        passed = False
    else:
        logger.info("  OK   returned None")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_get_statistics_deadline_fields():
    """With a deadline set, statistics must include deadline, misses, and miss rate."""
    passed = True
    logger.info("TEST 8: get_statistics includes deadline fields")
    prof = setup_profiler()
    prof.start_timing('task')
    prof.stop_timing('task', deadline=0.0)   # one miss
    prof.start_timing('task')
    prof.stop_timing('task', deadline=10.0)  # one hit
    try:
        stats = prof.get_statistics('task')
        if stats.get('deadline_misses') != 1:
            logger.warning(f"  FAIL deadline_misses={stats.get('deadline_misses')}, expected 1")
            passed = False
        elif not math.isclose(stats.get('deadline_miss_rate', -1), 0.5, abs_tol=1e-9):
            logger.warning(f"  FAIL deadline_miss_rate={stats.get('deadline_miss_rate')}, expected 0.5")
            passed = False
        else:
            logger.info("  OK   deadline fields present and correct")
    except Exception as e:
        logger.error(f"  get_statistics raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Reset

def test_reset():
    """reset(task) must clear only that task; reset() must clear everything."""
    passed = True
    logger.info("TEST 9: reset single task and reset all")
    prof = setup_profiler()
    for name in ('a', 'b'):
        prof.start_timing(name)
        prof.stop_timing(name, deadline=0.0)
    prof.reset('a')
    if 'a' in prof.execution_times or 'a' in prof.deadlines:
        logger.warning("  FAIL task 'a' still present after reset('a')")
        passed = False
    elif 'b' not in prof.execution_times:
        logger.warning("  FAIL task 'b' was wrongly cleared by reset('a')")
        passed = False
    else:
        prof.reset()
        if len(prof.execution_times) != 0 or len(prof.deadlines) != 0:
            logger.warning("  FAIL reset() did not clear all tasks")
            passed = False
    if passed:
        logger.info("  OK   per-task and global reset behave correctly")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Helpers

def test_timer_context_manager():
    """The Timer context manager must record one sample for its task."""
    passed = True
    logger.info("TEST 10: Timer context manager records a sample")
    prof = setup_profiler()
    with Timer(prof, 'ctx_task', deadline=10.0):
        time.sleep(0.005)
    count = len(prof.execution_times['ctx_task'])
    if count != 1:
        logger.warning(f"  FAIL recorded {count} samples, expected 1")
        passed = False
    else:
        logger.info("  OK   one with-block -> one sample")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_profile_decorator():
    """@profile must record into global_profiler and pass through the return value."""
    passed = True
    logger.info("TEST 11: @profile decorator records and returns")
    task_name = 'profiler_test_decorated_task'

    @profile(task_name)
    def add(a, b):
        return a + b

    result = add(2, 3)
    count = len(global_profiler.execution_times[task_name])
    if result != 5:
        logger.warning(f"  FAIL decorated function returned {result}, expected 5")
        passed = False
    elif count != 1:
        logger.warning(f"  FAIL global profiler recorded {count} samples, expected 1")
        passed = False
    else:
        logger.info("  OK   return value preserved, sample recorded globally")
    global_profiler.reset(task_name)
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_start_stop_returns_elapsed,
        test_stop_without_start,
        test_samples_recorded,
        test_history_size_cap,
        test_deadline_misses_counted,
        test_get_statistics_values,
        test_get_statistics_unknown_task,
        test_get_statistics_deadline_fields,
        test_reset,
        test_timer_context_manager,
        test_profile_decorator,
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

    logger.info("Performance Profiler Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
