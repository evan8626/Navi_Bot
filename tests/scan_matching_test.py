#!/usr/bin/env python3
"""
Acceptance tests for LidarProcessor.scan_match (ICP scan matching).

These are SPEC tests: they encode the intended contract and go green when the
implementation satisfies it. The contract under test:

    scan_match(current_scan, reference_scan,
               initial_guess=(0.0, 0.0, 0.0), max_iterations=..., tolerance=...)
        -> ((dx, dy, dtheta), fitness)

* Scans are sequences of (x, y) points (what __polar_to_cartesian yields).
* TRANSFORM CONVENTION — pinned to the project's own geometry utilities:
  applying the returned transform to each current point via
  utils.geometry.transform_point(p, (dx, dy, dtheta)) aligns it with the
  reference scan. (Test scans are generated with inverse_transform_point, so
  a correct matcher recovers the exact generating transform.)
* initial_guess is the motion prior (odometry delta): with a good prior the
  matcher must handle large motions it could not solve cold.
* fitness is the final alignment error (lower = better; ~0 on perfect data).
  Callers use it to decide whether to trust the match.
* Degenerate input (too few points) must NOT raise: return the initial guess
  and a clearly-bad fitness so the caller falls back to odometry.
* scan_match is pure: same inputs -> same outputs, no hidden cross-call
  state, inputs never mutated.

Tolerances: clean-data recovery within 2 cm / 0.5 deg; noisy or partial data
within 5-6 cm / 2-3 deg.
"""
import logging
import math
import sys

import numpy as np

from navi_bot.sensors.lidar_processing import LidarProcessor
from navi_bot.utils.geometry import (
    transform_point, inverse_transform_point, normalize_angle,
)

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Synthetic scan fixtures

def synthetic_room(spacing=0.05):
    """
    An asymmetric 'room' scan: two perpendicular walls of different lengths
    plus a round post off to one side. Asymmetry pins rotation uniquely —
    a symmetric cloud would let many transforms align equally well.
    """
    pts = []
    for x in np.arange(-1.0, 3.0, spacing):          # long wall along y = 2
        pts.append((float(x), 2.0))
    for y in np.arange(-1.5, 2.0, spacing):          # shorter wall along x = -1
        pts.append((-1.0, float(y)))
    for a in np.arange(0.0, 2 * math.pi, 0.35):      # post at (1.5, 0.5), r = 0.15
        pts.append((1.5 + 0.15 * math.cos(a), 0.5 + 0.15 * math.sin(a)))
    return pts


def make_current(reference, true_transform):
    """
    Build the 'current' scan such that applying true_transform to it (via
    transform_point) reproduces the reference exactly. A correct scan_match
    therefore returns true_transform.
    """
    return [inverse_transform_point(p, true_transform) for p in reference]


def call_scan_match(proc, current, reference, **kwargs):
    """
    Invoke scan_match and validate the return SHAPE against the contract:
    ((dx, dy, dtheta), fitness). Raises AssertionError with a spec message
    if the shape is wrong, so every test reports contract violations clearly.
    """
    result = proc.scan_match(current, reference, **kwargs)
    assert isinstance(result, tuple) and len(result) == 2, \
        f"scan_match must return ((dx, dy, dtheta), fitness), got {result!r}"
    transform, fitness = result
    assert len(tuple(transform)) == 3, \
        f"first element must be a 3-tuple (dx, dy, dtheta), got {transform!r}"
    for i, name in enumerate(('dx', 'dy', 'dtheta')):
        assert np.ndim(tuple(transform)[i]) == 0, \
            (f"{name} must be a SCALAR, got an array of shape "
             f"{np.shape(tuple(transform)[i])} — the transform is three numbers, "
             f"not point/vector arrays")
    assert np.ndim(fitness) == 0, \
        f"fitness must be a scalar, got shape {np.shape(fitness)}"
    return tuple(float(v) for v in transform), float(fitness)


def transform_close(got, want, lin_tol, ang_tol_deg):
    """Compare transforms: linear parts within lin_tol metres, angle within
    ang_tol_deg degrees (angle difference taken through normalize_angle)."""
    dlin = math.hypot(got[0] - want[0], got[1] - want[1])
    dang = abs(normalize_angle(got[2] - want[2]))
    return dlin <= lin_tol and dang <= math.radians(ang_tol_deg), dlin, dang


def recovery_test(label, true_transform, lin_tol=0.02, ang_tol_deg=0.5, **kwargs):
    """Shared body: generate scans from true_transform, demand recovery."""
    proc = LidarProcessor()
    reference = synthetic_room()
    current = make_current(reference, true_transform)
    try:
        got, fitness = call_scan_match(proc, current, reference, **kwargs)
    except Exception as e:
        logger.warning(f"  FAIL scan_match raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    ok, dlin, dang = transform_close(got, true_transform, lin_tol, ang_tol_deg)
    if ok:
        logger.info(f"  OK   {label}: recovered {tuple(round(v, 4) for v in got)} "
                    f"(off by {dlin*100:.2f} cm, {math.degrees(dang):.2f} deg; fitness {fitness:.5f})")
        logger.info("PASS")
        return True
    logger.warning(f"  FAIL {label}: got {tuple(round(v, 4) for v in got)}, "
                   f"want {true_transform} (off by {dlin*100:.2f} cm, {math.degrees(dang):.2f} deg)")
    logger.info("FAIL")
    return False


# MARK: Recovery on clean data

def test_identity():
    """Identity — matching a scan against itself yields (0, 0, 0) and ~zero fitness."""
    logger.info("TEST 1: identity — scan vs itself -> (0, 0, 0), fitness ~ 0")
    proc = LidarProcessor()
    scan = synthetic_room()
    try:
        got, fitness = call_scan_match(proc, list(scan), list(scan))
    except Exception as e:
        logger.warning(f"  FAIL scan_match raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    ok, dlin, dang = transform_close(got, (0.0, 0.0, 0.0), 0.01, 0.25)
    if not ok:
        logger.warning(f"  FAIL transform {got} is not identity")
        logger.info("FAIL")
        return False
    if fitness > 1e-6:
        logger.warning(f"  FAIL fitness {fitness} not ~0 on a perfect match")
        logger.info("FAIL")
        return False
    logger.info(f"  OK   identity recovered, fitness {fitness:.2e}")
    logger.info("PASS")
    return True


def test_pure_translation():
    """Pure translation — recovers (0.2, -0.1, 0) from clean data."""
    logger.info("TEST 2: pure translation (0.2, -0.1, 0)")
    return recovery_test("translation", (0.2, -0.1, 0.0))


def test_pure_rotation():
    """Pure rotation — recovers an 8 degree turn from clean data."""
    logger.info("TEST 3: pure rotation (0, 0, 8 deg)")
    return recovery_test("rotation", (0.0, 0.0, math.radians(8.0)))


def test_combined_motion():
    """Combined — recovers translation + rotation (0.15, 0.1, 5 deg)."""
    logger.info("TEST 4: combined motion (0.15, 0.1, 5 deg)")
    return recovery_test("combined", (0.15, 0.1, math.radians(5.0)))


def test_alignment_via_project_convention():
    """Convention — applying the returned transform via geometry.transform_point
    aligns current points onto the reference (mean residual < 2 cm)."""
    logger.info("TEST 5: convention — returned transform aligns via transform_point")
    proc = LidarProcessor()
    true_transform = (0.12, -0.08, math.radians(4.0))
    reference = synthetic_room()
    current = make_current(reference, true_transform)
    try:
        got, _ = call_scan_match(proc, current, reference)
    except Exception as e:
        logger.warning(f"  FAIL scan_match raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    moved = [transform_point(p, got) for p in current]
    ref = np.asarray(reference)
    residuals = [np.min(np.hypot(ref[:, 0] - x, ref[:, 1] - y)) for x, y in moved]
    mean_res = float(np.mean(residuals))
    if mean_res <= 0.02:
        logger.info(f"  OK   mean residual after applying transform: {mean_res*100:.2f} cm")
        logger.info("PASS")
        return True
    logger.warning(f"  FAIL mean residual {mean_res*100:.2f} cm — transform does not "
                   f"align current onto reference under transform_point()")
    logger.info("FAIL")
    return False


# MARK: The motion prior

def test_large_motion_with_prior():
    """Prior — a large motion (1.0, 0.6, 25 deg) is recovered when
    initial_guess carries an odometry-quality estimate of it."""
    logger.info("TEST 6: large motion with an odometry prior (initial_guess)")
    true_transform = (1.0, 0.6, math.radians(25.0))
    prior = (0.95, 0.55, math.radians(23.0))   # odometry-ish: close, not exact
    return recovery_test("large motion + prior", true_transform,
                         lin_tol=0.05, ang_tol_deg=2.0, initial_guess=prior)


# MARK: Robustness

def test_noisy_scan():
    """Noise — 1 cm gaussian noise on the current scan still recovers the
    motion within 5 cm / 2 deg."""
    logger.info("TEST 7: gaussian sensor noise (sigma = 1 cm)")
    proc = LidarProcessor()
    true_transform = (0.15, 0.05, math.radians(6.0))
    reference = synthetic_room()
    rng = np.random.default_rng(42)
    current = [(x + rng.normal(0, 0.01), y + rng.normal(0, 0.01))
               for x, y in make_current(reference, true_transform)]
    try:
        got, fitness = call_scan_match(proc, current, reference)
    except Exception as e:
        logger.warning(f"  FAIL scan_match raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    ok, dlin, dang = transform_close(got, true_transform, 0.05, 2.0)
    if ok:
        logger.info(f"  OK   recovered under noise (off by {dlin*100:.2f} cm, "
                    f"{math.degrees(dang):.2f} deg; fitness {fitness:.5f})")
        logger.info("PASS")
        return True
    logger.warning(f"  FAIL got {tuple(round(v, 4) for v in got)}, want {true_transform}")
    logger.info("FAIL")
    return False


def test_partial_overlap():
    """Partial overlap — the current scan misses 40% of the field of view
    (sensor occlusion) yet the motion is still recovered within 6 cm / 3 deg."""
    logger.info("TEST 8: partial overlap — 40% of current scan missing")
    proc = LidarProcessor()
    true_transform = (0.1, -0.05, math.radians(4.0))
    reference = synthetic_room()
    full = make_current(reference, true_transform)
    current = full[: int(len(full) * 0.6)]          # contiguous chunk dropped
    try:
        got, _ = call_scan_match(proc, current, reference)
    except Exception as e:
        logger.warning(f"  FAIL scan_match raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    ok, dlin, dang = transform_close(got, true_transform, 0.06, 3.0)
    if ok:
        logger.info(f"  OK   recovered with partial view (off by {dlin*100:.2f} cm, "
                    f"{math.degrees(dang):.2f} deg)")
        logger.info("PASS")
        return True
    logger.warning(f"  FAIL got {tuple(round(v, 4) for v in got)}, want {true_transform}")
    logger.info("FAIL")
    return False


# MARK: Fitness semantics

def test_fitness_reflects_quality():
    """Fitness — a genuine match scores far better (lower) than structurally
    unrelated scans, so the caller can tell when not to trust ICP."""
    logger.info("TEST 9: fitness — good match scores far better than a non-match")
    proc = LidarProcessor()
    reference = synthetic_room()
    good_current = make_current(reference, (0.1, 0.05, math.radians(3.0)))
    rng = np.random.default_rng(7)
    junk_current = [(float(x), float(y))
                    for x, y in rng.uniform(-1.0, 2.0, size=(120, 2))]
    try:
        _, good_fit = call_scan_match(proc, good_current, reference)
        _, junk_fit = call_scan_match(proc, junk_current, reference)
    except Exception as e:
        logger.warning(f"  FAIL scan_match raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    if junk_fit > 10 * max(good_fit, 1e-6) and junk_fit > 5e-3:
        logger.info(f"  OK   good fitness {good_fit:.5f} vs junk fitness {junk_fit:.5f}")
        logger.info("PASS")
        return True
    logger.warning(f"  FAIL good {good_fit:.6f} vs junk {junk_fit:.6f} — fitness does "
                   f"not separate trustworthy from untrustworthy matches")
    logger.info("FAIL")
    return False


def test_degenerate_input_degrades():
    """Degenerate — too few points must NOT raise: return the initial guess
    with a clearly-bad fitness so the caller falls back to odometry."""
    logger.info("TEST 10: degenerate input — no exception, prior returned, bad fitness")
    proc = LidarProcessor()
    reference = synthetic_room()
    prior = (0.3, -0.2, math.radians(10.0))
    passed = True
    for label, current in (('two points', [(0.0, 0.0), (1.0, 1.0)]), ('empty', [])):
        try:
            got, fitness = call_scan_match(proc, current, reference, initial_guess=prior)
        except Exception as e:
            logger.warning(f"  FAIL {label}: raised {type(e).__name__}: {e} — must degrade, not die")
            passed = False
            continue
        ok, _, _ = transform_close(got, prior, 1e-6, 1e-4)
        if not ok:
            logger.warning(f"  FAIL {label}: returned {got}, expected the initial guess {prior}")
            passed = False
        elif not (math.isinf(fitness) or fitness > 0.05):
            logger.warning(f"  FAIL {label}: fitness {fitness} does not signal 'do not trust'")
            passed = False
        else:
            logger.info(f"  OK   {label}: prior returned with bad fitness ({fitness})")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Purity

def test_stateless_and_nonmutating():
    """Purity — identical calls agree exactly, a garbage call in between has
    no effect, and input scans are never mutated."""
    logger.info("TEST 11: purity — deterministic, no hidden state, no input mutation")
    proc = LidarProcessor()
    reference = synthetic_room()
    current = make_current(reference, (0.15, 0.1, math.radians(5.0)))
    cur_copy = [tuple(p) for p in current]
    ref_copy = [tuple(p) for p in reference]
    passed = True
    try:
        first, _ = call_scan_match(proc, current, reference)
        call_scan_match(proc, [(0.0, 0.0)], reference,
                        initial_guess=(9.0, 9.0, 3.0))          # garbage in between
        second, _ = call_scan_match(proc, current, reference)
    except Exception as e:
        logger.warning(f"  FAIL raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    if any(abs(a - b) > 1e-9 for a, b in zip(first, second)):
        logger.warning(f"  FAIL results differ across identical calls: {first} vs {second}")
        passed = False
    else:
        logger.info("  OK   identical calls agree; garbage call left no residue")
    if [tuple(p) for p in current] != cur_copy or [tuple(p) for p in reference] != ref_copy:
        logger.warning("  FAIL scan_match mutated its input scans")
        passed = False
    else:
        logger.info("  OK   input scans unmodified")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_accepts_tuning_kwargs():
    """API — accepts initial_guess / max_iterations / tolerance keywords and
    returns promptly with a small iteration budget (no hang)."""
    logger.info("TEST 12: API — tuning kwargs accepted; small budget returns promptly")
    proc = LidarProcessor()
    reference = synthetic_room()
    current = make_current(reference, (0.05, 0.02, math.radians(2.0)))
    try:
        got, fitness = call_scan_match(proc, current, reference,
                                       initial_guess=(0.0, 0.0, 0.0),
                                       max_iterations=1, tolerance=1e-5)
    except TypeError as e:
        if 'argument' in str(e) or 'keyword' in str(e):
            logger.warning(f"  FAIL signature rejects tuning kwargs: {e}")
        else:
            logger.warning(f"  FAIL TypeError from inside scan_match (not the signature): {e}")
        logger.info("FAIL")
        return False
    except Exception as e:
        logger.warning(f"  FAIL raised {type(e).__name__}: {e}")
        logger.info("FAIL")
        return False
    logger.info(f"  OK   max_iterations=1 returned {tuple(round(v, 4) for v in got)} "
                f"(fitness {fitness:.5f}) without hanging")
    logger.info("PASS")
    return True


# MARK: Runner

def main():
    tests = [
        test_identity,
        test_pure_translation,
        test_pure_rotation,
        test_combined_motion,
        test_alignment_via_project_convention,
        test_large_motion_with_prior,
        test_noisy_scan,
        test_partial_overlap,
        test_fitness_reflects_quality,
        test_degenerate_input_degrades,
        test_stateless_and_nonmutating,
        test_accepts_tuning_kwargs,
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

    logger.info("Scan Matching (ICP) Acceptance Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
