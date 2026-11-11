"""Mutation check: arc position computed for the WRONG turn direction (dtheta still correct).
Runs the real kinematics_test suite against the mutant."""
import sys, logging, numpy as np
sys.argv = ['kinematics_test.py']
import navi_bot.control.kinematics as K

def mutant(self, v, omega, theta, dt):
    if abs(omega) < 1e-6:
        return v * np.cos(theta) * dt, v * np.sin(theta) * dt, 0
    w = -omega                      # BUG: position uses mirrored turn
    R = v / w
    d = w * dt
    dx = R * (np.sin(theta + d) - np.sin(theta))
    dy = R * (-np.cos(theta + d) + np.cos(theta))
    return dx, dy, omega * dt

K.DifferentialDriveKinematics.compute_pose_update = mutant
print("mutant left turn (v=1,w=1,dt=0.5) ->", mutant(None, 1.0, 1.0, 0.0, 0.5))
sys.path.insert(0, 'tests')
import kinematics_test as T
logging.disable(logging.INFO)
res = {t.__name__: t() for t in [T.test_pose_straight_displacement_magnitude, T.test_pose_straight_direction_matches_heading,
       T.test_pose_straight_no_rotation, T.test_pose_arc_dtheta_equals_omega_dt, T.test_pose_arc_chord_length,
       T.test_pose_arc_heading_independence, T.test_pose_zero_velocity_no_movement, T.test_pose_arc_omega_sign_symmetry]}
print(res)
print("ALL POSE TESTS PASS ON MUTANT:", all(res.values()))
