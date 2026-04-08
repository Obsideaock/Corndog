# test_path_straightness.py
# No servos move. Traces the actual body-frame path that iklegs_move
# would send a leg through during a move from (0,0,0) to (0.06,0,0).
# If the path is straight, body_y and body_z should stay constant.
# If they drift, the path is curving.

import math
import numpy as np
from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
    t_rightback, t_rightfront, t_leftfront, t_leftback, ikine
)

BODY_LEN = 0.186
BODY_WID = 0.078

LEG_TRANSFORMS = {0: t_leftfront, 1: t_rightfront, 2: t_leftback, 3: t_rightback}
INV_LEG = {}
ht_body0 = homog_transxyz(0,0,0) @ homog_rotxyz(0,0,0)
for leg_idx, tf_fn in LEG_TRANSFORMS.items():
    T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
    INV_LEG[leg_idx] = ht_inverse(T)

leg_cfg = {
    0: {'base': ( BODY_LEN/2,  BODY_WID/2, -0.16), 'scale': ( 3/3.5*5/5.5, 3/3.5, 3/2.5)},
    1: {'base': ( BODY_LEN/2, -BODY_WID/2, -0.16), 'scale': (-3/3.5*5/4,   3/2,   3/3.75)},
    2: {'base': (-BODY_LEN/2,  BODY_WID/2, -0.16), 'scale': ( 3/4,          3/3.5, 3/2.5)},
    3: {'base': (-BODY_LEN/2, -BODY_WID/2, -0.16), 'scale': (-3/2.5,        3/2,   3/3.75)},
}

# Simulates exactly what iklegs_move does internally for one leg
def trace_path(leg_idx, dx_total, dy_total, dz_total, steps=20):
    cfg = leg_cfg[leg_idx]
    inc_x = dx_total / steps
    inc_y = dy_total / steps
    inc_z = dz_total / steps

    print(f"\nLeg {leg_idx} path from (0,0,0) to ({dx_total},{dy_total},{dz_total}):")
    print(f"  {'step':>4}  {'body_x':>8}  {'body_y':>8}  {'body_z':>8}  {'hip_y4':>8}")

    start_body_y = cfg['base'][1]
    start_body_z = cfg['base'][2]

    for s in range(1, steps + 1):
        ux = inc_x * s
        uy = inc_y * s
        uz = inc_z * s
        xb = cfg['base'][0] + ux * cfg['scale'][0]
        yb = cfg['base'][1] + uy * cfg['scale'][1]
        zb = cfg['base'][2] + uz * cfg['scale'][2]
        P_body = np.array([xb, yb, zb, 1.0])
        P_hip = INV_LEG[leg_idx] @ P_body

        y_drift = yb - start_body_y
        z_drift = zb - start_body_z
        print(f"  {s:>4}  {xb:>8.5f}  {yb:>8.5f}  {zb:>8.5f}  {P_hip[1]:>8.5f}  "
              f"{'<-- y drifts!' if abs(y_drift) > 0.0001 else ''}"
              f"{'<-- z drifts!' if abs(z_drift) > 0.0001 else ''}")

print("=" * 70)
print("Tracing paths. Non-zero drift in body_y or body_z = curved path.")
print("hip_y4 growing = abduction motor will fire.")
print("=" * 70)

# Test forward-only move on all legs
for leg in (0, 1, 2, 3):
    trace_path(leg, dx_total=0.06, dy_total=0.0, dz_total=0.0, steps=20)
