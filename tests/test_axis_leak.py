# test_axis_leak.py
# Run this on the Pi. No servos move. Just prints numbers.
# What to look for: P_hip[1] (the y4 value that drives the abduction motor)
# should ideally be close to zero for all legs when dy=0.
# If legs 0 and 2 show a noticeably larger P_hip[1] than legs 1 and 3,
# the axis leak is confirmed.

import numpy as np
from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
    t_rightback, t_rightfront, t_leftfront, t_leftback
)

BODY_LEN = 0.186
BODY_WID = 0.078

LEG_TRANSFORMS = {
    0: t_leftfront,
    1: t_rightfront,
    2: t_leftback,
    3: t_rightback,
}

INV_LEG = {}
ht_body0 = homog_transxyz(0,0,0) @ homog_rotxyz(0,0,0)
for leg_idx, tf_fn in LEG_TRANSFORMS.items():
    T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
    INV_LEG[leg_idx] = ht_inverse(T)

leg_bases = {
    0: ( BODY_LEN/2,  BODY_WID/2, -0.16),
    1: ( BODY_LEN/2, -BODY_WID/2, -0.16),
    2: (-BODY_LEN/2,  BODY_WID/2, -0.16),
    3: (-BODY_LEN/2, -BODY_WID/2, -0.16),
}

test_offsets_x = [0.0, 0.02, 0.04, 0.06]  # forward steps, no sideways (dy=0)

print("=" * 60)
print("Testing x-only movement (dy=0 always)")
print("y4 = hip-frame sideways value. Should stay near 0.")
print("=" * 60)

for dx in test_offsets_x:
    print(f"\n--- dx = {dx:.3f}m forward ---")
    for leg_idx in (0, 1, 2, 3):
        bx, by, bz = leg_bases[leg_idx]
        P_body = np.array([bx + dx, by, bz, 1.0])
        P_hip = INV_LEG[leg_idx] @ P_body
        print(f"  Leg {leg_idx}: hip x={P_hip[0]:.5f}  y={P_hip[1]:.5f}  z={P_hip[2]:.5f}   <-- y4 (abduction driver)")
