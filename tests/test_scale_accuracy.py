from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import t_leftfront
import numpy as np

BODY_LEN, BODY_WID = 0.186, 0.078
T = t_leftfront(homog_transxyz(0,0,0) @ homog_rotxyz(0,0,0), BODY_LEN, BODY_WID)
INV = ht_inverse(T)

for z in [x * 0.01 for x in range(-30, 5)]:
    P = INV @ np.array([BODY_LEN/2, BODY_WID/2, z, 1.0])
    print(f"body z={z:.3f}  hip x={P[0]:.4f}  hip y={P[1]:.4f}  hip z={P[2]:.4f}")
