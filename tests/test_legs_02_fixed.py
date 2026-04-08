# test_all_legs.py — Decoupled IK + Mirror
#
# Two fixes combined:
#
# 1. Decoupled IK: replaces the library's ikine() on right legs.
#    - q1 uses x4_home instead of actual x4 → no drift during z/x moves
#    - q2/q3 use y4_home instead of actual y4 → no height change during lateral moves
#    - At home, produces identical results to original ikine
#
# 2. Mirror: left legs copy negated deltas from right legs.
#    - Eliminates left/right asymmetry entirely
#    - Left legs don't need their own transforms, IK, or scales

import time
import math
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice
from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
    t_rightfront, t_rightback, ikine
)

# ===========================================================================
# Hardware setup
# ===========================================================================

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

OE_PIN = 22
output_enable = OutputDevice(OE_PIN, active_high=False)

servo_channels = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]

servo_home = {
    0: 39, 1: 231, 4: 222, 5: 50,
    6: 128, 7: 130, 8: 133, 9: 135,
    10: 73, 11: 204, 14: 235, 15: 33
}

servos = {ch: servo.Servo(pca.channels[ch]) for ch in servo_channels}
for ch in servos:
    servos[ch].set_pulse_width_range(500, 2500)
    servos[ch].actuation_range = 270

servo_angles = {}

# ===========================================================================
# Robot geometry — only right legs needed
# ===========================================================================

BODY_LEN = 0.186
BODY_WID = 0.078
L1, L2, L3 = 0.055, 0.1075, 0.130

MAPPING_RIGHT = {
    1: {1: {'sign': -1, 'offset': 228}, 2: {'sign': -1, 'offset': 112}, 3: {'sign': -1, 'offset': 165}},
    3: {1: {'sign': +1, 'offset': 35},  2: {'sign': -1, 'offset': 89},  3: {'sign': -1, 'offset': 161}},
}

CHANNEL_MAP = {
    0: {1: 9,  2: 11, 3: 15},
    1: {1: 8,  2: 10, 3: 14},
    2: {1: 6,  2:  4, 3:  0},
    3: {1: 7,  2:  5, 3:  1},
}

# Mirror pairs: left_leg -> right_leg
MIRROR_PAIR = {0: 1, 2: 3}

# How left leg deltas relate to right leg deltas per joint:
#   joint 1 (hip): same sign (both splay or both tuck)
#   joint 2 (shoulder): negated (left/right are mirrored)
#   joint 3 (ankle): negated
MIRROR_SIGN = {1: +1, 2: -1, 3: -1}

# Right leg transforms and config
INV_RIGHT = {}
ht_body0 = homog_transxyz(0, 0, 0) @ homog_rotxyz(0, 0, 0)
for ri, tf_fn in {1: t_rightfront, 3: t_rightback}.items():
    T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
    INV_RIGHT[ri] = ht_inverse(T)

RIGHT_CFG = {
    1: {'base': ( BODY_LEN/2, -BODY_WID/2, -0.16), 'scale': (-3/3.5*5/4, 3/2, 3/3.75)},
    3: {'base': (-BODY_LEN/2, -BODY_WID/2, -0.16), 'scale': (-3/2.5,     3/2, 3/3.75)},
}

# Precompute home hip-frame coordinates for each right leg
HOME_HIP = {}
for ri in (1, 3):
    bx, by, bz = RIGHT_CFG[ri]['base']
    P = INV_RIGHT[ri] @ np.array([bx, by, bz, 1.0])
    HOME_HIP[ri] = (float(P[0]), float(P[1]), float(P[2]))

# Precompute right leg servo values at home (exact, not rounded)
RIGHT_HOME_SERVO = {}
for ri in (1, 3):
    hx, hy, hz = HOME_HIP[ri]
    q = ikine(hx, hy, hz, L1, L2, L3, legs12=None)
    cfg = MAPPING_RIGHT[ri]
    for j in (1, 2, 3):
        ch = CHANNEL_MAP[ri][j]
        RIGHT_HOME_SERVO[ch] = cfg[j]['sign'] * (q[j-1] * 180 / math.pi) + cfg[j]['offset']

# ===========================================================================
# Decoupled IK
# ===========================================================================

def decoupled_ikine(x4, y4, z4, l1, l2, l3, x4_home, y4_home):
    """Fully decoupled inverse kinematics.
    
    q1: computed with x4_home (not actual x4) → responds only to y changes.
    q2/q3: computed with y4_home (not actual y4) → respond only to x/z changes.
    
    At home (x4=x4_home, y4=y4_home), produces identical results to ikine().
    """
    # q1 from y4 and x4_home
    r_sq = x4_home**2 + y4**2 - l1**2
    if r_sq < 0:
        return None
    q1 = math.atan2(y4, x4_home) + math.atan2(math.sqrt(r_sq), -l1)

    # q2/q3 from x4, z4, and y4_home
    r_sq_sag = x4**2 + y4_home**2 - l1**2
    if r_sq_sag < 0:
        return None
    D = (r_sq_sag + z4**2 - l2**2 - l3**2) / (2 * l2 * l3)
    if abs(D) > 1.0:
        return None
    q3 = math.atan2(-math.sqrt(1 - D**2), D)
    q2 = math.atan2(z4, math.sqrt(r_sq_sag)) - math.atan2(l3 * math.sin(q3), l2 + l3 * math.cos(q3))

    return (q1, q2, q3)

# ===========================================================================
# Helpers
# ===========================================================================

def clamp_angle(a):
    return max(0.0, min(270.0, a))

def initialize_servo_angles():
    global servo_angles
    servo_angles = {ch: float(ang) for ch, ang in servo_home.items()}

def to_user_angles_right(leg_idx, theta_rads):
    cfg = MAPPING_RIGHT[leg_idx]
    return tuple(
        cfg[j]['sign'] * (theta_rads[j-1] * 180 / math.pi) + cfg[j]['offset']
        for j in (1, 2, 3)
    )

def move_motors_absolute(targets, steps=30, delay=0.01):
    current = {ch: servo_angles[ch] for ch in targets}
    increments = {ch: (targets[ch] - current[ch]) / steps for ch in targets}

    for step in range(steps):
        for ch in targets:
            new_angle = clamp_angle(current[ch] + increments[ch] * (step + 1))
            servos[ch].angle = new_angle
            servo_angles[ch] = new_angle
        time.sleep(delay)

    for ch, tgt in targets.items():
        clamped = clamp_angle(tgt)
        servos[ch].angle = clamped
        servo_angles[ch] = clamped

# ===========================================================================
# Move all 4 legs
# ===========================================================================

def move_all_legs(dx, dy, dz, steps=30, delay=0.01, verbose=True):
    targets = {}

    # --- Step 1: Compute right legs with decoupled IK ---
    right_servos = {}
    for ri in (1, 3):
        cfg = RIGHT_CFG[ri]
        sx, sy, sz = cfg['scale']
        xb = cfg['base'][0] + dx * sx
        yb = cfg['base'][1] + dy * sy
        zb = cfg['base'][2] + dz * sz

        P_body = np.array([xb, yb, zb, 1.0])
        P_hip = INV_RIGHT[ri] @ P_body
        hx_home, hy_home, _ = HOME_HIP[ri]

        if verbose:
            print(f"  Leg {ri} body target: x={xb:.5f}  y={yb:.5f}  z={zb:.5f}")
            print(f"  Leg {ri} hip target:  x={P_hip[0]:.5f}  y={P_hip[1]:.5f}  z={P_hip[2]:.5f}")

        result = decoupled_ikine(P_hip[0], P_hip[1], P_hip[2], L1, L2, L3, hx_home, hy_home)
        if result is None:
            print(f"  Leg {ri}: IK failed (unreachable) — skipping pair.")
            continue

        user_deg = to_user_angles_right(ri, result)

        if verbose:
            print(f"  Leg {ri} servo targets: "
                  f"{dict(zip(CHANNEL_MAP[ri].values(), [f'{a:.1f}' for a in user_deg]))}")

        for j in (1, 2, 3):
            ch = CHANNEL_MAP[ri][j]
            right_servos[ch] = user_deg[j-1]
            targets[ch] = clamp_angle(user_deg[j-1])

    # --- Step 2: Mirror right deltas to left legs ---
    for li, ri in MIRROR_PAIR.items():
        ok = all(CHANNEL_MAP[ri][j] in right_servos for j in (1, 2, 3))
        if not ok:
            if verbose:
                print(f"  Leg {li}: skipped (right pair unreachable)")
            continue

        left_vals = []
        for j in (1, 2, 3):
            ch_R = CHANNEL_MAP[ri][j]
            ch_L = CHANNEL_MAP[li][j]
            delta_R = right_servos[ch_R] - RIGHT_HOME_SERVO[ch_R]
            left_val = servo_home[ch_L] + MIRROR_SIGN[j] * delta_R
            targets[ch_L] = clamp_angle(left_val)
            left_vals.append(left_val)

        if verbose:
            print(f"  Leg {li} (mirrored from {ri}) servo targets: "
                  f"{dict(zip(CHANNEL_MAP[li].values(), [f'{a:.1f}' for a in left_vals]))}")

    if targets:
        move_motors_absolute(targets, steps=steps, delay=delay)
    else:
        print("  No legs moved (all IK failed).")

# ===========================================================================
# Stand up / Main
# ===========================================================================

def stand_up():
    print("Returning to home position...")
    for ch in servo_channels:
        servos[ch].angle = servo_home[ch]
    initialize_servo_angles()
    time.sleep(0.8)
    print("Home.")

def main():
    print("\n=== All 4 Legs Test (Decoupled IK + Mirror) ===")
    print("Hardware initializing...")

    output_enable.off()
    time.sleep(0.2)
    stand_up()

    print("\nEnter offsets: dx dy dz  (or 'quit')")
    print("  dx = forward/back  (positive = forward)")
    print("  dy = sideways      (positive = left)")
    print("  dz = up/down       (positive = up)\n")

    while True:
        try:
            raw = input("dx dy dz > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if raw.lower() in ('quit', 'q', 'exit'):
            break

        parts = raw.split()
        if len(parts) != 3:
            print("  Please enter exactly 3 values: dx dy dz")
            continue

        try:
            dx, dy, dz = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  Invalid numbers. Try again.")
            continue

        print(f"\nMoving to offset ({dx:.4f}, {dy:.4f}, {dz:.4f})...")
        move_all_legs(dx, dy, dz, steps=30, delay=0.01, verbose=True)
        print("Done.\n")

    print("\nExiting — returning to home.")
    stand_up()
    output_enable.on()
    for ch in servo_channels:
        pca.channels[ch].duty_cycle = 0
    print("Servos disabled.")

if __name__ == "__main__":
    main()
