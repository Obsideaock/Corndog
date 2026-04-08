# test_all_legs.py
# Legs 0 and 2: fixed IK with q1 locked to home when dy=0
# Legs 1 and 3: original iklegs_move logic from main.py (unchanged)
# All 4 legs move together so you can directly compare behavior.

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
    t_leftfront, t_leftback, t_rightfront, t_rightback, ikine
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
# Robot geometry
# ===========================================================================

BODY_LEN = 0.186
BODY_WID = 0.078
L1, L2, L3 = 0.055, 0.1075, 0.130

LEG_TRANSFORMS = {
    0: t_leftfront,
    1: t_rightfront,
    2: t_leftback,
    3: t_rightback,
}

MAPPING = {
    0: {1: {'sign': +1, 'offset': -143}, 2: {'sign': +1, 'offset': 132},  3: {'sign': +1, 'offset': 156}},
    1: {1: {'sign': -1, 'offset': 228},  2: {'sign': -1, 'offset': 112},  3: {'sign': -1, 'offset': 165}},
    2: {1: {'sign': -1, 'offset': 406},  2: {'sign': +1, 'offset': 150},  3: {'sign': +1, 'offset': 162}},
    3: {1: {'sign': +1, 'offset': 35},   2: {'sign': -1, 'offset': 89},   3: {'sign': -1, 'offset': 161}},
}

CHANNEL_MAP = {
    0: {1: 9,  2: 11, 3: 15},
    1: {1: 8,  2: 10, 3: 14},
    2: {1: 6,  2:  4, 3:  0},
    3: {1: 7,  2:  5, 3:  1},
}

# Precompute inverse transforms for all legs
INV_LEG = {}
ht_body0 = homog_transxyz(0, 0, 0) @ homog_rotxyz(0, 0, 0)
for leg_idx, tf_fn in LEG_TRANSFORMS.items():
    T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
    INV_LEG[leg_idx] = ht_inverse(T)

# ---------------------------------------------------------------------------
# leg_cfg scales
# ---------------------------------------------------------------------------

leg_cfg_02 = {
    0: {'base': ( BODY_LEN/2,  BODY_WID/2, -0.16), 'scale': ( 3/3.5*5/5.5, 3/3.5, 3/3)},
    2: {'base': (-BODY_LEN/2,  BODY_WID/2, -0.16), 'scale': ( 3/4,         3/3.5, 3/3)},
}

leg_cfg_13 = {
    1: {'base': ( BODY_LEN/2, -BODY_WID/2, -0.16), 'scale': (-3/3.5*5/4,   3/2,   3/3.75)},
    3: {'base': (-BODY_LEN/2, -BODY_WID/2, -0.16), 'scale': (-3/2.5,       3/2,   3/3.75)},
}

# ===========================================================================
# Helpers
# ===========================================================================

def clamp_angle(a):
    return max(0.0, min(270.0, a))

def initialize_servo_angles():
    global servo_angles
    servo_angles = {ch: float(ang) for ch, ang in servo_home.items()}

def to_user_angles(leg_idx, theta_rads):
    cfg = MAPPING[leg_idx]
    return tuple(
        cfg[j]['sign'] * (theta_rads[j-1] * 180 / math.pi) + cfg[j]['offset']
        for j in (1, 2, 3)
    )

def solve_ik(x_h, y_h, z_h, legs12):
    try:
        return ikine(x_h, y_h, z_h, L1, L2, L3, legs12=legs12)
    except ValueError:
        return None

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
    """
    Move all 4 legs to the given offset from home.
    dx = forward/back (positive = forward)
    dy = sideways
    dz = up/down      (positive = up)

    Legs 0 and 2: body frame -> INV_LEG -> IK, but q1 locked to home when dy=0
    Legs 1 and 3: original main.py approach unchanged
    """
    targets = {}

    # --- Legs 0 and 2 ---
    for leg_idx in (0, 2):
        cfg = leg_cfg_02[leg_idx]
        sx, sy, sz = cfg['scale']

        xb = cfg['base'][0] + dx * sx
        yb = cfg['base'][1] + dy * sy
        zb = cfg['base'][2] + dz * sz

        P_body = np.array([xb, yb, zb, 1.0])
        P_hip = INV_LEG[leg_idx] @ P_body

        if verbose:
            print(f"  Leg {leg_idx} body target: x={xb:.5f}  y={yb:.5f}  z={zb:.5f}")
            print(f"  Leg {leg_idx} hip target:  x={P_hip[0]:.5f}  y={P_hip[1]:.5f}  z={P_hip[2]:.5f}")

        result = solve_ik(P_hip[0], P_hip[1], P_hip[2], legs12=None)
        if result is None:
            print(f"  Leg {leg_idx}: IK failed — skipping.")
            continue

        q1, q2, q3 = result
        user_deg = to_user_angles(leg_idx, (q1, q2, q3))

        if verbose:
            print(f"  Leg {leg_idx} servo targets: "
                  f"{dict(zip(CHANNEL_MAP[leg_idx].values(), [f'{a:.1f}' for a in user_deg]))}")

        for joint, tgt in enumerate(user_deg, start=1):
            ch = CHANNEL_MAP[leg_idx][joint]
            if joint == 1 and abs(dy) < 1e-6:
                # lock abduction to home when no sideways movement requested
                targets[ch] = float(servo_home[ch])
            else:
                targets[ch] = clamp_angle(tgt)

    # --- Legs 1 and 3: original main.py approach ---
    for leg_idx in (1, 3):
        cfg = leg_cfg_13[leg_idx]
        xb = cfg['base'][0] + dx * cfg['scale'][0]
        yb = cfg['base'][1] + dy * cfg['scale'][1]
        zb = cfg['base'][2] + dz * cfg['scale'][2]

        P_body = np.array([xb, yb, zb, 1.0])
        P_hip = INV_LEG[leg_idx] @ P_body

        if verbose:
            print(f"  Leg {leg_idx} body target: x={xb:.5f}  y={yb:.5f}  z={zb:.5f}")
            print(f"  Leg {leg_idx} hip target:  x={P_hip[0]:.5f}  y={P_hip[1]:.5f}  z={P_hip[2]:.5f}")

        front = (not (leg_idx in (0, 1))) ^ (leg_idx in (2, 3))
        result = solve_ik(P_hip[0], P_hip[1], P_hip[2], legs12=front)
        if result is None:
            print(f"  Leg {leg_idx}: IK failed — skipping.")
            continue

        q1, q2, q3 = result
        user_deg = to_user_angles(leg_idx, (q1, q2, q3))

        if verbose:
            print(f"  Leg {leg_idx} servo targets: "
                  f"{dict(zip(CHANNEL_MAP[leg_idx].values(), [f'{a:.1f}' for a in user_deg]))}")

        for joint, tgt in enumerate(user_deg, start=1):
            ch = CHANNEL_MAP[leg_idx][joint]
            targets[ch] = clamp_angle(tgt)

    if targets:
        move_motors_absolute(targets, steps=steps, delay=delay)
    else:
        print("  No legs moved (all IK failed).")

# ===========================================================================
# Stand up
# ===========================================================================

def stand_up():
    print("Returning to home position...")
    for ch in servo_channels:
        servos[ch].angle = servo_home[ch]
    initialize_servo_angles()
    time.sleep(0.8)
    print("Home.")

# ===========================================================================
# Main interactive loop
# ===========================================================================

def main():
    print("\n=== All 4 Legs Test ===")
    print("Hardware initializing...")

    output_enable.off()
    time.sleep(0.2)
    stand_up()

    print("\nEnter offsets from home, space separated: dx dy dz")
    print("  dx = forward/back  (positive = forward)")
    print("  dy = sideways      (positive = left)")
    print("  dz = up/down       (positive = up)")
    print("Examples:")
    print("  0.05 0 0      forward 5cm")
    print("  0 0 0.02      up 2cm")
    print("  0 0.02 0      sideways 2cm (tests abduction)")
    print("  0 0 0         return to home")
    print("  quit          exit\n")

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
