# leg_square_test.py

import os
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(THIS_DIR)
sys.path.insert(0, PROJECT_ROOT)

import MoveLib as ml


# ============================================================
# MANUAL EDITS
# ============================================================

TEST_LEG = 0          # 0,1,2,3
TEST_PLANE = "z"      # "x", "y", or "z"
SQUARE_SIZE_M = 0.08  # 3 cm

# Manual calibration multipliers on top of MoveLib's internal scales
X_SCALE = 1.00
Y_SCALE = 1.00
Z_SCALE = 1.00

STEP_MULTIPLIER = 100
MOVE_SPEED = 50
MOVE_DELAY = 0.0
DWELL = 0.5

LOOP_FOREVER = True

# Center of the square, in MoveLib leg-offset coordinates
CENTER_X = 0.0
CENTER_Y = 0.0
CENTER_Z = 0.0

# Fixed pose for all legs when not being tested
REST_POSE = {
    0: (0.0, 0.0, 0.0),
    1: (0.0, 0.0, 0.0),
    2: (0.0, 0.0, 0.0),
    3: (0.0, 0.0, 0.0),
}


# ============================================================
# Helpers
# ============================================================

def axis_scaled(axis: str, size_m: float) -> float:
    if axis == "x":
        return size_m * X_SCALE
    if axis == "y":
        return size_m * Y_SCALE
    if axis == "z":
        return size_m * Z_SCALE
    raise ValueError(axis)


def build_square_points():
    hx = axis_scaled("x", SQUARE_SIZE_M) / 2.0
    hy = axis_scaled("y", SQUARE_SIZE_M) / 2.0
    hz = axis_scaled("z", SQUARE_SIZE_M) / 2.0

    if TEST_PLANE == "x":
        # hold X fixed, move in Y/Z
        return [
            (CENTER_X, CENTER_Y - hy, CENTER_Z - hz),
            (CENTER_X, CENTER_Y + hy, CENTER_Z - hz),
            (CENTER_X, CENTER_Y + hy, CENTER_Z + hz),
            (CENTER_X, CENTER_Y - hy, CENTER_Z + hz),
        ]

    if TEST_PLANE == "y":
        # hold Y fixed, move in X/Z
        return [
            (CENTER_X - hx, CENTER_Y, CENTER_Z - hz),
            (CENTER_X + hx, CENTER_Y, CENTER_Z - hz),
            (CENTER_X + hx, CENTER_Y, CENTER_Z + hz),
            (CENTER_X - hx, CENTER_Y, CENTER_Z + hz),
        ]

    if TEST_PLANE == "z":
        # hold Z fixed, move in X/Y
        return [
            (CENTER_X - hx, CENTER_Y - hy, CENTER_Z),
            (CENTER_X + hx, CENTER_Y - hy, CENTER_Z),
            (CENTER_X + hx, CENTER_Y + hy, CENTER_Z),
            (CENTER_X - hx, CENTER_Y + hy, CENTER_Z),
        ]

    raise ValueError("TEST_PLANE must be x, y, or z")


def move_to_pose(pose):
    ml.iklegs_move(
        pose,
        step_multiplier=STEP_MULTIPLIER,
        speed=MOVE_SPEED,
        delay=MOVE_DELAY,
    )


def build_pose_for_test_leg(target_xyz):
    pose = dict(REST_POSE)
    pose[TEST_LEG] = target_xyz
    return pose


def run_square():
    corners = build_square_points()

    print(f"TEST_LEG={TEST_LEG}")
    print(f"TEST_PLANE={TEST_PLANE}")
    print(f"SQUARE_SIZE_M={SQUARE_SIZE_M}")
    print(f"X_SCALE={X_SCALE} Y_SCALE={Y_SCALE} Z_SCALE={Z_SCALE}")
    print("Corners:")
    for i, c in enumerate(corners, 1):
        print(f"  {i}: {c}")

    # go to rest pose first
    move_to_pose(dict(REST_POSE))
    time.sleep(0.5)

    # move to first corner
    move_to_pose(build_pose_for_test_leg(corners[0]))
    time.sleep(DWELL)

    while True:
        for corner in corners[1:]:
            move_to_pose(build_pose_for_test_leg(corner))
            time.sleep(DWELL)

        move_to_pose(build_pose_for_test_leg(corners[0]))
        time.sleep(DWELL)

        if not LOOP_FOREVER:
            break


if __name__ == "__main__":
    try:
        ml.enable_servos()
        ml.initialize_servo_angles()
        ml.stand_up()
        ml.stop_gait(schedule_inactivity_reset=False)
        time.sleep(0.5)

        run_square()

    except KeyboardInterrupt:
        print("Stopped.")

    finally:
        try:
            move_to_pose(dict(REST_POSE))
            time.sleep(0.5)
        except Exception as e:
            print(f"Failed to return to rest pose: {e}")
