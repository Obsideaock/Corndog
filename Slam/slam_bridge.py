"""
slam_bridge.py — Bridge between Corndog's MoveLib and the SLAM core.

Runs ON THE ROBOT. It imports MoveLib (which already owns the I2C bus, the
BNO08x, and the gait engine) and exposes the current motion state. It only
READS hardware that MoveLib already initialised — it never opens the IMU or
servos a second time, so it is safe to run inside the same process as your
control listener.

Heading note: SLAM only uses *relative* yaw (it captures a one-time bias at
start), so the IMU's absolute heading offset and mount yaw don't matter — only
the sign/sense must match the LiDAR. If during bring-up the map rotates the
wrong way when you turn the robot, flip YAW_SIGN below.
"""

from __future__ import annotations
import math
import MoveLib as mlib

# --- bring-up calibration knobs ------------------------------------------
YAW_SIGN = +1.0      # flip to -1.0 if heading turns the wrong way vs the map
VEL_SCALE = 1.0      # commanded-vel -> realised-vel; tune after a tape-measure walk


def _quat_to_yaw(q):
    """BNO08x game quaternion is (i, j, k, real) = (x, y, z, w)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_yaw():
    """Relative heading (rad, CCW+). Falls back to the rotation vector."""
    try:
        q = mlib.bno.game_quaternion
    except Exception:
        q = mlib.bno.quaternion
    if q is None:
        return None
    return YAW_SIGN * _quat_to_yaw(q)


def read_tilt():
    """(pitch, roll) in rad from the mount-corrected gravity vector."""
    try:
        gx, gy, gz = mlib.get_gravity()        # already rotated into robot frame
    except Exception:
        return 0.0, 0.0
    pitch = math.atan2(gx, math.sqrt(gy * gy + gz * gz) + 1e-9)
    roll = math.atan2(gy, math.sqrt(gx * gx + gz * gz) + 1e-9)
    return pitch, roll


def read_velocity():
    """Commanded body-frame velocity (vx fwd, vy left, wz yaw) from the gait engine."""
    try:
        eng = mlib._ensure_engine()
        if not eng.is_active:
            return 0.0, 0.0, 0.0
        return float(eng.vx), float(eng.vy), float(eng.wz)
    except Exception:
        return 0.0, 0.0, 0.0


def read_motion(dt):
    """Assemble a MotionHint-compatible dict for the SLAM core."""
    yaw = read_yaw()
    pitch, roll = read_tilt()
    vx, vy, wz = read_velocity()
    return dict(yaw=yaw, vx=vx, vy=vy, wz=wz, pitch=pitch, roll=roll, dt=dt)
