"""
sim.py — A hardware-free simulator to validate the SLAM core.

It models the things that actually make a walking quadruped hard:
  * foot slip: the velocity we *command* (and feed to SLAM as a prior) differs
    from the velocity the body actually achieves -> dead reckoning drifts.
  * body bob: pitch/roll oscillate as it walks (reported to the tilt gate).
  * sensor noise + dropouts on the A1-class LiDAR.

If SLAM can reconstruct the floorplan and track the true path despite a badly
biased motion prior, the scan matcher is doing its job.
"""

from __future__ import annotations
import math
import numpy as np

# A small flat with two rooms joined by a doorway + an L corridor (metres).
WALLS = np.array([
    # outer shell (≈ 8m x 6m)
    [-4, -3, 4, -3], [4, -3, 4, 3], [4, 3, -4, 3], [-4, 3, -4, -3],
    # vertical divider with a doorway gap (gap between y=-0.6 and y=0.6)
    [0, -3, 0, -0.6], [0, 0.6, 0, 3],
    # some furniture-ish blocks (left room)
    [-3.4, 1.2, -2.2, 1.2], [-2.2, 1.2, -2.2, 2.2],
    # furniture (right room)
    [1.6, -2.4, 3.2, -2.4], [3.2, -2.4, 3.2, -1.2],
], dtype=np.float64)


def raycast(px, py, ang_world, max_range=6.0):
    """Cast one ray from (px,py) at world angle; return distance to nearest wall."""
    dx, dy = math.cos(ang_world), math.sin(ang_world)
    best = max_range
    for x1, y1, x2, y2 in WALLS:
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-9:
            continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / denom      # along ray
        u = ((x1 - px) * dy - (y1 - py) * dx) / denom      # along segment
        if t > 0 and 0 <= u <= 1 and t < best:
            best = t
    return best


def scan(pose, n_beams=360, max_range=6.0, noise=0.02, dropout=0.05, rng=None):
    """Produce (angles_body, ranges) for a 360° LiDAR at the given true pose."""
    rng = rng or np.random.default_rng()
    x, y, th = pose
    body_ang = np.linspace(0, 2 * math.pi, n_beams, endpoint=False)
    ranges = np.empty(n_beams)
    for i, a in enumerate(body_ang):
        r = raycast(x, y, th + a, max_range)
        ranges[i] = r
    ranges += rng.normal(0, noise, n_beams)
    keep = (ranges < max_range - 0.05) & (rng.random(n_beams) > dropout)
    return body_ang[keep].astype(np.float32), ranges[keep].astype(np.float32)


def trajectory(dt=0.18, rng=None):
    """
    Yield (true_pose, commanded_vx, commanded_vy, commanded_wz, pitch, roll).
    The commanded velocities are what SLAM receives as its prior; the TRUE pose
    advances with slip + bias so dead-reckoning alone would drift badly.
    """
    rng = rng or np.random.default_rng(0)
    # waypoint loop: tour both rooms through the doorway and come back (twice)
    wps = [(-2.5, -1.8), (-2.5, 1.8), (-0.0, 0.0), (2.6, 1.8),
           (2.6, -1.8), (0.0, 0.0), (-2.5, -1.8), (-2.5, 1.8),
           (2.6, 1.8), (2.6, -1.8), (-2.5, -1.8)]
    x, y, th = -2.5, -1.8, 0.0
    speed = 0.15                      # m/s, matches Corndog VX_SPEED
    t = 0.0
    yaw_drift = 0.0                   # slow accumulating IMU heading error
    for wx, wy in wps:
        while math.hypot(wx - x, wy - y) > 0.08:
            desired = math.atan2(wy - y, wx - x)
            # turn toward waypoint
            dth = math.atan2(math.sin(desired - th), math.cos(desired - th))
            wz_cmd = max(-1.0, min(1.0, dth * 2.0))
            vx_cmd = speed if abs(dth) < 0.5 else 0.04

            # TRUE motion: stronger slip (achieves ~75% of commanded fwd) so
            # dead reckoning under-shoots, plus small lateral disturbance.
            slip = 0.75 + rng.normal(0, 0.06)
            th += wz_cmd * dt + rng.normal(0, 0.015)
            x += vx_cmd * slip * math.cos(th) * dt + rng.normal(0, 0.006)
            y += vx_cmd * slip * math.sin(th) * dt + rng.normal(0, 0.006)

            # body bob: pitch/roll oscillation as legs cycle
            pitch = math.radians(6.0) * math.sin(t * 9.0)
            roll = math.radians(4.0) * math.sin(t * 9.0 + 1.0)

            # IMU yaw: true heading + slow drift (~0.03°/frame) + noise. This is
            # what makes dead-reckoning diverge and forces SLAM to earn its keep.
            yaw_drift += math.radians(0.012)
            imu_yaw = th + yaw_drift + rng.normal(0, math.radians(0.8))
            t += dt
            yield ((x, y, th), vx_cmd, 0.0, wz_cmd, pitch, roll, imu_yaw)
