"""
core.py — Hardware-agnostic SLAM core.

It knows nothing about RPLidar, the BNO08x, or the gait engine. You feed it:
  * a scan: beam angles (rad, 0=forward) + ranges (m), already quality-filtered
  * a MotionHint: absolute IMU yaw, body-frame commanded velocity, tilt, dt

and it returns the current pose estimate and maintains the map + trajectory.
This separation is what let us validate the whole algorithm in simulation
before putting it on the robot.
"""

from __future__ import annotations
import time
import math
import threading
from dataclasses import dataclass, field as dc_field
from collections import deque

import numpy as np

from occupancy import OccupancyGrid
import scan_match


@dataclass
class MotionHint:
    yaw: float | None = None        # absolute heading from IMU (rad), or None
    vx: float = 0.0                 # body-frame forward vel (m/s), commanded
    vy: float = 0.0                 # body-frame left vel (m/s), commanded
    wz: float = 0.0                 # yaw rate (rad/s), commanded
    pitch: float = 0.0              # rad, for tilt gating
    roll: float = 0.0               # rad
    dt: float = 0.1                 # seconds since last hint


@dataclass
class SlamConfig:
    size_m: float = 20.0
    res: float = 0.05
    tilt_gate_deg: float = 9.0      # skip MAP UPDATE above this body tilt
    min_match_pts: int = 40         # below this, coast on the prior
    trail_seconds: float = 30.0
    update_field_every: int = 2     # rebuild likelihood field every N frames
    field_radius_m: float = 6.5     # only rebuild field within this radius of robot
    vel_scale: float = 1.0          # commanded-vel -> real-vel calibration


class SlamCore:
    def __init__(self, cfg: SlamConfig | None = None):
        self.cfg = cfg or SlamConfig()
        self.grid = OccupancyGrid(self.cfg.size_m, self.cfg.res)
        self.pose = (0.0, 0.0, 0.0)            # x, y, theta (world)
        self._yaw_bias = None                  # aligns IMU yaw to map frame
        self._field = None
        self._frame = 0
        self.trail = deque()                   # (t, x, y)
        self.last_scan_world = (np.array([]), np.array([]))
        self.last_scan = (np.array([]), np.array([]))   # body frame
        self.lock = threading.Lock()
        self.stats = {"matched": 0, "coasted": 0, "gated": 0, "score": 0.0}

    # ---- prior from motion hint -----------------------------------------
    def _predict(self, hint: MotionHint):
        x, y, th = self.pose

        # Heading: trust the IMU's fused yaw (drift-resistant). Align it to the
        # map frame via a one-time bias captured on the first good hint.
        if hint.yaw is not None:
            if self._yaw_bias is None:
                self._yaw_bias = hint.yaw - th
            th_pred = hint.yaw - self._yaw_bias
        else:
            th_pred = th + hint.wz * hint.dt

        # Translation: integrate commanded body velocity into the world frame
        # using the predicted heading. This is only a seed; the matcher fixes
        # the (large) error from foot slip.
        s = self.cfg.vel_scale
        vx, vy = hint.vx * s, hint.vy * s
        dx = (vx * math.cos(th_pred) - vy * math.sin(th_pred)) * hint.dt
        dy = (vx * math.sin(th_pred) + vy * math.cos(th_pred)) * hint.dt
        return (x + dx, y + dy, th_pred)

    # ---- main entry per scan --------------------------------------------
    def update(self, angles, ranges, hint: MotionHint):
        angles = np.asarray(angles, dtype=np.float32)
        ranges = np.asarray(ranges, dtype=np.float32)
        with self.lock:
            self.last_scan = (angles, ranges)   # body frame, for nav obstacle stop
        prior = self._predict(hint)

        tilt = math.degrees(max(abs(hint.pitch), abs(hint.roll)))
        gated = tilt > self.cfg.tilt_gate_deg

        if self._field is None:
            # cold start: first scan defines the map; just drop it in at prior
            self._set_pose_and_map(prior, angles, ranges, do_map=not gated)
            self._field = scan_match.build_likelihood_field(
                self.grid, center=self.pose[:2], radius_m=self.cfg.field_radius_m)
            return self.pose

        if len(ranges) < self.cfg.min_match_pts:
            self.pose = prior
            self.stats["coasted"] += 1
        else:
            pose, score, spp = scan_match.match(
                self._field, self.grid, angles, ranges, prior)
            self.pose = pose
            self.stats["matched"] += 1
            self.stats["score"] = round(spp, 3)

        # Map update (skipped when the body is tilting badly mid-stride).
        if gated:
            self.stats["gated"] += 1
            self._record_scan_world(angles, ranges, write_map=False)
        else:
            self._record_scan_world(angles, ranges, write_map=True)
            self._frame += 1
            if self._frame % self.cfg.update_field_every == 0:
                self._field = scan_match.build_likelihood_field(
                    self.grid, center=self.pose[:2], radius_m=self.cfg.field_radius_m)

        self._push_trail()
        return self.pose

    # ---- helpers ---------------------------------------------------------
    def _scan_to_world(self, angles, ranges):
        x, y, th = self.pose
        xr = ranges * np.cos(angles)
        yr = ranges * np.sin(angles)
        c, s = math.cos(th), math.sin(th)
        xw = x + xr * c - yr * s
        yw = y + xr * s + yr * c
        return xw, yw

    def _record_scan_world(self, angles, ranges, write_map):
        xw, yw = self._scan_to_world(angles, ranges)
        with self.lock:
            self.last_scan_world = (xw, yw)
        if write_map:
            self.grid.integrate_scan(self.pose[0], self.pose[1], xw, yw)

    def _set_pose_and_map(self, pose, angles, ranges, do_map):
        self.pose = pose
        self._record_scan_world(angles, ranges, write_map=do_map)

    def _push_trail(self):
        now = time.monotonic()
        with self.lock:
            self.trail.append((now, self.pose[0], self.pose[1]))
            cutoff = now - self.cfg.trail_seconds
            while self.trail and self.trail[0][0] < cutoff:
                self.trail.popleft()

    # ---- persistence -----------------------------------------------------
    def save_map(self, path):
        self.grid.save(path)

    def load_map(self, path):
        self.grid = OccupancyGrid.load(path)
        self._field = scan_match.build_likelihood_field(self.grid)
