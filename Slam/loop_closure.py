"""
loop_closure.py — keyframe pose-graph loop closure for Corndog SLAM (v2.2).

The frontend (SlamCore) does fast scan-to-map matching, which drifts slowly over
long runs (you see it as walls that don't quite line up when you loop back to a
room). This module runs in the background and fixes that:

  1. Drop a *keyframe* (pose + body-frame scan) every ~0.5 m / 25 deg of travel.
  2. Consecutive keyframes are tied by an "odometry" edge (the frontend's own
     relative motion estimate).
  3. When a new keyframe lands near an OLD one (but far in time), verify the
     revisit with a scan-to-scan match against a clean local field built from
     the old keyframe — independent of the (possibly drifted) global map.
  4. A confirmed revisit adds a loop edge. Optimise the pose graph
     (pose_graph.py), then rebuild the map from the corrected keyframe poses and
     nudge the live pose/heading-bias to match.

Pure numpy, no scipy. The heavy work (verify + optimise + rebuild) only happens
when a loop is actually detected; the per-frame cost is just a distance check.
"""
from __future__ import annotations
import math
import threading
import time
import numpy as np

import pose_graph as pg
from occupancy import OccupancyGrid
from scan_match import build_likelihood_field, match


class LCConfig:
    def __init__(self):
        self.kf_dist = 0.5                 # metres of travel between keyframes
        self.kf_ang = math.radians(25)     # or this much rotation
        self.min_kf = 12                   # need this many before looking for loops
        self.seq_gap = 10                  # candidate must be >= this many kf back
        self.loop_radius = 1.6             # only consider old kf within this distance
        self.field_radius = 4.0            # local verification field half-size (m)
        self.sigma = 0.10
        self.match_min = 0.50              # min score/point to accept a revisit
        self.max_correction = 1.2          # reject matches that move kf more than this
        self.lin_window = 0.5
        self.lin_step = 0.05
        self.ang_window = math.radians(25)
        self.ang_step = math.radians(3)
        self.omega_odom = (80.0, 80.0, 300.0)
        self.omega_loop = (150.0, 150.0, 500.0)
        self.iters = 15
        self.rate_hz = 3.0
        self.apply_cooldown_s = 4.0        # min wall-time between optimise+rebuild passes


class LoopCloser:
    def __init__(self, core, cfg: LCConfig | None = None, start_thread=False):
        self.core = core
        self.cfg = cfg or LCConfig()
        self.kf = []                       # list of {"pose": (3,), "scan": (ang, rng)}
        self.edges = []                    # list[pose_graph.Edge]
        self._last_kf_pose = None
        self._last_apply_t = 0.0
        self._pending = False
        self.closures = 0
        self.last_info = {}
        self.lock = threading.Lock()
        if start_thread:
            threading.Thread(target=self._loop, daemon=True).start()

    # ---- background thread ----------------------------------------------
    def _loop(self):
        dt = 1.0 / self.cfg.rate_hz
        while True:
            time.sleep(dt)
            try:
                self.maybe_add_keyframe()
            except Exception:
                pass

    def status(self):
        with self.lock:
            return {"keyframes": len(self.kf), "loops": self.closures,
                    "edges": len(self.edges), **self.last_info}

    def reset(self):
        """Drop all keyframes/edges — call when the map is wiped."""
        with self.lock:
            self.kf = []
            self.edges = []
            self._last_kf_pose = None
            self._pending = False
            self.last_info = {}

    # ---- keyframe capture -----------------------------------------------
    def _snapshot(self):
        with self.core.lock:
            pose = np.array(self.core.pose, float)
            ang, rng = self.core.last_scan
            ang = np.array(ang, float); rng = np.array(rng, float)
        return pose, (ang, rng)

    def maybe_add_keyframe(self):
        pose, scan = self._snapshot()
        if len(scan[1]) < 20:
            return False
        if self._last_kf_pose is not None:
            d = math.hypot(pose[0] - self._last_kf_pose[0], pose[1] - self._last_kf_pose[1])
            da = abs(pg._wrap(pose[2] - self._last_kf_pose[2]))
            if d < self.cfg.kf_dist and da < self.cfg.kf_ang:
                return False
        with self.lock:
            idx = len(self.kf)
            self.kf.append({"pose": pose, "scan": scan})
            if idx > 0:
                z = pg.relative(self.kf[idx - 1]["pose"], pose)
                self.edges.append(pg.Edge(idx - 1, idx, z, np.diag(self.cfg.omega_odom)))
            self._last_kf_pose = pose
        if idx >= self.cfg.min_kf and self._detect_loops(idx):
            self._pending = True
        # the expensive optimise + map rebuild is throttled; loop edges keep
        # accumulating between passes and are all incorporated on the next one
        if self._pending and (time.time() - self._last_apply_t) >= self.cfg.apply_cooldown_s:
            self._optimize_and_apply()
            self._last_apply_t = time.time()
            self._pending = False
        return True

    # ---- revisit detection ----------------------------------------------
    def _detect_loops(self, idx):
        cur = self.kf[idx]["pose"]
        # nearest older keyframe within radius and far enough back in time
        best_j, best_d = -1, self.cfg.loop_radius
        for j in range(0, idx - self.cfg.seq_gap):
            pj = self.kf[j]["pose"]
            d = math.hypot(cur[0] - pj[0], cur[1] - pj[1])
            if d <= best_d:
                best_j, best_d = j, d
        if best_j < 0:
            return False
        z, spp, ok = self._verify(best_j, idx)      # one scan-match, the best candidate
        if not ok:
            return False
        with self.lock:
            self.edges.append(pg.Edge(best_j, idx, z, np.diag(self.cfg.omega_loop), loop=True))
            self.last_info = {"last_loop": (best_j, idx), "last_score": round(spp, 3)}
        return True

    def _local_field(self, pose, scan):
        R = self.cfg.field_radius
        res = self.core.grid.res
        g = OccupancyGrid(size_m=2 * R, res=res, origin=(pose[0] - R, pose[1] - R))
        ang, rng = scan
        c, s = math.cos(pose[2]), math.sin(pose[2])
        xb, yb = rng * np.cos(ang), rng * np.sin(ang)
        xs = pose[0] + xb * c - yb * s
        ys = pose[1] + xb * s + yb * c
        g.integrate_scan(pose[0], pose[1], xs, ys)
        return build_likelihood_field(g, sigma_m=self.cfg.sigma), g

    def _verify(self, a, b):
        pa = self.kf[a]["pose"]
        field, g = self._local_field(pa, self.kf[a]["scan"])
        ang_b, rng_b = self.kf[b]["scan"]
        pb = self.kf[b]["pose"]
        pose, sc, spp = match(field, g, ang_b, rng_b,
                              (pb[0], pb[1], pb[2]),
                              lin_window=self.cfg.lin_window, lin_step=self.cfg.lin_step,
                              ang_window=self.cfg.ang_window, ang_step=self.cfg.ang_step,
                              refine=True, decimate=2)
        if spp < self.cfg.match_min:
            return None, spp, False
        if math.hypot(pose[0] - pb[0], pose[1] - pb[1]) > self.cfg.max_correction:
            return None, spp, False
        z = pg.relative(pa, np.array(pose, float))
        return z, spp, True

    # ---- optimise + apply -----------------------------------------------
    def _optimize_and_apply(self):
        with self.lock:
            nodes = np.array([k["pose"] for k in self.kf])
            edges = list(self.edges)
        xo, info = pg.optimize(nodes, edges, iterations=self.cfg.iters)
        L = len(self.kf) - 1
        old = nodes[L]
        new = xo[L]
        Tcorr = pg.v2t(new) @ np.linalg.inv(pg.v2t(old))
        newgrid = self._rebuild_map(xo)
        with self.lock:
            for i, k in enumerate(self.kf):
                k["pose"] = xo[i]
            self._last_kf_pose = xo[L]
        with self.core.lock:
            live = np.array(self.core.pose, float)
            livenew = pg.t2v(Tcorr @ pg.v2t(live))
            self.core.grid = newgrid
            self.core.pose = (float(livenew[0]), float(livenew[1]), float(livenew[2]))
            dth = math.atan2(Tcorr[1, 0], Tcorr[0, 0])
            if hasattr(self.core, "_yaw_bias"):
                self.core._yaw_bias = pg._wrap(self.core._yaw_bias + dth)
        self.closures += 1
        self.last_info = {**self.last_info, "chi2": round(info["chi2"], 2),
                          "applied": True}

    def _rebuild_map(self, poses):
        src = self.core.grid
        g = OccupancyGrid(size_m=src.n * src.res, res=src.res, origin=(src.ox, src.oy))
        for i, k in enumerate(self.kf):
            p = poses[i]
            ang, rng = k["scan"]
            c, s = math.cos(p[2]), math.sin(p[2])
            xb, yb = rng * np.cos(ang), rng * np.sin(ang)
            xs = p[0] + xb * c - yb * s
            ys = p[1] + xb * s + yb * c
            g.integrate_scan(p[0], p[1], xs, ys)
        return g
