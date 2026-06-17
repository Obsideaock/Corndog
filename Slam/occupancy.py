"""
occupancy.py — Log-odds 2D occupancy grid for Corndog SLAM.

World frame: metres, x forward-ish / y left, right-handed, theta CCW.
The grid stores log-odds. Positive => likely occupied, negative => likely free.

Dynamic objects (people walking through) are handled by the free-space carving:
when a beam passes through a cell that used to hold a person, that cell gets a
free update and decays back toward unknown. No blanket time-decay needed, though
a small optional decay is provided.
"""

from __future__ import annotations
import numpy as np

# Inverse-sensor-model log-odds increments (tunable).
L_OCC = 0.85     # added to a cell that a beam endpoint lands in
L_FREE = -0.40   # added to cells a beam passes through
L_MIN = -5.0     # clamp (prevents over-confidence so the map stays adaptable)
L_MAX = 5.0


class OccupancyGrid:
    def __init__(self, size_m=20.0, res=0.05, origin=None):
        """
        size_m : side length of the (square) map in metres.
        res    : metres per cell (5 cm default).
        origin : world coords (x, y) of the grid's (0,0) corner. Defaults to
                 centring the world origin in the middle of the map.
        """
        self.res = float(res)
        self.n = int(round(size_m / res))
        if origin is None:
            half = size_m / 2.0
            origin = (-half, -half)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.log = np.zeros((self.n, self.n), dtype=np.float32)  # [row=y, col=x]

    # ---- coordinate transforms -------------------------------------------
    def world_to_cell(self, x, y):
        cx = ((np.asarray(x) - self.ox) / self.res).astype(np.int32)
        cy = ((np.asarray(y) - self.oy) / self.res).astype(np.int32)
        return cx, cy

    def in_bounds(self, cx, cy):
        return (cx >= 0) & (cx < self.n) & (cy >= 0) & (cy < self.n)

    # ---- map update ------------------------------------------------------
    def integrate_scan(self, rx, ry, xs, ys):
        """
        Carve free space along each beam and mark endpoints occupied.
        rx, ry : robot position in world metres.
        xs, ys : arrays of beam endpoint positions in world metres.
        """
        rcx, rcy = self.world_to_cell(rx, ry)
        ex, ey = self.world_to_cell(xs, ys)

        # Free-space carving: gather all "free" cells along every ray, then
        # apply endpoints as occupied last so endpoints win.
        free_cols, free_rows = _bresenham_rays(rcx, rcy, ex, ey)
        if free_cols.size:
            m = self.in_bounds(free_cols, free_rows)
            fc, fr = free_cols[m], free_rows[m]
            np.add.at(self.log, (fr, fc), L_FREE)

        m = self.in_bounds(ex, ey)
        ex, ey = ex[m], ey[m]
        np.add.at(self.log, (ey, ex), L_OCC - L_FREE)  # offset the free we may have added
        np.clip(self.log, L_MIN, L_MAX, out=self.log)

    def decay(self, factor=0.98):
        """Optional gentle pull toward unknown (helps very dynamic scenes)."""
        self.log *= factor

    # ---- derived views ---------------------------------------------------
    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log))

    def occupied_mask(self, thresh=0.65):
        return self.prob() >= thresh

    # ---- persistence -----------------------------------------------------
    def save(self, path):
        np.savez_compressed(path, log=self.log, res=self.res,
                            ox=self.ox, oy=self.oy, n=self.n)

    @classmethod
    def load(cls, path):
        d = np.load(path)
        g = cls.__new__(cls)
        g.log = d["log"].astype(np.float32)
        g.res = float(d["res"]); g.ox = float(d["ox"]); g.oy = float(d["oy"])
        g.n = int(d["n"])
        return g


def _bresenham_rays(x0, y0, x1, y1):
    """
    Vectorised integer line rasterisation from a single origin (x0,y0) to many
    endpoints (x1,y1). Returns concatenated (cols, rows) of the cells the rays
    pass through, EXCLUDING the endpoint cell. Uses a uniform-step DDA which is
    simple to vectorise and good enough for free-space carving.
    """
    x1 = np.asarray(x1, dtype=np.int32)
    y1 = np.asarray(y1, dtype=np.int32)
    if x1.size == 0:
        return np.empty(0, np.int32), np.empty(0, np.int32)

    dx = x1 - x0
    dy = y1 - y0
    steps = np.maximum(np.abs(dx), np.abs(dy)).astype(np.int32)
    steps = np.maximum(steps, 1)
    maxs = int(steps.max())

    # Parametric t in [0,1); skip the final endpoint cell (t<1 strictly).
    t = np.arange(maxs, dtype=np.float32)[:, None] / steps[None, :]  # (maxs, N)
    valid = t < 1.0
    xs = np.round(x0 + t * dx[None, :]).astype(np.int32)
    ys = np.round(y0 + t * dy[None, :]).astype(np.int32)
    return xs[valid], ys[valid]
