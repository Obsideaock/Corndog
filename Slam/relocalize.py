"""
relocalize.py — One-time global "where am I" for a loaded map (v2).

When you load a saved map and Corndog didn't start exactly where he left off,
this brute-forces his pose: it scores his first scan against the map at a coarse
grid of candidate positions × rotations, then refines the best one. Costs a few
seconds (acceptable as a startup step), then normal tracking takes over.

Returns (pose, confidence). Confidence is score-per-point in [0,1]; if it's low
the space was probably too ambiguous (or he's somewhere unmapped) and the caller
should fall back to the last-saved pose.
"""

from __future__ import annotations
import math
import numpy as np

import scan_match

FREE_THRESH = 0.35


def relocalize(grid, angles, ranges, pos_step_m=0.30, ang_step_deg=12,
               decimate=4, field=None):
    angles = np.asarray(angles, dtype=np.float32)
    ranges = np.asarray(ranges, dtype=np.float32)
    if len(ranges) < 20:
        return None, 0.0

    if field is None:
        field = scan_match.build_likelihood_field(grid)

    # candidate positions = free cells, subsampled
    free = grid.prob() < FREE_THRESH
    step = max(1, int(pos_step_m / grid.res))
    sub = free[::step, ::step]
    rr, cc = np.where(sub)
    if len(rr) == 0:
        return None, 0.0
    rows = rr * step
    cols = cc * step
    px = grid.ox + (cols + 0.5) * grid.res
    py = grid.oy + (rows + 0.5) * grid.res

    a = angles[::decimate]
    r = ranges[::decimate]
    xs = r * np.cos(a)
    ys = r * np.sin(a)
    npts = max(1, len(r))
    inv = 1.0 / grid.res
    n = grid.n

    best_pose, best_sc = None, -1.0
    for deg in range(0, 360, ang_step_deg):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        rx = xs * c - ys * s
        ry = xs * s + ys * c
        wx = px[:, None] + rx[None, :]
        wy = py[:, None] + ry[None, :]
        cx = ((wx - grid.ox) * inv).astype(np.int32)
        cy = ((wy - grid.oy) * inv).astype(np.int32)
        ok = (cx >= 0) & (cx < n) & (cy >= 0) & (cy < n)
        cxc = np.clip(cx, 0, n - 1)
        cyc = np.clip(cy, 0, n - 1)
        vals = np.where(ok, field[cyc, cxc], 0.0)
        scores = vals.sum(axis=1)
        k = int(np.argmax(scores))
        if scores[k] > best_sc:
            best_sc = float(scores[k])
            best_pose = (float(px[k]), float(py[k]), th)

    # refine the winner with a normal (wider) scan match
    pose, sc, spp = scan_match.match(
        field, grid, angles, ranges, best_pose,
        lin_window=pos_step_m, lin_step=0.05,
        ang_window=math.radians(ang_step_deg), ang_step=math.radians(3),
        decimate=2)
    return pose, spp
