"""
scan_match.py — Multi-resolution correlative scan matching against a likelihood
field derived from the occupancy grid (Hector-style scan-to-map; no odometry
required). v2 perf pass:

  * the likelihood field is rebuilt only in a WINDOW around the robot, not over
    the whole 20 m grid (the matcher never looks past LiDAR range anyway);
  * the inner (dx,dy) search is vectorised over dy, cutting Python-loop overhead;
  * optional point decimation for matching.

Net effect: markedly less CPU per scan than v1, leaving headroom for navigation
without slowing Corndog's gait. Pure numpy — no scipy.
"""

from __future__ import annotations
import math
import numpy as np


def _chamfer(field, radius, d_ortho, d_diag):
    """Radius-limited grayscale max-dilation (Gaussian-ish falloff)."""
    for _ in range(radius):
        f = field
        nb = f.copy()
        nb[1:, :]  = np.maximum(nb[1:, :],  f[:-1, :] * d_ortho)
        nb[:-1, :] = np.maximum(nb[:-1, :], f[1:, :]  * d_ortho)
        nb[:, 1:]  = np.maximum(nb[:, 1:],  f[:, :-1] * d_ortho)
        nb[:, :-1] = np.maximum(nb[:, :-1], f[:, 1:]  * d_ortho)
        nb[1:, 1:]   = np.maximum(nb[1:, 1:],   f[:-1, :-1] * d_diag)
        nb[1:, :-1]  = np.maximum(nb[1:, :-1],  f[:-1, 1:]  * d_diag)
        nb[:-1, 1:]  = np.maximum(nb[:-1, 1:],  f[1:, :-1]  * d_diag)
        nb[:-1, :-1] = np.maximum(nb[:-1, :-1], f[1:, 1:]   * d_diag)
        field = nb
    return field


def build_likelihood_field(grid, sigma_m=0.10, occ_thresh=0.65,
                           center=None, radius_m=None):
    """
    Likelihood field in [0,1], peaking on occupied cells. If center=(x,y) and
    radius_m are given, only that window is computed (rest stays 0) — much
    cheaper, and the matcher only samples near the robot anyway.
    """
    occ = grid.occupied_mask(occ_thresh)
    if not occ.any():
        return np.zeros_like(grid.log, dtype=np.float32)

    sigma_cells = max(1e-3, sigma_m / grid.res)
    radius = max(1, int(round(2.5 * sigma_cells)))
    d_ortho = math.exp(-1.0 / (2.0 * sigma_cells * sigma_cells))
    d_diag = math.exp(-2.0 / (2.0 * sigma_cells * sigma_cells))

    if center is not None and radius_m is not None:
        ci = int((center[0] - grid.ox) / grid.res)
        cj = int((center[1] - grid.oy) / grid.res)
        rad = int(radius_m / grid.res) + radius + 2
        r0, r1 = max(0, cj - rad), min(grid.n, cj + rad + 1)
        c0, c1 = max(0, ci - rad), min(grid.n, ci + rad + 1)
        field = np.zeros_like(grid.log, dtype=np.float32)
        if r1 > r0 and c1 > c0:
            sub = occ[r0:r1, c0:c1].astype(np.float32)
            field[r0:r1, c0:c1] = _chamfer(sub, radius, d_ortho, d_diag)
        return field

    return _chamfer(occ.astype(np.float32), radius, d_ortho, d_diag).astype(np.float32)


def match(field, grid, angles, ranges, prior_pose,
          lin_window=0.24, lin_step=0.04,
          ang_window=np.radians(10), ang_step=np.radians(2),
          refine=True, decimate=2):
    """
    Search (dx,dy,dtheta) around prior_pose to maximise the likelihood score.
    Coarse pass then a finer refine pass. Returns (best_pose, score, score/pt).
    """
    if decimate > 1 and len(ranges) > 120:
        angles = angles[::decimate]
        ranges = ranges[::decimate]
    xs_r = ranges * np.cos(angles)
    ys_r = ranges * np.sin(angles)
    npts = max(1, len(ranges))
    inv = 1.0 / grid.res
    n = grid.n

    def search(center, lw, ls, aw, as_):
        best_p, best_sc = center, -1.0
        dths = np.arange(-aw, aw + 1e-9, as_)
        offs = np.arange(-lw, lw + 1e-9, ls).astype(np.float32)
        for dth in dths:
            th = center[2] + dth
            c, s = math.cos(th), math.sin(th)
            xrot = xs_r * c - ys_r * s
            yrot = xs_r * s + ys_r * c
            # all dy candidates vectorised together
            yw_all = (center[1] + yrot)[None, :] + offs[:, None]      # (K, P)
            cy = ((yw_all - grid.oy) * inv).astype(np.int32)
            cy_ok = (cy >= 0) & (cy < n)
            cyc = np.clip(cy, 0, n - 1)
            for dx in offs:
                xw = center[0] + dx + xrot
                cx = ((xw - grid.ox) * inv).astype(np.int32)          # (P,)
                cx_ok = (cx >= 0) & (cx < n)
                cxc = np.clip(cx, 0, n - 1)
                vals = field[cyc, cxc[None, :]]                       # (K, P)
                vals = np.where(cy_ok & cx_ok[None, :], vals, 0.0)
                scores = vals.sum(axis=1)                             # (K,)
                k = int(np.argmax(scores))
                if scores[k] > best_sc:
                    best_sc = float(scores[k])
                    best_p = (center[0] + dx, center[1] + float(offs[k]), th)
        return best_p, best_sc

    pose, sc = search(prior_pose, lin_window, lin_step, ang_window, ang_step)
    if refine:
        pose, sc = search(pose, lin_step, lin_step / 4.0, ang_step, ang_step / 4.0)
    return pose, sc, sc / npts
