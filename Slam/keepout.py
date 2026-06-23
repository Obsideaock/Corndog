"""
keepout.py — persistent "don't walk here" layer (v2.1).

Two sources, one layer:
  * user-painted no-go rectangles (carpets, rugs, stairs, anything the LiDAR
    can't see but you know to avoid), and
  * auto-detected obstacles (small circles) stamped when the robot gets stuck
    trying to move through a spot the map thinks is clear.

Saved next to the map as <map>.nogo.json so it survives restarts. The planner
ORs this layer into its blocked mask; clearing is per-zone or wholesale.
"""
from __future__ import annotations
import os
import json
import threading
import numpy as np


class KeepoutStore:
    def __init__(self, map_path=None):
        self.path = (map_path + ".nogo.json") if map_path else None
        self._zones = []          # list of dicts with id/kind/coords/auto
        self._next = 1
        self._lock = threading.Lock()
        self.load()

    # ---- persistence ----
    def load(self):
        if self.path and os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    self._zones = json.load(f)
                self._next = max((z["id"] for z in self._zones), default=0) + 1
            except Exception:
                self._zones = []

    def save(self):
        if not self.path:
            return
        try:
            with open(self.path, "w") as f:
                json.dump(self._zones, f)
        except Exception:
            pass

    # ---- editing ----
    def add_rect(self, x0, y0, x1, y1):
        with self._lock:
            zid = self._next; self._next += 1
            self._zones.append({"id": zid, "kind": "rect", "auto": False,
                                "x0": min(x0, x1), "y0": min(y0, y1),
                                "x1": max(x0, x1), "y1": max(y0, y1)})
        self.save(); return zid

    def add_circle(self, x, y, r, auto=True):
        with self._lock:
            zid = self._next; self._next += 1
            self._zones.append({"id": zid, "kind": "circle", "auto": bool(auto),
                                "x": float(x), "y": float(y), "r": float(r)})
        self.save(); return zid

    def delete(self, zid):
        with self._lock:
            self._zones = [z for z in self._zones if z["id"] != zid]
        self.save()

    def clear(self, auto_only=False):
        with self._lock:
            if auto_only:
                self._zones = [z for z in self._zones if not z.get("auto")]
            else:
                self._zones = []
        self.save()

    def all(self):
        with self._lock:
            return [dict(z) for z in self._zones]

    def is_blocked_point(self, x, y):
        for z in self._zones:
            if z["kind"] == "rect":
                if z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"]:
                    return True
            else:
                if (x - z["x"]) ** 2 + (y - z["y"]) ** 2 <= z["r"] ** 2:
                    return True
        return False

    # ---- rasterise into a boolean grid mask ----
    def mask(self, grid):
        n = grid.n
        m = np.zeros((n, n), dtype=bool)
        with self._lock:
            zones = list(self._zones)
        for z in zones:
            if z["kind"] == "rect":
                ci0 = int((z["x0"] - grid.ox) / grid.res)
                ci1 = int((z["x1"] - grid.ox) / grid.res)
                cj0 = int((z["y0"] - grid.oy) / grid.res)
                cj1 = int((z["y1"] - grid.oy) / grid.res)
                ci0, ci1 = max(0, min(ci0, ci1)), min(n - 1, max(ci0, ci1))
                cj0, cj1 = max(0, min(cj0, cj1)), min(n - 1, max(cj0, cj1))
                m[cj0:cj1 + 1, ci0:ci1 + 1] = True
            else:
                rc = int(z["r"] / grid.res) + 1
                ci = int((z["x"] - grid.ox) / grid.res)
                cj = int((z["y"] - grid.oy) / grid.res)
                i0, i1 = max(0, ci - rc), min(n, ci + rc + 1)
                j0, j1 = max(0, cj - rc), min(n, cj + rc + 1)
                if i1 <= i0 or j1 <= j0:
                    continue
                jj, ii = np.ogrid[j0:j1, i0:i1]
                dx = (grid.ox + (ii + 0.5) * grid.res) - z["x"]
                dy = (grid.oy + (jj + 0.5) * grid.res) - z["y"]
                m[j0:j1, i0:i1] |= (dx * dx + dy * dy) <= z["r"] ** 2
        return m
