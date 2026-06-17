"""
nav.py — Path planning for Corndog (v2).

A* over the occupancy grid with a CONFIGURABLE clearance: occupied cells are
inflated by (robot footprint + clearance) so planned paths keep their distance
from walls and obstacles. Planning runs once per goal (and on occasional
re-plan), so it costs ~nothing in the steady-state SLAM loop.

Cell convention matches occupancy.py: grid.log[row=y, col=x].
"""

from __future__ import annotations
import math
import heapq
import numpy as np

# Corndog body is ~0.19 x 0.08 m; treat as a ~0.10 m radius footprint.
ROBOT_RADIUS_M = 0.10
DEFAULT_CLEARANCE_M = 0.10        # extra breathing room (user-configurable)
OCC_THRESH = 0.65
FREE_THRESH = 0.35


def _dilate(mask, r):
    """Boolean inflation by r cells (8-connected; conservative/octagonal)."""
    m = mask
    for _ in range(r):
        d = m.copy()
        d[1:, :]  |= m[:-1, :]; d[:-1, :] |= m[1:, :]
        d[:, 1:]  |= m[:, :-1]; d[:, :-1] |= m[:, 1:]
        d[1:, 1:] |= m[:-1, :-1]; d[1:, :-1] |= m[:-1, 1:]
        d[:-1, 1:]|= m[1:, :-1];  d[:-1, :-1]|= m[1:, 1:]
        m = d
    return m


def blocked_mask(grid, clearance_m=DEFAULT_CLEARANCE_M, occ_thresh=OCC_THRESH):
    """Occupied cells inflated by robot radius + clearance."""
    occ = grid.prob() >= occ_thresh
    r = max(1, int(round((ROBOT_RADIUS_M + clearance_m) / grid.res)))
    return _dilate(occ, r)


def _w2c(grid, x, y):
    return int((x - grid.ox) / grid.res), int((y - grid.oy) / grid.res)  # col, row


def _c2w(grid, col, row):
    return grid.ox + (col + 0.5) * grid.res, grid.oy + (row + 0.5) * grid.res


def _line_free(blocked, c0, r0, c1, r1):
    """True if the straight cell-line from (c0,r0) to (c1,r1) hits no blocked cell."""
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    steps = max(dc, dr, 1)
    for i in range(steps + 1):
        t = i / steps
        c = int(round(c0 + (c1 - c0) * t))
        r = int(round(r0 + (r1 - r0) * t))
        if blocked[r, c]:
            return False
    return True


def _smooth(blocked, cells):
    """String-pull: drop intermediate cells when line-of-sight allows."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1:
            (ci, ri), (cj, rj) = cells[i], cells[j]
            if _line_free(blocked, ci, ri, cj, rj):
                break
            j -= 1
        out.append(cells[j])
        i = j
    return out


class PlanResult:
    def __init__(self, ok, path=None, reason=""):
        self.ok = ok
        self.path = path or []      # list of world (x, y) waypoints
        self.reason = reason


def plan(grid, start_xy, goal_xy, clearance_m=DEFAULT_CLEARANCE_M):
    """
    A* from start to goal. Returns PlanResult. Goal is REJECTED if it is a wall,
    too close to one (inside the clearance), in unknown space, or unreachable.
    """
    n = grid.n
    prob = grid.prob()
    blocked = blocked_mask(grid, clearance_m)

    sc, sr = _w2c(grid, *start_xy)
    gc, gr = _w2c(grid, *goal_xy)
    if not (0 <= gc < n and 0 <= gr < n):
        return PlanResult(False, reason="goal off map")
    if prob[gr, gc] >= OCC_THRESH:
        return PlanResult(False, reason="goal is on a wall/obstacle")
    if prob[gr, gc] >= FREE_THRESH:
        return PlanResult(False, reason="goal is in unmapped (unknown) space")
    if blocked[gr, gc]:
        return PlanResult(False, reason="goal is too close to a wall")

    # If the robot itself sits inside the inflated zone, let it escape.
    if 0 <= sc < n and 0 <= sr < n:
        blocked[sr, sc] = False

    SQ2 = math.sqrt(2.0)
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]

    def h(c, r):
        return math.hypot(c - gc, r - gr)

    openh = [(h(sc, sr), 0.0, sc, sr)]
    came = {}
    g = {(sc, sr): 0.0}
    seen = set()
    found = False
    while openh:
        _, gcost, c, r = heapq.heappop(openh)
        if (c, r) in seen:
            continue
        seen.add((c, r))
        if (c, r) == (gc, gr):
            found = True
            break
        for dc, dr, w in nbrs:
            nc, nr = c + dc, r + dr
            if not (0 <= nc < n and 0 <= nr < n) or blocked[nr, nc]:
                continue
            ng = gcost + w
            if ng < g.get((nc, nr), 1e18):
                g[(nc, nr)] = ng
                came[(nc, nr)] = (c, r)
                heapq.heappush(openh, (ng + h(nc, nr), ng, nc, nr))

    if not found:
        return PlanResult(False, reason="no route to that spot")

    # backtrack -> cells -> smooth -> world waypoints
    cells = [(gc, gr)]
    cur = (gc, gr)
    while cur in came:
        cur = came[cur]
        cells.append(cur)
    cells.reverse()
    cells = _smooth(blocked, cells)
    path = [_c2w(grid, c, r) for c, r in cells]
    path[0] = (start_xy[0], start_xy[1])
    path[-1] = (goal_xy[0], goal_xy[1])
    return PlanResult(True, path=path)


# ──────────────────────────────────────────────────────────────────────────
# NavController — drives Corndog along a planned path, hard-stops for obstacles.
# Hardware-agnostic: give it the SlamCore (for pose, map, latest scan) and a
# drive(vx, vy, wz) callback (gait_command on the robot). Runs its own thread.
# ──────────────────────────────────────────────────────────────────────────
import math as _math
import time as _time
import threading as _threading


class NavConfig:
    def __init__(self):
        self.clearance_m = DEFAULT_CLEARANCE_M
        self.cruise_vx = 0.12          # forward speed while driving (m/s)
        self.turn_gain = 1.6           # wz per rad of heading error
        self.wz_max = 0.7              # rad/s cap
        self.turn_in_place = _math.radians(35)   # above this error, rotate first
        self.lookahead_m = 0.45        # pure-pursuit lookahead
        self.arrive_m = 0.10           # "arrived" tolerance (your 10 cm)
        self.stop_dist_m = 0.40        # obstacle hard-stop distance ahead
        self.cone_halfdeg = 30.0       # forward cone half-angle for obstacle check
        self.block_frames = 2          # consecutive blocked reads before pausing
        self.clear_frames = 3          # consecutive clear reads before resuming
        self.replan_m = 0.6            # if pushed this far off-path, replan
        self.rate_hz = 10.0


def _wrap(a):
    return (a + _math.pi) % (2 * _math.pi) - _math.pi


class NavController:
    def __init__(self, core, drive, cfg: NavConfig | None = None, start_thread=True):
        self.core = core
        self.drive = drive                  # drive(vx, vy, wz)
        self.cfg = cfg or NavConfig()
        self.state = "idle"                 # idle|driving|paused|arrived|blocked
        self.goal = None
        self.path = []
        self.reason = ""
        self._block_ct = 0
        self._clear_ct = 0
        self._lock = _threading.Lock()
        if start_thread:
            self._thread = _threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    # ---- public API ------------------------------------------------------
    def set_goal(self, x, y):
        res = plan(self.core.grid, (self.core.pose[0], self.core.pose[1]),
                   (x, y), clearance_m=self.cfg.clearance_m)
        with self._lock:
            if not res.ok:
                self.reason = res.reason
                return False, res.reason
            self.goal = (x, y)
            self.path = _densify(list(res.path))
            self.state = "driving"
            self.reason = ""
            self._block_ct = self._clear_ct = 0
        return True, ""

    def cancel(self):
        with self._lock:
            self.goal = None
            self.path = []
            self.state = "idle"
        self.drive(0.0, 0.0, 0.0)

    def snapshot(self):
        with self._lock:
            return dict(state=self.state, goal=self.goal,
                        path=list(self.path), reason=self.reason)

    # ---- obstacle check (forward cone in body frame) ---------------------
    def _blocked_ahead(self):
        ang, rng = self.core.last_scan
        if len(rng) == 0:
            return False
        half = _math.radians(self.cfg.cone_halfdeg)
        a = np.asarray(ang); r = np.asarray(rng)
        fwd = (np.abs(_np_wrap(a)) < half)
        near = fwd & (r < self.cfg.stop_dist_m)
        return bool(near.sum() >= 2)        # a couple of points, not a lone fleck

    # ---- main control loop ----------------------------------------------
    def _loop(self):
        dt = 1.0 / self.cfg.rate_hz
        while True:
            _time.sleep(dt)
            self.tick()

    def tick(self):
        with self._lock:
            state, path, goal = self.state, list(self.path), self.goal
        if state not in ("driving", "paused", "blocked") or not path:
            return

        x, y, th = self.core.pose

        # arrival
        if goal and _math.hypot(goal[0] - x, goal[1] - y) < self.cfg.arrive_m:
            self.drive(0.0, 0.0, 0.0)
            with self._lock:
                self.state = "arrived"; self.path = []
            return

        # obstacle hard-stop (debounced)
        if self._blocked_ahead():
            self._block_ct += 1; self._clear_ct = 0
            if self._block_ct >= self.cfg.block_frames:
                self.drive(0.0, 0.0, 0.0)
                with self._lock:
                    self.state = "paused"
                return
        else:
            self._clear_ct += 1
            if self._clear_ct >= self.cfg.clear_frames:
                self._block_ct = 0
                if state == "paused":
                    with self._lock:
                        self.state = "driving"

        with self._lock:
            if self.state == "paused":
                return

        # pure-pursuit target: farthest path point within lookahead
        tx, ty = self._lookahead_target(x, y, path)
        desired = _math.atan2(ty - y, tx - x)
        err = _wrap(desired - th)
        wz = max(-self.cfg.wz_max, min(self.cfg.wz_max, self.cfg.turn_gain * err))
        if abs(err) > self.cfg.turn_in_place:
            self.drive(0.0, 0.0, wz)          # rotate toward path first
        else:
            self.drive(self.cfg.cruise_vx, 0.0, wz)

    def _lookahead_target(self, x, y, path):
        Ld = self.cfg.lookahead_m
        # find closest point on the (dense) path, then look ahead FROM there so
        # we never re-target points already passed (the start, earlier waypoints)
        i0 = min(range(len(path)),
                 key=lambda i: _math.hypot(path[i][0] - x, path[i][1] - y))
        for px, py in path[i0:]:
            if _math.hypot(px - x, py - y) >= Ld:
                return px, py
        return path[-1]


def _np_wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _densify(path, step=0.12):
    """Resample a waypoint list to ~`step`-spaced points for smooth following."""
    if len(path) < 2:
        return list(path)
    out = [path[0]]
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        d = _math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / step))
        for k in range(1, n + 1):
            t = k / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return out
