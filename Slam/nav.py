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
DEFAULT_CLEARANCE_M = 0.15        # extra breathing room (user-configurable, v2.1: 15 cm)
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


def blocked_mask(grid, clearance_m=DEFAULT_CLEARANCE_M, occ_thresh=OCC_THRESH, zones=None):
    """Occupied cells (plus any keep-out zones) inflated by robot radius + clearance."""
    occ = grid.prob() >= occ_thresh
    if zones is not None:
        occ = occ | zones.mask(grid)
    r = max(1, int(round((ROBOT_RADIUS_M + clearance_m) / grid.res)))
    return _dilate(occ, r)


def reachable_free(grid, blocked, start_cell, max_iter=500):
    """Boolean mask of KNOWN-FREE cells connected to start_cell through free,
    non-blocked space only (never crossing unknown or walls). Iterative 4-neighbour
    flood fill, pure numpy. Used so roam only targets places it can actually get to."""
    free = grid.prob() < FREE_THRESH
    passable = free & (~blocked)
    reach = np.zeros_like(passable)
    sc, sr = start_cell
    n = grid.n
    if not (0 <= sc < n and 0 <= sr < n):
        return reach
    reach[sr, sc] = True
    prev = 0
    for _ in range(max_iter):
        g = reach.copy()
        g[1:, :] |= reach[:-1, :]
        g[:-1, :] |= reach[1:, :]
        g[:, 1:] |= reach[:, :-1]
        g[:, :-1] |= reach[:, 1:]
        g &= passable
        g |= reach
        s = int(g.sum())
        reach = g
        if s == prev:
            break
        prev = s
    return reach


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


def plan(grid, start_xy, goal_xy, clearance_m=DEFAULT_CLEARANCE_M, zones=None):
    """
    A* from start to goal. Returns PlanResult. Goal is REJECTED if it is a wall,
    too close to one (inside the clearance), in unknown space, in a keep-out
    zone, or unreachable.
    """
    n = grid.n
    prob = grid.prob()
    blocked = blocked_mask(grid, clearance_m, zones=zones)

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
        # v2.1: stuck detection (invisible obstacle the LiDAR can't see)
        self.stuck_secs = 1.6          # commanded-forward but no progress this long => stuck
        self.stuck_progress_m = 0.06   # "progress" threshold over that window
        self.auto_obstacle_r = 0.10    # radius of the keep-out blob we stamp
        self.recover_secs = 0.8        # back up this long after getting stuck
        self.recover_vx = 0.08         # reverse speed during recovery (m/s)
        # v2.1: roam
        self.roam_min_hop_m = 0.7      # don't pick a roam goal closer than this
        self.roam_max_hop_m = 3.0      # ...or farther than this (stay near, don't bolt to a far room)
        self.roam_samples = 48         # candidate cells sampled per roam pick
        # adaptive re-planning: recompute the path on the current map so a stale
        # path (new wall, closed door, drift) gets corrected instead of followed
        self.replan_period_s = 1.5
        self.replan_fail_limit = 2     # consecutive failed replans before declaring blocked


def _wrap(a):
    return (a + _math.pi) % (2 * _math.pi) - _math.pi


class NavController:
    def __init__(self, core, drive, cfg: NavConfig | None = None,
                 keepout=None, start_thread=True):
        self.core = core
        self.drive = drive                  # drive(vx, vy, wz)
        self.cfg = cfg or NavConfig()
        self.keepout = keepout              # KeepoutStore or None
        self.state = "idle"                 # idle|driving|paused|arrived|blocked
        self.goal = None
        self._active_goal = None            # goal we re-plan toward
        self.path = []
        self.reason = ""
        self._block_ct = 0
        self._clear_ct = 0
        # route / roam
        self.route = []
        self.route_i = 0
        self.loop = False
        self.roam = False
        self._visited = []
        # stuck tracking
        self._anchor = None
        self._anchor_t = 0.0
        self._recover_until = 0.0
        self._last_replan_t = 0.0
        self._replan_fails = 0
        self._lock = _threading.Lock()
        if start_thread:
            self._thread = _threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    # ---- planning core (used by goal / route / roam / stuck replan) -------
    def _plan_to(self, x, y):
        res = plan(self.core.grid, (self.core.pose[0], self.core.pose[1]),
                   (x, y), clearance_m=self.cfg.clearance_m, zones=self.keepout)
        with self._lock:
            if not res.ok:
                self.reason = res.reason
                return False, res.reason
            self.goal = (x, y)
            self._active_goal = (x, y)
            self.path = _densify(list(res.path))
            self.state = "driving"
            self.reason = ""
            self._block_ct = self._clear_ct = 0
            self._reset_progress()
        return True, ""

    def _replan(self):
        """Recompute the path to the active goal on the current map, preserving
        nav state and stuck-progress (unlike _plan_to). Adopts the new path if
        valid; after repeated failures, stops and reports blocked."""
        if not self._active_goal:
            return
        res = plan(self.core.grid, (self.core.pose[0], self.core.pose[1]),
                   self._active_goal, clearance_m=self.cfg.clearance_m, zones=self.keepout)
        with self._lock:
            if res.ok:
                self.path = _densify(list(res.path))
                self._replan_fails = 0
                if self.state in ("paused", "blocked"):
                    self.state = "driving"; self.reason = ""
                    self._block_ct = self._clear_ct = 0
            else:
                self._replan_fails += 1
                self.reason = res.reason
                if self._replan_fails >= self.cfg.replan_fail_limit:
                    self.path = []
                    self.state = "blocked"

    # ---- public API ------------------------------------------------------
    def set_goal(self, x, y):
        # an explicit click overrides roam/route
        with self._lock:
            self.route = []; self.roam = False
        return self._plan_to(x, y)

    def set_route(self, points, loop=True):
        pts = [(float(a), float(b)) for a, b in points]
        with self._lock:
            self.route = pts; self.loop = bool(loop); self.route_i = 0; self.roam = False
        if not pts:
            return False, "empty route"
        return self._plan_to(*pts[0])

    def set_roam(self, on):
        with self._lock:
            self.roam = bool(on)
            if on:
                self.route = []
            else:
                self.goal = None; self.path = []; self.state = "idle"
        if not on:
            self.drive(0.0, 0.0, 0.0)
        return True, ""

    def cancel(self):
        with self._lock:
            self.goal = self._active_goal = None
            self.path = []; self.route = []; self.roam = False
            self.state = "idle"
        self.drive(0.0, 0.0, 0.0)

    def snapshot(self):
        with self._lock:
            return dict(state=self.state, goal=self.goal, path=list(self.path),
                        reason=self.reason, roam=self.roam,
                        route=list(self.route), route_i=self.route_i, loop=self.loop)

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

    # ---- stuck detection (LiDAR-invisible obstacle) ----------------------
    def _reset_progress(self):
        self._anchor = (self.core.pose[0], self.core.pose[1])
        self._anchor_t = _time.time()

    def _check_stuck(self, x, y, th):
        """Called only while commanding forward motion. True if we handled a stuck."""
        if self._anchor is None:
            self._reset_progress(); return False
        if _math.hypot(x - self._anchor[0], y - self._anchor[1]) > self.cfg.stuck_progress_m:
            self._reset_progress()
            return False
        if _time.time() - self._anchor_t < self.cfg.stuck_secs:
            return False
        # stuck: stamp a keep-out blob just ahead, then back up and re-plan
        self.drive(0.0, 0.0, 0.0)
        ahead = 0.35
        ox, oy = x + ahead * _math.cos(th), y + ahead * _math.sin(th)
        if self.keepout is not None:
            self.keepout.add_circle(ox, oy, self.cfg.auto_obstacle_r, auto=True)
        self._reset_progress()
        with self._lock:
            self.state = "recovering"
            self._recover_until = _time.time() + self.cfg.recover_secs
        return True

    # ---- roam goal picker ------------------------------------------------
    def _pick_roam_goal(self):
        g = self.core.grid
        blocked = blocked_mask(g, self.cfg.clearance_m, zones=self.keepout)
        sc, sr = _w2c(g, self.core.pose[0], self.core.pose[1])
        ok_cells = reachable_free(g, blocked, (sc, sr))   # only what's truly connected
        js, is_ = np.where(ok_cells)
        if len(js) == 0:
            return None
        x, y = self.core.pose[0], self.core.pose[1]
        k = min(self.cfg.roam_samples, len(js))
        sel = np.random.choice(len(js), size=k, replace=False)
        best, best_score = None, -1.0
        for s in sel:
            wx = g.ox + (is_[s] + 0.5) * g.res
            wy = g.oy + (js[s] + 0.5) * g.res
            dxy = _math.hypot(wx - x, wy - y)
            if dxy < self.cfg.roam_min_hop_m or dxy > self.cfg.roam_max_hop_m:
                continue
            if self._visited:
                nd = min(_math.hypot(wx - vx, wy - vy) for vx, vy in self._visited[-25:])
            else:
                nd = _math.hypot(wx - x, wy - y)   # novelty = far from recent visits
            if nd > best_score:
                best_score, best = nd, (wx, wy)
        return best

    # ---- main control loop ----------------------------------------------
    def _loop(self):
        dt = 1.0 / self.cfg.rate_hz
        while True:
            _time.sleep(dt)
            self.tick()

    def tick(self):
        with self._lock:
            state, path, goal = self.state, list(self.path), self.goal
            roam, route, loop, ri = self.roam, list(self.route), self.loop, self.route_i

        # roaming: when not actively driving, pick the next random reachable spot
        if roam and state in ("idle", "arrived", "blocked"):
            g = self._pick_roam_goal()
            if g:
                self._plan_to(*g)
            return

        # recovery: back up briefly after a stuck event, then re-plan around the
        # freshly-stamped obstacle
        if state == "recovering":
            if _time.time() < self._recover_until:
                self.drive(-self.cfg.recover_vx, 0.0, 0.0)
                return
            self.drive(0.0, 0.0, 0.0)
            if self._active_goal:
                ok, _ = self._plan_to(*self._active_goal)
                if not ok:
                    with self._lock:
                        self.state = "blocked"; self.reason = "stuck — no way around"
            else:
                with self._lock:
                    self.state = "idle"
            return

        # adaptive re-planning: periodically recompute the path to the active goal
        # on the CURRENT map, so a stale path (new wall, closed door, drift) gets
        # corrected. Also lets a blocked/paused robot find a way around once the
        # obstacle has been mapped (which, with the motion gate, happens once stopped).
        if self._active_goal and state in ("driving", "paused", "blocked"):
            if _time.time() - self._last_replan_t >= self.cfg.replan_period_s:
                self._last_replan_t = _time.time()
                self._replan()
                with self._lock:
                    state, path, goal = self.state, list(self.path), self.goal

        if state not in ("driving", "paused", "blocked") or not path:
            return

        x, y, th = self.core.pose

        # arrival -> advance route / roam / finish
        if goal and _math.hypot(goal[0] - x, goal[1] - y) < self.cfg.arrive_m:
            self.drive(0.0, 0.0, 0.0)
            self._visited.append((x, y))
            if route:
                if loop:
                    ni = (ri + 1) % len(route)
                else:
                    ni = ri + 1
                if (not loop) and ni >= len(route):
                    with self._lock:
                        self.state = "arrived"; self.path = []; self.route = []
                    return
                with self._lock:
                    self.route_i = ni
                self._plan_to(*route[ni])
                return
            if roam:
                with self._lock:
                    self.state = "arrived"; self.path = []
                return
            with self._lock:
                self.state = "arrived"; self.path = []
            return

        # obstacle hard-stop (debounced)
        if self._blocked_ahead():
            self._block_ct += 1; self._clear_ct = 0
            if self._block_ct >= self.cfg.block_frames:
                self.drive(0.0, 0.0, 0.0)
                self._reset_progress()
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
            self._reset_progress()                # turning in place; no translation expected
            self.drive(0.0, 0.0, wz)
        else:
            if self._check_stuck(x, y, th):        # invisible-obstacle handling
                return
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
