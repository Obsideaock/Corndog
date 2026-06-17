#!/usr/bin/env python3
"""
corndog_slam.py — Localization-only version.

The script has two phases:

  1. WARMUP — robot stays still, scans accumulate into the map until
     enough cells are occupied. Standard SLAM-style mapping.
  2. LOCALIZE — map is FROZEN. The matcher just figures out where the
     robot is inside that map. No more map updates, so dynamic obstacles
     (people, pets, moved chairs) can't corrupt the map, and the matcher
     score formula naturally ignores blocked rays.

If you want to remap, hit C to clear and rebuild from scratch.

Requires: numpy, scipy, matplotlib, pyrplidar
Install:  pip3 install numpy scipy matplotlib pyrplidar --break-system-packages
Run:      python3 corndog_slam.py

Controls (click map window to focus first):
  S — save map to corndog_map.npy
  R — reset position to centre
  C — clear map and restart fresh (re-enters warmup)
  Q — quit
"""

import threading
import time
import math
import numpy as np
from scipy.ndimage import gaussian_filter
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar

# ── Config ────────────────────────────────────────────────────────────────────

PORT         = '/dev/ttyUSB0'
MIN_QUALITY  = 5

MAP_SIZE_M   = 10
MAP_SIZE_PX  = 500
RESOLUTION   = MAP_SIZE_M / MAP_SIZE_PX   # 0.02 m/cell

SLAM_HZ      = 5

SEARCH_CELLS = 5               # ±5 cells = ±10cm in x/y
SEARCH_DEG   = 5.0
SEARCH_STEP  = 1.0

MIN_OCCUPIED_CELLS = 500
BLUR_SIGMA         = 2.0

# Minimum total match score (sum across rays) to accept a pose update.
# With ~300-point scans, real walls give ~50-150 total score depending
# on how much of the room is in view. 20 is a low floor — below this we
# assume the match is unreliable (LiDAR mostly blocked) and hold pose.
MIN_MATCH_SCORE = 20.0

# ── Angle correction ──────────────────────────────────────────────────────────
ANGLE_OFFSET_DEG = 90
MIRROR_SCAN      = False

# ── Shared state ──────────────────────────────────────────────────────────────

latest_scan = []
scan_lock   = threading.Lock()
running     = True

occ_grid = np.full((MAP_SIZE_PX, MAP_SIZE_PX), 0.5, dtype=np.float32)

robot_x_m  = MAP_SIZE_M / 2.0
robot_y_m  = MAP_SIZE_M / 2.0
robot_th_r = 0.0
pose_lock  = threading.Lock()
trail      = []

# Set by the C key handler to force a fresh warmup
restart_warmup = False

# ── LiDAR reader ─────────────────────────────────────────────────────────────

def lidar_reader():
    global running

    lidar = PyRPlidar()
    lidar.connect(port=PORT, baudrate=115200, timeout=3)

    lidar.stop()
    lidar.set_motor_pwm(0)
    time.sleep(1.0)
    lidar.set_motor_pwm(500)
    time.sleep(2.0)

    print("LiDAR running...")

    try:
        scan_generator = lidar.start_scan_express(0)
        current_scan   = []

        for measurement in scan_generator():
            if not running:
                break
            try:
                if measurement.start_flag and current_scan:
                    good = [(m.quality, m.angle, m.distance)
                            for m in current_scan
                            if m.quality >= MIN_QUALITY and 50 < m.distance < 8000]
                    with scan_lock:
                        latest_scan[:] = good
                    current_scan = []
                current_scan.append(measurement)
            except (IndexError, AttributeError):
                current_scan = []

    except Exception as e:
        print(f"LiDAR error: {e}")
    finally:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        print("LiDAR disconnected.")

# ── Scan → world points ───────────────────────────────────────────────────────

def scan_to_points(scan, x_m, y_m, th_r):
    """Convert a raw scan to world-frame (x, y) hit points in metres."""
    if not scan:
        return np.zeros((0, 2))
    offset_r = math.radians(ANGLE_OFFSET_DEG)
    pts = []
    for (_, angle_deg, dist_mm) in scan:
        dist_m = dist_mm / 1000.0
        a = angle_deg if not MIRROR_SCAN else -angle_deg
        ray_angle = th_r - math.radians(a) + offset_r
        pts.append((x_m + dist_m * math.cos(ray_angle),
                    y_m + dist_m * math.sin(ray_angle)))
    return np.array(pts)

# ── Map update (warmup only) ──────────────────────────────────────────────────

def bresenham(r0, c0, r1, c1):
    dr = abs(r1 - r0); dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    while True:
        yield r0, c0
        if r0 == r1 and c0 == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc; r0 += sr
        if e2 < dr:
            err += dr; c0 += sc

OCCUPIED_LOCK = 0.25

def update_map(pts_m, rx_m, ry_m):
    """Update occupancy grid. Only used during warmup."""
    global occ_grid
    rx = int(np.clip(rx_m / RESOLUTION, 0, MAP_SIZE_PX - 1))
    ry = int(np.clip(ry_m / RESOLUTION, 0, MAP_SIZE_PX - 1))

    for (wx, wy) in pts_m:
        gx = int(wx / RESOLUTION)
        gy = int(wy / RESOLUTION)
        if not (0 <= gx < MAP_SIZE_PX and 0 <= gy < MAP_SIZE_PX):
            continue
        for r, c in bresenham(ry, rx, gy, gx):
            if 0 <= r < MAP_SIZE_PX and 0 <= c < MAP_SIZE_PX:
                if occ_grid[r, c] > OCCUPIED_LOCK:
                    occ_grid[r, c] = min(occ_grid[r, c] + 0.04, 0.95)
        occ_grid[gy, gx] = max(occ_grid[gy, gx] - 0.20, 0.05)

def count_occupied():
    return int(np.sum(occ_grid < 0.35))

# ── Scan matching ─────────────────────────────────────────────────────────────

def match_scan(scan, x_m, y_m, th_r):
    """
    Find the pose that best aligns the scan to the (frozen) map.

    Scoring is now a SUM across rays, not a mean. This matters for
    dynamic obstacles: a ray that hits a person standing in the room
    lands in known-free space and contributes ~0 to the sum. Real walls
    contribute their full Gaussian-blurred score. So the matcher just
    ignores blocked rays instead of being dragged off by them.

    Returns unchanged pose if total score too low or jump too large.
    """
    if len(scan) < 30:
        return x_m, y_m, th_r

    score_map = gaussian_filter(1.0 - occ_grid, sigma=BLUR_SIGMA)

    offset_r = math.radians(ANGLE_OFFSET_DEG)
    raw = []
    for (_, angle_deg, dist_mm) in scan:
        a = angle_deg if not MIRROR_SCAN else -angle_deg
        local_angle = -math.radians(a) + offset_r
        raw.append((dist_mm / 1000.0, local_angle))

    best_score = -1.0
    best_x, best_y, best_th = x_m, y_m, th_r
    step_m = RESOLUTION

    for dth in np.arange(-SEARCH_DEG, SEARCH_DEG + 0.01, SEARCH_STEP):
        th_c = th_r + math.radians(dth)

        for dx in range(-SEARCH_CELLS, SEARCH_CELLS + 1):
            for dy in range(-SEARCH_CELLS, SEARCH_CELLS + 1):
                cx = x_m + dx * step_m
                cy = y_m + dy * step_m

                score = 0.0
                for (dist_m, local_a) in raw:
                    wx = cx + dist_m * math.cos(th_c + local_a)
                    wy = cy + dist_m * math.sin(th_c + local_a)
                    gx = int(wx / RESOLUTION)
                    gy = int(wy / RESOLUTION)
                    if 0 <= gx < MAP_SIZE_PX and 0 <= gy < MAP_SIZE_PX:
                        score += score_map[gy, gx]

                if score > best_score:
                    best_score = score
                    best_x, best_y, best_th = cx, cy, th_c

    if best_score < MIN_MATCH_SCORE:
        return x_m, y_m, th_r

    jump_m   = math.sqrt((best_x - x_m)**2 + (best_y - y_m)**2)
    jump_deg = abs(math.degrees(best_th - th_r))
    if jump_m > SEARCH_CELLS * step_m * 1.1 or jump_deg > SEARCH_DEG * 1.1:
        return x_m, y_m, th_r

    return best_x, best_y, best_th

# ── SLAM thread ───────────────────────────────────────────────────────────────

def slam_thread():
    global robot_x_m, robot_y_m, robot_th_r, running, restart_warmup

    interval    = 1.0 / SLAM_HZ
    scan_count  = 0
    matching_on = False

    print("[SLAM] Building initial map — hold Corndog still...")

    while running:
        t0 = time.time()

        # C-key clears the map; drop back into warmup
        if restart_warmup:
            matching_on    = False
            restart_warmup = False
            print("[SLAM] Restarting warmup — hold Corndog still...")

        with scan_lock:
            scan = list(latest_scan)

        if len(scan) < 30:
            time.sleep(0.05)
            continue

        with pose_lock:
            x, y, th = robot_x_m, robot_y_m, robot_th_r

        if not matching_on:
            # WARMUP: paint scans into the map at the (stationary) starting pose
            pts = scan_to_points(scan, x, y, th)
            if len(pts):
                update_map(pts, x, y)

            if count_occupied() >= MIN_OCCUPIED_CELLS:
                matching_on = True
                print(f"[SLAM] Map ready ({count_occupied()} occupied cells) "
                      f"— map FROZEN, localization on. Move Corndog now.")

        else:
            # LOCALIZE: match scan against the frozen map. No update_map.
            new_x, new_y, new_th = match_scan(scan, x, y, th)

            with pose_lock:
                robot_x_m  = new_x
                robot_y_m  = new_y
                robot_th_r = new_th
                trail.append((new_x, new_y))
                if len(trail) > 2000:
                    trail.pop(0)

        scan_count += 1
        if scan_count % 25 == 0:
            with pose_lock:
                print(f"[SLAM] scan={scan_count} | "
                      f"mode={'localize' if matching_on else 'warmup'} | "
                      f"occupied={count_occupied()} | "
                      f"pos=({robot_x_m:.2f}m, {robot_y_m:.2f}m) | "
                      f"heading={math.degrees(robot_th_r):.1f}°")

        elapsed = time.time() - t0
        if interval - elapsed > 0:
            time.sleep(interval - elapsed)

# ── Visualizer ────────────────────────────────────────────────────────────────

def main():
    global running, robot_x_m, robot_y_m, robot_th_r, restart_warmup

    t_lidar = threading.Thread(target=lidar_reader, daemon=True)
    t_slam  = threading.Thread(target=slam_thread,  daemon=True)
    t_lidar.start()
    time.sleep(2.5)
    t_slam.start()

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')
    ax.tick_params(colors='gray')
    ax.set_xlabel('X (metres)', color='gray')
    ax.set_ylabel('Y (metres)', color='gray')

    img = ax.imshow(
        occ_grid,
        cmap='gray', vmin=0.0, vmax=1.0,
        origin='lower',
        extent=[0, MAP_SIZE_M, 0, MAP_SIZE_M],
        interpolation='nearest'
    )

    robot_dot,  = ax.plot([], [], 'ro', markersize=10, zorder=5, label='Corndog')
    trail_line, = ax.plot([], [], 'r-', linewidth=0.8, alpha=0.5)
    arrow_ref   = [None]

    ax.legend(loc='upper right', facecolor='#0f3460', labelcolor='white')

    def update(_):
        with pose_lock:
            x, y, th = robot_x_m, robot_y_m, robot_th_r
            t = list(trail)

        img.set_data(occ_grid)
        robot_dot.set_data([x], [y])

        if t:
            tx, ty = zip(*t)
            trail_line.set_data(tx, ty)

        if arrow_ref[0] is not None:
            arrow_ref[0].remove()
        arrow_ref[0] = ax.annotate(
            '',
            xy=(x + 0.35 * math.cos(th), y + 0.35 * math.sin(th)),
            xytext=(x, y),
            arrowprops=dict(arrowstyle='->', color='cyan', lw=2)
        )

        ax.set_title(
            f'Corndog Localization  |  pos ({x:.2f}m, {y:.2f}m)  |  '
            f'heading {math.degrees(th):.0f}°',
            color='white', size=10
        )

    def on_key(event):
        global running, robot_x_m, robot_y_m, robot_th_r, restart_warmup
        if event.key == 's':
            np.save('corndog_map.npy', occ_grid)
            print("Map saved → corndog_map.npy")
        elif event.key == 'r':
            with pose_lock:
                robot_x_m  = MAP_SIZE_M / 2
                robot_y_m  = MAP_SIZE_M / 2
                robot_th_r = 0.0
            print("Position reset to centre.")
        elif event.key == 'c':
            occ_grid[:] = 0.5
            with pose_lock:
                robot_x_m  = MAP_SIZE_M / 2
                robot_y_m  = MAP_SIZE_M / 2
                robot_th_r = 0.0
                trail.clear()
            restart_warmup = True
            print("Map cleared. Hold Corndog still to rebuild.")
        elif event.key == 'q':
            running = False
            plt.close()

    fig.canvas.mpl_connect('key_press_event', on_key)

    plt.tight_layout()
    plt.ion()
    plt.show(block=False)

    try:
        while running:
            update(None)
            plt.pause(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("Done.")

if __name__ == '__main__':
    main()
