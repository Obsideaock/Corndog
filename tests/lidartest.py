#!/usr/bin/env python3
"""
lidar_view.py — Live LiDAR radar display. No SLAM, no position tracking.
Just shows what the sensor sees in real time.

Requires: matplotlib, pyrplidar
Install:  pip3 install matplotlib pyrplidar --break-system-packages
Run:      python3 lidar_view.py
"""

import threading
import time
import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar

PORT        = '/dev/ttyUSB0'
MIN_QUALITY = 5
MAX_DIST_M  = 6.0   # Clip display at 6 metres

# ── Shared scan data ──────────────────────────────────────────────────────────

latest_angles = []
latest_dists  = []
scan_lock     = threading.Lock()
running       = True

# ── LiDAR reader thread ───────────────────────────────────────────────────────

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
                    angles = []
                    dists  = []
                    for m in current_scan:
                        if m.quality >= MIN_QUALITY and 50 < m.distance < MAX_DIST_M * 1000:
                            angles.append(math.radians(m.angle))
                            dists.append(m.distance / 1000.0)
                    with scan_lock:
                        latest_angles[:] = angles
                        latest_dists[:]  = dists
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

# ── Visualizer ────────────────────────────────────────────────────────────────

def main():
    global running

    t = threading.Thread(target=lidar_reader, daemon=True)
    t.start()
    time.sleep(2.5)   # Wait for motor to spin up

    fig = plt.figure(figsize=(8, 8), facecolor='#1a1a2e')
    ax  = fig.add_subplot(111, projection='polar')
    ax.set_facecolor('#16213e')
    ax.set_theta_zero_location('N')    # 0° at top = forward
    ax.set_theta_direction(-1)          # Clockwise
    ax.set_rlim(0, MAX_DIST_M)
    ax.set_rticks([1, 2, 3, 4, 5])
    ax.set_yticklabels(['1m','2m','3m','4m','5m'], color='gray', size=8)
    ax.tick_params(colors='gray')
    ax.grid(color='#0f3460', linewidth=0.5)

    dots = ax.scatter([], [], s=3, c=[], cmap='cool',
                      vmin=0, vmax=MAX_DIST_M, alpha=0.9)
    # Robot marker at centre
    ax.scatter([0], [0], s=120, c='#ff6b6b', zorder=5)

    info = fig.text(0.02, 0.02, '', color='gray', size=9)

    plt.tight_layout()
    plt.ion()
    plt.show(block=False)

    try:
        while running:
            with scan_lock:
                angles = list(latest_angles)
                dists  = list(latest_dists)

            if angles:
                import numpy as np
                pts = np.column_stack([angles, dists])
                dots.set_offsets(pts)
                dots.set_array(np.array(dists))
                ax.set_title(
                    f'LiDAR — {len(angles)} points | '
                    f'closest: {min(dists):.2f}m',
                    color='white', size=11, pad=15
                )
                info.set_text(f'farthest: {max(dists):.2f}m')

            plt.pause(0.1)   # ~10fps

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("Done.")

if __name__ == '__main__':
    main()
