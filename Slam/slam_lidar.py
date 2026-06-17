"""
slam_lidar.py — RPLIDAR A1 reader thread for SLAM.

Adapted from your lidar_view.py. Produces the latest complete scan as
(angles_rad, ranges_m) in the ROBOT BODY FRAME where 0 rad = forward and
angle increases counter-clockwise (to match the world/IMU convention).

If the map comes out mirrored during bring-up, flip LIDAR_CCW.
"""

from __future__ import annotations
import threading
import time
import math
import numpy as np
from pyrplidar import PyRPlidar

PORT = '/dev/ttyUSB0'
BAUD = 115200
MIN_QUALITY = 5
MAX_DIST_M = 6.0

# bring-up knobs
LIDAR_CCW = False          # RPLIDAR reports clockwise degrees; False negates to CCW
ANGLE_OFFSET_DEG = 0.0     # set if 0° on the sensor isn't the robot's nose (±5° ok)


class LidarReader:
    def __init__(self, port=PORT):
        self.port = port
        self._angles = np.array([], np.float32)
        self._ranges = np.array([], np.float32)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self.frames = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def get_scan(self):
        with self._lock:
            return self._angles.copy(), self._ranges.copy()

    def _publish(self, current_scan):
        ang, rng = [], []
        off = math.radians(ANGLE_OFFSET_DEG)
        sign = 1.0 if LIDAR_CCW else -1.0
        for m in current_scan:
            if m.quality >= MIN_QUALITY and 50 < m.distance < MAX_DIST_M * 1000:
                a = sign * math.radians(m.angle) + off
                ang.append(a)
                rng.append(m.distance / 1000.0)
        with self._lock:
            self._angles = np.array(ang, np.float32)
            self._ranges = np.array(rng, np.float32)
        self.frames += 1

    def _run(self):
        attempt = 0
        while self._running:
            attempt += 1
            lidar = PyRPlidar()
            try:
                lidar.connect(port=self.port, baudrate=BAUD, timeout=3)
                lidar.stop(); lidar.set_motor_pwm(0); time.sleep(1.0)
                lidar.set_motor_pwm(500); time.sleep(2.0)
                print(f"[LiDAR] running" + ("" if attempt == 1 else f" (recovered, attempt {attempt})"))
                gen = lidar.start_scan_express(0)
                current = []
                for meas in gen():
                    if not self._running:
                        break
                    try:
                        if meas.start_flag and current:
                            self._publish(current)
                            current = []
                        current.append(meas)
                    except (IndexError, AttributeError):
                        current = []
                # generator ended cleanly
                break
            except Exception as e:
                # The A1's "sync bytes are mismatched" desync on startup lands
                # here; tear down and reconnect instead of dying.
                print(f"[LiDAR] error: {e} — resyncing...")
                try:
                    lidar.stop(); lidar.set_motor_pwm(0); lidar.disconnect()
                except Exception:
                    pass
                if not self._running:
                    break
                time.sleep(min(1.0 + 0.5 * attempt, 4.0))   # backoff
                continue
            finally:
                try:
                    lidar.stop(); lidar.set_motor_pwm(0); lidar.disconnect()
                except Exception:
                    pass
        print("[LiDAR] stopped")
