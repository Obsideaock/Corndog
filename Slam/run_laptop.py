#!/usr/bin/env python3
"""
LAPTOP DEMO — full v2 with NO hardware.  Run:  python3 run_laptop.py
Then open http://localhost:8001/ and CLICK the map to send the (simulated)
Corndog there. It pre-maps a two-room flat, then drives to wherever you click,
through doorways, drawing the 35x24 cm footprint box. Tests the exact web UI,
planner, and follower you'll use on the robot.

Requires:  pip3 install numpy pillow
"""
import os, sys, math, time, threading
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core import SlamCore, SlamConfig, MotionHint
import sim, nav, web_server
from waypoints import WaypointStore

core = SlamCore(SlamConfig(size_m=16.0, res=0.05))
rng = np.random.default_rng(7)

# --- shared commanded velocity, written by the nav controller's drive() ---
cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
def drive(vx, vy, wz): cmd.update(vx=vx, vy=vy, wz=wz)

# --- phase 1: pre-map both rooms with the canned walk (capped for quick startup) ---
print("Mapping the demo flat...")
it = sim.trajectory(rng=rng); first = True; tp_last = None
for k, (tp, vx, vy, wz, p, r, y) in enumerate(it):
    a, rg = sim.scan(tp, noise=0.02, dropout=0.06, rng=rng)
    if first:
        core.pose = (tp[0], tp[1], tp[2]); core._yaw_bias = y - tp[2]; first = False
    core.update(a, rg, MotionHint(yaw=y, vx=vx, vy=vy, wz=wz, pitch=p, roll=r, dt=0.18))
    tp_last = tp
    if k >= 550:      # enough to map both rooms; keeps startup ~10 s
        break
print("Map ready. Open http://localhost:8001/ and click to drive.")

# --- phase 2: interactive closed loop ---
true = {"x": tp_last[0], "y": tp_last[1], "th": core.pose[2]}
yaw_drift = [0.0]

def sim_loop():
    DT = 0.1
    while True:
        slip = 0.85 + rng.normal(0, 0.04)
        true["th"] += cmd["wz"] * DT + rng.normal(0, 0.008)
        c, s = math.cos(true["th"]), math.sin(true["th"])
        true["x"] += (cmd["vx"] * c - cmd["vy"] * s) * slip * DT + rng.normal(0, 0.003)
        true["y"] += (cmd["vx"] * s + cmd["vy"] * c) * slip * DT + rng.normal(0, 0.003)
        a, rg = sim.scan((true["x"], true["y"], true["th"]), noise=0.02, dropout=0.06, rng=rng)
        yaw_drift[0] += math.radians(0.01)
        imu = true["th"] + yaw_drift[0] + rng.normal(0, math.radians(0.7))
        core.update(a, rg, MotionHint(yaw=imu, vx=cmd["vx"], vy=cmd["vy"], wz=cmd["wz"],
                                      pitch=math.radians(4) * math.sin(time.time() * 6), roll=0.0, dt=DT))
        time.sleep(DT)

nav_ctrl = nav.NavController(core, drive)        # its own follower thread
wp = WaypointStore(None)
web_server.serve(core, nav_ctrl, wp)
threading.Thread(target=sim_loop, daemon=True).start()
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    pass
