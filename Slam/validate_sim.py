#!/usr/bin/env python3
"""
validate_sim.py — Offline accuracy check, no hardware. Run from inside Slam/:

    python3 validate_sim.py

Runs the simulated walk, compares SLAM against dead-reckoning, prints error
metrics, and writes slam_sim_result.png (the reconstructed floorplan).
"""
import os
import sys
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core import SlamCore, SlamConfig, MotionHint
import sim
import render

rng = np.random.default_rng(7)
core = SlamCore(SlamConfig(size_m=16.0, res=0.05, vel_scale=1.0))

true_path, slam_path, dr_path = [], [], []
dr = [0.0, 0.0, 0.0]
dr_yaw_bias = None

n = 0
first = True
for true_pose, vx, vy, wz, pitch, roll, imu_yaw in sim.trajectory(rng=rng):
    ang, rng_m = sim.scan(true_pose, noise=0.02, dropout=0.06, rng=rng)
    hint = MotionHint(yaw=imu_yaw, vx=vx, vy=vy, wz=wz,
                      pitch=pitch, roll=roll, dt=0.18)

    if first:
        core.pose = (true_pose[0], true_pose[1], true_pose[2])
        core._yaw_bias = imu_yaw - true_pose[2]
        dr = [true_pose[0], true_pose[1], true_pose[2]]
        dr_yaw_bias = imu_yaw - true_pose[2]
        first = False

    dr[2] = imu_yaw - dr_yaw_bias
    dr[0] += vx * math.cos(dr[2]) * 0.18
    dr[1] += vx * math.sin(dr[2]) * 0.18

    pose = core.update(ang, rng_m, hint)

    true_path.append((true_pose[0], true_pose[1]))
    slam_path.append((pose[0], pose[1]))
    dr_path.append((dr[0], dr[1]))
    n += 1

true_path = np.array(true_path); slam_path = np.array(slam_path); dr_path = np.array(dr_path)
slam_err = np.linalg.norm(true_path - slam_path, axis=1)
dr_err = np.linalg.norm(true_path - dr_path, axis=1)

print(f"frames: {n}")
print(f"SLAM stats: {core.stats}")
print(f"  dead-reckoning  mean err {dr_err.mean():.3f} m | final {dr_err[-1]:.3f} m | max {dr_err.max():.3f} m")
print(f"  SLAM            mean err {slam_err.mean():.3f} m | final {slam_err[-1]:.3f} m | max {slam_err.max():.3f} m")
print(f"  drift reduction: {100*(1 - slam_err.mean()/dr_err.mean()):.0f}% mean")

im = render.render(core, upscale=4, true_path=true_path.tolist())
im.save("slam_sim_result.png")
print("saved slam_sim_result.png")

core.save_map("sim_map.npz")
core2 = SlamCore(SlamConfig(size_m=16.0, res=0.05))
core2.load_map("sim_map.npz")
print(f"persistence OK: reloaded grid occupied cells = {int(core2.grid.occupied_mask().sum())}")
os.remove("sim_map.npz")
