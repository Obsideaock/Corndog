# Corndog SLAM (v2.0)

Pure-Python 2D LiDAR SLAM for the Corndog quadruped. No ROS. Builds a live
occupancy-grid map and tracks where Corndog is on it while you walk it around.

Hector-style scan-to-map matching (the approach proven on the mike4192 SpotMicro
with an RPLidar A1), reimplemented in pure numpy and fused with your **BNO08x**
heading and the **gait engine's commanded velocity** as a motion prior.
Validated in simulation before any hardware: on a simulated two-room walk it
reconstructs the floorplan and tracks the path to ~12 cm mean error — a 92%
reduction vs dead-reckoning, at ~3x less CPU per scan than v1.

**v2.0 adds point-and-click navigation:** open the map in a browser, click a
spot, and Corndog plans a path (A* with adjustable wall clearance), drives there
following it, hard-stops for anything that appears in front, and arrives within
~10 cm. Plus a new canvas UI (pan/zoom, STOP, named "spots"), a 35x24 cm oriented
footprint box, and one-time relocalization when you resume a saved map.

## Where this folder goes
Drop the whole `Slam/` folder into your main Corndog directory, next to
`MoveLib.py`:

```
Corndog/                      <- your main folder
  MoveLib.py
  SteamDeckCommunication.py
  ... your other files
  Slam/                       <- this folder
    run_robot.py              <- run this ON THE ROBOT
    run_laptop.py             <- run this ON A LAPTOP to test (no hardware)
    validate_sim.py           <- optional offline accuracy check
    README.md
    core.py  occupancy.py  scan_match.py  render.py  sim.py
    slam_bridge.py  slam_lidar.py  slam_map_server.py  slam_app.py
```

The slam code reaches up one level to import `MoveLib` and
`SteamDeckCommunication`, so it must stay inside the main Corndog folder.

## Run it

**On the robot (the Pi):**
```
cd Slam
python3 run_robot.py                 # SLAM + your Steam Deck control
python3 run_robot.py --no-control    # SLAM only, robot stays still
python3 run_robot.py --load corndog_map.npz   # resume + relocalize in a saved map
python3 run_robot.py --control keys           # drive from the terminal (WASD)
```
Then open `http://<pi-ip>:8001/` in a browser. **Click anywhere on the map to
send Corndog there**; drag to pan, wheel to zoom, big red **STOP** to halt. Save
the current position as a named "spot" and tap it later to go back. You can still
drive Corndog manually (Steam Deck) and it maps as you walk. With `--load`, it
spends a few seconds at startup figuring out where it is in the loaded map before
tracking takes over.

**On a laptop first (no hardware needed):**
```
cd Slam
python3 run_laptop.py                 # then open http://localhost:8001/
python3 validate_sim.py               # prints metrics, writes slam_sim_result.png
```
`run_laptop.py` uses the exact same map view as the robot, fed by a simulator.

## Install
On the Pi: `pip3 install pillow pyrplidar --break-system-packages` — this code
uses your SYSTEM numpy (1.x) and needs no scipy. **Do NOT pip-install numpy or
scipy on the Pi:** numpy 2.x breaks `picamera2`/`simplejpeg` (the camera stack).

If you already did (you'll see a `numpy.dtype size changed` error), revert with:
```
pip3 uninstall -y numpy scipy
sudo apt install --reinstall python3-numpy
```
(your BNO08x / adafruit deps are already there via MoveLib).
On a laptop (for the demo): `pip3 install numpy pillow`.

## What it does (v2.0)
- Live **5 cm occupancy grid**, log-odds, with free-space carving so people
  walking through fade instead of becoming permanent walls.
- Pose from **scan matching** seeded by IMU yaw + commanded velocity.
- **Tilt gating**: scans taken mid-stride (body pitch/roll past a threshold)
  don't corrupt the map.
- **Live web map** at `:8001` — robot dot + heading, live scan (cyan) vs mapped
  walls (white), ~30 s trajectory trail (amber).
- **Save / load** maps (`.npz`), autosave every 15 s.
- Runs in the **same process** as your Steam Deck control (single MoveLib import,
  no hardware contention).
- **Faster matcher** (v2): the likelihood field is rebuilt only in a window
  around the robot and the search is vectorised — ~3x less CPU per scan than v1,
  with equal-or-better accuracy.
- **Click-to-go navigation** (v2): A* over the map with a configurable clearance
  (`ROBOT_RADIUS_M + DEFAULT_CLEARANCE_M` in `nav.py`, ~10 cm default) so paths
  keep off walls; pure-pursuit follower; forward-cone **obstacle hard-stop** that
  pauses and resumes; arrives within ~10 cm. Goals on walls, in unknown space, or
  unreachable are rejected with a reason.
- **Canvas web UI** (v2): crisp (no JPEG re-encode, lighter than the old MJPEG),
  click-to-go, pan/zoom, STOP, named waypoints, planned-path overlay, and a
  **35 x 24 cm oriented footprint box** so you can judge what he'll clear.
- **Relocalization** (v2): on `--load`, a one-time global "where am I" search
  places him in the saved map from a single scan.
- **LiDAR auto-resync** (v2): the A1's occasional startup byte-mismatch now
  retries/reconnects automatically instead of needing a restart.

## Deferred to v2.1 (as agreed)
**Loop closure + pose-graph optimisation** (removes the slow global drift on long
multi-loop runs — the faint wall-doubling in the sim image, and the main thing
between "arrives within ~10 cm on the map" and "within ~10 cm physically"),
sub-5 cm resolution, autonomous explore/auto-mapping, and dynamic obstacle
*re-routing* (today he hard-stops and waits rather than planning around).

## Bring-up calibration (once, on the robot)
Two sign conventions can't be known until it's on Corndog. Each is a one-line flip:

1. **Heading sense** — `YAW_SIGN` in `slam_bridge.py`. Turn Corndog ~90° left in
   place; if the map rotates the wrong way, set it to `-1.0`.
2. **LiDAR direction** — `LIDAR_CCW` in `slam_lidar.py`. If the map comes out
   mirrored (left/right flipped), toggle it. Use `ANGLE_OFFSET_DEG` if 0° isn't
   dead-ahead.
3. **Velocity scale** — `VEL_SCALE` in `slam_bridge.py`. Walk a measured 2 m; if
   the trail reads long/short, scale it. (Minor — the matcher fixes translation.)

Tilt gate (`tilt_gate_deg`, default 9°) and log-odds rates live in `core.py` /
`occupancy.py` for tuning to your gait.

## File map
```
core.py            SlamCore: motion prior + matching + map + trajectory (hardware-agnostic)
occupancy.py       log-odds grid, ray carving, save/load
scan_match.py      multi-res correlative scan matching + likelihood field
render.py          grid + overlays -> image (shared by web view + sim)
sim.py             hardware-free simulator (floorplan, foot-slip, body bob)
slam_bridge.py     [robot] BNO08x + gait engine -> motion hint
slam_lidar.py      [robot] RPLIDAR reader thread
nav.py             A* planner (configurable clearance) + pure-pursuit follower + obstacle stop
relocalize.py      global "where am I" search for a loaded map
web_server.py      canvas/JSON web UI (click-to-go, pan/zoom, STOP, waypoints, footprint box)
waypoints.py       named map locations, saved next to the map
slam_map_server.py old MJPEG map view (superseded by web_server.py; kept as fallback)
slam_app.py        [robot] launcher internals (called by run_robot.py)
run_robot.py       ON-ROBOT entry point
run_laptop.py      laptop demo (no hardware) — full click-to-navigate in sim
validate_sim.py    offline accuracy check
```
