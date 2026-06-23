"""
slam_app.py — ON-ROBOT entry point. Run this ON the Pi:

    python3 -m corndog_slam.slam_app                 # SLAM + Steam Deck control
    python3 -m corndog_slam.slam_app --no-control    # SLAM only (stationary test)
    python3 -m corndog_slam.slam_app --load map.npz  # resume an existing map

It brings up SLAM (LiDAR reader + core loop + map web view) and, in the SAME
process, your existing control so you can walk/stand Corndog while it maps.
Single MoveLib import => no hardware contention.

View the live map at  http://<pi-ip>:8001/
"""

from __future__ import annotations
import sys
import math
import time
import argparse
import threading

from core import SlamCore, SlamConfig, MotionHint
from slam_lidar import LidarReader
import slam_bridge
import web_server
import nav as navmod
import relocalize
from waypoints import WaypointStore
from keepout import KeepoutStore
from loop_closure import LoopCloser


def run_slam_loop(core: SlamCore, lidar: LidarReader,
                  save_path="corndog_map.npz", save_every=15.0,
                  target_dt=0.18):
    last = time.monotonic()
    last_save = last
    while True:
        t0 = time.monotonic()
        dt = t0 - last
        last = t0
        ang, rng = lidar.get_scan()
        if len(rng) >= 10:
            hint = MotionHint(**slam_bridge.read_motion(dt))
            core.update(ang, rng, hint)
            if save_path and (t0 - last_save) > save_every:
                core.save_map(save_path)
                last_save = t0
        # pace to roughly the LiDAR's rate
        time.sleep(max(0.0, target_dt - (time.monotonic() - t0)))


def keyboard_teleop(mlib):
    """
    Simple WASD teleop from the terminal (works over SSH). Latched velocity:
    a key sets the motion and Corndog keeps going until you change it or stop.

        w / s : forward / back        a / d : turn left / right
        q / e : strafe left / right   space : stop      x : quit
    """
    import tty
    import termios
    import select

    VX = getattr(mlib, "VX_SPEED", 0.15)
    VY = getattr(mlib, "VY_SPEED", 0.11)
    WZ = getattr(mlib, "WZ_MAX", 1.0) * 0.5      # gentle turns map better

    print("\r\n  Keyboard control:  w/s fwd/back   a/d turn   q/e strafe"
          "   [space] stop   x quit\r\n")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    vx = vy = wz = 0.0
    try:
        tty.setcbreak(fd)
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch == "w":   vx, vy, wz = VX, 0.0, 0.0
            elif ch == "s": vx, vy, wz = -VX, 0.0, 0.0
            elif ch == "a": vx, vy, wz = 0.0, 0.0, WZ        # +wz = left (CCW)
            elif ch == "d": vx, vy, wz = 0.0, 0.0, -WZ
            elif ch == "q": vx, vy, wz = 0.0, VY, 0.0        # +vy = left strafe
            elif ch == "e": vx, vy, wz = 0.0, -VY, 0.0
            elif ch in (" ", "k"): vx = vy = wz = 0.0
            elif ch == "x": break
            else: continue
            mlib.gait_command(vx, vy, wz)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            mlib.gait_command(0.0, 0.0, 0.0)
        except Exception:
            pass
        print("\r\n[Control] keyboard teleop ended; Corndog stopped.\r\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control", choices=["steamdeck", "keys", "none"],
                    default="steamdeck",
                    help="how to drive Corndog while mapping (default: steamdeck)")
    ap.add_argument("--no-control", action="store_true",
                    help="alias for --control none (stationary mapping)")
    ap.add_argument("--no-stand", action="store_true",
                    help="don't auto enable_servos()/stand_up() at start")
    ap.add_argument("--load", default=None, help="resume from a saved .npz map")
    ap.add_argument("--save", default="corndog_map.npz")
    ap.add_argument("--size", type=float, default=20.0, help="map side length (m)")
    ap.add_argument("--res", type=float, default=0.05, help="cell size (m)")
    ap.add_argument("--loopclose", action="store_true",
                    help="enable background loop-closure / pose-graph optimisation "
                         "(OFF by default — it can disturb live tracking on long runs)")
    ap.add_argument("--map-while-moving", action="store_true",
                    help="also build the map during straight travel (still gated on "
                         "turns); default only maps when essentially stopped")
    args = ap.parse_args()
    control = "none" if args.no_control else args.control

    import MoveLib as mlib
    if not args.no_stand:
        try:
            mlib.enable_servos(); mlib.stand_up()
            time.sleep(1.0)
            print("[Corndog] standing")
        except Exception as e:
            print(f"[Corndog] stand-up skipped: {e}")

    _scfg = SlamConfig(size_m=args.size, res=args.res, vel_scale=slam_bridge.VEL_SCALE)
    if args.map_while_moving:
        _scfg.map_gate_v = 0.30        # allow mapping during straight walking
        _scfg.map_gate_wz = 0.15       # ...but still skip turns (the main smear source)
        print("[SLAM] map-while-moving: mapping during straight travel, gated on turns")
    else:
        print("[SLAM] map writes gated to near-stationary (localisation stays live)")
    core = SlamCore(_scfg)
    map_path = args.save
    if args.load:
        core.load_map(args.load)
        map_path = args.load
        print(f"[SLAM] loaded map {args.load}")

    lidar = LidarReader()
    lidar.start()
    time.sleep(2.5)  # motor spin-up

    # One-time global relocalization when resuming a map (your ~10 s "where am I").
    if args.load:
        print("[SLAM] relocalizing in loaded map...")
        located = False
        for _ in range(40):
            ang, rng = lidar.get_scan()
            if len(rng) >= 40:
                yaw = slam_bridge.read_yaw()
                p, conf = relocalize.relocalize(core.grid, ang, rng)
                if p is not None and conf > 0.45:
                    core.pose = p
                    core._yaw_bias = (yaw if yaw is not None else p[2]) - p[2]
                    print(f"[SLAM] relocalized at ({p[0]:.2f}, {p[1]:.2f}, "
                          f"{math.degrees(p[2]):.0f}deg) confidence={conf:.2f}")
                    located = True
                    break
            time.sleep(0.2)
        if not located:
            print("[SLAM] couldn't confidently relocalize — starting at map origin; "
                  "drive him a few metres and tracking will lock on.")

    wp = WaypointStore(map_path)
    ko = KeepoutStore(map_path)
    nav_ctrl = navmod.NavController(core, mlib.gait_command, keepout=ko)
    loop_closer = LoopCloser(core, start_thread=True) if args.loopclose else None
    if loop_closer:
        print("[SLAM] loop closure ENABLED (background pose-graph optimisation)")
    web_server.serve(core, nav_ctrl, wp, ko, loop_closer)
    threading.Thread(target=run_slam_loop, args=(core, lidar, args.save),
                     daemon=True).start()
    print("[SLAM] running — open http://<pi-ip>:8001/  (click the map to send him there)")

    if control == "none":
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    elif control == "keys":
        keyboard_teleop(mlib)
    else:
        # Reuse the existing joystick control listener so you can walk/stand
        # Corndog while it maps. SteamDeckCommunication imports picamera2 at the
        # top purely for ITS camera stream, which SLAM doesn't need (we serve our
        # own map at :8001). If that camera import fails (a common numpy-2.x ABI
        # break vs the system simplejpeg), stub it so CONTROL STILL WORKS, and
        # just skip the robot camera feed.
        camera_ok = True
        try:
            import picamera2  # noqa: F401  -- use the real one if it imports
        except Exception as e:
            camera_ok = False
            print(f"[Control] robot camera disabled ({type(e).__name__}); "
                  f"walking/standing control still active. "
                  f"To restore the camera: pip3 install --break-system-packages 'numpy<2'")
            import types
            _stub = types.ModuleType("picamera2")

            class _DummyCam:  # accepts any call/attribute, does nothing
                def __init__(self, *a, **k):
                    pass

                def __getattr__(self, _):
                    return lambda *a, **k: None

            _stub.Picamera2 = _DummyCam
            sys.modules["picamera2"] = _stub

        try:
            import SteamDeckCommunication as ctrl
            if camera_ok:
                threading.Thread(target=ctrl.run_mjpeg_server, daemon=True).start()
            ctrl.run_control_listener()  # blocks; drive Corndog while it maps
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"[SLAM] control unavailable ({type(e).__name__}: {e}); "
                  f"continuing MAP-ONLY at :8001.")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass

    core.save_map(args.save)
    print(f"[SLAM] final map saved to {args.save}")


if __name__ == "__main__":
    main()
