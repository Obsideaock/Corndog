# SteamDeckCommunication.py
# Minimal Steam Deck comms script updated for the new MoveLib:
# - Uses mlib.joystick_to_cmd(lx, ly, rx) + mlib.gait_command(vx, vy, wz)
# - No walk/turn threads (the gait engine runs inside MoveLib only while commanded)
# - On initial "A" activation: triggers a stand reset via gait_command(0,0,0)
# - Keeps MJPEG streaming unchanged

from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import socket
import time
import re
import sys

import cv2
from picamera2 import Picamera2

import MoveLib as mlib

# ?? CONFIGURATION ??
MJPEG_PORT    = 8000
CONTROL_PORT  = 65432
CAM_WIDTH     = 1200
CAM_HEIGHT    = 800

# joystick thresholds
DEADZONE = 0.5

# activation lockout (prevents rapid re-triggering)
ACTIVATION_LOCK = 2.0

# --- global state ---
busy_until = 0.0
activated = False

# --- joystick state (latest axes) ---
lx = 0.0
ly = 0.0
rx = 0.0  # right stick X only (rotation)

_last_sent = (None, None, None)
_last_send_t = 0.0
SEND_MIN_PERIOD_S = 0.02  # don't spam faster than 50 Hz


def _try_call(fn_name: str, *args, **kwargs):
    """
    Try to call a function from MoveLib first, then from __main__ or a hardware module if available.
    This keeps your script compatible even if those helpers live outside MoveLib.
    """
    # 1) MoveLib
    fn = getattr(mlib, fn_name, None)
    if callable(fn):
        return fn(*args, **kwargs)

    # 2) __main__
    main_mod = sys.modules.get("__main__", None)
    if main_mod is not None:
        fn = getattr(main_mod, fn_name, None)
        if callable(fn):
            return fn(*args, **kwargs)

    # 3) optional hardware module if env var was set
    hw_mod_name = getattr(mlib, "HARDWARE_MODULE", None)
    if isinstance(hw_mod_name, str) and hw_mod_name:
        try:
            hw_mod = __import__(hw_mod_name)
            fn = getattr(hw_mod, fn_name, None)
            if callable(fn):
                return fn(*args, **kwargs)
        except Exception:
            pass

    return None


def _init_robot_if_available():
    """
    If your old stack expects servo init/enable, try to call them if present.
    If they don't exist, we just proceed with joystick gait control.
    """
    _try_call("initialize_servo_angles")
    _try_call("enable_servos")


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        picam = Picamera2()
        picam.configure(picam.create_preview_configuration(
            main={"size": (CAM_WIDTH, CAM_HEIGHT)}
        ))
        picam.start()

        try:
            while True:
                frame = picam.capture_array()
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                data = jpg.tobytes()

                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data + b"\r\n")
        except BrokenPipeError:
            pass
        finally:
            try:
                picam.close()
            except Exception:
                pass


def run_mjpeg_server():
    server = HTTPServer(("0.0.0.0", MJPEG_PORT), MJPEGHandler)
    print(f"[Video] MJPEG server at http://0.0.0.0:{MJPEG_PORT}/stream.mjpg")
    server.serve_forever()


def _send_cmd_from_axes(force: bool = False):
    """
    Convert current (lx, ly, rx) -> (vx, vy, wz) using MoveLib's mapping
    and send with gait_command. Rate-limited and change-detected.
    """
    global _last_sent, _last_send_t

    vx, vy, wz = mlib.joystick_to_cmd(lx, ly, rx, deadzone=DEADZONE)

    now = time.time()
    if not force:
        if now - _last_send_t < SEND_MIN_PERIOD_S:
            return
        if _last_sent == (vx, vy, wz):
            return

    mlib.gait_command(vx, vy, wz)
    _last_sent = (vx, vy, wz)
    _last_send_t = now


def run_control_listener():
    global busy_until, activated, lx, ly, rx

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CONTROL_PORT))
    sock.listen(1)

    print(f"[Control] Listening on TCP port {CONTROL_PORT} …")
    conn, addr = sock.accept()
    print(f"[Control] Connected by {addr}")

    # Accept floats with or without decimals
    stick_re = re.compile(r"(Left Stick|Right Stick)\s+([XY])\s+(-?\d+(?:\.\d+)?)")

    try:
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                raw = data.decode("utf-8", errors="ignore").strip()
                now = time.time()

                # 0) initial activation (press A once to "arm" control)
                if not activated:
                    parts = raw.split()
                    if parts and parts[0] == "A":
                        print("[Control] Initial activation: scheduling stand reset")
                        # Force stop + (MoveLib will do stand after 1.5s inactivity)
                        mlib.gait_command(0.0, 0.0, 0.0)
                        _last_sent = (0.0, 0.0, 0.0)
                        busy_until = now + ACTIVATION_LOCK
                        activated = True
                    continue

                # 1) optional global lockout window
                if now < busy_until:
                    continue

                # 2) joystick updates (preferred control path)
                stick_msgs = stick_re.findall(raw)
                if stick_msgs:
                    for side, axis, val in stick_msgs:
                        f = float(val)
                        if side.startswith("Left"):
                            if axis == "X":
                                lx = f
                            else:
                                ly = f
                        else:
                            # right stick: only rx (rotation). ignore right Y for now.
                            if axis == "X":
                                rx = f

                    _send_cmd_from_axes()
                    continue

                # 3) button presses (only keep A as an emergency stand reset for now)
                parts = raw.split()
                if not parts:
                    continue
                btn = parts[0]

                if btn == "A":
                    print("[Control] A pressed: stop gait (stand reset after 1.5s inactivity)")
                    mlib.gait_command(0.0, 0.0, 0.0)
                    _send_cmd_from_axes(force=True)
                    busy_until = now + 0.25
                else:
                    # For now ignore other actions (since MoveLib is gait-only)
                    # print(f"[Control] Ignored button: {btn}")
                    pass

    finally:
        # Safety: stop gait on disconnect
        try:
            mlib.gait_command(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            if hasattr(mlib, "shutdown"):
                mlib.shutdown()
        except Exception:
            pass
        print("[Control] Connection closed")


if __name__ == "__main__":
    _init_robot_if_available()
    threading.Thread(target=run_mjpeg_server, daemon=True).start()
    run_control_listener()
