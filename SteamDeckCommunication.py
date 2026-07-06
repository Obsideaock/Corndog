from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import socket
import time
import re
import os
import errno

import cv2
from picamera2 import Picamera2

import MoveLib as mlib
import corndog_pi_emotes as emotes

""" This is to run on the raspberry pi! Do not run it on the steamdeck

v2 changes:
  * EMOTE <token>   -> full emote library via corndog_pi_emotes (wheel support).
                       Buttons (A/B/X/Y/RB) now route through the same dispatcher,
                       so button state and wheel state can never disagree.
  * GAIT <key> <v>  -> live gait/motor tuning via mlib.set_gait_option(). Allowed
                       BEFORE activation (it never moves the robot), so the Deck
                       can push its saved config right after connecting.
  * PING <id>       -> replies "PONG <id> pose=<pose> act=<0|1>\\n" on the same
                       socket (Deck debug HUD: latency + robot mode).
  * Control listener now survives client disconnects: it goes back to accept()
    instead of exiting the process (wifi blips no longer kill the session).
  * Camera is opened ONCE and shared by all MJPEG clients (reconnects no longer
    race the previous handler for the camera).
"""

# Invalidate any saved Flipper motor state — the Steam Deck may leave the
# robot in a completely different position, so the Flipper must do a cold
# start next time rather than trusting a stale saved pose.
_FLIPPER_STATE_FILE = "/tmp/queue_state.json"
try:
    os.remove(_FLIPPER_STATE_FILE)
    print(f"[SteamDeck] Cleared Flipper state file ({_FLIPPER_STATE_FILE})")
except FileNotFoundError:
    pass  # nothing to clear, that's fine
except Exception as e:
    print(f"[SteamDeck] Warning: could not clear Flipper state file: {e}")


# CONFIGURATION
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
emotion_busy = False  # hard lock while an emote is executing

# --- joystick state (latest axes) ---
lx = 0.0
ly = 0.0
rx = 0.0
ry = 0.0

_last_sent = (None, None, None)
_last_send_t = 0.0
SEND_MIN_PERIOD_S = 0.02  # 50 Hz

# Button debounce (press/release or repeats)
_last_btn_time = {}
BTN_DEBOUNCE_S = 0.35


def _current_mode():
    """Listener-level mode derived from the emote state machine."""
    p = emotes.pose()
    if p.startswith("sitting"):
        return "sitting"
    if p == "kneeling":
        return "kneeling"
    if p == "standing":
        return "normal"
    return "posed"     # handstand / fetal / companion / down: no driving


# ==================== SHARED CAMERA + MJPEG ====================
_picam = None
_picam_lock = threading.Lock()


def _get_camera():
    global _picam
    with _picam_lock:
        if _picam is None:
            cam = Picamera2()
            cam.configure(cam.create_preview_configuration(
                main={"size": (CAM_WIDTH, CAM_HEIGHT)}
            ))
            cam.start()
            _picam = cam
        return _picam


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

        try:
            picam = _get_camera()
        except Exception as e:
            print(f"[Video] camera unavailable: {e}")
            return

        try:
            while True:
                frame = picam.capture_array()
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                data = jpg.tobytes()

                try:
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    break
                except OSError as e:
                    if e.errno in (errno.EPIPE, errno.ECONNRESET, errno.ECONNABORTED):
                        break
                    raise
        finally:
            # NOTE: camera stays open for the next client / reconnect
            self.close_connection = True


def run_mjpeg_server():
    server = HTTPServer(("0.0.0.0", MJPEG_PORT), MJPEGHandler)
    print(f"[Video] MJPEG server at http://0.0.0.0:{MJPEG_PORT}/stream.mjpg")
    server.serve_forever()


def _send_cmd_from_axes(force: bool = False):
    global _last_sent, _last_send_t

    vx, vy, wz = mlib.joystick_to_cmd(
        lx, ly, rx, ry,
        left_deadzone=DEADZONE,
        right_deadzone=DEADZONE
    )

    now = time.time()
    if not force:
        if now - _last_send_t < SEND_MIN_PERIOD_S:
            return
        if _last_sent == (vx, vy, wz):
            return

    mlib.gait_command(vx, vy, wz)
    _last_sent = (vx, vy, wz)
    _last_send_t = now


def _is_press_event(parts):
    if len(parts) <= 1:
        return True
    v = parts[1].strip().lower()
    if v in ("0", "release", "released", "up", "false"):
        return False
    if v in ("1", "press", "pressed", "down", "true"):
        return True
    return True


def _debounced(btn, now):
    last = _last_btn_time.get(btn, 0.0)
    if now - last < BTN_DEBOUNCE_S:
        return False
    _last_btn_time[btn] = now
    return True


def _stop_gait_for_emotion():
    if hasattr(mlib, "stop_gait"):
        mlib.stop_gait(schedule_inactivity_reset=False)
    else:
        mlib.gait_command(0.0, 0.0, 0.0)


def _drain_socket(conn):
    try:
        conn.setblocking(False)
        while True:
            try:
                data = conn.recv(4096)
                if not data:
                    break
            except BlockingIOError:
                break
            except Exception:
                break
    finally:
        try:
            conn.setblocking(True)
        except Exception:
            pass


def _reply(conn, text):
    """Best-effort line back to the Deck (PONG etc)."""
    try:
        conn.sendall((text + "\n").encode())
    except Exception:
        pass


def _run_emote(conn, token):
    """Blocking emote execution with the same locking as button emotes."""
    global emotion_busy, busy_until
    emotion_busy = True
    try:
        result = emotes.dispatch(token)
        print(f"[Control] EMOTE {token}: {result}")
    finally:
        emotion_busy = False
        _drain_socket(conn)
    busy_until = time.time() + 0.5


def _handle_client(conn, addr):
    global busy_until, activated, lx, ly, rx, ry, emotion_busy

    print(f"[Control] Connected by {addr}")
    stick_re = re.compile(r"(Left Stick|Right Stick)\s+([XY])\s+(-?\d+(?:\.\d+)?)")

    with conn:
        while True:
            data = conn.recv(2048)
            if not data:
                break

            raw_block = data.decode("utf-8", errors="ignore")
            lines = [ln.strip() for ln in raw_block.splitlines() if ln.strip()]

            for raw in lines:
                now = time.time()

                if emotion_busy:
                    continue

                parts = raw.split()
                if not parts:
                    continue

                # ---- always-available protocol (safe pre-activation) ----
                if parts[0] == "PING":
                    tok = parts[1] if len(parts) > 1 else "0"
                    _reply(conn, f"PONG {tok} pose={emotes.pose()} act={1 if activated else 0}")
                    continue

                if parts[0] == "GAIT" and len(parts) >= 3:
                    key = parts[1]
                    val = " ".join(parts[2:])
                    if not mlib.set_gait_option(key, val):
                        print(f"[Control] unknown GAIT key: {key}")
                    continue

                # ---- activation gate ----
                if not activated:
                    if parts[0] == "A" and _is_press_event(parts):
                        print("[Control] Initial activation: standing up")
                        try:
                            mlib.initialize_servo_angles()
                        except Exception:
                            pass
                        try:
                            mlib.enable_servos()
                        except Exception:
                            pass

                        emotion_busy = True
                        try:
                            _stop_gait_for_emotion()
                            mlib.stand_up()
                        finally:
                            emotion_busy = False
                            _drain_socket(conn)

                        busy_until = now + ACTIVATION_LOCK
                        activated = True
                    continue

                if now < busy_until:
                    continue

                # ---- sticks ----
                stick_msgs = stick_re.findall(raw)
                if stick_msgs:
                    if _current_mode() != "normal":
                        continue

                    for side, axis, val in stick_msgs:
                        f = float(val)
                        if side.startswith("Left"):
                            if axis == "X":
                                lx = f
                            else:
                                ly = f
                        else:
                            if axis == "X":
                                rx = f
                            else:
                                ry = f

                    _send_cmd_from_axes()
                    continue

                # ---- emote wheel ----
                if parts[0] == "EMOTE" and len(parts) >= 2:
                    _run_emote(conn, parts[1])
                    continue

                # ---- buttons ----
                btn = parts[0]

                if not _is_press_event(parts):
                    continue
                if not _debounced(btn, now):
                    continue

                mode = _current_mode()

                if btn == "A":
                    print("[Control] A pressed: stand")
                    _run_emote(conn, "stand")

                elif btn == "B":
                    print("[Control] B pressed: sit toggle")
                    _run_emote(conn, "sit")

                elif btn == "X":
                    if mode == "sitting":
                        print("[Control] X pressed (sitting): wave")
                        _run_emote(conn, "wave")
                    else:
                        print("[Control] X pressed: kneel toggle")
                        _run_emote(conn, "kneel")

                elif btn == "Y":
                    print("[Control] Y pressed: shake")
                    _run_emote(conn, "shake")

                elif btn == "RB":
                    print("[Control] RB pressed: dance")
                    _run_emote(conn, "dance")

                else:
                    pass


def run_control_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CONTROL_PORT))
    sock.listen(1)

    print(f"[Control] Listening on TCP port {CONTROL_PORT} ...")

    try:
        while True:
            conn, addr = sock.accept()
            try:
                _handle_client(conn, addr)
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                print(f"[Control] client error: {e}")
            finally:
                # client gone: halt cleanly, keep listening for a reconnect
                try:
                    _stop_gait_for_emotion()
                except Exception:
                    pass
                print("[Control] Client disconnected — waiting for reconnect")
    finally:
        try:
            _stop_gait_for_emotion()
        except Exception:
            pass
        try:
            if hasattr(mlib, "shutdown"):
                mlib.shutdown()
        except Exception:
            pass
        print("[Control] Listener closed")


if __name__ == "__main__":
    threading.Thread(target=run_mjpeg_server, daemon=True).start()
    run_control_listener()
