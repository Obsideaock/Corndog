from http.server import BaseHTTPRequestHandler, HTTPServer
import threading, socket, time, re, math

import cv2
from picamera2 import Picamera2
import MoveLib as mlib   # your library from above

# ?? CONFIGURATION ??
MJPEG_PORT    = 8000
CONTROL_PORT  = 65432
CAM_WIDTH     = 1200
CAM_HEIGHT    = 800

# stick behavior (binary with angular sectors)
STICK_DEADZONE = 0.2

# gait command magnitudes (use gait_engine defaults; just pick fixed values within its GUI ranges)
VX_CMD = 0.01   # forward/back (units/s)
VY_CMD = 0.01   # strafe (units/s)
WZ_CMD = 0.50   # turn (rad/s)

# lockout durations
REVERSE_SIT_LOCK   = 4.0
REVERSE_KNEEL_LOCK = 2.5

# — global state —
busy_until        = 0.0
mode              = "normal"
activated         = False

# — joystick inputs —
left_x            = 0.0
left_y            = 0.0
right_x           = 0.0
right_y           = 0.0

def _ang_diff_deg(a, b):
    # smallest signed diff a-b in degrees
    d = (a - b + 180.0) % 360.0 - 180.0
    return d

def _in_sector(angle_deg, center_deg, half_width_deg):
    return abs(_ang_diff_deg(angle_deg, center_deg)) <= half_width_deg

def left_stick_to_vx_vy(x, y):
    """
    Binary mapping with:
      - cardinals: 60° each (±30°)
      - diagonals: 30° each (±15°)
    Returns (vx, vy)
    """
    r = math.hypot(x, y)
    if r <= STICK_DEADZONE:
        return 0.0, 0.0

    ang = math.degrees(math.atan2(y, x))
    if ang < 0:
        ang += 360.0

    # Cardinals (±30° around axis)
    if _in_sector(ang, 90.0, 30.0):     # Up
        return +VX_CMD, 0.0
    if _in_sector(ang, 270.0, 30.0):    # Down
        return -VX_CMD, 0.0
    if _in_sector(ang, 0.0, 30.0) or _in_sector(ang, 360.0, 30.0):  # Right
        return 0.0, +VY_CMD
    if _in_sector(ang, 180.0, 30.0):    # Left
        return 0.0, -VY_CMD

    # Diagonals (±15° around diagonal)
    if _in_sector(ang, 45.0, 15.0):     # UpRight
        return +VX_CMD, +VY_CMD
    if _in_sector(ang, 135.0, 15.0):    # UpLeft
        return +VX_CMD, -VY_CMD
    if _in_sector(ang, 225.0, 15.0):    # DownLeft
        return -VX_CMD, -VY_CMD
    if _in_sector(ang, 315.0, 15.0):    # DownRight
        return -VX_CMD, +VY_CMD

    # Fallback (shouldn't happen with the sector tiling)
    return 0.0, 0.0

def right_stick_to_wz(x):
    if abs(x) <= STICK_DEADZONE:
        return 0.0
    # stick left (negative) => turn left (positive wz)
    return +WZ_CMD if x < 0 else -WZ_CMD

def _halt_gait():
    # stop gait motion and return to stand (keeps behavior consistent with old walk/turn)
    try:
        mlib.gait_stop_and_stand()
    except Exception:
        # if gait isn't initialized yet, just stand up
        mlib.stand_up()

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/stream.mjpg':
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header('Cache-Control','no-cache')
        self.send_header('Pragma','no-cache')
        self.send_header('Content-Type','multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()

        picam = Picamera2()
        picam.configure(picam.create_preview_configuration(
            main={"size": (CAM_WIDTH, CAM_HEIGHT)}
        ))
        picam.start()

        try:
            while True:
                frame = picam.capture_array()
                bgr   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, jpg = cv2.imencode('.jpg', bgr,
                                      [int(cv2.IMWRITE_JPEG_QUALITY),70])
                data = jpg.tobytes()

                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type','image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data + b'\r\n')
        except BrokenPipeError:
            pass
        finally:
            picam.close()

def run_mjpeg_server():
    server = HTTPServer(('0.0.0.0', MJPEG_PORT), MJPEGHandler)
    print(f"[Video] MJPEG server at http://0.0.0.0:{MJPEG_PORT}/stream.mjpg")
    server.serve_forever()

def run_control_listener():
    global busy_until, mode, left_x, left_y, right_x, right_y, activated

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', CONTROL_PORT))
    sock.listen(1)
    print(f"[Control] Listening on TCP port {CONTROL_PORT} …")
    conn, addr = sock.accept()
    print(f"[Control] Connected by {addr}")

    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                break

            raw = data.decode('utf-8').strip()
            now = time.time()

            # 0) initial activation
            if not activated:
                parts = raw.split()
                if parts and parts[0] == "A":
                    print("[Control] Initial activation: standing up")
                    mlib.stand_up()
                    busy_until = now + 3.5
                    activated = True
                continue

            # 1) joystick (vx/vy from left stick, wz from right stick) - binary w/ sectors
            stick_msgs = re.findall(r'(Left Stick|Right Stick) ([XY]) (-?\d+\.\d+)', raw)
            if stick_msgs:
                # Respect lockouts/posture modes: ignore driving inputs and keep it still
                if now < busy_until or mode in ("sitting", "kneeling"):
                    # Ensure gait isn't fighting the posture
                    try:
                        mlib.gait_command(0.0, 0.0, 0.0)
                    except Exception:
                        pass
                    continue

                for side, axis, val in stick_msgs:
                    fval = float(val)
                    if side.startswith('Left'):
                        if axis == 'X':
                            left_x = fval
                        else:
                            left_y = fval
                    else:
                        if axis == 'X':
                            right_x = fval
                        else:
                            right_y = fval

                vx, vy = left_stick_to_vx_vy(left_x, left_y)
                wz = right_stick_to_wz(right_x)

                # Run gait if any command present; else stop+stand (old behavior)
                if (abs(vx) + abs(vy) + abs(wz)) > 1e-6:
                    mlib.gait_command(vx, vy, wz)
                else:
                    _halt_gait()

                continue

            # 2) sit/kneel toggle
            parts = raw.split()
            if parts:
                btn = parts[0]
                if mode in ("sitting","kneeling"):
                    if now < busy_until:
                        continue
                    expected = "B" if mode=="sitting" else "X"
                    if btn == expected:
                        if mode=="sitting":
                            print("Unsitting")
                            _halt_gait()
                            mlib.unsit();   busy_until = now + REVERSE_SIT_LOCK
                        else:
                            print("Unkneeling")
                            _halt_gait()
                            mlib.unkeel(); busy_until = now + REVERSE_KNEEL_LOCK
                        mode = "normal"
                    continue

            # 3) global lockout
            if now < busy_until:
                continue

            # 4) button mappings (keep as-is; just halt gait first so motions don't fight)
            if btn == "A":
                print("A was pressed, standing up")
                _halt_gait()
                mlib.stand_up(); busy_until = now + 3.5
            elif btn == "B":
                print("B was pressed, sitting")
                _halt_gait()
                mlib.sit();      busy_until = now + 2.5; mode="sitting"
            elif btn == "X":
                print("X was pressed, kneeling")
                _halt_gait()
                mlib.kneel();    busy_until = now + 2.5; mode="kneeling"
            elif btn == "Y":
                print("Y was pressed, shaking")
                _halt_gait()
                mlib.shake();    busy_until = now + 12.0
            elif btn == "RB":
                print("RB was pressed, dancing")
                _halt_gait()
                mlib.dance();    busy_until = now + 4.0

    print("[Control] Connection closed")

if __name__ == '__main__':
    mlib.initialize_servo_angles()
    mlib.enable_servos()
    threading.Thread(target=run_mjpeg_server, daemon=True).start()
    run_control_listener()
