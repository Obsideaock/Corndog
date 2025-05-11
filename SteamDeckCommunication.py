from http.server import BaseHTTPRequestHandler, HTTPServer
import threading, socket, time, re

import cv2
from picamera2 import Picamera2
import MoveLib as mlib   # your library from above

# ?? CONFIGURATION ??
MJPEG_PORT    = 8000
CONTROL_PORT  = 65432
CAM_WIDTH     = 1200
CAM_HEIGHT    = 800

# joystick thresholds & speeds
WALK_THRESHOLD   = 0.5
WALK_SPEED       = 15

# lockout durations
REVERSE_SIT_LOCK   = 4.0
REVERSE_KNEEL_LOCK = 2.5

# — global state —
busy_until        = 0.0
mode              = "normal"
activated         = False

# — joystick inputs —
left_y            = 0.0
right_y           = 0.0

# — walking state & thread —
is_walking        = False
stop_walk_event   = threading.Event()
walk_thread       = None

# — turning state & threads —
is_turning_left   = False
is_turning_right  = False
turn_left_thread  = None
turn_right_thread = None


def walking_loop():
    """Perform walk cycles until stop_walk_event is set, then reset stance and stand up."""
    global is_walking
    print("[Control] → joystick walking started")
    mlib.walk_init(speed=WALK_SPEED)
    while not stop_walk_event.is_set():
        mlib.walk_cycle(speed=WALK_SPEED)
    mlib.walk_reset(speed=WALK_SPEED)
    mlib.stand_up()
    is_walking = False
    print("[Control] → joystick walking stopped")


def start_walk():
    """Stop any turn (blocking), then begin walking."""
    global walk_thread, is_walking
    if is_walking:
        return
    stop_turn_left(block=True)
    stop_turn_right(block=True)

    stop_walk_event.clear()
    is_walking = True
    walk_thread = threading.Thread(target=walking_loop, daemon=True)
    walk_thread.start()


def stop_walk(block=True):
    """Signal walk_loop to exit; optionally join until reset+stand complete."""
    global is_walking
    if not is_walking:
        return
    stop_walk_event.set()
    if block and walk_thread:
        walk_thread.join()


def turn_left_worker():
    """Toggle on library’s turn_left (blocking) until toggled off & return-moves done."""
    global is_turning_left
    print("[Control] → joystick turning left started")
    mlib.turn_left(speed=WALK_SPEED)
    # when mlib.turn_left returns, its own return-moves have run
    is_turning_left = False
    print("[Control] → joystick turning left finished")


def start_turn_left():
    """Stop walking (blocking), stop any right turn, then spin left."""
    global turn_left_thread, is_turning_left
    if is_turning_left:
        return
    stop_walk(block=True)
    stop_turn_right(block=True)

    is_turning_left = True
    turn_left_thread = threading.Thread(target=turn_left_worker, daemon=True)
    turn_left_thread.start()


def stop_turn_left(block=True):
    """Toggle-off library turn_left (runs its return moves), wait, then stand up once."""
    global is_turning_left
    if not is_turning_left:
        return
    # this call toggles the gait off and runs the library’s reset
    mlib.turn_left(speed=WALK_SPEED)
    if block and turn_left_thread:
        turn_left_thread.join()
    # now finalize with a full stand
    mlib.stand_up()
    is_turning_left = False


def turn_right_worker():
    print("[Control] → joystick turning right started")
    mlib.turn_right(speed=WALK_SPEED)
    is_turning_right = False
    print("[Control] → joystick turning right finished")


def start_turn_right():
    """Stop walking (blocking), stop any left turn, then spin right."""
    global turn_right_thread, is_turning_right
    if is_turning_right:
        return
    stop_walk(block=True)
    stop_turn_left(block=True)

    is_turning_right = True
    turn_right_thread = threading.Thread(target=turn_right_worker, daemon=True)
    turn_right_thread.start()


def stop_turn_right(block=True):
    """Toggle-off library turn_right (runs its return moves), wait, then stand up once."""
    global is_turning_right
    if not is_turning_right:
        return
    mlib.turn_right(speed=WALK_SPEED)
    if block and turn_right_thread:
        turn_right_thread.join()
    mlib.stand_up()
    is_turning_right = False


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
    global busy_until, mode, left_y, right_y, activated

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

            # 1) joystick walking/turning
            stick_msgs = re.findall(r'(Left Stick|Right Stick) ([XY]) (-?\d+\.\d+)', raw)
            if stick_msgs:
                for side, axis, val in stick_msgs:
                    if axis == 'Y':
                        if side.startswith('Left'):
                            left_y = float(val)
                        else:
                            right_y = float(val)

                on_walk  = left_y < -WALK_THRESHOLD and right_y < -WALK_THRESHOLD
                on_left  = left_y < -WALK_THRESHOLD and right_y >= -WALK_THRESHOLD
                on_right = right_y < -WALK_THRESHOLD and left_y >= -WALK_THRESHOLD

                # walk
                if on_walk:
                    start_walk()
                    continue
                if is_walking and not on_walk:
                    stop_walk(block=True)
                    continue

                # pivot right (left stick only)
                if on_left:
                    start_turn_right()
                    continue
                if is_turning_right and not on_left:
                    stop_turn_right(block=True)
                    continue

                # pivot left (right stick only)
                if on_right:
                    start_turn_left()
                    continue
                if is_turning_left and not on_right:
                    stop_turn_left(block=True)
                    continue

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
                            mlib.unsit();   busy_until = now + REVERSE_SIT_LOCK
                        else:
                            print("Unkneeling")
                            mlib.unkeel(); busy_until = now + REVERSE_KNEEL_LOCK
                        mode = "normal"
                    continue

            # 3) global lockout
            if now < busy_until:
                continue

            # 4) button mappings
            if btn == "A":
                print("A was pressed, standing up")
                mlib.stand_up(); busy_until = now + 3.5
            elif btn == "B":
                print("B was pressed, sitting")
                mlib.sit();      busy_until = now + 2.5; mode="sitting"
            elif btn == "X":
                print("X was pressed, kneeling")
                mlib.kneel();    busy_until = now + 2.5; mode="kneeling"
            elif btn == "Y":
                print("Y was pressed, shaking")
                mlib.shake();    busy_until = now + 12.0
            elif btn == "RB":
                print("RB was pressed, dancing")
                mlib.dance();    busy_until = now + 4.0

    print("[Control] Connection closed")


if __name__ == '__main__':
    mlib.initialize_servo_angles()
    mlib.enable_servos()
    threading.Thread(target=run_mjpeg_server, daemon=True).start()
    run_control_listener()
