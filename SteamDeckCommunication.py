from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import socket
import time
import re

import cv2
from picamera2 import Picamera2

import MoveLib as mlib

""" THis is to run on the raspberry pi! Do not run it on the steamdeck"""


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
mode = "normal"  # "normal" | "sitting" | "kneeling"
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

				try:
					self.wfile.write(b"--FRAME\r\n")
					self.send_header("Content-Type", "image/jpeg")
					self.send_header("Content-Length", str(len(data)))
					self.end_headers()
					self.wfile.write(data + b"\r\n")
					# self.wfile.flush()  # optional; may also raise on disconnect
				except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
					break
				except OSError as e:
					# swallow common “client went away” cases (Linux/macOS/Windows vary)
					if e.errno in (errno.EPIPE, errno.ECONNRESET, errno.ECONNABORTED):
						break
					raise

		finally:
			try:
				picam.close()
			except Exception:
				pass
			self.close_connection = True


def run_mjpeg_server():
	server = HTTPServer(("0.0.0.0", MJPEG_PORT), MJPEGHandler)
	print(f"[Video] MJPEG server at http://0.0.0.0:{MJPEG_PORT}/stream.mjpg")
	server.serve_forever()


def _send_cmd_from_axes(force: bool = False):
	"""Convert current (lx, ly, rx, ry) -> (vx, vy, wz) and send gait_command (rate limited)."""
	global _last_sent, _last_send_t

	# FIX: MoveLib uses left_deadzone/right_deadzone (not deadzone=)
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
	"""
	Returns True only for a press.
	Handles common formats:
	  "X" / "X 1" / "X pressed" => press
	  "X 0" / "X released"      => ignore
	"""
	if len(parts) <= 1:
		return True
	v = parts[1].strip().lower()
	if v in ("0", "release", "released", "up", "false"):
		return False
	if v in ("1", "press", "pressed", "down", "true"):
		return True
	# unknown token: treat as press (safer than missing inputs)
	return True


def _debounced(btn, now):
	last = _last_btn_time.get(btn, 0.0)
	if now - last < BTN_DEBOUNCE_S:
		return False
	_last_btn_time[btn] = now
	return True


def _stop_gait_for_emotion():
	# Stop gait WITHOUT scheduling the 1.5s inactivity reset (prevents “mystery” IK moves during emotes)
	if hasattr(mlib, "stop_gait"):
		mlib.stop_gait(schedule_inactivity_reset=False)
	else:
		# fallback (older MoveLib): this may schedule a reset; your current MoveLib has stop_gait
		mlib.gait_command(0.0, 0.0, 0.0)


def _drain_socket(conn):
	"""
	After a blocking emote, the TCP buffer may contain queued release/repeat events.
	Drain them so we don't accidentally trigger a toggle again.
	"""
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


def run_control_listener():
	global busy_until, activated, lx, ly, rx, ry, mode, emotion_busy

	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	sock.bind(("0.0.0.0", CONTROL_PORT))
	sock.listen(1)

	print(f"[Control] Listening on TCP port {CONTROL_PORT} …")
	conn, addr = sock.accept()
	print(f"[Control] Connected by {addr}")

	stick_re = re.compile(r"(Left Stick|Right Stick)\s+([XY])\s+(-?\d+(?:\.\d+)?)")

	try:
		with conn:
			while True:
				data = conn.recv(2048)
				if not data:
					break

				raw_block = data.decode("utf-8", errors="ignore")
				lines = [ln.strip() for ln in raw_block.splitlines() if ln.strip()]

				for raw in lines:
					now = time.time()

					# Hard lock: if an emote is currently executing, ignore everything
					# (but we still keep reading loop to avoid backlog growth)
					if emotion_busy:
						continue

					# 0) initial activation (press A once)
					if not activated:
						parts = raw.split()
						if parts and parts[0] == "A" and _is_press_event(parts):
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
							mode = "normal"
						continue

					# 1) lockout window
					if now < busy_until:
						continue

					# 2) joystick updates (only in normal mode)
					stick_msgs = stick_re.findall(raw)
					if stick_msgs:
						# ignore joystick while sitting/kneeling
						if mode != "normal":
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

					# 3) buttons
					parts = raw.split()
					if not parts:
						continue
					btn = parts[0]

					# press-only
					if not _is_press_event(parts):
						continue

					# debounce
					if not _debounced(btn, now):
						continue

					# If sitting/kneeling is toggled ON: disable everything except A and the matching toggle button.
					if mode == "sitting" and btn not in ("Y", "B", "X"):
						continue
					if mode == "kneeling" and btn != "X":
						continue
					if btn == "Y" and mode != "sitting":
						continue

					if btn == "A":
						print("[Control] A pressed: stand up")
						emotion_busy = True
						try:
							_stop_gait_for_emotion()
							mlib.stand_up()
							mode = "normal"
						finally:
							emotion_busy = False
							_drain_socket(conn)
						busy_until = now + 0.5

					elif btn == "B":
						print("[Control] B pressed: sit toggle")
						emotion_busy = True
						try:
							_stop_gait_for_emotion()
							if getattr(mlib, "_is_sitting", False) and hasattr(mlib, "unsit"):
								mlib.unsit()
								mode = "normal"
							else:
								mlib.sit()
								mode = "sitting"
						finally:
							emotion_busy = False
							_drain_socket(conn)
						busy_until = now + 0.5

					elif btn == "X":
						if mode == "sitting":
							print("[Control] X pressed (sitting): wave")
							emotion_busy = True
							try:
								_stop_gait_for_emotion()
								mlib.wave()
							finally:
								emotion_busy = False
								_drain_socket(conn)
							busy_until = now + 0.5

						else:
							# standing/normal -> kneel toggle
							print("[Control] X pressed: kneel toggle")
							emotion_busy = True
							try:
								_stop_gait_for_emotion()
								if getattr(mlib, "_is_kneeling", False) and hasattr(mlib, "unkneel"):
									mlib.unkneel()
									mode = "normal"
								else:
									mlib.kneel()
									mode = "kneeling"
							finally:
								emotion_busy = False
								_drain_socket(conn)
							busy_until = now + 0.5

					elif btn == "Y":
						print("[Control] Y pressed: shake")
						emotion_busy = True
						try:
							_stop_gait_for_emotion()
							mlib.shake()
						finally:
							emotion_busy = False
							_drain_socket(conn)
						busy_until = now + 0.5

					elif btn == "RB":
						print("[Control] RB pressed: dance")
						emotion_busy = True
						try:
							_stop_gait_for_emotion()
							mlib.dance()
						finally:
							emotion_busy = False
							_drain_socket(conn)
						busy_until = now + 0.5

					else:
						# ignore other buttons (START/BACK removed)
						pass

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
		print("[Control] Connection closed")


if __name__ == "__main__":
	threading.Thread(target=run_mjpeg_server, daemon=True).start()
	run_control_listener()
