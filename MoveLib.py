# MoveLib.py
# ---------------------------------------------------------------------------
# Corndog MoveLib (single-file, no "resolve symbols"):
# - Includes the hardware + IK code directly (PCA9685 servos + BNO08X IMU + IK)
# - Includes a headless gait engine loop using the same tick-style process as gait_engine_app.py
# - Includes "emotions" (sit/unsit, kneel/unkneel, shake, dance)
#
# Key behaviors:
# - gait_command(vx, vy, wz):
#     * nonzero starts the gait scheduler thread
#     * zero stops the gait scheduler thread and (after 1.5s inactivity) resets legs to neutral
# - stop_gait(schedule_inactivity_reset=False):
#     * stops gait WITHOUT scheduling the 1.5s reset (use this for emotions)
#
# Joystick mapping:
# - Left stick: 8-direction snap (±30° capture), binary motion
#     * Cardinals use VX_SPEED / VY_SPEED (note: right => vy NEGATIVE to match your old mapping)
#     * Diagonals scaled so resultant == DIAG_SPEED (pythagoras-correct, keeps vx/vy ratio)
# - Right stick: analog yaw from rx with deadzone, AND fades to 0 as |ry| -> 1
#     * preserves your old sign convention: rx>0 => wz NEGATIVE
# ---------------------------------------------------------------------------

from __future__ import annotations

import time
import math
import threading
import heapq
from dataclasses import dataclass
from typing import Callable, Dict, Tuple, Optional, Any

import board
import busio
import numpy as np
from gpiozero import OutputDevice
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_MAGNETOMETER,
	BNO_REPORT_LINEAR_ACCELERATION,
	BNO_REPORT_GRAVITY,
	BNO_REPORT_ROTATION_VECTOR,
	BNO_REPORT_GAME_ROTATION_VECTOR,
	BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)

from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
	t_rightback, t_rightfront, t_leftfront, t_leftback, ikine
)

import sys
sys.path.insert(0, "/home/Corndog/")
from lcd import lcd_library as lcd  # type: ignore


# =============================================================================
# Small helpers
# =============================================================================

def _clamp(x: float, lo: float, hi: float) -> float:
	return hi if x > hi else lo if x < lo else x


# =============================================================================
# Hardware setup (PCA9685 + servos + output enable)
# =============================================================================

_i2c_pca = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(_i2c_pca)
pca.frequency = 50

OE_PIN = 22
output_enable = OutputDevice(OE_PIN, active_high=False)

servo_channels = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]
servo_home: Dict[int, float] = {
	0: 39, 1: 231, 4: 222, 5: 50,
	6: 128, 7: 130, 8: 133, 9: 135,
	10: 73, 11: 199, 14: 235, 15: 33
}

servos: Dict[int, servo.Servo] = {ch: servo.Servo(pca.channels[ch]) for ch in servo_channels}
for ch in servos:
	servos[ch].set_pulse_width_range(500, 2500)
	servos[ch].actuation_range = 270

servo_angles: Dict[int, float] = {}


def initialize_servo_angles():
	"""Initialize servo_angles from the configured home positions."""
	global servo_angles
	servo_angles = {ch: float(ang) for ch, ang in servo_home.items()}


def enable_servos():
	output_enable.off()
	print("Servos enabled")


def disable_servos():
	try:
		lcd.clear()
	except Exception:
		pass
	output_enable.on()
	for ch in servo_channels:
		pca.channels[ch].duty_cycle = 0


def move_motors(movements: Dict[int, float], delay: float = 0.01, speed_multiplier: float = 10):
	"""
	Relative multi-servo move with interpolation.
	movements: {channel: delta_degrees}
	"""
	def clamp_angle(a: float) -> float:
		return max(0.0, min(270.0, a))

	global servo_angles
	if not servo_angles:
		initialize_servo_angles()

	channels = list(movements.keys())
	relative_angles = list(movements.values())

	current_angles = [servo_angles.get(ch, servo_home[ch]) for ch in channels]
	target_angles = [clamp_angle(cur + rel) for cur, rel in zip(current_angles, relative_angles)]

	max_step_count = 0.0
	for t, c in zip(target_angles, current_angles):
		max_step_count = max(max_step_count, abs(t - c) * 10.0)

	adjusted_step_count = max(1, int(round(max_step_count / max(1e-9, speed_multiplier))))

	increments = []
	for t, c in zip(target_angles, current_angles):
		increments.append((t - c) / adjusted_step_count if adjusted_step_count else 0.0)

	current_positions = dict(zip(channels, current_angles))

	for _ in range(adjusted_step_count):
		for i, ch in enumerate(channels):
			inc = increments[i]
			if inc != 0:
				new_angle = clamp_angle(current_positions[ch] + inc)
				current_positions[ch] = new_angle
				servo_angles[ch] = new_angle
				servos[ch].angle = new_angle
		time.sleep(delay)

	for i, ch in enumerate(channels):
		servo_angles[ch] = target_angles[i]
		servos[ch].angle = target_angles[i]


def stand_up():
	"""Home/stand routine (safe neutral)."""
	def set_motor_angles(chs, positions):
		for ch in chs:
			servos[ch].angle = positions[ch]
		time.sleep(0.5)

	target = {ch: servo_home[ch] for ch in servos}

	set_motor_angles([6, 7, 8, 9], target)
	time.sleep(0.2)
	set_motor_angles([5, 4, 0, 1], target)
	set_motor_angles([14, 15, 10, 11], target)

	initialize_servo_angles()
	time.sleep(0.5)
	zero_imu()


# =============================================================================
# IMU (BNO08X) + gravity helper
# =============================================================================

_i2c_imu = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(_i2c_imu, address=0x4B)

_features = [
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_MAGNETOMETER,
	BNO_REPORT_LINEAR_ACCELERATION,
	BNO_REPORT_GRAVITY,
	BNO_REPORT_ROTATION_VECTOR,
	BNO_REPORT_GAME_ROTATION_VECTOR,
	BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
]
for f in _features:
	bno.enable_feature(f)

acc_offset = (0.0, 0.0, 0.0)
gyro_offset = (0.0, 0.0, 0.0)
mag_offset = (0.0, 0.0, 0.0)
linacc_offset = (0.0, 0.0, 0.0)

quat_offset = (1.0, 0.0, 0.0, 0.0)
game_quat_offset = (1.0, 0.0, 0.0, 0.0)
geomag_quat_offset = (1.0, 0.0, 0.0, 0.0)


def quat_inverse(q):
	w, x, y, z = q
	return (w, -x, -y, -z)


def quat_multiply(a, b):
	w1, x1, y1, z1 = a
	w2, x2, y2, z2 = b
	return (
		w1*w2 - x1*x2 - y1*y2 - z1*z2,
		w1*x2 + x1*w2 + y1*z2 - z1*y2,
		w1*y2 - x1*z2 + y1*w2 + z1*x2,
		w1*z2 + x1*y2 - y1*x2 + z1*w2,
	)


def rotate_vector_by_quat(v, q):
	q_conj = quat_inverse(q)
	vx, vy, vz = v
	v_q = (0.0, vx, vy, vz)
	tmp = quat_multiply(q, v_q)
	res = quat_multiply(tmp, q_conj)
	return (res[1], res[2], res[3])


def euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
	r = math.radians(roll_deg)
	p = math.radians(pitch_deg)
	y = math.radians(yaw_deg)
	cr, sr = math.cos(r/2), math.sin(r/2)
	cp, sp = math.cos(p/2), math.sin(p/2)
	cy, sy = math.cos(y/2), math.sin(y/2)
	return (
		cy*cp*cr + sy*sp*sr,
		cy*cp*sr - sy*sp*cr,
		cy*sp*cr + sy*cp*sr,
		sy*cp*cr - cy*sp*sr,
	)


MOUNT_RPY_DEG = (-2.0, -2.0, 0.0)
MOUNT_Q = euler_deg_to_quat(*MOUNT_RPY_DEG)


def zero_imu():
	"""Apply the fixed mount offset (does not re-zero to current pose)."""
	global MOUNT_Q
	MOUNT_Q = euler_deg_to_quat(*MOUNT_RPY_DEG)


def get_gravity():
	"""Returns (gx,gy,gz) in g-units, rotated into robot frame using fixed mount quaternion."""
	g = bno.gravity
	gx, gy, gz = rotate_vector_by_quat(g, MOUNT_Q)
	return (gx/9.80665, gy/9.80665, gz/9.80665)


# =============================================================================
# IK + multi-leg mover (from your main)
# =============================================================================

BODY_LEN = 0.186  # meters
BODY_WID = 0.078  # meters
L1, L2, L3 = 0.055, 0.1075, 0.130

LEG_TRANSFORMS = {
	0: t_leftfront,
	1: t_rightfront,
	2: t_leftback,
	3: t_rightback,
}

MAPPING = {
	0: {1: {'sign': +1, 'offset':-143},    2: {'sign': +1, 'offset': 122+5},    3: {'sign': +1, 'offset': -86+242}},
	1: {1: {'sign': -1, 'offset': 228},    2: {'sign': -1, 'offset': 34+78},    3: {'sign': -1, 'offset': 305-140}},
	2: {1: {'sign': -1, 'offset': 406},    2: {'sign': +1, 'offset': 150},      3: {'sign': +1, 'offset': 162}},
	3: {1: {'sign': +1, 'offset': 35},     2: {'sign': -1, 'offset': 89},       3: {'sign': -1, 'offset': 161}},
}

CHANNEL_MAP = {
	0: {1: 9, 2: 11, 3: 15},
	1: {1: 8, 2: 10, 3: 14},
	2: {1: 6, 2: 4,  3: 0},
	3: {1: 7, 2: 5,  3: 1},
}

INV_LEG: Dict[int, np.ndarray] = {}
ht_body0 = homog_transxyz(0, 0, 0) @ homog_rotxyz(0, 0, 0)
for leg_idx, tf_fn in LEG_TRANSFORMS.items():
	T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
	INV_LEG[leg_idx] = ht_inverse(T)


def to_user_angles(leg_idx: int, theta_rads: Tuple[float, float, float]) -> Tuple[float, float, float]:
	cfg = MAPPING[leg_idx]
	return tuple(
		cfg[j]['sign'] * (theta_rads[j-1] * 180/math.pi) + cfg[j]['offset']
		for j in (1, 2, 3)
	)


def solve_ik_or_project(x_h, y_h, z_h, L1, L2, L3, legs12, *, max_up=2.0, min_down=0.1):
	vx, vy, vz = float(x_h), float(y_h), float(z_h)
	try:
		q1, q2, q3 = ikine(vx, vy, vz, L1, L2, L3, legs12=legs12)
		return q1, q2, q3, (vx, vy, vz), 1.0
	except ValueError:
		pass

	steps = [k * 0.02 for k in range(1, int(max(1, max_up / 0.02)))]
	scales = []
	for d in steps:
		s1 = 1.0 - d
		s2 = 1.0 + d
		if s1 >= min_down:
			scales.append(s1)
		if s2 <= max_up:
			scales.append(s2)

	for s in scales:
		xs, ys, zs = vx*s, vy*s, vz*s
		try:
			q1, q2, q3 = ikine(xs, ys, zs, L1, L2, L3, legs12=legs12)
			if s > 1.0:
				s *= 0.995
				xs, ys, zs = vx*s, vy*s, vz*s
			return q1, q2, q3, (xs, ys, zs), s
		except ValueError:
			continue

	return None, None, None, None, None


def iklegs_move(leg_offsets: Dict[int, Tuple[float, float, float]], step_multiplier=10, speed=10, delay=0.01):
	"""Move multiple legs together along straight-line paths in body-space."""
	def clamp_angle(a: float) -> float:
		return max(0.0, min(270.0, a))

	global servo_angles
	if not servo_angles:
		initialize_servo_angles()

	leg_cfg = {
		0: {'base': ((BODY_LEN/2),  BODY_WID/2,  -0.16), 'scale': ( 3/3.5*5/5.5,   3/3.5,   3/2.5)},
		1: {'base': ((BODY_LEN/2), -BODY_WID/2, -0.16), 'scale': (-3/3.5*5/4,     3/2,     3/3.75)},
		2: {'base': (-(BODY_LEN/2), BODY_WID/2, -0.16), 'scale': ( 3/4,           3/3.5,   3/2.5)},
		3: {'base': (-(BODY_LEN/2),-BODY_WID/2, -0.16), 'scale': (-3/2.5,         3/2,     3/3.75)},
	}

	max_steps = 0.0
	for dx, dy, dz in leg_offsets.values():
		max_steps = max(max_steps,
						abs(dx)*step_multiplier,
						abs(dy)*step_multiplier,
						abs(dz)*step_multiplier)

	steps = max(1, math.ceil(max_steps / max(1e-9, speed)))
	incs = {leg: (dx/steps, dy/steps, dz/steps) for leg, (dx, dy, dz) in leg_offsets.items()}

	for s in range(1, steps+1):
		step_movements: Dict[int, float] = {}

		for leg_idx, (inc_x, inc_y, inc_z) in incs.items():
			ux = inc_x * s
			uy = inc_y * s
			uz = inc_z * s

			cfg = leg_cfg[leg_idx]
			xb = cfg['base'][0] + ux * cfg['scale'][0]
			yb = cfg['base'][1] + uy * cfg['scale'][1]
			zb = cfg['base'][2] + uz * cfg['scale'][2]

			P_body = np.array([xb, yb, zb, 1.0])
			P_hip = INV_LEG[leg_idx] @ P_body

			front = (not (leg_idx in (0, 1))) ^ (leg_idx in (2, 3))
			q = solve_ik_or_project(P_hip[0], P_hip[1], P_hip[2], L1, L2, L3, front)
			if q[0] is None:
				continue
			q1, q2, q3, _, _ = q

			user_deg = to_user_angles(leg_idx, (q1, q2, q3))
			for joint, tgt in enumerate(user_deg, start=1):
				ch = CHANNEL_MAP[leg_idx][joint]
				step_movements[ch] = tgt - servo_angles[ch]

		for ch, delta in step_movements.items():
			new_ang = clamp_angle(servo_angles[ch] + delta)
			servos[ch].angle = new_ang
			servo_angles[ch] = new_ang

		time.sleep(delay)


# =============================================================================
# Emotions (sit/unsit, kneel/unkneel, shake, dance)
# =============================================================================

_motion_lock = threading.Lock()
_is_sitting = False
_is_kneeling = False


def _lcd_msg(msg: str):
	try:
		lcd.lcd(msg)
	except Exception:
		print(f"[LCD] {msg}")


def _lcd_clear():
	try:
		lcd.clear()
	except Exception:
		pass


def sit():
	"""Sit down (toggle state stored internally)."""
	global _is_sitting, _is_kneeling
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		if _is_sitting:
			return
		_is_kneeling = False
		_lcd_msg("Sitting")
		move_motors({0: -40, 4: 15, 5: -15, 1: 40})
		move_motors({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10})
		_is_sitting = True
		_lcd_clear()


def unsit():
	"""Stand up from sit."""
	global _is_sitting
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		if not _is_sitting:
			return
		_lcd_msg("Standing Up")
		move_motors({15: -90, 14: 90, 11: 60, 10: -60, 0: -10, 1: 10})
		move_motors({0: 40, 4: -15, 5: 15, 1: -40})
		_is_sitting = False
		_lcd_clear()


def kneel():
	"""Kneel down (toggle state stored internally)."""
	global _is_kneeling, _is_sitting
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		if _is_kneeling:
			return
		_is_sitting = False
		_lcd_msg("Kneeling")
		move_motors({15: -30, 11: 30, 10: -30, 14: 30})
		_is_kneeling = True
		_lcd_clear()


def unkneel():
	"""Stand up from kneel."""
	global _is_kneeling
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		if not _is_kneeling:
			return
		_lcd_msg("Standing Up")
		move_motors({15: 30, 11: -30, 10: 30, 14: -30})
		_is_kneeling = False
		_lcd_clear()

def dance():
	"""Simple dance loop."""
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		_lcd_msg("Dancing")
		for _ in range(4):
			iklegs_move({0:(0.0,0.0,+0.015),1:(0.0,0.0,-0.015),2:(0.0,0.0,+0.015),3:(0.0,0.0,-0.015)}, step_multiplier=10, speed=0.005, delay=0.0)
			time.sleep(0.3)
			iklegs_move({0:(0.0,0.0,-0.015),1:(0.0,0.0,+0.015),2:(0.0,0.0,-0.015),3:(0.0,0.0,+0.015)}, step_multiplier=10, speed=0.005, delay=0.0)
			time.sleep(0.3)
		iklegs_move({0:(0.0,0.0,0.0,),1:(0.0,0.0,0.0),2:(0.0,0.0,0.0),3:(0.0,0.0,0.0)}, step_multiplier=10, speed=20, delay=0.0)
		_lcd_clear()
		
def shake():
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		_lcd_msg("Shaking")
		move_motors({10: 110, 14: -80}, speed_multiplier=25)
		for _ in range(4):
			move_motors({14: 40}, speed_multiplier=15)
			time.sleep(.15)
			move_motors({14: -40}, speed_multiplier=15)
			time.sleep(.15)
		move_motors({10: -110, 14: 120}, speed_multiplier=25)
		move_motors({14: -40}, speed_multiplier=25)
		_lcd_msg("Sitting")
		
def wave():
	with _motion_lock:
		stop_gait(schedule_inactivity_reset=False)
		_lcd_msg("Waving")
		move_motors({10: 50, 14: 80, 8:-20}, speed_multiplier=25)
		for _ in range(4):
			move_motors({14: 20, 8:40}, speed_multiplier=15)
			time.sleep(.15)
			move_motors({14: -20, 8:-40}, speed_multiplier=15)
			time.sleep(.15)
		move_motors({10: -50, 14: -40, 8:20}, speed_multiplier=25)
		move_motors({14: -40}, speed_multiplier=25)
		_lcd_msg("Sitting")


# =============================================================================
# Headless scheduler (Tk-like after/after_cancel)
# =============================================================================

@dataclass
class _AfterTask:
	due: float
	seq: int
	callback: Callable[[], Any]
	cancelled: bool = False


class _AfterScheduler:
	def __init__(self):
		self._cv = threading.Condition()
		self._heap: list[tuple[float, int, _AfterTask]] = []
		self._seq = 0
		self._alive = True
		self._thread = threading.Thread(target=self._run, daemon=True)
		self._thread.start()

	def winfo_exists(self) -> bool:
		return self._alive

	def after(self, ms: int, callback: Callable[[], Any]) -> _AfterTask:
		due = time.monotonic() + (ms / 1000.0)
		with self._cv:
			self._seq += 1
			task = _AfterTask(due=due, seq=self._seq, callback=callback)
			heapq.heappush(self._heap, (task.due, task.seq, task))
			self._cv.notify()
			return task

	def after_cancel(self, task: _AfterTask):
		with self._cv:
			task.cancelled = True
			self._cv.notify()

	def shutdown(self):
		with self._cv:
			self._alive = False
			self._cv.notify()

	def _run(self):
		while True:
			with self._cv:
				if not self._alive:
					return
				if not self._heap:
					self._cv.wait()
					continue
				due, _, task = self._heap[0]
				now = time.monotonic()
				wait_s = due - now
				if wait_s > 0:
					self._cv.wait(timeout=wait_s)
					continue
				heapq.heappop(self._heap)
				if task.cancelled:
					continue
			try:
				task.callback()
			except Exception as e:
				print(f"[MoveLib] scheduled callback error: {e}")


# =============================================================================
# Gait Engine (same tick/process style as gait_engine_app.py)
# =============================================================================

Vec3 = Tuple[float, float, float]
Offsets = Dict[int, Vec3]


class GaitEngine:
	def __init__(
		self,
		*,
		iklegs_move_fn: Callable[..., Any],
		get_gravity_fn: Callable[[], Tuple[float, float, float]],
		body_len: float,
		body_wid: float,
		tk_window: Any,
		dt_ms: int = 50,
		step_hz: float = 1.15,
		swing_frac: float = 0.24,
		base_step_height: float = 0.045,
		min_step_height: float = 0.015,
		max_step_height: float = 0.055,
		imu_gain: float = 5.0,
		z_soft_limit: float = 0.04,
		height_limit: float = 0.10,
	):
		self._iklegs_move = iklegs_move_fn
		self._get_gravity = get_gravity_fn
		self._BL = float(body_len)
		self._BW = float(body_wid)
		self._win = tk_window

		self.dt_ms = int(dt_ms)
		self.dt = self.dt_ms / 1000.0
		self.step_hz = float(step_hz)
		self.swing_frac = float(swing_frac)

		self.speed_scale = 1.0

		self.height_offset = 0.0
		self.height_limit = float(height_limit)

		self.base_step_height = float(base_step_height)
		self.min_step_h = float(min_step_height)
		self.max_step_h = float(max_step_height)
		self.imu_gain = float(imu_gain)
		self.z_soft_limit = float(z_soft_limit)

		self._active = False
		self._after_id: Optional[_AfterTask] = None
		self.phase = 0.0

		self._prev_roll = None
		self._prev_pitch = None
		self._alpha = 0.07
		self._imu_enabled = True

		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0

		self.com_x = 0.0
		self.com_y = 0.0

		self._leg_xy = {
			0: (+self._BL/2.0, +self._BW/2.0),
			1: (+self._BL/2.0, -self._BW/2.0),
			2: (-self._BL/2.0, +self._BW/2.0),
			3: (-self._BL/2.0, -self._BW/2.0),
		}

		self.x_bias_front = 0.030
		self.x_bias_rear = 0.020
		self._stance_bias = {
			0: (+self.x_bias_front, 0.0, 0.0),
			1: (+self.x_bias_front, 0.0, 0.0),
			2: (-self.x_bias_rear, 0.0, 0.0),
			3: (-self.x_bias_rear, 0.0, 0.0),
		}

		self.set_gait("diagonal")

	@property
	def is_active(self) -> bool:
		return self._active

	def start(self):
		if self._active:
			return
		self._active = True
		self._tick()

	def stop(self):
		self._active = False
		if self._win and self._after_id is not None:
			try:
				self._win.after_cancel(self._after_id)
			except Exception:
				pass
		self._after_id = None

	def set_gait(self, name: str):
		if name not in ("creep", "wave", "diagonal"):
			raise ValueError("gait must be creep/wave/diagonal")
		if name == "diagonal":
			self._phase_off = {0: 0.0, 3: 0.0, 1: 0.5, 2: 0.5}
			return
		order = [0, 3, 1, 2] if name == "creep" else [0, 1, 3, 2]
		self._phase_off = {leg: i/4.0 for i, leg in enumerate(order)}

	def set_velocity(self, vx: float, vy: float, wz: float):
		self.vx, self.vy, self.wz = float(vx), float(vy), float(wz)

	def enable_imu(self, enabled: bool):
		self._imu_enabled = bool(enabled)

	def set_speed_scale(self, s: float):
		self.speed_scale = _clamp(float(s), 0.05, 3.0)

	def set_height_offset(self, z: float):
		self.height_offset = _clamp(float(z), -self.height_limit, self.height_limit)

	def _tick(self):
		if not self._active:
			return

		z_imu = self._compute_imu_dz() if self._imu_enabled else {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}

		if (abs(self.vx) + abs(self.vy) + abs(self.wz)) > 1e-6:
			self.phase = (self.phase + (self.step_hz * self.speed_scale) * self.dt) % 1.0

		leg_offsets: Offsets = {}
		for leg in (0, 1, 2, 3):
			phi = (self.phase + self._phase_off[leg]) % 1.0
			Dx, Dy = self._per_leg_body_displacement_per_cycle(leg)
			dx, dy, dz_traj = self._foot_trajectory(phi, Dx, Dy)

			dz_gait_imu = _clamp(dz_traj + z_imu[leg], -self.z_soft_limit, self.z_soft_limit)
			dz = _clamp(dz_gait_imu + self.height_offset, -self.height_limit, self.height_limit)

			bx, by, _ = self._stance_bias.get(leg, (0.0, 0.0, 0.0))
			leg_offsets[leg] = (dx + bx + self.com_x, dy + by + self.com_y, dz)

		self._iklegs_move(leg_offsets, step_multiplier=1, speed=25, delay=0.0)

		if self._win and self._win.winfo_exists():
			self._after_id = self._win.after(self.dt_ms, self._tick)
		else:
			self._active = False
			self._after_id = None

	def _per_leg_body_displacement_per_cycle(self, leg: int) -> Tuple[float, float]:
		T = 1.0 / max(1e-6, self.step_hz)
		rx, ry = self._leg_xy[leg]
		vx_leg = self.vx - self.wz * ry
		vy_leg = self.vy + self.wz * rx
		Dx = vx_leg * T
		Dy = vy_leg * T
		if abs(Dx) < 1e-6:
			Dx = 0.0
		if abs(Dy) < 1e-6:
			Dy = 0.0
		return Dx, Dy

	def _foot_trajectory(self, phi: float, Dx: float, Dy: float) -> Vec3:
		sfrac = _clamp(self.swing_frac, 0.05, 0.45)
		x_half, y_half = 0.5 * Dx, 0.5 * Dy

		if phi < sfrac:
			s = phi / max(1e-9, sfrac)
			# "eased" XY swing similar to the app (cosine ease)
			u = 0.5 - 0.5 * math.cos(math.pi * s)
			dx = -x_half + Dx * u
			dy = -y_half + Dy * u
			dz = self._step_height_adapt() * math.sin(math.pi * s)
		else:
			t = (phi - sfrac) / max(1e-9, (1.0 - sfrac))
			dx = +x_half - Dx * t
			dy = +y_half - Dy * t
			dz = 0.0
		return (dx, dy, dz)

	def _compute_imu_dz(self) -> Dict[int, float]:
		half_len = self._BL / 2.0
		half_wid = self._BW / 2.0

		gx, gy, gz = self._get_gravity()

		# roll/pitch from gravity; tuned to match your previous app behavior
		roll = math.atan2(-gy, gz)
		pitch = math.atan2(gx, math.hypot(gy, gz))

		if self._prev_roll is None:
			roll_f, pitch_f = roll, pitch
		else:
			a = self._alpha
			roll_f = (1 - a) * self._prev_roll + a * roll
			pitch_f = (1 - a) * self._prev_pitch + a * pitch
		self._prev_roll, self._prev_pitch = roll_f, pitch_f

		delta_fb = half_len * math.tan(pitch_f)
		delta_lr = half_wid * math.tan(roll_f)

		raw = {
			0: -delta_fb + delta_lr,
			1: -delta_fb - delta_lr,
			2: +delta_fb + delta_lr,
			3: +delta_fb - delta_lr,
		}
		return {leg: _clamp(self.imu_gain * v, -self.z_soft_limit, self.z_soft_limit) for leg, v in raw.items()}

	def _step_height_adapt(self) -> float:
		if self._prev_roll is None:
			return self.base_step_height
		tilt = math.hypot(self._prev_roll, self._prev_pitch)
		h = self.base_step_height * (1.0 + 1.5 * min(0.35, abs(tilt)))
		return _clamp(h, self.min_step_h, self.max_step_h)


# =============================================================================
# Gait public interface + inactivity reset (and "stop_gait" for emotions)
# =============================================================================

_ENGINE_LOCK = threading.Lock()
_ENGINE: Optional[GaitEngine] = None
_SCHED: Optional[_AfterScheduler] = None

_INACTIVITY_RESET_S = 1.5
_RESET_TIMER: Optional[threading.Timer] = None
_STATE_LOCK = threading.Lock()
_LAST_ACTIVE_T = 0.0

# Your configured speeds (from your latest "right values" setup)
VX_SPEED = 0.15      # forward/back cardinal magnitude
VY_SPEED = 0.11      # strafe cardinal magnitude
DIAG_SPEED = 0.13    # resultant magnitude for diagonals (EDIT THIS EASILY)
WZ_MAX = 1.00        # max yaw rate magnitude (analog)

LEFT_DEADZONE = 0.5
RIGHT_DEADZONE = 0.5


def _ensure_engine() -> GaitEngine:
	global _ENGINE
	with _ENGINE_LOCK:
		if _ENGINE is not None:
			return _ENGINE
		_ENGINE = GaitEngine(
			iklegs_move_fn=iklegs_move,
			get_gravity_fn=get_gravity,
			body_len=BODY_LEN,
			body_wid=BODY_WID,
			tk_window=None,
		)
		_ENGINE.set_gait("diagonal")
		_ENGINE.enable_imu(False)  # IMPORTANT: IMU compensation disabled for gait (per request)
		return _ENGINE


def _ensure_scheduler(eng: GaitEngine) -> _AfterScheduler:
	global _SCHED
	if _SCHED is None:
		_SCHED = _AfterScheduler()
		eng._win = _SCHED
	return _SCHED


def _shutdown_scheduler():
	global _SCHED
	if _SCHED is not None:
		try:
			_SCHED.shutdown()
		except Exception:
			pass
		_SCHED = None


def _cancel_reset_timer_locked():
	global _RESET_TIMER
	if _RESET_TIMER is not None:
		try:
			_RESET_TIMER.cancel()
		except Exception:
			pass
		_RESET_TIMER = None


def _schedule_reset_timer_locked():
	global _RESET_TIMER
	if _RESET_TIMER is not None:
		return
	t = threading.Timer(_INACTIVITY_RESET_S, _reset_to_neutral_if_still_inactive)
	t.daemon = True
	_RESET_TIMER = t
	t.start()


def _reset_to_neutral_if_still_inactive():
	global _RESET_TIMER
	eng = _ensure_engine()
	now = time.monotonic()
	with _STATE_LOCK:
		_RESET_TIMER = None
		if (now - _LAST_ACTIVE_T) + 1e-6 < _INACTIVITY_RESET_S:
			return
		if eng.is_active:
			return
	# neutral feet reset
	try:
		iklegs_move({0: (0, 0, 0), 1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, 0)},
					step_multiplier=10, speed=20, delay=0.0)
	except Exception as e:
		print(f"[MoveLib] inactivity neutral reset failed: {e}")


def stop_gait(*, schedule_inactivity_reset: bool = True):
	"""
	Stop gait immediately.
	If schedule_inactivity_reset=False, DO NOT schedule the 1.5s neutral reset (use for emotions).
	"""
	global _LAST_ACTIVE_T
	eng = _ensure_engine()

	# stop engine + scheduler
	eng.set_velocity(0.0, 0.0, 0.0)
	if eng.is_active:
		eng.stop()
	_shutdown_scheduler()

	with _STATE_LOCK:
		_cancel_reset_timer_locked()
		if schedule_inactivity_reset:
			# mark "inactive now" and schedule
			_LAST_ACTIVE_T = time.monotonic()
			_schedule_reset_timer_locked()


def gait_command(vx: float, vy: float, wz: float):
	"""
	Nonzero -> start gait scheduler + walk/turn.
	Zero    -> stop scheduler thread; after 1.5s inactivity -> neutral reset.
	"""
	global _LAST_ACTIVE_T

	eng = _ensure_engine()

	vx = _clamp(float(vx), -VX_SPEED, VX_SPEED)
	vy = _clamp(float(vy), -VY_SPEED, VY_SPEED)
	wz = _clamp(float(wz), -WZ_MAX, WZ_MAX)

	active = (abs(vx) + abs(vy) + abs(wz)) > 1e-6
	now = time.monotonic()

	with _STATE_LOCK:
		if active:
			_LAST_ACTIVE_T = now
			_cancel_reset_timer_locked()

	if active:
		eng.set_velocity(vx, vy, wz)
		if not eng.is_active:
			_ensure_scheduler(eng)
			eng.start()
	else:
		# stop and schedule inactivity reset
		stop_gait(schedule_inactivity_reset=True)


def joystick_to_cmd(
	lx: float, ly: float,
	rx: float, ry: float,
	*,
	left_deadzone: float = LEFT_DEADZONE,
	right_deadzone: float = RIGHT_DEADZONE
) -> Tuple[float, float, float]:
	"""
	Left stick: 8-direction snap, ±30° capture, binary motion.
	  - Cardinals: vx=±VX_SPEED, vy=±VY_SPEED (RIGHT => vy NEGATIVE to match your original mapping)
	  - Diagonals: scale components so hypot(vx,vy) == DIAG_SPEED (variable)

	Right stick: analog yaw from rx with deadzone AND vertical fade:
	  - perfectly left/right (ry≈0) => max yaw
	  - as stick goes up/down (|ry|->1) => yaw -> 0
	  - preserves your original sign: rx>0 => wz NEGATIVE
	"""
	lx = float(lx); ly = float(ly)
	rx = float(rx); ry = float(ry)

	# --- Translation (left stick) ---
	vx = 0.0
	vy = 0.0

	mag = math.hypot(lx, ly)
	if mag >= left_deadzone:
		fwd = -ly
		right = lx
		ang = (math.degrees(math.atan2(right, fwd)) + 360.0) % 360.0  # 0=fwd, 90=right

		dirs = [0, 45, 90, 135, 180, 225, 270, 315]
		nearest = min(dirs, key=lambda d: min((ang - d) % 360, (d - ang) % 360))
		diff = min((ang - nearest) % 360, (nearest - ang) % 360)

		if diff <= 30.0:
			want_fwd = nearest in (0, 45, 315)
			want_back = nearest in (180, 135, 225)
			want_right = nearest in (90, 45, 135)
			want_left = nearest in (270, 225, 315)

			diag = ((want_fwd or want_back) and (want_right or want_left))

			if diag:
				base = math.hypot(VX_SPEED, VY_SPEED)
				k = (DIAG_SPEED / base) if base > 1e-9 else 0.0
				sx = VX_SPEED * k
				sy = VY_SPEED * k

				vx = sx if want_fwd else (-sx if want_back else 0.0)
				# IMPORTANT: right => vy NEGATIVE (matches your original mapping)
				if want_right:
					vy = -sy
				elif want_left:
					vy = +sy
				else:
					vy = 0.0
			else:
				if want_fwd:
					vx = +VX_SPEED
				elif want_back:
					vx = -VX_SPEED
				else:
					vx = 0.0

				if want_right:
					vy = -VY_SPEED
				elif want_left:
					vy = +VY_SPEED
				else:
					vy = 0.0

	# --- Rotation (right stick) ---
	wz = 0.0
	ax = abs(rx)
	if ax >= right_deadzone:
		# deadzone-normalized horizontal strength
		nx = (ax - right_deadzone) / max(1e-9, (1.0 - right_deadzone))
		nx = _clamp(nx, 0.0, 1.0)

		# vertical fade: |ry|=0 -> 1.0 ; |ry|=1 -> 0.0
		fade = 1.0 - _clamp(abs(ry), 0.0, 1.0)

		strength = nx * fade

		# preserve original sign convention: rx>0 => wz NEGATIVE
		wz = -WZ_MAX * math.copysign(strength, rx)

	return vx, vy, wz


def shutdown():
	"""Stop gait engine and shut down internal scheduler + timer (safe on exit)."""
	try:
		stop_gait(schedule_inactivity_reset=False)
	except Exception:
		pass
