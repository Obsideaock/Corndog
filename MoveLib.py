# MoveLib.py
# ---------------------------------------------------------------------------
# Corndog MoveLib (minimal): walking + turning ONLY, implemented by directly
# embedding the same GaitEngine math and tick process from gait_engine_app.py.
#
# Public API:
#   - gait_command(vx, vy, wz): set commanded body-frame velocities.
#     * Nonzero cmd -> starts engine (self-ticking at dt_ms like the app)
#     * Zero cmd    -> stops engine immediately; after 1.5s of inactivity,
#                      sends a one-shot "stand" pose.
#
# Optional helper:
#   - joystick_to_cmd(lx, ly, rx, deadzone=0.5) -> (vx, vy, wz)
#
# Dependencies expected in your runtime (same as gait_engine_app.py):
#   - iklegs_move(leg_offsets, step_multiplier=..., speed=..., delay=...)
#   - get_gravity() -> (gx, gy, gz) in g-units (already mount-corrected)
#   - BODY_LEN, BODY_WID
#
# Notes:
#   - Default gait: "diagonal"
#   - IMU compensation: DISABLED by default (per your request)
# ---------------------------------------------------------------------------

from __future__ import annotations

import math
import os
import sys
import time
import threading
import heapq
from dataclasses import dataclass
from typing import Callable, Dict, Tuple, Optional, Any

# ---- Resolve dependencies from your runtime ---------------------------------
# Safest default: try __main__ first, then a module name if provided.
# You can override with environment variable CORNDOG_HARDWARE_MODULE.
HARDWARE_MODULE = os.environ.get("CORNDOG_HARDWARE_MODULE", "main")

Vec3 = Tuple[float, float, float]
Offsets = Dict[int, Vec3]


def _resolve_runtime():
	"""
	Returns:
		iklegs_move, get_gravity, BODY_LEN, BODY_WID, lcd, provider_module
	"""
	envs = []
	try:
		envs.append(sys.modules["__main__"])
	except Exception:
		pass
	if HARDWARE_MODULE:
		try:
			envs.append(__import__(HARDWARE_MODULE))
		except Exception:
			pass

	for env in envs:
		try:
			iklegs = getattr(env, "iklegs_move")
			getgrav = getattr(env, "get_gravity")
			BL = getattr(env, "BODY_LEN")
			BW = getattr(env, "BODY_WID")
			lcd = getattr(env, "lcd", None)  # optional
			return iklegs, getgrav, BL, BW, lcd, env
		except AttributeError:
			continue

	raise RuntimeError(
		"Could not find required symbols. Make sure this module can access:\n"
		"  iklegs_move, get_gravity, BODY_LEN, BODY_WID (and optionally lcd)\n"
		"Run this after your hardware script, or set CORNDOG_HARDWARE_MODULE to your module name."
	)


# ---- Minimal 'tk.after' compatible scheduler (single thread) -----------------
@dataclass
class _AfterTask:
	due: float
	seq: int
	callback: Callable[[], Any]
	cancelled: bool = False


class _AfterScheduler:
	"""Imitates the tiny subset of Tk needed by GaitEngine: after/after_cancel/winfo_exists."""
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
		# thread is daemon; no hard join necessary

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
				# due now
				heapq.heappop(self._heap)
				if task.cancelled:
					continue
			# Run outside lock
			try:
				task.callback()
			except Exception as e:
				# Don't kill scheduler on callback errors
				print(f"[MoveLib] Scheduled callback error: {e}")


# ---- Gait Engine (copied from gait_engine_app.py; GUI removed) --------------
class GaitEngine:
	"""
	Static gait engine (creep, wave, diagonal) with parametric foot trajectories and IMU-based dz compensation.
	Do NOT run your old 'live_ik_loop' at the same time; this engine already applies IMU tilt offsets.

	Public API:
	  - start(), stop(), is_active
	  - set_gait('creep' | 'wave' | 'diagonal')
	  - set_velocity(vx, vy, wz)   # body-frame (units/s, units/s, rad/s)
	  - set_params(step_hz=..., swing_frac=..., base_step_height=...)
	  - set_speed_scale(s)         # NEW: global gait tempo scaler
	  - set_height_offset(z)       # NEW: global height offset (dz) for all legs
	  - enable_imu(bool), set_imu_gain(...), set_z_soft_limit(...)
	  - set_com_offset(x=?, y=?)   # global XY bias for all legs
	"""

	def __init__(
		self,
		*,
		iklegs_move: Callable[..., Any],  # accepts kwargs in your runtime
		get_gravity: Callable[[], Tuple[float, float, float]],
		body_len: float,
		body_wid: float,
		tk_window: Any,
		dt_ms: int = 50,            # ~20 Hz
		step_hz: float = 1.15,
		swing_frac: float = 0.25,   # portion of cycle spent in swing
		base_step_height: float = 0.020,
	):
		self._iklegs_move = iklegs_move
		self._get_gravity = get_gravity
		self._BL = float(body_len)
		self._BW = float(body_wid)
		self._win = tk_window

		self.dt_ms = int(dt_ms)
		self.dt = self.dt_ms / 1000.0

		# Core gait params
		self.step_hz = float(step_hz)
		self.swing_frac = float(swing_frac)
		self.base_step_height = float(base_step_height)

		self.min_step_h = 0.010
		self.max_step_h = 0.040

		# NEW: global tempo scaler
		self.speed_scale = 1.0

		# NEW: global height offset (dz)
		self.height_offset = 0.0
		self.height_limit = 0.040
		self.z_soft_limit = 0.035

		# IMU compensation
		self.imu_gain = 0.022
		self._prev_roll = None
		self._prev_pitch = None
		self._alpha = 0.07
		self._imu_enabled = True

		# Cmd velocities
		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0

		# Global COM offset (applied to all legs)
		self.com_x = 0.0
		self.com_y = 0.0

		# Leg "home" xy (approx) in body frame
		self._leg_xy = {
			0: (+self._BL/2.0, +self._BW/2.0),   # FL
			1: (+self._BL/2.0, -self._BW/2.0),   # FR
			2: (-self._BL/2.0, +self._BW/2.0),   # RL
			3: (-self._BL/2.0, -self._BW/2.0),   # RR
		}

		# Biases
		self.x_bias_front = 0.030  # +30 mm
		self.x_bias_rear  = 0.020  # 20 mm (applied backward via negative sign below)
		self._stance_bias = {
			0: (+self.x_bias_front, 0.0, 0.0),
			1: (+self.x_bias_front, 0.0, 0.0),
			2: (-self.x_bias_rear,  0.0, 0.0),
			3: (-self.x_bias_rear,  0.0, 0.0),
		}

		# Default gait
		self.gait = "diagonal"
		self._phase_off = self._phase_offsets(self.gait)
		self.phase = 0.0

		self._after_id = None
		self._active = False

	def _phase_offsets(self, name: str) -> Dict[int, float]:
		# Returns phase offset in [0..1) per leg
		if name == "creep":
			return {0: 0.0, 3: 0.25, 1: 0.50, 2: 0.75}
		if name == "wave":
			return {0: 0.0, 1: 0.25, 3: 0.50, 2: 0.75}
		# diagonal: FL+RR together, FR+RL together
		return {0: 0.0, 3: 0.0, 1: 0.5, 2: 0.5}

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
		if name not in ('creep', 'wave', 'diagonal'):
			raise ValueError(f"Unknown gait: {name}")
		self.gait = name
		self._phase_off = self._phase_offsets(name)

	def set_velocity(self, vx: float, vy: float, wz: float):
		self.vx = float(vx)
		self.vy = float(vy)
		self.wz = float(wz)

	def set_params(self, *, step_hz: Optional[float] = None, swing_frac: Optional[float] = None, base_step_height: Optional[float] = None):
		if step_hz is not None:
			self.step_hz = float(step_hz)
		if swing_frac is not None:
			self.swing_frac = float(swing_frac)
		if base_step_height is not None:
			self.base_step_height = float(base_step_height)

	# NEW: global gait tempo scaler (affects phase advance only)
	def set_speed_scale(self, s: float):
		s = float(s)
		self.speed_scale = _clamp(s, 0.05, 3.0)

	# NEW: global height offset (dz) for all legs
	def set_height_offset(self, z: float):
		z = float(z)
		self.height_offset = _clamp(z, -self.height_limit, self.height_limit)

	def enable_imu(self, enabled: bool):
		self._imu_enabled = bool(enabled)

	def set_imu_gain(self, gain: float):
		self.imu_gain = float(gain)

	def set_z_soft_limit(self, zmax: float):
		self.z_soft_limit = float(zmax)

	# global XY bias setter
	def set_com_offset(self, x: Optional[float] = None, y: Optional[float] = None):
		if x is not None:
			self.com_x = float(x)
		if y is not None:
			self.com_y = float(y)

	def _tick(self):
		if not self._active:
			return

		# 1) IMU dz (optional)
		z_imu = self._compute_imu_dz() if self._imu_enabled else {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}

		# 2) Advance phase if any motion commanded
		if (abs(self.vx) + abs(self.vy) + abs(self.wz)) > 1e-5:
			# NEW: speed_scale affects phase (tempo) only, so step length math stays consistent
			self.phase = (self.phase + (self.step_hz * self.speed_scale) * self.dt) % 1.0

		# 3) Leg offsets
		leg_offsets: Offsets = {}
		for leg in (0, 1, 2, 3):
			phi = (self.phase + self._phase_off[leg]) % 1.0
			Dx, Dy = self._per_leg_body_displacement_per_cycle(leg)
			dx, dy, dz_traj = self._foot_trajectory(phi, Dx, Dy)

			# Soft clamp gait+IMU, then add global height offset, then absolute clamp.
			dz_gait_imu = _clamp(dz_traj + z_imu[leg], -self.z_soft_limit, self.z_soft_limit)
			dz = _clamp(dz_gait_imu + self.height_offset, -self.height_limit, self.height_limit)

			bx, by, _ = self._stance_bias.get(leg, (0.0, 0.0, 0.0))
			cx, cy = self.com_x, self.com_y
			leg_offsets[leg] = (dx + bx + cx, dy + by + cy, dz)

		# 4) Execute small step (iklegs_move interpolates internally)
		self._iklegs_move(leg_offsets, step_multiplier=1, speed=25, delay=0.0)

		# 5) Reschedule
		if self._win and self._win.winfo_exists():
			self._after_id = self._win.after(self.dt_ms, self._tick)
		else:
			self._active = False
			self._after_id = None

	# ----- Helpers -----
	def _per_leg_body_displacement_per_cycle(self, leg: int) -> Tuple[float, float]:
		# IMPORTANT: use *base* step_hz here (NOT speed_scale) so step length stays consistent
		T = 1.0 / max(1e-6, self.step_hz)

		rx, ry = self._leg_xy[leg]
		vx_leg = self.vx - self.wz * ry
		vy_leg = self.vy + self.wz * rx

		Dx = vx_leg * T
		Dy = vy_leg * T
		if abs(Dx) < 1e-5:
			Dx = 0.0
		if abs(Dy) < 1e-5:
			Dy = 0.0
		return Dx, Dy

	def _foot_trajectory(self, phi: float, Dx: float, Dy: float) -> Vec3:
		# piecewise: swing then stance
		sfrac = _clamp(self.swing_frac, 0.05, 0.45)
		x_half = Dx / 2.0
		y_half = Dy / 2.0

		if phi < sfrac:
			# Swing: move from back to front with a sine lift
			s = phi / sfrac
			dx = -x_half + Dx * s
			dy = -y_half + Dy * s
			dz = self._step_height_adapt() * math.sin(math.pi * s)
		else:
			# Stance: linear back drift (foot "sticks" to ground in world)
			t = (phi - sfrac) / (1.0 - sfrac)
			dx = +x_half - Dx * t
			dy = +y_half - Dy * t
			dz = 0.0
		return (dx, dy, dz)

	def _compute_imu_dz(self) -> Dict[int, float]:
		half_len = self._BL / 2.0
		half_wid = self._BW / 2.0

		gx, gy, gz = self._get_gravity()
		roll = math.atan2(gy, gz)
		pitch = math.atan2(-gx, math.sqrt(gy*gy + gz*gz))

		# low-pass
		if self._prev_roll is None:
			self._prev_roll = roll
			self._prev_pitch = pitch
		else:
			self._prev_roll = (1.0 - self._alpha) * self._prev_roll + self._alpha * roll
			self._prev_pitch = (1.0 - self._alpha) * self._prev_pitch + self._alpha * pitch

		roll = self._prev_roll
		pitch = self._prev_pitch

		# approximate dz at each foot due to tilt (small-angle)
		# z = pitch*x + roll*y, with x forward, y left
		raw = {
			0: (pitch * (+half_len) + roll * (+half_wid)),
			1: (pitch * (+half_len) + roll * (-half_wid)),
			2: (pitch * (-half_len) + roll * (+half_wid)),
			3: (pitch * (-half_len) + roll * (-half_wid)),
		}

		z = {}
		for leg, v in raw.items():
			z[leg] = _clamp(self.imu_gain * v, -self.z_soft_limit, self.z_soft_limit)
		return z

	def _step_height_adapt(self) -> float:
		if self._prev_roll is None:
			return self.base_step_height
		tilt = math.hypot(self._prev_roll, self._prev_pitch)
		k = 1.5
		h = self.base_step_height * (1.0 + k * min(0.35, abs(tilt)))
		return _clamp(h, self.min_step_h, self.max_step_h)


def _clamp(x: float, lo: float, hi: float) -> float:
	return hi if x > hi else lo if x < lo else x


# ---- MoveLib wrapper: command interface + inactivity reset -------------------
_ENGINE_LOCK = threading.Lock()
_ENGINE: Optional[GaitEngine] = None
_SCHED: Optional[_AfterScheduler] = None
_IKLEGS = None  # cached
_GETGRAV = None

# inactivity behavior
_INACTIVITY_RESET_S = 1.5
_RESET_TIMER: Optional[threading.Timer] = None
_LAST_ACTIVE_T = 0.0  # monotonic time
_STATE_LOCK = threading.Lock()


def _ensure_engine() -> GaitEngine:
	global _ENGINE, _SCHED, _IKLEGS, _GETGRAV
	with _ENGINE_LOCK:
		if _ENGINE is not None:
			return _ENGINE

		iklegs, getgrav, BL, BW, _lcd, _env = _resolve_runtime()
		_IKLEGS = iklegs
		_GETGRAV = getgrav

		_SCHED = None
		_ENGINE = GaitEngine(
			iklegs_move=iklegs,
			get_gravity=getgrav,
			body_len=BL,
			body_wid=BW,
			tk_window=None,
			dt_ms=50,         # keep EXACTLY as gait_engine_app default
			step_hz=1.15,
			swing_frac=0.25,
			base_step_height=0.020,
		)
		_ENGINE.set_gait("diagonal")
		_ENGINE.enable_imu(False)   # per request
		return _ENGINE



def _ensure_scheduler(eng: GaitEngine) -> _AfterScheduler:
	"""Ensure the internal 'after' scheduler exists and is attached to the engine."""
	global _SCHED
	if _SCHED is None:
		_SCHED = _AfterScheduler()
		eng._win = _SCHED  # attach scheduler (same interface as Tk)
	return _SCHED


def _shutdown_scheduler():
	"""Shut down the scheduler thread so the gait loop has no background thread when idle."""
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
	"""Starts the one-shot timer that will send stand pose after inactivity."""
	global _RESET_TIMER
	if _RESET_TIMER is not None:
		return
	t = threading.Timer(_INACTIVITY_RESET_S, _reset_to_stand_if_still_inactive)
	t.daemon = True
	_RESET_TIMER = t
	t.start()


def _reset_to_stand_if_still_inactive():
	"""Timer callback: if still inactive, send one stand pose."""
	global _RESET_TIMER
	eng = _ensure_engine()
	now = time.monotonic()

	with _STATE_LOCK:
		_RESET_TIMER = None
		inactive_for = now - _LAST_ACTIVE_T
		# Only reset if we've remained inactive for the full duration and the engine is stopped.
		if inactive_for + 1e-6 < _INACTIVITY_RESET_S:
			return
		if eng.is_active:
			return

	# Send stand pose outside lock
	_send_stand_pose(eng)


def _send_stand_pose(eng: GaitEngine):
	"""
	One-shot "standing" pose using the same math path as the engine:
	set phase so all legs are in stance branch, with Dx=Dy=0.
	"""
	# Ensure commanded velocities are zero
	eng.set_velocity(0.0, 0.0, 0.0)

	# Pick a phase where phi == swing_frac is stance (since condition is phi < swing_frac).
	phase_save = eng.phase
	eng.phase = float(_clamp(eng.swing_frac, 0.05, 0.45))

	z_imu = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}  # IMU disabled anyway
	leg_offsets: Offsets = {}
	for leg in (0, 1, 2, 3):
		phi = (eng.phase + eng._phase_off[leg]) % 1.0
		Dx, Dy = 0.0, 0.0
		dx, dy, dz_traj = eng._foot_trajectory(phi, Dx, Dy)

		dz_gait_imu = _clamp(dz_traj + z_imu[leg], -eng.z_soft_limit, eng.z_soft_limit)
		dz = _clamp(dz_gait_imu + eng.height_offset, -eng.height_limit, eng.height_limit)

		bx, by, _ = eng._stance_bias.get(leg, (0.0, 0.0, 0.0))
		cx, cy = eng.com_x, eng.com_y
		leg_offsets[leg] = (dx + bx + cx, dy + by + cy, dz)

	eng._iklegs_move(leg_offsets, step_multiplier=1, speed=25, delay=0.0)
	eng.phase = phase_save


# ---- Public API --------------------------------------------------------------
def gait_command(vx: float, vy: float, wz: float):
	"""
	Set body-frame velocity command.
	Nonzero -> start engine and walk/turn.
	Zero    -> stop engine immediately; after 1.5s of inactivity send stand pose.
	"""
	global _LAST_ACTIVE_T

	eng = _ensure_engine()

	# Clamp to your requested maxima
	vx = _clamp(float(vx), -0.15, 0.15)
	vy = _clamp(float(vy), -0.11, 0.11)
	wz = _clamp(float(wz), -1, 1)

	active = (abs(vx) + abs(vy) + abs(wz)) > 1e-6
	now = time.monotonic()

	with _STATE_LOCK:
		if active:
			_LAST_ACTIVE_T = now
			_cancel_reset_timer_locked()

	# Apply command + start/stop engine
	if active:
		eng.set_velocity(vx, vy, wz)
		if not eng.is_active:
			_ensure_scheduler(eng)
			eng.start()
	else:
		# stop immediately
		eng.set_velocity(0.0, 0.0, 0.0)
		if eng.is_active:
			eng.stop()
		_shutdown_scheduler()
		with _STATE_LOCK:
			# start inactivity timer; it will send a stand pose if nothing resumes
			_schedule_reset_timer_locked()


def joystick_to_cmd(lx: float, ly: float, rx: float, *, deadzone: float = 0.5) -> Tuple[float, float, float]:
	"""
	Map Steam Deck joystick axes to (vx, vy, wz) using your binary + 8-direction quantization rule.
	- Left stick: translation (vx/vy), snapped to 8 dirs (0/45/90/... degrees)
	- Right stick: rotation (wz) from rx only
	Assumes typical joystick conventions: up is ly=-1, right is lx=+1.
	"""
	lx = float(lx)
	ly = float(ly)
	rx = float(rx)

	# Translation
	vx = 0.0
	vy = 0.0
	mag = math.hypot(lx, ly)
	if mag >= deadzone:
		# Define forward axis as -ly (since up is negative).
		fwd = -ly
		right = lx
		ang = (math.degrees(math.atan2(right, fwd)) + 360.0) % 360.0  # 0=fwd, 90=right

		# Snap to nearest of 8 directions (every 45 degrees).
		dirs = [0, 45, 90, 135, 180, 225, 270, 315]
		nearest = min(dirs, key=lambda d: min((ang - d) % 360, (d - ang) % 360))

		# Only accept if within ±30° of that direction (your spec).
		diff = min((ang - nearest) % 360, (nearest - ang) % 360)
		if diff <= 30.0:
			if nearest in (0, 45, 315):
				vx = 0.15
			elif nearest in (180, 135, 225):
				vx = -0.15
			else:
				vx = 0.0

			if nearest in (90, 45, 135):
				vy = -0.11
			elif nearest in (270, 225, 315):
				vy = 0.11
			else:
				vy = 0.0

	# Rotation
	wz = 0.0
	if rx >= deadzone:
		wz = -1
	elif rx <= -deadzone:
		wz = 1

	return vx, vy, wz


def shutdown():
	"""Stop gait engine and shut down the internal scheduler."""
	global _ENGINE, _SCHED
	eng = _ENGINE
	if eng is not None and eng.is_active:
		try:
			eng.stop()
		except Exception:
			pass
	with _STATE_LOCK:
		_cancel_reset_timer_locked()
	if _SCHED is not None:
		try:
			_SCHED.shutdown()
		except Exception:
			pass
