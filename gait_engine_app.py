# gait_engine_app.py
# One-file app: GaitEngine + Tk GUI to control it.
# Relies on YOUR runtime providing:
#   - iklegs_move(leg_offsets, step_multiplier=..., speed=..., delay=...)
#   - get_gravity() -> (gx, gy, gz) in g-units (already mount-corrected)
#   - BODY_LEN, BODY_WID
# Optional:
#   - lcd (with .lcd(..) and .clear()) for on-device messages

from __future__ import annotations
import math
import sys
import tkinter as tk
from typing import Callable, Dict, Tuple, Optional, Any

# ---- Resolve dependencies from your runtime ---------------------------------
HARDWARE_MODULE = "main"  # e.g., "my_robot_main" (without .py). Leave None to try __main__ only.


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
		"Could not find required symbols. Make sure this app can access:\n"
		"  iklegs_move, get_gravity, BODY_LEN, BODY_WID (and optionally lcd)\n"
		"Run this after your hardware script, or set HARDWARE_MODULE to your module name."
	)


# ---- Gait Engine -------------------------------------------------------------
Vec3 = Tuple[float, float, float]
Offsets = Dict[int, Vec3]


class GaitEngine:
	"""
	Static gait engine (creep, wave, diagonal) with parametric foot trajectories and IMU-based dz compensation.
	Do NOT run your old 'live_ik_loop' at the same time; this engine already applies IMU tilt offsets.

	Public API:
	  - start(), stop(), is_active
	  - set_gait('creep' | 'wave' | 'diagonal')
	  - set_velocity(vx, vy, wz)   # body-frame (units/s, units/s, rad/s)
	  - set_params(step_hz=..., swing_frac=..., base_step_height=...)
	  - set_speed_scale(s)         # global gait tempo scaler
	  - set_height_offset(z)       # global height offset (dz) for all legs
	  - set_leg_z_scale(leg, scale)# NEW: per-leg Z multiplier
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
		tk_window: tk.Tk,
		dt_ms: int = 50,            # ~20 Hz
		step_hz: float = 1.15,       # cycles/s (slow & stable)
		swing_frac: float = 0.22,   # ≤ 0.25 ⇒ single-leg swing (diagonal will swing 2 legs)
		base_step_height: float = 0.04,
		min_step_height: float = 0.015,
		max_step_height: float = 0.055,
		imu_gain: float = 5.0,
		z_soft_limit: float = 0.04,
		height_limit: float = 0.10,   # absolute clamp for height_offset + gait dz
	):
		# Injected deps
		self._iklegs_move = iklegs_move
		self._get_gravity = get_gravity
		self._BL = float(body_len)
		self._BW = float(body_wid)
		self._win = tk_window

		# Timing / gait params
		self.dt_ms = int(dt_ms)
		self.dt    = self.dt_ms / 1000.0
		self.step_hz = float(step_hz)
		self.swing_frac = float(swing_frac)

		# Global speed scaler (tempo). 1.0 = normal.
		self.speed_scale = 1.0

		# Global height offset applied to all legs (dz)
		self.height_offset = 0.0
		self.height_limit = float(height_limit)

		# NEW: per-leg Z scaling (1.0 = normal, 1.2 = 120% vertical motion)
		self.leg_z_scale = {0: 1.0, 1: 1.0, 2: 1.0, 3: 1.0}

		# Heights & limits
		self.base_step_height = float(base_step_height)
		self.min_step_h = float(min_step_height)
		self.max_step_h = float(max_step_height)
		self.imu_gain = float(imu_gain)
		self.z_soft_limit = float(z_soft_limit)

		# State
		self._active = False
		self._after_id: Optional[str] = None
		self.phase = 0.0
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

		self.set_gait('diagonal')  # default

	# ----- Public controls -----
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
			raise ValueError("gait must be 'creep', 'wave', or 'diagonal'")
		self._gait_name = name

		if name == 'diagonal':
			# Diagonal pairs: (0 FL + 3 RR) together, then (1 FR + 2 RL)
			self._phase_off = {0: 0.0, 3: 0.0, 1: 0.5, 2: 0.5}
			return

		order = [0, 1, 3, 2] if name == 'creep' else [0, 1, 2, 3]
		self._phase_off = {leg: i/4.0 for i, leg in enumerate(order)}

	def set_velocity(self, vx: float, vy: float, wz: float):
		self.vx, self.vy, self.wz = float(vx), float(vy), float(wz)

	def set_params(
		self,
		*,
		step_hz: Optional[float] = None,
		swing_frac: Optional[float] = None,
		base_step_height: Optional[float] = None,
	):
		if step_hz is not None:
			self.step_hz = float(step_hz)
		if swing_frac is not None:
			if not 0.05 <= swing_frac <= 0.60:
				raise ValueError("swing_frac should be between 0.05 and 0.60")
			self.swing_frac = float(swing_frac)
		if base_step_height is not None:
			self.base_step_height = float(base_step_height)

	# global gait tempo scaler (affects phase advance only)
	def set_speed_scale(self, s: float):
		s = float(s)
		self.speed_scale = _clamp(s, 0.05, 3.0)

	# global height offset (dz) for all legs
	def set_height_offset(self, z: float):
		z = float(z)
		self.height_offset = _clamp(z, -self.height_limit, self.height_limit)

	# NEW: per-leg Z scaling (multiplies vertical motion)
	def set_leg_z_scale(self, leg: int, scale: float):
		leg = int(leg)
		scale = float(scale)
		self.leg_z_scale[leg] = _clamp(scale, 0.25, 2.5)

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

	# ----- Core tick -----
	def _tick(self):
		if not self._active:
			return

		# 1) IMU dz (optional)
		z_imu = self._compute_imu_dz() if self._imu_enabled else {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}

		# 2) Advance phase if any motion commanded
		if (abs(self.vx) + abs(self.vy) + abs(self.wz)) > 1e-5:
			self.phase = (self.phase + (self.step_hz * self.speed_scale) * self.dt) % 1.0

		# 3) Leg offsets
		leg_offsets: Offsets = {}
		for leg in (0, 1, 2, 3):
			phi = (self.phase + self._phase_off[leg]) % 1.0
			Dx, Dy = self._per_leg_body_displacement_per_cycle(leg)
			dx, dy, dz_traj = self._foot_trajectory(phi, Dx, Dy)

			# NEW: per-leg Z scaling applied to (gait dz + IMU dz)
			zscale = self.leg_z_scale.get(leg, 1.0)
			dz_scaled = (dz_traj + z_imu[leg]) * zscale

			# Soft clamp scaled (gait+IMU), then add global height offset, then absolute clamp.
			dz_gait_imu = _clamp(dz_scaled, -self.z_soft_limit, self.z_soft_limit)
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
		sfrac = self.swing_frac
		x_half, y_half = 0.5 * Dx, 0.5 * Dy
		if phi < sfrac:
			# Swing: forward reset with smooth x/y easing and cycloidal z
			s = phi / sfrac
			u = 0.5 - 0.5 * math.cos(math.pi * s)  # eased x/y
			dx = -x_half + Dx * u
			dy = -y_half + Dy * u
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


# ---- GUI App ----------------------------------------------------------------
class GaitApp:
	def __init__(self, window: tk.Tk, iklegs_move, get_gravity, BODY_LEN, BODY_WID, lcd=None):
		self.window = window
		self.window.title("Gait Engine Control")
		self.lcd = lcd

		self.engine = GaitEngine(
			iklegs_move=iklegs_move,
			get_gravity=get_gravity,
			body_len=BODY_LEN,
			body_wid=BODY_WID,
			tk_window=self.window,
		)

		# Layout
		self._build_controls()

	# UI builders
	def _build_controls(self):
		panel = tk.LabelFrame(self.window, text="Gait Engine", padx=8, pady=8)
		panel.pack(fill="x", padx=8, pady=8)

		# Start/Stop
		self.btn = tk.Button(panel, text="Start Gait", command=self._toggle)
		self.btn.grid(row=0, column=0, sticky="we", padx=4, pady=4)

		# Gait selector (includes diagonal)
		self.gait_var = tk.StringVar(value="diagonal")
		gait_menu = tk.OptionMenu(panel, self.gait_var, "creep", "wave", "diagonal", command=lambda _: self._on_gait())
		gait_menu.grid(row=0, column=1, sticky="we", padx=4, pady=4)

		# IMU toggle
		self.imu_var = tk.BooleanVar(value=True)
		chk_imu = tk.Checkbutton(panel, text="IMU compensation", variable=self.imu_var, command=self._on_imu)
		chk_imu.grid(row=0, column=2, sticky="w", padx=4, pady=4)

		# Velocity sliders
		self.vx_var = self._mk_scale(panel, "vx", -0.25, 0.25, 0.00, 0, self._on_vx, 0.01, row=1)
		self.vy_var = self._mk_scale(panel, "vy", -0.25, 0.25, 0.00, 1, self._on_vy, 0.01, row=1)
		self.wz_var = self._mk_scale(panel, "wz", -2.5,  2.5,  0.00, 2, self._on_wz,  0.01,  row=1)

		# Params
		self.shz_var = self._mk_param(panel, "step_hz",     0.30, 1.80, self.engine.step_hz,        0.05, 0, self._on_step_hz, row=2)
		self.swg_var = self._mk_param(panel, "swing_frac",  0.10, 0.60, self.engine.swing_frac,     0.01, 1, self._on_swing_frac, row=2)
		self.hgt_var = self._mk_param(panel, "step_height", 0.010, 0.060, self.engine.base_step_height, 0.001, 2, self._on_step_height, row=2)

		# Global speed + height sliders
		global_frame = tk.LabelFrame(panel, text="Global modifiers", padx=4, pady=4)
		global_frame.grid(row=3, column=0, columnspan=3, sticky="we", padx=4, pady=(6, 2))
		self.speed_var = self._mk_scale(global_frame, "speed", 0.10, 2.00, 1.00, 0, self._on_speed_scale, 0.05, row=0)
		self.zoff_var  = self._mk_scale(global_frame, "height", -0.06, 0.06, 0.00, 1, self._on_height_offset, 0.001, row=0)

		# COM offset sliders (global XY bias) — meters
		com_frame = tk.LabelFrame(panel, text="COM offset (global XY bias, m)", padx=4, pady=4)
		com_frame.grid(row=4, column=0, columnspan=3, sticky="we", padx=4, pady=(6, 2))
		self.comx_var = self._mk_scale(com_frame, "com_x", -0.10, 0.10, 0.00, 0, self._on_com_x, 0.01, row=0)
		self.comy_var = self._mk_scale(com_frame, "com_y", -0.05, 0.05, 0.00, 1, self._on_com_y, 0.01, row=0)

		# NEW: Per-leg Z scaling (%)
		zscale_frame = tk.LabelFrame(panel, text="Per-leg Z scale (%)", padx=4, pady=4)
		zscale_frame.grid(row=5, column=0, columnspan=3, sticky="we", padx=4, pady=(6, 2))
		self.fl_z = self._mk_scale(zscale_frame, "FL", 50, 250, 100, 0, lambda v: self._on_leg_z(0, v), 1, row=0)
		self.fr_z = self._mk_scale(zscale_frame, "FR", 50, 250, 100, 1, lambda v: self._on_leg_z(1, v), 1, row=0)
		self.rl_z = self._mk_scale(zscale_frame, "RL", 50, 250, 100, 0, lambda v: self._on_leg_z(2, v), 1, row=1)
		self.rr_z = self._mk_scale(zscale_frame, "RR", 50, 250, 100, 1, lambda v: self._on_leg_z(3, v), 1, row=1)

		# Home button
		btn_home = tk.Button(panel, text="Feet → Home", command=self._home_feet)
		btn_home.grid(row=6, column=0, sticky="we", padx=4, pady=6)

		# Status row
		self.status = tk.Label(self.window, text="Ready", anchor="w")
		self.status.pack(fill="x", padx=8, pady=(0, 8))

	def _mk_scale(self, parent, label, a, b, init, col, cb, res, row: int):
		fr = tk.Frame(parent)
		tk.Label(fr, text=label, width=8, anchor="w").pack(side="left")
		var = tk.DoubleVar(value=init)
		sc = tk.Scale(
			fr, from_=a, to_=b, resolution=res, orient="horizontal", length=220,
			variable=var, command=lambda _=None: cb(var.get())
		)
		sc.pack(side="left")
		fr.grid(row=row, column=col, sticky="w", padx=4, pady=2)
		return var

	def _mk_param(self, parent, label, a, b, init, res, col, cb, row: int):
		fr = tk.Frame(parent)
		tk.Label(fr, text=label, width=10, anchor="w").pack(side="left")
		var = tk.DoubleVar(value=init)
		sc = tk.Scale(
			fr, from_=a, to_=b, resolution=res, orient="horizontal", length=220,
			variable=var, command=lambda _=None: cb(var.get())
		)
		sc.pack(side="left")
		fr.grid(row=row, column=col, sticky="w", padx=4, pady=2)
		return var

	# Button callbacks
	def _toggle(self):
		if self.engine.is_active:
			self.engine.stop()
			self.btn.config(text="Start Gait")
			self._lcd_msg("Gait: Stopped")
			self._set_status("Stopped")
		else:
			self.engine.start()
			self.btn.config(text="Stop Gait")
			self._lcd_msg("Gait: Running")
			self._set_status("Running")

	def _home_feet(self):
		# Stop engine and return feet to shifted home offsets (includes COM bias + height offset)
		self.engine.stop()
		self.btn.config(text="Start Gait")
		self._lcd_msg("Feet: Home")
		iklegs_move = self.engine._iklegs_move
		ox, oy = self.engine.com_x, self.engine.com_y
		oz = self.engine.height_offset
		iklegs_move({0: (ox, oy, oz), 1: (ox, oy, oz), 2: (ox, oy, oz), 3: (ox, oy, oz)},
					step_multiplier=10, speed=20, delay=0.0)
		self._lcd_clear()

	def _on_gait(self):
		try:
			self.engine.set_gait(self.gait_var.get())
		except Exception as e:
			self._set_status(f"Gait error: {e}")

	def _on_imu(self):
		self.engine.enable_imu(self.imu_var.get())

	def _on_vx(self, v): self.engine.set_velocity(float(v), self.engine.vy, self.engine.wz)
	def _on_vy(self, v): self.engine.set_velocity(self.engine.vx, -float(v), self.engine.wz)
	def _on_wz(self, v): self.engine.set_velocity(self.engine.vx, self.engine.vy, -float(v))

	def _on_step_hz(self, v):     self.engine.set_params(step_hz=float(v))
	def _on_swing_frac(self, v):  self.engine.set_params(swing_frac=float(v))
	def _on_step_height(self, v): self.engine.set_params(base_step_height=float(v))

	# global speed + height callbacks
	def _on_speed_scale(self, v):   self.engine.set_speed_scale(float(v))
	def _on_height_offset(self, v): self.engine.set_height_offset(-float(v))

	# COM offset callbacks
	def _on_com_x(self, v): self.engine.set_com_offset(x=-float(v))
	def _on_com_y(self, v): self.engine.set_com_offset(y=-float(v))

	# NEW: per-leg Z scale callback (slider is percent)
	def _on_leg_z(self, leg: int, percent: float):
		scale = float(percent) / 100.0
		self.engine.set_leg_z_scale(leg, scale)

	# Status/LCD helpers
	def _set_status(self, s):
		self.status.config(text=s)

	def _lcd_msg(self, s):
		if self.lcd and hasattr(self.lcd, "lcd"):
			try:
				self.lcd.lcd(s)
			except Exception:
				pass

	def _lcd_clear(self):
		if self.lcd and hasattr(self.lcd, "clear"):
			try:
				self.lcd.clear()
			except Exception:
				pass


# ---- Main entry -------------------------------------------------------------
def main():
	# Pull in required runtime symbols from your environment
	iklegs_move, get_gravity, BODY_LEN, BODY_WID, lcd, provider_env = _resolve_runtime()

	# Try to stand up if the provider env has it (avoid hard import crash)
	if hasattr(provider_env, "stand_up"):
		print("[GaitApp] Standing up to initialize leg positions...")
		try:
			provider_env.stand_up()
		except Exception as e:
			print(f"[GaitApp] stand_up() failed: {e}")

	# Build the app window
	window = tk.Tk()
	_ = GaitApp(window, iklegs_move, get_gravity, BODY_LEN, BODY_WID, lcd=lcd)
	window.mainloop()


if __name__ == "__main__":
	main()
