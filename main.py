import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice
import sys
import tkinter as tk
from math import sqrt, sin, cos, tan, asin, acos, atan, degrees
import math
import numpy as np

# --- IMU setup ---
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
from spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
# import the low-level utilities:
from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
	t_rightback, t_rightfront, t_leftfront, t_leftback, ikine
)

sys.path.insert(0, '/home/Corndog/')
from lcd import lcd_library as lcd

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

OE_PIN = 22
output_enable = OutputDevice(OE_PIN, active_high=False)

# Define the servo channels and positions
servo_channels = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]
servo_home = {0: 39, 1: 231, 4: 222, 5: 50, 6: 128, 7: 130, 8: 133, 9: 135, 10: 73, 11: 194, 14: 240, 15: 37}
TURN_SCALE = 0.2
LEFT_CHANNELS  = {15, 11,  0,  4}  # FL ankle, FL thigh, BL ankle, BL thigh
RIGHT_CHANNELS = {14, 10,  1,  5}  # FR ankle, FR thigh, BR ankle, BR thigh

# Initialize servos on the specified channels
servos = {channel: servo.Servo(pca.channels[channel]) for channel in servo_channels}
for channel in servos:
	servos[channel].set_pulse_width_range(500, 2500)
	servos[channel].actuation_range = 270

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c, address=0x4B)

# Enable all the reports
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

# Offsets for zeroing
q_grav_correction = (1.0, 0.0, 0.0, 0.0)
acc_offset = (0.0, 0.0, 0.0)
gyro_offset = (0.0, 0.0, 0.0)
mag_offset = (0.0, 0.0, 0.0)
linacc_offset = (0.0, 0.0, 0.0)
# we will use quaternions for orientation-based zeroing
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

def quaternion_between_vectors(a, b):
	ax, ay, az = a
	bx, by, bz = b
	# cross product
	cx = ay*bz - az*by
	cy = az*bx - ax*bz
	cz = ax*by - ay*bx
	# dot product
	dot = ax*bx + ay*by + az*bz
	# norms
	na = math.sqrt(ax*ax + ay*ay + az*az)
	nb = math.sqrt(bx*bx + by*by + bz*bz)
	if na == 0 or nb == 0:
		return (1.0, 0.0, 0.0, 0.0)
	# build quaternion (w, x, y, z)
	w = na*nb + dot
	x, y, z = cx, cy, cz
	# handle the 180degrees corner case if vectors are opposite
	if w < 1e-6 * na * nb:
		# pick an arbitrary orthogonal axis
		if abs(ax) > abs(az):
			x, y, z = -ay, ax, 0.0
		else:
			x, y, z = 0.0, -az, ay
		w = 0.0
	# normalize
	norm = math.sqrt(w*w + x*x + y*y + z*z)
	return (w/norm, x/norm, y/norm, z/norm)

def rotate_vector_by_quat(v, q):
	# rotates vector v by quaternion q
	q_conj = quat_inverse(q)
	# represent v as quaternion with zero scalar part
	vx, vy, vz = v
	v_q = (0.0, vx, vy, vz)
	tmp = quat_multiply(q, v_q)
	res = quat_multiply(tmp, q_conj)
	return (res[1], res[2], res[3])

MOUNT_RPY_DEG = (-2.0, -2.0, 0.0)  # (roll, pitch, yaw) of the IMU relative to the robot body, in degrees

def euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
	r = math.radians(roll_deg); p = math.radians(pitch_deg); y = math.radians(yaw_deg)
	cr, sr = math.cos(r/2), math.sin(r/2)
	cp, sp = math.cos(p/2), math.sin(p/2)
	cy, sy = math.cos(y/2), math.sin(y/2)
	# quaternion (w, x, y, z)
	return (
		cy*cp*cr + sy*sp*sr,
		cy*cp*sr - sy*sp*cr,
		cy*sp*cr + sy*cp*sr,
		sy*cp*cr - cy*sp*sr,
	)

MOUNT_Q = euler_deg_to_quat(*MOUNT_RPY_DEG)

def zero_imu():
	"""Repurposed: do NOT zero to current pose; just apply the fixed, known mount offset."""
	global MOUNT_Q
	MOUNT_Q = euler_deg_to_quat(*MOUNT_RPY_DEG)

# Wrapper getters
def get_acceleration():
	x, y, z = bno.acceleration
	ox, oy, oz = acc_offset
	return (x-ox, y-oy, z-oz)

def get_gyroscope():
	x, y, z = bno.gyro
	ox, oy, oz = gyro_offset
	return (x-ox, y-oy, z-oz)

def get_magnetic():
	x, y, z = bno.magnetic
	ox, oy, oz = mag_offset
	return (x-ox, y-oy, z-oz)

def get_linear_acceleration():
	x, y, z = bno.linear_acceleration
	ox, oy, oz = linacc_offset
	return (x-ox, y-oy, z-oz)

def get_quaternion():
	q = bno.quaternion
	inv0 = quat_inverse(quat_offset)
	return quat_multiply(inv0, q)

def get_game_quaternion():
	q = bno.game_quaternion
	inv0 = quat_inverse(game_quat_offset)
	return quat_multiply(inv0, q)

def get_geomagnetic_quaternion():
	q = bno.geomagnetic_quaternion
	inv0 = quat_inverse(geomag_quat_offset)
	return quat_multiply(inv0, q)

def get_gravity():
	# rotate raw gravity into your zeroed frame:
	g = bno.gravity
	gx, gy, gz = rotate_vector_by_quat(g, MOUNT_Q)
	return (gx/9.80665, gy/9.80665, gz/9.80665)

def initialize_servo_angles():
	"""
	Initializes the servo_angles dictionary based on the initial standing positions.
	This function should be called once at the start of the program.
	"""
	servo_standing = servo_home
	global servo_angles
	servo_angles = {channel: angle for channel, angle in servo_standing.items()}

def move_motors(movements, delay=0.01, speed_multiplier=10):
	def clamp_angle(angle):
		return max(0, min(270, angle))  # 0-270

	global servo_angles

	# Extract channels and relative angles
	channels = list(movements.keys())
	relative_angles = list(movements.values())

	# Current angles
	current_angles = [
		servo_angles[channel] if channel in servo_angles
		else servo_standing[channel]
		for channel in channels
	]

	# Target angles (with clamp)
	target_angles = [
		clamp_angle(cur + rel)
		for cur, rel in zip(current_angles, relative_angles)
	]

	# Figure out the maximum steps (float, not int yet)
	max_step_count = max(
		abs(t - c) * 10  # 10 steps per degree
		for t, c in zip(target_angles, current_angles)
	)
	# Convert to int after dividing by speed
	adjusted_step_count = max(1, int(round(max_step_count / speed_multiplier)))

	# Increments
	increments = []
	for t, c in zip(target_angles, current_angles):
		if adjusted_step_count != 0:
			increments.append((t - c) / adjusted_step_count)
		else:
			increments.append(0)

	# Initialize a dict of current positions
	current_positions = dict(zip(channels, current_angles))

	# Move motors in small increments
	for step in range(adjusted_step_count):
		for i, channel in enumerate(channels):
			if increments[i] != 0:
				new_angle = current_positions[channel] + increments[i]
				new_angle = clamp_angle(new_angle)
				current_positions[channel] = new_angle
				servo_angles[channel] = new_angle
				servos[channel].angle = new_angle
		time.sleep(delay)

	# Optional tiny correction step to ensure final angle
	for i, channel in enumerate(channels):
		servo_angles[channel] = target_angles[i]
		servos[channel].angle = target_angles[i]

def stand_up():
	def set_motor_angles(channels, positions):
		for channel in channels:
			servos[channel].angle = positions[channel]
		time.sleep(0.5)  # Wait for all servos in the group to move

	# Define the target positions for all servos
	target_positions = {channel: servo_home[channel] for channel in servos}

	# Initialize sets of motors sequentially
	set_motor_angles([6, 7, 8, 9], target_positions)
	time.sleep(0.2)
	set_motor_angles([5, 4, 0, 1], target_positions)
	set_motor_angles([14, 15, 10, 11], target_positions)
	initialize_servo_angles()
	time.sleep(0.5)
	zero_imu()
	global prev_roll, prev_pitch
	prev_roll = prev_pitch = None

BODY_LEN = 0.186  # m
BODY_WID = 0.078  # m
L1, L2, L3 = 0.055, 0.1075, 0.130

LEG_TRANSFORMS = {
	0: t_leftfront,  1: t_rightfront,
	2: t_leftback,   3: t_rightback,
}

# 2) per-leg sign+offset and channel wiring
MAPPING = {
	0: {1:{'sign':+1,'offset':-143}, 2:{'sign':+1,'offset':  122}, 3:{'sign':+1,'offset': -86+246}},
	1: {1:{'sign':-1,'offset':228},  2:{'sign':-1,'offset':  34+78}, 3:{'sign':-1,'offset': 310-140}},
	2: {1:{'sign':-1,'offset':406},  2:{'sign':+1,'offset':  150}, 3:{'sign':+1,'offset':  162}},
	3: {1:{'sign':+1,'offset':35},   2:{'sign':-1,'offset':  89},  3:{'sign':-1,'offset': 161}},
}

CHANNEL_MAP = {
	0:{1:9,  2:11, 3:15},
	1:{1:8,  2:10, 3:14},
	2:{1:6,  2:4,  3:0 },
	3:{1:7,  2:5,  3:1 },
}

INV_LEG = {}
ht_body0 = homog_transxyz(0,0,0) @ homog_rotxyz(0,0,0)
for leg_idx, tf_fn in LEG_TRANSFORMS.items():
	T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
	INV_LEG[leg_idx] = ht_inverse(T)

def to_user_angles(leg_idx, theta_rads):
	cfg = MAPPING[leg_idx]
	return tuple(
		cfg[j]['sign'] * (theta_rads[j-1] * 180/math.pi) + cfg[j]['offset']
		for j in (1,2,3)
	)

# ---- Nearest-reachable IK wrapper -----------------------------------------
def solve_ik_or_project(x_h, y_h, z_h, L1, L2, L3, legs12, *, max_up=2.0, min_down=0.1):
	"""
	Try IK at (x_h, y_h, z_h). If it fails (unreachable), search for the nearest
	scale factor 's' so that IK(x*s, y*s, z*s) succeeds. The search is symmetric
	around s=1 (1, 0.98, 1.02, 0.96, 1.04, ...), which keeps the foot as close
	as possible to the requested point while staying inside limits.

	Returns: (q1, q2, q3, used_xyz_tuple, used_scale) or (None, None, None, None, None) on failure.
	"""
	vx, vy, vz = float(x_h), float(y_h), float(z_h)

	# First, try the exact target.
	try:
		q1, q2, q3 = ikine(vx, vy, vz, L1, L2, L3, legs12=legs12)
		return q1, q2, q3, (vx, vy, vz), 1.0
	except ValueError:
		pass

	# Build a symmetric scale sequence around 1.0 (closest first).
	steps = [k * 0.02 for k in range(1, int(max(1, max_up / 0.02)))]
	scales = []
	for d in steps:
		s1 = 1.0 - d
		s2 = 1.0 + d
		if s1 >= min_down:
			scales.append(s1)
		if s2 <= max_up:
			scales.append(s2)

	# Try each scaled point, picking the first that works.
	for s in scales:
		xs, ys, zs = vx * s, vy * s, vz * s
		try:
			q1, q2, q3 = ikine(xs, ys, zs, L1, L2, L3, legs12=legs12)
			# Slight inward nudge if we had to expand outward to avoid boundary jitter
			if s > 1.0:
				s *= 0.995
				xs, ys, zs = vx * s, vy * s, vz * s
			return q1, q2, q3, (xs, ys, zs), s
		except ValueError:
			continue

	# Could not find any reachable scale along this ray.
	return None, None, None, None, None

def move_single_leg_body_frame(leg_idx, x_body, y_body, z_body, safety=False, speed=10):
	"""
	Move ONE leg to (x_body, y_body, z_body) in the BODY frame.
	Internally transforms into hip frame, does IK, then calls move_motors.
	"""
	# A) Build the homogeneous body-frame point
	P_body = np.array([x_body, y_body, z_body, 1.0])

	# B) Transform into hip frame via precomputed inverse
	P_hip = INV_LEG[leg_idx] @ P_body
	x_h, y_h, z_h = P_hip[:3]

	# C) IK in hip frame with reach-safe projection
	front = (not (leg_idx in (0,1))) ^ (leg_idx in (2,3))
	q = solve_ik_or_project(x_h, y_h, z_h, L1, L2, L3, front)
	if q[0] is None:
		# Skip this leg this call if truly unreachable along the ray
		return
	q1, q2, q3, (x_used, y_used, z_used), s_used = q

	# D) Convert to user-space servo angles
	user_deg = to_user_angles(leg_idx, (q1, q2, q3))

	# E) Build the channel→delta map
	movements = {}
	for joint, tgt in enumerate(user_deg, start=1):
		ch = CHANNEL_MAP[leg_idx][joint]
		movements[ch] = tgt - servo_angles[ch]

	# F) Either print (safety) or actually move
	if safety:
		print(f"Leg {leg_idx} planned deltas: {movements} (scaled by {s_used:.3f})")
	else:
		move_motors(movements, speed_multiplier=speed)

def iklegs_move(leg_offsets, step_multiplier=10, speed=10, delay=0.01):
	"""
	Move multiple legs together along straight-line paths in body-space.

	leg_offsets: dict mapping leg_idx → (dx_u, dy_u, dz_u)
	  where (0,0,0) means “home” and units are the same as your IKTEST.
	step_multiplier: how many interpolation steps per unit of offset
	speed: higher → fewer total steps
	delay: pause between each micro-step (s)
	"""
	def clamp(a):
		return max(0, min(270, a))

	# per-leg home + scale from IKTEST
	leg_cfg = {
		0: {'base': ( (BODY_LEN/2),  BODY_WID/2,  -0.16),
			'scale': ( 3/3.5*5/5.5,       3/3.5,     3/2.5 )},
		1: {'base': ( (BODY_LEN/2), -BODY_WID/2, -0.16),
			'scale': (-3/3.5*5/4,       3/2,       3/3.75)},
		2: {'base': (-(BODY_LEN/2),  BODY_WID/2,  -0.16),
			'scale': ( 3/4,             3/3.5,     3/2.5 )},
		3: {'base': (-(BODY_LEN/2), -BODY_WID/2,  -0.16),
			'scale': (-3/2.5,          3/2,       3/3.75)},
	}

	# 1) figure out how many micro-steps
	max_steps = 0
	for dx, dy, dz in leg_offsets.values():
		max_steps = max(max_steps,
						abs(dx)*step_multiplier,
						abs(dy)*step_multiplier,
						abs(dz)*step_multiplier)
	
	MIN_STEPS = 1  # ensures smoothness even on tiny moves
	steps = max(MIN_STEPS, math.ceil(max_steps / max(1e-9, speed)))

	# 2) per-leg per-step increments in user-space
	incs = {
		leg: (dx/steps, dy/steps, dz/steps)
		for leg, (dx, dy, dz) in leg_offsets.items()
	}

	# 3) loop through each micro-step
	for s in range(1, steps+1):
		# combined delta-map for all servos this step
		step_movements = {}

		for leg_idx, (inc_x, inc_y, inc_z) in incs.items():
			# current offset in user-space
			ux = inc_x * s
			uy = inc_y * s
			uz = inc_z * s

			cfg = leg_cfg[leg_idx]
			# map into body coords
			xb = cfg['base'][0] + ux * cfg['scale'][0]
			yb = cfg['base'][1] + uy * cfg['scale'][1]
			zb = cfg['base'][2] + uz * cfg['scale'][2]

			# transform → hip, solve IK (reach-safe)
			P_body = np.array([xb, yb, zb, 1.0])
			P_hip  = INV_LEG[leg_idx] @ P_body
			front  = (not (leg_idx in (0,1))) ^ (leg_idx in (2,3))

			q = solve_ik_or_project(P_hip[0], P_hip[1], P_hip[2], L1, L2, L3, front)
			if q[0] is None:
				# Skip this leg for this micro-step if nothing reachable
				continue
			q1, q2, q3, (xh_used, yh_used, zh_used), s_used = q

			# convert to servo angles
			user_deg = to_user_angles(leg_idx, (q1, q2, q3))

			# assemble this leg’s channel→delta
			for joint, tgt in enumerate(user_deg, start=1):
				ch = CHANNEL_MAP[leg_idx][joint]
				step_movements[ch] = tgt - servo_angles[ch]

		# 4) apply all those little deltas in one go
		for ch, delta in step_movements.items():
			new = clamp(servo_angles[ch] + delta)
			servos[ch].angle = new
			servo_angles[ch] = new

		time.sleep(delay)

def enable_servos():
	output_enable.off()
	print("Servos enabled")

def disable_servos():
	lcd.clear()
	output_enable.on()
	for channel in servo_channels:
		pca.channels[channel].duty_cycle = 0  # Set the PWM signal to 0

# Global flag to track walking state
is_walking = False
is_sitting = False
is_kneeling = False
is_handstand = False
is_walking_backward = False
is_turning_right = False
is_turning_left  = False
is_amble = False
is_live_ik   = False
prev_roll, prev_pitch = None, None

# Tkinter GUI
def create_gui():
    walkspeed = 15
    global walk_button, is_walking, sit_button, is_sitting, kneel_button, is_kneeling, is_handstand, is_amble, is_live_ik, prev_roll, prev_pitch
    window = tk.Tk()
    window.title("Robot Control")

    # Prefer full screen (Esc to exit)
    window.attributes("-fullscreen", True)
    window.bind("<Escape>", lambda e: window.attributes("-fullscreen", False))

    # Root frame using grid (so hiding uses grid_remove and keeps slot memory)
    root = tk.Frame(window)
    root.grid(sticky="nsew")
    window.rowconfigure(0, weight=1)
    window.columnconfigure(0, weight=1)
    root.columnconfigure(0, weight=1)

    # Keep references in creation order (main buttons only)
    buttons_order = []  # list of (button, row) for MAIN MENU ONLY
    next_row = [0]      # simple counter in a list to mutate in closure
    context_buttons = []  # sit-context buttons kept separately

    def make_button(text, command, *, include_in_show_all=True):
        r = next_row[0]
        next_row[0] += 1
        b = tk.Button(root, text=text, command=command)
        b.grid(row=r, column=0, sticky="ew", padx=12, pady=6)
        if include_in_show_all:
            buttons_order.append((b, r))
        else:
            context_buttons.append((b, r))
        return b

    # Helpers
    def hide_all_main():
        for b, _ in buttons_order:
            if b.winfo_ismapped():
                b.grid_remove()

    def hide_all_context():
        for b, _ in context_buttons:
            if b.winfo_ismapped():
                b.grid_remove()

    def show_all():
        """Restore the main menu only (never shows context buttons)."""
        for b, _ in buttons_order:
            if not b.winfo_ismapped():
                b.grid()
        # explicitly keep context buttons hidden
        hide_all_context()
        window.update_idletasks()

    def show_only(*widgets):
        """Hide everything, then show only the provided widgets."""
        hide_all_main()
        hide_all_context()
        for w in widgets:
            if not w.winfo_ismapped():
                w.grid()
        window.update_idletasks()

    def hide_others(active_btn):
        """Legacy helper used by movement toggles: hide all except active_btn."""
        for b, _ in buttons_order:
            if b is not active_btn and b.winfo_ismapped():
                b.grid_remove()
        # context buttons also off in these modes
        hide_all_context()
        window.update_idletasks()

    # ------------------- Callbacks -------------------
    def ambletest():
        global is_amble
        if not is_amble:
            is_amble = True
            lcd.lcd("Ambling")
            amble_button.config(text="Stop Ambling")
            window.update()
            hide_others(amble_button)
            testingloop()
        else:
            is_amble = False
            lcd.lcd("Reseting to     Normal")
            iklegs_move({0:(0,0,0), 1:(0,0,0), 2:(0,0,0), 3:(0,0,0)})
            lcd.clear()
            amble_button.config(text="Test Amble")
            show_all()
            window.update()

    def testingloop():
        if not is_amble:
            return
        n=0.3
        h=6
        d=6
        iklegs_move({0:(-0.03+0.04,0,0.01*h), 1:(-0.01+0.04,0,0), 2:(-0.03,0,0), 3:(-0.05,0,0)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(0+0.04,0,0.01*h/2),      1:(-0.02+0.04,0,0), 2:(-0.04,0,0), 3:(-0.06,0,0)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(-0.01+0.04,0,0), 1:(-0.03+0.04,0,0), 2:(-0.05,0,0), 3:(-0.03,0,0.01*h)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(-0.02+0.04,0,0), 1:(-0.04+0.04,0,0), 2:(-0.06,0,0), 3:(0,0,0.01*h/2)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(-0.03+0.04,0,0), 1:(-0.05+0.04,0,0), 2:(-0.03,0,0.01*h), 3:(-0.01,0,0)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(-0.04+0.04,0,0), 1:(-0.06+0.04,0,0), 2:(0,0,0.01*h/2),    3:(-0.02,0,0)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(-0.05+0.04,0,0), 1:(-0.03+0.04,0,0.01*h), 2:(-0.01,0,0), 3:(-0.03,0,0)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        iklegs_move({0:(-0.06+0.04,0,0), 1:(0+0.04,0,0.01*h/2),     2:(-0.02,0,0), 3:(-0.04,0,0)}, step_multiplier=20, delay=0.1)
        time.sleep(n)
        window.after(0, testingloop)

    def walk():
        global is_walking, is_walking_backward
        is_walking_backward = False
        if not is_walking:
            is_walking = True
            lcd.lcd("Walking")
            walk_button.config(text="Stop Walking")
            window.update()
            hide_others(walk_button)
            move_motors({10: -20, 4: 20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
            move_motors({11: -20, 5: 20, 1: -35, 15: 35}, speed_multiplier=walkspeed)
            walking_loop()
        else:
            is_walking = False
            lcd.lcd("Reseting to     Normal")
            move_motors({11: 20, 5: -20, 0: -35, 14: 35}, speed_multiplier=walkspeed)
            move_motors({10: 20, 4: -20, 0: 35, 14: -35}, speed_multiplier=walkspeed)
            lcd.clear()
            walk_button.config(text="Start Walking")
            show_all()
            window.update()

    def walking_loop():
        if not is_walking:
            return
        lcd.lcd("Walking 1/4")
        move_motors({11: 40, 5: -40, 0: -30, 14: 30}, speed_multiplier=walkspeed)
        lcd.lcd("Walking 2/4")
        move_motors({10: 40, 4: -40, 0: 30, 14: -30}, speed_multiplier=walkspeed)
        lcd.lcd("Walking 3/4")
        move_motors({10: -40, 4: 40, 1: 30, 15: -30}, speed_multiplier=walkspeed)
        lcd.lcd("Walking 4/4")
        move_motors({11: -40, 5: 40, 1: -30, 15: 30}, speed_multiplier=walkspeed)
        window.after(0, walking_loop)

    def walk_backwards():
        global is_walking, is_walking_backward
        is_walking = False
        if not is_walking_backward:
            is_walking_backward = True
            lcd.lcd("Backwards")
            back_button.config(text="Stop Backwards")
            window.update()
            hide_others(back_button)
            move_motors({10: -20, 4: 20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
            move_motors({11: -20, 5: 20, 1: -35, 15: 35}, speed_multiplier=walkspeed)
            walking_loop_backwards()
        else:
            is_walking_backward = False
            lcd.lcd("Reseting to     Normal")
            move_motors({11: 20, 5: -20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
            move_motors({10: 20, 4: -20, 1: -35, 15: 35}, speed_multiplier=walkspeed)
            lcd.clear()
            back_button.config(text="Walk Backward")
            show_all()
            window.update()

    def walking_loop_backwards():
        if not is_walking_backward:
            return
        lcd.lcd("Back 1/4")
        move_motors({11:40, 5:-40, 15:-30, 1:30}, speed_multiplier=walkspeed)
        lcd.lcd("Back 2/4")
        move_motors({10: 40,  4:-40,15:30, 1:-30}, speed_multiplier=walkspeed)
        lcd.lcd("Back 3/4")
        move_motors({10: -40, 4:40, 0:-30, 14: 30}, speed_multiplier=walkspeed)
        lcd.lcd("Back 4/4")
        move_motors({11:-40, 5:40, 0:30,  14: -30}, speed_multiplier=walkspeed)
        window.after(0, walking_loop_backwards)

    def turn_left():
        global is_turning_left, is_turning_right, is_walking
        is_turning_right = False
        turn_right_button.config(text="Turn Right")
        is_walking = False
        walk_button.config(text="Start Walking")
        scale_map = lambda ch, v: v * (TURN_SCALE if ch in LEFT_CHANNELS else 1.0)
        if not is_turning_left:
            is_turning_left = True
            lcd.lcd("Turning Left")
            turn_left_button.config(text="Stop Left")
            window.update()
            hide_others(turn_left_button)
            pre1 = { ch: scale_map(ch, v) for ch, v in {10:-20, 4:20, 1:35, 15:-35}.items() }
            pre2 = { ch: scale_map(ch, v) for ch, v in {11:-20, 5:20, 1:-35, 15:35}.items() }
            move_motors(pre1, speed_multiplier=15)
            move_motors(pre2, speed_multiplier=15)
            turning_left_loop()
        else:
            is_turning_left = False
            lcd.lcd("Resetting")
            turn_left_button.config(text="Turn Left")
            window.update()
            post1 = { ch: scale_map(ch, v) for ch, v in {11:20, 5:-20, 0:-35, 14:35}.items() }
            post2 = { ch: scale_map(ch, v) for ch, v in {10:20, 4:-20, 0:35, 14:-35}.items() }
            move_motors(post1, speed_multiplier=15)
            move_motors(post2, speed_multiplier=15)
            lcd.clear()
            show_all()

    def turning_left_loop():
        if not is_turning_left:
            return
        L, R = TURN_SCALE, 1.0
        lcd.lcd("Turning Left 1/4")
        move_motors({11:  40*L, 5: -40*R,  0: -30*L, 14:  30*R}, speed_multiplier=15)
        lcd.lcd("Turning Left 2/4")
        move_motors({10:  40*R, 4: -40*L,  0:  30*L, 14: -30*R}, speed_multiplier=15)
        lcd.lcd("Turning Left 3/4")
        move_motors({10: -40*R, 4:  40*L,  1:  30*R, 15: -30*L}, speed_multiplier=15)
        lcd.lcd("Turning Left 4/4")
        move_motors({11: -40*L, 5:  40*R,  1: -30*R, 15:  30*L}, speed_multiplier=15)
        window.after(0, turning_left_loop)

    def turn_right():
        global is_turning_left, is_turning_right, is_walking
        is_turning_left = False
        turn_left_button.config(text="Turn Left")
        is_walking = False
        walk_button.config(text="Start Walking")
        scale_map = lambda ch, v: v * (TURN_SCALE if ch in RIGHT_CHANNELS else 1.0)
        if not is_turning_right:
            is_turning_right = True
            lcd.lcd("Turning Right")
            turn_right_button.config(text="Stop Right")
            window.update()
            hide_others(turn_right_button)
            pre1 = { ch: scale_map(ch, v) for ch, v in {10:-20, 4:20, 1:35, 15:-35}.items() }
            pre2 = { ch: scale_map(ch, v) for ch, v in {11:-20, 5:20, 1:-35, 15:35}.items() }
            move_motors(pre1, speed_multiplier=15)
            move_motors(pre2, speed_multiplier=15)
            turning_right_loop()
        else:
            is_turning_right = False
            lcd.lcd("Resetting")
            turn_right_button.config(text="Turn Right")
            window.update()
            post1 = { ch: scale_map(ch, v) for ch, v in {11:20, 5:-20, 0:-35, 14:35}.items() }
            post2 = { ch: scale_map(ch, v) for ch, v in {10:20, 4:-20, 0:35, 14:-35}.items() }
            move_motors(post1, speed_multiplier=15)
            move_motors(post2, speed_multiplier=15)
            lcd.clear()
            show_all()

    def turning_right_loop():
        if not is_turning_right:
            return
        R, L = TURN_SCALE, 1.0
        lcd.lcd("Turning Right 1/4")
        move_motors({11:  40*L, 5: -40*R,  0: -30*L, 14:  30*R}, speed_multiplier=15)
        lcd.lcd("Turning Right 2/4")
        move_motors({10:  40*R, 4: -40*L,  0:  30*L, 14: -30*R}, speed_multiplier=15)
        lcd.lcd("Turning Right 3/4")
        move_motors({10: -40*R, 4:  40*L,  1:  30*R, 15: -30*L}, speed_multiplier=15)
        lcd.lcd("Turning Right 4/4")
        move_motors({11: -40*L, 5:  40*R,  1: -30*R, 15:  30*L}, speed_multiplier=15)
        window.after(0, turning_right_loop)

    def kneel():
        global is_kneeling
        if not is_kneeling:
            is_kneeling = True
            lcd.lcd("Kneeling")
            kneel_button.config(text="Unkneel")
            window.update()
            hide_others(kneel_button)
            move_motors({15: -30, 11: 30, 10: -30, 14: 30})
        else:
            is_kneeling = False
            lcd.lcd("Standing Up")
            kneel_button.config(text="Kneel")
            window.update()
            move_motors({15: 30, 11: -30, 10: 30, 14: -30})
            stand_up()
            lcd.clear()
            show_all()

    def handstand():
        global is_handstand
        if not is_handstand:
            is_handstand = True
            lcd.lcd("Handstanding")
            handstand_button.config(text="Back Down")
            window.update()
            hide_others(handstand_button)
            move_motors({0: -40, 4: 10, 5: -10, 1: 40})
            move_motors({15: 85, 11: 0, 10: -0, 14: -85})
            move_motors({0: 50, 1: -50})
        else:
            is_handstand = False
            lcd.lcd("Standing Down")
            handstand_button.config(text="Handstand")
            window.update()
            move_motors({0: -10, 1: 10})
            move_motors({15: -85, 11: -0, 10: 0, 14: 85, 0: -40, 1: 40})
            move_motors({0: 40, 4: -10, 5: 10, 1: -40})
            stand_up()
            lcd.clear()
            show_all()

    # ---- Sit context: Shake + Unsit ----
    def arm_shake():
        move_motors({10: 130, 14: -80}, speed_multiplier=25)
        for _ in range(4):
            move_motors({14: 40}, speed_multiplier=15)
            time.sleep(.15)
            move_motors({14: -40}, speed_multiplier=15)
            time.sleep(.15)
        move_motors({10: -130, 14: 120}, speed_multiplier=25)
        move_motors({14: -40}, speed_multiplier=25)
        # keep sit-context visible after shaking
        show_only(shake_arm_button, unsit_button)

    def unsit():
        global is_sitting
        if is_sitting:
            is_sitting = False
            lcd.lcd("Standing Up")
            move_motors({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10})
            move_motors({0: 40, 4: -15, 5: 15, 1: -40})
            stand_up()
            lcd.clear()
            show_all()

    def sit():
        global is_sitting
        if not is_sitting:
            is_sitting = True
            lcd.lcd("Sitting")
            sit_button.config(text="Sit")  # label stays as Sit
            window.update()
            # take the sitting pose
            move_motors({0: -40, 4: 15, 5: -15, 1: 40})
            move_motors({15:90, 14:-90, 11:-60, 10:60, 0:10, 1:-10})
            # show ONLY the two context buttons (Sit itself is hidden)
            show_only(shake_arm_button, unsit_button)
        else:
            # already sitting → just ensure the context view is active
            show_only(shake_arm_button, unsit_button)

    def dance():
        lcd.lcd("Dancing")
        for _ in range(4):
            move_motors({15: -30, 0: -30, 14: 30, 1: 30})
            move_motors({15: 30, 0: 30, 14: -30, 1: -30})
        lcd.clear()

    def jump():
        lcd.lcd("Charging jump")
        iklegs_move({0:(0,0,0.03),1:(0,0,0.03),2:(0,0,0.03),3:(0,0,0.03)}, step_multiplier=20, speed=0.05)
        time.sleep(1)
        lcd.lcd("Jumping")
        iklegs_move({0:(0,0,-0.03),1:(0,0,-0.03),2:(0,0,-0.03),3:(0,0,-0.03)}, step_multiplier=10, speed=50, delay=0)
        time.sleep(0.2)
        iklegs_move({0:(0,0,0),1:(0,0,0),2:(0,0,0),3:(0,0,0)}, step_multiplier=10, speed=50, delay=0)
        lcd.clear()

    def show_live_imu():
        imu_win = tk.Toplevel(window)
        imu_win.title("Live IMU Data")
        imu_win.geometry("700x180")
        lbl_grav = tk.Label(imu_win, text="", justify="left", font=("Courier", 10))
        lbl_gyro = tk.Label(imu_win, text="", justify="left", font=("Courier", 10))
        lbl_mag  = tk.Label(imu_win, text="", justify="left", font=("Courier", 10))
        lbl_lin  = tk.Label(imu_win, text="", justify="left", font=("Courier", 10))
        lbl_quat = tk.Label(imu_win, text="", justify="left", font=("Courier", 10))
        for lbl in (lbl_grav, lbl_gyro, lbl_mag, lbl_lin, lbl_quat):
            lbl.pack(anchor="w", padx=10, pady=2)
        def update_imu():
            if not imu_win.winfo_exists():
                return
            grav  = get_gravity()
            gyro  = get_gyroscope()
            mag   = get_magnetic()
            lin   = get_linear_acceleration()
            quat  = get_quaternion()
            lbl_grav.config(text=f"Gravity (unit)     : x:{grav[0]:.2f}, y:{grav[1]:.2f}, z:{grav[2]:.2f}")
            lbl_gyro.config(text=f"Gyro (degrees/s)   : {gyro}")
            lbl_mag .config(text=f"Mag (microT)       : {mag}")
            lbl_lin .config(text=f"Lin Accel          : {lin}")
            lbl_quat.config(text=f"Quaternion         : {quat}")
            imu_win.after(100, update_imu)
        update_imu()

    def toggle_live_ik():
        global is_live_ik, prev_roll, prev_pitch
        if not is_live_ik:
            is_live_ik = True
            lcd.lcd("Live IK On")
            prev_roll, prev_pitch = None, None
            liveik_button.config(text="Stop Live IK")
            window.update()
            hide_others(liveik_button)
            live_ik_loop()
        else:
            is_live_ik = False
            lcd.lcd("Live IK Off")
            time.sleep(0.2)
            liveik_button.config(text="Start Live IK")
            lcd.clear()
            show_all()
            window.update()

    def live_ik_loop():
        global prev_roll, prev_pitch, is_live_ik
        half_len = BODY_LEN / 2.0
        half_wid = BODY_WID / 2.0
        alpha_smooth = 0.07
        LIVE_IK_GAIN = 5
        Z_SOFT_LIMIT = 0.04
        if not is_live_ik:
            return
        gx, gy, gz = get_gravity()
        roll  = math.atan2(-gy,       gz)
        pitch = math.atan2(gx, math.hypot(gy, gz))
        if prev_roll is None:
            roll_f, pitch_f = roll, pitch
        else:
            roll_f  = (1 - alpha_smooth) * prev_roll  + alpha_smooth * roll
            pitch_f = (1 - alpha_smooth) * prev_pitch + alpha_smooth * pitch
        prev_roll, prev_pitch = roll_f, pitch_f
        delta_fb = half_len * math.tan(pitch_f)
        delta_lr = half_wid * math.tan(roll_f)
        raw = {
            0: -delta_fb +  delta_lr,
            1: -delta_fb + -delta_lr,
            2: +delta_fb +  delta_lr,
            3: +delta_fb + -delta_lr,
        }
        z_off = {}
        for leg, val in raw.items():
            v = LIVE_IK_GAIN * val
            z_off[leg] = max(-Z_SOFT_LIMIT, min(Z_SOFT_LIMIT, v))
        leg_offsets = {idx: (0.0, 0.0, z_off[idx]) for idx in z_off}
        iklegs_move(leg_offsets, step_multiplier=1, speed=20, delay=0.0)
        window.after(50, live_ik_loop)

    def power_off():
        lcd.lcd("Down")
        move_motors({15: -40, 0: -40, 14: 40, 1: 40})
        disable_servos()
        lcd.clear()

    # --- Buttons (auto-row) -------------------------------------------------
    stand_button       = make_button("Stand Up", stand_up)
    amble_button       = make_button("Test amble", ambletest)
    walk_button        = make_button("Start Walking", walk)
    back_button        = make_button("Walk Backward", walk_backwards)
    turn_right_button  = make_button("Turn Right", turn_right)
    turn_left_button   = make_button("Turn Left",  turn_left)
    sit_button         = make_button("Sit", sit)
    kneel_button       = make_button("Kneel", kneel)
    handstand_button   = make_button("Handstand", handstand)
    # (Removed standalone Shake button)
    dance_button       = make_button("Dance", dance)
    jump_button        = make_button("Jump", jump)
    imulive_button     = make_button("Show Live IMU", show_live_imu)
    liveik_button      = make_button("Start Live IK", toggle_live_ik)

    # Sit-context buttons (NOT part of main menu; exclude from show_all)
    shake_arm_button   = make_button("Shake", arm_shake, include_in_show_all=False)
    unsit_button       = make_button("Unsit", unsit, include_in_show_all=False)

    lie_button         = make_button("Lie Down", power_off)
    # -----------------------------------------------------------------------

    # Hide the Sit-context buttons by default (context-only)
    hide_all_context()

    window.mainloop()



create_gui()
