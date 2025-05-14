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
from spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
# import the low-level utilities:
from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz, ht_inverse
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
	t_rightback, t_rightfront, t_leftfront, t_leftback, ikine
)

sys.path.insert(0, '/home/Corndog')
from lcd import lcd_library as lcd

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

OE_PIN = 22
output_enable = OutputDevice(OE_PIN, active_high=False)

# Define the servo channels and positions
servo_channels = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]
servo_home = {0: 39, 1: 231, 4: 222, 5: 50, 6: 128, 7: 130, 8: 133, 9:135, 10: 73, 11: 194, 14: 240, 15: 37}
TURN_SCALE = 0.2
LEFT_CHANNELS  = {15, 11,  0,  4}  # FL ankle, FL thigh, BL ankle, BL thigh
RIGHT_CHANNELS = {14, 10,  1,  5}  # FR ankle, FR thigh, BR ankle, BR thigh

# Initialize servos on the specified channels
servos = {channel: servo.Servo(pca.channels[channel]) for channel in servo_channels}
for channel in servos:
	servos[channel].set_pulse_width_range(500, 2500)
	servos[channel].actuation_range = 270


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
	# (only if you want to be EXACT at the end, but carefully)
	for i, channel in enumerate(channels):
		# Snap exactly to target angle
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

BODY_LEN = 0.186  # m
BODY_WID = 0.078  # m
L1, L2, L3 = 0.055, 0.1075, 0.130

LEG_TRANSFORMS = {
	0: t_leftfront,  1: t_rightfront,
	2: t_leftback,   3: t_rightback,
}

# 2) your per‐leg sign+offset and channel wiring (fill signs manually)
MAPPING = {
	0: {1:{'sign':+1,'offset':-143}, 2:{'sign':+1,'offset':  122}, 3:{'sign':+1,'offset': -86+246}},
	1: {1:{'sign':-1,'offset':228}, 2:{'sign':-1,'offset':  34+78}, 3:{'sign':-1,'offset': 310-140}},
	2: {1:{'sign':-1,'offset':406}, 2:{'sign':+1,'offset':  150}, 3:{'sign':+1,'offset':  162}},
	3: {1:{'sign':+1,'offset':35}, 2:{'sign':-1,'offset':  89}, 3:{'sign':-1,'offset': 161}},
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
	# T_body→hip
	T = tf_fn(ht_body0, BODY_LEN, BODY_WID)
	INV_LEG[leg_idx] = ht_inverse(T)

# reuse your existing move_motors() and servo_angles dict here…

def to_user_angles(leg_idx, theta_rads):
	cfg = MAPPING[leg_idx]
	return tuple(
		cfg[j]['sign'] * (theta_rads[j-1] * 180/math.pi) + cfg[j]['offset']
		for j in (1,2,3)
	)

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
	
	# C) IK in hip frame
	front = (not (leg_idx in (0,1))) ^ (leg_idx in (2,3))
	q1, q2, q3 = ikine(x_h, y_h, z_h, L1, L2, L3, legs12=front)
	
	# D) Convert to user‐space servo angles
	user_deg = to_user_angles(leg_idx, (q1, q2, q3))
	
	# E) Build the channel→delta map
	movements = {}
	for joint, tgt in enumerate(user_deg, start=1):
		ch = CHANNEL_MAP[leg_idx][joint]
		movements[ch] = tgt - servo_angles[ch]
	
	# F) Either print (safety) or actually move
	if safety:
		print(f"Leg {leg_idx} planned deltas: {movements}")
	else:
		move_motors(movements, speed_multiplier=speed)
		
def iklegs_move(leg_offsets, step_multiplier=10, speed=10, delay=0.01):
    """
    Move multiple legs together along straight‐line paths in body‐space.
    
    leg_offsets: dict mapping leg_idx → (dx_u, dy_u, dz_u)
      where (0,0,0) means “home” and units are the same as your IKTEST.
    step_multiplier: how many interpolation steps per unit of offset
    speed: higher → fewer total steps
    delay: pause between each micro‐step (s)
    """
    # your per‐leg home + scale from IKTEST
    leg_cfg = {
        0: {'base': ( BODY_LEN/2,  BODY_WID/2,  -0.16),
            'scale': ( 3/3.5,       3/3.5,     3/2.5 )},
        1: {'base': ( BODY_LEN/2, -BODY_WID/2, -0.16),
            'scale': (-3/3.5,       3/2,       3/3.75)},
        2: {'base': (-BODY_LEN/2,  BODY_WID/2,  -0.16),
            'scale': ( 3/2.5,       3/3.5,     3/2.5 )},
        3: {'base': (-BODY_LEN/2, -BODY_WID/2,  -0.16),
            'scale': (-3/2.5,       3/2,       3/3.75)},
    }

    def clamp(a):  
        return max(0, min(270, a))

    # 1) figure out how many micro‐steps
    max_steps = 0
    for dx, dy, dz in leg_offsets.values():
        max_steps = max(max_steps,
                        abs(dx)*step_multiplier,
                        abs(dy)*step_multiplier,
                        abs(dz)*step_multiplier)
    steps = max(1, int(round(max_steps/speed)))

    # 2) per‐leg per‐step increments in user‐space
    incs = {
        leg: (dx/steps, dy/steps, dz/steps)
        for leg, (dx, dy, dz) in leg_offsets.items()
    }

    # 3) loop through each micro‐step
    for s in range(1, steps+1):
        # build a combined delta‐map for all servos this step
        step_movements = {}

        for leg_idx, (inc_x, inc_y, inc_z) in incs.items():
            # current offset in user‐space
            ux = inc_x * s
            uy = inc_y * s
            uz = inc_z * s

            cfg = leg_cfg[leg_idx]
            # map into body coords
            xb = cfg['base'][0] + ux * cfg['scale'][0]
            yb = cfg['base'][1] + uy * cfg['scale'][1]
            zb = cfg['base'][2] + uz * cfg['scale'][2]

            # transform → hip, solve IK
            P_body = np.array([xb, yb, zb, 1.0])
            P_hip  = INV_LEG[leg_idx] @ P_body
            front = (not (leg_idx in (0,1))) ^ (leg_idx in (2,3))
            q1, q2, q3 = ikine(P_hip[0], P_hip[1], P_hip[2],
                               L1, L2, L3, legs12=front)

            # convert to servo angles
            user_deg = to_user_angles(leg_idx, (q1, q2, q3))

            # assemble this leg’s channel→delta
            for joint, tgt in enumerate(user_deg, start=1):
                ch = CHANNEL_MAP[leg_idx][joint]
                step_movements[ch] = tgt - servo_angles[ch]

        # 4) apply all those little deltas in one go
        for ch, delta in step_movements.items():
            new = clamp(servo_angles[ch] + delta)
            servos[ch].angle     = new
            servo_angles[ch]      = new

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

# Tkinter GUI
def create_gui():
	walkspeed = 15
	global walk_button, is_walking, sit_button, is_sitting, kneel_button, is_kneeling, is_handstand  # Declare walk_button and is_walking as global so it can be accessed in the walk function
	window = tk.Tk()
	window.title("Robot Control")

	def walk():
		global is_walking, is_walking_backward

		# If we’re in backward mode, shut that down first
		is_walking_backward = False

		if not is_walking:
			# Start forward walking
			is_walking = True
			lcd.lcd("Walking")
			walk_button.config(text="Stop Walking")
			window.update()

			move_motors({10: -20, 4: 20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
			move_motors({11: -20, 5: 20, 1: -35, 15: 35}, speed_multiplier=walkspeed)

			walking_loop()
		else:
			# Stop forward walking
			is_walking = False
			lcd.lcd("Reseting to     Normal")

			move_motors({11: 20, 5: -20, 0: -35, 14: 35}, speed_multiplier=walkspeed)
			move_motors({10: 20, 4: -20, 0: 35, 14: -35}, speed_multiplier=walkspeed)

			lcd.clear()
			walk_button.config(text="Start Walking")
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

		# If we’re in forward mode, shut that down first
		is_walking = False

		if not is_walking_backward:
			# Start backward walking
			is_walking_backward = True
			lcd.lcd("Backwards")
			walk_button.config(text="Stop Backwards")
			window.update()

			# Reverse the two-step start sequence
			move_motors({10: -20, 4: 20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
			move_motors({11: -20, 5: 20, 1: -35, 15: 35}, speed_multiplier=walkspeed)

			walking_loop_backwards()
		else:
			# Stop backward walking
			is_walking_backward = False
			lcd.lcd("Reseting to     Normal")

			# Reverse the reset sequence
			move_motors({11: 20, 5: -20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
			move_motors({10: 20, 4: -20, 1: -35, 15: 35}, speed_multiplier=walkspeed)

			lcd.clear()
			walk_button.config(text="Start Backwards")
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

			# PRE-LOOP OFFSETS (scaled on left side)
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

			# POST-LOOP RETURN (also scaled on left side)
			post1 = { ch: scale_map(ch, v) for ch, v in {11:20, 5:-20, 0:-35, 14:35}.items() }
			post2 = { ch: scale_map(ch, v) for ch, v in {10:20, 4:-20, 0:35, 14:-35}.items() }
			move_motors(post1, speed_multiplier=15)
			move_motors(post2, speed_multiplier=15)

			lcd.clear()

	def turning_left_loop():
		if not is_turning_left:
			return

		# scales: left side small, right side full
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

			# PRE-LOOP OFFSETS (scaled on right side)
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

			# POST-LOOP RETURN (scaled on right side)
			post1 = { ch: scale_map(ch, v) for ch, v in {11:20, 5:-20, 0:-35, 14:35}.items() }
			post2 = { ch: scale_map(ch, v) for ch, v in {10:20, 4:-20, 0:35, 14:-35}.items() }
			move_motors(post1, speed_multiplier=15)
			move_motors(post2, speed_multiplier=15)

			lcd.clear()

	def turning_right_loop():
		if not is_turning_right:
			return

		# scales: right side small, left side full
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
			# Start sitting
			is_kneeling = True
			lcd.lcd("Kneeling")
			kneel_button.config(text="Unkneel")  # Update button text to "Stand Up"
			window.update()
			# Perform sitting motion
			move_motors({15: -30, 11: 30, 10: -30, 14: 30})

		else:
			# Stand back up
			is_kneeling = False
			lcd.lcd("Standing Up")
			kneel_button.config(text="Kneel")  # Update button text back to "Sit"
			window.update()
			# Perform standing motion (reverse of sitting)
			move_motors({15: 30, 11: -30, 10: 30, 14: -30})
			stand_up()
			lcd.clear()

	def handstand():
		global is_handstand

		if not is_handstand:
			# Start sitting
			is_handstand = True
			lcd.lcd("Handstanding")
			handstand_button.config(text="Back Down")  # Update button text to "Stand Up"
			window.update()
			# Perform sitting motion
			move_motors({0: -40, 4: 10, 5: -10, 1: 40})
			move_motors({15: 85, 11: 0, 10: -0, 14: -85})
			move_motors({0: 50, 1: -50})

		else:
			# Stand back up
			is_handstand = False
			lcd.lcd("Standing Down")
			handstand_button.config(text="Handstand")  # Update button text back to "Sit"
			window.update()
			# Perform standing motion (reverse of sitting)
			move_motors({0: -10, 1: 10})
			move_motors({15: -85, 11: -0, 10: 0, 14: 85, 0: -40, 1: 40})
			move_motors({0: 40, 4: -10, 5: 10, 1: -40})
			stand_up()
			lcd.clear()
	def sit():
		global is_sitting

		if not is_sitting:
			# Start sitting
			is_sitting = True
			lcd.lcd("Sitting")
			sit_button.config(text="Stand Up")  # Update button text to "Stand Up"
			window.update()
			# Perform sitting motion
			move_motors({0: -40, 4: 15, 5: -15, 1: 40})
			move_motors({15:90, 14:-90, 11:-60, 10:60, 0:10, 1:-10})

		else:
			# Stand back up
			is_sitting = False
			lcd.lcd("Standing Up")
			sit_button.config(text="Sit")  # Update button text back to "Sit"
			window.update()
			# Perform standing motion (reverse of sitting)
			move_motors({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10})
			move_motors({0: 40, 4: -15, 5: 15, 1: -40})
			stand_up()
			lcd.clear()

	def shake():
		lcd.lcd("Shaking")
		move_motors({0: -40, 4: 15, 5: -15, 1: 40})
		move_motors({15:90, 14:-90, 11:-60, 10:60, 0:10, 1:-10})
		time.sleep(0.5)
		move_motors({10: 130, 14: -80}, speed_multiplier=25)
		time.sleep(.75)
		for _ in range(4):
			move_motors({14: 40}, speed_multiplier=15)
			time.sleep(0.15)
			move_motors({14: -40}, speed_multiplier=15)
			time.sleep(0.15)
		time.sleep(.25)
		move_motors({10: -130, 14: 120}, speed_multiplier=25)
		move_motors({14: -40}, speed_multiplier=25)
		move_motors({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10})
		move_motors({0: 40, 4: -15, 5: 15, 1: -40})
		stand_up()
		lcd.clear()

	def dance():
		lcd.lcd("Dancing")
		for _ in range(4):
			move_motors({15: -30, 0: -30, 14: 30, 1: 30})
			move_motors({15: 30, 0: 30, 14: -30, 1: -30})
		lcd.clear()

	def jump():
		lcd.lcd("Charging jump")
		move_motors({0: -40, 4: 30, 5: -30, 1: 40, 15: -30, 11: 30, 14: 30, 10: -30})
		lcd.lcd("Jumping")
		move_motors({0: 50, 4: -30, 5: 30, 1: -50, 15: 50, 11: -20, 14: -50, 10: 20}, delay=0.00001, speed_multiplier=1000)
		time.sleep(0.2)
		move_motors({0: -50, 4: 30, 5: -30, 1: 50, 15: -50, 11: 20, 14: 50, 10: -20}, delay=0.00001, speed_multiplier=1000)
		time.sleep(0.2)
		move_motors({0: 40, 4: -30, 5: 30, 1: -40, 15: 30, 11: -30, 14: -30, 10: 30})
		stand_up()
		lcd.clear()

	def IKTEST():
		n=0.1
		while True:
			iklegs_move({0:(0,0,0), 1:(0,0,0), 2:(0,0,0), 3:(0,0,0)}, delay=0.1)
			time.sleep(n)
			iklegs_move({0:(0,0,0.03), 1:(0,0,0.03), 2:(0,0,0.03), 3:(0,0,0.03)}, delay=0.1)
			time.sleep(n)
			iklegs_move({0:(0.03,0,0.03), 1:(0.03,0,0.03), 2:(0.03,0,0.03), 3:(0.03,0,0.03)}, delay=0.1)
			time.sleep(n)
			iklegs_move({0:(0.03,0,0), 1:(0.03,0,0), 2:(0.03,0,0), 3:(0.03,0,0)}, delay=0.1)
			time.sleep(n)
			
			
	def power_off():
		lcd.lcd("Down")
		move_motors({15: -40, 0: -40, 14: 40, 1: 40})
		disable_servos()
		lcd.clear()

	tk.Button(window, text="Stand Up", command=stand_up).pack()
	tk.Button(window, text="IK Test", command=IKTEST).pack()

	# Create the walk button with an initial label
	walk_button = tk.Button(window, text="Start Walking", command=walk)
	walk_button.pack()
	
	back_button = tk.Button(window, text="Walk Backward", command=walk_backwards)
	back_button.pack()
	
	turn_right_button = tk.Button(window, text="Turn Right", command=turn_right)
	turn_right_button.pack()

	turn_left_button  = tk.Button(window, text="Turn Left",  command=turn_left)
	turn_left_button.pack()

	# Create the sit button with an initial label
	sit_button = tk.Button(window, text="Sit", command=sit)
	sit_button.pack()

	kneel_button = tk.Button(window, text="Kneel", command=kneel)
	kneel_button.pack()

	handstand_button = tk.Button(window, text="Handstand", command=handstand)
	handstand_button.pack()

	tk.Button(window, text="Shake", command=shake).pack()
	tk.Button(window, text="Dance", command=dance).pack()
	tk.Button(window, text="Jump", command=jump).pack()
	tk.Button(window, text="Lie Down", command=power_off).pack()

	window.mainloop()

# Run the GUI
create_gui()
