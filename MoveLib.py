import time
import math
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice
# kinematics imports
from spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics.utilities.transformations import homog_transxyz, homog_rotxyz
from spot_micro_kinematics.utilities.spot_micro_kinematics import (
    t_rightback, t_rightfront, t_leftfront, t_leftback, ikine
)
# optional LCD feedback
import sys
sys.path.insert(0, '/home/Corndog')
from lcd import lcd_library as lcd

# ——— Hardware setup ———
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

OE_PIN = 22
output_enable = OutputDevice(OE_PIN, active_high=False)

SERVO_CHANNELS = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]
SERVO_HOME = {
    0: 40,  1: 230, 4: 225, 5: 45,
    6: 130, 7: 130, 8: 130, 9: 135,
    10: 70, 11: 200,14: 235,15: 40
}

# build servo objects
servos = {}
for ch in SERVO_CHANNELS:
    s = servo.Servo(pca.channels[ch])
    s.set_pulse_width_range(500, 2500)
    s.actuation_range = 270
    servos[ch] = s

# current angles store
servo_angles = {}

def initialize_servo_angles():
    """Initialize servo_angles from SERVO_HOME."""
    global servo_angles
    servo_angles = dict(SERVO_HOME)

# ——— Enable/Disable ———
def enable_servos():
    """Enable all servos."""
    output_enable.off()
    print("Servos enabled")

def disable_servos():
    """Disable all servos (kill PWM)."""
    lcd.clear()
    output_enable.on()
    for ch in SERVO_CHANNELS:
        pca.channels[ch].duty_cycle = 0

# ——— Core motion primitive ———
def move_motors(movements: dict, delay: float = 0.01, speed_multiplier: float = 10):
    """
    movements: {channel: delta_angle}
    delay: per-step pause
    speed_multiplier: larger → faster
    """
    def clamp(a): return max(0, min(270, a))

    chans    = list(movements)
    currents = [servo_angles.get(c, SERVO_HOME[c]) for c in chans]
    targets  = [clamp(curr + movements[c]) for curr, c in zip(currents, chans)]

    max_delta = max(abs(t - c) for t, c in zip(targets, currents))
    steps     = max(1, int(round(max_delta * 10 / speed_multiplier)))
    incs      = [(t - c) / steps for t, c in zip(targets, currents)]
    pos       = dict(zip(chans, currents))

    for _ in range(steps):
        for i, ch in enumerate(chans):
            pos[ch] = clamp(pos[ch] + incs[i])
            servos[ch].angle = pos[ch]
            servo_angles[ch] = pos[ch]
        time.sleep(delay)
    # snap to final
    for ch, tgt in zip(chans, targets):
        servos[ch].angle = tgt
        servo_angles[ch] = tgt

# ——— Basic postures ———
def stand_up():
    """Return to home stance in three groups."""
    def set_group(chs):
        for c in chs:
            servos[c].angle = SERVO_HOME[c]
        time.sleep(0.5)
    groups = [
        [6, 7, 8, 9],
        [5, 4, 0, 1],
        [14, 15, 10, 11]
    ]
    set_group(groups[0])
    time.sleep(0.2)
    set_group(groups[1])
    set_group(groups[2])
    initialize_servo_angles()

def sit():
    lcd.lcd("Sitting")
    move_motors({0: -40, 4: 15, 5: -15, 1: 40})
    move_motors({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10})

def kneel():
    lcd.lcd("Kneeling")
    move_motors({15: -30, 11: 30, 10: -30, 14: 30})

def unsit():
    lcd.lcd("Standing Up")
    move_motors({15: -90, 14: 90, 11: 60, 10: -60, 0: -10, 1: 10})
    move_motors({0: 40, 4: -15, 5: 15, 1: -40})
    stand_up()
    lcd.clear()

def unkeel():
    lcd.lcd("Standing Up")
    move_motors({15: 30, 11: -30, 10: 30, 14: -30})
    stand_up()
    lcd.clear()

# ——— Fun moves ———
def shake():
    lcd.lcd("Shaking")
    move_motors({0:-40, 4:15, 5:-15, 1:40})
    move_motors({15:90, 14:-90,11:-60,10:60,0:10,1:-10})
    time.sleep(0.5)
    move_motors({10:115, 14:-80}, speed_multiplier=25)
    time.sleep(.75)
    for _ in range(4):
        move_motors({14:40}, speed_multiplier=15)
        time.sleep(0.15)
        move_motors({14:-40}, speed_multiplier=15)
        time.sleep(0.15)
    time.sleep(.25)
    move_motors({10:-130, 14:120}, speed_multiplier=25)
    move_motors({14:-40}, speed_multiplier=25)
    move_motors({15:-90,14:90,11:60,10:-60,0:-10,1:10})
    move_motors({0:40,4:-15,5:15,1:-40})
    stand_up()
    lcd.clear()

def dance():
    lcd.lcd("Dancing")
    for _ in range(4):
        move_motors({15:-30, 0:-30, 14:30, 1:30})
        move_motors({15:30, 0:30, 14:-30, 1:-30})
    lcd.clear()

# ——— Walking primitives ———
WALK_SPEED = 15
def walk_init(speed=WALK_SPEED):
    move_motors({10:-20,4:20,1:35,15:-35}, speed_multiplier=speed)
    move_motors({11:-20,5:20,1:-35,15:35}, speed_multiplier=speed)

def walk_cycle(speed=WALK_SPEED):
    move_motors({11:40,5:-40,0:-30,14:30}, speed_multiplier=speed)
    move_motors({10:40,4:-40,0:30,14:-30}, speed_multiplier=speed)
    move_motors({10:-40,4:40,1:30,15:-30}, speed_multiplier=speed)
    move_motors({11:-40,5:40,1:-30,15:30}, speed_multiplier=speed)

def walk_reset(speed=WALK_SPEED):
    move_motors({11:20,5:-20,0:-35,14:35}, speed_multiplier=speed)
    move_motors({10:20,4:-20,0:35,14:-35}, speed_multiplier=speed)

# ——— Turn in place ———
# scale factors for turning
TURN_SCALE    = 0.2
LEFT_CHANNELS = {15, 11, 0, 4}
RIGHT_CHANNELS= {14, 10, 1, 5}
# state flags
is_turning_left  = False
is_turning_right = False

def turn_left(speed=WALK_SPEED):
    """Start/stop a left-spin gait."""
    global is_turning_left, is_turning_right
    # ensure not also turning right
    is_turning_right = False

    scale_map = lambda ch, v: v * (TURN_SCALE if ch in LEFT_CHANNELS else 1.0)

    if not is_turning_left:
        is_turning_left = True
        lcd.lcd("Turning Left")
        # prep moves
        pre1 = {ch: scale_map(ch, v) for ch, v in {10:-20,4:20,1:35,15:-35}.items()}
        pre2 = {ch: scale_map(ch, v) for ch, v in {11:-20,5:20,1:-35,15:35}.items()}
        move_motors(pre1, speed_multiplier=speed)
        move_motors(pre2, speed_multiplier=speed)
        turning_left_loop(speed)
    else:
        is_turning_left = False
        lcd.lcd("Resetting")
        # return moves
        post1 = {ch: scale_map(ch, v) for ch, v in {11:20,5:-20,0:-35,14:35}.items()}
        post2 = {ch: scale_map(ch, v) for ch, v in {10:20,4:-20,0:35,14:-35}.items()}
        move_motors(post1, speed_multiplier=speed)
        move_motors(post2, speed_multiplier=speed)
        lcd.clear()

def turning_left_loop(speed):
    if not is_turning_left:
        return
    L, R = TURN_SCALE, 1.0
    lcd.lcd("TL 1/4")
    move_motors({11:40*L, 5:-40*R, 0:-30*L,14:30*R}, speed_multiplier=speed)
    lcd.lcd("TL 2/4")
    move_motors({10:40*R, 4:-40*L, 0:30*L,14:-30*R}, speed_multiplier=speed)
    lcd.lcd("TL 3/4")
    move_motors({10:-40*R,4:40*L, 1:30*R,15:-30*L}, speed_multiplier=speed)
    lcd.lcd("TL 4/4")
    move_motors({11:-40*L,5:40*R, 1:-30*R,15:30*L}, speed_multiplier=speed)
    # re-invoke
    turning_left_loop(speed)

def turn_right(speed=WALK_SPEED):
    """Start/stop a right-spin gait."""
    global is_turning_left, is_turning_right
    is_turning_left = False

    scale_map = lambda ch, v: v * (TURN_SCALE if ch in RIGHT_CHANNELS else 1.0)

    if not is_turning_right:
        is_turning_right = True
        lcd.lcd("Turning Right")
        pre1 = {ch: scale_map(ch, v) for ch, v in {10:-20,4:20,1:35,15:-35}.items()}
        pre2 = {ch: scale_map(ch, v) for ch, v in {11:-20,5:20,1:-35,15:35}.items()}
        move_motors(pre1, speed_multiplier=speed)
        move_motors(pre2, speed_multiplier=speed)
        turning_right_loop(speed)
    else:
        is_turning_right = False
        lcd.lcd("Resetting")
        post1 = {ch: scale_map(ch, v) for ch, v in {11:20,5:-20,0:-35,14:35}.items()}
        post2 = {ch: scale_map(ch, v) for ch, v in {10:20,4:-20,0:35,14:-35}.items()}
        move_motors(post1, speed_multiplier=speed)
        move_motors(post2, speed_multiplier=speed)
        lcd.clear()

def turning_right_loop(speed):
    if not is_turning_right:
        return
    R, L = TURN_SCALE, 1.0
    lcd.lcd("TR 1/4")
    move_motors({11:40*L,5:-40*R, 0:-30*L,14:30*R}, speed_multiplier=speed)
    lcd.lcd("TR 2/4")
    move_motors({10:40*R,4:-40*L, 0:30*L,14:-30*R}, speed_multiplier=speed)
    lcd.lcd("TR 3/4")
    move_motors({10:-40*R,4:40*L, 1:30*R,15:-30*L}, speed_multiplier=speed)
    lcd.lcd("TR 4/4")
    move_motors({11:-40*L,5:40*R, 1:-30*R,15:30*L}, speed_multiplier=speed)
    turning_right_loop(speed)

# ——— Inverse-kinematics utilities ———
BODY_LEN, BODY_WID = 0.186, 0.078
L1, L2, L3 = 0.055, 0.1075, 0.130
LEG_TRANSFORMS = [t_leftfront, t_rightfront, t_leftback, t_rightback]
MAPPING = {
    0:{1:{'sign':+1,'offset':130}, 2:{'sign':+1,'offset':-15}, 3:{'sign':+1,'offset':40}},
    1:{1:{'sign':-1,'offset':130}, 2:{'sign':-1,'offset':0},   3:{'sign':-1,'offset':0}},
    2:{1:{'sign':-1,'offset':130}, 2:{'sign':+1,'offset':0},   3:{'sign':+1,'offset':180}},
    3:{1:{'sign':+1,'offset':130}, 2:{'sign':-1,'offset':0},   3:{'sign':-1,'offset':0}},
}
SERVO_MAP = {
    (0,1):0,(0,2):1,(0,3):4,
    (1,1):5,(1,2):6,(1,3):7,
    (2,1):8,(2,2):9,(2,3):10,
    (3,1):11,(3,2):14,(3,3):15,
}

def to_user_angles(leg_idx, thetas):
    """Convert raw IK (rad) to servo angles (deg + offset)."""
    out = []
    for j in (1,2,3):
        deg = math.degrees(thetas[j-1])
        cfg = MAPPING[leg_idx][j]
        out.append(cfg['sign'] * deg + cfg['offset'])
    return tuple(out)

def full_ik_move(x, y, z, safety=False, steps=10):
    """Linearly move body from (0,0,0) to (x,y,z)."""
    start = np.zeros(3)
    target = np.array([x, y, z])
    for i, t in enumerate(np.linspace(0, 1, steps)):
        pos = (1 - t) * start + t * target
        sm = SpotMicroStickFigure(x=pos[0], y=pos[1], z=pos[2], phi=0, theta=0, psi=0)
        all_thetas = sm.get_leg_angles()
        cmds = {}
        for li, thetas in enumerate(all_thetas):
            uangs = to_user_angles(li, thetas)
            for ji, ua in enumerate(uangs, start=1):
                ch = SERVO_MAP[(li, ji)]
                cmds[ch] = ua - servo_angles[ch]
        if safety:
            print(f"Step {i+1}/{steps}: {cmds}")
        else:
            move_motors(cmds)

def single_leg_ik_hip_frame(leg_idx, xh, yh, zh):
    """Return (q1,q2,q3) for one leg in its hip frame."""
    ht_body = homog_transxyz(0, 0, 0) @ homog_rotxyz(0, 0, 0)
    t_leg = LEG_TRANSFORMS[leg_idx](ht_body, BODY_LEN, BODY_WID)
    p_body = t_leg @ np.array([xh, yh, zh, 1.0])
    p_local = np.linalg.inv(t_leg) @ p_body
    front = (leg_idx in (0, 1))
    return ikine(p_local[0], p_local[1], p_local[2],
                 L1, L2, L3, legs12=front)

def move_single_leg(leg_idx, x, y, z, safety=False, speed=10):
    """IK-move one leg to (x,y,z) in hip frame."""
    q1, q2, q3 = single_leg_ik_hip_frame(leg_idx, x, y, z)
    uangs = to_user_angles(leg_idx, (q1, q2, q3))
    cmds = {}
    for j, ua in enumerate(uangs, start=1):
        ch = SERVO_MAP[(leg_idx, j)]
        cmds[ch] = ua - servo_angles[ch]
    if safety:
        print(f"Leg {leg_idx} → {cmds}")
    else:
        move_motors(cmds, speed_multiplier=speed)
