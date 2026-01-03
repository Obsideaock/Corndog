import time
import math
import numpy as np
import threading
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

# --- Gait engine import (do not modify gait_engine_app.py) ---
from gait_engine_app import GaitEngine


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
    move_motors({10:130, 14:-80}, speed_multiplier=25)
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


# =============================================================================
#   GAIT ENGINE INTEGRATION  (replaces ONLY the hand-scripted gait system)
# =============================================================================

# Simple IMU stub: keep gait engine happy (no tilt compensation).
def get_gravity():
    # gx, gy, gz in g-units; default "upright"
    return (0.0, 0.0, 1.0)

# Home foot positions in each leg's IK "hip frame" (meters).
# These are reasonable defaults; gait_engine_app provides only *offsets* from home.
_GAIT_HOME_FOOT = {
    0: ( 0.060,  0.050, -0.160),  # FL
    1: ( 0.060, -0.050, -0.160),  # FR
    2: (-0.060,  0.050, -0.160),  # RL
    3: (-0.060, -0.050, -0.160),  # RR
}

def iklegs_move(leg_offsets, step_multiplier=1, speed=25, delay=0.0):
    """
    Adapter required by gait_engine_app.py.

    leg_offsets: {leg_idx: (dx, dy, dz)} meters, applied to each leg's home foot position.
    step_multiplier: larger => smoother/slower (we reduce speed_multiplier accordingly)
    speed: passed through to move_motors speed_multiplier (after scaling)
    delay: per-step delay passed to move_motors
    """
    cmds = {}

    for leg in (0, 1, 2, 3):
        dx, dy, dz = leg_offsets.get(leg, (0.0, 0.0, 0.0))
        hx, hy, hz = _GAIT_HOME_FOOT[leg]
        x = hx + float(dx)
        y = hy + float(dy)
        z = hz + float(dz)

        q1, q2, q3 = single_leg_ik_hip_frame(leg, x, y, z)
        uangs = to_user_angles(leg, (q1, q2, q3))

        for j, ua in enumerate(uangs, start=1):
            ch = SERVO_MAP[(leg, j)]
            curr = servo_angles.get(ch, SERVO_HOME[ch])
            cmds[ch] = ua - curr

    # Interpret step_multiplier as "more smoothing": lower effective speed => more internal steps
    sm = max(1, int(step_multiplier))
    eff_speed = max(0.5, float(speed) / sm)

    if cmds:
        move_motors(cmds, delay=float(delay), speed_multiplier=eff_speed)

class _AfterScheduler:
    """
    Minimal scheduler with the subset of Tk API that GaitEngine uses:
      - after(ms, callback) -> id
      - after_cancel(id)
      - winfo_exists()
    Implemented with threading.Timer so we can run without Tk mainloop.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._timers = {}
        self._next_id = 1
        self._alive = True

    def after(self, ms, callback):
        with self._lock:
            if not self._alive:
                return "0"
            tid = str(self._next_id)
            self._next_id += 1

        def _wrapped():
            # Remove handle before running (matches typical after semantics)
            with self._lock:
                self._timers.pop(tid, None)
            try:
                callback()
            except Exception:
                pass

        t = threading.Timer(ms / 1000.0, _wrapped)
        t.daemon = True
        with self._lock:
            self._timers[tid] = t
        t.start()
        return tid

    def after_cancel(self, tid):
        with self._lock:
            t = self._timers.pop(str(tid), None)
        if t:
            try:
                t.cancel()
            except Exception:
                pass

    def winfo_exists(self):
        return self._alive

    def close(self):
        with self._lock:
            self._alive = False
            timers = list(self._timers.values())
            self._timers.clear()
        for t in timers:
            try:
                t.cancel()
            except Exception:
                pass

_gait_engine = None
_gait_sched = None
_gait_lock = threading.Lock()

# -----------------------------
# Gait preset (match GUI)
# -----------------------------
_GAIT_PRESET = {
    "gait": "diagonal",
    "step_hz": 1.15,
    "swing_frac": 0.22,
    "step_height": 0.040,
    "speed_scale": 1.00,
    "height_offset": 0.000,
    "com_x": 0.00,
    "com_y": 0.00,
    "imu_enabled": False,   # GUI checkbox in your screenshot is unchecked
}

def _apply_gait_preset(eng):
    """Force the gait engine to the same behavior/params as gait_engine_app GUI."""
    eng.set_gait(_GAIT_PRESET["gait"])
    eng.set_params(
        step_hz=_GAIT_PRESET["step_hz"],
        swing_frac=_GAIT_PRESET["swing_frac"],
        base_step_height=_GAIT_PRESET["step_height"],
    )
    eng.set_speed_scale(_GAIT_PRESET["speed_scale"])
    eng.set_height_offset(_GAIT_PRESET["height_offset"])
    eng.set_com_offset(x=_GAIT_PRESET["com_x"], y=_GAIT_PRESET["com_y"])
    eng.enable_imu(_GAIT_PRESET["imu_enabled"])


def _ensure_gait():
    global _gait_engine, _gait_sched
    with _gait_lock:
        if _gait_engine is not None:
            return _gait_engine

        _gait_sched = _AfterScheduler()
        _gait_engine = GaitEngine(
            iklegs_move=iklegs_move,
            get_gravity=get_gravity,
            body_len=BODY_LEN,
            body_wid=BODY_WID,
            tk_window=_gait_sched,
            dt_ms=50,
            # These defaults already match the GUI, but we still hard-set below.
        )

        # Force preset so it behaves exactly like your 3rd script GUI config
        _apply_gait_preset(_gait_engine)

        return _gait_engine

def gait_command(vx: float, vy: float, wz: float):
    """
    Primary entry point for driving:
      vx, vy in units/s; wz in rad/s.
    Nonzero => engine runs; all zero => engine stops.
    """
    eng = _ensure_gait()
    eng.set_velocity(float(vx), float(vy), float(wz))
    if (abs(vx) + abs(vy) + abs(wz)) > 1e-6:
        eng.start()
    else:
        eng.stop()

def gait_stop_and_stand():
    eng = _ensure_gait()
    eng.set_velocity(0.0, 0.0, 0.0)
    eng.stop()
    stand_up()


# ——— Walking/turning primitives (compat wrappers; now gait-engine driven) ———
WALK_SPEED = 15  # kept for compatibility

# default commanded values if someone calls legacy walk_* APIs
_DEFAULT_VX = 0.20
_DEFAULT_VY = 0.20
_DEFAULT_WZ = 1.50

def walk_init(speed=WALK_SPEED):
    # Start gait with a default forward command (legacy API)
    gait_command(_DEFAULT_VX, 0.0, 0.0)

def walk_cycle(speed=WALK_SPEED):
    # Legacy callers expect this to "do a step"; gait runs on its own scheduler.
    time.sleep(0.05)

def walk_reset(speed=WALK_SPEED):
    gait_stop_and_stand()

# Legacy toggle-style turning calls (kept so older callers don't break)
_is_turning_left = False
_is_turning_right = False

def turn_left(speed=WALK_SPEED):
    global _is_turning_left, _is_turning_right
    _is_turning_right = False
    if not _is_turning_left:
        _is_turning_left = True
        lcd.lcd("Turning Left")
        gait_command(0.0, 0.0, +_DEFAULT_WZ)
    else:
        _is_turning_left = False
        lcd.lcd("Resetting")
        gait_stop_and_stand()
        lcd.clear()

def turn_right(speed=WALK_SPEED):
    global _is_turning_left, _is_turning_right
    _is_turning_left = False
    if not _is_turning_right:
        _is_turning_right = True
        lcd.lcd("Turning Right")
        gait_command(0.0, 0.0, -_DEFAULT_WZ)
    else:
        _is_turning_right = False
        lcd.lcd("Resetting")
        gait_stop_and_stand()
        lcd.clear()
