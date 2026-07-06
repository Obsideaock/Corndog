"""
corndog_pi_emotes.py — Pi-side dispatch for "EMOTE <token>" commands.

The Steam Deck's emote wheel sends tokens from cdc_emotes.py; this module maps
each token to real MoveLib motion. It owns a tiny pose state machine so that:

  * toggling emotes (handstand, fetal, limp, sit, kneel) exit themselves when
    fired again,
  * "stand" is a universal escape hatch from ANY pose,
  * firing a new pose emote from inside another one unwinds the old pose first
    (e.g. handstand while sitting -> unsit, then handstand),
  * sit-context emotes (wave, shake, limp) sit the robot first if needed,
  * "liedown" powers the servos down; the next emote re-enables + stands.

Motion sequences are ported 1:1 from main.py's GUI callbacks, so they land on
the same servo deltas you've already tuned.

dispatch(token) is blocking (call it from the control listener with
emotion_busy held, exactly like the button emotes). Returns the human-readable
result string for logging, or raises nothing — unknown tokens are ignored.
"""

from __future__ import annotations
import time

import MoveLib as mlib
from cdc_emotes import is_valid

# ---- pose state --------------------------------------------------------
# "standing" | "sitting" | "kneeling" | "handstand" | "fetal" | "companion" | "down"
_pose = "standing"
_is_limp = False
_companion_stage = 0        # 0 = not in flow; 1 = open; 2 = set; 3 = closed


def pose() -> str:
    """Current pose, for PONG/status reporting. Includes limp as a suffix."""
    return _pose + ("+limp" if _is_limp else "")


def _stop():
    mlib.stop_gait(schedule_inactivity_reset=False)


# ---- pose exits (each returns robot to standing) ------------------------

def _exit_sit():
    global _is_limp
    if _is_limp:
        _unlimp_motion()
    mlib.unsit()


def _exit_kneel():
    mlib.unkneel()


def _exit_handstand():
    mlib._lcd_msg("Standing Down")
    mlib.move_motors({11: -25, 10: 25})
    mlib.move_motors({0: -10, 1: 10})
    mlib.move_motors({15: -85, 11: 10, 10: -10, 14: 85, 0: -50, 1: 50})
    mlib.move_motors({0: 40, 4: -20, 5: 20, 1: -40})
    mlib.stand_up()
    mlib._lcd_clear()


def _exit_fetal():
    mlib._lcd_msg("Growing Up")
    mlib.move_motors({8: 30, 9: -30, 6: 30, 7: -30})
    mlib.move_motors({15: 40, 0: 40, 14: -40, 1: -40})
    mlib._lcd_clear()


def _exit_companion():
    global _companion_stage
    mlib._lcd_msg("Companion: Reset")
    mlib.iklegs_move({0: (0, 0, 0), 1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, 0)},
                     step_multiplier=15, speed=10, delay=0.01)
    _companion_stage = 0
    mlib._lcd_clear()


def _exit_down():
    mlib.enable_servos()
    time.sleep(0.2)
    mlib.stand_up()


_EXITS = {
    "sitting": _exit_sit,
    "kneeling": _exit_kneel,
    "handstand": _exit_handstand,
    "fetal": _exit_fetal,
    "companion": _exit_companion,
    "down": _exit_down,
}


def _to_standing():
    """Unwind whatever pose we're in back to a normal stand."""
    global _pose
    fn = _EXITS.get(_pose)
    if fn is not None:
        fn()
    _pose = "standing"


# ---- pose entries / one-shots -------------------------------------------

def _enter_handstand():
    mlib._lcd_msg("Handstanding")
    mlib.move_motors({0: -50, 4: 20, 5: -20, 1: 50})
    mlib.move_motors({15: 85, 11: -10, 10: 10, 14: -85})
    mlib.move_motors({0: 50, 1: -50})
    mlib.move_motors({11: 25, 10: -25})
    mlib._lcd_clear()


def _enter_fetal():
    mlib._lcd_msg("Fetal mode")
    mlib.move_motors({15: -40, 0: -40, 14: 40, 1: 40, 8: -30, 9: 30, 6: -30, 7: 30})
    mlib._lcd_clear()


def _companion_advance():
    """Each 'companion' emote fire advances the flow one stage; final fire resets."""
    global _companion_stage
    if _companion_stage == 0:
        mlib._lcd_msg("Companion: Open")
        mlib.iklegs_move({0: (0.05, 0, -0.09), 1: (0.05, 0, -0.09),
                          2: (-0.05, 0, -0.09), 3: (-0.05, 0, -0.09)},
                         step_multiplier=15, speed=10, delay=0.01)
        _companion_stage = 1
        return "companion open (fire again: set)"
    if _companion_stage == 1:
        mlib._lcd_msg("Companion: Set")
        mlib.move_motors({4: 0, 1: -10, 5: 50, 14: -25, 10: 10, 15: 0, 11: 0, 6: -30, 9: 35})
        time.sleep(0.1)
        mlib.move_motors({0: -10})
        _companion_stage = 2
        return "companion set (fire again: close)"
    if _companion_stage == 2:
        mlib._lcd_msg("Companion: Closed")
        mlib.iklegs_move({0: (0.05, 0, -0.09), 1: (0.05, 0, -0.09),
                          2: (-0.05, 0, -0.09), 3: (-0.05, 0, -0.09)},
                         step_multiplier=15, speed=10, delay=0.01)
        _companion_stage = 3
        return "companion closed (fire again: reset)"
    _exit_companion()
    return "companion reset"


def _jump():
    mlib._lcd_msg("Charging jump")
    mlib.iklegs_move({0: (0, 0, 0.05), 1: (0, 0, 0.05), 2: (0, 0, 0.05), 3: (0, 0, 0.05)},
                     step_multiplier=20, speed=0.05)
    time.sleep(1)
    mlib._lcd_msg("Jumping")
    # speed_mode: slam every leg to full extension at once — this is exactly
    # the case the toggle exists for.
    mlib.iklegs_move({0: (0, 0, -0.05), 1: (0, 0, -0.05), 2: (0, 0, -0.05), 3: (0, 0, -0.05)},
                     speed=50, delay=0, speed_mode=True)
    time.sleep(0.2)
    mlib.iklegs_move({0: (0, 0, 0), 1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, 0)},
                     speed=50, delay=0)
    mlib._lcd_clear()


def _seizure():
    mlib._lcd_msg("Seizing")
    mlib.move_motors({0: 40, 1: -40, 4: 40, 5: 40, 6: 40, 7: -40, 8: 40, 9: -40,
                      10: 40, 11: 40, 14: 40, 15: -60}, speed_multiplier=20)
    time.sleep(2)
    mlib.move_motors({6: -40, 7: 40, 8: -40, 9: 40})
    time.sleep(0.5)
    mlib.move_motors({0: -40, 1: 40, 4: -40, 5: -40, 10: -40, 11: -40, 14: -40, 15: 60})
    mlib._lcd_clear()


def _limp_motion():
    global _is_limp
    mlib._lcd_msg("Going Limp")
    mlib.move_motors({10: 95, 11: -95, 4: -50, 5: 50})
    _is_limp = True
    mlib._lcd_clear()


def _unlimp_motion():
    global _is_limp
    mlib._lcd_msg("Unlimping")
    mlib.move_motors({10: -95, 11: 95, 4: 50, 5: -50})
    _is_limp = False
    mlib._lcd_clear()


def _liedown():
    mlib._lcd_msg("Down")
    mlib.move_motors({15: -40, 0: -40, 14: 40, 1: 40})
    mlib.disable_servos()
    mlib._lcd_clear()


# ---- dispatcher ----------------------------------------------------------

def dispatch(token: str) -> str:
    """Execute one emote token. Blocking. Returns a log string."""
    global _pose

    token = (token or "").strip().lower()
    if not is_valid(token):
        return f"unknown emote '{token}'"

    _stop()

    # Anything fired while powered down first re-enables + stands.
    if _pose == "down" and token != "liedown":
        _exit_down()
        _pose = "standing"
        if token == "stand":
            return "stood up (was lying down)"

    # ---- universal escape hatch ----
    if token == "stand":
        _to_standing()
        return "standing"

    # ---- toggle poses: firing the pose you're already in exits it ----
    if token == "sit":
        if _pose == "sitting":
            _to_standing(); return "unsat"
        _to_standing(); mlib.sit(); _pose = "sitting"; return "sitting"

    if token == "kneel":
        if _pose == "kneeling":
            _to_standing(); return "unkneeled"
        _to_standing(); mlib.kneel(); _pose = "kneeling"; return "kneeling"

    if token == "handstand":
        if _pose == "handstand":
            _to_standing(); return "back down"
        _to_standing(); _enter_handstand(); _pose = "handstand"; return "handstand"

    if token == "fetal":
        if _pose == "fetal":
            _to_standing(); return "grew up"
        _to_standing(); _enter_fetal(); _pose = "fetal"; return "fetal"

    if token == "companion":
        if _pose not in ("companion", "standing"):
            _to_standing()
        result = _companion_advance()
        _pose = "companion" if _companion_stage else "standing"
        return result

    # ---- sit-context emotes: auto-sit first ----
    if token in ("wave", "shake", "limp"):
        if _pose != "sitting":
            _to_standing(); mlib.sit(); _pose = "sitting"
        if token == "wave":
            mlib.wave(); return "waved"
        if token == "shake":
            mlib.shake(); return "shook"
        # limp toggles
        if _is_limp:
            _unlimp_motion(); return "unlimped"
        _limp_motion(); return "limp"

    # ---- stand-context one-shots ----
    if token in ("dance", "jump", "seizure"):
        _to_standing()
        if token == "dance":
            mlib.dance(); return "danced"
        if token == "jump":
            _jump(); return "jumped"
        _seizure(); return "seized"

    if token == "liedown":
        if _pose == "down":
            _exit_down(); _pose = "standing"; return "back up"
        _to_standing(); _liedown(); _pose = "down"; return "lying down (servos off)"

    return f"unhandled emote '{token}'"
