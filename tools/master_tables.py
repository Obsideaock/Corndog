#!/usr/bin/env python3
"""
master_tables.py — THE single place where Corndog's reference-pose numbers
live. align.py and calibrate.py both import from here.

Don't hand-type these: run `corndog run tools/master_finder.py` on YOUR
calibrated robot, nudge it from home into the reference pose, and it prints
this whole file's contents ready to paste in.

Definitions:
  REFERENCE POSE  = every joint visually straight or at exactly 90 deg
                    (the pose builders can verify with no tools)
  HOME POSE       = the standing pose the robot returns to (servo_home)
"""

from corndog_config import SERVO_CHANNELS

# --------------------------------------------------------------------------
# 1) Commanded servo angle (deg, 0-270) per channel at the REFERENCE pose.
#    Part A (align.py) drives here for horn attachment; Part B (calibrate.py)
#    starts here for trimming.
# TODO(Leo): placeholder — replace with master_finder.py output.
REFERENCE_POSE_DEG = {ch: 135.0 for ch in SERVO_CHANNELS}

# --------------------------------------------------------------------------
# 2) Signed degrees FROM the reference pose TO the home pose, per channel
#    ("the reverse master angles"). After trimming:
#        servo_home[ch] = trimmed_reference[ch] + REF_TO_HOME_DELTA_DEG[ch]
# TODO(Leo): placeholder — while any value is None, calibrate.py saves the
# trimmed reference only and leaves servo_home untouched (with a warning).
REF_TO_HOME_DELTA_DEG = {ch: None for ch in SERVO_CHANNELS}

# --------------------------------------------------------------------------
# 3) Kinematic joint angle theta (deg) at the reference pose, right legs only
#    (1 = right front, 3 = right back; left legs mirror and need nothing).
#    Used to recompute the IK mapping offsets:
#        offset = trimmed_reference - sign * theta
# TODO(Leo): placeholder — while None, calibrate.py keeps the previous
# offsets. master_finder.py prints an ESTIMATE of these from the current
# mapping; sanity-check against the kinematic model before trusting.
REFERENCE_JOINT_ANGLES_DEG = {
    1: {1: None, 2: None, 3: None},
    3: {1: None, 2: None, 3: None},
}


def deltas_ready() -> bool:
    return all(v is not None for v in REF_TO_HOME_DELTA_DEG.values())


def thetas_ready() -> bool:
    return all(v is not None
               for js in REFERENCE_JOINT_ANGLES_DEG.values()
               for v in js.values())
