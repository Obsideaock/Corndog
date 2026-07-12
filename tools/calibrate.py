#!/usr/bin/env python3
"""
calibrate.py — PART B of assembly: the trim tool.

Run this AFTER the robot is fully assembled. Horn splines never land
perfectly, so each joint can be a few degrees off from truly straight / 90.
This tool:

  1. energizes EVERY servo to the reference pose the moment you confirm
     (so hold him or lay him on his back FIRST),
  2. lets you select any joint and nudge it by 0.1 / 0.5 / 1 / 5 deg until
     it is visually perfect,
  3. on save: stores the trimmed reference angles, applies the master
     reference->home deltas to compute each servo's true home angle, and
     recomputes the right-leg IK mapping offsets — all written to
     ~/.config/corndog/calibration.json, which the robot reads at startup.

Usage:
    corndog calibrate
    python3 tools/calibrate.py
    python3 tools/calibrate.py --simulate
"""

from __future__ import annotations

import argparse
import curses
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from corndog_config import (                                   # noqa: E402
    SERVO_CHANNELS, CHANNEL_MAP, LEG_NAMES, JOINT_NAMES,
    load_calibration, save_calibration, calibration_path,
)
from master_tables import (                                    # noqa: E402
    REFERENCE_POSE_DEG, REF_TO_HOME_DELTA_DEG,
    REFERENCE_JOINT_ANGLES_DEG, deltas_ready, thetas_ready,
)
from servo_rig import Rig, open_rig                            # noqa: E402

STEP_SIZES = [0.1, 0.5, 1.0, 5.0]
ROWS = [(leg, joint) for leg in (0, 1, 2, 3) for joint in (1, 2, 3)]

GRN = "\033[32m"; YLW = "\033[33m"; RED = "\033[31m"; RST = "\033[0m"


class UI:
    def __init__(self, scr, rig: Rig):
        self.scr = scr
        self.rig = rig
        self.sel = 0
        self.step_i = 2                    # default 1.0 deg
        self.msg = "Nudge each joint until it is truly straight / 90 deg, then press s"
        curses.curs_set(0)
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_RED, -1)
        curses.init_pair(2, curses.COLOR_GREEN, -1)
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        curses.init_pair(4, curses.COLOR_CYAN, -1)
        scr.timeout(150)

    def draw(self):
        scr = self.scr
        scr.erase()
        h, w = scr.getmaxyx()
        A = curses.color_pair(1) | curses.A_BOLD
        G = curses.color_pair(2)
        Y = curses.color_pair(3)
        C = curses.color_pair(4)

        title = " CORNDOG CALIBRATION — PART B: TRIM "
        scr.addnstr(0, 0, "=" * (w - 1), w - 1, A)
        scr.addnstr(0, max(0, (w - len(title)) // 2), title, w - 1, A)
        if self.rig.simulate:
            scr.addnstr(0, max(0, w - 14), "(SIMULATION)", 12, Y)

        hdr = f"{'':2}{'LEG':<12}{'JOINT':<10}{'CH':>3}  {'ANGLE':>8}  {'REF':>8}  {'TRIM':>7}  {'PWR':>3}"
        scr.addnstr(2, 2, hdr, w - 3, C | curses.A_BOLD)

        for i, (leg, joint) in enumerate(ROWS):
            ch = CHANNEL_MAP[leg][joint]
            a = self.rig.angles.get(ch)
            ref = REFERENCE_POSE_DEG[ch]
            row = f"{LEG_NAMES[leg]:<12}{JOINT_NAMES[joint]:<10}{ch:>3}  "
            row += f"{a:8.1f}  " if a is not None else f"{'--':>8}  "
            row += f"{ref:8.1f}  "
            row += (f"{a - ref:+7.1f}  " if a is not None else f"{'--':>7}  ")
            row += " ON" if self.rig.torque.get(ch) else "off"
            y = 3 + i
            if y >= h - 6:
                break
            attr = curses.A_REVERSE | A if i == self.sel else curses.A_NORMAL
            scr.addnstr(y, 2, ("> " if i == self.sel else "  ") + row, w - 3, attr)

        y = 3 + len(ROWS) + 1
        scr.addnstr(y, 2, f"step: {STEP_SIZES[self.step_i]:g} deg   "
                          f"(keys 1-4 = 0.1 / 0.5 / 1 / 5)", w - 3, C)
        if not deltas_ready():
            scr.addnstr(y + 1, 2, "NOTE: master ref->home deltas not filled in yet — "
                                  "save stores reference angles only", w - 3, Y)
        scr.addnstr(min(h - 3, y + 2), 2, self.msg, w - 3, G)
        scr.addnstr(h - 2, 2, "up/down select   left/right or -/+ nudge   "
                              "r re-send reference pose", w - 3, C)
        scr.addnstr(h - 1, 2, "t torque OFF (sel)   T ALL off   e re-enable   "
                              "s SAVE   q quit", w - 3, C)
        scr.refresh()

    def sel_channel(self) -> int:
        leg, joint = ROWS[self.sel]
        return CHANNEL_MAP[leg][joint]

    def nudge(self, direction: int):
        ch = self.sel_channel()
        cur = self.rig.angles.get(ch)
        if cur is None:
            cur = REFERENCE_POSE_DEG[ch]
        self.rig.goto(ch, cur + direction * STEP_SIZES[self.step_i], ramp=False)
        leg, joint = ROWS[self.sel]
        self.msg = (f"{LEG_NAMES[leg]} {JOINT_NAMES[joint]} (ch {ch}) -> "
                    f"{self.rig.angles[ch]:.1f} deg "
                    f"({self.rig.angles[ch] - REFERENCE_POSE_DEG[ch]:+.1f} from ref)")

    def save(self):
        cal = load_calibration()
        ref = {ch: (self.rig.angles.get(ch)
                    if self.rig.angles.get(ch) is not None
                    else REFERENCE_POSE_DEG[ch])
               for ch in SERVO_CHANNELS}

        servo_home = dict(cal["servo_home"])
        mapping = {l: {j: dict(m) for j, m in js.items()}
                   for l, js in cal["mapping_right"].items()}

        if deltas_ready():
            for ch in SERVO_CHANNELS:
                servo_home[ch] = ref[ch] + float(REF_TO_HOME_DELTA_DEG[ch])

        if thetas_ready():
            for leg in (1, 3):
                for j in (1, 2, 3):
                    ch = CHANNEL_MAP[leg][j]
                    sign = mapping[leg][j]["sign"]
                    theta = float(REFERENCE_JOINT_ANGLES_DEG[leg][j])
                    mapping[leg][j]["offset"] = ref[ch] - sign * theta

        p = save_calibration(servo_home, mapping, reference_angles=ref)
        extra = "" if deltas_ready() else " (reference only — deltas pending)"
        self.msg = f"Saved -> {p}{extra}"

    def loop(self):
        while True:
            self.draw()
            k = self.scr.getch()
            if k == -1:
                continue
            if k in (ord("q"), 27):
                return
            elif k in (curses.KEY_UP, ord("k")):
                self.sel = (self.sel - 1) % len(ROWS)
            elif k in (curses.KEY_DOWN, ord("j")):
                self.sel = (self.sel + 1) % len(ROWS)
            elif k in (curses.KEY_RIGHT, ord("+"), ord("=")):
                self.nudge(+1)
            elif k in (curses.KEY_LEFT, ord("-"), ord("_")):
                self.nudge(-1)
            elif k in (ord("1"), ord("2"), ord("3"), ord("4")):
                self.step_i = int(chr(k)) - 1
            elif k == ord("r"):
                self.msg = "Re-sending the full reference pose..."
                self.draw()
                self.rig.goto_many(dict(REFERENCE_POSE_DEG))
                self.msg = "Reference pose re-sent (all trims cleared)"
            elif k == ord("t"):
                ch = self.sel_channel()
                self.rig.torque_off(ch)
                self.msg = f"Torque OFF on channel {ch} (move it by hand)"
            elif k == ord("T"):
                self.rig.torque_off()
                self.msg = "Torque OFF on ALL channels"
            elif k == ord("e"):
                ch = self.sel_channel()
                self.rig.torque_on(ch)
                self.msg = f"Channel {ch} re-enabled at last commanded angle"
            elif k == ord("s"):
                self.save()


def main():
    ap = argparse.ArgumentParser(description="Corndog Part B: trim calibration")
    ap.add_argument("--simulate", action="store_true",
                    help="run without hardware (UI test / dry run)")
    args = ap.parse_args()

    print()
    print(f"  {GRN}CORNDOG CALIBRATION — PART B: TRIM{RST}")
    print("  ----------------------------------")
    print(f"  {YLW}Please place Corndog on his back, or hold him so he doesn't")
    print(f"  push himself around by moving.{RST}")
    print()
    print(f"  {RED}ALL motors turn on and snap to the reference pose the moment")
    print(f"  you confirm.{RST}")
    if args.simulate:
        print("  [simulation mode — no hardware will move]")
    else:
        ans = input("  Is he on his back / held safely? [y/N] ").strip().lower()
        if ans != "y":
            print("  Aborted — nothing was moved. Flip him over and come back.")
            sys.exit(1)

    rig = open_rig(args.simulate)
    print("  Energizing all servos to the reference pose...")
    rig.goto_many(dict(REFERENCE_POSE_DEG))

    try:
        curses.wrapper(lambda scr: UI(scr, rig).loop())
    finally:
        if not args.simulate:
            rig.torque_off()
        print(f"  Torque released. Calibration file: {calibration_path()}")


if __name__ == "__main__":
    main()
