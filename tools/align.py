#!/usr/bin/env python3
"""
align.py — PART A of assembly: the "horn pose" holder.

Run this at the very start of assembly, BEFORE any horns are attached.
It drives every connected servo to the reference pose (every joint straight
or at exactly 90 deg) and HOLDS it there while you:

  1. plug each servo into its channel (the table below shows which servo
     goes where — label the wires now, before they disappear into the frame),
  2. press each horn on in the visually-correct orientation,
  3. unplug the servos and start assembling.

You can leave everything plugged in the whole time — plugging a servo in
late is fine too; press its number to (re)send its angle.

Usage:
    corndog align                    (via the installed launcher)
    python3 tools/align.py           (directly)
    python3 tools/align.py --simulate
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from corndog_config import SERVO_CHANNELS, CHANNEL_MAP, LEG_NAMES, JOINT_NAMES  # noqa: E402
from master_tables import REFERENCE_POSE_DEG                                    # noqa: E402
from servo_rig import open_rig, confirm                                         # noqa: E402

CYN = "\033[36m"; GRN = "\033[32m"; YLW = "\033[33m"; RST = "\033[0m"


def label_table() -> str:
    lines = []
    lines.append(f"  {CYN}{'CH':>3}  {'LEG':<12}{'JOINT':<10}{'ANGLE':>7}{RST}")
    lines.append("  " + "-" * 36)
    for leg in (0, 1, 2, 3):
        for joint in (1, 2, 3):
            ch = CHANNEL_MAP[leg][joint]
            lines.append(f"  {ch:>3}  {LEG_NAMES[leg]:<12}{JOINT_NAMES[joint]:<10}"
                         f"{REFERENCE_POSE_DEG[ch]:>7.1f}")
        lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="Corndog Part A: horn-pose holder")
    ap.add_argument("--simulate", action="store_true",
                    help="run without hardware (dry run)")
    args = ap.parse_args()

    print()
    print(f"  {GRN}CORNDOG ASSEMBLY — PART A: HORN POSE{RST}")
    print("  ------------------------------------")
    print("  Every servo will be driven to its reference angle and HELD there")
    print("  so you can attach the horns in the correct orientation.")
    print("  Servos should be loose / not yet mounted in the frame, or the")
    print("  robot should be free to move without pushing against anything.")

    confirm("Servos powered and free to move?", args.simulate)

    rig = open_rig(args.simulate)
    print()
    print("  Sending all channels to the horn pose...")
    rig.goto_many(dict(REFERENCE_POSE_DEG))
    print(f"  {GRN}Holding.{RST} Attach horns now. Wire labels:")
    print()
    print(label_table())

    print(f"  {YLW}Type a channel number to re-send just that servo (useful when")
    print(f"  plugging them in one at a time), or press Enter to finish.{RST}")
    try:
        while True:
            raw = input("  channel (Enter = done): ").strip()
            if raw == "":
                break
            try:
                ch = int(raw)
            except ValueError:
                print("  not a number")
                continue
            if ch not in SERVO_CHANNELS:
                print(f"  channel {ch} isn't used (valid: {SERVO_CHANNELS})")
                continue
            rig.goto(ch, REFERENCE_POSE_DEG[ch], ramp=False)
            print(f"  ch {ch} -> {REFERENCE_POSE_DEG[ch]:.1f} deg")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        rig.torque_off()
        print(f"  {GRN}Torque released — safe to unplug. Happy assembling!{RST}")


if __name__ == "__main__":
    main()
