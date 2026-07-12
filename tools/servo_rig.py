#!/usr/bin/env python3
"""
servo_rig.py — shared PCA9685 layer for the calibration tools.

Deliberately does NOT import MoveLib: during assembly the IMU / LCD / camera
may not be wired yet, and MoveLib initializes hardware at import time. This
talks straight to the PCA9685 (or fakes it with simulate=True).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from corndog_config import SERVO_CHANNELS  # noqa: E402

RAMP_DEG_PER_S = 90.0
ANGLE_MIN, ANGLE_MAX = 0.0, 270.0


class Rig:
    def __init__(self, simulate: bool):
        self.simulate = simulate
        self.angles: dict[int, float | None] = {ch: None for ch in SERVO_CHANNELS}
        self.torque: dict[int, bool] = {ch: False for ch in SERVO_CHANNELS}
        if simulate:
            self.pca = None
            self.servos = None
            return
        import board, busio                              # noqa: E401
        from adafruit_pca9685 import PCA9685
        from adafruit_motor import servo as af_servo
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.servos = {}
        for ch in SERVO_CHANNELS:
            s = af_servo.Servo(self.pca.channels[ch])
            s.set_pulse_width_range(500, 2500)
            s.actuation_range = 270
            self.servos[ch] = s

    def _write(self, ch: int, angle: float):
        angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))
        if not self.simulate:
            self.servos[ch].angle = angle
        self.angles[ch] = angle
        self.torque[ch] = True

    def goto(self, ch: int, target: float, ramp: bool = True):
        """Move one channel. Ramps only if the current position is known —
        the first write to a channel is always an instant jump (the servo's
        physical position is unknown until commanded)."""
        target = max(ANGLE_MIN, min(ANGLE_MAX, target))
        cur = self.angles.get(ch)
        if not ramp or cur is None or self.simulate:
            self._write(ch, target)
            return
        step_t = 0.02
        step_d = RAMP_DEG_PER_S * step_t
        a = cur
        while abs(target - a) > step_d:
            a += step_d if target > a else -step_d
            self._write(ch, a)
            time.sleep(step_t)
        self._write(ch, target)

    def goto_many(self, targets: dict[int, float]):
        """Move several channels together (synchronized finish). Channels
        with unknown position jump; known ones ramp."""
        unknown = {ch: t for ch, t in targets.items()
                   if self.angles.get(ch) is None}
        for ch, t in unknown.items():
            self._write(ch, t)
        known = {ch: t for ch, t in targets.items() if ch not in unknown}
        if not known or self.simulate:
            return
        start = {ch: self.angles[ch] for ch in known}
        dur = max(abs(known[ch] - start[ch]) for ch in known) / RAMP_DEG_PER_S
        steps = max(1, int(dur / 0.02))
        for s in range(1, steps + 1):
            f = s / steps
            for ch, t in known.items():
                self._write(ch, start[ch] + (t - start[ch]) * f)
            time.sleep(0.02)

    def torque_off(self, ch: int | None = None):
        chans = [ch] if ch is not None else SERVO_CHANNELS
        for c in chans:
            if not self.simulate:
                self.pca.channels[c].duty_cycle = 0
            self.torque[c] = False

    def torque_on(self, ch: int):
        a = self.angles.get(ch)
        if a is not None:
            self._write(ch, a)


def open_rig(simulate: bool) -> Rig:
    """Create the rig or exit with a friendly message."""
    try:
        return Rig(simulate=simulate)
    except Exception as e:
        print(f"  Could not talk to the PCA9685 ({e}).")
        print("  Check wiring / I2C enabled, or run with --simulate.")
        raise SystemExit(1)


def confirm(prompt: str, simulate: bool) -> None:
    """Safety confirmation; exits unless the user types y."""
    print()
    print(f"  {prompt}")
    if simulate:
        print("  [simulation mode — no hardware will move]")
        return
    ans = input("  Ready? [y/N] ").strip().lower()
    if ans != "y":
        print("  Aborted — nothing was moved.")
        raise SystemExit(1)
