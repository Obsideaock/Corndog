#!/usr/bin/env python3
"""
corndog_config.py — single source of truth for Corndog's on-disk configuration.

Every script that needs servo homes / IK mapping offsets should do:

    from corndog_config import load_calibration
    cal = load_calibration()
    servo_home     = cal["servo_home"]        # {channel:int -> angle:float}
    mapping_right  = cal["mapping_right"]     # {leg:int -> {joint:int -> {'sign','offset'}}}

If no calibration file exists yet (fresh install, robot never calibrated),
the FACTORY_* defaults below are returned — these are the values that were
previously hardcoded in main.py / MoveLib.py, so behavior is unchanged
until the owner runs `corndog calibrate`.

File locations (all path-independent, no /home/Corndog assumptions):
    config dir : $CORNDOG_CONFIG_DIR  or  ~/.config/corndog/
    calibration: <config dir>/calibration.json
    install dir: <config dir>/install.env   (written by install.sh)
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

def config_dir() -> Path:
    d = os.environ.get("CORNDOG_CONFIG_DIR")
    p = Path(d) if d else Path.home() / ".config" / "corndog"
    p.mkdir(parents=True, exist_ok=True)
    return p


def calibration_path() -> Path:
    return config_dir() / "calibration.json"


# ---------------------------------------------------------------------------
# Factory defaults — EXACTLY the values currently hardcoded in MoveLib.py.
# These are only used when calibration.json does not exist.
# ---------------------------------------------------------------------------

SERVO_CHANNELS = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]

FACTORY_SERVO_HOME = {
    0: 39.0,  1: 231.0,  4: 222.0,  5: 50.0,
    6: 128.0, 7: 130.0,  8: 133.0,  9: 135.0,
    10: 80.0, 11: 204.0, 14: 235.0, 15: 33.0,
}

# Right legs only (1 = right front, 3 = right back). Left legs mirror deltas.
FACTORY_MAPPING_RIGHT = {
    1: {1: {"sign": -1, "offset": 228}, 2: {"sign": -1, "offset": 119}, 3: {"sign": -1, "offset": 165}},
    3: {1: {"sign": +1, "offset": 35},  2: {"sign": -1, "offset": 89},  3: {"sign": -1, "offset": 161}},
}

# leg index -> joint -> PCA9685 channel   (copied from MoveLib.CHANNEL_MAP)
CHANNEL_MAP = {
    0: {1: 9, 2: 11, 3: 15},   # left  front
    1: {1: 8, 2: 10, 3: 14},   # right front
    2: {1: 6, 2: 4,  3: 0},    # left  back
    3: {1: 7, 2: 5,  3: 1},    # right back
}

LEG_NAMES = {0: "Left Front", 1: "Right Front", 2: "Left Back", 3: "Right Back"}
JOINT_NAMES = {1: "Hip", 2: "Shoulder", 3: "Knee"}


def _normalize(raw: dict) -> dict:
    """JSON keys come back as strings; convert to the int-keyed dicts the
    robot code expects, and fill any missing entries from factory values."""
    out = {
        "version": raw.get("version", 0),
        "saved": raw.get("saved"),
        "servo_home": dict(FACTORY_SERVO_HOME),
        "mapping_right": {l: {j: dict(v) for j, v in js.items()}
                          for l, js in FACTORY_MAPPING_RIGHT.items()},
        "reference_angles": {},
        "source": raw.get("source", "factory"),
    }
    for k, v in (raw.get("servo_home") or {}).items():
        out["servo_home"][int(k)] = float(v)
    for l, js in (raw.get("mapping_right") or {}).items():
        for j, m in js.items():
            out["mapping_right"][int(l)][int(j)] = {
                "sign": int(m["sign"]), "offset": float(m["offset"])
            }
    for k, v in (raw.get("reference_angles") or {}).items():
        out["reference_angles"][int(k)] = float(v)
    return out


def load_calibration() -> dict:
    """Load calibration.json, falling back to factory defaults per-field."""
    p = calibration_path()
    if p.exists():
        try:
            return _normalize(json.loads(p.read_text(encoding="utf-8")))
        except Exception as e:
            print(f"[corndog_config] WARNING: could not read {p} ({e}); "
                  f"using factory defaults")
    return _normalize({})


def save_calibration(servo_home: dict, mapping_right: dict,
                     reference_angles: dict | None = None,
                     source: str = "calibrate-tool") -> Path:
    """Atomically write calibration.json (backs up the previous file)."""
    p = calibration_path()
    if p.exists():
        backup = p.with_suffix(".json.bak")
        backup.write_text(p.read_text(encoding="utf-8"), encoding="utf-8")
    data = {
        "version": 1,
        "saved": datetime.now().isoformat(timespec="seconds"),
        "source": source,
        "servo_home": {str(k): round(float(v), 3) for k, v in servo_home.items()},
        "mapping_right": {
            str(l): {str(j): {"sign": int(m["sign"]),
                              "offset": round(float(m["offset"]), 3)}
                     for j, m in js.items()}
            for l, js in mapping_right.items()
        },
        "reference_angles": {str(k): round(float(v), 3)
                             for k, v in (reference_angles or {}).items()},
    }
    tmp = p.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
    tmp.replace(p)
    return p


def is_calibrated() -> bool:
    return calibration_path().exists()


if __name__ == "__main__":
    cal = load_calibration()
    print(f"calibration file: {calibration_path()}"
          f"  (exists: {is_calibrated()})")
    print(f"source: {cal['source']}   saved: {cal['saved']}")
    for ch in SERVO_CHANNELS:
        print(f"  ch {ch:>2}: home {cal['servo_home'][ch]:7.2f}")
