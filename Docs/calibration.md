# Calibration Guide

Calibration is one of the most important parts of getting Corndog working correctly.

## Current workflow

### 1. Find servo endpoints

Run:

```bash
python3 SingularMotortest.py
```

Use this to determine the usable range for each servo.

### 2. Set the standing pose

- Place the robot upside down
- Manually adjust each joint
- Find a pose where all joints are aligned consistently
- Save those values into `servo_home`

### 3. Review mapping values

Check `MAPPING` and `CHANNEL_MAP` in `main.py`.

### 4. Tune offsets

Adjust offsets until the robot’s actual physical home pose matches the intended pose.

## Example offset adjustment

```text
Leg 3 planned deltas:
{7: 10.3512659818415784, 5: -6.30598966999280464, 1: -30.17742776376383063}
```

Before:

```python
3: {
  1: {'sign': +1, 'offset': 45},
  2: {'sign': -1, 'offset': 83},
  3: {'sign': -1, 'offset': 131}
}
```

After:

```python
3: {
  1: {'sign': +1, 'offset': 35},
  2: {'sign': -1, 'offset': 89},
  3: {'sign': -1, 'offset': 161}
}
```

## Status

This page will be expanded with photos, better examples, and troubleshooting notes.
