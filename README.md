# Project Corndog

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![GitHub Stars](https://img.shields.io/github/stars/Obsideaock/Corndog.svg)](https://github.com/Obsideaock/Corndog/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/Obsideaock/Corndog.svg)](https://github.com/Obsideaock/Corndog/issues)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red)

> An open-source quadruped robot designed to make robotics accessible — one walking robot at a time.

![Corndog Walking](https://media.openlonehollow.com/gifs/Walking.gif?v=2)

Corndog started as a workaround to a college rule (no drones on campus → build a walking robot instead) and turned into a nearly two-year deep dive into robotics, kinematics, and control systems. Based on the open-source MicroSpot design and inspired by Boston Dynamics' Spot, it's a small quadruped with 3 degrees of freedom per leg, a full inverse kinematics stack, IMU feedback, and a growing set of behaviors.

**As featured on American Ninja Warrior Season 17**

---

## Current Status

| Feature | Status |
|---|---|
| Static walking gaits (forward/backward) | ✅ Working |
| Turn left / turn right | ✅ Working |
| Standing & balancing | ✅ Working |
| Inverse kinematics (IK) | ✅ Working |
| IMU feedback (BNO08X) | ✅ Working |
| Custom animations (dance, wave, handstand, etc.) | ✅ Working |
| Companion mode (shoulder perch) | ✅ Working |
| Steam Deck wireless control + video stream | ✅ Working |
| Dynamic gaits | 🔧 In progress |
| ML-Agents gait learning | 🔧 In progress (environment skeleton done) |
| Custom power PCB | 🔧 In progress |
| Kit available for purchase | 🔜 Coming soon |
| Assembly guide | 🔜 In progress |

---

## Build at a Glance

| Spec | Value |
|---|---|
| Size | ~15 inches |
| Legs | 4 (quadruped) |
| Degrees of freedom | 12 total (3 per leg) |
| Brain | Raspberry Pi 5 |
| Servo driver | Adafruit PCA9685 |
| Actuation range | 0–270° per servo |
| Power | LiPo battery + buck converter (custom PCB coming) |
| Construction | 3D printed (PLA) |
| Control | Tkinter GUI on-device, or Steam Deck over TCP |

---

## Estimated Cost

Corndog is open-source — you can print and source everything yourself. If you'd rather skip the work, a kit is coming soon through [OpenLonehollow](https://openlonehollow.com/Projects/corndog/corndog.html).

| Tier | What You Get | Estimated Price |
|---|---|---|
| **Printed Parts Only** | All 3D printed structural components | ~$60–80 |
| **Unassembled Kit** | Printed parts + all electronics | ~$450–500 |
| **Assembled & Calibrated** | Fully built, programmed, and ready to go | ~$800 |

Want to build it yourself from scratch? Everything you need is in this repo and the [assembly guide](https://openlonehollow.com/Projects/corndog/assembly-guide/index.html).

---

## Hardware

| Component | Notes | Example Link |
|---|---|---|
| Raspberry Pi 5 | Raspbian 64-bit | [Amazon](https://a.co/d/dEzpuJt) |
| Adafruit PCA9685 PWM Driver | I²C address 0x40 | [Adafruit](https://www.adafruit.com/product/815) |
| Servo Motors × 12 | 0–270°, pulse 500–2500 µs | [Amazon](https://a.co/d/4C4s5K8) |
| Adafruit BNO08X IMU | I²C address 0x4B | — |
| LiPo Battery + Power Module | Custom PCB solution coming | [Amazon](https://a.co/d/3LN6lL5) |
| I²C LCD Display | For status readout | — |
| Wires, headers, standoffs | Standard hobby hardware | — |

> A full Bill of Materials with exact quantities and sources is in progress and will be linked here.

---

## 3D Printed Parts

Most structural parts come from [KDY0523's Thingiverse design](https://www.thingiverse.com/thing:3445283), with reinforced legs from [mike4192's SpotMicro fork](https://github.com/mike4192/spotMicro).

Modifications and custom parts (like the shoulder harness for companion mode) are included in the `/3DModels` folder.

- All parts currently printed in **PLA**
- Experimented with ASA for joints (more strength) but it snapped under stress and PLA is easier to work with
- Thanks to **BTULab (where I work now) @ CU Boulder** for printing assistance

---

## Software Dependencies

All code targets **Python 3.8+** on Raspbian 64-bit.

```bash
sudo apt update && sudo apt install -y python3-pip python3-venv python3-tk \
  libopencv-dev network-manager

pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit \
  gpiozero numpy opencv-python picamera2
```

Also required (manual install):
- **[spot-micro-kinematics](https://github.com/mike4192/spot_micro_kinematics_python/tree/master)** — IK and stick-figure utilities

---

## Key Scripts

| Script | What it does |
|---|---|
| `main.py` | Core control: IK, all movements, Tkinter GUI |
| `SteamDeckCommunication.py` | Joystick control over TCP + MJPEG video stream |
| `SingularMotortest.py` | Manual servo calibration tool |
| `MoveLib.py` | Movement library (aliased by SteamDeck script) |
| `gait_engine_app.py` | Gait engine module |
| `StartupLCD.py` | LCD startup display |
| `BTtest.py` | Bluetooth testing |
| `flipper_menu.py` / `FlipperTed.py` | Flipper Zero integration |

---

## Setup & Calibration

> Full calibration guide coming soon. Here's the short version:

**1. Clone and install**
```bash
git clone https://github.com/Obsideaock/Corndog.git
cd Corndog
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**2. Servo calibration**

Run `SingularMotortest.py` to manually find the 0°–270° endpoints for each servo.

Place the robot upside down and adjust each joint to find the standing position (all joints equidistant from ground). Plug those angles into `servo_home` in `main.py`.

Check the channel-to-joint mapping in `MAPPING` and `CHANNEL_MAP`.

Enable `safety=True` to print planned deltas without moving anything:
```
# Example output:
Leg 3 planned deltas: {7: 10.35, 5: -6.30, 1: -30.17}

# Take the inverse of each delta and add it to your offsets:
Before: 3: {1:{'sign':+1,'offset':45}, 2:{'sign':-1,'offset':83}, 3:{'sign':-1,'offset':131}}
After:  3: {1:{'sign':+1,'offset':35}, 2:{'sign':-1,'offset':89}, 3:{'sign':-1,'offset':161}}
```

Repeat until the robot stands level.

**3. Run**
```bash
# GUI control (on-device)
python3 main.py

# Steam Deck / remote control + video stream
python3 SteamDeckCommunication.py
# Stream available at http://<pi-ip>:8000/stream.mjpg
# Control on TCP port 65432
```

---

## Roadmap

- [x] Static walking gaits
- [x] Turn left / right
- [x] Inverse kinematics
- [x] IMU integration
- [x] Companion mode
- [x] Steam Deck control
- [ ] Assembly guide (in progress)
- [ ] Full BOM
- [ ] Custom power PCB
- [ ] Dynamic gaits
- [ ] ML-Agents gait training
- [ ] Web dashboard
- [ ] Kit launch

---

## Contributing

Issues and PRs are welcome — please use [GitHub Issues](https://github.com/Obsideaock/Corndog/issues) and follow existing code style. The project is licensed [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), so use and adapt freely with credit.

---

## Acknowledgements

- **[KDY0523](https://www.thingiverse.com/thing:3445283)** — original 3D design
- **[Mike4192](https://github.com/mike4192/spotMicro)** — reinforced legs and IK library
- **[BTULab @ CU Boulder](https://www.colorado.edu/atlas/btu-lab)** — 3D printing support

---

*Built by [Leo Heuring (Obsideaock)](https://openlonehollow.com) · [openlonehollow.com](https://openlonehollow.com)*
