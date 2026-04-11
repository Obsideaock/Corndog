# Corndog

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![GitHub Issues](https://img.shields.io/github/issues/Obsideaock/Corndog.svg)](https://github.com/Obsideaock/Corndog/issues)
[![GitHub Stars](https://img.shields.io/github/stars/Obsideaock/Corndog.svg)](https://github.com/Obsideaock/Corndog/stargazers)

A fun, open-source quadruped microbot built to be **real, buildable, and learnable**.

Corndog is a Raspberry Pi 5-powered walking robot built around the idea that robotics should be approachable, remixable, and full of personality. It started as a no-fly alternative to drones and has grown into a platform for experimentation with locomotion, calibration, control systems, and machine learning.

![Corndog hero image](/Assets/IMG_4835.jpg)

> Walking GIF coming soon.

---

## What is Corndog?

Corndog is a quadruped microbot project built for two kinds of people:

- **Hobby builders** who want a real robot they can print, assemble, study, and improve
- **Showcase viewers** who want to see a robotics project with personality, polish, and room to grow

The long-term goal is for Corndog to become easy to reproduce and learn from, while keeping the deeper technical details open on GitHub.

---

## Highlights

- Real quadruped robot platform
- Raspberry Pi 5 based
- Steam Deck control support
- Flipper Zero control support
- Inverse kinematics experiments in progress
- Machine learning environment in progress
- Lower-cost philosophy compared with commercial quadrupeds
- Designed to be remade, studied, and improved
- Featured on *American Ninja Warrior* Season 17, Episode 2

---

## Current Status

Corndog is already usable, but still **very beginner-unfriendly**.

### Completed / effectively working
- Standing
- Basic walking
- Calibration workflow
- Steam Deck control
- Flipper Zero control

### In progress
- Walking with improved IK
- Turning with IK
- Machine learning environment
- Battery solution

### Not started yet
- Autonomous behavior

---

## Build Status

- **3D-printable today:** yes
- **Printed parts used in the build are included:** yes
- **Custom/non-obvious undocumented parts:** no
- **Wiring instructions:** coming soon
- **Assembly guide:** coming soon
- **Battery guide / updated power documentation:** coming soon

At the moment, the repo is best for people who are comfortable experimenting, troubleshooting, and learning as they go.

---

## Estimated Cost

Estimated build cost is **around $500 USD**, depending on parts sourcing.

This reflects the current prototype-oriented setup and may change as the battery system and sourcing become more standardized.

---

## Why Corndog?

Corndog is built around a few core ideas:

### Lower-cost philosophy
The aim is to create a quadruped platform that is more accessible than commercial robot dogs while still being capable and fun.

### Personality matters
This is a technical project, but it is also meant to feel expressive, cute, and memorable.

### Machine learning experimentation
Corndog is not just a remote-control robot. It is also a testbed for gait generation, control experimentation, and future ML-based behavior.

### Easy to remake and learn from
The project is meant to reward curiosity. The long-term goal is not just “look at my robot,” but “here’s a robot you can understand and build too.”

---

## Hardware

| Component | Example Link | Notes |
|---|---|---|
| Raspberry Pi 5 | [Amazon](https://a.co/d/dEzpuJt) | Raspberry Pi OS / Raspbian 64-bit |
| Adafruit PCA9685 16-Channel PWM Driver | [Amazon](https://a.co/d/57Q8URR) / [Adafruit](https://www.adafruit.com/product/815) | I²C address `0x40` |
| Servo Motors (x12) | [Amazon](https://a.co/d/4C4s5K8) | 0–270° actuation range, pulse width 500–2500 µs |
| LiPo Battery + Power Module | [Amazon](https://a.co/d/3LN6lL5) | Current solution; updated battery system in progress |
| Wiring / headers / standoffs / misc. hardware | — | Jumper wires, heat-shrink, nylon standoffs, etc. |

> A full BOM and sourcing guide are coming soon.

---

## 3D-Printed Parts

Most of the structural design is based on KDY0523’s [Thingiverse design](https://www.thingiverse.com/thing:3445283).

This build also uses:

- Reinforced leg parts from [Mike4192’s SpotMicro fork](https://github.com/mike4192/spotMicro)
- Experimental printing support from **BTULab at CU Boulder**
- PLA for the current primary printed configuration

### Material notes

I tested **ASA** for stronger joints, but in practice it still failed under extreme stress and was more difficult to work with. For now, I have gone back to **PLA** because it is easier to print, iterate on, and replace.

---

## Software Requirements

Corndog currently targets **Python 3.8+** on Raspberry Pi OS / Raspbian.

### System packages

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-tk libopencv-dev network-manager
```

### Python packages

```bash
pip3 install adafruit-circuitpython-pca9685 \
  adafruit-circuitpython-servokit \
  gpiozero \
  numpy \
  opencv-python \
  picamera2
```

### Additional dependencies

- **spot-micro-kinematics**  
  Used for inverse kinematics and stick-figure utilities  
  https://github.com/mike4192/spot_micro_kinematics_python/tree/master

- **tkinter**  
  Used for the local GUI

- **nmcli**  
  Used for Wi-Fi status display

---

## Installation

Clone the repository:

```bash
git clone https://github.com/Obsideaock/Corndog.git
cd Corndog
```

Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

> Setup documentation is still evolving. This repo is usable today, but not yet streamlined for first-time builders.

---

## Calibration

Servo calibration is one of the most important parts of getting Corndog working properly.

### 1. Find servo endpoints

```bash
python3 SingularMotortest.py
```

### 2. Set the standing pose

- Place the robot upside down
- Manually adjust each joint
- Find a pose where the joints are aligned consistently
- Save updated values into `servo_home`

### 3. Check mappings

Review `MAPPING` and `CHANNEL_MAP` in `main.py`.

### 4. Tune offsets

Adjust offsets until the physical robot matches the intended home pose.

Example:

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

Full calibration notes are being expanded in [`docs/calibration.md`](docs/calibration.md).

---

## Usage

### Steam Deck / remote control mode

```bash
python3 SteamDeckCommunication.py
```

### Local movement / control tools

```bash
python3 main.py
```

### Other included scripts

- `MoveLib.py` — shared movement/control logic
- `SingularMotortest.py` — manual servo calibration
- `FlipperTed.py` / related Flipper scripts — Flipper Zero control tooling
- `StartupLCD.py` — LCD startup/status behavior
- `gait_engine_app.py` — gait experimentation tooling

More control/setup notes are coming soon in [`docs/controls.md`](docs/controls.md).

---

## Machine Learning

The repo includes an early machine learning environment for gait experimentation.

This part of the project is still in progress, but Corndog is intended to become a platform for testing not only direct control, but also learned movement and future autonomous behavior.

Documentation for training, reproducibility, and results is coming soon.

---

## American Ninja Warrior

Corndog appeared as “Spot Micro” on **American Ninja Warrior**, Season 17, Episode 2.

Watch here:  
https://www.youtube.com/watch?v=WpAhqLwTt5Q

---

## Roadmap

- Improved IK walking
- IK turning
- Updated battery solution
- Wiring guide
- Full assembly guide
- BOM / sourcing documentation
- Better beginner documentation
- Expanded machine learning workflow
- Autonomous behavior experiments
- Web/kit-style documentation structure over time

A more detailed roadmap is coming soon in [`docs/roadmap.md`](docs/roadmap.md).

---

## Documentation

Additional project documentation is being added over time:

- [Assembly Guide](docs/assembly.md)
- [Wiring Guide](docs/wiring.md)
- [Calibration Guide](docs/calibration.md)
- [Controls Guide](docs/controls.md)
- [Bill of Materials](docs/bom.md)
- [Roadmap](docs/roadmap.md)

---

## Contributing

Contributions, ideas, and experiments are welcome.

Especially helpful areas include:

- gait generation
- IK improvements
- documentation
- builder experience improvements
- simulation / ML experimentation

If you want to help, open an issue or pull request.

---

## License

This project is licensed under **CC BY 4.0**.

You are free to use, modify, and redistribute this work with attribution to **Obsideaock / Corndog**.

---

## Acknowledgements

- **KDY0523** for the original 3D design
- **Mike4192** for reinforced SpotMicro leg modifications
- **BTULab @ CU Boulder** for printing support
- The open-source robotics community

---

Made with ❤️ by **Obsideaock**