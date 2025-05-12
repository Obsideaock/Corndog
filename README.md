# Project Corndog

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)  
[![GitHub Issues](https://img.shields.io/github/issues/Obsideaock/Corndog.svg)](https://github.com/Obsideaock/Corndog/issues)  
[![GitHub Stars](https://img.shields.io/github/stars/Obsideaock/Corndog.svg)](https://github.com/Obsideaock/Corndog/stargazers)

## Table of Contents

1. [Overview](#overview)  
2. [Motivation](#motivation)  
3. [Hardware Requirements](#hardware-requirements)  
4. [3D-Printed Parts](#3d-printed-parts)  
5. [Software Dependencies](#software-dependencies)  
6. [Installation & Setup](#installation--setup)  
7. [Usage](#usage)  
8. [Machine Learning Integration (WIP)](#machine-learning-integration-wip)  
9. [American Ninja Warrior Feature](#american-ninja-warrior-feature)  
10. [Contribution & License](#contribution--license)  
11. [Roadmap](#roadmap)  
12. [Acknowledgements](#acknowledgements)  

---

## Overview
Project Corndog is an open-source quadruped microbot built around a Raspberry Pi 5 and Adafruit PCA9685 servo drivers. Originally conceived as a ground-based “drone” alternative, it’s grown into both a hobbyist platform and research testbed for sensors/coding and machine learning.

![Corndog in Halloween witch hat](/Assets/IMG_4835.jpg)

## Motivation
- **No-fly alternative**: college restrictions on drones → build a walking “drone.”  
- **Affordability & openness**: aim to undercut commercial quadrupeds and share every file.  
- **Research & fun**: test IK, ML-Agents environments, and even starred on **American Ninja Warrior** (season 17).

## Hardware Requirements
| Component                                | Link (example)                                          | Notes                                                         |
|------------------------------------------|---------------------------------------------------------|---------------------------------------------------------------|
| Raspberry Pi 5                           | [Amazon](https://a.co/d/dEzpuJt)                                                | Raspbian (64-bit) flashed                                    |
| Adafruit PCA9685 16-Channel PWM Driver   | [Amazon](https://a.co/d/57Q8URR), [Adafruit](https://www.adafruit.com/product/815)                                    | I²C address 0x40                                              |
| Servo Motors (×12)                       | [Amazon](https://a.co/d/4C4s5K8)                                                | 0–270° actuation range, pulse 500–2500 µs                    |
| Lithium-Polymer Battery + Power Module   | [Amazon](https://a.co/d/3LN6lL5)                                                | Custom solution coming soon                                 |
| Wires, Breadboard, GPIO Headers, etc.    | No specifics needed/used                                 | jumper wires, heat-shrink tubing, nylon standoffs, etc.      |

## 3D-Printed Parts
Most structural parts come from KDY0523’s [Thingiverse design](https://www.thingiverse.com/thing:3445283).  
- **Custom harness** (for travel)—photo's and links coming soon.  
- **Reinforced legs** from [Mike4192’s SpotMicro fork](https://github.com/mike4192/spotMicro).  
- **BTULab at CU Boulder** assisted with experimental printing. (My school!)

Currently used PLA for all parts. Experimented with ASA for the joints for increased strength, but still snapped under extreme circumstances and pla is way easier to use so I swapped back.

## Software Dependencies
All code targets **Python 3.8+** on Raspbian. You’ll need:

```bash
sudo apt update && sudo apt install -y python3-pip python3-venv python3-tk \
  libopencv-dev network-manager
pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit \
  gpiozero numpy opencv-python picamera2
```

- **spot-micro-kinematics**: for IK & stick-figure utilities   (Manually install from [here](https://github.com/mike4192/spot_micro_kinematics_python/tree/master) 
- **tkinter**: GUI controls  
- **MoveLib**: alias for `main.py` routines in `SteamDeckCommunication.py`  (Found in this repo)
- **nmcli**: used by `DisplayWifi.py` to show WiFi status on LCD  

## Installation & Setup

1. **Clone the repo**  
```bash
git clone https://github.com/Obsideaock/Corndog.git
cd Corndog
```

2. **Create & activate a virtual environment**  
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

3. **Servo calibration**  
- Run `SingularMotortest.py` to manually find 0°–270° endpoints.
- Place robot upside down and manually adjust each joint to find standing position where all of the joints are the same distance from the ground. Place updated angles into servo_home.
- Note channel ↔ joint mapping in `main.py`’s `MAPPING` & `CHANNEL_MAP`.  
- Edit offsets in `MAPPING` until the “home” pose matches your physical mech. Put safety=true, take the printed values for each motor and add the inverse (example below)
> Given from safety: Leg 3 planned deltas: ```{7: 10.3512659818415784, 5: -6.30598966999280464, 1: -30.17742776376383063}```\
> Beforehand: ```3: {1:{'sign':+1,'offset':45}, 2:{'sign':-1,'offset':  83}, 3:{'sign':-1,'offset': 131}}```\
> Post editing offsets: ```3: {1:{'sign':+1,'offset':35}, 2:{'sign':-1,'offset':  86}, 3:{'sign':-1,'offset': 161}}```

4. **Auto-start WiFi display** (optional)  
- Add `DisplayWifi.py` to `/etc/rc.local` or create a systemd service to run on boot.


## Usage after setup

1. **Enable servos** and initialize:  
```bash
python3 SteamDeckCommunication.py
```
> Steamdeck script coming soon

- Streams MJPEG video at `http://<pi-ip>:8000/stream.mjpg`  
- Listens for joystick/control commands on TCP port 65432

2. **Direct GUI control** (no Steam Deck):  
```bash
python3 main.py
```

- Opens a Tkinter window with buttons for walking, turning, kneeling, handstand, dance, jump, etc.

3. **Available scripts**  
- `main.py`: high-level movements & IK demos  
- `SingularMotortest.py`: manual servo calibration  
- `DisplayWifi.py`: WiFi status on LCD  
- `SteamDeckCommunication.py`: joystick control + video streaming  

## Machine Learning Integration (WIP)

The `Spot-Micro-Machine-Learning` folder contains an ML-Agents environment for gait learning.  
> **Status:** environment skeleton in place—training scripts & pretrained weights forthcoming.

## American Ninja Warrior Feature

Corndog (as “Spot Micro”) appears in **American Ninja Warrior S17**—episode & air date TBD. Once the episodes/season have come out, limited links will be availabe.
> **Media:** links to video clips and press coverage will be added once available.

## Contribution & License

- **License:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)  
  Use, modify, and redistribute—please credit “Project Corndog by Obsideaock.”  
- **Issues & PRs:** please use [GitHub Issues](https://github.com/Obsideaock/Corndog/issues) and submit pull requests following existing style.

## Roadmap

- 🔧 Improved IK calibration tool (in progress)  
- 🔋 Custom power-board integration  (in progress)
- 🦾 Advanced gaits via ML-Agents (shared models) (Feel free to offer help)
- 🖥️ Web dashboard for remote control & monitoring  
- 📷 ANW media gallery (post-season)

## Acknowledgements

- **KDY0523** for the original 3D design ([Thingiverse](https://www.thingiverse.com/thing:3445283))  
- **Mike4192** for reinforced leg mods ([GitHub](https://github.com/mike4192/spotMicro))  
- **BTULab @ CU Boulder** for printing assistance ([BTULab](https://www.colorado.edu/atlas/btu-lab))
- **All contributors** and the open-source robotics community  

---

*Made with ❤️ by Obsideaock*

