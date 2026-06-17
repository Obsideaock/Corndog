#!/usr/bin/env python3
"""
RUN THIS ON THE ROBOT (the Pi).

This file lives in the `Slam/` folder inside your main Corndog directory (the
one with MoveLib.py). Run it from inside the Slam folder:

    cd Slam
    python3 run_robot.py                # SLAM + Steam Deck control
    python3 run_robot.py --no-control   # SLAM only (robot stays still)
    python3 run_robot.py --load corndog_map.npz   # resume a saved map

Then open  http://<pi-ip>:8001/  in a browser to watch the map.
"""
import sys
import os

# Make the slam modules (this folder) and the parent Corndog folder
# (MoveLib.py, SteamDeckCommunication.py) both importable.
_HERE = os.path.dirname(os.path.abspath(__file__))      # .../Corndog/Slam
_PARENT = os.path.dirname(_HERE)                          # .../Corndog
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from slam_app import main

if __name__ == "__main__":
    main()
