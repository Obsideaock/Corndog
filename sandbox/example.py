#!/usr/bin/env python3
"""
example.py — your starting point for making Corndog do things.

COPY THIS FILE FIRST (this original belongs to the official software and
gets refreshed by updates):

    cp ~/Corndog/sandbox/example.py ~/Corndog/sandbox/mytest.py

Then run your copy the proper way (supervisor paused automatically):

    corndog run sandbox/mytest.py

Or in Geany: open your copy and press F5 (if you set the build command
from the user guide).

Below is one working demo, followed by EVERY movement command commented
out. Uncomment whatever you want to try. He should be standing on the
floor with some space around him.
"""

# MoveLib is Corndog's movement library — this one import gives you
# everything: it connects the servos, IMU, and LCD automatically.
import MoveLib as dog

from time import sleep


# ============================================================
# A tiny working demo: stand up, wave hello, sit back down.
# ============================================================

dog.stand_up()          # always start here — safely rises to the home pose
sleep(1.0)

dog.wave()              # lifts a paw and waves it
sleep(1.0)

dog.sit()               # sits down
sleep(2.0)

dog.unsit()             # ...and stands back up
sleep(1.0)


# ============================================================
# Every command, one by one (uncomment to try):
# ============================================================

# ---- poses & tricks ----
# dog.sit()             # sit down (call unsit() to stand back up)
# dog.unsit()           # stand up from sitting
# dog.kneel()           # kneel down on the front legs
# dog.unkneel()         # rise from kneeling
# dog.dance()           # a little dance
# dog.shake()           # offers a paw to shake
# dog.wave()            # waves a paw

# ---- moving the body directly (inverse kinematics) ----
# Move each leg's foot by (x, y, z) in METERS relative to where it is.
# Legs: 0 = left front, 1 = right front, 2 = left back, 3 = right back.
# +x = forward, +y = left, +z = up. Small numbers! 0.02 is 2 cm.
#
# dog.iklegs_move({0: (0.0, 0.0, 0.02),   # lift left-front foot 2 cm
#                  1: (0.0, 0.0, 0.0),
#                  2: (0.0, 0.0, 0.0),
#                  3: (0.0, 0.0, 0.0)})
#
# Moving ALL feet down = body up (and vice versa):
# dog.iklegs_move({i: (0.0, 0.0, -0.02) for i in range(4)})   # body rises
# dog.iklegs_move({i: (0.0, 0.0, +0.02) for i in range(4)})   # body lowers

# ---- walking (the gait engine) ----
# Give him a velocity: vx (m/s forward), vy (m/s left), wz (rad/s turn).
# He walks until you tell him to stop.
#
# dog.gait_command(0.12, 0.0, 0.0)    # walk forward
# sleep(3.0)
# dog.gait_command(0.0, 0.0, 0.8)     # turn in place (counter-clockwise)
# sleep(2.0)
# dog.stop_gait()                     # stop walking (returns to neutral)

# ---- tuning the walk (same knobs as the gait app / Steam Deck) ----
# print(dog.get_gait_options())               # see every option + value
# dog.set_gait_option("step_hz", 1.3)         # faster steps
# dog.set_gait_option("step_height", 0.05)    # higher steps
# dog.set_gait_option("gait", "creep")        # "diagonal", "creep", "wave"

# ---- the IMU (balance sensor) ----
# dog.zero_imu()                  # call while he's level: sets "this is flat"
# print(dog.get_gravity())        # gravity vector — which way is down

# ---- servo power ----
# dog.disable_servos()            # go limp (torque off — catch him!)
# dog.enable_servos()             # torque back on at last commanded angles

# ============================================================
# Always end your scripts like this:
# ============================================================
dog.sit()
sleep(1.0)
dog.shutdown()          # stops the gait engine + releases everything cleanly
print("Done! Now edit this file (your copy of it) and make him do stuff.")
