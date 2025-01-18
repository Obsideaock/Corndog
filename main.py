import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice
import sys
import tkinter as tk
from math import sqrt, sin, cos, tan, asin, acos, atan, degrees

sys.path.insert(0, '/home/Corndog')
from lcd import lcd_library as lcd

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

OE_PIN = 22
output_enable = OutputDevice(OE_PIN, active_high=False)

# Define the servo channels and positions
servo_channels = [0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15]
servo_home = {0: 40, 1: 230, 4: 230, 5: 40, 6: 130, 7: 130, 8: 130, 9:135, 10: 65, 11: 210, 14: 240, 15: 30}

# Initialize servos on the specified channels
servos = {channel: servo.Servo(pca.channels[channel]) for channel in servo_channels}
for channel in servos:
    servos[channel].set_pulse_width_range(500, 2500)
    servos[channel].actuation_range = 270



def initialize_servo_angles():
    """
    Initializes the servo_angles dictionary based on the initial standing positions.
    This function should be called once at the start of the program.
    """
    servo_standing = servo_home
    global servo_angles
    servo_angles = {channel: angle for channel, angle in servo_standing.items()}


def move_motors(movements, delay=0.01, speed_multiplier=10):
    def clamp_angle(angle):
        return max(0, min(270, angle))  # 0-270

    global servo_angles

    # Extract channels and relative angles
    channels = list(movements.keys())
    relative_angles = list(movements.values())

    # Current angles
    current_angles = [
        servo_angles[channel] if channel in servo_angles 
        else servo_standing[channel] 
        for channel in channels
    ]

    # Target angles (with clamp)
    target_angles = [
        clamp_angle(cur + rel) 
        for cur, rel in zip(current_angles, relative_angles)
    ]

    # Figure out the maximum steps (float, not int yet)
    max_step_count = max(
        abs(t - c) * 10  # 10 steps per degree
        for t, c in zip(target_angles, current_angles)
    )
    # Convert to int after dividing by speed
    adjusted_step_count = max(1, int(round(max_step_count / speed_multiplier)))

    # Increments
    increments = []
    for t, c in zip(target_angles, current_angles):
        if adjusted_step_count != 0:
            increments.append((t - c) / adjusted_step_count)
        else:
            increments.append(0)

    # Initialize a dict of current positions
    current_positions = dict(zip(channels, current_angles))

    # Move motors in small increments
    for step in range(adjusted_step_count):
        for i, channel in enumerate(channels):
            if increments[i] != 0:
                new_angle = current_positions[channel] + increments[i]
                new_angle = clamp_angle(new_angle)
                current_positions[channel] = new_angle
                servo_angles[channel] = new_angle
                servos[channel].angle = new_angle
        time.sleep(delay)

    # Optional tiny correction step to ensure final angle
    # (only if you want to be EXACT at the end, but carefully)
    for i, channel in enumerate(channels):
        # Snap exactly to target angle
        servo_angles[channel] = target_angles[i]
        servos[channel].angle = target_angles[i]

def stand_up():
    def set_motor_angles(channels, positions):
        for channel in channels:
            servos[channel].angle = positions[channel]
        time.sleep(0.5)  # Wait for all servos in the group to move

    # Define the target positions for all servos
    target_positions = {channel: servo_home[channel] for channel in servos}

    # Initialize sets of motors sequentially
    set_motor_angles([6, 7, 8, 9], target_positions)
    time.sleep(0.2)
    set_motor_angles([5, 4, 0, 1], target_positions)
    set_motor_angles([14, 15, 10, 11], target_positions)
    initialize_servo_angles()


def enable_servos():
    output_enable.off()
    print("Servos enabled")


def disable_servos():
    lcd.clear()
    output_enable.on()
    for channel in servo_channels:
        pca.channels[channel].duty_cycle = 0  # Set the PWM signal to 0
    


# Global flag to track walking state
is_walking = False
is_sitting = False
is_kneeling = False
is_handstand = False

# Tkinter GUI
def create_gui():
    walkspeed = 15
    global walk_button, is_walking, sit_button, is_sitting, kneel_button, is_kneeling, is_handstand  # Declare walk_button and is_walking as global so it can be accessed in the walk function
    window = tk.Tk()
    window.title("Robot Control")

    def walk():
        global is_walking

        if not is_walking:
            # Start walking
            is_walking = True
            lcd.lcd("Walking")
            walk_button.config(text="Stop Walking")  # Update button text to "Stop Walking"
            window.update()

            move_motors({10: -20, 4: 20, 1: 35, 15: -35}, speed_multiplier=walkspeed)
            move_motors({11: -20, 5: 20, 1: -35, 15: 35}, speed_multiplier=walkspeed)

            walking_loop()  # Start the walking loop

        else:
            # Stop walking
            is_walking = False

            lcd.lcd("Reseting to     Normal")
            move_motors({11: 20, 5: -20, 0: -35, 14: 35}, speed_multiplier=walkspeed)
            move_motors({10: 20, 4: -20, 0: 35, 14: -35}, speed_multiplier=walkspeed)

            lcd.clear()
            walk_button.config(text="Start Walking")  # Update button text back to "Start Walking"
            window.update()

    def walking_loop():
        # If walking is still enabled, move the motors and then call this function again
        if is_walking:
            lcd.lcd("Walking 1/4")
            move_motors({11: 40, 5: -40, 0: -30, 14: 30}, speed_multiplier=walkspeed)
            lcd.lcd("Walking 2/4")
            move_motors({10: 40, 4: -40, 0: 30, 14: -30}, speed_multiplier=walkspeed)
            lcd.lcd("Walking 3/4")
            move_motors({10: -40, 4: 40, 1: 30, 15: -30}, speed_multiplier=walkspeed)
            lcd.lcd("Walking 4/4")
            move_motors({11: -40, 5: 40, 1: -30, 15: 30}, speed_multiplier=walkspeed)

            # Schedule the walking_loop to be called again after 100ms
            window.after(0, walking_loop)

    def kneel():
        global is_kneeling

        if not is_kneeling:
            # Start sitting
            is_kneeling = True
            lcd.lcd("Kneeling")
            kneel_button.config(text="Unkneel")  # Update button text to "Stand Up"
            window.update()
            # Perform sitting motion
            move_motors({15: -30, 11: 30, 10: -30, 14: 30})

        else:
            # Stand back up
            is_kneeling = False
            lcd.lcd("Standing Up")
            kneel_button.config(text="Kneel")  # Update button text back to "Sit"
            window.update()
            # Perform standing motion (reverse of sitting)
            move_motors({15: 30, 11: -30, 10: 30, 14: -30})
            stand_up()
            lcd.clear()

    def handstand():
        global is_handstand

        if not is_handstand:
            # Start sitting
            is_handstand = True
            lcd.lcd("Handstanding")
            handstand_button.config(text="Back Down")  # Update button text to "Stand Up"
            window.update()
            # Perform sitting motion
            move_motors({0: -40, 4: 10, 5: -10, 1: 40})
            move_motors({15: 85, 11: 0, 10: -0, 14: -85})
            move_motors({0: 50, 1: -50})

        else:
            # Stand back up
            is_handstand = False
            lcd.lcd("Standing Down")
            handstand_button.config(text="Handstand")  # Update button text back to "Sit"
            window.update()
            # Perform standing motion (reverse of sitting)
            move_motors({0: -10, 1: 10})
            move_motors({15: -85, 11: -0, 10: 0, 14: 85, 0: -40, 1: 40})
            move_motors({0: 40, 4: -10, 5: 10, 1: -40})
            stand_up()
            lcd.clear()
    def sit():
        global is_sitting

        if not is_sitting:
            # Start sitting
            is_sitting = True
            lcd.lcd("Sitting")
            sit_button.config(text="Stand Up")  # Update button text to "Stand Up"
            window.update()
            # Perform sitting motion
            move_motors({0: -40, 4: 15, 5: -15, 1: 40})
            move_motors({15:90, 14:-90, 11:-60, 10:60, 0:10, 1:-10})

        else:
            # Stand back up
            is_sitting = False
            lcd.lcd("Standing Up")
            sit_button.config(text="Sit")  # Update button text back to "Sit"
            window.update()
            # Perform standing motion (reverse of sitting)
            move_motors({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10})
            move_motors({0: 40, 4: -15, 5: 15, 1: -40})
            stand_up()
            lcd.clear()

    def shake():
        lcd.lcd("Shaking")
        move_motors({0: -40, 4: 15, 5: -15, 1: 40})
        move_motors({15:90, 14:-90, 11:-60, 10:60, 0:10, 1:-10})
        time.sleep(0.5)
        move_motors({10: 130, 14: -80}, speed_multiplier=25)
        time.sleep(.75)
        for _ in range(4):
            move_motors({14: 40}, speed_multiplier=15)
            time.sleep(0.15)
            move_motors({14: -40}, speed_multiplier=15)
            time.sleep(0.15)
        time.sleep(.25)
        move_motors({10: -130, 14: 120}, speed_multiplier=25)
        move_motors({14: -40}, speed_multiplier=25)
        move_motors({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10})
        move_motors({0: 40, 4: -15, 5: 15, 1: -40})
        stand_up()
        lcd.clear()

    def dance():
        lcd.lcd("Dancing")
        for _ in range(4):
            move_motors({15: -30, 0: -30, 14: 30, 1: 30})
            move_motors({15: 30, 0: 30, 14: -30, 1: -30})
        lcd.clear()

    def jump():
        lcd.lcd("Charging jump")
        move_motors({0: -40, 4: 30, 5: -30, 1: 40, 15: -30, 11: 30, 14: 30, 10: -30})
        lcd.lcd("Jumping")
        move_motors({0: 50, 4: -30, 5: 30, 1: -50, 15: 50, 11: -20, 14: -50, 10: 20}, delay=0.00001, speed_multiplier=1000)
        time.sleep(0.2)
        move_motors({0: -50, 4: 30, 5: -30, 1: 50, 15: -50, 11: 20, 14: 50, 10: -20}, delay=0.00001, speed_multiplier=1000)
        time.sleep(0.2)
        move_motors({0: 40, 4: -30, 5: 30, 1: -40, 15: 30, 11: -30, 14: -30, 10: 30})
        stand_up()
        lcd.clear()

    def IKTEST():
        flleg(1, 1, 15)

    def power_off():
        lcd.lcd("Down")
        move_motors({15: -40, 0: -40, 14: 40, 1: 40})
        disable_servos()
        lcd.clear()

    tk.Button(window, text="Stand Up", command=stand_up).pack()

    # Create the walk button with an initial label
    walk_button = tk.Button(window, text="Start Walking", command=walk)
    walk_button.pack()

    # Create the sit button with an initial label
    sit_button = tk.Button(window, text="Sit", command=sit)
    sit_button.pack()

    kneel_button = tk.Button(window, text="Kneel", command=kneel)
    kneel_button.pack()

    handstand_button = tk.Button(window, text="Handstand", command=handstand)
    handstand_button.pack()

    tk.Button(window, text="Shake", command=shake).pack()
    tk.Button(window, text="Dance", command=dance).pack()
    tk.Button(window, text="Jump", command=jump).pack()
    tk.Button(window, text="IKTest", command=IKTEST).pack()
    tk.Button(window, text="Lie Down", command=power_off).pack()

    window.mainloop()

# Run the GUI
create_gui()
