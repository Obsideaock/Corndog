import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice

# --- Setup ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

OE_PIN = 27  # BCM 27 = physical pin 13
output_enable = OutputDevice(OE_PIN, active_high=True)

TEST_CHANNEL = 0  # change this to any channel you have a servo on

s = servo.Servo(pca.channels[TEST_CHANNEL])
s.set_pulse_width_range(500, 2500)
s.actuation_range = 270


def enable_servos():
    output_enable.off()  # LOW
    print("Enabled (OE LOW)")


def disable_servos():
    pca.channels[TEST_CHANNEL].duty_cycle = 0
    output_enable.on()   # HIGH
    print("Disabled (OE HIGH, duty_cycle=0)")


try:
    print("Starting test...")

    disable_servos()
    time.sleep(1)

    enable_servos()
    time.sleep(0.5)

    for angle in [30, 90, 150, 210, 150, 90, 30]:
        print(f"Setting angle {angle}")
        s.angle = angle
        time.sleep(0.8)

    print("Disabling now...")
    disable_servos()

    print("Try to manually move the servo horn now. It should stop receiving pulses.")
    time.sleep(5)

finally:
    try:
        disable_servos()
    except Exception:
        pass
    try:
        pca.deinit()
    except Exception:
        pass
    print("Done.")
