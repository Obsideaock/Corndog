import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize servo motor on channel 7
servo_channel = 14
enable_device = OutputDevice(17)
my_servo = servo.Servo(pca.channels[servo_channel])
my_servo.set_pulse_width_range(500, 2500)
my_servo.actuation_range = 270

try:
    while True:
        # Prompt user for the angle
        angle = input("Enter the angle for the servo (0-270) or 'q' to quit: ")
        
        if angle.lower() == 'q':
            enable_device.off()
            pca.channels[servo_channel].duty_cycle = 0
            break
        
        try:
            angle = float(angle)
            if 0 <= angle <= 270:
                my_servo.angle = angle
                print(f"Channel {servo_channel} moved to {angle} degrees")
            else:
                print("Please enter a valid angle between 0 and 270.")
        except ValueError:
            print("Invalid input. Please enter a numeric value between 0 and 270.")

        time.sleep(0.5)  # Adding a small delay to ensure the servo has time to move

finally:
    # Cleanup
    pca.deinit()
