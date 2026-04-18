#!/usr/bin/env python3
"""
Ultrasonic Sensor Test Script for Raspberry Pi 5
Supports HC-SR04 and similar ultrasonic distance sensors

Hardware Setup:
- VCC -> 5V
- GND -> GND
- TRIG -> GPIO Pin (default: GPIO 23)
- ECHO -> GPIO Pin (default: GPIO 24)

Note: ECHO pin outputs 5V, but RPi GPIO pins are 3.3V tolerant.
Use a voltage divider (1kΩ and 2kΩ resistors) for safety.
"""

import time
import sys

try:
    from gpiozero import DistanceSensor
    USE_GPIOZERO = True
except ImportError:
    print("gpiozero not found. Install with: sudo apt-get install python3-gpiozero")
    print("Falling back to RPi.GPIO method...")
    USE_GPIOZERO = False
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("ERROR: Neither gpiozero nor RPi.GPIO is available!")
        print("Install one with:")
        print("  sudo apt-get install python3-gpiozero")
        print("  or")
        print("  sudo apt-get install python3-rpi.gpio")
        sys.exit(1)

# Pin Configuration (BCM numbering)
#TRIG_PIN = 5
#ECHO_PIN = 6

TRIG_PIN = 27
ECHO_PIN = 17

# Measurement settings
MAX_DISTANCE = 4  # Maximum distance in meters (400cm)
SAMPLE_DELAY = 0.1  # Delay between measurements in seconds


def test_with_gpiozero():
    """Test using gpiozero library (recommended for Pi 5)"""
    print("\n=== Testing with gpiozero library ===")
    print(f"TRIG: GPIO {TRIG_PIN}, ECHO: GPIO {ECHO_PIN}")
    print(f"Max distance: {MAX_DISTANCE}m")
    print("Press Ctrl+C to stop\n")
    
    sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN, max_distance=MAX_DISTANCE)
    
    try:
        while True:
            distance = sensor.distance * 100  # Convert to cm
            print(f"Distance: {distance:.2f} cm")
            time.sleep(SAMPLE_DELAY)
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        sensor.close()


def test_with_rpi_gpio():
    """Test using RPi.GPIO library (alternative method)"""
    print("\n=== Testing with RPi.GPIO library ===")
    print(f"TRIG: GPIO {TRIG_PIN}, ECHO: GPIO {ECHO_PIN}")
    print("Press Ctrl+C to stop\n")
    
    # Setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    
    print("Waiting for sensor to settle...")
    time.sleep(2)
    
    try:
        while True:
            # Send trigger pulse
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.00001)  # 10 microsecond pulse
            GPIO.output(TRIG_PIN, False)
            
            # Wait for echo to start
            timeout = time.time() + 1  # 1 second timeout
            while GPIO.input(ECHO_PIN) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    print("Timeout waiting for echo start")
                    break
            
            # Wait for echo to end
            timeout = time.time() + 1
            while GPIO.input(ECHO_PIN) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    print("Timeout waiting for echo end")
                    break
            
            # Calculate distance
            try:
                pulse_duration = pulse_end - pulse_start
                # Speed of sound = 34300 cm/s
                # Distance = (Time × Speed) / 2 (round trip)
                distance = (pulse_duration * 34300) / 2
                
                if distance < 2 or distance > 400:
                    print(f"Out of range: {distance:.2f} cm")
                else:
                    print(f"Distance: {distance:.2f} cm")
            except:
                print("Measurement error")
            
            time.sleep(SAMPLE_DELAY)
            
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        GPIO.cleanup()


def main():
    """Main function to run the appropriate test"""
    print("=" * 50)
    print("Ultrasonic Sensor Test for Raspberry Pi 5")
    print("=" * 50)
    
    if USE_GPIOZERO:
        test_with_gpiozero()
    else:
        test_with_rpi_gpio()


if __name__ == "__main__":
    main()
