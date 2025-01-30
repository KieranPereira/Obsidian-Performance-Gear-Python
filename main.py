import time
import math
import json
from machine import Pin, I2C  # For ESP32 MicroPython
from mpu6050 import MPU6050  # External library for MPU6050

# Define I2C Pins for ESP32
I2C_SDA = 21
I2C_SCL = 22

# Define the LED Pin (optional)
LED_PIN = 2


# Initialize I2C and MPU6050
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
mpu = MPU6050(i2c)

# Initialize LED Pin
led = Pin(LED_PIN, Pin.OUT)

# Timing Variables
last_update = time.ticks_ms()  # Timer for MPU6050 updates
last_blink = time.ticks_ms()   # Timer for LED blinking
led_state = False              # Track LED state (ON/OFF)

def calculate_angle(a_x, a_y, a_z):
    """Calculate tilt angle relative to the Z-axis."""
    return math.atan2(a_z, math.sqrt(a_x**2 + a_y**2)) * (180.0 / math.pi)

# Main Loop
while True:
    current_millis = time.ticks_ms()

    # MPU6050 Data Update (Every 500ms)
    if time.ticks_diff(current_millis, last_update) >= 500:
        last_update = current_millis

        # Read accelerometer, gyroscope, and temperature data
        accel = mpu.get_accel()
        gyro = mpu.get_gyro()
        temp = mpu.get_temp()

        # Calculate tilt angle
        angle = calculate_angle(accel['x'], accel['y'], accel['z'])

        # Create JSON data
        json_data = {
            "accel_x": accel['x'],
            "accel_y": accel['y'],
            "accel_z": accel['z'],
            "gyro_x": gyro['x'],
            "gyro_y": gyro['y'],
            "gyro_z": gyro['z'],
            "temp": temp,
            "angle": angle,
            "status": "Done" if 85 <= angle <= 95 else "In Progress"
        }

        # Print JSON data
        print(json.dumps(json_data))

    # LED Blinking Logic (Every 1000ms)
    if time.ticks_diff(current_millis, last_blink) >= 1000:
        last_blink = current_millis
        led_state = not led_state
        led.value(led_state)
