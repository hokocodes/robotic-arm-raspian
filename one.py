import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C bus and create PCA9685 object
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Set the frequency to 50Hz for standard servos

# Function to set servo angle
def set_servo_angle(channel, angle):
    pulse_length = 1000000 / 50  # 50Hz
    pulse = angle * 1000 / 180 + 500  # Calculate pulse width
    pca.channels[channel].duty_cycle = int(pulse * 65535 / 1000000)

# Move servo to different angles
try:
    set_servo_angle(0, 0)    # Servo on channel 0 to 0 degrees
    time.sleep(2)
    set_servo_angle(0, 90)   # Servo on channel 0 to 90 degrees
    time.sleep(2)
    set_servo_angle(0, 180)  # Servo on channel 0 to 180 degrees
    time.sleep(2)
finally:
    pca.deinit()  # Clean up and release resources