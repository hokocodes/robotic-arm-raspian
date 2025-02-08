# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=8)

r = input()


kit.continuous_servo[0].throttle = 1
print("Servo 0, Throttle = 1")
time.sleep(1)
kit.continuous_servo[0].throttle = -1
print("Servo 0, Throttle = -1")
time.sleep(1)
kit.continuous_servo[1].throttle = 1
print("Servo 1, Throttle = 1")
time.sleep(1)
kit.continuous_servo[1].throttle = -1
print("Servo 1,Throttle = -1")
time.sleep(1)
kit.continuous_servo[2].throttle = 1
print("Servo 2, Throttle = 1")
time.sleep(1)
kit.continuous_servo[2].throttle = -1
print("Servo 2, Throttle = -1")
time.sleep(1)
kit.continuous_servo[3].throttle = 1
print("Servo 3, Throttle = 1")
time.sleep(1)
kit.continuous_servo[3].throttle = -1
print("Servo 3, Throttle = -1")
time.sleep(1)
kit.continuous_servo[4].throttle = 1
print("Servo 4, Throttle = 1")
time.sleep(1)
kit.continuous_servo[4].throttle = -1
print("Servo 4, Throttle = -1")
time.sleep(1)
kit.continuous_servo[5].throttle = 1
print("Servo 5, Throttle = 1")
time.sleep(1)
kit.continuous_servo[5].throttle = -1
print("Servo 5, Throttle = -1")
time.sleep(1)
