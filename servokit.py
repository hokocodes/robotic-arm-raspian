# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=8)

r = input()

if r=="1":
    while True:
        kit.servo[0].angle = 180
        time.sleep(1)
        kit.servo[0].angle = 0
        time.sleep(1)
        kit.servo[0].angle = 360
        time.sleep(1)
elif r=="2":
    while True:
        kit.continuous_servo[0].throttle = 1
        time.sleep(1)
        kit.continuous_servo[0].throttle = -1
        time.sleep(1)
        kit.servo[0].angle = 0
        time.sleep(1)
        kit.continuous_servo[0].throttle = 0



