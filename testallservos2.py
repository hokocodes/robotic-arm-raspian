# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

while True:
    kit.servo[0].angle = 180
    print("Servo 0, Angle = 180")
    time.sleep(60)
    kit.servo[0].angle = 0
    print("Servo 0, Angle = 0")
    time.sleep(60)
