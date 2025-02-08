# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=8)


while True:
    x = input("Enter servo number to set ({}), or anything else to exit: ")
    if x == str(i):
        kit.continuous_servo[i].throttle = 1
        print("Servo {}, Throttle = 1".format(i))
        time.sleep(1)
        kit.continuous_servo[i].throttle = -1
        print("Servo {}, Throttle = -1".format(i))
        time.sleep(1)
